// 控制摆杆起摆，并在进入稳摆状态后输出状态信号。
module swing_up #(
    parameter integer TICK_HZ = 500,
    parameter [15:0] BASE_ANGLE = 16'd8316,
    parameter [15:0] ANGLE_MIN = BASE_ANGLE - 16'd2962,
    parameter [15:0] ANGLE_MAX = BASE_ANGLE + 16'd2138,
    parameter signed [14:0] START_PWM = 15'sd3500,
    parameter signed [14:0] PEAK_PWM_MIN = START_PWM,
    parameter integer START_TIME_MS = 100,
    parameter integer TURN_SAMPLE_TIMEOUT_MS = 40,
    parameter [15:0] ADC_WRAP = 16'd27100,
    parameter [15:0] ADC_STEP_MAX = 16'd2000,
    parameter integer ADC_TIMEOUT_MS = 500,
    parameter integer SWING_TIMEOUT_MS = 10000,
    parameter integer CONFIRM_TIME_MS = 100,
    parameter integer CONFIRM_ERROR_MAX = 500,
    parameter integer CONFIRM_STEP_MAX = 20000 / TICK_HZ,
    parameter integer CAPTURE_ANGLE_ERROR_MAX = 1000,
    parameter integer TURN_CONFIRM_SAMPLES = 3
) (
    input wire sys_clk,
    input wire rst_n,
    input wire tick,
    input wire start_enable,
    input wire raw_valid,
    input wire [15:0] raw_angle,
    output reg [15:0] angle_value,
    output reg angle_valid,
    output wire balance_enable,
    output reg balance_confirmed,
    output reg signed [14:0] swing_pwm,
    output reg capture_pulse,
    output reg [2:0] state
);
    localparam [2:0] WAIT_ADC = 0, WAIT_PEAK = 1,
                     KICK_POS = 2, RETURN_NEG = 3,
                     KICK_NEG = 4, RETURN_POS = 5,
                     BALANCE = 6, STOP = 7;
    localparam integer PULSE_TICKS = (TICK_HZ * START_TIME_MS / 1000 < 1) ?
                                     1 : TICK_HZ * START_TIME_MS / 1000;
    localparam integer TURN_STALE_TICKS = (TICK_HZ * TURN_SAMPLE_TIMEOUT_MS / 1000 < 1) ?
                                          1 : TICK_HZ * TURN_SAMPLE_TIMEOUT_MS / 1000;
    localparam integer TURN_SAMPLES = TURN_CONFIRM_SAMPLES < 1 ? 1 : TURN_CONFIRM_SAMPLES;
    localparam integer ADC_TICKS = (TICK_HZ * ADC_TIMEOUT_MS / 1000 < 1) ?
                                   1 : TICK_HZ * ADC_TIMEOUT_MS / 1000;
    localparam integer SWING_TICKS = (TICK_HZ * SWING_TIMEOUT_MS / 1000 < 1) ?
                                     1 : TICK_HZ * SWING_TIMEOUT_MS / 1000;
    localparam integer PULSE_W = (PULSE_TICKS < 2) ? 1 : $clog2(PULSE_TICKS);
    localparam integer TURN_AGE_W = (TURN_STALE_TICKS < 2) ? 1 : $clog2(TURN_STALE_TICKS);
    localparam integer TURN_COUNT_W = $clog2(TURN_SAMPLES + 1);
    localparam integer ADC_W = $clog2(ADC_TICKS + 1);
    localparam integer SWING_W = $clog2(SWING_TICKS + 1);
    localparam integer CONFIRM_TICKS = (TICK_HZ * CONFIRM_TIME_MS / 1000 < 1) ?
                                       1 : TICK_HZ * CONFIRM_TIME_MS / 1000;
    localparam integer CONFIRM_W = $clog2(CONFIRM_TICKS + 1);
    localparam signed [18:0] PWM_RECOVERY_STEP = START_PWM - PEAK_PWM_MIN >= 4 ?
                                               (START_PWM - PEAK_PWM_MIN) / 4 : 1;
    localparam signed [17:0] WRAP_COUNT = {2'b00, ADC_WRAP};
    localparam signed [17:0] HALF_WRAP = WRAP_COUNT / 2;
    localparam signed [17:0] STEP_LIMIT = {2'b00, ADC_STEP_MAX};
    localparam integer CAPTURE_MIN = $signed({1'b0, BASE_ANGLE}) - CAPTURE_ANGLE_ERROR_MAX;
    localparam integer CAPTURE_MAX = $signed({1'b0, BASE_ANGLE}) + CAPTURE_ANGLE_ERROR_MAX;
    localparam signed [17:0] LOW_PHASE = $signed({2'b00, ANGLE_MIN}) -
                                        $signed({2'b00, BASE_ANGLE});
    localparam signed [17:0] HIGH_PHASE = $signed({2'b00, ANGLE_MAX}) -
                                         $signed({2'b00, BASE_ANGLE});
    // 即使错过狭窄窗口的接近阶段，也必须在下一个转折点重新布防。
    localparam signed [17:0] LOW_CAPTURE_PHASE = LOW_PHASE > -CAPTURE_ANGLE_ERROR_MAX ?
                                               LOW_PHASE : -CAPTURE_ANGLE_ERROR_MAX;
    localparam signed [17:0] HIGH_CAPTURE_PHASE = HIGH_PHASE < CAPTURE_ANGLE_ERROR_MAX ?
                                                HIGH_PHASE : CAPTURE_ANGLE_ERROR_MAX;

    reg have_angle;
    reg adc_seen;
    reg signed [17:0] phase_value;
    reg signed [17:0] raw_delta, phase_next, phase_delta;
    reg [PULSE_W-1:0] pulse_count;
    reg signed [14:0] pulse_amplitude;
    reg [ADC_W-1:0] adc_age;
    reg [SWING_W-1:0] swing_age;
    reg start_pending;
    reg turn_sample_pending, turn_prev_valid, turn_pipeline, turn_fresh_snapshot;
    reg signed [17:0] turn_prev_phase, turn_phase_snapshot, turn_previous_snapshot;
    reg signed [17:0] turn_peak_phase;
    reg signed [18:0] low_capture_gap, high_capture_gap;
    reg signed [18:0] low_peak_gap, high_peak_gap;
    reg low_peak_valid, high_peak_valid;
    reg [TURN_AGE_W-1:0] turn_sample_age;
    reg turn_direction_valid, turn_direction, turn_candidate_direction;
    reg [TURN_COUNT_W-1:0] turn_count;
    reg turn_pulse, turn_low_side;
    reg capture_armed;
    reg confirm_pending, confirm_prev_valid;
    reg signed [17:0] confirm_prev_phase;
    reg [CONFIRM_W-1:0] confirm_count;
    reg [1:0] confirm_pipeline;
    reg signed [17:0] confirm_phase_snapshot, confirm_previous_snapshot;
    reg signed [18:0] confirm_step;
    reg confirm_fresh_snapshot, confirm_near;
    reg [1:0] confirmed_exit_count;

    wire startup_10xxx = !have_angle && raw_angle >= 16'd10000 && raw_angle <= 16'd10999;
    // 并行比较常量窗口，避免先进行回绕加法、再进行有符号比较。
    wire direct_step_ok = raw_delta >= -HALF_WRAP && raw_delta <= HALF_WRAP &&
                          raw_delta >= -STEP_LIMIT && raw_delta <= STEP_LIMIT;
    wire positive_wrap_step_ok = raw_delta > HALF_WRAP &&
                                 raw_delta >= WRAP_COUNT - STEP_LIMIT &&
                                 raw_delta <= WRAP_COUNT + STEP_LIMIT;
    wire negative_wrap_step_ok = raw_delta < -HALF_WRAP &&
                                 raw_delta >= -WRAP_COUNT - STEP_LIMIT &&
                                 raw_delta <= -WRAP_COUNT + STEP_LIMIT;
    wire raw_acceptable = raw_angle <= ADC_WRAP && !startup_10xxx &&
                          (!have_angle || direct_step_ok || positive_wrap_step_ok || negative_wrap_step_ok);
    wire sample_accepted = raw_valid && raw_acceptable;
    wire history_break = raw_valid && (!raw_acceptable ||
                          (have_angle && (raw_delta > HALF_WRAP || raw_delta < -HALF_WRAP ||
                                          phase_delta > HALF_WRAP || phase_delta < -HALF_WRAP)));
    wire adc_expired = adc_seen && adc_age >= ADC_TICKS;
    wire swinging = state >= WAIT_PEAK && state <= RETURN_POS;
    wire attempting = swinging || (state == BALANCE && !balance_confirmed);
    wire swing_expired = attempting && swing_age >= SWING_TICKS;
    wire in_balance_range = angle_value >= ANGLE_MIN && angle_value <= ANGLE_MAX;
    wire raw_in_balance_range = raw_angle >= ANGLE_MIN && raw_angle <= ANGLE_MAX;
    wire raw_in_capture_window = raw_in_balance_range &&
                                 $signed({1'b0, raw_angle}) >= CAPTURE_MIN &&
                                 $signed({1'b0, raw_angle}) <= CAPTURE_MAX;
    wire restart_request = start_enable &&
                           (sample_accepted ? !raw_in_balance_range :
                            (!angle_valid || adc_expired || !in_balance_range));
    wire signed [18:0] turn_peak_gap = turn_low_side ? low_capture_gap : high_capture_gap;
    wire same_side_peak_valid = turn_low_side ? low_peak_valid : high_peak_valid;
    wire signed [18:0] previous_peak_gap = turn_low_side ? low_peak_gap : high_peak_gap;
    wire signed [18:0] peak_advance = previous_peak_gap - turn_peak_gap;
    // 简化限制：使用半周期线性预测；若实机峰值增长明显变化，则需重新校准。
    wire signed [18:0] predicted_peak_gap = same_side_peak_valid && peak_advance > 0 ?
                                          turn_peak_gap - (peak_advance >>> 1) : turn_peak_gap;
    wire signed [18:0] peak_pwm_request = $signed(PEAK_PWM_MIN) + (predicted_peak_gap >>> 2);
    wire signed [14:0] peak_pwm_target = peak_pwm_request >= START_PWM ? START_PWM :
                                       peak_pwm_request <= PEAK_PWM_MIN ? PEAK_PWM_MIN :
                                       peak_pwm_request[14:0];
    wire peak_can_recover = !same_side_peak_valid || peak_advance <= 0;
    // 只有预测仍要求满驱动时才使用半区间恢复。
    wire signed [18:0] peak_pwm_recovery = $signed(pulse_amplitude) +
        (peak_pwm_request >= START_PWM ? (PWM_RECOVERY_STEP <<< 1) : PWM_RECOVERY_STEP);
    wire signed [17:0] confirm_phase = sample_accepted ? phase_next : phase_value;
    wire turn_positive = turn_phase_snapshot > turn_previous_snapshot;
    wire turn_high = turn_phase_snapshot > HIGH_CAPTURE_PHASE &&
                     turn_previous_snapshot > HIGH_CAPTURE_PHASE;
    wire turn_low = turn_phase_snapshot < LOW_CAPTURE_PHASE &&
                    turn_previous_snapshot < LOW_CAPTURE_PHASE;
    wire confirm_quiet = confirm_near &&
                         confirm_step >= -CONFIRM_STEP_MAX && confirm_step <= CONFIRM_STEP_MAX;

    assign balance_enable = state == BALANCE && angle_valid && in_balance_range &&
                            !adc_expired && !swing_expired;
    // 反转脉冲到来时峰值已稳定；选择方向前先计算两侧结果。
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            low_capture_gap <= LOW_CAPTURE_PHASE;
            high_capture_gap <= -HIGH_CAPTURE_PHASE;
        end else begin
            low_capture_gap <= LOW_CAPTURE_PHASE - turn_peak_phase;
            high_capture_gap <= turn_peak_phase - HIGH_CAPTURE_PHASE;
        end
    end
    always @* begin
        raw_delta = $signed({2'b00, raw_angle}) - $signed({2'b00, angle_value});

        // 以直立位置为中心计算有符号相位，使传感器的 27100/0 回绕边界落在同一侧。
        phase_next = $signed({2'b00, raw_angle}) - $signed({2'b00, BASE_ANGLE});
        if (phase_next > HALF_WRAP)
            phase_next = phase_next - WRAP_COUNT;
        else if (phase_next < -HALF_WRAP)
            phase_next = phase_next + WRAP_COUNT;
        phase_delta = phase_next - phase_value;
    end

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            angle_value <= 16'd0;
            angle_valid <= 1'b0;
            phase_value <= 18'sd0;
            have_angle <= 1'b0;
            adc_seen <= 1'b0;
            adc_age <= 0;
        end else if (restart_request && (!angle_valid || adc_expired)) begin
            // 保留有效锚点；只有手动重试时才用被拒绝或过期的数据重新定位。
            angle_value <= 16'd0;
            angle_valid <= 1'b0;
            phase_value <= 18'sd0;
            have_angle <= 1'b0;
            adc_seen <= 1'b0;
            adc_age <= 0;
        end else begin
            if (tick && adc_seen && !adc_expired)
                adc_age <= adc_age + 1'b1;
            if (adc_expired)
                angle_valid <= 1'b0;
            if (raw_valid) begin
                if (!adc_seen && raw_angle <= ADC_WRAP) begin
                    adc_seen <= 1'b1;
                    adc_age <= 0;
                end
                if (raw_acceptable) begin
                    angle_value <= raw_angle;
                    phase_value <= phase_next;
                    angle_valid <= 1'b1;
                    have_angle <= 1'b1;
                    adc_age <= 0;
                end else begin
                    // 回绕跳变后，禁止重新定位到重复出现的 10xxx 平台值。
                    angle_valid <= 1'b0;
                end
            end
        end
    end

    // 按控制周期取快照；原始 ADC 事件和旧值不能作为额外的方向证据。
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            turn_sample_pending <= 0;
            turn_prev_valid <= 0;
            turn_pipeline <= 0;
            turn_fresh_snapshot <= 0;
            turn_prev_phase <= 0;
            turn_phase_snapshot <= 0;
            turn_previous_snapshot <= 0;
            turn_peak_phase <= 0;
            turn_sample_age <= 0;
            turn_direction_valid <= 0;
            turn_direction <= 0;
            turn_candidate_direction <= 0;
            turn_count <= 0;
            turn_pulse <= 0;
            turn_low_side <= 0;
        end else begin
            turn_pulse <= 0;
            turn_pipeline <= 0;
            if (state != WAIT_PEAK || restart_request || history_break || adc_expired ||
                (!angle_valid && !sample_accepted)) begin
                turn_sample_pending <= 0;
                turn_prev_valid <= 0;
                turn_sample_age <= 0;
                turn_direction_valid <= 0;
                turn_count <= 0;
                turn_peak_phase <= 0;
            end else begin
                if (sample_accepted)
                    turn_sample_pending <= 1;
                if (turn_pipeline) begin
                    if (!turn_fresh_snapshot || (!turn_high && !turn_low) ||
                        (turn_high && turn_phase_snapshot < turn_peak_phase) ||
                        (turn_low && turn_phase_snapshot > turn_peak_phase))
                        turn_peak_phase <= turn_phase_snapshot;
                    if (!turn_fresh_snapshot || (!turn_high && !turn_low)) begin
                        turn_direction_valid <= 0;
                        turn_count <= 0;
                    end else if (turn_phase_snapshot == turn_previous_snapshot) begin
                        // 平坦峰值保留接近方向信息，但不能证明已经反转。
                        turn_count <= 0;
                    end else if (turn_direction_valid && turn_positive == turn_direction) begin
                        turn_count <= 0;
                    end else if (TURN_SAMPLES == 1 ||
                                 (turn_count >= TURN_SAMPLES - 1 &&
                                  turn_positive == turn_candidate_direction)) begin
                        if (turn_direction_valid && ((turn_high && turn_positive) ||
                                                     (turn_low && !turn_positive))) begin
                            turn_pulse <= 1;
                            turn_low_side <= turn_low;
                        end else
                            // 刚确认的新方向会开始一轮新的峰值测量。
                            turn_peak_phase <= turn_phase_snapshot;
                        turn_direction <= turn_positive;
                        turn_direction_valid <= 1;
                        turn_count <= 0;
                    end else begin
                        turn_candidate_direction <= turn_positive;
                        turn_count <= turn_positive == turn_candidate_direction ? turn_count + 1'b1 : 1;
                    end
                end
                if (tick) begin
                    turn_sample_pending <= 0;
                    if (turn_sample_pending || sample_accepted) begin
                        turn_sample_age <= 0;
                        turn_pipeline <= 1;
                        turn_phase_snapshot <= confirm_phase;
                        turn_previous_snapshot <= turn_prev_phase;
                        turn_fresh_snapshot <= turn_prev_valid;
                        turn_prev_phase <= confirm_phase;
                        turn_prev_valid <= 1;
                    end else if (turn_sample_age >= TURN_STALE_TICKS - 1) begin
                        turn_prev_valid <= 0;
                        turn_direction_valid <= 0;
                        turn_count <= 0;
                        turn_peak_phase <= 0;
                    end else
                        turn_sample_age <= turn_sample_age + 1'b1;
                end
            end
        end
    end

    // 正常跨越回绕边界时保留已完成的峰值；PID 驱动和无效采样会中断峰值趋势。
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            low_peak_gap <= 0;
            high_peak_gap <= 0;
            low_peak_valid <= 0;
            high_peak_valid <= 0;
        end else if (restart_request || state == WAIT_ADC || state == BALANCE || state == STOP ||
                     adc_expired || (raw_valid && !raw_acceptable) ||
                     (!angle_valid && !sample_accepted) ||
                     (state == WAIT_PEAK && turn_sample_age >= TURN_STALE_TICKS - 1)) begin
            low_peak_valid <= 0;
            high_peak_valid <= 0;
        end else if (state == WAIT_PEAK && turn_pulse && angle_valid && !history_break) begin
            if (turn_low_side) begin
                low_peak_gap <= turn_peak_gap;
                low_peak_valid <= 1;
            end else begin
                high_peak_gap <= turn_peak_gap;
                high_peak_valid <= 1;
            end
        end
    end

    // 只有连续且有效的安静控制周期才能确认捕获成功。
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            balance_confirmed <= 1'b0;
            confirm_pending <= 1'b0;
            confirm_prev_valid <= 1'b0;
            confirm_prev_phase <= 0;
            confirm_count <= 0;
            confirm_pipeline <= 0;
            confirm_phase_snapshot <= 0;
            confirm_previous_snapshot <= 0;
            confirm_step <= 0;
            confirm_fresh_snapshot <= 0;
            confirm_near <= 0;
        end else if (state != BALANCE || capture_pulse || !angle_valid) begin
            balance_confirmed <= state == BALANCE && angle_valid && CONFIRM_TIME_MS == 0;
            confirm_pending <= 1'b0;
            confirm_prev_valid <= 1'b0;
            confirm_prev_phase <= 0;
            confirm_count <= 0;
            confirm_pipeline <= 0;
        end else begin
            // 在 50 MHz 下分别流水处理校验后的相位选择、速度计算和持续时间计数。
            confirm_pipeline <= {confirm_pipeline[0], tick};
            if (sample_accepted)
                confirm_pending <= 1'b1;
            if (tick) begin
                confirm_pending <= 1'b0;
                confirm_prev_valid <= angle_valid && !history_break &&
                                      (confirm_pending || sample_accepted);
                confirm_prev_phase <= confirm_phase;
                confirm_phase_snapshot <= confirm_phase;
                confirm_previous_snapshot <= confirm_prev_phase;
                confirm_fresh_snapshot <= !history_break &&
                                          (confirm_pending || sample_accepted) && confirm_prev_valid;
            end
            if (confirm_pipeline[0]) begin
                confirm_step <= $signed({confirm_phase_snapshot[17], confirm_phase_snapshot}) -
                                $signed({confirm_previous_snapshot[17], confirm_previous_snapshot});
                confirm_near <= confirm_fresh_snapshot &&
                                confirm_phase_snapshot >= -CONFIRM_ERROR_MAX &&
                                confirm_phase_snapshot <= CONFIRM_ERROR_MAX;
            end
            if (confirm_pipeline[1]) begin
                if (!confirm_quiet)
                    confirm_count <= 0;
                else if (!balance_confirmed) begin
                    if (confirm_count >= CONFIRM_TICKS - 1)
                        balance_confirmed <= 1'b1;
                    else
                        confirm_count <= confirm_count + 1'b1;
                end
            end
        end
    end

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= WAIT_ADC;
            swing_pwm <= 15'sd0;
            confirmed_exit_count <= 0;
            capture_pulse <= 1'b0;
            pulse_count <= 0;
            pulse_amplitude <= START_PWM;
            swing_age <= 0;
            start_pending <= 1'b0;
            capture_armed <= 1'b1;
        end else begin
            if (adc_expired || swing_expired)
                swing_pwm <= 15'sd0;
            else if (state == KICK_POS || state == RETURN_POS)
                swing_pwm <= pulse_amplitude;
            else if (state == KICK_NEG || state == RETURN_NEG)
                swing_pwm <= -pulse_amplitude;
            else
                swing_pwm <= 15'sd0;

            capture_pulse <= 1'b0;
            if (state != BALANCE)
                confirmed_exit_count <= 0;
            if (tick && attempting && !swing_expired)
                swing_age <= swing_age + 1'b1;

            if (restart_request) begin
                state <= WAIT_ADC;
                start_pending <= 1'b1;
                pulse_count <= 0;
                pulse_amplitude <= START_PWM;
                swing_age <= 0;
                capture_armed <= 1'b1;
                confirmed_exit_count <= 0;
            end else if (((swinging || state == BALANCE || start_pending) && adc_expired) || swing_expired) begin
                state <= STOP;
                start_pending <= 1'b0;
            end else if (swinging && capture_armed && sample_accepted && raw_in_capture_window) begin
                state <= BALANCE;
                swing_pwm <= 15'sd0;
                capture_pulse <= 1'b1;
                pulse_count <= 0;
                pulse_amplitude <= START_PWM;
            end else begin
                case (state)
                    WAIT_ADC: begin
                        // 发生运动后也可离开含义不明确的启动 10xxx 平台值。
                        if (start_pending && sample_accepted) begin
                            start_pending <= 1'b0;
                            pulse_count <= 0;
                            if (!raw_in_balance_range)
                                state <= KICK_POS;
                        end
                    end
                    KICK_POS, RETURN_NEG, KICK_NEG, RETURN_POS: begin
                        if (tick) begin
                            if (pulse_count >= PULSE_TICKS - 1) begin
                                pulse_count <= 0;
                                case (state)
                                    KICK_POS: state <= RETURN_NEG;
                                    KICK_NEG: state <= RETURN_POS;
                                    default: state <= WAIT_PEAK;
                                endcase
                            end else begin
                                pulse_count <= pulse_count + 1'b1;
                            end
                        end
                    end
                    WAIT_PEAK: begin
                        if (turn_pulse && angle_valid && !history_break) begin
                            state <= turn_low_side ? KICK_NEG : KICK_POS;
                            capture_armed <= 1'b1;
                            pulse_count <= 0;
                            // 同时锁存两个半区结果；仅在停滞或回退的峰值处恢复。
                            if (peak_pwm_target <= pulse_amplitude)
                                pulse_amplitude <= peak_pwm_target;
                            else if (peak_can_recover)
                                pulse_amplitude <= peak_pwm_target <= peak_pwm_recovery ?
                                                   peak_pwm_target : peak_pwm_recovery[14:0];
                        end
                    end
                    BALANCE: begin
                        if (!angle_valid)
                            state <= STOP;
                        else if (!in_balance_range) begin
                            if (balance_confirmed)
                                // 连续两次系统时钟周期检测到越界后停止。
                                if (confirmed_exit_count >= 1) begin
                                    state <= STOP;
                                    confirmed_exit_count <= 0;
                                end else
                                    confirmed_exit_count <= confirmed_exit_count + 1'b1;
                            else begin
                                state <= WAIT_PEAK;
                                // 再次进行 PID 捕获前，必须先检测到新的转折点。
                                capture_armed <= 1'b0;
                            end
                        end else
                            confirmed_exit_count <= 0;
                    end
                    STOP: state <= STOP;
                    default: state <= STOP;
                endcase
            end
        end
    end
    // 简化限制：启动时的 10xxx 值含义不唯一；应先确认非 10xxx 的锚点。
endmodule
