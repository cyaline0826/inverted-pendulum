// 根据摆杆角度误差计算电机控制量。
module pid_angle #(
    parameter [15:0] STOP_MIN = 16'd6000,
    parameter [15:0] STOP_MAX = 16'd12000,
    parameter signed [15:0] KP = 16'sd4,
    parameter signed [15:0] KI = 16'sd0,
    parameter signed [15:0] KD = 16'sd0,
    parameter integer GAIN_SHIFT = 4,
    parameter signed [14:0] OUT_MAX = 15'sd10000,
    parameter signed [31:0] INTEGRAL_MAX = 32'sd200000,
    parameter TRACK_OUTSIDE_RANGE = 1'b0,
    parameter integer INTEGRAL_ERROR_MAX = 32768,
    parameter HOLD_INTEGRAL_OUTSIDE_ZONE = 1'b0,
    parameter signed [15:0] CAPTURE_KD = KD,
    parameter integer CAPTURE_TICKS = 0,
    parameter integer CAPTURE_EXIT_ERROR_MAX = 1000,
    parameter integer CAPTURE_EXIT_STEP_MAX = 20, // 每个 PID 周期允许的 ADC 计数变化量。
    parameter integer CAPTURE_EXIT_TICKS = 4,
    parameter integer D_FILTER_SHIFT = 0, // 0 表示旁路；1 表示与上一次估计值取平均。
    parameter signed [15:0] CAPTURE_KP = KP,
    parameter signed [15:0] CAPTURE_BRAKE_KD = 16'sd0,
    parameter integer CAPTURE_BRAKE_STEP_MAX = 60,
    parameter integer CAPTURE_BRAKE_OUT_MAX = 2000,
    parameter integer CAPTURE_BRAKE_FILTER_SHIFT = 0, // 仅滤波额外制动项；基础 D 项保持不变。
    parameter USE_TIMED_VELOCITY = 1'b0,
    parameter integer CAPTURE_BRAKE_ERROR_MAX = CAPTURE_EXIT_ERROR_MAX,
    parameter integer CAPTURE_QUIET_ERROR_MAX = CAPTURE_EXIT_ERROR_MAX,
    parameter USE_TIMED_BALANCE = 1'b0
) (
    input  wire sys_clk,
    input  wire rst_n,
    input  wire sample_tick,
    input  wire [15:0] target_value,
    input  wire [15:0] angle_value,
    output reg signed [14:0] pid_out,
    output reg pid_enable,
    output reg signed [15:0] error,
    output reg signed [31:0] integral_term,
    input  wire measurement_fresh, // 自上一个 PID 周期以来是否收到可信的 ADC 采样。
    output reg capture_active, // 更新捕获状态前，本次 PWM 计算所使用的增益。
    output reg signed [15:0] diagnostic_step,
    output reg signed [15:0] diagnostic_d,
    output reg signed [15:0] diagnostic_brake,
    input wire signed [15:0] measured_velocity, // 每个标称控制周期的 ADC 计数变化；ADC 值上升时为正。
    input wire tracking_only, // 只更新角度历史，不推进捕获、积分或 PWM。
    input wire steady_state // 顶层稳摆确认和捕获保持阶段均已完成。
);

    localparam integer CAPTURE_W = (CAPTURE_TICKS < 2) ? 1 : $clog2(CAPTURE_TICKS + 1);
    localparam integer CAPTURE_EXIT_W = (CAPTURE_EXIT_TICKS < 2) ? 1 : $clog2(CAPTURE_EXIT_TICKS + 1);
    reg [CAPTURE_W-1:0] capture_count;
    reg [CAPTURE_EXIT_W-1:0] capture_quiet_count;
    wire signed [15:0] derivative_gain = (capture_count < CAPTURE_TICKS) ? CAPTURE_KD : KD;
    wire signed [15:0] proportional_gain = (capture_count < CAPTURE_TICKS) ? CAPTURE_KP : KP;

    reg signed [15:0] last_angle;
    reg angle_prev_valid;
    reg signed [15:0] error_next;
    reg signed [15:0] derivative_raw;
    reg signed [15:0] derivative;
    reg signed [31:0] derivative_accum;
    reg signed [31:0] derivative_next;
    reg signed [31:0] integral_accum;
    reg signed [31:0] integral_next;
    reg signed [31:0] integral_step;
    reg signed [47:0] integral_product;
    reg signed [47:0] diagnostic_d_product;
    reg signed [47:0] pid_sum;
    reg signed [47:0] pid_scaled;
    reg signed [31:0] capture_brake_step;
    reg signed [31:0] brake_velocity_accum, brake_velocity_next, brake_velocity;
    reg signed [47:0] capture_brake_sum;
    reg signed [47:0] brake_product, brake_capped, brake_scaled;
    reg signed [31:0] brake_ramp_weight;
    reg signed [79:0] brake_ramp_product;
    reg signed [31:0] brake_error_abs;
    localparam integer BRAKE_RAMP_WIDTH = CAPTURE_BRAKE_ERROR_MAX > CAPTURE_EXIT_ERROR_MAX ?
                                        CAPTURE_BRAKE_ERROR_MAX - CAPTURE_EXIT_ERROR_MAX : 1;
    localparam signed [47:0] BRAKE_LIMIT = CAPTURE_BRAKE_OUT_MAX * (48'sd1 << GAIN_SHIFT);
    reg integral_in_zone;
    wire in_control_range = (angle_value >= STOP_MIN) &&
                             (angle_value <= STOP_MAX);
    wire timed_velocity_active = USE_TIMED_VELOCITY || (USE_TIMED_BALANCE && steady_state);
    wire capture_quiet = (CAPTURE_EXIT_TICKS > 0) && measurement_fresh && angle_prev_valid &&
                         (error_next >= -CAPTURE_QUIET_ERROR_MAX) &&
                         (error_next <= CAPTURE_QUIET_ERROR_MAX) &&
                         (derivative >= -CAPTURE_EXIT_STEP_MAX) &&
                         (derivative <= CAPTURE_EXIT_STEP_MAX);

    always @* begin
        error_next = $signed({1'b0, target_value}) -
                     $signed({1'b0, angle_value});
        derivative_raw = angle_prev_valid ? last_angle - $signed(angle_value) : 16'sd0;
        // 保留小数余量，避免平滑处理抹去缓慢的实测运动。
        derivative_next = angle_prev_valid ? derivative_accum -
                          (derivative_accum >>> D_FILTER_SHIFT) + derivative_raw : 32'sd0;
        derivative = timed_velocity_active ? -measured_velocity : derivative_next >>> D_FILTER_SHIFT;
        // 保留小数余量可维持稳定制动，同时避免原始 ADC 读取的波动。
        brake_velocity_next = angle_prev_valid ? brake_velocity_accum -
                              (brake_velocity_accum >>> CAPTURE_BRAKE_FILTER_SHIFT) + derivative : 32'sd0;
        brake_velocity = timed_velocity_active ? derivative : brake_velocity_next >>> CAPTURE_BRAKE_FILTER_SHIFT;
        integral_step = KI * error_next;
        integral_in_zone = (error_next >= -INTEGRAL_ERROR_MAX) &&
                           (error_next <= INTEGRAL_ERROR_MAX);

        // 在积分区外冻结学习到的起摆补偿；摆杆跌落时仍清除该补偿。
        if (!integral_in_zone)
            integral_next = HOLD_INTEGRAL_OUTSIDE_ZONE ? integral_accum : 32'sd0;
        else if (integral_accum + error_next > INTEGRAL_MAX)
            integral_next = INTEGRAL_MAX;
        else if (integral_accum + error_next < -INTEGRAL_MAX)
            integral_next = -INTEGRAL_MAX;
        else
            integral_next = integral_accum + error_next;

        integral_product = KI * integral_next;
        diagnostic_d_product = derivative_gain * derivative;
        // 仅在接近目标时延长制动；远离目标时保留抬升控制能力。
        brake_error_abs = error_next < 0 ? -$signed({error_next[15], error_next}) : error_next;
        capture_brake_step = 0;
        if (capture_count < CAPTURE_TICKS && angle_prev_valid &&
            (brake_error_abs <= CAPTURE_EXIT_ERROR_MAX ||
             (brake_error_abs < CAPTURE_BRAKE_ERROR_MAX &&
              ((error_next > 0 && brake_velocity < 0) ||
               (error_next < 0 && brake_velocity > 0))))) begin
            if (brake_velocity > CAPTURE_BRAKE_STEP_MAX)
                capture_brake_step = brake_velocity - CAPTURE_BRAKE_STEP_MAX;
            else if (brake_velocity < -CAPTURE_BRAKE_STEP_MAX)
                capture_brake_step = brake_velocity + CAPTURE_BRAKE_STEP_MAX;
        end
        brake_product = CAPTURE_BRAKE_KD * capture_brake_step;
        brake_capped = brake_product > BRAKE_LIMIT ? BRAKE_LIMIT :
                       brake_product < -BRAKE_LIMIT ? -BRAKE_LIMIT : brake_product;
        brake_ramp_weight = BRAKE_RAMP_WIDTH;
        if (brake_error_abs > CAPTURE_EXIT_ERROR_MAX && brake_error_abs < CAPTURE_BRAKE_ERROR_MAX)
            brake_ramp_weight = CAPTURE_BRAKE_ERROR_MAX - brake_error_abs;
        // 使用独立的有符号中间值，确保斜坡期间限幅结果始终有界。
        brake_ramp_product = brake_capped * brake_ramp_weight;
        brake_scaled = brake_ramp_product / BRAKE_RAMP_WIDTH;
        capture_brake_sum = brake_scaled > BRAKE_LIMIT ? BRAKE_LIMIT :
                            brake_scaled < -BRAKE_LIMIT ? -BRAKE_LIMIT : brake_scaled;
        pid_sum = (proportional_gain * error_next) + integral_product +
                  (derivative_gain * derivative) + capture_brake_sum;
        pid_scaled = pid_sum >>> GAIN_SHIFT;
    end

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            pid_out          <= 15'sd0;
            pid_enable       <= 1'b0;
            error            <= 16'sd0;
            last_angle       <= 16'sd0;
            angle_prev_valid <= 1'b0;
            derivative_accum <= 32'sd0;
            brake_velocity_accum <= 32'sd0;
            integral_accum   <= 32'sd0;
            integral_term    <= 32'sd0;
            capture_count    <= 0;
            capture_quiet_count <= 0;
            capture_active   <= 1'b0;
            diagnostic_step  <= 0;
            diagnostic_d     <= 0;
            diagnostic_brake <= 0;
        end else if (sample_tick) begin
            if (in_control_range && !tracking_only) begin
                // 摆杆接近直立且运动平稳后，同时释放捕获阶段增益。
                if (capture_count < CAPTURE_TICKS) begin
                    if (capture_quiet && capture_quiet_count >= CAPTURE_EXIT_TICKS - 1) begin
                        capture_count <= CAPTURE_TICKS;
                        capture_quiet_count <= 0;
                    end else begin
                        capture_count <= capture_count + 1'b1;
                        capture_quiet_count <= capture_quiet ? capture_quiet_count + 1'b1 : 0;
                    end
                end else begin
                    capture_quiet_count <= 0;
                end
                pid_out          <= (pid_scaled > OUT_MAX) ? OUT_MAX :
                                    (pid_scaled < -OUT_MAX) ? -OUT_MAX :
                                    pid_scaled[14:0];
                pid_enable       <= 1'b1;
                capture_active   <= capture_count < CAPTURE_TICKS;
                error            <= error_next;
                // 上报本次指令使用的积分贡献，饱和时也照常上报。
                integral_term    <= integral_product >>> GAIN_SHIFT;
                diagnostic_step  <= -derivative_raw;
                diagnostic_d     <= (diagnostic_d_product >>> GAIN_SHIFT) > 32767 ? 16'sd32767 :
                                    (diagnostic_d_product >>> GAIN_SHIFT) < -32768 ? 16'sh8000 :
                                    diagnostic_d_product >>> GAIN_SHIFT;
                diagnostic_brake <= capture_brake_sum >>> GAIN_SHIFT;
                last_angle       <= $signed(angle_value);
                angle_prev_valid <= 1'b1;
                derivative_accum <= derivative_next;
                brake_velocity_accum <= brake_velocity_next;
                // PWM 到达上限时停止累积积分；允许降低饱和程度的积分变化。
                if (!integral_in_zone ||
                    !((pid_scaled > OUT_MAX && integral_step > 0) ||
                      (pid_scaled < -OUT_MAX && integral_step < 0)))
                    integral_accum <= integral_next;
            end else begin
                pid_out          <= 15'sd0;
                pid_enable       <= 1'b0;
                capture_active   <= 1'b0;
                diagnostic_step  <= 0;
                diagnostic_d     <= 0;
                diagnostic_brake <= 0;
                error            <= 16'sd0;
                // 起摆阶段需要输入速度，但不允许产生越界的积分项或输出。
                last_angle       <= TRACK_OUTSIDE_RANGE ? $signed(angle_value) : 16'sd0;
                angle_prev_valid <= TRACK_OUTSIDE_RANGE;
                derivative_accum <= TRACK_OUTSIDE_RANGE ? derivative_next : 32'sd0;
                brake_velocity_accum <= TRACK_OUTSIDE_RANGE ? brake_velocity_next : 32'sd0;
                integral_accum   <= 32'sd0;
                integral_term    <= 32'sd0;
                capture_count    <= 0;
                capture_quiet_count <= 0;
            end
        end
    end

endmodule
