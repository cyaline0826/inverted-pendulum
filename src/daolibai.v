// 顶层模块，连接起摆、稳摆、位置控制、显示和串口功能。
module daolibai #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer CONTROL_HZ  = 500, // 内环周期：2 ms。
    parameter integer POSITION_HZ = 100, // 外环周期：10 ms。
    parameter integer ENCODER_SAMPLE_HZ = 1000,
    parameter signed [15:0] BASE_ANGLE_TARGET = 16'sd8316,
    parameter signed [15:0] ANGLE_MIN = BASE_ANGLE_TARGET - 16'sd2962,
    parameter signed [15:0] ANGLE_MAX = BASE_ANGLE_TARGET + 16'sd2138,
    parameter signed [15:0] ANGLE_OFFSET_MAX = 16'sd1200,
    parameter signed [15:0] POSITION_OFFSET_STEP = 16'sd10,
    parameter signed [31:0] POSITION_TARGET = 32'sd0,
    parameter [15:0] KNOB_ADC_FULL_SCALE = 16'd27065, // A0 高端值。
    parameter signed [31:0] KNOB_TRAVEL_COUNTS = 32'sd1533, //摆臂每转编码器计数。
    parameter signed [31:0] KNOB_TARGET_STEP = 32'sd4, // 每个 10 ms 位置周期的计数步长。
    parameter signed [15:0] POS_KP = 16'sd15,
    parameter signed [15:0] POS_KD = 16'sd620,
    parameter integer POS_GAIN_SHIFT = 4,
    parameter signed [14:0] START_PWM = 15'sd3000, // 占空比 30%；
    parameter signed [14:0] PEAK_PWM_MIN = 15'sd2680, // 接近目标窗口时的经验下限；
    parameter integer START_TIME_MS = 100,
    parameter integer TURN_SAMPLE_TIMEOUT_MS = 40, // 未收到新采样后清除转向历史。
    parameter [15:0] ADC_WRAP = 16'd27100,
    parameter [15:0] ADC_STEP_MAX = 16'd2000, // 每个有效 ADC 采样允许的最大变化量。
    parameter integer ADC_TIMEOUT_MS = 500,
    parameter integer SWING_TIMEOUT_MS = 10000,
    parameter integer CONFIRM_TIME_MS = 100, // 设为 0 时立即确认稳摆。
    parameter integer CONFIRM_ERROR_MAX = 500,
    parameter integer CAPTURE_ANGLE_ERROR_MAX = 1000, // 限制PID 误差。
    parameter integer CAPTURE_HOLD_MS = 300, // 捕获阶段 P/D 控制和外环保持的最长时间。
    parameter signed [15:0] CAPTURE_KD = 16'sd350, // 仅用于捕获阶段的阻尼试调值；
    parameter integer D_FILTER_SHIFT = 0, 
    parameter signed [15:0] CAPTURE_KP = 16'sd30,
    parameter signed [15:0] CAPTURE_BRAKE_KD = 16'sd250,
    parameter integer CAPTURE_BRAKE_STEP_MAX = 30000 / CONTROL_HZ,
    parameter integer CAPTURE_BRAKE_OUT_MAX = 2000, // 额外阻尼 PWM 上限
    parameter integer CAPTURE_BRAKE_FILTER_SHIFT = 0, // 旁路滤波。
    parameter CAPTURE_DIAGNOSTIC = 1'b0, 
    parameter USE_TIMED_VELOCITY = 1'b0, 
    parameter integer CAPTURE_BRAKE_ERROR_MAX = 2500, 
    parameter integer TURN_CONFIRM_SAMPLES = 3 // 每个方向需要确认的有效非零步数。
) (
    input  wire sys_clk,
    input  wire rst_n,
    input  wire KEY1,
    input  wire KEY2,
    input  wire uart_rx,
    input  wire enc_a,
    input  wire enc_b,
    inout  wire adc_scl,
    inout  wire adc_sda,
    inout  wire oled_scl,
    inout  wire oled_sda,
    output wire uart_tx,
    output wire PWMB,
    output reg  BIN1,
    output reg  BIN2
);

    localparam integer CONTROL_DIV = CLK_FREQ_HZ / CONTROL_HZ;
    localparam integer CONTROL_W = (CONTROL_DIV < 2) ? 1 : $clog2(CONTROL_DIV);
    localparam integer POSITION_DIV = CLK_FREQ_HZ / POSITION_HZ;
    localparam integer POSITION_W = (POSITION_DIV < 2) ? 1 : $clog2(POSITION_DIV);
    localparam integer BAUD_RATE = CAPTURE_DIAGNOSTIC ? 38_400 : 115_200;
    localparam integer UART_SAMPLE_HZ = CAPTURE_DIAGNOSTIC ? 100 : 50;
    localparam integer UART_SAMPLE_DIV = CLK_FREQ_HZ / UART_SAMPLE_HZ;
    localparam integer LINE_LEN = CAPTURE_DIAGNOSTIC ? 37 : 25;
    localparam integer UART_FIELD_COUNT = CAPTURE_DIAGNOSTIC ? 6 : 2;
    localparam signed [14:0] MAX_PWM = 15'sd10000;
    localparam signed [15:0] PID_ANGLE_MIN = ANGLE_MIN;
    localparam signed [15:0] PID_ANGLE_MAX = ANGLE_MAX;
    localparam integer CAPTURE_TICKS = CONTROL_HZ * CAPTURE_HOLD_MS / 1000;
    localparam integer CAPTURE_W = (CAPTURE_TICKS < 2) ? 1 : $clog2(CAPTURE_TICKS + 1);

    reg [CONTROL_W-1:0] control_cnt;
    reg sample_tick;
    reg [POSITION_W-1:0] position_cnt;
    reg position_tick;
    reg [CAPTURE_W-1:0] capture_count;

    wire [15:0] adc_data_a2;
    wire adc_valid;
    wire [15:0] knob_adc_value;
    wire knob_adc_valid;
    wire [15:0] angle_value;
    wire adc_ready;
    wire angle_sample_tick;
    reg adc_valid_d;
    reg angle_update_pending;
    wire control_enable;
    wire balance_enable;
    wire balance_confirmed;
    wire capture_pulse;
    wire [2:0] swing_state;
    wire signed [14:0] swing_pwm;
    reg pid_rst_n;
    wire position_rst_n;
    wire key1_down;
    wire key2_down, key2_profile_active, key2_busy, key2_in_position;
    wire [15:0] key2_accepted_steps;
    wire signed [31:0] key2_position_offset, key2_requested_offset, key2_velocity_q8;

    wire signed [31:0] enc_position;
    wire signed [31:0] enc_speed;
    reg signed [31:0] position_feedback;
    wire signed [31:0] position_target;
    wire signed [31:0] knob_target_position;
    wire signed [31:0] position_velocity_q8;
    wire signed [31:0] knob_requested_position;
    wire signed [32:0] key2_position_error;
    wire signed [33:0] key2_position_error_abs, key2_speed_abs;
    reg signed [31:0] balance_position_origin;
    wire signed [31:0] serial_target_absolute;
    wire signed [32:0] serial_target_relative;
    wire serial_target_relative_fits;
    wire serial_command_valid;
    wire [7:0] uart_rx_data;
    wire uart_rx_valid;
    reg signed [31:0] serial_target_local;
    reg signed [31:0] serial_target_requested;
    wire signed [32:0] serial_target_ramp_error;
    reg serial_target_active;
    reg balance_confirmed_d;
    wire signed [15:0] angle_offset;
    wire position_result_valid;
    wire signed [31:0] sampled_position;
    wire signed [32:0] sampled_position_delta;
    reg signed [15:0] applied_angle_offset;
    reg signed [31:0] applied_position;
    reg signed [32:0] applied_position_delta;
    reg signed [15:0] position_offset_limit;
    wire signed [15:0] next_position_offset_limit =
        position_offset_limit < ANGLE_OFFSET_MAX - POSITION_OFFSET_STEP ?
        position_offset_limit + POSITION_OFFSET_STEP : ANGLE_OFFSET_MAX;
    reg  signed [15:0] angle_target;

    wire signed [14:0] pid_out;
    wire signed [15:0] angle_loop_error;
    wire signed [31:0] angle_integral_term;
    wire capture_active;
    wire signed [15:0] diagnostic_step, diagnostic_d, diagnostic_brake;
    wire signed [15:0] measured_velocity;
    reg diagnostic_tick_d, diagnostic_capture_d, diagnostic_fresh_d;
    reg [15:0] diagnostic_angle_d;
    reg diagnostic_started, diagnostic_done;
    reg [9:0] diagnostic_count, diagnostic_read;
    reg [82:0] diagnostic_memory [0:511];
    reg [82:0] diagnostic_record;
    reg [15:0] pid_angle_value;
    reg signed [15:0] pid_target_value;
    reg [15:0] pid_angle_snapshot;
    reg signed [15:0] pid_target_snapshot, pid_velocity_snapshot;
    reg pid_fresh_snapshot, pid_capture_snapshot, pid_tracking_snapshot;
    reg pid_steady_snapshot;
    reg pid_pending, pid_request_enabled;
    reg [4:0] pid_settle_count;
    wire pid_snapshot_in_range = pid_angle_snapshot >= PID_ANGLE_MIN &&
                                 pid_angle_snapshot <= PID_ANGLE_MAX;
    wire pid_result_tick = pid_pending && pid_settle_count == 0 && !angle_sample_tick;
    wire pid_compute_tick = pid_result_tick && pid_request_enabled && pid_rst_n &&
                            adc_ready && (!pid_snapshot_in_range || pid_tracking_snapshot || balance_enable);
    wire signed [14:0] motor_speed;
    reg signed [14:0] motor_command;
    wire pwm_out;
    wire motor_dir;
    reg  motor_dir_d;
    reg  dir_deadtime;
    reg motor_stopped;

    reg [31:0] uart_sample_cnt;
    reg uart_sample_req;
    reg signed [31:0] uart_target_value;
    reg signed [31:0] uart_act_value;
    reg signed [31:0] uart_position_value;
    reg signed [15:0] uart_out_value;
    // KI=1 且右移 3 位时，积分贡献限制在 +/-25000 以内。
    reg signed [15:0] uart_integral_value;
    reg [3:0] uart_control_stage; // 位置模式：状态 0-7，待确认状态 8，已确认状态 9。
    reg signed [31:0] uart_rotor_speed; 
    reg signed [31:0] uart_position_target;
    wire signed [31:0] uart_fields [0:5];
    reg [31:0] uart_binary [0:UART_FIELD_COUNT-1];
    reg [39:0] uart_decimal [0:UART_FIELD_COUNT-1];
    reg [39:0] decimal_adjusted [0:UART_FIELD_COUNT-1];
    reg [UART_FIELD_COUNT-1:0] uart_negative;
    reg [5:0] decimal_left;
    integer decimal_field, decimal_digit, transmit_field;
    reg [7:0] line_index;
    reg sending;
    reg tx_start;
    reg [7:0] tx_data;
    wire tx_busy;
    assign uart_fields[0] = CAPTURE_DIAGNOSTIC ? uart_target_value : uart_position_target;
    assign uart_fields[1] = CAPTURE_DIAGNOSTIC ? uart_act_value : uart_position_value;
    assign uart_fields[2] = {{16{uart_out_value[15]}}, uart_out_value};
    assign uart_fields[3] = {{16{uart_integral_value[15]}}, uart_integral_value};
    assign uart_fields[4] = {29'd0, uart_control_stage};
    assign uart_fields[5] = uart_rotor_speed;
    assign motor_speed = !rst_n ? 15'sd0 :
                         balance_enable ? (control_enable ? pid_out : 15'sd0) : swing_pwm;
    assign PWMB = (motor_speed != 0) && pwm_out;
    assign position_target = serial_target_active ? serial_target_local : knob_target_position;
    assign position_velocity_q8 = serial_target_active ? 32'sd0 : key2_velocity_q8;
    assign key2_position_error = $signed({knob_requested_position[31], knob_requested_position}) -
                                 $signed({position_feedback[31], position_feedback});
    assign key2_position_error_abs = key2_position_error[32] ?
                                    -$signed({key2_position_error[32], key2_position_error}) :
                                     $signed({key2_position_error[32], key2_position_error});
    assign key2_speed_abs = sampled_position_delta[32] ?
                            -$signed({sampled_position_delta[32], sampled_position_delta}) :
                             $signed({sampled_position_delta[32], sampled_position_delta});
    assign key2_in_position = (key2_position_error_abs <= 34'sd8) &&
                              (key2_speed_abs <= 34'sd1) &&
                              (angle_loop_error >= -16'sd100) &&
                              (angle_loop_error <= 16'sd100);
    assign serial_target_relative =
        $signed({serial_target_absolute[31], serial_target_absolute}) -
        $signed({balance_position_origin[31], balance_position_origin});
    assign serial_target_relative_fits = serial_target_relative[32] == serial_target_relative[31];
    assign serial_target_ramp_error =
        $signed({serial_target_requested[31], serial_target_requested}) -
        $signed({serial_target_local[31], serial_target_local});
    // 捕获阶段请求立即计算，不必再等待一个 2 ms 控制周期。
    assign angle_sample_tick = sample_tick || capture_pulse;
    // 先让角度环稳定摆杆，再允许位置环移动目标。
    assign position_rst_n = rst_n && balance_confirmed && !capture_pulse &&
                            capture_count >= CAPTURE_TICKS;
    // 只有稳摆状态确认后才设定位置零点。
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            position_feedback <= 0;
            motor_command <= 0;
        end else begin
            position_feedback <= enc_position - balance_position_origin;
            motor_command <= motor_speed;
        end
    end

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            balance_position_origin <= 32'sd0;
            balance_confirmed_d <= 1'b0;
        end else begin
            balance_confirmed_d <= balance_confirmed;
            if (balance_confirmed && !balance_confirmed_d)
                balance_position_origin <= enc_position;
        end
    end

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            serial_target_local <= 0;
            serial_target_requested <= 0;
            serial_target_active <= 0;
        end else if (!position_rst_n || !control_enable) begin
            serial_target_local <= 0;
            serial_target_requested <= 0;
            serial_target_active <= 0;
        end else if (serial_command_valid && serial_target_relative_fits) begin
            if (!serial_target_active)
                serial_target_local <= position_target;
            serial_target_requested <= serial_target_relative[31:0];
            serial_target_active <= 1;
        end else if (serial_target_active && position_tick) begin
            if (serial_target_ramp_error > $signed({1'b0, KNOB_TARGET_STEP}))
                serial_target_local <= serial_target_local + KNOB_TARGET_STEP;
            else if (serial_target_ramp_error < -$signed({1'b0, KNOB_TARGET_STEP}))
                serial_target_local <= serial_target_local - KNOB_TARGET_STEP;
            else
                serial_target_local <= serial_target_requested;
        end
    end

    // 将复位门控寄存，避免状态译码毛刺清除捕获阶段的 D 项历史。
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n)
            pid_rst_n <= 1'b0;
        else
            pid_rst_n <= adc_ready && (swing_state >= 3'd1 && swing_state <= 3'd6);
    end

    // ADC 校验结果在 raw_valid 后一个时钟更新；保留该结果直到 PID 读取。
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            adc_valid_d <= 1'b0;
            angle_update_pending <= 1'b0;
            pid_angle_value <= 16'd0;
            pid_target_value <= 16'sd0;
            pid_angle_snapshot <= 0;
            pid_target_snapshot <= 0;
            pid_velocity_snapshot <= 0;
            pid_fresh_snapshot <= 0;
            pid_capture_snapshot <= 0;
            pid_tracking_snapshot <= 0;
            pid_steady_snapshot <= 0;
            pid_pending <= 0;
            pid_request_enabled <= 0;
            pid_settle_count <= 0;
        end else begin
            adc_valid_d <= adc_valid;
            if (!pid_rst_n || !adc_ready || angle_sample_tick)
                angle_update_pending <= 1'b0;
            else if (adc_valid_d)
                angle_update_pending <= 1'b1;
            if (angle_sample_tick) begin
                pid_angle_snapshot <= angle_value;
                pid_target_snapshot <= angle_target;
                pid_velocity_snapshot <= measured_velocity;
                pid_fresh_snapshot <= angle_update_pending || (adc_valid_d && adc_ready);
                pid_capture_snapshot <= capture_pulse;
                pid_tracking_snapshot <= !balance_enable;
                pid_steady_snapshot <= balance_enable && position_rst_n;
                pid_request_enabled <= pid_rst_n && adc_ready;
                // 将输入和历史值保持 32 个时钟周期；daolibai.sdc 只约束这条路径。
                pid_pending <= 1;
                pid_settle_count <= 31;
            end else begin
                if (!pid_rst_n || !adc_ready ||
                    (pid_snapshot_in_range && !pid_tracking_snapshot && !balance_enable))
                    pid_request_enabled <= 0;
                if (pid_pending) begin
                    if (pid_settle_count != 0)
                        pid_settle_count <= pid_settle_count - 1'b1;
                    else
                        pid_pending <= 0;
                end
            end
            if (pid_compute_tick) begin
                pid_angle_value <= pid_angle_snapshot;
                pid_target_value <= pid_target_snapshot;
            end
        end
    end

    key #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .ACTIVE_LEVEL(1'b0)
    ) u_key1 (
        .sys_clk  (sys_clk),
        .rst_n    (rst_n),
        .key_in   (KEY1),
        .key_state(),
        .key_down (key1_down)
    );

    key #(.CLK_FREQ_HZ(CLK_FREQ_HZ), .ACTIVE_LEVEL(1'b0)) u_key2 (
        .sys_clk(sys_clk), .rst_n(rst_n), .key_in(KEY2),
        .key_state(), .key_down(key2_down));

    uart #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .BAUD_RATE   (BAUD_RATE)
    ) u_debug_uart (
        .sys_clk  (sys_clk),
        .rst_n    (rst_n),
        .uart_rx  (uart_rx),
        .uart_tx  (uart_tx),
        .rx_data  (uart_rx_data),
        .rx_valid (uart_rx_valid),
        .tx_data  (tx_data),
        .tx_start (tx_start),
        .tx_busy  (tx_busy),
        .tx_done  ()
    );

    uart_position u_position_command (
        .sys_clk(sys_clk),
        .rst_n(rst_n),
        .rx_data(uart_rx_data),
        .rx_valid(uart_rx_valid),
        .accept_enable(position_rst_n && control_enable),
        .target_position(serial_target_absolute),
        .target_valid(serial_command_valid)
    );

    function [7:0] signed_field_char;
        input [4:0] field;
        input [3:0] pos;
        input [3:0] digits;
        reg [39:0] shifted;
        begin
            shifted = uart_decimal[field] >> ((digits - pos) * 4);
            signed_field_char = pos == 0 ? (uart_negative[field] ? "-" : "+") :
                                          "0" + shifted[3:0];
        end
    endfunction

    function [7:0] line_char;
        input [7:0] pos;
        begin
            if (CAPTURE_DIAGNOSTIC) begin
                if (pos < 6) line_char = signed_field_char(0, pos, 5);
                else if (pos < 13 && pos > 6) line_char = signed_field_char(1, pos-7, 5);
                else if (pos < 20 && pos > 13) line_char = signed_field_char(2, pos-14, 5);
                else if (pos < 27 && pos > 20) line_char = signed_field_char(3, pos-21, 5);
                else if (pos == 28) line_char = "0" + uart_decimal[4][3:0];
                else if (pos >= 30 && pos < 36) line_char = signed_field_char(5, pos-30, 5);
                else line_char = pos == 36 ? 8'h0A : ",";
            end else begin
                if (pos < 11) line_char = signed_field_char(0, pos, 10);
                else if (pos == 11) line_char = ",";
                else if (pos < 23) line_char = signed_field_char(1, pos-12, 10);
                else line_char = pos == 23 ? 8'h0D : 8'h0A;
            end
        end
    endfunction

    // 通过移位加 3 算法在 32 个时钟周期内转换冻结帧，避免 RAM 到十进制转换路径上的除法器。
    always @* begin
        for (decimal_field=0; decimal_field<UART_FIELD_COUNT; decimal_field=decimal_field+1) begin
            decimal_adjusted[decimal_field] = uart_decimal[decimal_field];
            for (decimal_digit=0; decimal_digit<10; decimal_digit=decimal_digit+1)
                if (uart_decimal[decimal_field][decimal_digit*4 +: 4] >= 5)
                    decimal_adjusted[decimal_field][decimal_digit*4 +: 4] =
                        uart_decimal[decimal_field][decimal_digit*4 +: 4] + 4'd3;
        end
    end

    // A2 角度采样优先；仅在稳摆确认后才低频采集 A0 电位器。
    adc #(
        .CLK_FREQ_HZ  (CLK_FREQ_HZ),
        .I2C_FREQ_HZ  (400_000),
        .ADS1115_ADDR (7'h48),
        .CONV_WAIT_US (1000)
    ) u_angle_adc (
        .sys_clk     (sys_clk),
        .rst_n       (rst_n),
        .a0_enable   (balance_confirmed && control_enable),
        .adc_data    (),
        .adc_data_a2 (adc_data_a2),
        .adc_valid   (adc_valid),
        .adc_data_a0 (knob_adc_value),
        .adc_a0_valid(knob_adc_valid),
        .config_data (),
        .busy        (),
        .ack_error   (),
        .adc_scl     (adc_scl),
        .adc_sda     (adc_sda)
    );

    swing_up #(
        .TICK_HZ          (CONTROL_HZ),
        .BASE_ANGLE       (BASE_ANGLE_TARGET),
        .ANGLE_MIN        (PID_ANGLE_MIN),
        .ANGLE_MAX        (PID_ANGLE_MAX),
        .START_PWM        (START_PWM),
        .PEAK_PWM_MIN     (PEAK_PWM_MIN),
        .START_TIME_MS    (START_TIME_MS),
        .TURN_SAMPLE_TIMEOUT_MS(TURN_SAMPLE_TIMEOUT_MS),
        .TURN_CONFIRM_SAMPLES(TURN_CONFIRM_SAMPLES),
        .ADC_WRAP         (ADC_WRAP),
        .ADC_STEP_MAX     (ADC_STEP_MAX),
        .ADC_TIMEOUT_MS   (ADC_TIMEOUT_MS),
        .SWING_TIMEOUT_MS (SWING_TIMEOUT_MS),
        .CONFIRM_TIME_MS  (CONFIRM_TIME_MS),
        .CONFIRM_ERROR_MAX(CONFIRM_ERROR_MAX),
        .CAPTURE_ANGLE_ERROR_MAX(CAPTURE_ANGLE_ERROR_MAX)
    ) u_swing_control (
        .sys_clk        (sys_clk),
        .rst_n          (rst_n),
        .tick           (sample_tick),
        .start_enable   (key1_down),
        .raw_valid      (adc_valid),
        .raw_angle      (adc_data_a2),
        .angle_value    (angle_value),
        .angle_valid    (adc_ready),
        .balance_enable (balance_enable),
        .balance_confirmed(balance_confirmed),
        .swing_pwm      (swing_pwm),
        .capture_pulse  (capture_pulse),
        .state          (swing_state)
    );

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            capture_count <= 0;
        end else begin
            if (!balance_enable || capture_pulse)
                capture_count <= 0;
            else if (sample_tick && capture_count < CAPTURE_TICKS)
                capture_count <= capture_count + 1'b1;
        end
    end

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            control_cnt <= {CONTROL_W{1'b0}};
            sample_tick <= 1'b0;
        end else begin
            sample_tick <= 1'b0;

            // 捕获阶段的立即计算会开启一个新的完整控制间隔。
            if (capture_pulse) begin
                control_cnt <= {CONTROL_W{1'b0}};
            end else if (control_cnt >= CONTROL_DIV - 1) begin
                control_cnt <= {CONTROL_W{1'b0}};
                sample_tick <= 1'b1;
            end else begin
                control_cnt <= control_cnt + 1'b1;
            end
        end
    end

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            position_cnt <= {POSITION_W{1'b0}};
            position_tick <= 1'b0;
        end else begin
            position_tick <= 1'b0;

            if (position_cnt >= POSITION_DIV - 1) begin
                position_cnt <= {POSITION_W{1'b0}};
                position_tick <= 1'b1;
            end else begin
                position_cnt <= position_cnt + 1'b1;
            end
        end
    end

    encoder #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .SAMPLE_HZ   (ENCODER_SAMPLE_HZ),
        .COUNT_WIDTH (32)
    ) u_position_encoder (
        .sys_clk  (sys_clk),
        .rst_n    (rst_n),
        .enc_a    (enc_a),
        .enc_b    (enc_b),
        .position (enc_position),
        .speed    (enc_speed)
    );

    oled #(.CLK_FREQ_HZ(CLK_FREQ_HZ), .I2C_FREQ_HZ(300_000), .REFRESH_MS(50)) u_position_oled (
        .sys_clk(sys_clk), .rst_n(rst_n),
        .display_mode(balance_confirmed ? 3'd4 : 3'd3),
        .text_data(balance_confirmed ? 128'h7377696E675F73756363657373212020 :
                                       128'h7377696E672E2E2E2E20202020202020),
        .text_data2(balance_confirmed ? 128'h706F736974696F6E3A20202020202020 : 128'd0),
        .number_data(position_feedback), .oled_scl(oled_scl), .oled_sda(oled_sda));

    position_half_turn #(.POSITION_HZ(POSITION_HZ), .COUNTS_PER_TURN(KNOB_TRAVEL_COUNTS)) u_key2_motion (
        .sys_clk(sys_clk), .rst_n(rst_n), .enable(position_rst_n && control_enable && !serial_target_active),
        .position_tick(position_tick), .step_request(key2_down), .in_position(key2_in_position),
        .position_offset(key2_position_offset), .requested_offset(key2_requested_offset),
        .velocity_q8(key2_velocity_q8), .profile_active(key2_profile_active),
        .accepted_steps(key2_accepted_steps), .busy(key2_busy));

    knob_position #(
        .BASE_TARGET(POSITION_TARGET), .ADC_FULL_SCALE(KNOB_ADC_FULL_SCALE),
        .TRAVEL_COUNTS(KNOB_TRAVEL_COUNTS), .TARGET_STEP(KNOB_TARGET_STEP)
    ) u_knob_target (
        .sys_clk(sys_clk), .rst_n(rst_n), .enable(balance_confirmed && control_enable && !serial_target_active),
        .position_tick(position_tick), .adc_value(knob_adc_value), .adc_valid(knob_adc_valid),
        .motion_active(key2_profile_active),
        .position_offset(key2_position_offset), .requested_offset(key2_requested_offset),
        .target_position(knob_target_position), .requested_position(knob_requested_position)
    );

    // KEY2 提供规划的半圈速度；A0 步进仍只由位置误差决定。
    pid_position #(
        .KP              (POS_KP),
        .KD              (POS_KD),
        .GAIN_SHIFT      (POS_GAIN_SHIFT),
        .OUT_MAX         (ANGLE_OFFSET_MAX)
    ) u_position_loop (
        .sys_clk      (sys_clk),
        .rst_n        (rst_n),
        .sample_tick  (position_tick),
        .enable       (position_rst_n && control_enable),
        .position     (position_feedback),
        .target_position(position_target),
        .target_velocity_q8(position_velocity_q8),
        .angle_offset (angle_offset),
        .error        (),
        .output_valid (position_result_valid),
        .sampled_position(sampled_position),
        .sampled_delta(sampled_position_delta)
    );

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            applied_angle_offset <= 16'sd0;
            applied_position <= 0;
            applied_position_delta <= 0;
            position_offset_limit <= 16'sd0;
            angle_target <= BASE_ANGLE_TARGET;
        end else if (!position_rst_n || !control_enable) begin
            applied_angle_offset <= 16'sd0;
            applied_position <= 0;
            applied_position_delta <= 0;
            position_offset_limit <= 16'sd0;
            angle_target <= BASE_ANGLE_TARGET;
        end else begin
            // 逐步释放控制权限，但不延迟 PD 制动或方向反转。
            if (position_tick)
                position_offset_limit <= next_position_offset_limit;
            // 立即提交已完成的结果，不再等待下一个 10 ms 周期。
            if (position_result_valid) begin
                applied_position <= sampled_position;
                applied_position_delta <= sampled_position_delta;
                if (angle_offset > position_offset_limit)
                    applied_angle_offset <= position_offset_limit;
                else if (angle_offset < -position_offset_limit)
                    applied_angle_offset <= -position_offset_limit;
                else
                    applied_angle_offset <= angle_offset;
            end
            angle_target <= BASE_ANGLE_TARGET + applied_angle_offset;
        end
    end

    // 使用校验后的 ADC 时间戳，消除 ADC 到控制周期之间变化的采样间隔。
    angle_velocity_timed #(.CLK_FREQ_HZ(CLK_FREQ_HZ), .CONTROL_HZ(CONTROL_HZ),
        .STEP_MAX(ADC_STEP_MAX), .AVERAGE_INTERVALS(USE_TIMED_VELOCITY ? 2 : 1)) u_velocity (
        .sys_clk(sys_clk), .rst_n(rst_n), .angle_valid(adc_ready),
        .measurement_tick(adc_valid_d && adc_ready), .angle_value(angle_value),
        .velocity(measured_velocity));

    // 角度内环：将测得的角度转换为电机 PWM 指令。
    pid_angle #(
        .STOP_MIN     (PID_ANGLE_MIN),
        .STOP_MAX     (PID_ANGLE_MAX),
        .KP           (16'sd30),
        .KI           (16'sd1),
        .KD           (16'sd250),
        .CAPTURE_KD   (CAPTURE_KD),
        .CAPTURE_KP   (CAPTURE_KP),
        .CAPTURE_BRAKE_KD(CAPTURE_BRAKE_KD),
        .CAPTURE_BRAKE_STEP_MAX(CAPTURE_BRAKE_STEP_MAX),
        .CAPTURE_BRAKE_OUT_MAX(CAPTURE_BRAKE_OUT_MAX),
        .CAPTURE_BRAKE_ERROR_MAX(CAPTURE_BRAKE_ERROR_MAX),
        .CAPTURE_BRAKE_FILTER_SHIFT(CAPTURE_BRAKE_FILTER_SHIFT),
        .CAPTURE_TICKS(CAPTURE_TICKS),
        .CAPTURE_QUIET_ERROR_MAX(CONFIRM_ERROR_MAX), // 调整安静状态的释放条件，不改变制动斜坡。
        .D_FILTER_SHIFT(D_FILTER_SHIFT),
        .USE_TIMED_VELOCITY(USE_TIMED_VELOCITY),
        .USE_TIMED_BALANCE(!CAPTURE_DIAGNOSTIC),
        .GAIN_SHIFT   (3),
        .OUT_MAX      (MAX_PWM),
        .INTEGRAL_ERROR_MAX (500), // 当前目标附近的 ADC 计数范围；需实机校准。
        .HOLD_INTEGRAL_OUTSIDE_ZONE (1'b1),
        .TRACK_OUTSIDE_RANGE (1'b1)
    ) u_angle_loop (
        .tracking_only(pid_tracking_snapshot),
        .steady_state(pid_steady_snapshot),
        .sys_clk     (sys_clk),
        .rst_n       (pid_rst_n),
        .sample_tick (pid_compute_tick),
        .measurement_fresh(pid_fresh_snapshot),
        .measured_velocity(pid_velocity_snapshot),
        .target_value(pid_target_snapshot),
        .angle_value (pid_angle_snapshot),
        .pid_out     (pid_out),
        .pid_enable  (control_enable),
        .error       (angle_loop_error),
        .integral_term(angle_integral_term),
        .capture_active(capture_active),
        .diagnostic_step(diagnostic_step),
        .diagnostic_d(diagnostic_d),
        .diagnostic_brake(diagnostic_brake)
    );

    pwm #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .PWM_FREQ_HZ (20_000)
    ) u_motor_pwm (
        .sys_clk    (sys_clk),
        .rst_n      (rst_n),
        .duty_value (motor_command),
        .motor_dir  (motor_dir),
        .pwm_out    (pwm_out)
    );

    // 每次复位只捕获一次；缓冲不得阻塞或改变 PID 计算。
    always @(posedge sys_clk)
        diagnostic_record <= diagnostic_memory[diagnostic_read[8:0]];

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            diagnostic_tick_d <= 0;
            diagnostic_capture_d <= 0;
            diagnostic_fresh_d <= 0;
            diagnostic_angle_d <= 0;
            diagnostic_started <= 0;
            diagnostic_done <= 0;
            diagnostic_count <= 0;
        end else if (CAPTURE_DIAGNOSTIC) begin
            // 停机或故障帧也要补齐，使失败的捕获同样能回放全部 512 行。
            diagnostic_tick_d <= pid_result_tick;
            diagnostic_capture_d <= pid_capture_snapshot;
            diagnostic_fresh_d <= pid_fresh_snapshot;
            diagnostic_angle_d <= pid_angle_snapshot;
            if (diagnostic_tick_d && !diagnostic_done &&
                (diagnostic_started || diagnostic_capture_d)) begin
                diagnostic_started <= 1;
                diagnostic_memory[diagnostic_count[8:0]] <= {
                    diagnostic_angle_d,
                    diagnostic_step,
                    {motor_speed[14], motor_speed},
                    diagnostic_d,
                    (balance_enable && control_enable ?
                     (diagnostic_fresh_d ? (balance_confirmed ? 3'd2 : 3'd1) :
                                            (balance_confirmed ? 3'd4 : 3'd3)) : 3'd0),
                    diagnostic_brake};
                diagnostic_count <= diagnostic_count + 1'b1;
                if (diagnostic_count == 511)
                    diagnostic_done <= 1;
            end
        end
    end

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            uart_sample_cnt <= 32'd0;
            uart_sample_req <= 1'b0;
            uart_target_value <= 32'sd0;
            uart_act_value <= 32'sd0;
            uart_position_value <= 32'sd0;
            uart_out_value <= 16'sd0;
            uart_integral_value <= 16'sd0;
            uart_control_stage <= 4'd0;
            uart_rotor_speed <= 16'sd0;
            uart_position_target <= 0;
            diagnostic_read <= 0;
        end else begin
            uart_sample_req <= 1'b0;

            if (uart_sample_cnt >= UART_SAMPLE_DIV - 1) begin
                uart_sample_cnt <= 32'd0;
                uart_sample_req <= !CAPTURE_DIAGNOSTIC ||
                                   (diagnostic_done && diagnostic_read < 512);
                // 普通 CSV：位置目标、编码器相对实际位置。
                uart_position_value <= position_feedback;
                uart_target_value <= balance_enable && control_enable ? pid_target_value : angle_target;
                uart_act_value <= {16'd0, balance_enable && control_enable ? pid_angle_value : angle_value};
                uart_out_value <= motor_speed;
                uart_integral_value <= balance_enable && control_enable ?
                                       angle_integral_term[15:0] : 16'sd0;
                uart_control_stage <= balance_enable && control_enable ?
                                      (balance_confirmed ? 4'd9 : 4'd8) : {1'b0, swing_state};
                uart_rotor_speed <= applied_position_delta > 33'sd2147483647 ? 32'sh7fffffff :
                                    applied_position_delta < -33'sd2147483648 ? 32'sh80000000 :
                                    applied_position_delta[31:0];
                uart_position_target <= position_target;
                if (CAPTURE_DIAGNOSTIC && diagnostic_done && diagnostic_read < 512) begin
                    // 回放字段：角度、角度增量、PWM、基础 D 项、采样有效性/阶段、额外制动。
                    uart_target_value <= {16'd0, diagnostic_record[82:67]};
                    uart_act_value <= {{16{diagnostic_record[66]}}, diagnostic_record[66:51]};
                    uart_out_value <= diagnostic_record[50:35];
                    uart_integral_value <= diagnostic_record[34:19];
                    uart_control_stage <= diagnostic_record[18:16];
                    uart_rotor_speed <= {{16{diagnostic_record[15]}}, diagnostic_record[15:0]};
                    diagnostic_read <= diagnostic_read + 1'b1;
                end
            end else begin
                uart_sample_cnt <= uart_sample_cnt + 1'b1;
            end
        end
    end

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            sending <= 1'b0;
            line_index <= 8'd0;
            tx_start <= 1'b0;
            tx_data <= 8'h00;
            decimal_left <= 0;
            uart_negative <= 0;
            for (transmit_field=0; transmit_field<UART_FIELD_COUNT; transmit_field=transmit_field+1) begin
                uart_binary[transmit_field] <= 0;
                uart_decimal[transmit_field] <= 0;
            end
        end else begin
            tx_start <= 1'b0;

            if (!sending && decimal_left == 0 && uart_sample_req) begin
                decimal_left <= 32;
                line_index <= 8'd0;
                for (transmit_field=0; transmit_field<UART_FIELD_COUNT; transmit_field=transmit_field+1) begin
                    uart_negative[transmit_field] <= uart_fields[transmit_field][31];
                    uart_binary[transmit_field] <= uart_fields[transmit_field][31] ?
                        -uart_fields[transmit_field] : uart_fields[transmit_field];
                    uart_decimal[transmit_field] <= 0;
                end
            end else if (decimal_left != 0) begin
                decimal_left <= decimal_left - 1'b1;
                for (transmit_field=0; transmit_field<UART_FIELD_COUNT; transmit_field=transmit_field+1) begin
                    uart_decimal[transmit_field] <= {decimal_adjusted[transmit_field][38:0], uart_binary[transmit_field][31]};
                    uart_binary[transmit_field] <= {uart_binary[transmit_field][30:0], 1'b0};
                end
                if (decimal_left == 1) sending <= 1'b1;
            end else if (sending && !tx_busy && !tx_start) begin
                tx_data <= line_char(line_index);
                tx_start <= 1'b1;

                if (line_index == LINE_LEN - 1) begin
                    sending <= 1'b0;
                    line_index <= 8'd0;
                end else begin
                    line_index <= line_index + 1'b1;
                end
            end
        end
    end

    // 方向引脚使用寄存后的零值判断；发生故障时 PWMB 仍会立即停止。
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) motor_stopped <= 1'b1;
        else motor_stopped <= motor_speed == 15'sd0;
    end

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            motor_dir_d <= 1'b0;
            dir_deadtime <= 1'b1;
            BIN1 <= 1'b0;
            BIN2 <= 1'b0;
        end else if (motor_stopped) begin
            dir_deadtime <= 1'b1;
            BIN1 <= 1'b0;
            BIN2 <= 1'b0;
        end else if (motor_dir != motor_dir_d) begin
            motor_dir_d <= motor_dir;
            dir_deadtime <= 1'b1;
            BIN1 <= 1'b0;
            BIN2 <= 1'b0;
        end else if (dir_deadtime) begin
            dir_deadtime <= 1'b0;
            BIN1 <= motor_dir;
            BIN2 <= ~motor_dir;
        end else begin
            BIN1 <= motor_dir;
            BIN2 <= ~motor_dir;
        end
    end

endmodule


