// 根据 ADC 角度采样估算摆杆角速度。
// 将 ADC 位移归一到控制周期；硬件顶层使用一个采样间隔以限制延迟。
module angle_velocity_timed #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer CONTROL_HZ = 500,
    parameter [15:0] STEP_MAX = 16'd2000,
    parameter integer AVERAGE_INTERVALS = 2 
) (
    input wire sys_clk, rst_n, angle_valid, measurement_tick,
    input wire [15:0] angle_value,
    output wire signed [15:0] velocity
);
    localparam integer PERIOD = CLK_FREQ_HZ / CONTROL_HZ;
    localparam integer MAX_AGE = CLK_FREQ_HZ / 125; // 超过 8 ms 未收到数据时，速度历史失效。
    localparam integer AGE_W = $clog2(MAX_AGE + 1);
    reg [AGE_W-1:0] age, previous_interval;
    reg [1:0] history;
    reg [15:0] previous_angle, older_angle;
    reg signed [15:0] held_velocity;
    reg [47:0] dividend, quotient;
    reg [AGE_W:0] divisor;
    reg [AGE_W+1:0] remainder;
    reg [5:0] divide_left;
    reg divide_negative;
    wire signed [17:0] step = $signed({2'b0, angle_value}) - $signed({2'b0, previous_angle});
    wire discontinuity = step > $signed({2'b0, STEP_MAX}) || step < -$signed({2'b0, STEP_MAX});
    wire usable = angle_valid && history != 0 && age < MAX_AGE - 1 && !discontinuity;
    reg signed [17:0] span;
    reg [AGE_W:0] elapsed;
    reg signed [47:0] numerator;
    wire [AGE_W+1:0] shifted_remainder = (remainder << 1) | dividend[47];
    wire subtract_divisor = shifted_remainder >= {1'b0, divisor};
    wire [47:0] next_quotient = (quotient << 1) | subtract_divisor;
   always @* begin
        span = AVERAGE_INTERVALS > 1 && history >= 2 ?
               $signed({2'b0, angle_value}) - $signed({2'b0, older_angle}) : step;
        elapsed = {1'b0, age} + 1'b1;
        if (AVERAGE_INTERVALS > 1 && history >= 2) elapsed = elapsed + {1'b0, previous_interval};
        numerator = span * PERIOD;
        // 读取时刻近似转换时刻；若 ADC 相位抖动仍存在，应改用转换就绪时刻。
    end
   assign velocity = !angle_valid || age >= MAX_AGE - 1 ? 16'sd0 :
                      held_velocity;
   always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            age <= 0; previous_interval <= 0; history <= 0;
            previous_angle <= 0; older_angle <= 0; held_velocity <= 0;
            dividend <= 0; quotient <= 0; divisor <= 1; remainder <= 0;
            divide_left <= 0; divide_negative <= 0;
        end else if (!angle_valid) begin
            age <= 0; previous_interval <= 0; history <= 0;
            held_velocity <= 0;
            divide_left <= 0;
        end else if (measurement_tick) begin
            age <= 0;
            previous_angle <= angle_value;
            older_angle <= previous_angle;
            previous_interval <= age + 1'b1;
            history <= usable ? 2 : 1;
            if (usable) begin
                dividend <= numerator < 0 ? -numerator : numerator;
                divisor <= elapsed;
                divide_negative <= numerator < 0;
                quotient <= 0; remainder <= 0; divide_left <= 48;
            end else begin
                held_velocity <= 0; divide_left <= 0;
            end
        end else begin
            if (age < MAX_AGE) age <= age + 1'b1;
            if (divide_left != 0) begin
                dividend <= dividend << 1;
                quotient <= next_quotient;
                remainder <= subtract_divisor ? shifted_remainder - divisor : shifted_remainder;
                divide_left <= divide_left - 1'b1;
                if (divide_left == 1)
                    held_velocity <= divide_negative ?
                        (next_quotient > 32767 ? -16'sd32767 : -$signed(next_quotient[15:0])) :
                        (next_quotient > 32767 ? 16'sd32767 : next_quotient[15:0]);
            end
        end
    end
endmodule
