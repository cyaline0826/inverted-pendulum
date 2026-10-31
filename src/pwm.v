// 将带符号占空比转换为电机方向和 PWM 输出。
module pwm #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer PWM_FREQ_HZ = 20_000
) (
    input  wire        sys_clk,
    input  wire        rst_n,
    input  wire signed [14:0] duty_value,    //设置速度: 5000=正转50%, -5000=反转50%
    output reg         motor_dir,     //1=正转, 0=反转
    output reg         pwm_out
);

    localparam integer PWM_PERIOD = CLK_FREQ_HZ / PWM_FREQ_HZ;

    reg [11:0] period_cnt;
    reg [13:0] duty_latched;
    wire [31:0] duty_count;
    wire [14:0] duty_abs;
    wire [13:0] duty_clamped;

    assign duty_abs     = duty_value[14] ? -duty_value : duty_value;
    assign duty_clamped = (duty_abs > 15'd10000) ? 14'd10000 : duty_abs[13:0];
    // 周期为 2500 个时钟时，换算结果正好是 duty_latched / 4。
    assign duty_count = (PWM_PERIOD == 2500) ? (duty_latched >> 2) :
                        (PWM_PERIOD * duty_latched) / 14'd10000;

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            period_cnt   <= 12'd0;
            duty_latched <= 14'd0;
            motor_dir    <= 1'b1;
            pwm_out      <= 1'b0;
        end else begin
            if (period_cnt >= PWM_PERIOD - 1)
                begin
                    period_cnt   <= 12'd0;
                    duty_latched <= duty_clamped;
                    motor_dir    <= ~duty_value[14];
                end
            else
                period_cnt <= period_cnt + 1'b1;

            pwm_out <= (period_cnt < duty_count);
        end
    end

endmodule
