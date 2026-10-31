// 将电位器 ADC 采样转换为位置目标。
module knob_position #(
    parameter signed [31:0] BASE_TARGET = 32'sd0,
    parameter [15:0] ADC_FULL_SCALE = 16'd26400, // ADS1115 量程为 +/-4.096 V 时对应 3.3 V。
    parameter signed [31:0] TRAVEL_COUNTS = 32'sd1533,
    parameter signed [31:0] TARGET_STEP = 32'sd4,
    parameter [15:0] ADC_DEADBAND = 16'd32
) (
    input wire sys_clk, rst_n, enable, position_tick,
    input wire [15:0] adc_value,
    input wire adc_valid,
    input wire motion_active,
    input wire signed [31:0] position_offset, requested_offset,
    output reg signed [31:0] target_position,
    output reg signed [31:0] requested_position
);
    // 使用编译期校准值，避免在 50 MHz 路径中使用可变除法器。
    localparam signed [31:0] SCALE_Q20 =
        (64'sd1048576 * TRAVEL_COUNTS + ADC_FULL_SCALE / 2) / ADC_FULL_SCALE;
    reg reference_valid;
    reg [15:0] reference_adc, accepted_adc;
    reg signed [16:0] sample_delta;
    reg signed [48:0] scaled_delta;
    reg signed [31:0] desired_target, knob_target;
    reg [1:0] pending;
    wire [15:0] bounded_adc = adc_value[15] ? 16'd0 :
        adc_value > ADC_FULL_SCALE ? ADC_FULL_SCALE : adc_value;
    wire signed [16:0] knob_step = $signed({1'b0, bounded_adc}) - $signed({1'b0, accepted_adc});
    wire signed [48:0] rounded_delta = scaled_delta < 0 ?
        -((-scaled_delta + 49'sd524288) >>> 20) : (scaled_delta + 49'sd524288) >>> 20;
    wire signed [32:0] target_error = $signed({desired_target[31], desired_target}) -
                                              $signed({knob_target[31], knob_target});
    // 按 100 Hz、每转 1533 个计数预留每周期 3 个计数；校准值变化时需重新计算。
    wire signed [31:0] slew_step = motion_active ?
        (TARGET_STEP > 32'sd3 ? TARGET_STEP - 32'sd3 : 32'sd0) : TARGET_STEP;

    // 将源数据求和移出位置 PID 的误差相减路径。
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            target_position <= BASE_TARGET; requested_position <= BASE_TARGET;
        end else if (!enable) begin
            target_position <= BASE_TARGET; requested_position <= BASE_TARGET;
        end else begin
            target_position <= knob_target + position_offset;
            requested_position <= desired_target + requested_offset;
        end
    end

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            reference_valid <= 0;
            reference_adc <= 0;
            accepted_adc <= 0;
            sample_delta <= 0;
            scaled_delta <= 0;
            desired_target <= BASE_TARGET;
            knob_target <= BASE_TARGET;
            pending <= 0;
        end else if (!enable) begin
            reference_valid <= 0;
            desired_target <= BASE_TARGET;
            knob_target <= BASE_TARGET;
            pending <= 0;
        end else begin
            pending <= {pending[0], 1'b0};
            if (adc_valid) begin
                if (!reference_valid) begin
                    reference_adc <= bounded_adc;
                    accepted_adc <= bounded_adc;
                    reference_valid <= 1;
                end else if (knob_step >= $signed({1'b0, ADC_DEADBAND}) ||
                             knob_step <= -$signed({1'b0, ADC_DEADBAND})) begin
                    accepted_adc <= bounded_adc;
                    sample_delta <= $signed({1'b0, bounded_adc}) - $signed({1'b0, reference_adc});
                    pending[0] <= 1;
                end
            end
            if (pending[0]) scaled_delta <= sample_delta * SCALE_Q20;
            if (pending[1]) desired_target <= BASE_TARGET + rounded_delta[31:0];
            if (position_tick && reference_valid) begin
                if (target_error > slew_step)
                    knob_target <= knob_target + slew_step;
                else if (target_error < -slew_step)
                    knob_target <= knob_target - slew_step;
                else
                    knob_target <= desired_target;
            end
        end
    end
endmodule
