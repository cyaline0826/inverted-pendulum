// 根据编码器位置误差计算角度目标偏移。
module pid_position #(
    parameter signed [15:0] KP = 16'sd2,
    parameter signed [15:0] KD = 16'sd0,
    parameter integer GAIN_SHIFT = 4,
    parameter signed [15:0] OUT_MAX = 16'sd1200
) (
    input  wire sys_clk,
    input  wire rst_n,
    input  wire sample_tick,
    input  wire enable,
    input  wire signed [31:0] position,
    input  wire signed [31:0] target_position,
    input  wire signed [31:0] target_velocity_q8,
    output reg signed [15:0] angle_offset,
    output reg signed [31:0] error,
    output reg output_valid,
    output reg signed [31:0] sampled_position,
    output reg signed [32:0] sampled_delta
);

    reg history_valid, sample_history_valid;
    reg signed [31:0] last_position;
    reg signed [32:0] error_sample, delta_sample;
    reg signed [40:0] velocity_sample;
    reg signed [31:0] target_velocity_sample;
    reg signed [41:0] derivative_sample;
    reg signed [48:0] proportional;
    reg signed [57:0] differential;
    reg signed [58:0] scaled;
    reg signed [15:0] bounded;
    reg [5:0] pending;
    reg slow_history_valid, slow_direction;
    reg [4:0] idle_ticks, slow_period;
    reg signed [40:0] velocity_next;
    wire signed [32:0] error_next = $signed({target_position[31], target_position}) -
                                   $signed({position[31], position});
    wire signed [32:0] delta_next = history_valid ?
        $signed({position[31], position}) - $signed({last_position[31], last_position}) : 33'sd0;
    wire [4:0] elapsed_ticks = idle_ticks < 5'd16 ? idle_ticks + 5'd1 : 5'd16;
    wire [4:0] held_period = slow_period > elapsed_ticks ? slow_period : elapsed_ticks;
    wire signed [58:0] pid_sum = {{2{proportional[48]}}, proportional, 8'd0} +
                                {differential[57], differential};

    // 使用 Q8 计数/周期表示速度，避免在 50 MHz 采样路径中使用可变除法器。
    function [8:0] count_per_tick;
        input [4:0] period_ticks;
        begin
            case(period_ticks)
                1: count_per_tick=9'd256;
                2: count_per_tick=9'd128;
                3: count_per_tick=9'd85;
                4: count_per_tick=9'd64;
                5: count_per_tick=9'd51;
                6: count_per_tick=9'd43;
                7: count_per_tick=9'd37;
                8: count_per_tick=9'd32;
                9: count_per_tick=9'd28;
                10: count_per_tick=9'd26;
                11: count_per_tick=9'd23;
                12: count_per_tick=9'd21;
                13: count_per_tick=9'd20;
                14: count_per_tick=9'd18;
                15: count_per_tick=9'd17;
                16: count_per_tick=9'd16;
                default: count_per_tick=9'd0;
            endcase
        end
    endfunction

    always @* begin
        velocity_next = $signed({delta_sample, 8'd0});
        if ((delta_sample == 33'sd1 || delta_sample == -33'sd1) &&
            slow_history_valid && slow_direction == delta_sample[32])
            velocity_next = slow_direction ? -$signed({32'd0, count_per_tick(elapsed_ticks)}) :
                                             $signed({32'd0, count_per_tick(elapsed_ticks)});
        else if (delta_sample == 0 && slow_history_valid && slow_period > 1 && elapsed_ticks < 16)
            velocity_next = slow_direction ? -$signed({32'd0, count_per_tick(held_period)}) :
                                             $signed({32'd0, count_per_tick(held_period)});
    end

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            angle_offset <= 16'sd0;
            error        <= 32'sd0;
            output_valid <= 0;
            sampled_position <= 0;
            sampled_delta <= 0;
            history_valid <= 0;
            sample_history_valid <= 0;
            last_position <= 0;
            error_sample <= 0;
            delta_sample <= 0;
            derivative_sample <= 0;
            velocity_sample <= 0;
            target_velocity_sample <= 0;
            slow_history_valid <= 0;
            slow_direction <= 0;
            idle_ticks <= 0;
            slow_period <= 1;
            proportional <= 0;
            differential <= 0;
            scaled <= 0;
            bounded <= 0;
            pending <= 0;
        end else if (!enable) begin
            angle_offset <= 0;
            error <= 0;
            output_valid <= 0;
            sampled_position <= 0;
            sampled_delta <= 0;
            history_valid <= 0;
            sample_history_valid <= 0;
            delta_sample <= 0;
            target_velocity_sample <= 0;
            slow_history_valid <= 0;
            slow_direction <= 0;
            idle_ticks <= 0;
            slow_period <= 1;
            pending <= 0;
        end else begin
    // 明确划分流水级，使非零增益运算保持单周期路径；
            pending <= {pending[4:0], sample_tick};
            output_valid <= 0;
            if (sample_tick) begin
                error_sample <= error_next;
                delta_sample <= delta_next;
                target_velocity_sample <= target_velocity_q8;
                sample_history_valid <= history_valid;
                last_position <= position;
                history_valid <= 1;
                error <= error_next > 33'sd2147483647 ? 32'sh7fffffff :
                         error_next < -33'sd2147483648 ? 32'sh80000000 : error_next[31:0];
            end
    // 将位置相减 -> 速度选择 -> 取负拆分到不同的时钟路径。
            if (pending[0]) begin
                velocity_sample <= velocity_next;
                // 间歇的单计数变化使用周期速度；反转和快速运动使用原始 D 项。
                if (!sample_history_valid) begin
                    slow_history_valid <= 0;
                    idle_ticks <= 0;
                    slow_period <= 1;
                end else if (delta_sample == 33'sd1 || delta_sample == -33'sd1) begin
                    slow_period <= slow_history_valid && slow_direction == delta_sample[32] ? elapsed_ticks : 5'd1;
                    slow_direction <= delta_sample[32];
                    slow_history_valid <= 1;
                    idle_ticks <= 0;
                end else if (delta_sample == 0) begin
                    idle_ticks <= elapsed_ticks;
                    // 简化限制：停止检测在 16 个外环周期后超时；若缓慢爬行更久，需重新校准。
                    if (elapsed_ticks >= 16) slow_history_valid <= 0;
                end else begin
                    slow_history_valid <= 0;
                    idle_ticks <= 0;
                    slow_period <= 1;
                end
            end
            // 使用 KEY2 规划速度，同时在静止时保留基于实测运动的阻尼。
            if (pending[1]) derivative_sample <=
                $signed({{10{target_velocity_sample[31]}}, target_velocity_sample}) -
                $signed({velocity_sample[40], velocity_sample});
            if (pending[2]) begin
                proportional <= KP * error_sample;
                differential <= KD * derivative_sample;
            end
            if (pending[3]) scaled <= pid_sum >>> (GAIN_SHIFT + 8);
            if (pending[4])
                bounded <= scaled > OUT_MAX ? OUT_MAX :
                           scaled < -OUT_MAX ? -OUT_MAX : scaled[15:0];
            if (pending[5]) begin
                angle_offset <= bounded;
                output_valid <= 1;
                sampled_position <= last_position;
                sampled_delta <= delta_sample;
            end
        end
    end

endmodule
