// 为半圈移动生成位置轨迹和速度规划。
module position_half_turn #(
    parameter integer POSITION_HZ = 100,
    parameter signed [31:0] COUNTS_PER_TURN = 32'sd1533
) (
    input wire sys_clk, rst_n, enable, position_tick, step_request, in_position,
    output reg signed [31:0] position_offset,
    output reg signed [31:0] requested_offset,
    output wire signed [31:0] velocity_q8,
    output reg profile_active,
    output reg [15:0] accepted_steps,
    output reg busy
);
    localparam integer ACCEL_TICKS = POSITION_HZ * 2 / 5 > 0 ? POSITION_HZ * 2 / 5 : 1;
    localparam integer DECEL_TICKS = POSITION_HZ * 8 / 5 > 0 ? POSITION_HZ * 8 / 5 : 1;
    localparam integer CRUISE_TICKS = POSITION_HZ * 2;
    localparam integer TOTAL_TICKS = ACCEL_TICKS + CRUISE_TICKS + DECEL_TICKS;
    localparam integer SETTLE_TICKS = POSITION_HZ / 5 > 0 ? POSITION_HZ / 5 : 1;
    localparam integer SETTLE_W = $clog2(SETTLE_TICKS + 1);
    localparam integer TICK_W = $clog2(TOTAL_TICKS + 1);
    // 行程和持续时间同时加倍，可保持之前四分之一圈动作的角速度。
    localparam signed [51:0] HALF_TURN_Q20 = COUNTS_PER_TURN * 52'sd524288;
    localparam signed [31:0] PEAK_Q20 = HALF_TURN_Q20 * 2 / (ACCEL_TICKS + DECEL_TICKS + 2 * CRUISE_TICKS);
    localparam signed [31:0] ACCEL_Q20 = PEAK_Q20 / ACCEL_TICKS;
    localparam signed [31:0] DECEL_Q20 = PEAK_Q20 / DECEL_TICKS;
    localparam signed [51:0] MAX_POSITION_Q20 = 52'sd2147483647 <<< 20;
    reg [TICK_W-1:0] move_tick;
    reg [SETTLE_W-1:0] settle_count;
    reg signed [51:0] position_q20, goal_q20, segment_goal_q20;
    reg signed [31:0] velocity_q20, velocity_next;
    wire accept_step = step_request && goal_q20 <= MAX_POSITION_Q20 - HALF_TURN_Q20;
    // 将规划的分数速度输入 D 项；对舍入后的目标值作差会产生突变。
    assign velocity_q8 = velocity_q20 >>> 12;
    wire signed [51:0] rounded_position = position_q20 + 52'sd524288;
    wire signed [51:0] rounded_goal = goal_q20 + 52'sd524288;
    // 将 Q20 舍入与后续电位器求和、位置误差相减分开。
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            position_offset <= 0; requested_offset <= 0;
        end else if (!enable) begin
            position_offset <= 0; requested_offset <= 0;
        end else begin
            position_offset <= rounded_position >>> 20;
            requested_offset <= rounded_goal >>> 20;
        end
    end

    always @* begin
        velocity_next = velocity_q20;
        if (move_tick < ACCEL_TICKS)
            velocity_next = velocity_q20 + ACCEL_Q20;
        else if (move_tick >= ACCEL_TICKS + CRUISE_TICKS)
            velocity_next = velocity_q20 > DECEL_Q20 ? velocity_q20 - DECEL_Q20 : 32'sd0;
    end

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            position_q20 <= 0; goal_q20 <= 0; segment_goal_q20 <= 0;
            velocity_q20 <= 0; move_tick <= 0; busy <= 0;
            profile_active <= 0; settle_count <= 0; accepted_steps <= 0;
        end else if (!enable) begin
            position_q20 <= 0; goal_q20 <= 0; segment_goal_q20 <= 0;
            velocity_q20 <= 0; move_tick <= 0; busy <= 0;
            profile_active <= 0; settle_count <= 0; accepted_steps <= 0;
        end else begin
            if (accept_step) begin
                // 保留每个消抖后的请求；当前动作段继续使用自己的终点。
                goal_q20 <= goal_q20 + HALF_TURN_Q20;
                busy <= 1;
                settle_count <= 0;
                if (accepted_steps != 16'hffff) accepted_steps <= accepted_steps + 1'b1;
            end
            if (!profile_active && (goal_q20 > position_q20 || accept_step)) begin
                segment_goal_q20 <= position_q20 + HALF_TURN_Q20;
                velocity_q20 <= 0;
                move_tick <= 0;
                busy <= 1;
                profile_active <= 1;
                settle_count <= 0;
            end
            if (position_tick && profile_active) begin
                if (move_tick == TOTAL_TICKS - 1) begin
                    position_q20 <= segment_goal_q20;
                    velocity_q20 <= 0;
                    profile_active <= 0;
                end else begin
                    position_q20 <= position_q20 + velocity_next;
                    velocity_q20 <= velocity_next;
                    move_tick <= move_tick + 1'b1;
                end
            end
            if (position_tick && busy && !profile_active && goal_q20 == position_q20 && !accept_step) begin
                if (!in_position) settle_count <= 0;
                else if (settle_count >= SETTLE_TICKS - 1) begin
                    busy <= 0;
                    settle_count <= 0;
                end else settle_count <= settle_count + 1'b1;
            end
        end
    end
endmodule
