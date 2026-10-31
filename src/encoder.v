// 对 AB 相编码器进行正交解码，输出位置和速度。
module encoder #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer SAMPLE_HZ   = 1000,
    parameter integer COUNT_WIDTH = 32
) (
    input  wire sys_clk,
    input  wire rst_n,
    input  wire enc_a,
    input  wire enc_b,
    output reg  signed [COUNT_WIDTH-1:0] position,
    output reg  signed [COUNT_WIDTH-1:0] speed
);

    // 采用四倍频正交解码。position 累加 A/B 的每个边沿，speed
    // 表示一个采样周期内的净计数变化。
    localparam integer SAMPLE_DIV = CLK_FREQ_HZ / SAMPLE_HZ;
    localparam integer SAMPLE_W   = (SAMPLE_DIV < 2) ? 1 : $clog2(SAMPLE_DIV);
    localparam signed [COUNT_WIDTH-1:0] COUNT_ONE = 1;

    reg a_sync0;
    reg a_sync1;
    reg b_sync0;
    reg b_sync1;
    reg a_d;
    reg a_prev;
    reg b_d;
    reg b_prev;
    reg [SAMPLE_W-1:0] sample_cnt;
    reg sample_tick;
    reg signed [COUNT_WIDTH-1:0] position_prev;
    reg [4:0] sync_ready;

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            a_sync0 <= 1'b0;
            a_sync1 <= 1'b0;
            b_sync0 <= 1'b0;
            b_sync1 <= 1'b0;
            a_d     <= 1'b0;
            a_prev  <= 1'b0;
            b_d     <= 1'b0;
            b_prev  <= 1'b0;
            sync_ready <= 0;
        end else begin
            a_sync0 <= enc_a;
            a_sync1 <= a_sync0;
            b_sync0 <= enc_b;
            b_sync1 <= b_sync0;
            a_d     <= a_sync1;
            b_d     <= b_sync1;
            a_prev  <= a_d;
            b_prev  <= b_d;
            sync_ready <= {sync_ready[3:0], 1'b1};
        end
    end

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            position <= {COUNT_WIDTH{1'b0}};
        end else if (!sync_ready[4]) begin
            // 开始计数前先记录 A/B 的初始状态。
            position <= {COUNT_WIDTH{1'b0}};
        end else if (a_d != a_prev) begin
            if (a_d == b_d)
                position <= position - COUNT_ONE;
            else
                position <= position + COUNT_ONE;
        end else if (b_d != b_prev) begin
            if (b_d == a_d)
                position <= position + COUNT_ONE;
            else
                position <= position - COUNT_ONE;
        end
    end

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            sample_cnt  <= {SAMPLE_W{1'b0}};
            sample_tick <= 1'b0;
        end else begin
            sample_tick <= 1'b0;
            if (sample_cnt >= SAMPLE_DIV - 1) begin
                sample_cnt  <= {SAMPLE_W{1'b0}};
                sample_tick <= 1'b1;
            end else begin
                sample_cnt <= sample_cnt + 1'b1;
            end
        end
    end

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            position_prev <= {COUNT_WIDTH{1'b0}};
            speed         <= {COUNT_WIDTH{1'b0}};
        end else if (sample_tick) begin
            speed         <= position - position_prev;
            position_prev <= position;
        end
    end

endmodule
