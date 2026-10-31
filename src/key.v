// 对按键输入消抖，并在按下时输出单周期脉冲。
module key #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer DEBOUNCE_MS = 20,
    parameter         ACTIVE_LEVEL = 1'b0
) (
    input  wire sys_clk,
    input  wire rst_n,
    input  wire key_in,
    output reg  key_state,
    output reg  key_down
);

    localparam integer DEBOUNCE_CYCLES = CLK_FREQ_HZ / 1000 * DEBOUNCE_MS;
    localparam integer CNT_WIDTH = (DEBOUNCE_CYCLES <= 1) ? 1 : $clog2(DEBOUNCE_CYCLES);

    reg key_sync1;
    reg key_sync2;
    reg key_sample;
    reg [CNT_WIDTH-1:0] debounce_cnt;

    wire key_active = (key_sync2 == ACTIVE_LEVEL);

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            key_sync1   <= ~ACTIVE_LEVEL;
            key_sync2   <= ~ACTIVE_LEVEL;
            key_sample  <= 1'b0;
            key_state   <= 1'b0;
            key_down    <= 1'b0;
            debounce_cnt <= {CNT_WIDTH{1'b0}};
        end else begin
            key_sync1 <= key_in;
            key_sync2 <= key_sync1;
            key_down  <= 1'b0;

            if (key_active != key_sample) begin
                key_sample   <= key_active;
                debounce_cnt <= {CNT_WIDTH{1'b0}};
            end else if (debounce_cnt < DEBOUNCE_CYCLES - 1) begin
                debounce_cnt <= debounce_cnt + 1'b1;
            end else if (key_state != key_sample) begin
                key_state <= key_sample;
                if (key_sample)
                    key_down <= 1'b1;
            end
        end
    end

endmodule
