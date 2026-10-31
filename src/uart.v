// 实现 UART 串口收发。
module uart #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer BAUD_RATE   = 9_600
) (
    input  wire       sys_clk,
    input  wire       rst_n,
    input  wire       uart_rx,
    output reg        uart_tx,
    output reg [7:0]  rx_data,
    output reg        rx_valid,
    input  wire [7:0] tx_data,
    input  wire        tx_start,
    output reg        tx_busy,
    output reg        tx_done
);

    localparam integer BAUD_DIV = CLK_FREQ_HZ / BAUD_RATE;
    localparam integer BAUD_W = (BAUD_DIV < 2) ? 1 : $clog2(BAUD_DIV);
    localparam integer HALF_BAUD_DIV = BAUD_DIV / 2;
    localparam integer HALF_BAUD_W = (HALF_BAUD_DIV < 2) ? 1 : $clog2(HALF_BAUD_DIV);

    reg rx_meta;
    reg rx_sync;
    reg [1:0] rx_state;
    reg [BAUD_W-1:0] rx_cnt;
    reg [3:0] rx_bit_cnt;
    reg [7:0] rx_shift;

    localparam [1:0] RX_IDLE=0, RX_START=1, RX_DATA=2, RX_STOP=3;

    reg [1:0] tx_state;
    reg [BAUD_W-1:0] tx_cnt;
    reg [3:0] tx_bit_cnt;
    reg [7:0] tx_shift;

    localparam [1:0] TX_IDLE=0, TX_START=1, TX_DATA=2, TX_STOP=3;

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_meta <= 1'b1;
            rx_sync <= 1'b1;
            rx_state <= RX_IDLE;
            rx_cnt <= 0;
            rx_bit_cnt <= 0;
            rx_shift <= 0;
            rx_data <= 0;
            rx_valid <= 1'b0;
        end else begin
            rx_meta <= uart_rx;
            rx_sync <= rx_meta;
            rx_valid <= 1'b0;

            case (rx_state)
                RX_IDLE: begin
                    rx_cnt <= 0;
                    if (!rx_sync) begin
                        rx_state <= RX_START;
                        rx_cnt <= 0;
                    end
                end

                RX_START: begin
                    if (rx_cnt >= HALF_BAUD_DIV - 1) begin
                        rx_cnt <= 0;
                        if (!rx_sync) begin
                            rx_bit_cnt <= 0;
                            rx_state <= RX_DATA;
                        end else begin
                            rx_state <= RX_IDLE;
                        end
                    end else begin
                        rx_cnt <= rx_cnt + 1'b1;
                    end
                end

                RX_DATA: begin
                    if (rx_cnt >= BAUD_DIV - 1) begin
                        rx_cnt <= 0;
                        rx_shift[rx_bit_cnt] <= rx_sync;
                        if (rx_bit_cnt == 7)
                            rx_state <= RX_STOP;
                        else
                            rx_bit_cnt <= rx_bit_cnt + 1'b1;
                    end else begin
                        rx_cnt <= rx_cnt + 1'b1;
                    end
                end

                RX_STOP: begin
                    if (rx_cnt >= BAUD_DIV - 1) begin
                        rx_cnt <= 0;
                        rx_state <= RX_IDLE;
                        if (rx_sync) begin
                            rx_data <= rx_shift;
                            rx_valid <= 1'b1;
                        end
                    end else begin
                        rx_cnt <= rx_cnt + 1'b1;
                    end
                end

                default: rx_state <= RX_IDLE;
            endcase
        end
    end

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_state <= TX_IDLE;
            tx_cnt <= 0;
            tx_bit_cnt <= 0;
            tx_shift <= 0;
            uart_tx <= 1'b1;
            tx_busy <= 1'b0;
            tx_done <= 1'b0;
        end else begin
            tx_done <= 1'b0;

            case (tx_state)
                TX_IDLE: begin
                    uart_tx <= 1'b1;
                    tx_cnt <= 0;
                    tx_bit_cnt <= 0;
                    tx_busy <= 1'b0;
                    if (tx_start) begin
                        tx_shift <= tx_data;
                        tx_busy <= 1'b1;
                        tx_state <= TX_START;
                        uart_tx <= 1'b0;
                    end
                end

                TX_START: begin
                    if (tx_cnt >= BAUD_DIV - 1) begin
                        tx_cnt <= 0;
                        tx_bit_cnt <= 0;
                        uart_tx <= tx_shift[0];
                        tx_state <= TX_DATA;
                    end else begin
                        tx_cnt <= tx_cnt + 1'b1;
                    end
                end

                TX_DATA: begin
                    if (tx_cnt >= BAUD_DIV - 1) begin
                        tx_cnt <= 0;
                        if (tx_bit_cnt == 7) begin
                            uart_tx <= 1'b1;
                            tx_state <= TX_STOP;
                        end else begin
                            tx_bit_cnt <= tx_bit_cnt + 1'b1;
                            uart_tx <= tx_shift[tx_bit_cnt + 1'b1];
                        end
                    end else begin
                        tx_cnt <= tx_cnt + 1'b1;
                    end
                end

                TX_STOP: begin
                    if (tx_cnt >= BAUD_DIV - 1) begin
                        tx_cnt <= 0;
                        uart_tx <= 1'b1;
                        tx_busy <= 1'b0;
                        tx_done <= 1'b1;
                        tx_state <= TX_IDLE;
                    end else begin
                        tx_cnt <= tx_cnt + 1'b1;
                    end
                end

                default: tx_state <= TX_IDLE;
            endcase
        end
    end

endmodule
