// 提供 I2C 总线读写时序控制。
module i2c #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer I2C_FREQ_HZ = 300_000
) (
    input  wire        sys_clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire [6:0]  dev_addr,
    input  wire [9:0]  wr_len,
    input  wire [7:0]  wr_data,
    output reg  [9:0]  wr_index,
    input  wire [9:0]  rd_len,
    output reg  [7:0]  rd_data,
    output reg  [9:0]  rd_index,
    output reg         rd_valid,
    output reg         busy,
    output reg         done,
    output reg         ack_error,
    inout  wire        i2c_scl,
    inout  wire        i2c_sda
);

    // 每个 I2C 位占用三个节拍：SCL 拉低、低电平期间设置数据、SCL 拉高。
    // 在进入 S_LOAD 前，wr_data 必须能根据 wr_index 组合生成有效数据。
    localparam integer I2C_DIV = CLK_FREQ_HZ / I2C_FREQ_HZ / 3;
    localparam integer DIV_W = (I2C_DIV < 2) ? 1 : $clog2(I2C_DIV);
    localparam integer TBUF_CYCLES = CLK_FREQ_HZ / 500_000;  // 不少于 2 us。
    localparam integer TBUF_W = (TBUF_CYCLES < 2) ? 1 : $clog2(TBUF_CYCLES);

    localparam [4:0] S_IDLE=0, S_START=1, S_LOAD=2,
                     S_SEND_LOW=3, S_SEND_SETUP=4, S_SEND_HIGH=5,
                     S_ACK_LOW=6, S_ACK_RELEASE=7, S_ACK_HIGH=8, S_ACK_SAMPLE=9,
                     S_READ_LOW=10, S_READ_RELEASE=11, S_READ_HIGH=12, S_READ_SAMPLE=13,
                     S_MACK_LOW=14, S_MACK_SETUP=15, S_MACK_HIGH=16,
                     S_STOP_LOW=17, S_STOP_HIGH=18, S_STOP_RELEASE=19, S_BUS_FREE=20;

    localparam [1:0] P_ADDR_W=0, P_WRITE=1, P_ADDR_R=2, P_READ=3;

    reg [DIV_W-1:0] div_cnt;
    reg [TBUF_W-1:0] tbuf_cnt;
    reg [4:0] state;
    reg [1:0] phase;
    reg [7:0] tx_byte;
    reg [7:0] rx_shift;
    reg [3:0] bit_cnt;
    reg scl_low;
    reg sda_low;
    reg start_meta;
    reg start_sync;
    reg start_last;

    assign i2c_scl = scl_low ? 1'b0 : 1'bz;
    assign i2c_sda = sda_low ? 1'b0 : 1'bz;

    wire scl_in = i2c_scl;
    wire sda_in = i2c_sda;
    wire start_rise = start_sync && !start_last;

    task load_tx_byte;
        begin
            // 仅在此处、每个字节开始时采样 wr_data。
            // 此后整个字节都由 tx_byte 发送。
            if (phase == P_ADDR_W)
                tx_byte <= {dev_addr, 1'b0};
            else if (phase == P_ADDR_R)
                tx_byte <= {dev_addr, 1'b1};
            else
                tx_byte <= wr_data;
            bit_cnt <= 7;
            state <= S_SEND_LOW;
        end
    endtask

    task start_stop;
        begin
            state <= S_STOP_LOW;
        end
    endtask

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            div_cnt <= 0;
            tbuf_cnt <= 0;
            state <= S_IDLE;
            phase <= P_ADDR_W;
            tx_byte <= 0;
            rx_shift <= 0;
            bit_cnt <= 7;
            scl_low <= 0;
            sda_low <= 0;
            start_meta <= 0;
            start_sync <= 0;
            start_last <= 0;
            wr_index <= 0;
            rd_data <= 0;
            rd_index <= 0;
            rd_valid <= 0;
            busy <= 0;
            done <= 0;
            ack_error <= 0;
        end else begin
            start_meta <= start;
            start_sync <= start_meta;
            start_last <= start_sync;
            done <= 0;
            rd_valid <= 0;

            if (state == S_IDLE) begin
                scl_low <= 0;
                sda_low <= 0;
                div_cnt <= 0;
                tbuf_cnt <= 0;
                if (start_rise && (wr_len != 0 || rd_len != 0)) begin
                    busy <= 1;
                    ack_error <= 0;
                    wr_index <= 0;
                    rd_index <= 0;
                    phase <= (wr_len != 0) ? P_ADDR_W : P_ADDR_R;
                    state <= S_START;
                end else begin
                    busy <= 0;
                end
            end else if (state == S_BUS_FREE) begin
                scl_low <= 0;
                sda_low <= 0;
                if (tbuf_cnt >= TBUF_CYCLES - 1) begin
                    busy <= 0;
                    done <= 1;
                    state <= S_IDLE;
                end else begin
                    tbuf_cnt <= tbuf_cnt + 1'b1;
                end
            end else if (div_cnt < I2C_DIV - 1) begin
                div_cnt <= div_cnt + 1'b1;
            end else begin
                div_cnt <= 0;

                case (state)
                    S_START: begin
                        scl_low <= 0;
                        sda_low <= 1;
                        load_tx_byte;
                    end

                    S_LOAD: begin
                        // 该字节期间 wr_index 保持稳定。
                        // 在传输任何 SCL 数据位之前先锁存完整字节。
                        load_tx_byte;
                    end

                    S_SEND_LOW: begin
                        scl_low <= 1;
                        state <= S_SEND_SETUP;
                    end

                    S_SEND_SETUP: begin
                        sda_low <= ~tx_byte[bit_cnt];
                        state <= S_SEND_HIGH;
                    end

                    S_SEND_HIGH: begin
                        scl_low <= 0;
                        if (scl_in) begin
                            if (bit_cnt == 0)
                                state <= S_ACK_LOW;
                            else begin
                                bit_cnt <= bit_cnt - 1'b1;
                                state <= S_SEND_LOW;
                            end
                        end
                    end

                    S_ACK_LOW: begin
                        scl_low <= 1;
                        state <= S_ACK_RELEASE;
                    end

                    S_ACK_RELEASE: begin
                        sda_low <= 0;
                        state <= S_ACK_HIGH;
                    end

                    S_ACK_HIGH: begin
                        scl_low <= 0;
                        if (scl_in)
                            state <= S_ACK_SAMPLE;
                    end

                    S_ACK_SAMPLE: begin
                        if (sda_in) begin
                            ack_error <= 1;
                            start_stop;
                        end else if (phase == P_ADDR_W) begin
                            if (wr_len == 0) begin
                                if (rd_len != 0) begin
                                    phase <= P_ADDR_R;
                                    state <= S_START;
                                end else begin
                                    start_stop;
                                end
                            end else begin
                                phase <= P_WRITE;
                                state <= S_LOAD;
                            end
                        end else if (phase == P_WRITE) begin
                            if (wr_index == wr_len - 1) begin
                                if (rd_len != 0) begin
                                    phase <= P_ADDR_R;
                                    state <= S_START;
                                end else begin
                                    start_stop;
                                end
                            end else begin
                                wr_index <= wr_index + 1'b1;
                                state <= S_LOAD;
                            end
                        end else if (phase == P_ADDR_R) begin
                            phase <= P_READ;
                            bit_cnt <= 7;
                            rx_shift <= 0;
                            state <= S_READ_LOW;
                        end else begin
                            start_stop;
                        end
                    end

                    S_READ_LOW: begin
                        scl_low <= 1;
                        state <= S_READ_RELEASE;
                    end

                    S_READ_RELEASE: begin
                        sda_low <= 0;
                        state <= S_READ_HIGH;
                    end

                    S_READ_HIGH: begin
                        scl_low <= 0;
                        if (scl_in)
                            state <= S_READ_SAMPLE;
                    end

                    S_READ_SAMPLE: begin
                        rx_shift[bit_cnt] <= sda_in;
                        if (bit_cnt == 0) begin
                            // rd_data 与 rd_valid 同时寄存。
                            // rd_index 会保持不变，直到后续的 MACK 阶段，
                            // 接收方因此可在 rd_valid 有效的时钟沿同时采样两者。
                            rd_data <= {rx_shift[7:1], sda_in};
                            rd_valid <= 1;
                            state <= S_MACK_LOW;
                        end else begin
                            bit_cnt <= bit_cnt - 1'b1;
                            state <= S_READ_LOW;
                        end
                    end

                    S_MACK_LOW: begin
                        scl_low <= 1;
                        state <= S_MACK_SETUP;
                    end

                    S_MACK_SETUP: begin
                        sda_low <= (rd_index == rd_len - 1) ? 1'b0 : 1'b1;
                        state <= S_MACK_HIGH;
                    end

                    S_MACK_HIGH: begin
                        scl_low <= 0;
                        if (scl_in) begin
                            if (rd_index == rd_len - 1) begin
                                state <= S_STOP_LOW;
                            end else begin
                                rd_index <= rd_index + 1'b1;
                                bit_cnt <= 7;
                                rx_shift <= 0;
                                state <= S_READ_LOW;
                            end
                        end
                    end

                    S_STOP_LOW: begin
                        scl_low <= 1;
                        sda_low <= 1;
                        state <= S_STOP_HIGH;
                    end

                    S_STOP_HIGH: begin
                        scl_low <= 0;
                        sda_low <= 1;
                        if (scl_in)
                            state <= S_STOP_RELEASE;
                    end

                    S_STOP_RELEASE: begin
                        scl_low <= 0;
                        sda_low <= 0;
                        tbuf_cnt <= 0;
                        state <= S_BUS_FREE;
                    end

                    default: state <= S_IDLE;
                endcase
            end
        end
    end

endmodule
