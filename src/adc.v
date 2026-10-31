// 通过 I2C 读取 ADS1115 的 A2 和 A0 通道数据。
module adc #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer I2C_FREQ_HZ = 300_000,
    parameter [6:0]   ADS1115_ADDR = 7'h48,
    parameter integer CONV_WAIT_US = 1200,
    parameter integer A0_SAMPLE_HZ = 20
) (
    input  wire        sys_clk,
    input  wire        rst_n,
    output reg  [15:0] adc_data,
    output reg  [15:0] adc_data_a2,
    output reg         adc_valid,
    output reg  [15:0] config_data,
    output wire        busy,
    output reg         ack_error,
    inout  wire        adc_scl,
    inout  wire        adc_sda,
    input  wire        a0_enable,
    output reg  [15:0] adc_data_a0,
    output reg         adc_a0_valid
);

    localparam integer CONV_WAIT_CYCLES = CLK_FREQ_HZ / 1000000 * CONV_WAIT_US;
    localparam integer WAIT_W = (CONV_WAIT_CYCLES < 2) ? 1 : $clog2(CONV_WAIT_CYCLES);
    localparam integer A0_DIV = CLK_FREQ_HZ / A0_SAMPLE_HZ;
    localparam integer A0_W = A0_DIV < 2 ? 1 : $clog2(A0_DIV);

    localparam [3:0] S_CONFIG=0, S_WAIT_CONFIG=1,
                     S_CONFIG_PTR=2, S_WAIT_CONFIG_PTR=3,
                     S_CONFIG_READ=4, S_WAIT_CONFIG_READ=5,
                     S_CONV_WAIT=6, S_CONV_PTR=7,
                     S_WAIT_CONV_PTR=8, S_CONV_READ=9,
                     S_WAIT_CONV_READ=10;

    localparam [2:0]  CFG_MUX_AIN2_GND = 3'b110;
    localparam [2:0]  CFG_MUX_AIN0_GND = 3'b100;
    localparam [2:0]  CFG_PGA_4V096    = 3'b001;
    localparam        CFG_MODE_SINGLE  = 1'b1;
    localparam [2:0]  CFG_DR_860SPS    = 3'b111;
    localparam        CFG_COMP_TRAD    = 1'b0;
    localparam        CFG_COMP_ACT_LOW = 1'b0;
    localparam        CFG_COMP_NONLAT  = 1'b0;
    localparam [1:0]  CFG_COMP_DISABLE = 2'b11;

    reg draining, channel_a0;
    reg [A0_W-1:0] a0_age; // 启动带标签的单次转换前，先清理复位或错误遗留的转换。
    wire [15:0] config_current = {
        !draining,
        channel_a0 ? CFG_MUX_AIN0_GND : CFG_MUX_AIN2_GND,
        CFG_PGA_4V096,
        CFG_MODE_SINGLE,
        CFG_DR_860SPS,
        CFG_COMP_TRAD,
        CFG_COMP_ACT_LOW,
        CFG_COMP_NONLAT,
        CFG_COMP_DISABLE
    };

    reg [3:0] state;
    reg [WAIT_W-1:0] wait_cnt;
    reg i2c_start;
    reg [9:0] wr_len;
    reg [9:0] rd_len;
    reg [7:0] wr_data;
    reg [7:0] pointer_reg;
    reg [15:0] read_shift;

    wire [9:0] wr_index;
    wire [7:0] rd_data;
    wire [9:0] rd_index;
    wire rd_valid;
    wire i2c_busy;
    wire i2c_done;
    wire i2c_ack_error;

    assign busy = (state != S_CONFIG) || i2c_busy || i2c_start;

    i2c #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .I2C_FREQ_HZ (I2C_FREQ_HZ)
    ) u_adc_i2c (
        .sys_clk   (sys_clk),
        .rst_n     (rst_n),
        .start     (i2c_start),
        .dev_addr  (ADS1115_ADDR),
        .wr_len    (wr_len),
        .wr_data   (wr_data),
        .wr_index  (wr_index),
        .rd_len    (rd_len),
        .rd_data   (rd_data),
        .rd_index  (rd_index),
        .rd_valid  (rd_valid),
        .busy      (i2c_busy),
        .done      (i2c_done),
        .ack_error (i2c_ack_error),
        .i2c_scl   (adc_scl),
        .i2c_sda   (adc_sda)
    );

    always @* begin
        // I2C 仅在 S_LOAD 状态采样 wr_data，
        // 整个字节传输期间 wr_index 保持不变。
        wr_data = 8'h00;
        if (wr_len == 10'd3) begin
            case (wr_index)
                0: wr_data = 8'h01;
                1: wr_data = config_current[15:8];
                default: wr_data = config_current[7:0];
            endcase
        end else begin
            // 选择 ADS1115 的目标寄存器。
            wr_data = pointer_reg;
        end
    end

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_CONFIG;
            wait_cnt <= 0;
            i2c_start <= 0;
            wr_len <= 0;
            rd_len <= 0;
            read_shift <= 0;
            adc_data <= 0;
            adc_data_a2 <= 0;
            adc_valid <= 0;
            adc_data_a0 <= 0;
            adc_a0_valid <= 0;
            draining <= 1;
            channel_a0 <= 0;
            a0_age <= A0_DIV - 1;
            config_data <= 0;
            pointer_reg <= 0;
            ack_error <= 0;
        end else begin
            i2c_start <= 0;
            adc_valid <= 0;
            adc_a0_valid <= 0;
            if (!a0_enable)
                a0_age <= A0_DIV - 1;
            else if (a0_age < A0_DIV - 1)
                a0_age <= a0_age + 1'b1;

            if (rd_valid) begin
                if (rd_index == 0)
                    read_shift[15:8] <= rd_data;
                else
                    read_shift[7:0] <= rd_data;
            end

            if (i2c_done && i2c_ack_error) begin
                ack_error <= 1'b1;
                draining <= 1;
                channel_a0 <= 0;
                state <= S_CONFIG;
            end else case (state)
                S_CONFIG: begin
                    wr_len <= 10'd3;
                    rd_len <= 10'd0;
                    i2c_start <= 1'b1;
                    state <= S_WAIT_CONFIG;
                end

                S_WAIT_CONFIG: begin
                    if (i2c_done) begin
                        ack_error <= 1'b0;
                        wait_cnt <= 0;
                        state <= S_CONV_WAIT;
                    end
                end

                S_CONFIG_PTR: begin
                    wr_len <= 10'd1;
                    rd_len <= 10'd0;
                    pointer_reg <= 8'h01;
                    i2c_start <= 1'b1;
                    state <= S_WAIT_CONFIG_PTR;
                end

                S_WAIT_CONFIG_PTR: begin
                    if (i2c_done) begin
                        state <= S_CONFIG_READ;
                    end
                end

                S_CONFIG_READ: begin
                    wr_len <= 10'd0;
                    rd_len <= 10'd2;
                    read_shift <= 0;
                    i2c_start <= 1'b1;
                    state <= S_WAIT_CONFIG_READ;
                end

                S_WAIT_CONFIG_READ: begin
                    if (i2c_done) begin
                        config_data <= read_shift;
                        if (!read_shift[15]) begin
                            // 寄存器指针仍指向配置寄存器；再次轮询，无需重新等待完整转换周期。
                            state <= S_CONFIG_READ;
                        end else if (draining) begin
                            draining <= 0;
                            channel_a0 <= 0;
                            state <= S_CONFIG;
                        end else begin
                            state <= S_CONV_PTR;
                        end
                    end
                end

                S_CONV_WAIT: begin
                    if (wait_cnt >= CONV_WAIT_CYCLES - 1) begin
                        // 每次写入配置时都已选择寄存器 1。
                        state <= S_CONFIG_READ;
                    end else begin
                        wait_cnt <= wait_cnt + 1'b1;
                    end
                end

                S_CONV_PTR: begin
                    wr_len <= 10'd1;
                    rd_len <= 10'd0;
                    pointer_reg <= 8'h00;
                    i2c_start <= 1'b1;
                    state <= S_WAIT_CONV_PTR;
                end

                S_WAIT_CONV_PTR: begin
                    if (i2c_done) begin
                        state <= S_CONV_READ;
                    end
                end

                S_CONV_READ: begin
                    wr_len <= 10'd0;
                    rd_len <= 10'd2;
                    read_shift <= 0;
                    i2c_start <= 1'b1;
                    state <= S_WAIT_CONV_READ;
                end

                S_WAIT_CONV_READ: begin
                    if (i2c_done) begin
                        if (channel_a0) begin
                            adc_data_a0 <= read_shift;
                            adc_a0_valid <= 1'b1;
                        end else begin
                            adc_data <= read_shift;
                            adc_data_a2 <= read_shift;
                            adc_valid <= 1'b1;
                        end
                        // 仅在 A2 转换后插入一次 A0 采样，然后立即返回 A2。
                        if (!channel_a0 && a0_enable && a0_age >= A0_DIV - 1) begin
                            channel_a0 <= 1;
                            a0_age <= 0;
                        end else channel_a0 <= 0;
                        state <= S_CONFIG;
                    end
                end

                default: state <= S_CONFIG;
            endcase
        end
    end

endmodule
