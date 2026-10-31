// 驱动 OLED 并显示当前状态、文本和位置数值。
module oled #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer I2C_FREQ_HZ = 300_000,
    parameter integer REFRESH_MS  = 50
) (
    input  wire         sys_clk,
    input  wire         rst_n,
    input  wire [2:0]   display_mode,
    input  wire [127:0] text_data,
    input  wire [127:0] text_data2,
    input  wire [31:0]  number_data,
    inout  wire         oled_scl,
    inout  wire         oled_sda
);

    localparam integer REFRESH_CYCLES = CLK_FREQ_HZ / 1000 * REFRESH_MS;
    localparam integer REFRESH_W = (REFRESH_CYCLES < 2) ? 1 : $clog2(REFRESH_CYCLES);
    localparam [6:0] OLED_ADDR = 7'h3C;

    localparam [2:0] F_INIT=0, F_CLEAR_CMD=1, F_CLEAR_DATA=2,
                     F_TEXT_CMD=3, F_TEXT_DATA=4;
    localparam [1:0] C_IDLE=0, C_WAIT=1;

    reg [REFRESH_W-1:0] refresh_cnt;
    reg [1:0] ctrl_state;
    reg [2:0] frame_type;
    reg [9:0] frame_last;
    reg [2:0] page_cnt;
    reg init_done;
    reg clear_done;
    reg [2:0] mode_latched;
    reg [127:0] text_latched;
    reg [127:0] text2_latched;
    reg [31:0] number_latched;

    reg i2c_start;
    wire i2c_done;
    wire i2c_busy;
    wire i2c_ack_error;
    wire [9:0] i2c_byte_index;
    wire [7:0] i2c_rd_data;
    wire [9:0] i2c_rd_index;
    wire i2c_rd_valid;
    wire [9:0] i2c_len = frame_last + 10'd1;
    wire [7:0] i2c_tx_data;

    reg [7:0] font_ascii;
    reg [2:0] font_col;
    wire [7:0] font_data;

    oled_number u_font (
        .ascii (font_ascii),
        .col   (font_col),
        .data  (font_data)
    );

    i2c #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .I2C_FREQ_HZ (I2C_FREQ_HZ)
    ) u_oled_i2c (
        .sys_clk    (sys_clk),
        .rst_n      (rst_n),
        .start      (i2c_start),
        .dev_addr   (OLED_ADDR),
        .wr_len     (i2c_len),
        .wr_data    (i2c_tx_data),
        .wr_index   (i2c_byte_index),
        .rd_len     (10'd0),
        .rd_data    (i2c_rd_data),
        .rd_index   (i2c_rd_index),
        .rd_valid   (i2c_rd_valid),
        .busy       (i2c_busy),
        .done       (i2c_done),
        .ack_error  (i2c_ack_error),
        .i2c_scl    (oled_scl),
        .i2c_sda    (oled_sda)
    );

    function [7:0] decimal_char;
        input [31:0] value;
        input [3:0] pos;
        reg [31:0] magnitude;
        begin
            magnitude = value[31] ? (~value + 1'b1) : value;
            case (pos)
                0: decimal_char = value[31] ? "-" : " ";
                1: decimal_char = magnitude / 1000000000 % 10 + "0";
                2: decimal_char = magnitude / 100000000  % 10 + "0";
                3: decimal_char = magnitude / 10000000   % 10 + "0";
                4: decimal_char = magnitude / 1000000    % 10 + "0";
                5: decimal_char = magnitude / 100000     % 10 + "0";
                6: decimal_char = magnitude / 10000      % 10 + "0";
                7: decimal_char = magnitude / 1000       % 10 + "0";
                8: decimal_char = magnitude / 100        % 10 + "0";
                9: decimal_char = magnitude / 10         % 10 + "0";
                default: decimal_char = magnitude % 10 + "0";
            endcase
        end
    endfunction

    function [7:0] display_char;
        input [4:0] pos;
        begin
            if (mode_latched == 3'd3) begin
                if (page_cnt == 0 && pos < 16)
                    display_char = text_latched[127-pos*8 -: 8];
                else
                    display_char = " ";
            end else if (mode_latched == 3'd4) begin
                case (page_cnt)
                    0: display_char = (pos < 16) ? text_latched[127-pos*8 -: 8] : " ";
                    1: display_char = (pos < 16) ? text2_latched[127-pos*8 -: 8] : " ";
                    2: display_char = (pos < 11) ? decimal_char(number_latched, pos[3:0]) : " ";
                    default: display_char = " ";
                endcase
            end else if (page_cnt == 1) begin
                display_char = (pos < 11) ? decimal_char(number_latched, pos[3:0]) : " ";
            end else if (mode_latched == 2'd0) begin
                if (pos >= 16)
                    display_char = " ";
                else if (page_cnt == 0)
                    display_char = text_latched[127-pos*8 -: 8];
                else
                    display_char = text2_latched[127-pos*8 -: 8];
            end
            else if (mode_latched == 2'd1)
                display_char = (pos < 11) ? decimal_char(number_latched, pos[3:0]) : " ";
            else
                display_char = (pos < 16 && number_latched[31-pos]) ? "1" : "0";
        end
    endfunction

    always @* begin
        font_ascii = " ";
        font_col = 3'd0;
        if (frame_type == F_TEXT_DATA &&
            (page_cnt < 2 || (mode_latched == 3'd4 && page_cnt == 2)) &&
            i2c_byte_index >= 1 && i2c_byte_index < 97) begin
            font_ascii = display_char((i2c_byte_index - 1) / 6);
            font_col = (i2c_byte_index - 1) % 6;
        end
    end

    function [7:0] frame_byte;
        input [2:0] ftype;
        input [9:0] pos;
        input [2:0] page;
        begin
            frame_byte = 8'h00;
            case (ftype)
                F_INIT: begin
                    case (pos)
                        0: frame_byte=8'h00;
                        1: frame_byte=8'hAE; 2: frame_byte=8'hD5;
                        3: frame_byte=8'h80; 4: frame_byte=8'hA8;
                        5: frame_byte=8'h3F; 6: frame_byte=8'hD3;
                        7: frame_byte=8'h00; 8: frame_byte=8'h40;
                        9: frame_byte=8'h81; 10: frame_byte=8'h8F;
                        11: frame_byte=8'h8D; 12: frame_byte=8'h14;
                        13: frame_byte=8'h20; 14: frame_byte=8'h02;
                        15: frame_byte=8'hA1; 16: frame_byte=8'hC8;
                        17: frame_byte=8'hDA; 18: frame_byte=8'h12;
                        19: frame_byte=8'hA4; 20: frame_byte=8'hA6;
                        21: frame_byte=8'hAF;
                        default: frame_byte=8'h00;
                    endcase
                end
                F_CLEAR_CMD, F_TEXT_CMD: begin
                    case (pos)
                        0: frame_byte=8'h00;
                        1: frame_byte=8'hB0 | page;
                        2: frame_byte=8'h00;
                        3: frame_byte=8'h10;
                        default: frame_byte=8'h00;
                    endcase
                end
                F_CLEAR_DATA: begin
                    if (pos == 0) frame_byte=8'h40;
                    else frame_byte=8'h00;
                end
                F_TEXT_DATA: begin
                    if (pos == 0) frame_byte=8'h40;
                    else if ((page < 2 || (mode_latched == 3'd4 && page == 2)) &&
                             pos < 97 && ((pos - 1) % 6) < 5) frame_byte=font_data;
                    else frame_byte=8'h00;
                end
                default: frame_byte=8'h00;
            endcase
        end
    endfunction

    assign i2c_tx_data = frame_byte(frame_type, i2c_byte_index, page_cnt);

    task start_frame;
        input [2:0] next_type;
        input [9:0] last_pos;
        begin
            frame_type <= next_type;
            frame_last <= last_pos;
            i2c_start <= 1'b1;
            ctrl_state <= C_WAIT;
        end
    endtask

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            refresh_cnt <= 0;
            ctrl_state <= C_IDLE;
            frame_type <= F_INIT;
            frame_last <= 0;
            page_cnt <= 0;
            init_done <= 0;
            clear_done <= 0;
            mode_latched <= 0;
            text_latched <= 0;
            text2_latched <= 0;
            number_latched <= 0;
            i2c_start <= 0;
        end else begin
            i2c_start <= 0;

            if (ctrl_state == C_IDLE && init_done && refresh_cnt < REFRESH_CYCLES - 1)
                refresh_cnt <= refresh_cnt + 1'b1;

            case (ctrl_state)
                C_IDLE: begin
                    if (!init_done) begin
                        start_frame(F_INIT, 10'd21);
                    end else if (refresh_cnt >= REFRESH_CYCLES - 1) begin
                        refresh_cnt <= 0;
                        mode_latched <= display_mode;
                        text_latched <= text_data;
                        text2_latched <= text_data2;
                        number_latched <= number_data;
                        page_cnt <= 0;
                        start_frame(F_TEXT_CMD, 10'd3);
                    end
                end

                C_WAIT: begin
                    if (i2c_done) begin
                        if (frame_type == F_INIT) begin
                            init_done <= 1;
                            page_cnt <= 0;
                            refresh_cnt <= 0;
                            mode_latched <= display_mode;
                            text_latched <= text_data;
                            text2_latched <= text_data2;
                            number_latched <= number_data;
                            start_frame(F_TEXT_CMD, 10'd3);
                        end else if (frame_type == F_CLEAR_CMD) begin
                            start_frame(F_CLEAR_DATA, 10'd128);
                        end else if (frame_type == F_CLEAR_DATA) begin
                            if (page_cnt == 7) begin
                                clear_done <= 1;
                                page_cnt <= 0;
                                refresh_cnt <= 0;
                                mode_latched <= display_mode;
                                text_latched <= text_data;
                                text2_latched <= text_data2;
                                number_latched <= number_data;
                                start_frame(F_TEXT_CMD, 10'd3);
                            end else begin
                                page_cnt <= page_cnt + 1'b1;
                                start_frame(F_CLEAR_CMD, 10'd3);
                            end
                        end else if (frame_type == F_TEXT_CMD) begin
                            start_frame(F_TEXT_DATA, 10'd128);
                        end else if (frame_type == F_TEXT_DATA) begin
                            if (page_cnt == 7) begin
                                page_cnt <= 0;
                                ctrl_state <= C_IDLE;
                            end else begin
                                page_cnt <= page_cnt + 1'b1;
                                start_frame(F_TEXT_CMD, 10'd3);
                            end
                        end else begin
                            ctrl_state <= C_IDLE;
                        end
                    end
                end

                default: ctrl_state <= C_IDLE;
            endcase
        end
    end

endmodule
