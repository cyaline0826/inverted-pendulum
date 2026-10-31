// 解析串口收到的有符号十进制位置目标。
module uart_position (
    input  wire sys_clk,
    input  wire rst_n,
    input  wire [7:0] rx_data,
    input  wire rx_valid,
    input  wire accept_enable,
    output reg signed [31:0] target_position,
    output reg target_valid
);

    reg [31:0] magnitude;
    reg negative;
    reg line_started;
    reg line_has_digit;
    reg line_invalid;

    wire is_digit = rx_data >= "0" && rx_data <= "9";
    wire [3:0] digit = rx_data[3:0];
    wire line_end = rx_data == 8'h0D || rx_data == 8'h0A;
    wire magnitude_overflow =
        magnitude > 32'd214748364 ||
        (magnitude == 32'd214748364 &&
         ((negative && digit > 4'd8) || (!negative && digit > 4'd7)));

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            magnitude <= 0;
            negative <= 0;
            line_started <= 0;
            line_has_digit <= 0;
            line_invalid <= 0;
            target_position <= 0;
            target_valid <= 0;
        end else begin
            target_valid <= 0;

            if (rx_valid) begin
                if (line_end) begin
                    if (line_has_digit && !line_invalid && accept_enable) begin
                        target_position <= negative ? -$signed(magnitude) : $signed(magnitude);
                        target_valid <= 1;
                    end
                    magnitude <= 0;
                    negative <= 0;
                    line_started <= 0;
                    line_has_digit <= 0;
                    line_invalid <= 0;
                end else if (!line_invalid) begin
                    if (rx_data == "-" || rx_data == "+") begin
                        if (!line_started) begin
                            line_started <= 1;
                            negative <= rx_data == "-";
                        end else begin
                            line_invalid <= 1;
                        end
                    end else if (is_digit) begin
                        line_started <= 1;
                        if (magnitude_overflow) begin
                            line_invalid <= 1;
                        end else begin
                            magnitude <= (magnitude << 3) + (magnitude << 1) + digit;
                            line_has_digit <= 1;
                        end
                    end else begin
                        line_invalid <= 1;
                    end
                end
            end
        end
    end
endmodule
