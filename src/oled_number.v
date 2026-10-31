// 将 ASCII 字符转换为 OLED 点阵字形。
module oled_number (
    input  wire [7:0] ascii,
    input  wire [2:0] col,
    output reg  [7:0] data
);
    reg [39:0] glyph;
    reg [7:0] c;

    always @* begin
        c     = 8'h20;
        glyph = 40'h0000000000;
        data  = 8'h00;

        c = ascii;
        case (c)
            "0": glyph=40'h3E5149453E; "1": glyph=40'h0000427F40;
            "2": glyph=40'h4261514946; "3": glyph=40'h2241494936;
            "4": glyph=40'h183E127F10; "5": glyph=40'h2745454539;
            "6": glyph=40'h3E49494930; "7": glyph=40'h0101710907;
            "8": glyph=40'h3649494936; "9": glyph=40'h064949493E;
            "A": glyph=40'h7E1111117E; "B": glyph=40'h7F49494936;
            "C": glyph=40'h3E41414122; "D": glyph=40'h7F4141221C;
            "E": glyph=40'h7F49494941; "F": glyph=40'h7F09090901;
            "G": glyph=40'h3E41495132; "H": glyph=40'h7F0808087F;
            "I": glyph=40'h00417F4100; "J": glyph=40'h2040413F01;
            "K": glyph=40'h7F08142241; "L": glyph=40'h7F40404040;
            "M": glyph=40'h7F020C027F; "N": glyph=40'h7F0408107F;
            "O": glyph=40'h3E4141413E; "P": glyph=40'h7F09090906;
            "Q": glyph=40'h3E4141513E; "R": glyph=40'h7F09192946;
            "S": glyph=40'h2649494931; "T": glyph=40'h017F010101;
            "U": glyph=40'h3F4040403F; "V": glyph=40'h1F2040201F;
            "W": glyph=40'h3F4030403F; "X": glyph=40'h631C1C6363;
            "Y": glyph=40'h0708780807; "Z": glyph=40'h6149514543;
            "a": glyph=40'h2054545478; "b": glyph=40'h7F48444438;
            "c": glyph=40'h3844444420; "d": glyph=40'h384444487F;
            "e": glyph=40'h3854545418; "f": glyph=40'h087E090102;
            "g": glyph=40'h0C5252523E; "h": glyph=40'h7F08040478;
            "i": glyph=40'h00447D4000; "j": glyph=40'h2040443D00;
            "k": glyph=40'h7F10284400; "l": glyph=40'h00417F4000;
            "m": glyph=40'h7C04180478; "n": glyph=40'h7C08040478;
            "o": glyph=40'h3844444438; "p": glyph=40'h7C14141408;
            "q": glyph=40'h081414187C; "r": glyph=40'h7C08040408;
            "s": glyph=40'h4854545420; "t": glyph=40'h043F444020;
            "u": glyph=40'h3C4040207C; "v": glyph=40'h1C2040201C;
            "w": glyph=40'h3C4030403C; "x": glyph=40'h4428102844;
            "y": glyph=40'h0C5050503C; "z": glyph=40'h4464544C44;
            "!": glyph=40'h00005F0000; "_": glyph=40'h4040404040;
            ".": glyph=40'h0000606000;
            ",": glyph=40'h0000603000; ":": glyph=40'h0014140000;
            "-": glyph=40'h0008080800; "/": glyph=40'h4020100804;
            "?": glyph=40'h0201590906; " ": glyph=40'h0000000000;
            default: glyph=40'h0000000000;
        endcase
        case (col)
            0: data=glyph[39:32]; 1: data=glyph[31:24];
            2: data=glyph[23:16]; 3: data=glyph[15:8];
            4: data=glyph[7:0]; default: data=8'h00;
        endcase
    end
endmodule
