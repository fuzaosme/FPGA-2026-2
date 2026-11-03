// ===========================================================================
// style_proc_96.v
// 扩-1：六档风格选择，在 96bit（4 像素/拍）域上做：
//   0 = 原图（彩色）      1 = 自适应二值化      2 = 线稿（白底黑边）
//   3 = 色块卡通          4 = 伪彩热力图        5 = 检测掩码高亮
//
// 输入 96bit 像素排布（与 isp 一致）：
//   p0 = [95:72], p1 = [71:48], p2 = [47:24], p3 = [23:0]
//   每个 24bit = {R[23:16], G[15:8], B[7:0]}
// ===========================================================================

`timescale 1ns/1ps

module style_proc_96 #(
    parameter integer DISPLAY_GAMMA_ENABLE = 1,
    parameter integer DISPLAY_COLOR_TONE_ENABLE = 1
) (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire [2:0]  I_style,
    input  wire [95:0] I_rgb,     // AWB 彩色
    input  wire [95:0] I_bin,     // 自适应二值化输出（0 或 FFFFFF）
    input  wire [3:0]  I_mask,    // 颜色掩码，bit3..0 = 第0..3列
    input  wire        I_tvalid,
    output reg  [95:0] O_tdata
);

    wire [23:0] p0 = I_rgb[95:72];
    wire [23:0] p1 = I_rgb[71:48];
    wire [23:0] p2 = I_rgb[47:24];
    wire [23:0] p3 = I_rgb[23: 0];

    // 亮度：Y = R/4 + G/2 + B/4（近似灰度）
    function [7:0] F_y;
        input [23:0] p;
        begin
            F_y = {2'b0, p[23:18]} + {1'b0, p[15:9]} + {2'b0, p[7:2]};
        end
    endfunction

    wire [7:0] y0 = F_y(p0);
    wire [7:0] y1 = F_y(p1);
    wire [7:0] y2 = F_y(p2);
    wire [7:0] y3 = F_y(p3);

    // 上一拍最后一个像素的亮度，用于线稿跨拍边沿
    reg [7:0] prev_y;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)
            prev_y <= 8'd0;
        else if (I_tvalid)
            prev_y <= y3;
    end

    // 水平梯度（线稿/卡通用）
    function [7:0] F_absdiff;
        input [7:0] a, b;
        begin
            F_absdiff = (a > b) ? (a - b) : (b - a);
        end
    endfunction

    localparam [7:0] EDGE_TH = 8'd40;
    wire e0 = (F_absdiff(y0, prev_y) > EDGE_TH);
    wire e1 = (F_absdiff(y1, y0) > EDGE_TH);
    wire e2 = (F_absdiff(y2, y1) > EDGE_TH);
    wire e3 = (F_absdiff(y3, y2) > EDGE_TH);

    // 卡通色块：每通道二值化
    function [23:0] F_cartoon;
        input [23:0] p;
        begin
            F_cartoon = { (p[23:16] > 8'd127) ? 8'hFF : 8'h00,
                          (p[15: 8] > 8'd127) ? 8'hFF : 8'h00,
                          (p[ 7: 0] > 8'd127) ? 8'hFF : 8'h00 };
        end
    endfunction

    // 伪彩热力图 LUT（按亮度分 8 档）
    function [23:0] F_heat;
        input [7:0] y;
        reg [23:0] c;
        begin
            case (y[7:5])
                3'd0: c = 24'h0000FF;   // 蓝
                3'd1: c = 24'h00FFFF;   // 青
                3'd2: c = 24'h00FF00;   // 绿
                3'd3: c = 24'hFFFF00;   // 黄
                3'd4: c = 24'hFF8000;   // 橙
                3'd5: c = 24'hFF0000;   // 红
                3'd6: c = 24'hFF00FF;   // 品红
                3'd7: c = 24'hFFFFFF;   // 白
                default: c = 24'h000000;
            endcase
            F_heat = c;
        end
    endfunction

    // 掩码高亮：命中 -> 品红；未命中 -> 灰度
    function [23:0] F_hl;
        input [23:0] p;
        input [7:0]  y;
        input        m;
        begin
            F_hl = m ? 24'hFF00FF : {y, y, y};
        end
    endfunction

    // Display-only engineering curve: round(255*(input/255)^(1/2.2)).
    // The same transfer function is used for R/G/B, with fixed 0/255 endpoints.
    // Case ROM is self-contained: no runtime/synthesis initialization file.
    function [7:0] F_gamma8;
        input [7:0] code;
        begin
            case (code)
                8'd0: F_gamma8 = 8'd0;
                8'd1: F_gamma8 = 8'd21;
                8'd2: F_gamma8 = 8'd28;
                8'd3: F_gamma8 = 8'd34;
                8'd4: F_gamma8 = 8'd39;
                8'd5: F_gamma8 = 8'd43;
                8'd6: F_gamma8 = 8'd46;
                8'd7: F_gamma8 = 8'd50;
                8'd8: F_gamma8 = 8'd53;
                8'd9: F_gamma8 = 8'd56;
                8'd10: F_gamma8 = 8'd59;
                8'd11: F_gamma8 = 8'd61;
                8'd12: F_gamma8 = 8'd64;
                8'd13: F_gamma8 = 8'd66;
                8'd14: F_gamma8 = 8'd68;
                8'd15: F_gamma8 = 8'd70;
                8'd16: F_gamma8 = 8'd72;
                8'd17: F_gamma8 = 8'd74;
                8'd18: F_gamma8 = 8'd76;
                8'd19: F_gamma8 = 8'd78;
                8'd20: F_gamma8 = 8'd80;
                8'd21: F_gamma8 = 8'd82;
                8'd22: F_gamma8 = 8'd84;
                8'd23: F_gamma8 = 8'd85;
                8'd24: F_gamma8 = 8'd87;
                8'd25: F_gamma8 = 8'd89;
                8'd26: F_gamma8 = 8'd90;
                8'd27: F_gamma8 = 8'd92;
                8'd28: F_gamma8 = 8'd93;
                8'd29: F_gamma8 = 8'd95;
                8'd30: F_gamma8 = 8'd96;
                8'd31: F_gamma8 = 8'd98;
                8'd32: F_gamma8 = 8'd99;
                8'd33: F_gamma8 = 8'd101;
                8'd34: F_gamma8 = 8'd102;
                8'd35: F_gamma8 = 8'd103;
                8'd36: F_gamma8 = 8'd105;
                8'd37: F_gamma8 = 8'd106;
                8'd38: F_gamma8 = 8'd107;
                8'd39: F_gamma8 = 8'd109;
                8'd40: F_gamma8 = 8'd110;
                8'd41: F_gamma8 = 8'd111;
                8'd42: F_gamma8 = 8'd112;
                8'd43: F_gamma8 = 8'd114;
                8'd44: F_gamma8 = 8'd115;
                8'd45: F_gamma8 = 8'd116;
                8'd46: F_gamma8 = 8'd117;
                8'd47: F_gamma8 = 8'd118;
                8'd48: F_gamma8 = 8'd119;
                8'd49: F_gamma8 = 8'd120;
                8'd50: F_gamma8 = 8'd122;
                8'd51: F_gamma8 = 8'd123;
                8'd52: F_gamma8 = 8'd124;
                8'd53: F_gamma8 = 8'd125;
                8'd54: F_gamma8 = 8'd126;
                8'd55: F_gamma8 = 8'd127;
                8'd56: F_gamma8 = 8'd128;
                8'd57: F_gamma8 = 8'd129;
                8'd58: F_gamma8 = 8'd130;
                8'd59: F_gamma8 = 8'd131;
                8'd60: F_gamma8 = 8'd132;
                8'd61: F_gamma8 = 8'd133;
                8'd62: F_gamma8 = 8'd134;
                8'd63: F_gamma8 = 8'd135;
                8'd64: F_gamma8 = 8'd136;
                8'd65: F_gamma8 = 8'd137;
                8'd66: F_gamma8 = 8'd138;
                8'd67: F_gamma8 = 8'd139;
                8'd68: F_gamma8 = 8'd140;
                8'd69: F_gamma8 = 8'd141;
                8'd70: F_gamma8 = 8'd142;
                8'd71: F_gamma8 = 8'd143;
                8'd72: F_gamma8 = 8'd144;
                8'd73: F_gamma8 = 8'd144;
                8'd74: F_gamma8 = 8'd145;
                8'd75: F_gamma8 = 8'd146;
                8'd76: F_gamma8 = 8'd147;
                8'd77: F_gamma8 = 8'd148;
                8'd78: F_gamma8 = 8'd149;
                8'd79: F_gamma8 = 8'd150;
                8'd80: F_gamma8 = 8'd151;
                8'd81: F_gamma8 = 8'd151;
                8'd82: F_gamma8 = 8'd152;
                8'd83: F_gamma8 = 8'd153;
                8'd84: F_gamma8 = 8'd154;
                8'd85: F_gamma8 = 8'd155;
                8'd86: F_gamma8 = 8'd156;
                8'd87: F_gamma8 = 8'd156;
                8'd88: F_gamma8 = 8'd157;
                8'd89: F_gamma8 = 8'd158;
                8'd90: F_gamma8 = 8'd159;
                8'd91: F_gamma8 = 8'd160;
                8'd92: F_gamma8 = 8'd160;
                8'd93: F_gamma8 = 8'd161;
                8'd94: F_gamma8 = 8'd162;
                8'd95: F_gamma8 = 8'd163;
                8'd96: F_gamma8 = 8'd164;
                8'd97: F_gamma8 = 8'd164;
                8'd98: F_gamma8 = 8'd165;
                8'd99: F_gamma8 = 8'd166;
                8'd100: F_gamma8 = 8'd167;
                8'd101: F_gamma8 = 8'd167;
                8'd102: F_gamma8 = 8'd168;
                8'd103: F_gamma8 = 8'd169;
                8'd104: F_gamma8 = 8'd170;
                8'd105: F_gamma8 = 8'd170;
                8'd106: F_gamma8 = 8'd171;
                8'd107: F_gamma8 = 8'd172;
                8'd108: F_gamma8 = 8'd173;
                8'd109: F_gamma8 = 8'd173;
                8'd110: F_gamma8 = 8'd174;
                8'd111: F_gamma8 = 8'd175;
                8'd112: F_gamma8 = 8'd175;
                8'd113: F_gamma8 = 8'd176;
                8'd114: F_gamma8 = 8'd177;
                8'd115: F_gamma8 = 8'd178;
                8'd116: F_gamma8 = 8'd178;
                8'd117: F_gamma8 = 8'd179;
                8'd118: F_gamma8 = 8'd180;
                8'd119: F_gamma8 = 8'd180;
                8'd120: F_gamma8 = 8'd181;
                8'd121: F_gamma8 = 8'd182;
                8'd122: F_gamma8 = 8'd182;
                8'd123: F_gamma8 = 8'd183;
                8'd124: F_gamma8 = 8'd184;
                8'd125: F_gamma8 = 8'd184;
                8'd126: F_gamma8 = 8'd185;
                8'd127: F_gamma8 = 8'd186;
                8'd128: F_gamma8 = 8'd186;
                8'd129: F_gamma8 = 8'd187;
                8'd130: F_gamma8 = 8'd188;
                8'd131: F_gamma8 = 8'd188;
                8'd132: F_gamma8 = 8'd189;
                8'd133: F_gamma8 = 8'd190;
                8'd134: F_gamma8 = 8'd190;
                8'd135: F_gamma8 = 8'd191;
                8'd136: F_gamma8 = 8'd192;
                8'd137: F_gamma8 = 8'd192;
                8'd138: F_gamma8 = 8'd193;
                8'd139: F_gamma8 = 8'd194;
                8'd140: F_gamma8 = 8'd194;
                8'd141: F_gamma8 = 8'd195;
                8'd142: F_gamma8 = 8'd195;
                8'd143: F_gamma8 = 8'd196;
                8'd144: F_gamma8 = 8'd197;
                8'd145: F_gamma8 = 8'd197;
                8'd146: F_gamma8 = 8'd198;
                8'd147: F_gamma8 = 8'd199;
                8'd148: F_gamma8 = 8'd199;
                8'd149: F_gamma8 = 8'd200;
                8'd150: F_gamma8 = 8'd200;
                8'd151: F_gamma8 = 8'd201;
                8'd152: F_gamma8 = 8'd202;
                8'd153: F_gamma8 = 8'd202;
                8'd154: F_gamma8 = 8'd203;
                8'd155: F_gamma8 = 8'd203;
                8'd156: F_gamma8 = 8'd204;
                8'd157: F_gamma8 = 8'd205;
                8'd158: F_gamma8 = 8'd205;
                8'd159: F_gamma8 = 8'd206;
                8'd160: F_gamma8 = 8'd206;
                8'd161: F_gamma8 = 8'd207;
                8'd162: F_gamma8 = 8'd207;
                8'd163: F_gamma8 = 8'd208;
                8'd164: F_gamma8 = 8'd209;
                8'd165: F_gamma8 = 8'd209;
                8'd166: F_gamma8 = 8'd210;
                8'd167: F_gamma8 = 8'd210;
                8'd168: F_gamma8 = 8'd211;
                8'd169: F_gamma8 = 8'd212;
                8'd170: F_gamma8 = 8'd212;
                8'd171: F_gamma8 = 8'd213;
                8'd172: F_gamma8 = 8'd213;
                8'd173: F_gamma8 = 8'd214;
                8'd174: F_gamma8 = 8'd214;
                8'd175: F_gamma8 = 8'd215;
                8'd176: F_gamma8 = 8'd215;
                8'd177: F_gamma8 = 8'd216;
                8'd178: F_gamma8 = 8'd217;
                8'd179: F_gamma8 = 8'd217;
                8'd180: F_gamma8 = 8'd218;
                8'd181: F_gamma8 = 8'd218;
                8'd182: F_gamma8 = 8'd219;
                8'd183: F_gamma8 = 8'd219;
                8'd184: F_gamma8 = 8'd220;
                8'd185: F_gamma8 = 8'd220;
                8'd186: F_gamma8 = 8'd221;
                8'd187: F_gamma8 = 8'd221;
                8'd188: F_gamma8 = 8'd222;
                8'd189: F_gamma8 = 8'd223;
                8'd190: F_gamma8 = 8'd223;
                8'd191: F_gamma8 = 8'd224;
                8'd192: F_gamma8 = 8'd224;
                8'd193: F_gamma8 = 8'd225;
                8'd194: F_gamma8 = 8'd225;
                8'd195: F_gamma8 = 8'd226;
                8'd196: F_gamma8 = 8'd226;
                8'd197: F_gamma8 = 8'd227;
                8'd198: F_gamma8 = 8'd227;
                8'd199: F_gamma8 = 8'd228;
                8'd200: F_gamma8 = 8'd228;
                8'd201: F_gamma8 = 8'd229;
                8'd202: F_gamma8 = 8'd229;
                8'd203: F_gamma8 = 8'd230;
                8'd204: F_gamma8 = 8'd230;
                8'd205: F_gamma8 = 8'd231;
                8'd206: F_gamma8 = 8'd231;
                8'd207: F_gamma8 = 8'd232;
                8'd208: F_gamma8 = 8'd232;
                8'd209: F_gamma8 = 8'd233;
                8'd210: F_gamma8 = 8'd233;
                8'd211: F_gamma8 = 8'd234;
                8'd212: F_gamma8 = 8'd234;
                8'd213: F_gamma8 = 8'd235;
                8'd214: F_gamma8 = 8'd235;
                8'd215: F_gamma8 = 8'd236;
                8'd216: F_gamma8 = 8'd236;
                8'd217: F_gamma8 = 8'd237;
                8'd218: F_gamma8 = 8'd237;
                8'd219: F_gamma8 = 8'd238;
                8'd220: F_gamma8 = 8'd238;
                8'd221: F_gamma8 = 8'd239;
                8'd222: F_gamma8 = 8'd239;
                8'd223: F_gamma8 = 8'd240;
                8'd224: F_gamma8 = 8'd240;
                8'd225: F_gamma8 = 8'd241;
                8'd226: F_gamma8 = 8'd241;
                8'd227: F_gamma8 = 8'd242;
                8'd228: F_gamma8 = 8'd242;
                8'd229: F_gamma8 = 8'd243;
                8'd230: F_gamma8 = 8'd243;
                8'd231: F_gamma8 = 8'd244;
                8'd232: F_gamma8 = 8'd244;
                8'd233: F_gamma8 = 8'd245;
                8'd234: F_gamma8 = 8'd245;
                8'd235: F_gamma8 = 8'd246;
                8'd236: F_gamma8 = 8'd246;
                8'd237: F_gamma8 = 8'd247;
                8'd238: F_gamma8 = 8'd247;
                8'd239: F_gamma8 = 8'd248;
                8'd240: F_gamma8 = 8'd248;
                8'd241: F_gamma8 = 8'd249;
                8'd242: F_gamma8 = 8'd249;
                8'd243: F_gamma8 = 8'd249;
                8'd244: F_gamma8 = 8'd250;
                8'd245: F_gamma8 = 8'd250;
                8'd246: F_gamma8 = 8'd251;
                8'd247: F_gamma8 = 8'd251;
                8'd248: F_gamma8 = 8'd252;
                8'd249: F_gamma8 = 8'd252;
                8'd250: F_gamma8 = 8'd253;
                8'd251: F_gamma8 = 8'd253;
                8'd252: F_gamma8 = 8'd254;
                8'd253: F_gamma8 = 8'd254;
                8'd254: F_gamma8 = 8'd255;
                8'd255: F_gamma8 = 8'd255;
                default: F_gamma8 = code;
            endcase
        end
    endfunction

    // Color-preserving display gain. m=max(R,G,B). T(m) is Gamma2.2 with
    // a smooth toe: round(255*(m/255)^(1/2.2)*(259/255)*m/(m+4)).
    // The same Q12 gain T(m)/m scales all three channels of ONE pixel.
    // No black subtraction/hard cutoff; pure black stays black, 255 stays255.
    // This is an engineering display curve, not a calibrated sensor profile.
    function [15:0] F_color_gain;
        input [7:0] maximum;
        begin
            case (maximum)
                8'd0: F_color_gain = 16'd0;
                8'd1: F_color_gain = 16'd16384;
                8'd2: F_color_gain = 16'd20480;
                8'd3: F_color_gain = 16'd20480;
                8'd4: F_color_gain = 16'd20480;
                8'd5: F_color_gain = 16'd19661;
                8'd6: F_color_gain = 16'd19115;
                8'd7: F_color_gain = 16'd18725;
                8'd8: F_color_gain = 16'd18432;
                8'd9: F_color_gain = 16'd17749;
                8'd10: F_color_gain = 16'd17203;
                8'd11: F_color_gain = 16'd17129;
                8'd12: F_color_gain = 16'd16384;
                8'd13: F_color_gain = 16'd16069;
                8'd14: F_color_gain = 16'd15799;
                8'd15: F_color_gain = 16'd15292;
                8'd16: F_color_gain = 16'd15104;
                8'd17: F_color_gain = 16'd14697;
                8'd18: F_color_gain = 16'd14564;
                8'd19: F_color_gain = 16'd14228;
                8'd20: F_color_gain = 16'd13926;
                8'd21: F_color_gain = 16'd13653;
                8'd22: F_color_gain = 16'd13405;
                8'd23: F_color_gain = 16'd13178;
                8'd24: F_color_gain = 16'd12971;
                8'd25: F_color_gain = 16'd12780;
                8'd26: F_color_gain = 16'd12603;
                8'd27: F_color_gain = 16'd12288;
                8'd28: F_color_gain = 16'd12142;
                8'd29: F_color_gain = 16'd12006;
                8'd30: F_color_gain = 16'd11742;
                8'd31: F_color_gain = 16'd11627;
                8'd32: F_color_gain = 16'd11520;
                8'd33: F_color_gain = 16'd11295;
                8'd34: F_color_gain = 16'd11204;
                8'd35: F_color_gain = 16'd11001;
                8'd36: F_color_gain = 16'd10923;
                8'd37: F_color_gain = 16'd10738;
                8'd38: F_color_gain = 16'd10671;
                8'd39: F_color_gain = 16'd10503;
                8'd40: F_color_gain = 16'd10342;
                8'd41: F_color_gain = 16'd10290;
                8'd42: F_color_gain = 16'd10142;
                8'd43: F_color_gain = 16'd10097;
                8'd44: F_color_gain = 16'd9961;
                8'd45: F_color_gain = 16'd9830;
                8'd46: F_color_gain = 16'd9706;
                8'd47: F_color_gain = 16'd9674;
                8'd48: F_color_gain = 16'd9557;
                8'd49: F_color_gain = 16'd9446;
                8'd50: F_color_gain = 16'd9339;
                8'd51: F_color_gain = 16'd9316;
                8'd52: F_color_gain = 16'd9216;
                8'd53: F_color_gain = 16'd9119;
                8'd54: F_color_gain = 16'd9026;
                8'd55: F_color_gain = 16'd8937;
                8'd56: F_color_gain = 16'd8850;
                8'd57: F_color_gain = 16'd8767;
                8'd58: F_color_gain = 16'd8757;
                8'd59: F_color_gain = 16'd8678;
                8'd60: F_color_gain = 16'd8602;
                8'd61: F_color_gain = 16'd8528;
                8'd62: F_color_gain = 16'd8456;
                8'd63: F_color_gain = 16'd8387;
                8'd64: F_color_gain = 16'd8320;
                8'd65: F_color_gain = 16'd8255;
                8'd66: F_color_gain = 16'd8192;
                8'd67: F_color_gain = 16'd8131;
                8'd68: F_color_gain = 16'd8072;
                8'd69: F_color_gain = 16'd8014;
                8'd70: F_color_gain = 16'd7958;
                8'd71: F_color_gain = 16'd7904;
                8'd72: F_color_gain = 16'd7851;
                8'd73: F_color_gain = 16'd7799;
                8'd74: F_color_gain = 16'd7749;
                8'd75: F_color_gain = 16'd7700;
                8'd76: F_color_gain = 16'd7653;
                8'd77: F_color_gain = 16'd7607;
                8'd78: F_color_gain = 16'd7562;
                8'd79: F_color_gain = 16'd7518;
                8'd80: F_color_gain = 16'd7475;
                8'd81: F_color_gain = 16'd7433;
                8'd82: F_color_gain = 16'd7343;
                8'd83: F_color_gain = 16'd7304;
                8'd84: F_color_gain = 16'd7266;
                8'd85: F_color_gain = 16'd7228;
                8'd86: F_color_gain = 16'd7192;
                8'd87: F_color_gain = 16'd7156;
                8'd88: F_color_gain = 16'd7121;
                8'd89: F_color_gain = 16'd7087;
                8'd90: F_color_gain = 16'd7009;
                8'd91: F_color_gain = 16'd6977;
                8'd92: F_color_gain = 16'd6945;
                8'd93: F_color_gain = 16'd6915;
                8'd94: F_color_gain = 16'd6885;
                8'd95: F_color_gain = 16'd6855;
                8'd96: F_color_gain = 16'd6784;
                8'd97: F_color_gain = 16'd6756;
                8'd98: F_color_gain = 16'd6729;
                8'd99: F_color_gain = 16'd6703;
                8'd100: F_color_gain = 16'd6676;
                8'd101: F_color_gain = 16'd6651;
                8'd102: F_color_gain = 16'd6586;
                8'd103: F_color_gain = 16'd6562;
                8'd104: F_color_gain = 16'd6538;
                8'd105: F_color_gain = 16'd6515;
                8'd106: F_color_gain = 16'd6453;
                8'd107: F_color_gain = 16'd6431;
                8'd108: F_color_gain = 16'd6409;
                8'd109: F_color_gain = 16'd6388;
                8'd110: F_color_gain = 16'd6367;
                8'd111: F_color_gain = 16'd6310;
                8'd112: F_color_gain = 16'd6290;
                8'd113: F_color_gain = 16'd6271;
                8'd114: F_color_gain = 16'd6252;
                8'd115: F_color_gain = 16'd6197;
                8'd116: F_color_gain = 16'd6179;
                8'd117: F_color_gain = 16'd6162;
                8'd118: F_color_gain = 16'd6109;
                8'd119: F_color_gain = 16'd6092;
                8'd120: F_color_gain = 16'd6076;
                8'd121: F_color_gain = 16'd6059;
                8'd122: F_color_gain = 16'd6010;
                8'd123: F_color_gain = 16'd5994;
                8'd124: F_color_gain = 16'd5979;
                8'd125: F_color_gain = 16'd5964;
                8'd126: F_color_gain = 16'd5916;
                8'd127: F_color_gain = 16'd5902;
                8'd128: F_color_gain = 16'd5888;
                8'd129: F_color_gain = 16'd5842;
                8'd130: F_color_gain = 16'd5829;
                8'd131: F_color_gain = 16'd5816;
                8'd132: F_color_gain = 16'd5772;
                8'd133: F_color_gain = 16'd5759;
                8'd134: F_color_gain = 16'd5747;
                8'd135: F_color_gain = 16'd5704;
                8'd136: F_color_gain = 16'd5692;
                8'd137: F_color_gain = 16'd5681;
                8'd138: F_color_gain = 16'd5639;
                8'd139: F_color_gain = 16'd5628;
                8'd140: F_color_gain = 16'd5617;
                8'd141: F_color_gain = 16'd5578;
                8'd142: F_color_gain = 16'd5567;
                8'd143: F_color_gain = 16'd5557;
                8'd144: F_color_gain = 16'd5518;
                8'd145: F_color_gain = 16'd5508;
                8'd146: F_color_gain = 16'd5499;
                8'd147: F_color_gain = 16'd5461;
                8'd148: F_color_gain = 16'd5452;
                8'd149: F_color_gain = 16'd5443;
                8'd150: F_color_gain = 16'd5407;
                8'd151: F_color_gain = 16'd5398;
                8'd152: F_color_gain = 16'd5363;
                8'd153: F_color_gain = 16'd5354;
                8'd154: F_color_gain = 16'd5346;
                8'd155: F_color_gain = 16'd5312;
                8'd156: F_color_gain = 16'd5304;
                8'd157: F_color_gain = 16'd5296;
                8'd158: F_color_gain = 16'd5263;
                8'd159: F_color_gain = 16'd5255;
                8'd160: F_color_gain = 16'd5222;
                8'd161: F_color_gain = 16'd5215;
                8'd162: F_color_gain = 16'd5208;
                8'd163: F_color_gain = 16'd5177;
                8'd164: F_color_gain = 16'd5170;
                8'd165: F_color_gain = 16'd5139;
                8'd166: F_color_gain = 16'd5132;
                8'd167: F_color_gain = 16'd5126;
                8'd168: F_color_gain = 16'd5096;
                8'd169: F_color_gain = 16'd5090;
                8'd170: F_color_gain = 16'd5060;
                8'd171: F_color_gain = 16'd5054;
                8'd172: F_color_gain = 16'd5049;
                8'd173: F_color_gain = 16'd5019;
                8'd174: F_color_gain = 16'd5014;
                8'd175: F_color_gain = 16'd4985;
                8'd176: F_color_gain = 16'd4980;
                8'd177: F_color_gain = 16'd4975;
                8'd178: F_color_gain = 16'd4947;
                8'd179: F_color_gain = 16'd4943;
                8'd180: F_color_gain = 16'd4915;
                8'd181: F_color_gain = 16'd4911;
                8'd182: F_color_gain = 16'd4884;
                8'd183: F_color_gain = 16'd4879;
                8'd184: F_color_gain = 16'd4875;
                8'd185: F_color_gain = 16'd4849;
                8'd186: F_color_gain = 16'd4845;
                8'd187: F_color_gain = 16'd4819;
                8'd188: F_color_gain = 16'd4815;
                8'd189: F_color_gain = 16'd4790;
                8'd190: F_color_gain = 16'd4786;
                8'd191: F_color_gain = 16'd4761;
                8'd192: F_color_gain = 16'd4757;
                8'd193: F_color_gain = 16'd4754;
                8'd194: F_color_gain = 16'd4729;
                8'd195: F_color_gain = 16'd4726;
                8'd196: F_color_gain = 16'd4702;
                8'd197: F_color_gain = 16'd4699;
                8'd198: F_color_gain = 16'd4675;
                8'd199: F_color_gain = 16'd4672;
                8'd200: F_color_gain = 16'd4649;
                8'd201: F_color_gain = 16'd4646;
                8'd202: F_color_gain = 16'd4623;
                8'd203: F_color_gain = 16'd4621;
                8'd204: F_color_gain = 16'd4618;
                8'd205: F_color_gain = 16'd4596;
                8'd206: F_color_gain = 16'd4593;
                8'd207: F_color_gain = 16'd4571;
                8'd208: F_color_gain = 16'd4569;
                8'd209: F_color_gain = 16'd4547;
                8'd210: F_color_gain = 16'd4545;
                8'd211: F_color_gain = 16'd4523;
                8'd212: F_color_gain = 16'd4521;
                8'd213: F_color_gain = 16'd4500;
                8'd214: F_color_gain = 16'd4498;
                8'd215: F_color_gain = 16'd4477;
                8'd216: F_color_gain = 16'd4475;
                8'd217: F_color_gain = 16'd4455;
                8'd218: F_color_gain = 16'd4453;
                8'd219: F_color_gain = 16'd4433;
                8'd220: F_color_gain = 16'd4431;
                8'd221: F_color_gain = 16'd4411;
                8'd222: F_color_gain = 16'd4410;
                8'd223: F_color_gain = 16'd4390;
                8'd224: F_color_gain = 16'd4389;
                8'd225: F_color_gain = 16'd4369;
                8'd226: F_color_gain = 16'd4368;
                8'd227: F_color_gain = 16'd4349;
                8'd228: F_color_gain = 16'd4348;
                8'd229: F_color_gain = 16'd4329;
                8'd230: F_color_gain = 16'd4328;
                8'd231: F_color_gain = 16'd4309;
                8'd232: F_color_gain = 16'd4308;
                8'd233: F_color_gain = 16'd4289;
                8'd234: F_color_gain = 16'd4289;
                8'd235: F_color_gain = 16'd4270;
                8'd236: F_color_gain = 16'd4270;
                8'd237: F_color_gain = 16'd4252;
                8'd238: F_color_gain = 16'd4251;
                8'd239: F_color_gain = 16'd4233;
                8'd240: F_color_gain = 16'd4233;
                8'd241: F_color_gain = 16'd4215;
                8'd242: F_color_gain = 16'd4214;
                8'd243: F_color_gain = 16'd4197;
                8'd244: F_color_gain = 16'd4197;
                8'd245: F_color_gain = 16'd4180;
                8'd246: F_color_gain = 16'd4179;
                8'd247: F_color_gain = 16'd4162;
                8'd248: F_color_gain = 16'd4162;
                8'd249: F_color_gain = 16'd4145;
                8'd250: F_color_gain = 16'd4145;
                8'd251: F_color_gain = 16'd4129;
                8'd252: F_color_gain = 16'd4129;
                8'd253: F_color_gain = 16'd4112;
                8'd254: F_color_gain = 16'd4112;
                8'd255: F_color_gain = 16'd4096;
                default: F_color_gain = 16'd4096;
            endcase
        end
    endfunction

    function [7:0] F_max_rgb;
        input [23:0] rgb;
        reg [7:0] rg;
        begin
            rg=(rgb[23:16]>rgb[15:8]) ? rgb[23:16] : rgb[15:8];
            F_max_rgb=(rg>rgb[7:0]) ? rg : rgb[7:0];
        end
    endfunction

    // Exact v3 integer result for a dominant channel; no curve approximation.
    function [7:0] F_peak_code;
        input [7:0] maximum;
        begin
            case(maximum)
                8'd0: F_peak_code=8'd0;
                8'd1: F_peak_code=8'd4;
                8'd2: F_peak_code=8'd10;
                8'd3: F_peak_code=8'd15;
                8'd4: F_peak_code=8'd20;
                8'd5: F_peak_code=8'd24;
                8'd6: F_peak_code=8'd28;
                8'd7: F_peak_code=8'd32;
                8'd8: F_peak_code=8'd36;
                8'd9: F_peak_code=8'd39;
                8'd10: F_peak_code=8'd42;
                8'd11: F_peak_code=8'd46;
                8'd12: F_peak_code=8'd48;
                8'd13: F_peak_code=8'd51;
                8'd14: F_peak_code=8'd54;
                8'd15: F_peak_code=8'd56;
                8'd16: F_peak_code=8'd59;
                8'd17: F_peak_code=8'd61;
                8'd18: F_peak_code=8'd64;
                8'd19: F_peak_code=8'd66;
                8'd20: F_peak_code=8'd68;
                8'd21: F_peak_code=8'd70;
                8'd22: F_peak_code=8'd72;
                8'd23: F_peak_code=8'd74;
                8'd24: F_peak_code=8'd76;
                8'd25: F_peak_code=8'd78;
                8'd26: F_peak_code=8'd80;
                8'd27: F_peak_code=8'd81;
                8'd28: F_peak_code=8'd83;
                8'd29: F_peak_code=8'd85;
                8'd30: F_peak_code=8'd86;
                8'd31: F_peak_code=8'd88;
                8'd32: F_peak_code=8'd90;
                8'd33: F_peak_code=8'd91;
                8'd34: F_peak_code=8'd93;
                8'd35: F_peak_code=8'd94;
                8'd36: F_peak_code=8'd96;
                8'd37: F_peak_code=8'd97;
                8'd38: F_peak_code=8'd99;
                8'd39: F_peak_code=8'd100;
                8'd40: F_peak_code=8'd101;
                8'd41: F_peak_code=8'd103;
                8'd42: F_peak_code=8'd104;
                8'd43: F_peak_code=8'd106;
                8'd44: F_peak_code=8'd107;
                8'd45: F_peak_code=8'd108;
                8'd46: F_peak_code=8'd109;
                8'd47: F_peak_code=8'd111;
                8'd48: F_peak_code=8'd112;
                8'd49: F_peak_code=8'd113;
                8'd50: F_peak_code=8'd114;
                8'd51: F_peak_code=8'd116;
                8'd52: F_peak_code=8'd117;
                8'd53: F_peak_code=8'd118;
                8'd54: F_peak_code=8'd119;
                8'd55: F_peak_code=8'd120;
                8'd56: F_peak_code=8'd121;
                8'd57: F_peak_code=8'd122;
                8'd58: F_peak_code=8'd124;
                8'd59: F_peak_code=8'd125;
                8'd60: F_peak_code=8'd126;
                8'd61: F_peak_code=8'd127;
                8'd62: F_peak_code=8'd128;
                8'd63: F_peak_code=8'd129;
                8'd64: F_peak_code=8'd130;
                8'd65: F_peak_code=8'd131;
                8'd66: F_peak_code=8'd132;
                8'd67: F_peak_code=8'd133;
                8'd68: F_peak_code=8'd134;
                8'd69: F_peak_code=8'd135;
                8'd70: F_peak_code=8'd136;
                8'd71: F_peak_code=8'd137;
                8'd72: F_peak_code=8'd138;
                8'd73: F_peak_code=8'd139;
                8'd74: F_peak_code=8'd140;
                8'd75: F_peak_code=8'd141;
                8'd76: F_peak_code=8'd142;
                8'd77: F_peak_code=8'd143;
                8'd78: F_peak_code=8'd144;
                8'd79: F_peak_code=8'd145;
                8'd80: F_peak_code=8'd146;
                8'd81: F_peak_code=8'd147;
                8'd82: F_peak_code=8'd147;
                8'd83: F_peak_code=8'd148;
                8'd84: F_peak_code=8'd149;
                8'd85: F_peak_code=8'd150;
                8'd86: F_peak_code=8'd151;
                8'd87: F_peak_code=8'd152;
                8'd88: F_peak_code=8'd153;
                8'd89: F_peak_code=8'd154;
                8'd90: F_peak_code=8'd154;
                8'd91: F_peak_code=8'd155;
                8'd92: F_peak_code=8'd156;
                8'd93: F_peak_code=8'd157;
                8'd94: F_peak_code=8'd158;
                8'd95: F_peak_code=8'd159;
                8'd96: F_peak_code=8'd159;
                8'd97: F_peak_code=8'd160;
                8'd98: F_peak_code=8'd161;
                8'd99: F_peak_code=8'd162;
                8'd100: F_peak_code=8'd163;
                8'd101: F_peak_code=8'd164;
                8'd102: F_peak_code=8'd164;
                8'd103: F_peak_code=8'd165;
                8'd104: F_peak_code=8'd166;
                8'd105: F_peak_code=8'd167;
                8'd106: F_peak_code=8'd167;
                8'd107: F_peak_code=8'd168;
                8'd108: F_peak_code=8'd169;
                8'd109: F_peak_code=8'd170;
                8'd110: F_peak_code=8'd171;
                8'd111: F_peak_code=8'd171;
                8'd112: F_peak_code=8'd172;
                8'd113: F_peak_code=8'd173;
                8'd114: F_peak_code=8'd174;
                8'd115: F_peak_code=8'd174;
                8'd116: F_peak_code=8'd175;
                8'd117: F_peak_code=8'd176;
                8'd118: F_peak_code=8'd176;
                8'd119: F_peak_code=8'd177;
                8'd120: F_peak_code=8'd178;
                8'd121: F_peak_code=8'd179;
                8'd122: F_peak_code=8'd179;
                8'd123: F_peak_code=8'd180;
                8'd124: F_peak_code=8'd181;
                8'd125: F_peak_code=8'd182;
                8'd126: F_peak_code=8'd182;
                8'd127: F_peak_code=8'd183;
                8'd128: F_peak_code=8'd184;
                8'd129: F_peak_code=8'd184;
                8'd130: F_peak_code=8'd185;
                8'd131: F_peak_code=8'd186;
                8'd132: F_peak_code=8'd186;
                8'd133: F_peak_code=8'd187;
                8'd134: F_peak_code=8'd188;
                8'd135: F_peak_code=8'd188;
                8'd136: F_peak_code=8'd189;
                8'd137: F_peak_code=8'd190;
                8'd138: F_peak_code=8'd190;
                8'd139: F_peak_code=8'd191;
                8'd140: F_peak_code=8'd192;
                8'd141: F_peak_code=8'd192;
                8'd142: F_peak_code=8'd193;
                8'd143: F_peak_code=8'd194;
                8'd144: F_peak_code=8'd194;
                8'd145: F_peak_code=8'd195;
                8'd146: F_peak_code=8'd196;
                8'd147: F_peak_code=8'd196;
                8'd148: F_peak_code=8'd197;
                8'd149: F_peak_code=8'd198;
                8'd150: F_peak_code=8'd198;
                8'd151: F_peak_code=8'd199;
                8'd152: F_peak_code=8'd199;
                8'd153: F_peak_code=8'd200;
                8'd154: F_peak_code=8'd201;
                8'd155: F_peak_code=8'd201;
                8'd156: F_peak_code=8'd202;
                8'd157: F_peak_code=8'd203;
                8'd158: F_peak_code=8'd203;
                8'd159: F_peak_code=8'd204;
                8'd160: F_peak_code=8'd204;
                8'd161: F_peak_code=8'd205;
                8'd162: F_peak_code=8'd206;
                8'd163: F_peak_code=8'd206;
                8'd164: F_peak_code=8'd207;
                8'd165: F_peak_code=8'd207;
                8'd166: F_peak_code=8'd208;
                8'd167: F_peak_code=8'd209;
                8'd168: F_peak_code=8'd209;
                8'd169: F_peak_code=8'd210;
                8'd170: F_peak_code=8'd210;
                8'd171: F_peak_code=8'd211;
                8'd172: F_peak_code=8'd212;
                8'd173: F_peak_code=8'd212;
                8'd174: F_peak_code=8'd213;
                8'd175: F_peak_code=8'd213;
                8'd176: F_peak_code=8'd214;
                8'd177: F_peak_code=8'd215;
                8'd178: F_peak_code=8'd215;
                8'd179: F_peak_code=8'd216;
                8'd180: F_peak_code=8'd216;
                8'd181: F_peak_code=8'd217;
                8'd182: F_peak_code=8'd217;
                8'd183: F_peak_code=8'd218;
                8'd184: F_peak_code=8'd219;
                8'd185: F_peak_code=8'd219;
                8'd186: F_peak_code=8'd220;
                8'd187: F_peak_code=8'd220;
                8'd188: F_peak_code=8'd221;
                8'd189: F_peak_code=8'd221;
                8'd190: F_peak_code=8'd222;
                8'd191: F_peak_code=8'd222;
                8'd192: F_peak_code=8'd223;
                8'd193: F_peak_code=8'd224;
                8'd194: F_peak_code=8'd224;
                8'd195: F_peak_code=8'd225;
                8'd196: F_peak_code=8'd225;
                8'd197: F_peak_code=8'd226;
                8'd198: F_peak_code=8'd226;
                8'd199: F_peak_code=8'd227;
                8'd200: F_peak_code=8'd227;
                8'd201: F_peak_code=8'd228;
                8'd202: F_peak_code=8'd228;
                8'd203: F_peak_code=8'd229;
                8'd204: F_peak_code=8'd230;
                8'd205: F_peak_code=8'd230;
                8'd206: F_peak_code=8'd231;
                8'd207: F_peak_code=8'd231;
                8'd208: F_peak_code=8'd232;
                8'd209: F_peak_code=8'd232;
                8'd210: F_peak_code=8'd233;
                8'd211: F_peak_code=8'd233;
                8'd212: F_peak_code=8'd234;
                8'd213: F_peak_code=8'd234;
                8'd214: F_peak_code=8'd235;
                8'd215: F_peak_code=8'd235;
                8'd216: F_peak_code=8'd236;
                8'd217: F_peak_code=8'd236;
                8'd218: F_peak_code=8'd237;
                8'd219: F_peak_code=8'd237;
                8'd220: F_peak_code=8'd238;
                8'd221: F_peak_code=8'd238;
                8'd222: F_peak_code=8'd239;
                8'd223: F_peak_code=8'd239;
                8'd224: F_peak_code=8'd240;
                8'd225: F_peak_code=8'd240;
                8'd226: F_peak_code=8'd241;
                8'd227: F_peak_code=8'd241;
                8'd228: F_peak_code=8'd242;
                8'd229: F_peak_code=8'd242;
                8'd230: F_peak_code=8'd243;
                8'd231: F_peak_code=8'd243;
                8'd232: F_peak_code=8'd244;
                8'd233: F_peak_code=8'd244;
                8'd234: F_peak_code=8'd245;
                8'd235: F_peak_code=8'd245;
                8'd236: F_peak_code=8'd246;
                8'd237: F_peak_code=8'd246;
                8'd238: F_peak_code=8'd247;
                8'd239: F_peak_code=8'd247;
                8'd240: F_peak_code=8'd248;
                8'd241: F_peak_code=8'd248;
                8'd242: F_peak_code=8'd249;
                8'd243: F_peak_code=8'd249;
                8'd244: F_peak_code=8'd250;
                8'd245: F_peak_code=8'd250;
                8'd246: F_peak_code=8'd251;
                8'd247: F_peak_code=8'd251;
                8'd248: F_peak_code=8'd252;
                8'd249: F_peak_code=8'd252;
                8'd250: F_peak_code=8'd253;
                8'd251: F_peak_code=8'd253;
                8'd252: F_peak_code=8'd254;
                8'd253: F_peak_code=8'd254;
                8'd254: F_peak_code=8'd255;
                8'd255: F_peak_code=8'd255;
                default: F_peak_code=8'd0;
            endcase
        end
    endfunction

    function [7:0] F_scaled;
        input [23:0] product;
        reg [24:0] rounded;
        begin
            rounded={1'b0,product}+25'd2048;
            F_scaled=(|rounded[24:20]) ? 8'd255 : rounded[19:12];
        end
    endfunction

    function [23:0] F_display_rgb;
        input [23:0] rgb;
        begin
            F_display_rgb = (DISPLAY_GAMMA_ENABLE && !DISPLAY_COLOR_TONE_ENABLE) ?
                {F_gamma8(rgb[23:16]), F_gamma8(rgb[15:8]), F_gamma8(rgb[7:0])} : rgb;
        end
    endfunction

    // 单像素风格输出
    function [23:0] F_style_px;
        input [2:0]  s;
        input [23:0] rgb;
        input [23:0] bin;
        input [7:0]  y;
        input        is_edge;
        input        m;
        reg [23:0] o;
        begin
            case (s)
                3'd0: o = F_display_rgb(rgb);
                3'd1: o = bin;
                3'd2: o = is_edge ? 24'h000000 : 24'hFFFFFF;
                3'd3: o = is_edge ? 24'h000000 : F_cartoon(rgb);
                3'd4: o = F_heat(y);
                3'd5: o = F_hl(rgb, y, m);
                default: o = rgb;
            endcase
            F_style_px = o;
        end
    endfunction

    wire [23:0] b0 = I_bin[95:72];
    wire [23:0] b1 = I_bin[71:48];
    wire [23:0] b2 = I_bin[47:24];
    wire [23:0] b3 = I_bin[23: 0];

    // Same two stages as v3. Ties select R, then G, then B.
    // The dominant channel is a 256x8 exact ROM; only the other two multiply.
    reg valid_s1;
    reg [2:0] mode_s1;
    reg [95:0] style_s1;
    wire [15:0] gain_s1[0:3];
    reg [7:0] minor_a_s1[0:3],minor_b_s1[0:3];
    wire [7:0] peak_s1[0:3];
    reg [1:0] dominant_s1[0:3];
    wire [23:0] product_a[0:3],product_b[0:3];
    genvar k;
    generate for(k=0;k<4;k=k+1) begin: display_multiply
        // Exact gain and peak share a synchronous ROM address. Reset and
        // validity stay on the control path, preserving the two-stage timing.
        wire [23:0] curve_q;
        display_tone_curve_rom curve_rom(I_clk,F_max_rgb(I_rgb[k*24 +:24]),curve_q);
        assign peak_s1[k]=curve_q[23:16];
        assign gain_s1[k]=curve_q[15:0];
        assign product_a[k]=minor_a_s1[k]*gain_s1[k];
        assign product_b[k]=minor_b_s1[k]*gain_s1[k];
    end endgenerate
    integer j;
    reg [23:0] pixel_now;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_tdata<=0;valid_s1<=0;mode_s1<=0;style_s1<=0;
            for(j=0;j<4;j=j+1) begin
                minor_a_s1[j]<=0;minor_b_s1[j]<=0;dominant_s1[j]<=0;
            end
        end else begin
            valid_s1<=I_tvalid;
            if(I_tvalid) begin
                mode_s1<=I_style;
                for(j=0;j<4;j=j+1) begin
                    pixel_now=I_rgb[j*24 +:24];
                    if(pixel_now[23:16]>=pixel_now[15:8] && pixel_now[23:16]>=pixel_now[7:0]) begin
                        dominant_s1[j]<=0;minor_a_s1[j]<=pixel_now[15:8];minor_b_s1[j]<=pixel_now[7:0];
                    end else if(pixel_now[15:8]>=pixel_now[7:0]) begin
                        dominant_s1[j]<=1;minor_a_s1[j]<=pixel_now[23:16];minor_b_s1[j]<=pixel_now[7:0];
                    end else begin
                        dominant_s1[j]<=2;minor_a_s1[j]<=pixel_now[23:16];minor_b_s1[j]<=pixel_now[15:8];
                    end
                end
                style_s1 <= { F_style_px(I_style,p0,b0,y0,e0,I_mask[3]),
                              F_style_px(I_style,p1,b1,y1,e1,I_mask[2]),
                              F_style_px(I_style,p2,b2,y2,e2,I_mask[1]),
                              F_style_px(I_style,p3,b3,y3,e3,I_mask[0]) };
            end
            if(valid_s1) begin
                if(mode_s1==0 && DISPLAY_GAMMA_ENABLE && DISPLAY_COLOR_TONE_ENABLE) begin
                    for(j=0;j<4;j=j+1) case(dominant_s1[j])
                        0: O_tdata[j*24 +:24]<={peak_s1[j],F_scaled(product_a[j]),F_scaled(product_b[j])};
                        1: O_tdata[j*24 +:24]<={F_scaled(product_a[j]),peak_s1[j],F_scaled(product_b[j])};
                        default: O_tdata[j*24 +:24]<={F_scaled(product_a[j]),F_scaled(product_b[j]),peak_s1[j]};
                    endcase
                end else O_tdata<=style_s1;
            end
        end
    end
endmodule

// Exact packed {peak8,gainQ12[15:0]} table from F_peak_code/F_color_gain.
// Separate hierarchy avoids TD's generate-local ROM elaboration warning.
module display_tone_curve_rom(input wire clk,input wire [7:0] maximum,
 output reg [23:0] q);
    (* rom_style="bram" *) reg [23:0] curve[0:255];
        initial begin
            curve[0]=24'h000000;
            curve[1]=24'h044000;
            curve[2]=24'h0a5000;
            curve[3]=24'h0f5000;
            curve[4]=24'h145000;
            curve[5]=24'h184ccd;
            curve[6]=24'h1c4aab;
            curve[7]=24'h204925;
            curve[8]=24'h244800;
            curve[9]=24'h274555;
            curve[10]=24'h2a4333;
            curve[11]=24'h2e42e9;
            curve[12]=24'h304000;
            curve[13]=24'h333ec5;
            curve[14]=24'h363db7;
            curve[15]=24'h383bbc;
            curve[16]=24'h3b3b00;
            curve[17]=24'h3d3969;
            curve[18]=24'h4038e4;
            curve[19]=24'h423794;
            curve[20]=24'h443666;
            curve[21]=24'h463555;
            curve[22]=24'h48345d;
            curve[23]=24'h4a337a;
            curve[24]=24'h4c32ab;
            curve[25]=24'h4e31ec;
            curve[26]=24'h50313b;
            curve[27]=24'h513000;
            curve[28]=24'h532f6e;
            curve[29]=24'h552ee6;
            curve[30]=24'h562dde;
            curve[31]=24'h582d6b;
            curve[32]=24'h5a2d00;
            curve[33]=24'h5b2c1f;
            curve[34]=24'h5d2bc4;
            curve[35]=24'h5e2af9;
            curve[36]=24'h602aab;
            curve[37]=24'h6129f2;
            curve[38]=24'h6329af;
            curve[39]=24'h642907;
            curve[40]=24'h652866;
            curve[41]=24'h672832;
            curve[42]=24'h68279e;
            curve[43]=24'h6a2771;
            curve[44]=24'h6b26e9;
            curve[45]=24'h6c2666;
            curve[46]=24'h6d25ea;
            curve[47]=24'h6f25ca;
            curve[48]=24'h702555;
            curve[49]=24'h7124e6;
            curve[50]=24'h72247b;
            curve[51]=24'h742464;
            curve[52]=24'h752400;
            curve[53]=24'h76239f;
            curve[54]=24'h772342;
            curve[55]=24'h7822e9;
            curve[56]=24'h792292;
            curve[57]=24'h7a223f;
            curve[58]=24'h7c2235;
            curve[59]=24'h7d21e6;
            curve[60]=24'h7e219a;
            curve[61]=24'h7f2150;
            curve[62]=24'h802108;
            curve[63]=24'h8120c3;
            curve[64]=24'h822080;
            curve[65]=24'h83203f;
            curve[66]=24'h842000;
            curve[67]=24'h851fc3;
            curve[68]=24'h861f88;
            curve[69]=24'h871f4e;
            curve[70]=24'h881f16;
            curve[71]=24'h891ee0;
            curve[72]=24'h8a1eab;
            curve[73]=24'h8b1e77;
            curve[74]=24'h8c1e45;
            curve[75]=24'h8d1e14;
            curve[76]=24'h8e1de5;
            curve[77]=24'h8f1db7;
            curve[78]=24'h901d8a;
            curve[79]=24'h911d5e;
            curve[80]=24'h921d33;
            curve[81]=24'h931d09;
            curve[82]=24'h931caf;
            curve[83]=24'h941c88;
            curve[84]=24'h951c62;
            curve[85]=24'h961c3c;
            curve[86]=24'h971c18;
            curve[87]=24'h981bf4;
            curve[88]=24'h991bd1;
            curve[89]=24'h9a1baf;
            curve[90]=24'h9a1b61;
            curve[91]=24'h9b1b41;
            curve[92]=24'h9c1b21;
            curve[93]=24'h9d1b03;
            curve[94]=24'h9e1ae5;
            curve[95]=24'h9f1ac7;
            curve[96]=24'h9f1a80;
            curve[97]=24'ha01a64;
            curve[98]=24'ha11a49;
            curve[99]=24'ha21a2f;
            curve[100]=24'ha31a14;
            curve[101]=24'ha419fb;
            curve[102]=24'ha419ba;
            curve[103]=24'ha519a2;
            curve[104]=24'ha6198a;
            curve[105]=24'ha71973;
            curve[106]=24'ha71935;
            curve[107]=24'ha8191f;
            curve[108]=24'ha91909;
            curve[109]=24'haa18f4;
            curve[110]=24'hab18df;
            curve[111]=24'hab18a6;
            curve[112]=24'hac1892;
            curve[113]=24'had187f;
            curve[114]=24'hae186c;
            curve[115]=24'hae1835;
            curve[116]=24'haf1823;
            curve[117]=24'hb01812;
            curve[118]=24'hb017dd;
            curve[119]=24'hb117cc;
            curve[120]=24'hb217bc;
            curve[121]=24'hb317ab;
            curve[122]=24'hb3177a;
            curve[123]=24'hb4176a;
            curve[124]=24'hb5175b;
            curve[125]=24'hb6174c;
            curve[126]=24'hb6171c;
            curve[127]=24'hb7170e;
            curve[128]=24'hb81700;
            curve[129]=24'hb816d2;
            curve[130]=24'hb916c5;
            curve[131]=24'hba16b8;
            curve[132]=24'hba168c;
            curve[133]=24'hbb167f;
            curve[134]=24'hbc1673;
            curve[135]=24'hbc1648;
            curve[136]=24'hbd163c;
            curve[137]=24'hbe1631;
            curve[138]=24'hbe1607;
            curve[139]=24'hbf15fc;
            curve[140]=24'hc015f1;
            curve[141]=24'hc015ca;
            curve[142]=24'hc115bf;
            curve[143]=24'hc215b5;
            curve[144]=24'hc2158e;
            curve[145]=24'hc31584;
            curve[146]=24'hc4157b;
            curve[147]=24'hc41555;
            curve[148]=24'hc5154c;
            curve[149]=24'hc61543;
            curve[150]=24'hc6151f;
            curve[151]=24'hc71516;
            curve[152]=24'hc714f3;
            curve[153]=24'hc814ea;
            curve[154]=24'hc914e2;
            curve[155]=24'hc914c0;
            curve[156]=24'hca14b8;
            curve[157]=24'hcb14b0;
            curve[158]=24'hcb148f;
            curve[159]=24'hcc1487;
            curve[160]=24'hcc1466;
            curve[161]=24'hcd145f;
            curve[162]=24'hce1458;
            curve[163]=24'hce1439;
            curve[164]=24'hcf1432;
            curve[165]=24'hcf1413;
            curve[166]=24'hd0140c;
            curve[167]=24'hd11406;
            curve[168]=24'hd113e8;
            curve[169]=24'hd213e2;
            curve[170]=24'hd213c4;
            curve[171]=24'hd313be;
            curve[172]=24'hd413b9;
            curve[173]=24'hd4139b;
            curve[174]=24'hd51396;
            curve[175]=24'hd51379;
            curve[176]=24'hd61374;
            curve[177]=24'hd7136f;
            curve[178]=24'hd71353;
            curve[179]=24'hd8134f;
            curve[180]=24'hd81333;
            curve[181]=24'hd9132f;
            curve[182]=24'hd91314;
            curve[183]=24'hda130f;
            curve[184]=24'hdb130b;
            curve[185]=24'hdb12f1;
            curve[186]=24'hdc12ed;
            curve[187]=24'hdc12d3;
            curve[188]=24'hdd12cf;
            curve[189]=24'hdd12b6;
            curve[190]=24'hde12b2;
            curve[191]=24'hde1299;
            curve[192]=24'hdf1295;
            curve[193]=24'he01292;
            curve[194]=24'he01279;
            curve[195]=24'he11276;
            curve[196]=24'he1125e;
            curve[197]=24'he2125b;
            curve[198]=24'he21243;
            curve[199]=24'he31240;
            curve[200]=24'he31229;
            curve[201]=24'he41226;
            curve[202]=24'he4120f;
            curve[203]=24'he5120d;
            curve[204]=24'he6120a;
            curve[205]=24'he611f4;
            curve[206]=24'he711f1;
            curve[207]=24'he711db;
            curve[208]=24'he811d9;
            curve[209]=24'he811c3;
            curve[210]=24'he911c1;
            curve[211]=24'he911ab;
            curve[212]=24'hea11a9;
            curve[213]=24'hea1194;
            curve[214]=24'heb1192;
            curve[215]=24'heb117d;
            curve[216]=24'hec117b;
            curve[217]=24'hec1167;
            curve[218]=24'hed1165;
            curve[219]=24'hed1151;
            curve[220]=24'hee114f;
            curve[221]=24'hee113b;
            curve[222]=24'hef113a;
            curve[223]=24'hef1126;
            curve[224]=24'hf01125;
            curve[225]=24'hf01111;
            curve[226]=24'hf11110;
            curve[227]=24'hf110fd;
            curve[228]=24'hf210fc;
            curve[229]=24'hf210e9;
            curve[230]=24'hf310e8;
            curve[231]=24'hf310d5;
            curve[232]=24'hf410d4;
            curve[233]=24'hf410c1;
            curve[234]=24'hf510c1;
            curve[235]=24'hf510ae;
            curve[236]=24'hf610ae;
            curve[237]=24'hf6109c;
            curve[238]=24'hf7109b;
            curve[239]=24'hf71089;
            curve[240]=24'hf81089;
            curve[241]=24'hf81077;
            curve[242]=24'hf91076;
            curve[243]=24'hf91065;
            curve[244]=24'hfa1065;
            curve[245]=24'hfa1054;
            curve[246]=24'hfb1053;
            curve[247]=24'hfb1042;
            curve[248]=24'hfc1042;
            curve[249]=24'hfc1031;
            curve[250]=24'hfd1031;
            curve[251]=24'hfd1021;
            curve[252]=24'hfe1021;
            curve[253]=24'hfe1010;
            curve[254]=24'hff1010;
            curve[255]=24'hff1000;
        end
    always @(posedge clk) q<=curve[maximum];
endmodule
