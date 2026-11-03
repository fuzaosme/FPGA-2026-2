// ===========================================================================
// uart_report.v
// 把跟踪结果及低功耗诊断打包成文本，通过 UART 周期发送到上位机。
// 格式（共 62 字节，含行尾 CRLF）：
//   $T,<id>,<type>,<x3hex>,<y3hex>,<红>,<绿>,<黄>,<圆>,<置信度>,
//     <蓝牙命令2hex>,K<跳过帧数4hex>,
//     <R2><r2>,<G2><g2>,<Y2><y2>,<B2><b2>,<b2>,<m2>\r\n
//   type 字符：R=红灯 Y=黄灯 G=绿灯 ?=其它
//   x/y 为 3 位十六进制（省去二进制转十进制）
//   阈值那 9 个字段各 2 位十六进制（与 thr_regs 的 R/r/G/g/Y/y/B/b/m 一一对应）
//   K = 批 3 低功耗降频累计跳过的帧数，数字不再增长说明已经回到全速
//   独立诊断行（12 字节，含 CRLF）：$L,<LP状态>,K<跳过帧数4hex>\r\n
//   独立 $L 行每个发送周期都会发；即使当前没有交通灯目标也能查看 LP/K。
// ===========================================================================
`timescale 1ns/1ps

module uart_report #(
    parameter ARROW_FEATURE=0,
    parameter DETECT_FEATURE=0,
    parameter CAMERA_FEATURE=0,
    parameter CLK_FREQ    = 50000000,
    parameter SEND_PERIOD = 25000000    // 0.5s 发一次（50MHz 计数值）
)(
    input wire I_arrow_valid,input wire [191:0] I_arrow_snapshot,
    input wire [39:0] I_detect_status,
    input wire [351:0] I_camera_snapshot,
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_valid,        // 是否有跟踪目标
    input  wire [1:0]  I_id,
    input  wire [2:0]  I_type,         // 0红物 1蓝牌 2黄牌 3绿牌 4红灯 5黄灯 6绿灯
    input  wire [11:0] I_x,
    input  wire [11:0] I_y,
    input  wire [11:0] I_mt_num,   // {黄,绿,红} 各 4bit
    input  wire [3:0]  I_circ,     // 圆形目标个数
    input  wire [3:0]  I_conf,     // 置信度 0~9
    input  wire [7:0]  I_bt_cmd,   // 最后发给小车的命令字节（0=还没发过）
    input  wire [15:0] I_skip_cnt, // 批 3：低功耗降频累计跳过的帧数
    input  wire        I_lp_active,// 当前是否处于低功耗降频模式
    // 当前 9 个颜色阈值的副本（与 thr_regs 同域，不用跨时钟域）
    // [63:56]R [55:48]r [47:40]G [39:32]g [31:24]Y [23:16]y [15:8]B [7:4]b [3:0]m
    input  wire [63:0] I_thr,
    output wire        O_uart_tx
);
    wire        tx_busy;
    reg         tx_start;
    reg  [7:0]  tx_byte;

    uart_tx #(.CLK_FREQ(CLK_FREQ)) u_tx (
        .I_clk  (I_clk),
        .I_rst_n(I_rst_n),
        .I_start(tx_start),
        .I_data (tx_byte),
        .O_busy (tx_busy),
        .O_tx   (O_uart_tx)
    );

    // ---- 周期触发 ----
    reg [25:0] tick_cnt;
    reg        tick;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            tick_cnt <= 26'd0;
            tick     <= 1'b0;
        end
        else begin
            tick <= 1'b0;
            if (tick_cnt >= SEND_PERIOD - 1) begin
                tick_cnt <= 26'd0;
                tick     <= 1'b1;
            end
            else begin
                tick_cnt <= tick_cnt + 26'd1;
            end
        end
    end

    // ---- 十六进制字符 ----
    function [7:0] F_hex;
        input [3:0] h;
        begin
            F_hex = (h < 4'd10) ? (8'h30 + h) : (8'h37 + h);
        end
    endfunction

    // ---- 类型字符 ----
    function [7:0] F_type;
        input [2:0] t;
        begin
            case (t)
                3'd4: F_type = 8'h52;  // R
                3'd5: F_type = 8'h59;  // Y
                3'd6: F_type = 8'h47;  // G
                default: F_type = 8'h3F; // ?
            endcase
        end
    endfunction

    reg [1:0] q_I_id;wire [1:0] r_I_id=ARROW_FEATURE?q_I_id:I_id;
    reg [2:0] q_I_type;wire [2:0] r_I_type=ARROW_FEATURE?q_I_type:I_type;
    reg [11:0] q_I_x;wire [11:0] r_I_x=ARROW_FEATURE?q_I_x:I_x;
    reg [11:0] q_I_y;wire [11:0] r_I_y=ARROW_FEATURE?q_I_y:I_y;
    reg [11:0] q_I_mt_num;wire [11:0] r_I_mt_num=ARROW_FEATURE?q_I_mt_num:I_mt_num;
    reg [3:0] q_I_circ;wire [3:0] r_I_circ=ARROW_FEATURE?q_I_circ:I_circ;
    reg [3:0] q_I_conf;wire [3:0] r_I_conf=ARROW_FEATURE?q_I_conf:I_conf;
    reg [7:0] q_I_bt_cmd;wire [7:0] r_I_bt_cmd=ARROW_FEATURE?q_I_bt_cmd:I_bt_cmd;
    reg [63:0] q_I_thr;wire [63:0] r_I_thr=ARROW_FEATURE?q_I_thr:I_thr;
    // ---- 当前字节 ----
    // $T 消息 62 字节（idx 0..61），索引必须 6 位。
    reg [6:0] idx;
    reg report_v;
    reg report_status,spending;reg report_camera,ppending;reg [351:0] camera_snapshot;reg [15:0] camera_crc;reg [39:0] status_snapshot;reg [15:0] status_crc;
    wire traffic_active=!DETECT_FEATURE || I_detect_status[7];
    reg       report_lp;
    reg       report_has_target;
    reg       report_lp_active;
    reg [15:0] report_skip_cnt;

    localparam [1:0] IDLE = 2'd0, SENT = 2'd1, DONE = 2'd2;
    reg [1:0] state;
    reg [191:0] latest,vsnap;
    reg [31:0] vcounter,mscounter,seq,vseq,last_lamp,last_sign;
    reg [15:0] lamp_age,sign_age,vlage,vsage,vcrc;
    reg vpending,seen;
    wire take_s=state==IDLE && !tick && spending && DETECT_FEATURE;
    wire take_v=state==IDLE && !tick && !take_s && vpending && ARROW_FEATURE;
    wire take_p=state==IDLE && !tick && !take_s && !take_v && ppending && CAMERA_FEATURE;
    function [15:0] crcbyte;
      input [15:0] c;input [7:0] b;reg [15:0] t;integer i;
      begin t=c^{b,8'd0};for(i=0;i<8;i=i+1)t=t[15]?(t<<1)^16'h1021:t<<1;crcbyte=t;end
    endfunction
    function [7:0] lampchar;input [1:0] c;input valid,conflict;
      begin if(conflict) lampchar="U";else if(!valid) lampchar="N";
       else case(c) 0:lampchar="R";1:lampchar="Y";2:lampchar="G";default:lampchar="U";endcase end
    endfunction
    function [7:0] dirchar;input [1:0] d;
      begin case(d) 0:dirchar="N";1:dirchar="L";2:dirchar="R";default:dirchar="U";endcase end
    endfunction
    always @(posedge I_clk or negedge I_rst_n) begin
      if(!I_rst_n) begin spending<=0;ppending<=0;latest<=0;vcounter<=0;mscounter<=0;vpending<=0;seen<=0;lamp_age<=16'hFFFF;sign_age<=16'hFFFF;last_lamp<=0;last_sign<=0;end
      else begin
       if(vcounter>=CLK_FREQ/10-1) begin vcounter<=0;vpending<=1;spending<=1;ppending<=1;end else vcounter<=vcounter+1'b1;
       if(take_v) vpending<=0;
       if(take_s)spending<=0;
       if(take_p)ppending<=0;
       if(mscounter>=CLK_FREQ/1000-1) begin
        mscounter<=0;
        if(lamp_age!=16'hFFFF) lamp_age<=lamp_age+1'b1;
        if(sign_age!=16'hFFFF) sign_age<=sign_age+1'b1;
       end else mscounter<=mscounter+1'b1;
       if(I_arrow_valid && ARROW_FEATURE) begin
        latest<=I_arrow_snapshot;seen<=1;
        if(!seen || I_arrow_snapshot[159:128]!=last_lamp || I_arrow_snapshot[191:160]!=latest[191:160]) begin
         lamp_age<=16'd34;last_lamp<=I_arrow_snapshot[159:128];
        end
        if(!seen || I_arrow_snapshot[127:96]!=last_sign || I_arrow_snapshot[191:160]!=latest[191:160]) begin
         sign_age<=16'd34;last_sign<=I_arrow_snapshot[127:96];
        end
       end
      end
    end

    wire [15:0] selected_crc=report_camera ? camera_crc : report_status ? status_crc : vcrc;
    wire [15:0] next_crc=crcbyte(selected_crc,cur_byte);
    reg [7:0] cur_byte;
    reg [3:0] cur_nibble;
    reg cur_is_hex;
    always @(*) begin
        cur_byte=8'h0a;cur_nibble=4'd0;cur_is_hex=1'b0;
        if(report_camera)begin
            case(idx)
             0:cur_byte="$";1:cur_byte="P";2:cur_byte=",";3:cur_byte="1";4:cur_byte=",";
             93:cur_byte="*";94:begin cur_nibble=camera_crc[15:12];cur_is_hex=1'b1;end 95:begin cur_nibble=camera_crc[11:8];cur_is_hex=1'b1;end
             96:begin cur_nibble=camera_crc[7:4];cur_is_hex=1'b1;end 97:begin cur_nibble=camera_crc[3:0];cur_is_hex=1'b1;end 98:cur_byte=13;99:cur_byte=10;
             default:begin
              cur_byte=10;
              if(idx>=5&&idx<=92)begin cur_nibble=camera_snapshot[351:348];cur_is_hex=1'b1;end
             end
            endcase
        end else if(report_status)begin
            case(idx)
            7'd0:cur_byte=8'h24;
            7'd1:cur_byte="S";
            7'd2:cur_byte=",";
            7'd3:cur_byte="1";
            7'd4:cur_byte=",";
            7'd5:begin cur_nibble=status_snapshot[39 -:4];cur_is_hex=1'b1;end
            7'd6:begin cur_nibble=status_snapshot[35 -:4];cur_is_hex=1'b1;end
            7'd7:begin cur_nibble=status_snapshot[31 -:4];cur_is_hex=1'b1;end
            7'd8:begin cur_nibble=status_snapshot[27 -:4];cur_is_hex=1'b1;end
            7'd9:begin cur_nibble=status_snapshot[23 -:4];cur_is_hex=1'b1;end
            7'd10:begin cur_nibble=status_snapshot[19 -:4];cur_is_hex=1'b1;end
            7'd11:begin cur_nibble=status_snapshot[15 -:4];cur_is_hex=1'b1;end
            7'd12:begin cur_nibble=status_snapshot[11 -:4];cur_is_hex=1'b1;end
            7'd13:cur_byte=",";
            7'd14:cur_byte=(status_snapshot[7]?8'h31:8'h30);
            7'd15:cur_byte=",";
            7'd16:cur_byte=8'h30+{6'd0,status_snapshot[6:5]};
            7'd17:cur_byte=",";
            7'd18:cur_byte=(status_snapshot[4]?8'h31:8'h30);
            7'd19:cur_byte=",";
            7'd20:begin cur_nibble=status_snapshot[3:0];cur_is_hex=1'b1;end
            7'd21:cur_byte="*";
            7'd22:begin cur_nibble=status_crc[15:12];cur_is_hex=1'b1;end
            7'd23:begin cur_nibble=status_crc[11:8];cur_is_hex=1'b1;end
            7'd24:begin cur_nibble=status_crc[7:4];cur_is_hex=1'b1;end
            7'd25:begin cur_nibble=status_crc[3:0];cur_is_hex=1'b1;end
            7'd26:cur_byte=8'h0D;
            7'd27:cur_byte=8'h0A;
            default:cur_byte=8'h0A;
            endcase
        end else if(report_v) begin
            case(idx)
            7'd0: cur_byte=8'h24;
            7'd1: cur_byte=8'h56;
            7'd2: cur_byte=8'h2C;
            7'd3: cur_byte=8'h31;
            7'd4: cur_byte=8'h2C;
            7'd5: begin cur_nibble=vsnap[191:188];cur_is_hex=1'b1;end
            7'd6: begin cur_nibble=vsnap[187:184];cur_is_hex=1'b1;end
            7'd7: begin cur_nibble=vsnap[183:180];cur_is_hex=1'b1;end
            7'd8: begin cur_nibble=vsnap[179:176];cur_is_hex=1'b1;end
            7'd9: begin cur_nibble=vsnap[175:172];cur_is_hex=1'b1;end
            7'd10: begin cur_nibble=vsnap[171:168];cur_is_hex=1'b1;end
            7'd11: begin cur_nibble=vsnap[167:164];cur_is_hex=1'b1;end
            7'd12: begin cur_nibble=vsnap[163:160];cur_is_hex=1'b1;end
            7'd13: cur_byte=8'h2C;
            7'd14: begin cur_nibble=vseq[31:28];cur_is_hex=1'b1;end
            7'd15: begin cur_nibble=vseq[27:24];cur_is_hex=1'b1;end
            7'd16: begin cur_nibble=vseq[23:20];cur_is_hex=1'b1;end
            7'd17: begin cur_nibble=vseq[19:16];cur_is_hex=1'b1;end
            7'd18: begin cur_nibble=vseq[15:12];cur_is_hex=1'b1;end
            7'd19: begin cur_nibble=vseq[11:8];cur_is_hex=1'b1;end
            7'd20: begin cur_nibble=vseq[7:4];cur_is_hex=1'b1;end
            7'd21: begin cur_nibble=vseq[3:0];cur_is_hex=1'b1;end
            7'd22: cur_byte=8'h2C;
            7'd23: begin cur_nibble=vsnap[159:156];cur_is_hex=1'b1;end
            7'd24: begin cur_nibble=vsnap[155:152];cur_is_hex=1'b1;end
            7'd25: begin cur_nibble=vsnap[151:148];cur_is_hex=1'b1;end
            7'd26: begin cur_nibble=vsnap[147:144];cur_is_hex=1'b1;end
            7'd27: begin cur_nibble=vsnap[143:140];cur_is_hex=1'b1;end
            7'd28: begin cur_nibble=vsnap[139:136];cur_is_hex=1'b1;end
            7'd29: begin cur_nibble=vsnap[135:132];cur_is_hex=1'b1;end
            7'd30: begin cur_nibble=vsnap[131:128];cur_is_hex=1'b1;end
            7'd31: cur_byte=8'h2C;
            7'd32: cur_byte=lampchar(vsnap[95:94],vsnap[81],vsnap[83]);
            7'd33: cur_byte=8'h2C;
            7'd34: begin cur_nibble=vlage[15:12];cur_is_hex=1'b1;end
            7'd35: begin cur_nibble=vlage[11:8];cur_is_hex=1'b1;end
            7'd36: begin cur_nibble=vlage[7:4];cur_is_hex=1'b1;end
            7'd37: begin cur_nibble=vlage[3:0];cur_is_hex=1'b1;end
            7'd38: cur_byte=8'h2C;
            7'd39: begin cur_nibble=vsnap[127:124];cur_is_hex=1'b1;end
            7'd40: begin cur_nibble=vsnap[123:120];cur_is_hex=1'b1;end
            7'd41: begin cur_nibble=vsnap[119:116];cur_is_hex=1'b1;end
            7'd42: begin cur_nibble=vsnap[115:112];cur_is_hex=1'b1;end
            7'd43: begin cur_nibble=vsnap[111:108];cur_is_hex=1'b1;end
            7'd44: begin cur_nibble=vsnap[107:104];cur_is_hex=1'b1;end
            7'd45: begin cur_nibble=vsnap[103:100];cur_is_hex=1'b1;end
            7'd46: begin cur_nibble=vsnap[99:96];cur_is_hex=1'b1;end
            7'd47: cur_byte=8'h2C;
            7'd48: cur_byte=dirchar(vsnap[93:92]);
            7'd49: cur_byte=8'h2C;
            7'd50: cur_byte=8'h30+(vsnap[91:85]/100);
            7'd51: cur_byte=8'h30+((vsnap[91:85]/10)%10);
            7'd52: cur_byte=8'h30+(vsnap[91:85]%10);
            7'd53: cur_byte=8'h2C;
            7'd54: begin cur_nibble=vsage[15:12];cur_is_hex=1'b1;end
            7'd55: begin cur_nibble=vsage[11:8];cur_is_hex=1'b1;end
            7'd56: begin cur_nibble=vsage[7:4];cur_is_hex=1'b1;end
            7'd57: begin cur_nibble=vsage[3:0];cur_is_hex=1'b1;end
            7'd58: cur_byte=8'h2C;
            7'd59: cur_byte=8'h30;
            7'd60: begin cur_nibble=vsnap[84:81];cur_is_hex=1'b1;end
            7'd61: cur_byte=8'h2A;
            7'd62: begin cur_nibble=vcrc[15:12];cur_is_hex=1'b1;end
            7'd63: begin cur_nibble=vcrc[11:8];cur_is_hex=1'b1;end
            7'd64: begin cur_nibble=vcrc[7:4];cur_is_hex=1'b1;end
            7'd65: begin cur_nibble=vcrc[3:0];cur_is_hex=1'b1;end
            7'd66: cur_byte=8'h0D;
            7'd67: cur_byte=8'h0A;
            default:cur_byte=8'h0A;
            endcase
        end else if (report_lp) begin
            // 独立低功耗诊断行，固定 12 字节：$L,<0|1>,Khhhh\r\n
            case (idx)
                6'd0:  cur_byte = 8'h24; // $
                6'd1:  cur_byte = 8'h4C; // L
                6'd2:  cur_byte = 8'h2C; // ,
                6'd3:  cur_byte = report_lp_active ? 8'h31 : 8'h30;
                6'd4:  cur_byte = 8'h2C; // ,
                6'd5:  cur_byte = 8'h4B; // K
                6'd6:  begin cur_nibble=report_skip_cnt[15:12];cur_is_hex=1'b1;end
                6'd7:  begin cur_nibble=report_skip_cnt[11:8];cur_is_hex=1'b1;end
                6'd8:  begin cur_nibble=report_skip_cnt[7:4];cur_is_hex=1'b1;end
                6'd9:  begin cur_nibble=report_skip_cnt[3:0];cur_is_hex=1'b1;end
                6'd10: cur_byte = 8'h0D; // \r
                6'd11: cur_byte = 8'h0A; // \n
                default: cur_byte = 8'h24;
            endcase
        end
        else begin
            // $T,<id>,<type>,<x>,<y>,<red>,<green>,<yellow>\r\n
            // 后面三个字段是本次新加的多目标个数
            case (idx)
            6'd0:  cur_byte = 8'h24;                    // $
            6'd1:  cur_byte = 8'h54;                    // T
            6'd2:  cur_byte = 8'h2C;                    // ,
            6'd3:  cur_byte = 8'h30 + {6'd0, r_I_id};     // id
            6'd4:  cur_byte = 8'h2C;                    // ,
            6'd5:  cur_byte = F_type(r_I_type);           // type
            6'd6:  cur_byte = 8'h2C;                    // ,
            6'd7:  begin cur_nibble=r_I_x[11:8];cur_is_hex=1'b1;end
            6'd8:  begin cur_nibble=r_I_x[7:4];cur_is_hex=1'b1;end
            6'd9:  begin cur_nibble=r_I_x[3:0];cur_is_hex=1'b1;end
            6'd10: cur_byte = 8'h2C;                    // ,
            6'd11: begin cur_nibble=r_I_y[11:8];cur_is_hex=1'b1;end
            6'd12: begin cur_nibble=r_I_y[7:4];cur_is_hex=1'b1;end
            6'd13: begin cur_nibble=r_I_y[3:0];cur_is_hex=1'b1;end
            6'd14: cur_byte = 8'h2C;                    // ,
            6'd15: cur_byte = 8'h30 + {4'd0, r_I_mt_num[3:0]};   // 红
            6'd16: cur_byte = 8'h2C;
            6'd17: cur_byte = 8'h30 + {4'd0, r_I_mt_num[7:4]};   // 绿
            6'd18: cur_byte = 8'h2C;
            6'd19: cur_byte = 8'h30 + {4'd0, r_I_mt_num[11:8]};  // 黄
            6'd20: cur_byte = 8'h2C;                    // ,
            6'd21: cur_byte = 8'h30 + {4'd0, r_I_circ};   // 圆形目标个数
            6'd22: cur_byte = 8'h2C;                    // ,
            6'd23: cur_byte = 8'h30 + {4'd0, r_I_conf};   // 置信度
            // 追加：发给小车的命令字节（2 位十六进制）
            //   46='F' 前进（绿灯）  53='S' 停车（红/黄灯）  00=还没发过
            6'd24: cur_byte = 8'h2C;                    // ,
            6'd25: begin cur_nibble=r_I_bt_cmd[7:4];cur_is_hex=1'b1;end
            6'd26: begin cur_nibble=r_I_bt_cmd[3:0];cur_is_hex=1'b1;end
            // ---- 阈值回读：9 个字段，各 2 位十六进制 ----
            // 低功耗跳过帧计数（K 是标号）；准确 LP 开关状态由独立 $L 行报告。
            // $T 为兼容旧上位机仍携带 K；idx 是 6 位，因此所有 case 项也必须写 6'd。
            6'd27: cur_byte = 8'h2C;   // ,
            6'd28: cur_byte = 8'h4B;   // K
            6'd29: begin cur_nibble=I_skip_cnt[15:12];cur_is_hex=1'b1;end
            6'd30: begin cur_nibble=I_skip_cnt[11:8];cur_is_hex=1'b1;end
            6'd31: begin cur_nibble=I_skip_cnt[7:4];cur_is_hex=1'b1;end
            6'd32: begin cur_nibble=I_skip_cnt[3:0];cur_is_hex=1'b1;end
            // ---- 阈值回读：9 个字段，各 2 位十六进制 ----
            6'd33: cur_byte = 8'h2C;   // ,
            6'd34: begin cur_nibble=r_I_thr[63:60];cur_is_hex=1'b1;end   // R(高): exactly four bits
            6'd35: begin cur_nibble=r_I_thr[59:56];cur_is_hex=1'b1;end   // R
            6'd36: cur_byte = 8'h2C;   // ,
            6'd37: begin cur_nibble=r_I_thr[55:52];cur_is_hex=1'b1;end   // r(高)
            6'd38: begin cur_nibble=r_I_thr[51:48];cur_is_hex=1'b1;end   // r
            6'd39: cur_byte = 8'h2C;   // ,
            6'd40: begin cur_nibble=r_I_thr[47:44];cur_is_hex=1'b1;end   // G(高)
            6'd41: begin cur_nibble=r_I_thr[43:40];cur_is_hex=1'b1;end   // G
            6'd42: cur_byte = 8'h2C;   // ,
            6'd43: begin cur_nibble=r_I_thr[39:36];cur_is_hex=1'b1;end   // g(高)
            6'd44: begin cur_nibble=r_I_thr[35:32];cur_is_hex=1'b1;end   // g
            6'd45: cur_byte = 8'h2C;   // ,
            6'd46: begin cur_nibble=r_I_thr[31:28];cur_is_hex=1'b1;end   // Y(高)
            6'd47: begin cur_nibble=r_I_thr[27:24];cur_is_hex=1'b1;end   // Y
            6'd48: cur_byte = 8'h2C;   // ,
            6'd49: begin cur_nibble=r_I_thr[23:20];cur_is_hex=1'b1;end   // y(高)
            6'd50: begin cur_nibble=r_I_thr[19:16];cur_is_hex=1'b1;end   // y
            6'd51: cur_byte = 8'h2C;   // ,
            6'd52: begin cur_nibble=r_I_thr[15:12];cur_is_hex=1'b1;end   // B(高)
            6'd53: begin cur_nibble=r_I_thr[11:8];cur_is_hex=1'b1;end   // B
            6'd54: cur_byte = 8'h2C;   // ,
            6'd55: cur_byte = 8'h30;                        // b(高)
            6'd56: begin cur_nibble={4'd0, r_I_thr[7:4]};cur_is_hex=1'b1;end  // b
            6'd57: cur_byte = 8'h2C;   // ,
            6'd58: cur_byte = 8'h30;                        // m(高)
            6'd59: begin cur_nibble={4'd0, r_I_thr[3:0]};cur_is_hex=1'b1;end  // m
            6'd60: cur_byte = 8'h0D;   // \r
            6'd61: cur_byte = 8'h0A;   // \n
            default: cur_byte = 8'h24;
            endcase
        end
        // Select four data bits before the single ASCII encoder. Encoding
        // every field first duplicates the conversion and widens its mux.
        if(cur_is_hex)cur_byte=F_hex(cur_nibble);
    end

    // ---- 发送状态机 ----
    reg       sent;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            q_I_id<=0; q_I_type<=0; q_I_x<=0; q_I_y<=0; q_I_mt_num<=0; q_I_circ<=0; q_I_conf<=0; q_I_bt_cmd<=0; q_I_thr<=0; 
            report_status<=0;report_camera<=0;camera_snapshot<=0;camera_crc<=16'hFFFF;status_snapshot<=0;status_crc<=16'hFFFF;
            state <= IDLE;
            idx   <= 6'd0;
            tx_start <= 1'b0;
            sent  <= 1'b0;
            report_lp <= 1'b0;report_v<=0;vsnap<=0;seq<=0;vseq<=0;vlage<=16'hFFFF;vsage<=16'hFFFF;vcrc<=16'hFFFF;
            report_has_target <= 1'b0;
            report_lp_active <= 1'b0;
            report_skip_cnt <= 16'd0;
        end
        else begin
            tx_start <= 1'b0;
            case (state)
                IDLE: begin
                    report_camera<=0;
                    if (tick) begin
                        state <= SENT;
                        idx   <= 6'd0;
                        sent  <= 1'b0;
                        report_lp <= 1'b1;report_v<=0;report_status<=0;
                        q_I_id<=I_id; q_I_type<=I_type; q_I_x<=I_x; q_I_y<=I_y; q_I_mt_num<=I_mt_num; q_I_circ<=I_circ; q_I_conf<=I_conf; q_I_bt_cmd<=I_bt_cmd; q_I_thr<=I_thr; 
                        report_has_target <= I_valid && traffic_active;
                        report_lp_active <= I_lp_active;
                        report_skip_cnt <= I_skip_cnt;
                    end else if(take_s)begin
                        state<=SENT;idx<=0;sent<=0;report_status<=1;report_v<=0;report_lp<=0;
                        status_snapshot<=I_detect_status;status_crc<=16'hFFFF;
                    end else if(take_v) begin
                        state<=SENT;idx<=0;sent<=0;report_v<=1;report_lp<=0;report_status<=0;
                        vsnap<=traffic_active?latest:192'd0;vseq<=seq;seq<=seq+1'b1;vlage<=lamp_age;vsage<=sign_age;vcrc<=16'hFFFF;
                    end else if(take_p)begin
                        state<=SENT;idx<=0;sent<=0;report_camera<=1;report_status<=0;report_v<=0;report_lp<=0;
                        camera_snapshot<=I_camera_snapshot;camera_crc<=16'hFFFF;
                    end
                end
                SENT: begin
                    if (!sent) begin
                        tx_byte  <= cur_byte;
                        if(report_camera&&idx>=5&&idx<=92)camera_snapshot<={camera_snapshot[347:0],4'd0};
                        if(report_camera&&idx>=1&&idx<93)camera_crc<=next_crc;
                         if(report_status && idx>=1 && idx<7'd21)status_crc<=next_crc;
                         if(report_v && idx>=1 && idx<7'd61) vcrc<=next_crc;
                        tx_start <= 1'b1;
                        sent     <= 1'b1;
                    end
                    if (tx_busy)
                        state <= DONE;
                end
                DONE: begin
                    if (!tx_busy) begin
                        sent <= 1'b0;
                        if(report_camera&&idx==99)begin state<=IDLE;report_camera<=0;end
                        else if(report_status && idx==7'd27)begin state<=IDLE;report_status<=0;end
                         else if(report_v && idx==7'd67) begin state<=IDLE;report_v<=0;end
                         else if (report_lp && (idx == 6'd11)) begin
                            if (report_has_target) begin
                                report_lp <= 1'b0;
                                idx <= 6'd0;
                                state <= SENT;
                            end
                            else begin
                                report_lp <= 1'b0;
                                state <= IDLE;
                            end
                        end
                        else if (!report_camera && !report_status && !report_lp && !report_v && (idx == 6'd61)) begin
                            state <= IDLE;
                        end
                        else begin
                            idx   <= idx + 6'd1;
                            state <= SENT;
                        end
                    end
                end
                default: state <= IDLE;
            endcase
        end
    end

endmodule
