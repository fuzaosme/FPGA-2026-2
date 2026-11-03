// ===========================================================================
//  adaptive_bin_96.v
//  基础要求（2）的图像处理算法：帧统计自适应阈值二值化
//
//  与参考例程 isp/edge_detect_96.v（Sobel 梯度 + 固定阈值 EDGE_TH=80）的区别：
//    1) 算法类别不同：参考做的是"边缘检测"（一阶梯度 + 硬阈值，输出细轮廓）；
//       这里做的是"二值化"（整帧自适应阈值，输出黑白分割），不做任何梯度运算。
//    2) 阈值来源不同：参考的阈值是写死的常数；这里的阈值由上一帧画面的亮度
//       统计自动算出，环境变亮/变暗时阈值自动跟随，不需要重新调参。
//    3) 实现结构不同：没有行缓存、没有 3x3 窗口、不用 BRAM，每拍独立判决，
//       不改变行/帧结构（对基础要求（3）"稳定输出"最友好）。
//    4) 视觉结果不同：参考输出黑底上的细白线；这里输出整片黑白分割画面。
//
//  接口与 edge_detect_96 完全一致（96bit = 4 像素/拍，tvalid/tuser/tlast 透传），
//  所以 isp_top 里只换例化、链路其它部分一行都不用动。
//
//  阈值算法：
//      帧亮度均值 ≈ (整帧像素亮度和) / (IMG_WIDTH*IMG_HEIGHT)
//      thr_new = 帧亮度和 >> THR_SHIFT          // 用右移代替除法
//      THR_SHIFT 取"使 2^THR_SHIFT 略大于总像素数"的整数：
//          1280*720 = 921600，2^20 = 1048576
//          => thr_new ≈ 0.879 × 帧均值
//      再做一阶平滑（IIR），避免阈值逐帧跳动造成画面闪烁：
//          thr <= thr - thr/2^IIR_SHIFT + thr_new/2^IIR_SHIFT
//
//  判决：灰度 > thr + thr/2^BIAS_SHIFT 判为白，否则为黑。
//        偏置取"阈值的一个固定比例"（而不是一个固定数值），这样当整个画面
//        同比例变亮/变暗时，判决线也跟着同比例移动，分割结果保持"尺度无关"：
//        画面亮度 ×k  ->  thr ×k  ->  判决线 ×k  ->  白/黑结论不变。
//        （如果偏置是固定的数值，暗场景里这个固定偏置会显得过大，
//          把本来该判白的目标吃掉——这是实际仿真里验证到的。）
// ===========================================================================

`timescale 1ns/1ps

module adaptive_bin_96 #(
    parameter IMG_WIDTH  = 1280,
    parameter IMG_HEIGHT = 720,
    parameter integer THR_SHIFT = 20,      // 帧亮度和的右移位数
    parameter integer BIAS_SHIFT = 3,      // 判决偏置 = thr 的 1/2^BIAS_SHIFT
    parameter [7:0]   THR_INIT  = 8'd96,   // 首帧统计出来之前的默认阈值
    parameter integer IIR_SHIFT = 3        // 阈值时域平滑（一阶 IIR）
) (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_tlast,
    input  wire        I_tuser,
    input  wire [95:0] I_tdata,
    input  wire        I_tvalid,
    input  wire        I_tready,
    output reg         O_tlast,
    output reg         O_tuser,
    output reg  [95:0] O_tdata,
    output reg         O_tvalid,
    output wire        O_tready
);

    assign O_tready = I_tready;

    // -----------------------------------------------------------------------
    // 1. 灰度化：gray = R/4 + G/2 + B/4（用右移代替除法，只用加法器）
    //    24bit 像素 = {R[23:16], G[15:8], B[7:0]}，取高位分别右移 2/1/2 位
    // -----------------------------------------------------------------------
    wire [23:0] pix0 = I_tdata[95:72];
    wire [23:0] pix1 = I_tdata[71:48];
    wire [23:0] pix2 = I_tdata[47:24];
    wire [23:0] pix3 = I_tdata[23: 0];

    wire [7:0] gray0 = {2'b0,pix0[23:18]} + {1'b0,pix0[15:9]} + {2'b0,pix0[7:2]};
    wire [7:0] gray1 = {2'b0,pix1[23:18]} + {1'b0,pix1[15:9]} + {2'b0,pix1[7:2]};
    wire [7:0] gray2 = {2'b0,pix2[23:18]} + {1'b0,pix2[15:9]} + {2'b0,pix2[7:2]};
    wire [7:0] gray3 = {2'b0,pix3[23:18]} + {1'b0,pix3[15:9]} + {2'b0,pix3[7:2]};

    // -----------------------------------------------------------------------
    // 2. 帧亮度统计 -> 自适应阈值
    // -----------------------------------------------------------------------
    wire [9:0] beat_sum = {2'b0,gray0} + {2'b0,gray1} + {2'b0,gray2} + {2'b0,gray3};

    reg [27:0] frame_acc;          // 1280*720*253 < 2^28
    reg [7:0]  thr;                // 当前判决阈值

    wire [27:0] thr_raw = frame_acc >> THR_SHIFT;
    wire [7:0]  thr_new = (thr_raw > 28'd255) ? 8'd255 : thr_raw[7:0];
    wire [7:0]  cur_dec = thr     >> IIR_SHIFT;   // thr    * (1 - 1/2^IIR)
    wire [7:0]  new_dec = thr_new >> IIR_SHIFT;   // thr_new*      1/2^IIR

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            frame_acc <= 28'd0;
            thr       <= THR_INIT;
        end
        else if(I_tvalid) begin
            if(I_tuser) begin
                // 一帧的第一个有效拍：此刻 frame_acc 正好是上一整帧的亮度和
                thr       <= thr - cur_dec + new_dec;
                frame_acc <= {18'd0, beat_sum};        // 从这一拍开始统计新的一帧
            end
            else begin
                frame_acc <= frame_acc + {18'd0, beat_sum};
            end
        end
    end

    // -----------------------------------------------------------------------
    // 3. 判决 + 输出（白/黑二分）
    // -----------------------------------------------------------------------
    // 判决线 = thr + thr/8（按比例偏置，保证对整体亮度变化"尺度无关"）
    wire [8:0] thr_cmp = {1'b0, thr} + {1'b0, (thr >> BIAS_SHIFT)};
    wire w0 = ({1'b0,gray0} > thr_cmp);
    wire w1 = ({1'b0,gray1} > thr_cmp);
    wire w2 = ({1'b0,gray2} > thr_cmp);
    wire w3 = ({1'b0,gray3} > thr_cmp);

    wire [23:0] out0 = w0 ? 24'hFFFFFF : 24'h000000;
    wire [23:0] out1 = w1 ? 24'hFFFFFF : 24'h000000;
    wire [23:0] out2 = w2 ? 24'hFFFFFF : 24'h000000;
    wire [23:0] out3 = w3 ? 24'hFFFFFF : 24'h000000;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_tlast  <= 1'b0;
            O_tuser  <= 1'b0;
            O_tvalid <= 1'b0;
            O_tdata  <= 96'd0;
        end
        else begin
            O_tlast  <= I_tlast;
            O_tuser  <= I_tuser;
            O_tvalid <= I_tvalid;
            O_tdata  <= I_tvalid ? {out0,out1,out2,out3} : 96'd0;
        end
    end

endmodule
