module isp_top #(
    parameter [7:0] CAMERA_BLACK_LEVEL=8'd16, // covered-lens measurement, 2026-10-28
    // 批 3：连续多少帧"完全没有目标"才进入降频。90 帧 ≈ 1.5s @60fps。
    //   原来是 30 帧（0.5s），真机上太短：场景里偶发一帧误检就能把它叫醒，
//   表现为 OSD/数码管的 LP 一直跳。配合下面"S_any_tgt_f 连续三帧确认"一起用。
    // 做成参数是为了仿真能覆盖成小值，不必真跑 30 帧（真机用默认值）。
    parameter [15:0] IDLE_FRAMES = 16'd90,
    // Engineering display curve. Set to 0 for bit-exact pre-Gamma display.
    // AWB/detectors/motion/adaptive binarization always receive pre-Gamma RGB.
    parameter integer ARROW_ENABLE = 0,
    parameter integer TRAFFIC_DEFAULT_ON = 0,
    parameter [127:0] ARROW_DEFAULT_CFG = 128'd0,
    parameter integer DISPLAY_GAMMA_ENABLE = 1,
    parameter integer DISPLAY_COLOR_TONE_ENABLE = 1
) (
    input 			axi4s_video_aclk,
    input 			I_rst_n			,
    input 			I_tlast			,
    input 			I_tuser			,
    input [39:0] 	I_tdata			,
    input 			I_tvalid		,
    input [9:0] 	I_tdest			,
    input 			O_tready		,
    input [2:0] 	I_disp_mode		,   // 0=显示原图(彩色)  1=显示处理结果(黑白)
    output [127:0] 	O_tdata			,
    output 			O_tlast			,
    output 			O_tuser			,
        output 			O_tvalid		,
    output 			I_tready		,
    output 			O_box_valid		,
    output [11:0] 	O_box_x0		,
    output [11:0] 	O_box_y0		,
    output [11:0] 	O_box_x1		,
        output [11:0] 	O_box_y1		,
    output 			O_plate_valid	,
    output [11:0] 	O_plate_x0		,
    output [11:0] 	O_plate_y0		,
    output [11:0] 	O_plate_x1		,
    output [11:0] 	O_plate_y1		,
    output [1:0] 	O_plate_type,
    // 进-4：运动检测区域（只给显示端画框用；与 O_box_* 分开，
    //        因为 O_box_* 在内部被 target_tracker 当颜色目标输入，不能动）
    output        O_motion_valid,
    output [11:0] O_motion_x0,
    output [11:0] O_motion_y0,
    output [11:0] O_motion_x1,
    output [11:0] O_motion_y1,
    output reg [15:0] O_latency,
    output 			O_t0_valid,
    output [1:0] 	O_t0_id,
    output [2:0] 	O_t0_type,
    output [11:0] 	O_t0_x0,
    output [11:0] 	O_t0_y0,
    output [11:0] 	O_t0_x1,
    output [11:0] 	O_t0_y1,
    output 			O_t1_valid,
    output [1:0] 	O_t1_id,
    output [2:0] 	O_t1_type,
    output [11:0] 	O_t1_x0,
    output [11:0] 	O_t1_y0,
    output [11:0] 	O_t1_x1,
    output [11:0] 	O_t1_y1,
    // ---- 多目标颜色块（分时复用：每帧轮换一种颜色）----
    //   O_mt_tog : 快照更新握手（跨到 HDMI 域）
    //   O_mt_num : {黄[3:0], 绿[3:0], 红[3:0]}
    //   O_mt_tgt : 3 色 x 3 目标，每个 64bit；颜色 c 占 [c*192 +: 192]
    output        	O_mt_tog,
    output [15:0] 	O_mt_num,
    output [527:0]	O_mt_tgt,
    output [15:0] 	O_mt_conf,   // 每类最高置信度，各 4bit
    // ---- 批 3：低功耗按需降频 ----
    //   O_lp_xfer[15:0] = 累计"跳过没跑检测"的帧数，[16] = 当前是否在降频（休眠）
    //   O_lp_tog        = 更新握手；整组一起换，HDMI 域靠 toggle 边沿锁存
    output [16:0]	O_lp_xfer,
    output       	O_lp_tog,
    // ---- 颜色块检测阈值（串口在线可调，见 design_top_wrapper 的 thr_regs）----
    input [7:0] 	I_thr_red_min,
    input [7:0] 	I_thr_red_gap,
    input [7:0] 	I_thr_grn_min,
    input [7:0] 	I_thr_grn_gap,
    input [7:0] 	I_thr_yel_min,
    input [7:0] 	I_thr_yel_bdiff,
    input [7:0] 	I_thr_yel_bmax,
    input [3:0] 	I_thr_blk_pix,
    input [3:0] 	I_thr_min_blk,
    // 批 3：低功耗按需降频总开关（串口 L0 关 / L1 开，默认开）
    input wire I_arrow_cfg_valid,input wire [127:0] I_arrow_cfg,
    output wire [191:0] O_arrow_snapshot,output wire O_arrow_snapshot_valid,
    output wire [4:0] O_visible_signs,
    input       	I_lp_en,
    input wire I_detect_cfg_valid, input wire [34:0] I_detect_cfg,
    output wire [39:0] O_detect_status,
    input wire [2:0] I_debug_view,input wire I_camera_unstable,
    output reg [287:0] O_camera_snapshot,output reg O_camera_snapshot_valid,
    output wire [8:0] O_camera_meter,output wire O_camera_meter_valid
);
 
// 颜色块与交通灯共用同一套串口可调阈值（原来交通灯走编译期常量，两套不一致）。
// 必须声明在第一次使用之前，否则 vlog 会把它当成隐式 1bit 网线，报重复声明。
// Host absolute thresholds retain their original sensor-code meaning. Gaps
// are translation-invariant; subtract the calibrated pedestal from minima and
// the yellow blue ceiling once, just as for the image samples.
function [7:0] calibrated_threshold;
 input [7:0] value;
 begin calibrated_threshold=(value>CAMERA_BLACK_LEVEL)?value-CAMERA_BLACK_LEVEL:8'd0;end
endfunction
wire [55:0] S_thr56 = {calibrated_threshold(I_thr_red_min), I_thr_red_gap,
                       calibrated_threshold(I_thr_grn_min), I_thr_grn_gap,
                       calibrated_threshold(I_thr_yel_min), I_thr_yel_bdiff,
                       calibrated_threshold(I_thr_yel_bmax)};

wire         m_aixs_tvalid;  //synthesis keep 
  wire [127:0] m_aixs_tdata;  //synthesis keep 
  wire         m_aixs_tuser;  //synthesis keep 
  wire         m_aixs_tlast;  //synthesis keep 
  wire         m_aixs_tready;  //synthesis keep 
  
  wire [ 95:0] m_aixs_tdata_96;  //synthesis keep 


  wire         awb_O_tlast;  //synthesis keep 
  wire         awb_O_tuser;  //synthesis keep 
  wire [ 95:0] awb_O_tdata;  //synthesis keep 
  wire         awb_O_tvalid;  //synthesis keep 
  wire         awb_O_tready;  //synthesis keep 

  wire         awb_O_tuser_128;  //synthesis keep 
  wire         awb_O_tvalid_128;  //synthesis keep 
  wire [127:0] awb_O_tdata_128;  //synthesis keep 

  // Detection control is committed atomically at a live AWB frame boundary.
  reg [34:0] detect_pending;reg detect_pending_valid;
  reg [31:0] detect_epoch;reg traffic_enabled;reg [1:0] motion_profile;
  wire detect_apply=detect_pending_valid && awb_O_tuser;
  wire camera_unstable=(I_camera_unstable===1'b1);
  wire motion_flush=(detect_apply && (detect_pending[1:0]!=motion_profile))||camera_unstable;
  wire motion_ready;wire [3:0] motion_num;wire [191:0] motion_targets;wire motion_toggle;
  assign O_detect_status={detect_epoch,traffic_enabled,motion_profile,motion_ready,motion_num};
  always @(posedge axi4s_video_aclk or negedge I_rst_n)begin
   if(!I_rst_n)begin detect_pending<=0;detect_pending_valid<=0;detect_epoch<=0;
    traffic_enabled<=TRAFFIC_DEFAULT_ON;motion_profile<=0;end
   else begin
    if(detect_apply)begin detect_epoch<=detect_pending[34:3];traffic_enabled<=detect_pending[2];
     motion_profile<=detect_pending[1:0];detect_pending_valid<=0;end
    if(I_detect_cfg_valid)begin detect_pending<=I_detect_cfg;detect_pending_valid<=1;end
   end
  end
  // ---- LP skip gate for color/light/region detection only ----
  // Keep tvalid/tuser/tlast running so detector coordinates stay aligned.
  // The motion frame-difference path intentionally bypasses this zero-fill gate:
  // it must compare consecutive live frames or static backgrounds become false motion.
  // This keeps motion_diff live at frame rate; skipped maps are still zeroed before the shared region engine.
  // Display/DDR/HDMI paths remain ungated and always carry the live image.
  reg [2:0] disp_mode_sync,disp_mode_sync2;
  reg  S_det_skip;
  // Shared recognition sample: [1 2 1]/4 across x,x+1,x+2, centred at
  // the same representative x+1 used by both detectors. Average each channel
  // equally so hue is preserved; no display Gamma or extra digital gain.
  // This removes single-pixel noise BEFORE colour comparisons fragment masks.
  function [7:0] recognition_filter;
   input [7:0] a,b,c;
   reg [9:0] sum;
   begin sum={2'b0,a}+{1'b0,b,1'b0}+{2'b0,c}+10'd2;recognition_filter=sum[9:2];end
  endfunction
  wire [23:0] recognition_pixel={
   recognition_filter(awb_O_tdata[95:88],awb_O_tdata[71:64],awb_O_tdata[47:40]),
   recognition_filter(awb_O_tdata[87:80],awb_O_tdata[63:56],awb_O_tdata[39:32]),
   recognition_filter(awb_O_tdata[79:72],awb_O_tdata[55:48],awb_O_tdata[31:24])};
  wire [95:0] recognition_rgb={4{recognition_pixel}};
  wire [95:0] det_tdata = (S_det_skip || !(traffic_enabled || disp_mode_sync2==5) || camera_unstable) ? 96'd0 : recognition_rgb;
  // 批 3：低功耗"补看"时冻结相位轮换（声明放在这里，因为下面的 mt_cyc 要用到它；
  // 赋值在文件后半段，Verilog 允许赋值在后面，但**声明必须在前**，否则会被当成
  // 隐式 1bit 网线并报重复声明）
  wire S_rot_freeze;

  // 运动支路专用的一拍寄存器（见 u_motion_diff 处的说明）：
  // 数据 + 三个流标志一起打拍，保证 motion_diff 内部的坐标递推不错位。
  reg  [95:0] mot_tdata_r;
  reg         mot_tvalid_r, mot_tuser_r, mot_tlast_r;
  always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
      if (!I_rst_n) begin
          mot_tdata_r  <= 96'd0;
          mot_tvalid_r <= 1'b0;
          mot_tuser_r  <= 1'b0;
          mot_tlast_r  <= 1'b0;
      end
      else begin
          // Motion frame difference must use consecutive live frames; LP zero-fill would cause false full-frame motion.
          mot_tdata_r  <= awb_O_tdata;
          mot_tvalid_r <= awb_O_tvalid;
          mot_tuser_r  <= awb_O_tuser;
          mot_tlast_r  <= awb_O_tlast;
      end
  end

    // ISP 图像处理级的输出（本设计为"帧统计自适应阈值二值化"，见 adaptive_bin_96.v）
    wire         proc_O_tready;  //synthesis keep

    assign awb_O_tready  = proc_O_tready;
    assign proc_O_tready = O_tready;
  wire display_packed_last;
  assign O_tdata  = awb_O_tdata_128;
  assign O_tlast  = display_packed_last;
  // video_in uses O_tuser as an EARLY FIFO-reset strobe, not AXI SOF.
  // Preserve its original three-clock AWB lead. Pixel-associated SOF stays
  // with the tagged display stream and is still checked inside the packer.
  reg [2:0] display_frame_reset_pipe;
  always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
   if(!I_rst_n)display_frame_reset_pipe<=0;
   else display_frame_reset_pipe<={display_frame_reset_pipe[1:0],awb_O_tuser};
  end
  assign O_tuser=display_frame_reset_pipe[2];
  assign O_tvalid = awb_O_tvalid_128   ;
  
  //assign m_aixs_tready = O_tready;
  //assign O_tdata  = m_aixs_tdata;
  //assign O_tlast  = m_aixs_tlast    ;
  //assign O_tuser  = m_aixs_tuser    ;
  //assign O_tvalid = m_aixs_tvalid   ;
 
 
wire [95:0] camera_raw_gray;
wire [19:0] camera_gain_r,camera_gain_b,camera_samples;
wire camera_samples_valid,camera_awb_stats;
demosaic #(
    .IMG_HEIGHT          (720),  // 图像高度
    .IMG_WIDTH           (1280), // 图像宽度
    .data_complete_delay (50  ),
    .BAYER_MODE          ("BGGR")
)
u_demosaic
(
    .O_raw_gray(camera_raw_gray),
    .I_clk   			(axi4s_video_aclk)	,   // 时钟信号
    .I_rst_n 			(I_rst_n)	,   // 复位信号，低有效
    .axi4s_video_tdata 	(I_tdata)	,  // AXI4-Stream视频数据
    .axi4s_video_tdest 	(I_tdest)	,
    .axi4s_video_tlast 	(I_tlast)	,  // 行结束信号
    .axi4s_video_tvalid	(I_tvalid)	,  // 数据有效信号
    .axi4s_video_tuser 	(I_tuser)	,  // 帧开始信号
    .axi4s_video_tready	(I_tready)	,  // 从模块准备好接受数据
    .O_tlast  			(m_aixs_tlast)	,  // 输出行结束信号
    .O_tuser  			(m_aixs_tuser)	,  // 输出帧开始信号
    .O_tdata  			(m_aixs_tdata)	,  // 输出数据
    .O_tvalid 			(m_aixs_tvalid)	,  // 输出数据有效信号
    .O_tready   		(m_aixs_tready)	   // 输出数据准备好信号
);
 
wire [95:0] camera_sensor_rgb;
data128_96 u_data128_96 (
    .I_tdata(m_aixs_tdata),
    .O_tdata(camera_sensor_rgb)
);

// Measured residual black pedestal is 16 RAW8 codes in all RGB channels.
// Subtract once after interpolation, before both AWB and AE; RAW tap stays
// unmodified so the calibration can be independently checked again.
genvar black_ch;
generate for(black_ch=0;black_ch<12;black_ch=black_ch+1)begin: calibrated_black
 wire [7:0] c=camera_sensor_rgb[black_ch*8+:8];
 assign m_aixs_tdata_96[black_ch*8+:8]=(c>CAMERA_BLACK_LEVEL)?c-CAMERA_BLACK_LEVEL:8'd0;
end endgenerate

// Lift shadows before both AWB/recognition and display, retaining zero black.
// Three continuous ranges use only bit selection: 2x below 64, +64 up to
// 127, then half-slope highlights. No hard white clipping or added latency.
// AE retains the calibrated linear tap, so it cannot cancel this lift.
function [7:0] camera_shadow_lift;
 input [7:0] c;
 begin
  if(c<8'd64) camera_shadow_lift={c[6:0],1'b0};
  else if(c<8'd128) camera_shadow_lift={2'b10,c[5:0]};
  else camera_shadow_lift={1'b1,c[7:1]};
 end
endfunction
wire [95:0] camera_lift_rgb;
genvar lift_ch;
generate for(lift_ch=0;lift_ch<12;lift_ch=lift_ch+1)begin: shadow_lift
 assign camera_lift_rgb[lift_ch*8+:8]=camera_shadow_lift(m_aixs_tdata_96[lift_ch*8+:8]);
end endgenerate

awb #(
    .IMG_HEIGHT(720),
    .IMG_WIDTH (1280)
) u_awb (
    .O_gain_r(camera_gain_r),.O_gain_b(camera_gain_b),.O_samples(camera_samples),
    .O_samples_valid(camera_samples_valid),.O_frame_stats(camera_awb_stats),
    .I_clk   (axi4s_video_aclk),
    .I_rst_n (I_rst_n),
    .I_tlast (m_aixs_tlast),
    .I_tuser (m_aixs_tuser),
    .I_tdata (camera_lift_rgb),
    .I_tvalid(m_aixs_tvalid),
    .I_tready(m_aixs_tready),
    .O_tlast (awb_O_tlast ),
    .O_tuser (awb_O_tuser ),
    .O_tdata (awb_O_tdata ),
    .O_tvalid(awb_O_tvalid),
    .O_tready(awb_O_tready)
);

assign O_box_valid = 1'b0;
assign O_box_x0    = 12'd0;
assign O_box_y0    = 12'd0;
assign O_box_x1    = 12'd0;
assign O_box_y1    = 12'd0;

// ---- 车牌识别已按用户要求取消（备份见 lab_ex4_basic_plate_backup_20261006）----
assign O_plate_valid = 1'b0;
assign O_plate_type  = 2'd0;
assign O_plate_x0    = 12'd0;
assign O_plate_y0    = 12'd0;
assign O_plate_x1    = 12'd0;
assign O_plate_y1    = 12'd0;

// ---- 交通灯识别：红/黄/绿圆形目标 ----
wire        S_light_valid;
wire [1:0]  S_light_color;
wire [11:0] S_light_x0, S_light_y0, S_light_x1, S_light_y1;

wire [127:0] arrow_cfg;wire arrow_flush,arrow_lamp_eval;wire [2:0] arrow_lamp_colors;
wire [146:0] display_lamp_targets;
wire raw_light_valid,raw_lamp_eval;wire [1:0] raw_light_color;wire [2:0] raw_lamp_colors;
wire [11:0] raw_light_x0,raw_light_y0,raw_light_x1,raw_light_y1;
light_detect #(
    .ROI_ENABLE(ARROW_ENABLE),
    .IMG_WIDTH (1280),
    .IMG_HEIGHT(720),
    .MIN_PIX   (20'd40)
) u_light_detect (
    .I_clk   (axi4s_video_aclk),
    .I_rst_n (I_rst_n),
    .I_tvalid(awb_O_tvalid),
    .I_tuser (awb_O_tuser),
    .I_tlast (awb_O_tlast),
    .I_tdata (det_tdata),
    .O_light_valid (raw_light_valid),
    .O_light_color (raw_light_color),
    .O_light_x0    (raw_light_x0),
    .O_light_y0    (raw_light_y0),
    .O_light_x1    (raw_light_x1),
    .O_light_y1    (raw_light_y1),
    .I_thr         (S_thr56),
    .I_roi_active(ARROW_ENABLE && arrow_cfg[127:96]!=0),.I_roi(arrow_cfg[95:48]),
    .O_eval(raw_lamp_eval),.O_colors(raw_lamp_colors),.O_display_targets(display_lamp_targets)
);
light_confirm_v5 light_confirmation(.I_clk(axi4s_video_aclk),.I_rst_n(I_rst_n),
 .I_enable(traffic_enabled),.I_flush(detect_apply|arrow_flush|camera_unstable),.I_eval(raw_lamp_eval),
 .I_valid(raw_light_valid),.I_color(raw_light_color),.I_colors(raw_lamp_colors),
 .I_x0(raw_light_x0),.I_y0(raw_light_y0),.I_x1(raw_light_x1),.I_y1(raw_light_y1),
 .O_valid(S_light_valid),.O_color(S_light_color),.O_colors(arrow_lamp_colors),
 .O_x0(S_light_x0),.O_y0(S_light_y0),.O_x1(S_light_x1),.O_y1(S_light_y1),.O_eval(arrow_lamp_eval));
wire [191:0] arrow_snapshot_raw;wire arrow_snapshot_raw_valid;
generate if(ARROW_ENABLE) begin: arrow_enabled
arrow_vision #(.ENABLE_CONTROL(1),.DEFAULT_CFG(ARROW_DEFAULT_CFG)) arrow(.I_enable(traffic_enabled || disp_mode_sync2==5),.I_external_flush(detect_apply|camera_unstable),.I_clk(axi4s_video_aclk),.I_rst_n(I_rst_n),
 .I_tvalid(awb_O_tvalid),.I_tuser(awb_O_tuser),.I_tlast(awb_O_tlast),.I_rgb(det_tdata),
 .I_cfg_valid(I_arrow_cfg_valid),.I_cfg(I_arrow_cfg),// Recognition snapshots still serve display and telemetry.
 .I_lamp_done(raw_lamp_eval),.I_lamp_colors(raw_lamp_colors),
 .O_cfg(arrow_cfg),.O_flush(arrow_flush),.O_snapshot(arrow_snapshot_raw),.O_snapshot_valid(arrow_snapshot_raw_valid),.O_sign_box());
end else begin: no_arrow
 assign arrow_cfg=0;assign arrow_flush=0;assign arrow_snapshot_raw=0;assign arrow_snapshot_raw_valid=0;
end endgenerate
assign O_arrow_snapshot=traffic_enabled?arrow_snapshot_raw:192'd0;
assign O_arrow_snapshot_valid=ARROW_ENABLE && (arrow_snapshot_raw_valid || (!traffic_enabled && awb_O_tuser));

// ---- 进-2 目标跟踪：颜色/车牌/交通灯 -> 稳定 ID ----
target_tracker u_target_tracker (
    .I_clk   (axi4s_video_aclk),
    .I_rst_n (I_rst_n && traffic_enabled && !detect_apply),
    .I_tuser (awb_O_tuser),
    .I_box_valid (O_box_valid),
    .I_box_x0    (O_box_x0),
    .I_box_y0    (O_box_y0),
    .I_box_x1    (O_box_x1),
    .I_box_y1    (O_box_y1),
    .I_plate_valid (O_plate_valid),
    .I_plate_x0    (O_plate_x0),
    .I_plate_y0    (O_plate_y0),
    .I_plate_x1    (O_plate_x1),
    .I_plate_y1    (O_plate_y1),
    .I_plate_type  (O_plate_type),
    .I_light_valid (S_light_valid),
    .I_light_color (S_light_color),
    .I_light_x0    (S_light_x0),
    .I_light_y0    (S_light_y0),
    .I_light_x1    (S_light_x1),
    .I_light_y1    (S_light_y1),
    .O_t0_valid (O_t0_valid),
    .O_t0_id    (O_t0_id),
    .O_t0_type  (O_t0_type),
    .O_t0_x0    (O_t0_x0),
    .O_t0_y0    (O_t0_y0),
    .O_t0_x1    (O_t0_x1),
    .O_t0_y1    (O_t0_y1),
    .O_t1_valid (O_t1_valid),
    .O_t1_id    (O_t1_id),
    .O_t1_type  (O_t1_type),
    .O_t1_x0    (O_t1_x0),
    .O_t1_y0    (O_t1_y0),
    .O_t1_x1    (O_t1_x1),
    .O_t1_y1    (O_t1_y1)
);
// Two-register bypass taps preserve the AWB stream timing.
reg [95:0] camera_raw1,camera_raw2,camera_pre1,camera_pre2;
reg [2:0] camera_view_active,camera_view_requested;
always @(*)begin
 case(I_debug_view)1,2,3,4,5:camera_view_requested=I_debug_view;default:camera_view_requested=0;endcase
end
always @(posedge axi4s_video_aclk or negedge I_rst_n)begin
 if(!I_rst_n)begin camera_raw1<=0;camera_raw2<=0;camera_pre1<=0;camera_pre2<=0;camera_view_active<=0;end
 else begin
  if(m_aixs_tready)begin camera_raw1<=camera_raw_gray;camera_raw2<=camera_raw1;
   camera_pre1<=m_aixs_tdata_96;camera_pre2<=camera_pre1;end
  if(awb_O_tuser)camera_view_active<=camera_view_requested;
 end
end
wire [2:0] camera_view=awb_O_tuser?camera_view_requested:camera_view_active;
wire [95:0] camera_display_rgb=(camera_view==1)?camera_raw2:(camera_view==2)?camera_pre2:awb_O_tdata;
wire [19:0] camera_quant_black,camera_edge_pixels;
// ---- Display-only streaming effects; mode is latched at the next SOF. ----
always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
 if(!I_rst_n)begin disp_mode_sync<=0;disp_mode_sync2<=0;end
 else begin disp_mode_sync<=I_disp_mode;disp_mode_sync2<=disp_mode_sync;end
end
wire [95:0] mux_tdata;
wire mux_tlast,mux_tuser,mux_tvalid;
wire [11:0] display_x,display_y;
display_fx_96 #(.DISPLAY_GAMMA_ENABLE(DISPLAY_GAMMA_ENABLE),
 .DISPLAY_COLOR_TONE_ENABLE(DISPLAY_COLOR_TONE_ENABLE)) u_display_fx(
 .I_clk(axi4s_video_aclk),.I_rst_n(I_rst_n),.I_tvalid(awb_O_tvalid),
 .I_tuser(awb_O_tuser),.I_tlast(awb_O_tlast),.I_rgb(camera_display_rgb),.I_mode(disp_mode_sync2),
 .I_lamp_targets(display_lamp_targets),.I_sign_valid(arrow_snapshot_raw[82] && (arrow_snapshot_raw[93:92]==1 || arrow_snapshot_raw[93:92]==2)),
 .I_sign_dir(arrow_snapshot_raw[93:92]),.O_visible_signs(O_visible_signs),
 .I_sign_box(arrow_snapshot_raw[80:33]),.I_clear_targets(!traffic_enabled || camera_unstable || detect_apply || arrow_flush),.I_debug_view(camera_view),.O_quant_black(camera_quant_black),.O_edge_pixels(camera_edge_pixels),
 .O_rgb(mux_tdata),.O_tlast(mux_tlast),.O_tuser(mux_tuser),.O_tvalid(mux_tvalid),
 .O_x(display_x),.O_y(display_y));

// A dedicated pre-tone luminance meter feeds AE every frame. One diagnostic
// meter rotates between stages every 16 frames; no duplicate RGB accumulators.
wire [159:0] camera_ae_stats,camera_selected_stats;wire camera_ae_done,camera_diag_done;
wire [9:0] camera_input_y={2'd0,m_aixs_tdata_96[95:88]}+{1'd0,m_aixs_tdata_96[87:80],1'b0}+{2'd0,m_aixs_tdata_96[79:72]}+10'd2;
wire [95:0] camera_ae_rgb={12{camera_input_y[9:2]}};
camera_meter96 pre_meter(axi4s_video_aclk,I_rst_n,m_aixs_tvalid,m_aixs_tuser,m_aixs_tlast,camera_ae_rgb,camera_ae_stats,camera_ae_done);
assign O_camera_meter={camera_ae_stats[0],camera_ae_stats[127:120]};
assign O_camera_meter_valid=camera_ae_done;
reg [1:0] camera_report_stage;
wire [95:0] camera_diag_rgb=(camera_report_stage==0)?camera_raw_gray:
 (camera_report_stage==1)?m_aixs_tdata_96:(camera_report_stage==2)?recognition_rgb:mux_tdata;
wire camera_diag_v=(camera_report_stage<2)?m_aixs_tvalid:(camera_report_stage==2)?awb_O_tvalid:mux_tvalid;
wire camera_diag_u=(camera_report_stage<2)?m_aixs_tuser:(camera_report_stage==2)?awb_O_tuser:mux_tuser;
wire camera_diag_l=(camera_report_stage<2)?m_aixs_tlast:(camera_report_stage==2)?awb_O_tlast:mux_tlast;
camera_meter96 diagnostic_meter(axi4s_video_aclk,I_rst_n,camera_diag_v,camera_diag_u,camera_diag_l,camera_diag_rgb,camera_selected_stats,camera_diag_done);
always @(posedge axi4s_video_aclk or negedge I_rst_n)begin
 if(!I_rst_n)begin camera_report_stage<=0;O_camera_snapshot<=0;O_camera_snapshot_valid<=0;end
 else begin
  O_camera_snapshot_valid<=0;
  if(camera_diag_done&&camera_selected_stats[0]&&camera_selected_stats[131:128]==0)begin
   O_camera_snapshot<={14'd0,camera_view,1'b0,camera_report_stage,{5'd0,camera_view==0,camera_unstable,camera_samples_valid},camera_edge_pixels,camera_quant_black,
     camera_gain_b,camera_gain_r,camera_samples,camera_selected_stats};
   O_camera_snapshot_valid<=1;camera_report_stage<=camera_report_stage+1'b1;
  end
 end
end
// Continuous motion processing is independent of traffic and LP gates.
motion_v5 motion(.I_clk(axi4s_video_aclk),.I_rst_n(I_rst_n),.I_flush(motion_flush),
 .I_tvalid(mot_tvalid_r),.I_tuser(mot_tuser_r),.I_tlast(mot_tlast_r),.I_rgb(mot_tdata_r),
 .I_profile(motion_profile),.O_ready(motion_ready),.O_toggle(motion_toggle),.O_num(motion_num),.O_targets(motion_targets));
assign O_motion_valid=motion_num!=0;
assign O_motion_x0=motion_targets[47:36];assign O_motion_y0=motion_targets[35:24];
assign O_motion_x1=motion_targets[23:12];assign O_motion_y1=motion_targets[11:0];

// 阶段A：ISP 处理延迟测量（单位：微秒，MIPI 字节时钟 = 60MHz）
//   从 ISP 入口的帧首 I_tuser 复位计数，到 ISP 出口的帧首 mux_tuser 锁存，
//   量到的就是这一帧在 ISP 流水线里待的时间（demosaic 行缓存 + awb + 风格处理）。
//   ★ 原来复位点写的是 awb_O_tuser（出口），锁存点是 mux_tuser（出口+1拍），
//     等于在量"帧周期-延迟"，再被 hdmi_mixer 里 999 的上限一夹就恒显示 000。
reg [15:0] lat_us;
reg [15:0] lat_latched;
reg [5:0]  lat_div;
always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
    if(!I_rst_n) begin
        lat_us      <= 16'd0;
        lat_div     <= 6'd0;
        lat_latched <= 16'd0;
        O_latency   <= 16'd0;
    end
    else begin
        if (I_tuser) begin
            lat_us  <= 16'd0;
            lat_div <= 6'd0;
        end
        else if (lat_div == 6'd59) begin
            lat_div <= 6'd0;
            lat_us  <= lat_us + 16'd1;
        end
        else begin
            lat_div <= lat_div + 6'd1;
        end
        if (mux_tuser)
            lat_latched <= lat_us;
        O_latency <= lat_latched;
    end
end

    display_pack96 u_data_96bit_to_128bit(
        .clk(axi4s_video_aclk),.rst(I_rst_n),.v(mux_tvalid),.u(mux_tuser),.l(mux_tlast),.d(mux_tdata),
        .ov(awb_O_tvalid_128),.ou(awb_O_tuser_128),.ol(display_packed_last),.od(awb_O_tdata_128));




//data96_128 u_data96_128 (
//    .I_tdata(awb_O_tdata),
//    .O_tdata(awb_O_tdata_128)
//);
	
// Generic colour objects are intentionally removed. Only the motion bank
// is exported; legacy colour counters and shape-confidence fields are zero.
assign O_mt_tog=motion_toggle;
assign O_mt_num={motion_num,12'd0};
assign O_mt_conf=16'd0;
genvar motion_slot;
generate for(motion_slot=0;motion_slot<3;motion_slot=motion_slot+1)begin: motion_pack
 assign O_mt_tgt[396+motion_slot*44 +:44]={motion_targets[motion_slot*64+63],
  motion_targets[motion_slot*64+46 -:11],motion_targets[motion_slot*64+33 -:10],
  motion_targets[motion_slot*64+22 -:11],motion_targets[motion_slot*64+9 -:10],1'b0};
end endgenerate
assign O_mt_tgt[395:0]=396'd0;

// =======================================================================
// 批 3：低功耗按需降频
//
// 控制律：连续 IDLE_FRAMES 帧"什么目标都没有"（跟踪槽/交通灯/运动框/箭头全空）
//   才进入降频。降频采用 "3 帧真跑 + 2 帧空跑" 的 5 帧周期：相位 0/1/2 认真
//   检测，相位 3/4 把检测输入强制成 0（见 det_tdata），K 只在两个空跑帧各 +1。
//   任何一路出现目标就"**补看**"：当前帧立即真跑、相位冻结，连续三帧有证据才
//   回全速（一闪而过的毛刺在补看那一帧就没了，见 S_conf_look / S_ev_cnt）。
//   串口 L0/L1 可以整个关掉/打开（I_lp_en，默认开）。
//
// 为什么必须是 "3 帧真跑 + ≤2 帧空跑"（实测扫描 1+3 / 2+2 / 3+1 / 3+3 / 4+4）：
//   1) sign_detect 的 completed-bank 匹配、灯色投票都要**连续三帧**都看到目标，
//      只给两帧时箭头方向永远凑不齐；2+2 及更稀的占空比会整体丢箭头/灯色。
//   2) sign_detect 连续 3 帧没方向就会切换分割模式（misses 饱和到 3），空跑
//      ≥3 帧会让它在空帧上把模式切走，目标回来时反而更难收敛；空跑 ≤2 帧
//      只累积 2 次 miss，不会触发切换。
//   所以 3+2 是"仍能保住箭头/灯/运动全部识别"的前提下最省的一档。
//
// 为什么准入条件必须是"全空"：空跑的 2 帧里所有检测输出都会变成空，要是画面里
//   本来有目标，使用者会看到框闪、批 2 的倒计时也会跟着冻一下。把条件定成
//   "全空"，这个机制对使用者就是完全不可见的。
//
// 代价（报告里如实写）：休眠期检测刷新率 60%，即每 5 帧省 2 帧检测工作，
//   最坏唤醒延迟约 2 帧 33ms。换来的可验证数字就是下面的跳过帧计数 K。
// =======================================================================
// IDLE_FRAMES 是模块参数（见模块头）
reg [15:0] S_idle_cnt;
reg [2:0]  S_lp_ph;
reg        S_lp_sleep;
reg [15:0] S_skip_cnt;
reg [16:0] S_lp_xfer;
reg        S_lp_tog;

// 去毛刺用"**看到就补看**"，不是"隔几个采样期再数"：
//   证据一来（S_ev = 灯/运动/箭头），这一帧立即真跑并冻结相位，接下来连续真跑，
//   三帧都有才醒（S_ev_cnt 饱和到 3）。
//   为什么是 3 帧：一闪而过的东西会让**帧差**连着报两帧（出现那帧比上一帧差、消失
//   那帧又比上一帧差），2 帧窗恰好挡不住，3 帧才分得开。
//   真机上噪声大、LP 还在跳的话，可以把窗口再加长，或者直接发 L0 关掉。
// Motion wake is qualified by O_motion_valid and a minimum 8-block count above.
// This path uses the same three-frame confirmation and evidence hold as other targets.
wire S_fast_tgt = O_t0_valid | O_t1_valid | S_light_valid;
// Motion candidates require a substantial frame-difference count; S_ev_cnt below
// still demands three consecutive evidence frames before the LP state wakes.
wire S_motion_tgt = motion_num!=0;
// Arrow evidence reuses the existing arrow snapshot instead of adding a detector:
// bit 82 = confirmed sign, bits [93:92] = current matched direction. Either one
// means the sign path saw something and the LP probe must stop blanking frames.
wire S_sign_tgt = (arrow_snapshot_raw[93:92] != 2'd0) || arrow_snapshot_raw[82];
wire S_ev       = S_fast_tgt | S_motion_tgt | S_sign_tgt;

reg [1:0] S_ev_cnt;
always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
    if (!I_rst_n) S_ev_cnt <= 2'd0;
    // 只统计"真跑过检测"的帧；S_det_skip 在这一拍还是"上一帧跳没跳"
    else if (I_tuser && !S_det_skip) begin
        if (S_ev) S_ev_cnt <= (S_ev_cnt == 2'd3) ? 2'd3 : (S_ev_cnt + 2'd1);
        else      S_ev_cnt <= 2'd0;
    end
end

wire S_wake_now   = (S_ev_cnt >= 2'd3);          // 连着三帧都对上 → 满足唤醒条件
wire S_conf_look  = S_ev | (S_ev_cnt != 2'd0);   // 有证据/正在补看 → 这一帧也要真跑
assign S_rot_freeze = 1'b0; // no phase-rotated colour region engine remains  // 补看时冻结相位轮换（查同一路）

// ---- "醒着"必须带记忆，不能只看"连续三帧" ----
//   ★ 踩过的坑：一开始把 S_wake_now 直接当成"醒着"的条件，结果目标明明一直在，
//     系统也醒了又睡。原因：区域引擎是 4 相位分时的，同一个颜色**4 帧才轮到一次**，
//     所以"连续三帧都有证据"永远凑不齐 —— 旧代码里那个"陈旧快照 mt_num_bank"
//     其实一直在偷偷充当这个记忆（它拿着旧结果不让系统睡），我把它换掉之后这个
//     记忆就没了。
//   改法：唤醒用"确认三帧"（去毛刺），**醒着**用一个证据记忆：
//     最近 EV_HOLD 帧内只要见过证据就保持醒着（>16 帧，能覆盖休眠期最稀的证据间隔），
//     证据彻底断了才允许回到降频。
localparam [4:0] EV_HOLD = 5'd24;    // ≈0.4s @60fps
reg [4:0] S_ev_age;                  // 距上次"本帧有证据"过了几帧（饱和）
always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
    if (!I_rst_n) S_ev_age <= 5'd0;
    else if (I_tuser) begin
        if (S_ev)                     S_ev_age <= 5'd0;
        else if (S_ev_age != 5'h1F)   S_ev_age <= S_ev_age + 5'd1;
    end
end

reg S_awake;
always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
    if (!I_rst_n) S_awake <= 1'b0;
    else if (I_tuser) begin
        if (S_wake_now)                  S_awake <= 1'b1;   // 确认了 → 醒
        else if (S_ev_age >= EV_HOLD)    S_awake <= 1'b0;   // 证据断久了 → 回降频
    end
end

// 总开关（串口 L0/L1，默认开）：关掉时强制全速，K 字段也不再增长。
// The old build forced full speed whenever ARROW_ENABLE!=0, which is why the
// host LP switch could never leave "全速" and K stayed 0. The 3-live/2-blank
// duty cycle above keeps the three consecutive frames the detectors need, so
// arrow builds can sleep without losing recognition.
wire S_lp_awake = S_awake | ~I_lp_en | ~traffic_enabled;

always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
    if (!I_rst_n) begin
        S_idle_cnt <= 16'd0;
        S_lp_ph    <= 3'd0;
        S_lp_sleep <= 1'b0;
        S_skip_cnt <= 16'd0;
        S_lp_xfer  <= 17'd0;
        S_lp_tog   <= 1'b0;
        S_det_skip <= 1'b0;
    end
    else if (I_tuser) begin
        if (S_lp_awake) begin
            S_idle_cnt <= 16'd0;
            S_lp_sleep <= 1'b0;
            S_lp_ph    <= 3'd0;
            S_det_skip <= 1'b0;
        end
        else if (S_idle_cnt < IDLE_FRAMES) begin
            S_idle_cnt <= S_idle_cnt + 16'd1;
            S_det_skip <= 1'b0;
        end
        else begin
            S_lp_sleep <= 1'b1;
            // 补看优先：有证据就跳过"跳帧节拍"，这一帧真跑（相位不推进，查同一路）
            if (S_conf_look) begin
                S_det_skip <= 1'b0;
            end
            // 相位 0/1/2 = 认真跑，3/4 = 跳。必须连续真跑三帧，箭头
            // completed-bank 匹配和灯色投票才能像全速一样成立；空跑不超过
            // 两帧，才不会触发 sign_detect 连续 3 次 miss 的模式切换。
            else begin
                S_lp_ph <= (S_lp_ph == 3'd4) ? 3'd0 : (S_lp_ph + 3'd1);
                if (S_lp_ph >= 3'd3) begin
                    S_det_skip <= 1'b1;
                    S_skip_cnt <= S_skip_cnt + 16'd1;
                end
                else begin
                    S_det_skip <= 1'b0;
                end
            end
        end

        if (S_lp_xfer !== {S_lp_sleep, S_skip_cnt}) begin
            S_lp_xfer <= {S_lp_sleep, S_skip_cnt};
            S_lp_tog  <= ~S_lp_tog;
        end
    end
end

assign O_lp_xfer = S_lp_xfer;
assign O_lp_tog  = S_lp_tog;

endmodule

// ===========================================================================
// 以下三个模块原先各自独立成文件，靠工程文件清单加载。
// 踩过一次坑：TD 重写 .al 时把新增文件的条目删掉，综合就报 black box。
// 现在直接并进 isp_top.v —— 只要这个文件在编译列表里，模块就一定存在，
// 不再依赖任何工程文件登记。
// ===========================================================================

// ===========================================================================
// blk_act_gen.v —— 把 4 像素/拍的掩码聚合成"块级活动向量"
//
// 【做什么】
//   color_mask_96 每拍吐 4 个像素的 0/1 掩码（mask[3] 是第 0 列）。
//   一块是 16x16 像素，即 4 拍 x 16 行。这里对每个块列累加"目标像素个数"，
//   每收满 16 行就结算一次：某块列的像素数 >= BLK_PIX_MIN 才算"这个块有目标"，
//   得到一根 BX 位的活动向量，一拍交给 multi_region。
//
// 【为什么块内要设阈值，而不是"有 1 个像素就算"】
//   颜色掩码本身有噪点（尤其是绿灯/黄灯这种和背景接近的情况）。逐拍
//   "有点就算"会让块级活动图满是孤立噪点，区域引擎就跟着乱。设一个
//   块内像素数下限，等于做了一次 3% 的块级滤波，不用额外的形态学就能
//   把大部分椒盐噪点挡掉。
//
// 【量的对应关系】
//   一行 1280 像素 = 320 拍；一个块列 16 像素 = 4 拍。所以拍号 >> 2 就是块列号。
//   16 行 = 一个块行；720 行 = 45 个块行。
//
// 时钟域：axi4s_video_aclk（60MHz，与 color_mask_96 同域）
// ===========================================================================

`timescale 1ns/1ps

module blk_act_gen #(
    parameter integer BX           = 80,    // 一行多少个块（1280/16）
    parameter integer BY           = 45,    // 一帧多少块行（720/16）
    parameter integer BEAT_PER_BLK = 4,     // 一个块列多少拍（16 像素 / 4 像素每拍）
    parameter integer LINE_OF_BLK  = 16,    // 一个块行多少行
    parameter integer BLK_PIX_MIN  = 12,    // 块内目标像素数下限（0..256）
    parameter integer CNT_W        = 4      // 计数器位宽：饱和在 BLK_PIX_MIN 就够
)(
    input  wire          I_clk,
    input  wire          I_rst_n,
    input  wire          I_tvalid,
    input  wire          I_tuser,     // 帧开始
    input  wire          I_tlast,     // 行结束
    input  wire [3:0]    I_mask,      // 4 像素：mask[3]=第0列 ... mask[0]=第3列
    input  wire [3:0]    I_blk_pix_min, // 块内目标像素数下限（串口可调）

    output reg           O_row_v,     // 一个块行结算完成，只拉高一拍
    output reg [BX-1:0]  O_row_act,   // 该块行的活动向量
    output reg           O_frame_last // 与 O_row_v 同时有效：本帧最后一个块行
);

    // 行内拍计数：需要覆盖 BEAT_PER_BLK*BX = 320 拍
    reg [8:0]  beat_x;
    reg [3:0]  line_i;     // 块行内第几行 0..15
    reg [5:0]  blk_y;      // 第几个块行 0..44

    // 每个块列的目标像素数。★ 这里故意做成"饱和计数器"而不是 0..256 的完整计数：
    // 判据只需要"够不够 BLK_PIX_MIN(12)"，够了就钳住不再加，所以 4 位就够
    // （11+4=15 不会溢出）。位宽从 9 降到 4，80 路读写选择网络直接省掉一半多。
    reg [CNT_W-1:0] blk_cnt [0:BX-1];

    // 注意：Verilog 里"加法表达式"的位宽由最宽的那个操作数决定，写
    // {1'b0,x}+{1'b0,y}+... 只有 2 位，4 个 1 加起来会溢出成 0。
    // 必须把每个操作数先扩到 3 位（最大和 4）。
    wire [2:0] pop  = {2'b00, I_mask[3]} + {2'b00, I_mask[2]} +
                      {2'b00, I_mask[1]} + {2'b00, I_mask[0]};
    wire [6:0] col  = beat_x[8:2];   // >>2：4 拍一个块列

    // 阈值降到 CNT_W 位用（参数是 integer，直接取位段容易越界，这里显式截）
    wire [CNT_W-1:0] BLK_MIN_C = I_blk_pix_min[CNT_W-1:0];

    integer i;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            beat_x      <= 9'd0;
            line_i      <= 4'd0;
            blk_y       <= 6'd0;
            O_row_v     <= 1'b0;
            O_row_act   <= {BX{1'b0}};
            O_frame_last<= 1'b0;
            for (i = 0; i < BX; i = i + 1) blk_cnt[i] <= {CNT_W{1'b0}};
        end
        else begin
            O_row_v <= 1'b0;   // 默认拉低，只在下面结算那一拍拉高

            // ---- 帧首同步一下行/拍计数，防止丢行把块行对齐搞偏 ----
            if (I_tuser) begin
                beat_x <= 9'd0;
            end

            // ---- 每拍累加 4 个像素 ----
            if (I_tvalid) begin
                // pop 是 3 位，直接写 pop[CNT_W-1:0] 会取到不存在的 pop[3]
                // （结果是 x，整条累加链就废了），所以显式零扩展。
                if (blk_cnt[col] < BLK_MIN_C)
                    blk_cnt[col] <= blk_cnt[col] + {1'b0, pop};
                beat_x <= I_tlast ? 9'd0 : (beat_x + 9'd1);
            end

            // ---- 行结束：数够 16 行就结算一个块行 ----
            if (I_tlast) begin
                if (line_i == LINE_OF_BLK-1) begin
                    for (i = 0; i < BX; i = i + 1) begin
                        O_row_act[i] <= (blk_cnt[i] >= BLK_MIN_C);
                        blk_cnt[i]   <= {CNT_W{1'b0}};
                    end
                    O_row_v      <= 1'b1;
                    O_frame_last <= (blk_y == BY-1);
                    line_i       <= 4'd0;
                    blk_y        <= (blk_y == BY-1) ? 6'd0 : (blk_y + 6'd1);
                end
                else begin
                    line_i <= line_i + 4'd1;
                end
            end
        end
    end

endmodule

// ===========================================================================
// multi_region.v —— 块级多目标区域检测（行游程 + 区域生长 + 重叠归并）
//
// 【要解决的问题】
//   原来 blob_stat 一种颜色只给"整帧一个包围盒"（min/max 累加器），
//   所以同色的多个物体会被粘成一个框，"目标数量"也统计不出来。
//
// 【做法】输入是按块（16x16 像素）串行进来的"这一块有没有目标像素"：
//   一行 80 个块，一帧 45 个块行（正好和 motion_diff 的块网格一致）。
//   每收完一个块行，就在这一行里找"游程"（连续为 1 的一段块），
//   每个游程拿去和已存在的区域比：
//     a) x 上有交叠（左右各留 1 块容差），且这个区域在上一块行或本块行
//        刚被更新过     -> 并入它（x0/x1/y1/nblk 一起扩）
//     b) 同时命中多个区域 -> 全部并成一个（防止一个物体被拆成两个框）
//     c) 一个都没命中   -> 开一个新区域槽（槽用完就顶掉块数最少的那个）
//   帧末：丢掉块数 < MIN_BLK 的噪声区域，按块数从大到小排序，输出前 NOUT 个。
//
// 【为什么不用标准两遍连通域标记】
//   标准做法要存标签图 + 等价表，还要帧末多遍扫描，ERAM 和时序都被吃掉。
//   块级（16x16）+ 行游程生长只用寄存器：不占 ERAM、没有多遍扫描、
//   时序风险最低，而演示场景（交通灯、颜色块彼此分开）的包围盒精度够用。
//
// 【老实说的取舍】
//   1. 不是严格连通域：x、y 都重叠但"对角线不相接"的两个物体会被并成一个。
//      本场景（物体之间留有空隙）撞不到；写报告时按"行游程区域生长"描述。
//   2. 区域槽固定 NREG 个，第 NREG+1 个目标会被并进块数最少的槽。
//   3. 包围盒量化到 16 像素（块粒度），不是像素级紧贴。
//
// 【接口时序】
//   块行以"并行向量"方式给进来，不是逐块串行：
//   I_row_v      ：一个块行的活动向量有效，只拉高一拍。
//   I_row_act    ：BX 位向量，第 i 位 = 第 i 个块有没有目标像素。
//   I_frame_last ：和 I_row_v 同时有效，表示这是本帧最后一个块行。
//   为什么用并行：块行之间隔 16 个行周期（约 5120 拍），而本模块扫一行最多
//   几百拍，余量足够；反过来用"80 拍串行"，块行边界会和下一行的行消隐/数据
//   挤在一起，容易踩时序。I_row_v 到来时向量会被立刻拷进内部 rowbits。
//
// 时钟域：axi4s_video_aclk（60MHz）
// ===========================================================================

`timescale 1ns/1ps

module multi_region #(
    parameter integer BX      = 80,   // 一行多少块
    parameter integer BY      = 45,   // 多少块行
    parameter integer NREG    = 6,    // 内部区域槽数
    parameter integer NOUT    = 3,    // 输出目标个数上限
    parameter integer RUNTIME_MIN=0,
    parameter integer SHAPE_ENABLE=1,
    parameter integer MIN_BLK = 4,    // 块数下限，低于它算噪声
    parameter integer FILL_NUM = 1,   // 填充率下限的分子：nblk*FILL_DEN >= w*h
    parameter integer FILL_DEN = 4,   // 默认 1/4，即包围盒里至少 25% 是目标块
    parameter integer BLK_SHF = 4,    // 块边长 = 2^BLK_SHF 像素
    parameter integer NOUT_W  = 4     // 目标个数位宽
)(
    input  wire              I_clk,
    input  wire              I_rst_n,
    input  wire              I_row_v,
    input  wire [BX-1:0]     I_row_act,
    input  wire [3:0]        I_min_blk,   // 区域块数下限（串口可调）
    input  wire              I_frame_last,

    output reg               O_valid,     // 帧末更新：上一帧是否找到目标
    output reg               O_done,      // 扫描排序全部结束，只拉高一拍（快照握手用）
    output reg  [NOUT_W-1:0] O_num,       // 目标个数 0..NOUT
    // NOUT 个目标，每个 64bit：
    //   { valid(1), nblk(15), x0(12), y0(12), x1(12), y1(12) }  —— 像素坐标
    // O_tgt[63:0] 是块数最多的那个目标
    output wire [NOUT*64-1:0] O_tgt,
    // 每个输出目标的形状判据：1 = 圆形。与 O_tgt 同一次排序输出。
    output reg  [NOUT-1:0]    O_shape,
    // 本帧最高置信度 0~9（填充率分级）。没有目标时为 0。
    output reg  [3:0]         O_conf,
    output wire O_row_ready
);

    localparam [6:0] BXLAST = BX - 1;

    // ---------------- 状态机 ----------------
    localparam [2:0] S_WAIT  = 3'd0,
                     S_SCAN  = 3'd1,
                     S_MATCH = 3'd2,
                     S_PMERG = 3'd3,
                     S_UPD   = 3'd4,
                     S_FIN   = 3'd5,
                     S_SORT  = 3'd6;

    reg [2:0] st;
    assign O_row_ready=(st==S_WAIT);

    // ---------------- 输入流 ----------------
    reg [5:0]    row_y;      // 第几个块行
    reg          frm_last;   // 本帧最后一个块行标记
    reg [BX-1:0] rowbits;    // 本块行的活动位图

    // ---------------- 游程扫描 ----------------
    reg [6:0] scan_i;
    reg       run_on;
    reg [6:0] a, b;          // 当前游程 [a,b]

    // ---------------- 匹配 ----------------
    reg [NREG-1:0] hit;
    reg [3:0]      n_hit;
    reg [2:0]      first_hit;
    reg [2:0]      k;
    reg [2:0]      sel;
    reg [2:0]      sel_now;
    reg            free_found;

    // ---------------- 区域表（纯寄存器） ----------------
    reg [6:0]  rg_x0 [0:NREG-1];
    reg [6:0]  rg_x1 [0:NREG-1];
    reg [5:0]  rg_y0 [0:NREG-1];
    reg [5:0]  rg_y1 [0:NREG-1];
    reg [11:0] rg_nb [0:NREG-1];
    reg        rg_v  [0:NREG-1];
    // 形状判据结果（帧末 S_FIN 里算好，S_SORT 直接搬出去）
    reg        rg_circ [0:NREG-1];
    // 行宽剖面：首行宽 / 末行宽 / 最宽行宽（块数）。
    // 圆是上下窄、中间最宽；三角是单调变宽（首+末 >= 最宽）。
    // 单靠填充率在小目标上分不开圆和三角，要靠这个剖面。
    reg [7:0]  rg_wfirst [0:NREG-1];
    reg [7:0]  rg_wlast  [0:NREG-1];
    reg [7:0]  rg_wmax   [0:NREG-1];
    // 帧末定格：最后一个块行扫完那一拍，把算形状/置信度要用的量
    // 从区域表打进平坦寄存器。这样后面的乘加链从寄存器出发，
    // 不含"6 选 1 数组读出 + 减法"，ISP 域时序能省一大截。
    reg [11:0] fin_nb_r  [0:NREG-1];
    reg [7:0]  fin_w_r   [0:NREG-1];
    reg [7:0]  fin_h_r   [0:NREG-1];

    // ---------------- 输出暂存 ----------------
    reg [NOUT_W-1:0] out_cnt;
    reg [2:0]        srt_i;
    reg              srt_any;
    reg [2:0]        srt_best;
    reg [2:0]        srt_sel;
    reg              srt_have;
    reg [63:0]       tgt_w;
    reg [63:0]       tgt0, tgt1, tgt2, tgt3;

    wire [6:0] run_len = b - a + 7'd1;

    assign O_tgt = {tgt3, tgt2, tgt1, tgt0};

    integer m;
    reg     hit_v;
    reg [7:0]  fill_w, fill_h;
    // 帧末统计：真正通过滤波的区域个数（不能按 NOUT 截断）
    reg [3:0]  fin_cnt;
    reg        drop_k;
    reg [3:0]  cnt_now;
    // ---- 形状判据（圆形）用临时量 ----
    reg [11:0] fin_area;
    reg [15:0] circ_n20, circ_a11, circ_n10, circ_a9;
    reg [15:0] circ_a17, circ_a13, circ_a3, circ_n4;
    reg [8:0]  circ_prof;
    reg        circ_ok;
    // ---- 置信度用 ----
    reg [3:0]  conf_now, conf_max, conf_next;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            st         <= S_WAIT;
            row_y      <= 6'd0;
            frm_last   <= 1'b0;
            rowbits    <= {BX{1'b0}};
            scan_i     <= 7'd0;
            run_on     <= 1'b0;
            a          <= 7'd0;
            b          <= 7'd0;
            hit        <= {NREG{1'b0}};
            n_hit      <= 4'd0;
            first_hit  <= 3'd0;
            k          <= 3'd0;
            sel        <= 3'd0;
            sel_now    <= 3'd0;
            free_found <= 1'b0;
            out_cnt    <= {NOUT_W{1'b0}};
            srt_i      <= 3'd0;
            srt_any    <= 1'b0;
            srt_best   <= 3'd0;
            srt_sel    <= 3'd0;
            srt_have   <= 1'b0;
            O_valid    <= 1'b0;
            O_done     <= 1'b0;
            O_num      <= {NOUT_W{1'b0}};
            O_shape    <= {NOUT{1'b0}};
            O_conf     <= 4'd0;
            tgt0       <= 64'd0;
            tgt1       <= 64'd0;
            tgt2       <= 64'd0;
            tgt3       <= 64'd0;
            for (m = 0; m < NREG; m = m + 1) begin
                rg_x0[m] <= 7'd0;
                rg_x1[m] <= 7'd0;
                rg_y0[m] <= 6'd0;
                rg_y1[m] <= 6'd0;
                rg_nb[m] <= 12'd0;
                rg_v [m] <= 1'b0;
                rg_circ[m] <= 1'b0;
                rg_wfirst[m] <= 8'd0;
                rg_wlast [m] <= 8'd0;
                rg_wmax  [m] <= 8'd0;
                fin_nb_r [m] <= 12'd0;
                fin_w_r  [m] <= 8'd0;
                fin_h_r  [m] <= 8'd0;
            end
        end
        else begin
            O_done <= 1'b0;   // 默认拉低，只在排序收尾那一拍拉高

            // ---------- 输入流：收一个块行的活动向量 ----------
            // 立刻整份拷进 rowbits，前端不需要保持；同时启动本行的扫描。
            if (I_row_v) begin
                rowbits  <= I_row_act;
                frm_last <= I_frame_last;
                st       <= S_SCAN;
                scan_i   <= 7'd0;
                run_on   <= 1'b0;
            end

            case (st)
            // 等待下一行：什么都不做，让上面输入流那段的 st<=S_SCAN 生效
            // （同一个 always 块里对同一个 reg 的后一次赋值会覆盖前一次，
            //   所以这里绝对不能写 st <= S_WAIT）
            S_WAIT: begin end

            // ---------------- 逐块扫一行，切出游程 ----------------
            S_SCAN: begin
                if (scan_i >= BX) begin
                    // 本行扫完（scan_i 走到 BX 说明后面没有块了）
                    if (frm_last) begin
                        st      <= S_FIN;
                        k       <= 3'd0;
                        fin_cnt <= 4'd0;
                        out_cnt <= {NOUT_W{1'b0}};
                        O_valid <= 1'b0;
                        O_num   <= {NOUT_W{1'b0}};
                        O_conf  <= 4'd0;
                        conf_max <= 4'd0;
                        // 区域表已定格：打进平坦寄存器
                        for (m = 0; m < NREG; m = m + 1) begin
                            fin_nb_r[m] <= rg_nb[m];
                            fin_w_r [m] <= {1'b0, rg_x1[m]} - {1'b0, rg_x0[m]} + 8'd1;
                            fin_h_r [m] <= {1'b0, rg_y1[m]} - {1'b0, rg_y0[m]} + 8'd1;
                        end
                        tgt0    <= 64'd0;
                        tgt1    <= 64'd0;
                        tgt2    <= 64'd0;
                        tgt3    <= 64'd0;
                    end
                    else begin
                        row_y <= row_y + 6'd1;
                        st    <= S_WAIT;
                    end
                end
                else if (rowbits[scan_i]) begin
                    if (!run_on) begin
                        run_on <= 1'b1;
                        a      <= scan_i;
                    end
                    b      <= scan_i;
                    // 关键：扫过的位置一定要前进，游程正好顶到最后一个块时
                    // 走到 BX，下一次进来才会判定"本行扫完"，
                    // 否则最后一个块会被反复当成新游程，状态机死循环。
                    scan_i <= scan_i + 7'd1;
                    if (scan_i == BXLAST) begin
                        run_on    <= 1'b0;
                        st        <= S_MATCH;
                        k         <= 3'd0;
                        n_hit     <= 4'd0;
                        hit       <= {NREG{1'b0}};
                        first_hit <= 3'd0;
                    end
                end
                else begin
                    if (run_on) begin
                        // 游程 [a,b] 到此结束，去匹配
                        run_on    <= 1'b0;
                        st        <= S_MATCH;
                        k         <= 3'd0;
                        n_hit     <= 4'd0;
                        hit       <= {NREG{1'b0}};
                        first_hit <= 3'd0;
                    end
                    else begin
                        scan_i <= scan_i + 7'd1;
                    end
                end
            end

            // ---------------- 逐个区域比命中 ----------------
            S_MATCH: begin
                hit_v  = rg_v[k] &&
                         (row_y    <= rg_y1[k] + 6'd1) &&
                         (a        <= rg_x1[k] + 7'd1) &&
                         (rg_x0[k] <= b        + 7'd1);
                hit[k] <= hit_v;
                if (k == NREG-1) begin
                    k  <= 3'd0;
                    st <= S_PMERG;
                end
                else begin
                    k <= k + 3'd1;
                end
            end

            // ---------------- 命中多个就先归并 ----------------
            S_PMERG: begin
                if(hit[k])begin
                    if(n_hit==0)first_hit<=k;
                    else begin
                        if(rg_x0[k]<rg_x0[first_hit])rg_x0[first_hit]<=rg_x0[k];
                        if(rg_x1[k]>rg_x1[first_hit])rg_x1[first_hit]<=rg_x1[k];
                        if(rg_y0[k]<rg_y0[first_hit])rg_y0[first_hit]<=rg_y0[k];
                        if(rg_y1[k]>rg_y1[first_hit])rg_y1[first_hit]<=rg_y1[k];
                        rg_nb[first_hit]<=rg_nb[first_hit]+rg_nb[k];
                        rg_v[k]<=0;
                        if(rg_wmax[k]>rg_wmax[first_hit])rg_wmax[first_hit]<=rg_wmax[k];
                    end
                    n_hit<=n_hit+1'b1;
                end
                if(k==NREG-1)begin st<=S_UPD;k<=0;sel<=0;free_found<=0;end
                else k<=k+1'b1;
            end

            // ---------------- 扩展命中的区域 / 开新区域 ----------------
            S_UPD: begin
                if (n_hit != 4'd0) begin
                    // 命中：扩到 first_hit，一拍搞定
                    if (a < rg_x0[first_hit]) rg_x0[first_hit] <= a;
                    if (b > rg_x1[first_hit]) rg_x1[first_hit] <= b;
                    if (row_y > rg_y1[first_hit]) rg_y1[first_hit] <= row_y;
                    rg_nb[first_hit] <= rg_nb[first_hit] + {5'd0, run_len};
                    rg_v [first_hit] <= 1'b1;
                    // 行宽剖面：末行宽 = 本次游程长；最宽行取历史最大
                    rg_wlast[first_hit] <= {1'b0, run_len};
                    if ({1'b0, run_len} > rg_wmax[first_hit])
                        rg_wmax[first_hit] <= {1'b0, run_len};
                    k                <= 3'd0;
                    st               <= S_SCAN;
                end
                else begin
                    // 没命中：优先找空槽；没有空槽才顶掉块数最少的那个。
                    // 注意必须在"本拍"就算出结果，不能靠下一拍的非阻塞赋值，
                    // 否则第 8 个槽（k=NREG-1）的判断会迟到一拍。
                    sel_now = free_found ? sel :
                              (!rg_v[k]  ? k   :
                               ((rg_nb[k] < rg_nb[sel]) ? k : sel));
                    if (!free_found && !rg_v[k]) begin
                        free_found <= 1'b1;
                    end
                    sel <= sel_now;
                    if (k == NREG-1) begin
                        rg_x0[sel_now] <= a;
                        rg_x1[sel_now] <= b;
                        rg_y0[sel_now] <= row_y;
                        rg_y1[sel_now] <= row_y;
                        rg_nb[sel_now] <= {5'd0, run_len};
                        rg_v [sel_now] <= 1'b1;
                        rg_wfirst[sel_now] <= {1'b0, run_len};
                        rg_wlast [sel_now] <= {1'b0, run_len};
                        rg_wmax  [sel_now] <= {1'b0, run_len};
                        k          <= 3'd0;
                        st         <= S_SCAN;
                    end
                    else begin
                        k <= k + 3'd1;
                    end
                end
            end

            // ---------------- 帧末：丢噪声 ----------------
            S_FIN: begin
                // 两条噪声判据：
                //   ① 块数太少（小块、零星噪点）
                //   ② 填充率太低：块数*FILL_DEN < 包围盒块面积*FILL_NUM
                //      —— 散落的噪点被行游程"合并"成一个大框时，框很大但里面
                //         几乎没东西，这条能直接挡掉（不需要除法）
                // ---- 形状 / 置信度：全部从"帧末定格寄存器"出发 ----
                fill_w = fin_w_r[k];
                fill_h = fin_h_r[k];
                drop_k = rg_v[k] &&
                         ((fin_nb_r[k] < (RUNTIME_MIN ? I_min_blk : MIN_BLK)) ||
                          ((fin_nb_r[k] * FILL_DEN) < (fill_w * fill_h * FILL_NUM)));
                if (drop_k)
                    rg_v[k] <= 1'b0;

                // ---- 形状判据：圆形 ----
                // 三条判据一起用，缺一条都不行：
                //  (1) 包围盒近似方正：块宽高差 <= 1
                //  (2) 填充率在 [0.55, 0.90]：分掉实心方形(约 1.0)
                //  (3) 行宽剖面 首行+末行 <= 最宽行：分掉三角形
                //      圆上下窄、中间最宽；三角一路变宽，首+末必然 >= 最宽。
                //      * 小目标上单靠 (2) 分不开：半径 r 的圆格点填充率是
                //        pi*r^2/(2r+1)^2，r=4 时只有 0.62，而直角三角能爬到 0.57。
                // 全是移位/小加法，不做通用乘法，也不做除法。
                fin_area  = {4'd0, fill_w} * {4'd0, fill_h};
                circ_n20  = ({4'd0, fin_nb_r[k]} << 4) + ({4'd0, fin_nb_r[k]} << 2);
                circ_a11  = ({4'd0, fin_area} << 3) + ({4'd0, fin_area} << 1) + {4'd0, fin_area};
                circ_n10  = ({4'd0, fin_nb_r[k]} << 3) + ({4'd0, fin_nb_r[k]} << 1);
                circ_a9   = ({4'd0, fin_area} << 3) + {4'd0, fin_area};
                circ_prof = {1'b0, rg_wfirst[k]} + {1'b0, rg_wlast[k]};
                circ_ok   = (fill_w + 8'd1 >= fill_h) && (fill_h + 8'd1 >= fill_w) &&
                            (circ_n20 >= circ_a11) && (circ_n10 <= circ_a9) &&
                            (circ_prof <= {1'b0, rg_wmax[k]});
                rg_circ[k] <= SHAPE_ENABLE && circ_ok && rg_v[k] && !drop_k;

                // ---- 置信度：填充率分级，档位 9/7/5/3/1 ----
                //   填充率 0.85 -> 9   0.75 -> 7   0.65 -> 5   0.55 -> 3   其余 -> 1
                //   复用上面已经算出的 circ_n20 / circ_a11，只多 4 个小常数积。
                circ_a17  = ({4'd0, fin_area} << 4) + {4'd0, fin_area};
                circ_a13  = ({4'd0, fin_area} << 3) + ({4'd0, fin_area} << 2) + {4'd0, fin_area};
                circ_a3   = ({4'd0, fin_area} << 1) + {4'd0, fin_area};
                circ_n4   = {2'd0, fin_nb_r[k], 2'b00};
                if      (circ_n20 >= circ_a17) conf_now = 4'd9;
                else if (circ_n4  >= circ_a3 ) conf_now = 4'd7;
                else if (circ_n20 >= circ_a13) conf_now = 4'd5;
                else if (circ_n20 >= circ_a11) conf_now = 4'd3;
                else                           conf_now = 4'd1;
                conf_next = ((!drop_k && rg_v[k]) && (conf_now > conf_max))
                            ? conf_now : conf_max;
                // ★ 个数按"通过滤波的区域数"统计，不是"输出几个框"。
                //   之前 O_num 被 NOUT(=3) 截断，画面里超过 3 个时显示就是错的。
                cnt_now = fin_cnt + ((!drop_k && rg_v[k]) ? 4'd1 : 4'd0);
                if (k == NREG-1) begin
                    st       <= S_SORT;
                    k        <= 3'd0;
                    srt_i    <= 3'd0;
                    srt_any  <= 1'b0;
                    srt_best <= 3'd0;
                    out_cnt  <= {NOUT_W{1'b0}};
                    O_num    <= cnt_now;
                    O_valid  <= (cnt_now != 4'd0);
                    O_conf   <= SHAPE_ENABLE ? conf_next : 4'd0;   // 本帧最高置信度
                end
                else begin
                    fin_cnt  <= cnt_now;
                    conf_max <= conf_next;
                    k <= k + 3'd1;
                end
            end

            // ---------------- 帧末：按块数从大到小挑 NOUT 个 ----------------
            S_SORT: begin
                // 本拍这个 k（含 k=NREG-1）也纳入候选
                if (rg_v[k] && ((!srt_any) || (rg_nb[k] > rg_nb[srt_best]))) begin
                    srt_sel  = k;
                    srt_have = 1'b1;
                end
                else begin
                    srt_sel  = srt_best;
                    srt_have = srt_any;
                end

                if (k == NREG-1) begin
                    if (srt_have) begin
                        // 注意：拼接表达式里每个操作数是"自定宽"的，
                        // 直接写 rg_x0<<4 会按 rg_x0 的位宽（7bit）截断！
                        // 所以先零扩展到 12bit 再移位。
                        tgt_w = {1'b1,
                                 {3'd0, rg_nb[srt_sel]},
                                 ({5'd0, rg_x0[srt_sel]} << BLK_SHF),
                                 ({6'd0, rg_y0[srt_sel]} << BLK_SHF),
                                 ((({5'd0, rg_x1[srt_sel]} + 12'd1) << BLK_SHF) - 12'd1),
                                 ((({6'd0, rg_y1[srt_sel]} + 12'd1) << BLK_SHF) - 12'd1)};
                        case (srt_i)
                            3'd0: tgt0 <= tgt_w;
                            3'd1: tgt1 <= tgt_w;
                            3'd2: tgt2 <= tgt_w;
                            default: tgt3 <= tgt_w;
                        endcase
                        O_shape[srt_i] <= rg_circ[srt_sel];
                        rg_v[srt_sel] <= 1'b0;
                        out_cnt       <= out_cnt + 4'd1;
                    end
                    if ((srt_i == NOUT-1) || (!srt_have)) begin
                        // O_num / O_valid 已经在 S_FIN 里按"通过滤波的区域数"定好，
                        // 这里只负责把最大的 NOUT 个框填进 tgt0..tgt3
                        // ---- 没填满的槽要清掉 ----
                        // 原来 tgt1/tgt2 不填就保留上一帧的值（valid 还是 1），
                        // HDMI 域会把已经消失的目标画成"幽灵框"。
                        if (!srt_have) begin
                            case (srt_i)
                                3'd0: begin tgt0 <= 64'd0; O_shape[0] <= 1'b0; end
                                3'd1: begin tgt1 <= 64'd0; O_shape[1] <= 1'b0; end
                                default: begin tgt2 <= 64'd0; O_shape[2] <= 1'b0; end
                            endcase
                        end
                        if (srt_i < 3'd1) begin tgt1 <= 64'd0; O_shape[1] <= 1'b0; end
                        if (srt_i < 3'd2) begin tgt2 <= 64'd0; O_shape[2] <= 1'b0; end
                        O_done  <= 1'b1;
                        for (m = 0; m < NREG; m = m + 1) rg_v[m] <= 1'b0;
                        row_y <= 6'd0;
                        st    <= S_WAIT;
                    end
                    else begin
                        srt_i    <= srt_i + 3'd1;
                        k        <= 3'd0;
                        srt_any  <= 1'b0;
                        srt_best <= 3'd0;
                    end
                end
                else begin
                    srt_best <= srt_sel;
                    srt_any  <= srt_have;
                    k <= k + 3'd1;
                end
            end

            default: st <= S_WAIT;
            endcase
        end
    end

endmodule

// ===========================================================================
// color_mask_thr_96.v —— 阈值可在线改的色掩码（只给颜色块检测那一路用）
//
// 和 color_mask_96 判据完全一样，区别只有一个：7 个阈值是"输入端口"而不是
// parameter，所以可以接串口下发的寄存器。交通灯那一路仍然用 color_mask_96
// 的编译期常量，互不影响。
//
// mask 位序与 color_mask_96 一致：mask[3] = 第 0 列 ... mask[0] = 第 3 列
// ===========================================================================
`timescale 1ns/1ps
