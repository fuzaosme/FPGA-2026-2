// ===========================================================================
// blob_stat.v   （时序优化版，基于 2026-10-06 22:00 的工程版本）
//
// M2：单目标包围盒统计
// 在 96bit（4 像素/拍）流上跟踪一帧内"目标颜色像素"的
//   min_x / max_x / min_y / max_y、像素数 cnt。
// 每个新帧的 tuser（帧开始拍）到来时，锁存上一帧的结果并复位本帧累计。
//
// I_mask[3..0] 与列的关系（和 color_mask_96 的 O_mask 一致）：
//   mask[3] = 第 0 列，mask[2] = 第 1 列，mask[1] = 第 2 列，mask[0] = 第 3 列
//
// ★ 相对原版只做两处时序优化，功能完全不变：
//   ① 入口打拍：把 I_mask 与 I_tvalid/I_tuser/I_tlast 一起寄存，
//      长链被切成 "color_mask(组合) → 寄存器" 和 "寄存器 → min/max 树" 两段。
//      四路伴随信号同时延迟 1 拍，坐标与掩码严格保持对齐，不会错位。
//      —— 实测原版这条路径是 MIPI 60MHz 域最差路径（SWNS 仅 0.136ns）。
//   ② min/max 由 4 级级联比较改成 2 级平衡树。
//
// ★ 保留了 O_cnt 输出（light_detect.v 用它算填充率），不能删。
// ===========================================================================

`timescale 1ns/1ps

module blob_stat #(
    parameter IMG_WIDTH  = 1280,
    parameter IMG_HEIGHT = 720,
    parameter [19:0] MIN_PIX = 20'd200   // 目标像素数下限，低于它视为噪声
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_tvalid,
    input  wire        I_tuser,          // 帧开始
    input  wire        I_tlast,          // 行结束
    input  wire [3:0]  I_mask,
    output reg         O_box_valid,
    output reg  [11:0] O_x0,
    output reg  [11:0] O_y0,
    output reg  [11:0] O_x1,
    output reg  [11:0] O_y1,
    output reg  [19:0] O_cnt
);

    // ---- ① 入口打拍：掩码与伴随信号一起延迟 1 拍 ----
    reg        r_tvalid, r_tuser, r_tlast;
    reg [3:0]  r_mask;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            r_tvalid <= 1'b0;
            r_tuser  <= 1'b0;
            r_tlast  <= 1'b0;
            r_mask   <= 4'd0;
        end
        else begin
            r_tvalid <= I_tvalid;
            r_tuser  <= I_tuser;
            r_tlast  <= I_tlast;
            r_mask   <= I_mask;
        end
    end

    reg [11:0] col_base;
    reg [11:0] row;
    reg [19:0] cnt;
    reg [11:0] min_x, max_x, min_y, max_y;

    // 本拍的"真实坐标"：tuser 拍是第 0 行第 0 列，其它拍用已累计的计数器。
    wire [11:0] cur_col = r_tuser ? 12'd0 : col_base;
    wire [11:0] cur_row = r_tuser ? 12'd0 : row;

    wire [2:0] n_hit = {2'd0, r_mask[0]} + {2'd0, r_mask[1]} +
                       {2'd0, r_mask[2]} + {2'd0, r_mask[3]};
    wire any_hit = |r_mask;

    // col_base starts at zero and advances by four, including counter wrap.
    // The four x positions are ordered and share their high ten bits. Encode
    // the first/last hit instead of comparing four twelve-bit candidates.
    wire [1:0] first_lane = r_mask[3] ? 2'd0 : r_mask[2] ? 2'd1 :
                            r_mask[1] ? 2'd2 : 2'd3;
    wire [1:0] last_lane = r_mask[0] ? 2'd3 : r_mask[1] ? 2'd2 :
                           r_mask[2] ? 2'd1 : 2'd0;
    wire [11:0] min_hit = any_hit ? {cur_col[11:2], first_lane} : 12'hfff;
    wire [11:0] max_hit = any_hit ? {cur_col[11:2], last_lane} : 12'd0;

    // 累计基值：tuser 拍从头开始，其它拍在当前累计值上继续。
    wire [11:0] base_min_x = r_tuser ? 12'hFFF : min_x;
    wire [11:0] base_max_x = r_tuser ? 12'h000 : max_x;
    wire [11:0] base_min_y = r_tuser ? 12'hFFF : min_y;
    wire [11:0] base_max_y = r_tuser ? 12'h000 : max_y;
    wire [19:0] base_cnt   = r_tuser ? 20'd0   : cnt;

    wire [11:0] min_x_new = (min_hit < base_min_x) ? min_hit : base_min_x;
    wire [11:0] max_x_new = (max_hit > base_max_x) ? max_hit : base_max_x;
    wire [11:0] min_y_new = any_hit ? ((base_min_y < cur_row) ? base_min_y : cur_row) : base_min_y;
    wire [11:0] max_y_new = any_hit ? ((base_max_y > cur_row) ? base_max_y : cur_row) : base_max_y;

    // ---- 帧边界锁存上一帧结果（读的是本拍还没被更新的累计值） ----
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_box_valid <= 1'b0;
            O_x0 <= 12'd0;
            O_y0 <= 12'd0;
            O_x1 <= 12'd0;
            O_y1 <= 12'd0;
            O_cnt <= 20'd0;
        end
        else if (r_tuser) begin
            O_x0 <= min_x;
            O_y0 <= min_y;
            O_x1 <= max_x;
            O_y1 <= max_y;
            O_cnt <= cnt;
            O_box_valid <= (cnt >= MIN_PIX);
        end
    end

    // ---- 逐拍累计包围盒 ----
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            col_base <= 12'd0;
            row      <= 12'd0;
            cnt      <= 20'd0;
            min_x    <= 12'hFFF;
            max_x    <= 12'd0;
            min_y    <= 12'hFFF;
            max_y    <= 12'd0;
        end
        else if (r_tvalid || r_tuser) begin
            cnt   <= base_cnt + n_hit;
            min_x <= min_x_new;
            max_x <= max_x_new;
            min_y <= min_y_new;
            max_y <= max_y_new;

            if (r_tuser) begin
                // tuser 拍可能和 valid 同时来（仿真），也可能早 1 拍（真板 AWB）
                col_base <= r_tvalid ? 12'd4 : 12'd0;
                row      <= 12'd0;
            end
            else if (r_tlast) begin
                col_base <= 12'd0;
                row      <= row + 12'd1;
            end
            else begin
                col_base <= col_base + 12'd4;
            end
        end
    end

endmodule

// Run-based bounded localization for recognition. A full-ROI scan chooses the
// longest compact colour run, seeding the following frame's search window.
// Statistics use complete runs (>=4 colour pixels), preserving four-lane edge
// coordinates while excluding isolated speckles. There is no permanent ROI
// lock: the seed scan always sees the full configured ROI. Larger competing
// same-colour objects can still win; this is not a general component labeler.
module color_component_stat #(
 parameter IMG_WIDTH=1280,IMG_HEIGHT=720,
 parameter SHAPE_ENABLE=0,
 parameter [19:0] MIN_PIX=40,
 parameter [11:0] MIN_DIM=12,MAX_DIM=640
)(input wire I_clk,I_rst_n,I_tvalid,I_tuser,I_tlast,
 input wire [3:0] I_mask,
 output reg O_box_valid,output reg [11:0] O_x0,O_y0,O_x1,O_y1,
 output reg [19:0] O_cnt,output wire [19:0] O_envelope,output wire O_taper);
 reg v,u,l;reg [3:0] mask;
 reg [11:0] x,y,run_start,run_end,run_count;
 reg running;
 reg [11:0] peak_width,peak_x,peak_y,seed_width,seed_x,seed_y;
 reg [11:0] min_x,max_x,min_y,max_y;
 reg [19:0] count;
 wire [11:0] xx=u ? 12'd0 : x,yy=u ? 12'd0 : y;
 wire [2:0] hits={2'd0,mask[3]}+{2'd0,mask[2]}+{2'd0,mask[1]}+{2'd0,mask[0]};
 wire [1:0] first=mask[3] ? 0 : mask[2] ? 1 : mask[1] ? 2 : 3;
 wire [1:0] last=mask[0] ? 3 : mask[1] ? 2 : mask[2] ? 1 : 0;
 wire [11:0] rx0=running ? run_start : {xx[11:2],first};
 wire [11:0] rx1=(hits!=0) ? {xx[11:2],last} : run_end;
 wire [12:0] width={1'b0,rx1}-{1'b0,rx0}+13'd1;
 wire [12:0] rc=(running ? {1'b0,run_count} : 13'd0)+hits;
 wire finish=v && !u && (running || hits!=0) && (hits==0 || l);
 wire run_ok=width<=MAX_DIM && rc>=4;
 wire [12:0] mid=({1'b0,rx0}+{1'b0,rx1})>>1;
 wire [12:0] radius_x={1'b0,seed_width},radius_y={seed_width,1'b0};
 wire [12:0] xlo=({1'b0,seed_x}>radius_x) ? {1'b0,seed_x}-radius_x : 13'd0;
 wire [12:0] ylo=({1'b0,seed_y}>radius_y) ? {1'b0,seed_y}-radius_y : 13'd0;
 wire [12:0] xhi={1'b0,seed_x}+radius_x,yhi={1'b0,seed_y}+radius_y;
 wire in_window=(seed_width==0) || ({1'b0,rx0}>=xlo && {1'b0,rx1}<=xhi &&
  {1'b0,yy}>=ylo && {1'b0,yy}<=yhi);
 // Sum the OUTER horizontal span of each selected colour row. A white
 // lamp centre removes colour pixels but not its outer silhouette. Diamonds
 // and triangles have about half-box span area; squares/rectangle rims have
 // full-box span area. Near-circular disks/rims are between these bounds.
 generate if(SHAPE_ENABLE)begin: shape
  reg row_has;reg [11:0] row_left,row_right;
  reg [19:0] envelope,completed_envelope;
  reg seen_row,first_cap,last_cap,completed_taper;
  // Runs of >=2 colour pixels contribute to the envelope so a thin rim keeps
  // its silhouette even when a row splits around a white centre. Isolated
  // single pixels are still ignored, and the 31/32 upper bound in
  // light_detect keeps a filled square/rim (~1.0) rejected.
  // Restrict silhouette rows to the previous counted box: a static target
  // keeps its rows, while noise rows above/below the lamp can no longer
  // inflate the envelope on a dim, noisy frame.
  wire row_in_box=(O_y0<=O_y1)&&yy>=O_y0&&yy<=O_y1;
  wire take_run=finish&&width<=MAX_DIM&&rc>=2&&in_window&&row_in_box;
  wire has=row_has||take_run;
  wire [11:0] left_edge=take_run&&(!row_has||rx0<row_left)?rx0:row_left;
  wire [11:0] right_edge=take_run&&(!row_has||rx1>row_right)?rx1:row_right;
  wire [12:0] span=has?{1'b0,right_edge}-{1'b0,left_edge}+13'd1:13'd0;
  assign O_envelope=completed_envelope;
  assign O_taper=completed_taper;
  // Cap taper rejects the thick plus/cross silhouette that the envelope fill
  // alone cannot separate from a disk. It is skipped below 16 px: a very small
  // ring's top chord is a large fraction of its box, and size/aspect already
  // bound such blobs. Uses the previous counted box, so it adds one frame.
  wire [12:0] reference_width={1'b0,O_x1}-{1'b0,O_x0}+13'd1;
  wire cap_ok=(reference_width<13'd16)||
   ({span,3'd0}<={reference_width,1'b0}+{1'b0,reference_width});
  // Sum the outer horizontal span of each counted colour row. A white lamp
  // centre removes colour pixels but not the outer silhouette. A circle gives
  // pi/4 (~0.785) of the box area, a triangle or diamond ~0.5, a square or
  // rectangular rim ~1.0, so the 5/8..15/16 band is a near-circular test that
  // needs no per-row memory and no previous-frame geometry.
  always @(posedge I_clk or negedge I_rst_n)begin
   if(!I_rst_n)begin row_has<=0;row_left<=0;row_right<=0;envelope<=0;completed_envelope<=0;seen_row<=0;first_cap<=0;last_cap<=0;completed_taper<=0;end
   else if(u)begin completed_envelope<=envelope;envelope<=0;row_has<=0;completed_taper<=seen_row&&first_cap&&last_cap;seen_row<=0;first_cap<=0;last_cap<=0;end
   else if(v)begin
    if(take_run)begin row_has<=1;row_left<=left_edge;row_right<=right_edge;end
    if(l)begin
     envelope<=envelope+span;row_has<=0;
     if(has)begin if(!seen_row)first_cap<=cap_ok;last_cap<=cap_ok;seen_row<=1;end
    end
   end
  end
 end else begin: no_shape
  assign O_envelope=0;assign O_taper=1;
 end endgenerate
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin
   v<=0;u<=0;l<=0;mask<=0;x<=0;y<=0;run_start<=0;run_end<=0;run_count<=0;running<=0;
   peak_width<=0;peak_x<=0;peak_y<=0;seed_width<=0;seed_x<=0;seed_y<=0;
   min_x<=12'hfff;max_x<=0;min_y<=0;max_y<=0;count<=0;
   O_box_valid<=0;O_x0<=0;O_y0<=0;O_x1<=0;O_y1<=0;O_cnt<=0;
  end else begin
   v<=I_tvalid;u<=I_tuser;l<=I_tlast;mask<=I_mask;
   if(u)begin
    O_box_valid<=count>=MIN_PIX;O_cnt<=count;
    O_x0<=min_x;O_y0<=min_y;O_x1<=max_x;O_y1<=max_y;
    count<=0;min_x<=12'hfff;max_x<=0;min_y<=0;max_y<=0;
    seed_width<=peak_width;seed_x<=peak_x;seed_y<=peak_y;peak_width<=0;
    x<=v ? 12'd4 : 12'd0;y<=0;
    running<=v && hits!=0 && !l;
    run_start<={10'd0,first};run_end<={10'd0,last};run_count<=v ? hits : 0;
   end else if(v)begin
    if(l)begin x<=0;y<=y+1'b1;end else x<=x+12'd4;
    if(hits!=0)begin
     if(!running)run_start<={xx[11:2],first};
     run_end<={xx[11:2],last};run_count<=rc[11:0];running<=1;
    end
    if(finish)begin
     running<=0;run_count<=0;
     if(run_ok)begin
      if(width>peak_width)begin peak_width<=width[11:0];peak_x<=mid[11:0];peak_y<=yy;end
      if(in_window)begin
       count<=count+rc;
       if(rx0<min_x)min_x<=rx0;
       if(rx1>max_x)max_x<=rx1;
       if(count==0)min_y<=yy;
       max_y<=yy;
      end
     end
    end
   end
  end
 end
endmodule
