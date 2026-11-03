// ===========================================================================
// buzzer_ctrl.v   （2026-10-12 扩展：加入"红灯 + 有车在动"事件）
// 本文件里现在有两个模块（都跑在 HDMI 像素时钟域）：
//   buzzer_ctrl —— 蜂鸣器事件（下面这个）
//   phase_timer —— 交通灯相位倒计时（2026-10-15 批 2，在文件末尾）
// 塞在同一个文件里是有原因的：新建 .v 会被 TD 改写工程时把条目丢掉
// （交接简报踩坑清单第 6 条），所以新模块一律并进已在编译清单的文件。
//
// 两个触发源，都是"事件"而不是"状态"——路口红灯是常态，一直响等于没信息量：
//   ① 相位切换：检测到的灯色发生变化（原有逻辑）
//   ② 红灯相位下画面里出现运动目标（当作"有车在动"）
//      用运动计数而不是"红目标数>0"：交通灯的灯面本身就是一个红色圆形目标，
//      拿它当"有车"必然恒真、没有信息量。
//   条件用上升沿触发一次，条件消失后才能再次触发，避免连续鸣叫。
//
// O_buzzer 低电平响、高电平停（板上三极管反相）。
// ===========================================================================
`timescale 1ns/1ps

module buzzer_ctrl #(
    parameter [25:0] BEEP_CYCLES = 26'd37_500_000   // 0.5s @75MHz
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_light_valid,
    input  wire [1:0]  I_light_color,   // 0红 1黄 2绿
    input  wire [3:0]  I_mot_num,       // 运动目标数
    output reg         O_buzzer         // 低电平响
);

    reg        vld_prev;
    reg [1:0]  col_prev;
    reg [25:0] beep_cnt;

    // 记录"上一次检测到的颜色"（只在有效时更新，无效时保留）
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            vld_prev <= 1'b0;
            col_prev <= 2'd0;
        end
        else if (I_light_valid) begin
            vld_prev <= 1'b1;
            col_prev <= I_light_color;
        end
        else begin
            vld_prev <= 1'b0;      // col_prev 故意不清零，用于"丢失后回来"判断
        end
    end

    // 触发源 ①：相位切换（颜色变化）的那一拍
    wire trig_ph = I_light_valid && vld_prev && (I_light_color != col_prev);

    // 触发源 ②：红灯 + 有运动目标，取条件的上升沿
    wire cond_mot = I_light_valid && (I_light_color == 2'd0) && (I_mot_num != 4'd0);
    reg  cond_prev;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) cond_prev <= 1'b0;
        else          cond_prev <= cond_mot;
    end
    wire trig_mot = cond_mot && !cond_prev;

    wire trig = trig_ph || trig_mot;

    // 单次鸣响：trig 触发后倒数 BEEP_CYCLES 拍
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            beep_cnt <= 26'd0;
            O_buzzer <= 1'b1;                              // 停
        end
        else begin
            if (trig)                    beep_cnt <= BEEP_CYCLES;
            else if (beep_cnt != 26'd0)  beep_cnt <= beep_cnt - 26'd1;

            O_buzzer <= (beep_cnt != 26'd0) ? 1'b0 : 1'b1;
        end
    end

endmodule

// ===========================================================================
// phase_timer.v —— 交通灯相位倒计时（2026-10-15 批 2）
//
// 【这个倒计时是什么 —— 别包装成"读取信号机"】
//   我们**没有真实信号机的配时接口**，读到的是摄像头画面里的灯色。所以做的是
//   "相位剩余时间估计"：检测到相位（红/黄/绿）**切换**时，按该相位的**典型时长
//   预设**装载倒计时，然后 1Hz 递减到 0。报告/答辩里要如实这么说。
//   预设值做成参数（红 25s / 黄 3s / 绿 20s），改参数就能调，不用动逻辑。
//
// 【为什么相位要防抖】
//   灯色是逐帧判出来的（颜色掩码 + 圆形判据），过曝、遮挡、远处小目标都可能让
//   某一帧掉成别的颜色。不防抖的话倒计时会被反复复位、数字乱跳。这里要求新颜色
//   **连续保持 DEB_CYCLES（0.25s）**才受理；第一次锁定不防抖，看到就收。
//
// 【为什么用 BCD 而不是二进制】
//   OSD 和数码管都是"一位一位"显示的，直接用 BCD 就不需要任何除法器；
//   递减就是"个位减到 0 就向十位借 1"—— 和 hdmi_mixer 里 E2E/LAT 的减法状态机
//   是同一个思路：能用减法+进位解决的就别用除法器（除法器又占 LUT 又吃时序）。
//
// 【灯丢了怎么办】只在"此刻还能看到灯"的时候递减，灯丢了数字就冻住。
//   这样不会出现"画面里没有灯、倒计时还在自己跑"的假象。
// ===========================================================================
module phase_timer #(
    parameter [26:0] TICK_CYCLES = 27'd74_250_000,   // 1 秒 @74.25MHz
    parameter [24:0] DEB_CYCLES  = 25'd18_562_500,   // 0.25 秒相位防抖
    parameter [3:0]  RED_T = 4'd2, RED_U = 4'd5,     // 红相位预置 25s
    parameter [3:0]  YEL_T = 4'd0, YEL_U = 4'd3,     // 黄相位预置 3s
    parameter [3:0]  GRN_T = 4'd2, GRN_U = 4'd0      // 绿相位预置 20s
)(
    input  wire       I_clk,
    input  wire       I_rst_n,
    input  wire       I_light_valid,     // 本帧检测到交通灯
    input  wire [1:0] I_light_color,     // 0红 1黄 2绿
    output reg  [3:0] O_sec_t,           // 剩余秒 十位（BCD）
    output reg  [3:0] O_sec_u,           // 剩余秒 个位（BCD）
    output reg  [1:0] O_phase,           // 当前相位（0红 1黄 2绿）
    output reg        O_vld              // 已锁定过相位；此前 OSD/数码管留空
);
    reg [26:0] tick_cnt;
    reg [24:0] deb_cnt;
    reg        seen;

    wire tick   = (tick_cnt == TICK_CYCLES - 27'd1);
    wire is_new = I_light_valid && (I_light_color != O_phase);
    // 受理条件：还没锁过相位 -> 看到有效就收；已锁过 -> 新颜色要连续保持到防抖阈值
    wire ph_arm = seen ? (is_new && (deb_cnt >= DEB_CYCLES - 25'd1)) : I_light_valid;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            tick_cnt <= 27'd0;
            deb_cnt  <= 25'd0;
            seen     <= 1'b0;
            O_sec_t  <= 4'd0;
            O_sec_u  <= 4'd0;
            O_phase  <= 2'd0;
            O_vld    <= 1'b0;
        end
        else begin
            // ---- 秒节拍 ----
            tick_cnt <= tick ? 27'd0 : (tick_cnt + 27'd1);

            // ---- 相位防抖计数：这个"新颜色"连续保持多久了 ----
            if (!is_new)      deb_cnt <= 25'd0;
            else if (!ph_arm) deb_cnt <= deb_cnt + 25'd1;

            if (ph_arm) begin
                // ---- 相位受理：装载该相位的预置时长 ----
                O_phase  <= I_light_color;
                seen     <= 1'b1;
                tick_cnt <= 27'd0;      // 装载这一拍重新起算，别让切换后立刻走掉一秒
                case (I_light_color)
                    2'd0:    begin O_sec_t <= RED_T; O_sec_u <= RED_U; end
                    2'd1:    begin O_sec_t <= YEL_T; O_sec_u <= YEL_U; end
                    default: begin O_sec_t <= GRN_T; O_sec_u <= GRN_U; end
                endcase
            end
            // ---- 1Hz 递减：只在"此刻还能看到灯"时走 ----
            else if (tick && I_light_valid && ((O_sec_t != 4'd0) || (O_sec_u != 4'd0))) begin
                if (O_sec_u != 4'd0)
                    O_sec_u <= O_sec_u - 4'd1;
                else begin
                    O_sec_u <= 4'd9;    // 个位向十位借 1
                    O_sec_t <= O_sec_t - 4'd1;
                end
            end

            O_vld <= seen;
        end
    end
endmodule
