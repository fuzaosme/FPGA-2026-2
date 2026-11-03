

module design_top_wrapper (
    input wire        I_sys_clk,
    input wire        I_rst_n,
      
    output wire       O_cam_scl,
    inout  wire       IO_cam_sda,
    output wire       O_cam_24m,
    output wire       O_cam_rst,
      
    inout wire [1:0]  I_button,
    input wire        I_uart_rx,

    output wire       O_screen_pwm,
    output wire [3:0] O_led,          // 板载 LED1..LED4（调试指示，见下面的注释）
    output wire       O_tmds_ch0_p,
    output wire       O_tmds_ch1_p,
    output wire       O_tmds_ch2_p,
    output wire       O_tmds_clk_p,

    inout wire        IO_rx_clk_pad_n, 
    inout wire        IO_rx_clk_pad_p, 
    inout wire[3:0]   IO_rx_data_pad_n,
    inout wire[3:0]   IO_rx_data_pad_p,

    output wire[12:0] ddr_addr,
    output wire[ 1:0] ddr_ba,
    output wire[ 0:0] ddr_cke,
    output wire[ 0:0] ddr_odt,
    output wire[ 0:0] ddr_cs_n,
    output wire       ddr_ras_n,
    output wire       ddr_cas_n,
    output wire       ddr_we_n,
    output wire[ 0:0] ddr_ck_p,
    output wire[ 0:0] ddr_ck_n,
    inout wire[1:0]   ddr_dm,
    inout wire[15:0]  ddr_dq,
    inout wire[1:0]   ddr_dqs_p,
    inout wire[1:0]   ddr_dqs_n,
    output wire       O_uart_tx,
    output wire [7:0] O_disp_seg,
    output wire [7:0] O_disp_sel,
    output wire       O_buzzer,
    output wire       O_bt_tx
);


    wire        	S_100m_clk;
    wire        	S_24m_clk;
    wire        	S_aux_50m_clk;
	wire        	S_10m_clk;
		
	wire        	S_pll_lock;
    wire        	S_rst_n;
	
	wire [15:0] 	S_ae;
    wire [15:0] 	S_ag;
    wire 			S_cam_cfg_done;
    wire 			S_ae_cfg_done;
    wire 			S_ae_req; 
	
	wire        	S_csi_rx_clk;  
    wire        	S_hs_rx_valid;  
    wire[15:0]  	S_hs_rx_data;   
	wire[1:0]   	S_lane_error;
	
	wire        	S_csi_frame_start;       
	wire        	S_csi_frame_end;         
	wire        	S_csi_valid;             
	wire[31:0]  	S_csi_data;  
		
	wire        	S_raw10_frame_start; 
	wire        	S_raw10_frame_end;   
	wire        	S_raw10_valid;       
	wire [39:0] 	S_raw10_data;
		
	wire 			S_axis_tlast;  
	wire 			S_axis_tuser;  
	wire [39:0]		S_axis_tdata;  
	wire 			S_axis_tvalid; 
	
	wire [39:0]		S_raw_tdata ;
	wire			S_raw_tlast ;
	wire 			S_raw_tvalid;
	wire 			S_raw_tuser ;
	
    wire			S_ISP_O_tready;
    // The display pipeline is a continuous camera stream, consumed every
    // cycle by video_in's asynchronous FIFO. There is no reverse ready port
    // on video_in. An undriven ready stalls AWB after its first valid packet,
    // leaving constant pixels/flags at the input of all display effects.
    // DDR command backpressure is handled on the FIFO's DDR side instead.
    assign S_ISP_O_tready = 1'b1;
    wire [2:0]		S_disp_mode;     // 六档风格：0原图 1二值 2线稿 3卡通 4伪彩 5掩码高亮
	wire [127:0]	S_ISP_O_tdata ;
	wire 			S_ISP_O_tlast ;
	wire 			S_ISP_O_tuser ;
	wire 			S_ISP_O_tvalid; 
	

    wire        S_ddr_clk;
    wire        S_hdmi_pixel_clk;
    wire        S_hdmi_serial_clk;
    wire        S_hdmi_rst_n;
    wire        S_video_out_rst_n;


    wire        S_vi_128b_frame_start;
    wire        S_vi_128b_valid;      
    wire[127:0] S_vi_128b_data;    
    
    wire        S_video_out_rd_busy;
    wire        S_video_in_wr_busy; 
    wire[1:0]   S_video_out_rp;     

    wire        S_ddr_user_wr_en;       
    wire        S_ddr_user_rd_en;       
    wire[24:0]  S_ddr_user_addr;        
    wire[127:0] S_ddr_user_wr_data;     
    wire        S_ddr_user_ready;       
    wire        S_ddr_user_rd_valid;    
    wire[127:0] S_ddr_user_rd_data;     

    wire        S_vi_ddr_wr_en;   
    wire[24:0]  S_vi_ddr_wr_addr; 
    wire[127:0] S_vi_ddr_wr_data;
    wire        S_vi_ddr_wr_ready;

    wire        S_vo_ddr_rd_en;   
    wire[24:0]  S_vo_ddr_rd_addr; 
    wire        S_vo_ddr_rd_ready;
    wire        S_vo_dbg_fifo_rst;
    wire        S_vo_dbg_ddr_rd_valid;
    wire        S_vo_dbg_fifo_rd_en;
    wire        S_vo_dbg_fifo_empty;

    wire        S_init_calib_complete; 
    wire[24:0]  S_mc_app_addr;          
    wire[2:0]   S_mc_app_cmd;           
    wire        S_mc_app_en;            
    wire[127:0] S_mc_app_wdf_data;      
    wire        S_mc_app_wdf_end;       
    wire[15:0]  S_mc_app_wdf_mask;      
    wire        S_mc_app_wdf_wren;      
    wire[127:0] S_mc_app_rd_data;       
    wire        S_mc_app_rd_data_end;   
    wire        S_mc_app_rd_data_valid; 
    wire        S_mc_app_rdy;           
    wire        S_mc_app_wdf_rdy;       
    wire        S_mc_dbg_fifo_afull;
    wire        S_mc_dbg_app_rdy;
    wire        S_mc_dbg_wdf_rdy;
    wire        S_mc_dbg_cmd_pop;

    wire        S_hdmi_vsync;
    wire        S_hdmi_hsync;
    wire        S_hdmi_de;
    wire        S_hdmi_user;
    wire        S_hdmi_last;
    wire        S_video_out_vsync;
    wire        S_hdmi_window_rd_en;
    wire        S_hdmi_out_vsync;
    wire        S_hdmi_out_hsync;
    wire        S_hdmi_out_de;
    wire[23:0]  S_hdmi_out_data;

    wire[23:0]  S_video_rd_data;
    reg         S_dbg_raw_seen;
    reg         S_dbg_isp_seen;
    reg         S_dbg_ddr_wr_seen;
    reg         S_dbg_ddr_rd_seen;
    reg         S_dbg_cam_cfg_seen;
    reg         S_dbg_hs_seen;
    reg         S_dbg_csi_seen;
    reg         S_dbg_raw10_seen;
    reg         S_dbg_isp_fs_seen;
    reg         S_dbg_isp_valid_seen;
    reg         S_dbg_ddr_req_seen;
    reg         S_dbg_ddr_ready_seen;
    reg [1:0]   S_dbg_raw_seen_sync;
    reg [1:0]   S_dbg_isp_seen_sync;
    reg [1:0]   S_dbg_ddr_wr_seen_sync;
    reg [1:0]   S_dbg_ddr_rd_seen_sync;
    reg [1:0]   S_dbg_cam_cfg_seen_sync;
    reg [1:0]   S_dbg_hs_seen_sync;
    reg [1:0]   S_dbg_csi_seen_sync;
    reg [1:0]   S_dbg_raw10_seen_sync;
    reg [1:0]   S_dbg_isp_fs_seen_sync;
    reg [1:0]   S_dbg_isp_valid_seen_sync;
    reg [1:0]   S_dbg_ddr_req_seen_sync;
    reg [1:0]   S_dbg_ddr_ready_seen_sync;
    reg         S_dbg_video_nonzero_seen;
    reg         S_dbg_rd_req_seen;
    reg         S_dbg_rd_valid_seen;
    reg         S_dbg_rd_nonzero_seen;
    reg [1:0]   S_dbg_rd_req_seen_sync;
    reg [1:0]   S_dbg_rd_valid_seen_sync;
    reg [1:0]   S_dbg_rd_nonzero_seen_sync;
    reg         S_dbg_mc_init_seen;
    reg         S_dbg_mc_app_rdy_seen;
    reg         S_dbg_mc_wdf_rdy_seen;
    reg         S_dbg_mc_cmd_pop_seen;
    reg [1:0]   S_dbg_mc_init_seen_sync;
    reg [1:0]   S_dbg_mc_app_rdy_seen_sync;
    reg [1:0]   S_dbg_mc_wdf_rdy_seen_sync;
    reg [1:0]   S_dbg_mc_cmd_pop_seen_sync;
    wire[3:0]   S_hdmi_debug_status;
    wire        S_lane_error_any;


    assign O_screen_pwm = 1'b1;
    assign S_video_out_vsync = ~S_hdmi_vsync;
	
	
    assign S_rst_n 		= S_pll_lock;
    assign S_hdmi_rst_n = S_pll_lock;

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_raw_seen <= 1'b0;
        else if(S_raw10_frame_start | S_axis_tvalid)
            S_dbg_raw_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_seen <= 1'b0;
        else if(S_ISP_O_tuser | S_ISP_O_tvalid)
            S_dbg_isp_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_wr_seen <= 1'b0;
        else if(S_vi_ddr_wr_en)
            S_dbg_ddr_wr_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_rd_seen <= 1'b0;
        else if(S_ddr_user_rd_valid)
            S_dbg_ddr_rd_seen <= 1'b1;
    end

    always @(posedge S_24m_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_cam_cfg_seen <= 1'b0;
        else if(S_cam_cfg_done)
            S_dbg_cam_cfg_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_hs_seen <= 1'b0;
        else if(S_hs_rx_valid)
            S_dbg_hs_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_csi_seen <= 1'b0;
        else if(S_csi_frame_start | S_csi_valid)
            S_dbg_csi_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_raw10_seen <= 1'b0;
        else if(S_raw10_frame_start | S_raw10_valid)
            S_dbg_raw10_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_fs_seen <= 1'b0;
        else if(S_ISP_O_tuser)
            S_dbg_isp_fs_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_valid_seen <= 1'b0;
        else if(S_ISP_O_tvalid)
            S_dbg_isp_valid_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_req_seen <= 1'b0;
        else if(S_vi_ddr_wr_en)
            S_dbg_ddr_req_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_ready_seen <= 1'b0;
        else if(S_ddr_user_ready)
            S_dbg_ddr_ready_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_req_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_req_seen <= 1'b0;
        else if(S_vo_ddr_rd_en)
            S_dbg_rd_req_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_valid_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_valid_seen <= 1'b0;
        else if(S_ddr_user_rd_valid)
            S_dbg_rd_valid_seen <= 1'b1;
    end

    always @(posedge S_hdmi_pixel_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_nonzero_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_nonzero_seen <= 1'b0;
        else if(S_hdmi_window_rd_en && (S_video_rd_data != 24'd0))
            S_dbg_rd_nonzero_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_init_seen <= 1'b0;
        else if(S_init_calib_complete)
            S_dbg_mc_init_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_app_rdy_seen <= 1'b0;
        else if(S_mc_dbg_app_rdy)
            S_dbg_mc_app_rdy_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_wdf_rdy_seen <= 1'b0;
        else if(S_mc_dbg_wdf_rdy)
            S_dbg_mc_wdf_rdy_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_cmd_pop_seen <= 1'b0;
        else if(S_mc_dbg_cmd_pop)
            S_dbg_mc_cmd_pop_seen <= 1'b1;
    end

    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin
            S_dbg_cam_cfg_seen_sync <= 2'b00;
            S_dbg_hs_seen_sync      <= 2'b00;
            S_dbg_csi_seen_sync     <= 2'b00;
            S_dbg_raw10_seen_sync   <= 2'b00;
            S_dbg_raw_seen_sync    <= 2'b00;
            S_dbg_isp_seen_sync    <= 2'b00;
            S_dbg_ddr_wr_seen_sync <= 2'b00;
            S_dbg_ddr_rd_seen_sync <= 2'b00;
            S_dbg_isp_fs_seen_sync  <= 2'b00;
            S_dbg_isp_valid_seen_sync <= 2'b00;
            S_dbg_ddr_req_seen_sync <= 2'b00;
            S_dbg_ddr_ready_seen_sync <= 2'b00;
            S_dbg_video_nonzero_seen <= 1'b0;
            S_dbg_rd_req_seen_sync <= 2'b00;
            S_dbg_rd_valid_seen_sync <= 2'b00;
            S_dbg_rd_nonzero_seen_sync <= 2'b00;
            S_dbg_mc_init_seen_sync <= 2'b00;
            S_dbg_mc_app_rdy_seen_sync <= 2'b00;
            S_dbg_mc_wdf_rdy_seen_sync <= 2'b00;
            S_dbg_mc_cmd_pop_seen_sync <= 2'b00;
        end
        else begin
            S_dbg_cam_cfg_seen_sync <= {S_dbg_cam_cfg_seen_sync[0], S_dbg_cam_cfg_seen};
            S_dbg_hs_seen_sync      <= {S_dbg_hs_seen_sync[0],      S_dbg_hs_seen};
            S_dbg_csi_seen_sync     <= {S_dbg_csi_seen_sync[0],     S_dbg_csi_seen};
            S_dbg_raw10_seen_sync   <= {S_dbg_raw10_seen_sync[0],   S_dbg_raw10_seen};
            S_dbg_raw_seen_sync    <= {S_dbg_raw_seen_sync[0],    S_dbg_raw_seen};
            S_dbg_isp_seen_sync    <= {S_dbg_isp_seen_sync[0],    S_dbg_isp_seen};
            S_dbg_ddr_wr_seen_sync <= {S_dbg_ddr_wr_seen_sync[0], S_dbg_ddr_wr_seen};
            S_dbg_ddr_rd_seen_sync <= {S_dbg_ddr_rd_seen_sync[0], S_dbg_ddr_rd_seen};
            S_dbg_isp_fs_seen_sync  <= {S_dbg_isp_fs_seen_sync[0],  S_dbg_isp_fs_seen};
            S_dbg_isp_valid_seen_sync <= {S_dbg_isp_valid_seen_sync[0], S_dbg_isp_valid_seen};
            S_dbg_ddr_req_seen_sync <= {S_dbg_ddr_req_seen_sync[0], S_dbg_ddr_req_seen};
            S_dbg_ddr_ready_seen_sync <= {S_dbg_ddr_ready_seen_sync[0], S_dbg_ddr_ready_seen};
            S_dbg_rd_req_seen_sync <= {S_dbg_rd_req_seen_sync[0], S_dbg_rd_req_seen};
            S_dbg_rd_valid_seen_sync <= {S_dbg_rd_valid_seen_sync[0], S_dbg_rd_valid_seen};
            S_dbg_rd_nonzero_seen_sync <= {S_dbg_rd_nonzero_seen_sync[0], S_dbg_rd_nonzero_seen};
            S_dbg_mc_init_seen_sync <= {S_dbg_mc_init_seen_sync[0], S_dbg_mc_init_seen};
            S_dbg_mc_app_rdy_seen_sync <= {S_dbg_mc_app_rdy_seen_sync[0], S_dbg_mc_app_rdy_seen};
            S_dbg_mc_wdf_rdy_seen_sync <= {S_dbg_mc_wdf_rdy_seen_sync[0], S_dbg_mc_wdf_rdy_seen};
            S_dbg_mc_cmd_pop_seen_sync <= {S_dbg_mc_cmd_pop_seen_sync[0], S_dbg_mc_cmd_pop_seen};
            if(S_hdmi_window_rd_en && (S_video_rd_data != 24'd0))
                S_dbg_video_nonzero_seen <= 1'b1;
        end
    end

    assign S_lane_error_any = |S_lane_error;

    assign S_vo_dbg_fifo_rst = 1'b0;
    assign S_vo_dbg_ddr_rd_valid = 1'b0;
    assign S_vo_dbg_fifo_rd_en = 1'b0;
    assign S_vo_dbg_fifo_empty = 1'b0;
    assign S_mc_dbg_fifo_afull = 1'b0;
    assign S_mc_dbg_app_rdy = 1'b0;
    assign S_mc_dbg_wdf_rdy = 1'b0;
    assign S_mc_dbg_cmd_pop = 1'b0;

    assign S_hdmi_debug_status = {
        (|S_ddr_user_rd_data),
        S_ddr_user_rd_valid,
        S_vo_ddr_rd_en,
        S_vi_ddr_wr_en
    };
	
	assign O_cam_rst 	= 1'b1;
    assign O_cam_24m 	= S_24m_clk;
	
	
	
	
    PLL u_PLL(
        .refclk      ( I_sys_clk         ),
        .reset    	 ( ~I_rst_n          ),
        
        .clk0_out    ( S_100m_clk        ),
        .clk1_out    ( S_24m_clk         ),

        .clk4_out    ( S_hdmi_pixel_clk  ),
        .clk5_out    ( S_hdmi_serial_clk ),

        .lock        ( S_pll_lock        )

    );


	

  
  // IQ_BUILD_ID_AUTOGENERATED: snapshot builds replace only this literal.
  localparam [31:0] IQ_BUILD_ID=32'h49513031;
  wire [8:0] camera_meter_i,camera_meter_a;wire camera_meter_iv,camera_meter_av,camera_meter_busy;
  wire [287:0] camera_snapshot_i,camera_snapshot_h;wire camera_snapshot_iv,camera_snapshot_hv,camera_snapshot_busy;
  wire [2:0] camera_view_h,camera_view_i;wire camera_view_hv,camera_view_iv,camera_view_busy;
  reg [2:0] camera_view_pending,camera_view_active;reg camera_view_wait;
  wire camera_fault,camera_unstable;wire [7:0] camera_error;
  wire [15:0] camera_applied_exp;wire [7:0] camera_applied_gain;wire camera_at_limit;
  reg frame_toggle;reg [2:0] frame_sync;
  always @(posedge S_csi_rx_clk or negedge S_rst_n)begin
   if(!S_rst_n)frame_toggle<=0;else if(S_raw10_frame_start)frame_toggle<=~frame_toggle;
  end
  always @(posedge S_24m_clk or negedge S_rst_n)begin
   if(!S_rst_n)frame_sync<=0;else frame_sync<={frame_sync[1:0],frame_toggle};
  end
  arrow_mailbox #(.W(9)) camera_meter_mailbox(
   .S_clk(S_csi_rx_clk),.S_rst_n(S_rst_n),.S_valid(camera_meter_iv&&!camera_meter_busy),.S_data(camera_meter_i),.S_busy(camera_meter_busy),
   .D_clk(S_24m_clk),.D_rst_n(S_rst_n),.D_valid(camera_meter_av),.D_data(camera_meter_a));
  camera_auto_exposure camera_ae(
   .clk(S_24m_clk),.rst(S_rst_n),.frame_tick(frame_sync[2]^frame_sync[1]),
   .stats_valid(camera_meter_av),.mean_y(camera_meter_a[7:0]),.stats_good(camera_meter_a[8]),
   .cfg_ready(S_ae_cfg_done),.cfg_fault(camera_fault),.applied_exp(camera_applied_exp),.applied_gain(camera_applied_gain),
   .request(S_ae_req),.exposure(S_ae),.gain(S_ag),.unstable(camera_unstable),.at_limit(camera_at_limit));
  uicfgcs500_720p #(.CLK_DIV(24000000/100000-1)) u_uicfgcs500(
   .I_clk(S_24m_clk),.I_rst_n(S_rst_n),.I_ae_req(S_ae_req),.I_ae(S_ae),.I_ag(S_ag),
   .O_cam_scl(O_cam_scl),.IO_cam_sda(IO_cam_sda),.O_cfg_done(S_cam_cfg_done),.O_ae_cfg_done(S_ae_cfg_done),
   .O_fault(camera_fault),.O_error(camera_error),.O_applied_ae(camera_applied_exp),.O_applied_ag(camera_applied_gain));
  reg [1:0] camera_unstable_sync;
  always @(posedge S_csi_rx_clk or negedge S_rst_n)begin
   if(!S_rst_n)camera_unstable_sync<=3;else camera_unstable_sync<={camera_unstable_sync[0],camera_unstable};
  end
  wire [31:0] camera_control_a={camera_error[3:0],camera_at_limit,camera_unstable,camera_fault,S_ae_cfg_done,camera_applied_gain,camera_applied_exp};
  wire [31:0] camera_control_h;wire camera_control_hv,camera_control_busy;
  reg [31:0] camera_control_sent;
  wire camera_control_send=!camera_control_busy&&(camera_control_a!=camera_control_sent);
  always @(posedge S_24m_clk or negedge S_rst_n)begin
   if(!S_rst_n)camera_control_sent<=32'hffffffff;else if(camera_control_send)camera_control_sent<=camera_control_a;
  end
  arrow_mailbox #(.W(32)) camera_control_mailbox(
   .S_clk(S_24m_clk),.S_rst_n(S_rst_n),.S_valid(camera_control_send),.S_data(camera_control_a),.S_busy(camera_control_busy),
   .D_clk(S_hdmi_pixel_clk),.D_rst_n(S_hdmi_rst_n),.D_valid(camera_control_hv),.D_data(camera_control_h));
  arrow_mailbox #(.W(288)) camera_snapshot_mailbox(
   .S_clk(S_csi_rx_clk),.S_rst_n(S_rst_n),.S_valid(camera_snapshot_iv&&!camera_snapshot_busy),.S_data(camera_snapshot_i),.S_busy(camera_snapshot_busy),
   .D_clk(S_hdmi_pixel_clk),.D_rst_n(S_hdmi_rst_n),.D_valid(camera_snapshot_hv),.D_data(camera_snapshot_h));
  always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n)begin
   if(!S_hdmi_rst_n)begin camera_view_pending<=0;camera_view_wait<=0;end
   else begin
    if(camera_view_wait&&!camera_view_busy)camera_view_wait<=0;
    if(camera_view_hv)begin camera_view_pending<=camera_view_h;camera_view_wait<=1;end
   end
  end
  arrow_mailbox #(.W(3)) camera_view_mailbox(
   .S_clk(S_hdmi_pixel_clk),.S_rst_n(S_hdmi_rst_n),.S_valid(camera_view_wait&&!camera_view_busy),.S_data(camera_view_pending),.S_busy(camera_view_busy),
   .D_clk(S_csi_rx_clk),.D_rst_n(S_rst_n),.D_valid(camera_view_iv),.D_data(camera_view_i));
  always @(posedge S_csi_rx_clk or negedge S_rst_n)begin
   if(!S_rst_n)camera_view_active<=0;else if(camera_view_iv)camera_view_active<=camera_view_i;
  end

    mipi_dphy_rx_ph1p_mipiio_wrapper#(
        .DPHY_RX_LOCATION      ( "DPHY0" ),
        .HS_EQUALIZER          ( "0dB"   ),//"0dB","1.5dB","3dB","4.5dB","6dB","7.5dB","9dB","10.5dB"
        .HS_VGA_GAIN           ( "8dB" ),//"-3dB","-1.5dB","0dB","1.5dB","3dB","4.5dB","6dB","7.5dB"
        .LANE_NUM              ( 2       ),
        .BYTE_NUM              ( 1       )
    )u_mipi_dphy_rx_ph1p_mipiio_wrapper(
        .I_lp_clk              ( S_100m_clk       ),
        .I_rst                 ( ~S_rst_n         ),

        .I_clk_lane_in_delay   ( 6'd0             ),
        .I_data_lane0_in_delay ( 6'd0             ),
        .I_data_lane1_in_delay ( 6'd0             ),
        .I_data_lane2_in_delay ( 6'd0             ),
        .I_data_lane3_in_delay ( 6'd0             ),

        .I_lane_invert         ( 4'b0000          ),
     
        .O_hs_rx_clk           ( S_csi_rx_clk     ),
        .O_hs_rx_valid         ( S_hs_rx_valid    ),
        .O_hs_rx_data          ( S_hs_rx_data     ),
      
        .O_lp_rx_lane0_p       (  ),
        .O_lp_rx_lane0_n       (  ),
      
        .I_lp_tx_en            ( 1'b0             ),
        .I_lp_tx_lane0_p       ( 1'b1             ),
        .I_lp_tx_lane0_n       ( 1'b1             ),
      
        .O_lane_match_error    (                  ),
        .O_lane_error          ( S_lane_error     ),
      
        .IO_rx_clk_pad_n       ( IO_rx_clk_pad_n  ),
        .IO_rx_clk_pad_p       ( IO_rx_clk_pad_p  ),
        .IO_rx_data_pad_n      ( IO_rx_data_pad_n ),
        .IO_rx_data_pad_p      ( IO_rx_data_pad_p )
    );
	
	
 //  cwc cwc_inst
 //(
 //    .probe0(S_hs_rx_valid),
 //    .probe1(S_hs_rx_data),
 //    .probe2(S_lane_error),
 //    .clk(S_csi_rx_clk)
 //);

 //csi 解码为RAW数据
csi_unpacket_2lane u_csi_unpacket(
.I_clk                 ( S_csi_rx_clk       ),
.I_rst_n               ( S_rst_n        	),
.I_hs_valid            ( S_hs_rx_valid     ),
.I_hs_data             ( S_hs_rx_data      ),

.O_csi_frame_start     ( S_csi_frame_start ),
.O_csi_frame_end       ( S_csi_frame_end   ),
.O_csi_valid           ( S_csi_valid       ),
.O_csi_data            ( S_csi_data        )
);


//解码为RAW8
raw10_unpacket_2lane u_raw10_unpacket (
.I_clk  (S_csi_rx_clk),
.I_rst_n(S_rst_n),

.I_csi_frame_start(S_csi_frame_start),
.I_csi_frame_end  (S_csi_frame_end),
.I_csi_valid      (S_csi_valid),
.I_csi_data       (S_csi_data),

.O_raw10_frame_start(S_raw10_frame_start),
.O_raw10_frame_end  (S_raw10_frame_end),
.O_raw10_valid      (S_raw10_valid),
.O_raw10_data       (S_raw10_data)
  );


//将数据转为stream流
uial2axis #(
.IMG_WIDTH(1280),
.IMG_HEIGHT(720),
.INPUT_DATA_WIDTH(40)
) 
u_uial2axis (
.I_native_clk(S_csi_rx_clk),
.I_rst_n     (S_rst_n),
.I_data      (S_raw10_data       ),
.I_data_valid(S_raw10_valid      ),
.I_data_start(S_raw10_frame_start),
.I_data_end  (S_raw10_frame_end  ),
.axis_tvalid (S_axis_tvalid),
.axis_tdata  (S_axis_tdata ),
.axis_tuser  (S_axis_tuser ),
.axis_tlast  (S_axis_tlast )
);
	

// 原厂HD模式直接输出1280x720 RAW10，不再进行FPGA固定窗口裁剪。
assign S_raw_tdata  = S_axis_tdata;
assign S_raw_tlast  = S_axis_tlast;
assign S_raw_tvalid = S_axis_tvalid;
assign S_raw_tuser  = S_axis_tuser;


//cwc1 cwc1_Inst
//  (
//      .probe0(S_axis_tuser),
//      .probe1(S_axis_tvalid),
//      .probe2(S_axis_tlast),
//      .probe3(S_axis_tdata),
//      .probe4(S_ISP_O_tuser),
//      .probe5(S_ISP_O_tvalid),
//      .probe6(S_ISP_O_tlast),
//      .probe7(S_ISP_O_tdata),
//      .probe8 (S_raw_tdata ),
//      .probe9 (S_raw_tlast ),
//      .probe10(S_raw_tvalid),
//      .probe11(S_raw_tuser ),
//      .clk(S_csi_rx_clk)
//  );


//ISP算法顶层模块
// 显示模式切换按键：按一下在"原图"和"处理结果"之间切换
// 用 100MHz 时钟（一直存在的时钟），按键经过同步+去抖，输出送到 ISP 做显示选择
disp_mode_toggle #(
    .DB_CNT(1000000)        // 约 10ms 去抖
) u_disp_mode_toggle (
    .I_clk  (S_100m_clk),
    .I_rst_n(S_rst_n),
    .I_btn  (I_button),     // 两个按键任意一个按下都切换
    .O_mode (S_disp_mode)
);

// ---- 上位机回控：串口接收 M0~M5 切换显示模式 ----
wire [7:0] S_uart_byte;
wire       S_uart_byte_valid;
wire [2:0] S_uart_mode;
wire       S_uart_mode_valid;
wire       S_lp_en;             // 批 3：低功耗降频开关（串口 L0/L1，上电默认开）
// "C" 前缀分流后的两路字节流（见 uart_cmd.v 里的 uart_esc_filter）
wire [7:0] S_cmd_byte;        // 干净流：给 uart_cmd / thr_regs
wire       S_cmd_byte_valid;
wire [7:0] S_bt_man_byte;     // 小车手动命令字节：给蓝牙 TX
wire       S_bt_man_valid;

uart_rx #(.CLK_FREQ(75000000)) u_uart_rx (
    .I_clk  (S_hdmi_pixel_clk),
    .I_rst_n(S_hdmi_rst_n),
    .I_rx   (I_uart_rx),
    .O_data (S_uart_byte),
    .O_valid(S_uart_byte_valid)
);

// 上位机命令分流："C"+<1 字节> = 小车命令（只发蓝牙），其余 = 阈值/模式命令。
// 不加这一级的话，"CB"（后退）里的 'B' 会顺带把阈值解析器 armed 起来。
wire [7:0] arrow_legacy_byte;wire arrow_legacy_valid;
wire [127:0] arrow_cfg_h,arrow_cfg_i;wire arrow_cfg_hv,arrow_cfg_iv,arrow_cfg_busy;
wire [191:0] arrow_snap_i,arrow_snap_h;wire arrow_snap_iv,arrow_snap_hv,arrow_snap_busy;
wire [34:0] detect_cfg_h,detect_cfg_i;wire detect_cfg_hv,detect_cfg_iv,detect_cfg_busy;
reg [34:0] detect_cfg_pending;reg detect_cfg_wait;
wire [39:0] detect_status_i,detect_status_h;wire detect_status_iv,detect_status_busy;
reg [39:0] detect_status_sent;
wire detect_send=detect_cfg_wait && !detect_cfg_busy;
always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n)begin
 if(!S_hdmi_rst_n)begin detect_cfg_pending<=0;detect_cfg_wait<=0;end
 else begin
  if(detect_send)detect_cfg_wait<=0;
  if(detect_cfg_hv)begin detect_cfg_pending<=detect_cfg_h;detect_cfg_wait<=1;end
 end
end
wire status_send=(detect_status_i!=detect_status_sent)&&!detect_status_busy;
always @(posedge S_csi_rx_clk or negedge S_rst_n)begin
 if(!S_rst_n)detect_status_sent<=0;
 else if(status_send)detect_status_sent<=detect_status_i;
end
arrow_mailbox #(.W(35)) detect_configuration_mailbox(
 .S_clk(S_hdmi_pixel_clk),.S_rst_n(S_hdmi_rst_n),.S_valid(detect_send),.S_data(detect_cfg_pending),.S_busy(detect_cfg_busy),
 .D_clk(S_csi_rx_clk),.D_rst_n(S_rst_n),.D_valid(detect_cfg_iv),.D_data(detect_cfg_i));
arrow_mailbox #(.W(40)) detect_status_mailbox(
 .S_clk(S_csi_rx_clk),.S_rst_n(S_rst_n),.S_valid(status_send),.S_data(detect_status_i),.S_busy(detect_status_busy),
 .D_clk(S_hdmi_pixel_clk),.D_rst_n(S_hdmi_rst_n),.D_valid(detect_status_iv),.D_data(detect_status_h));
arrow_command command_router(.I_clk(S_hdmi_pixel_clk),.I_rst_n(S_hdmi_rst_n),.I_byte(S_uart_byte),.I_valid(S_uart_byte_valid),
 .O_legacy_byte(arrow_legacy_byte),.O_legacy_valid(arrow_legacy_valid),.O_cfg(arrow_cfg_h),.O_cfg_valid(arrow_cfg_hv),
 .O_detect_cfg(detect_cfg_h),.O_detect_valid(detect_cfg_hv),
 .O_camera_view(camera_view_h),.O_camera_valid(camera_view_hv));
arrow_mailbox #(.W(128)) cfg_mailbox(.S_clk(S_hdmi_pixel_clk),.S_rst_n(S_hdmi_rst_n),.S_valid(arrow_cfg_hv),.S_data(arrow_cfg_h),.S_busy(arrow_cfg_busy),
 .D_clk(S_csi_rx_clk),.D_rst_n(S_rst_n),.D_valid(arrow_cfg_iv),.D_data(arrow_cfg_i));
arrow_mailbox #(.W(192)) obs_mailbox(.S_clk(S_csi_rx_clk),.S_rst_n(S_rst_n),.S_valid(arrow_snap_iv),.S_data(arrow_snap_i),.S_busy(arrow_snap_busy),
 .D_clk(S_hdmi_pixel_clk),.D_rst_n(S_hdmi_rst_n),.D_valid(arrow_snap_hv),.D_data(arrow_snap_h));
uart_esc_filter u_uart_esc_filter (
    .I_clk           (S_hdmi_pixel_clk),
    .I_rst_n         (S_hdmi_rst_n),
    .I_byte          (arrow_legacy_byte),
    .I_byte_valid    (arrow_legacy_valid),
    .O_cmd_byte      (S_cmd_byte),
    .O_cmd_byte_valid(S_cmd_byte_valid),
    .O_esc_byte      (S_bt_man_byte),
    .O_esc_byte_valid(S_bt_man_valid)
);

uart_cmd u_uart_cmd (
    .I_clk       (S_hdmi_pixel_clk),
    .I_rst_n     (S_hdmi_rst_n),
    .I_byte      (S_cmd_byte),
    .I_byte_valid(S_cmd_byte_valid),
    .O_mode      (S_uart_mode),
    .O_mode_valid(S_uart_mode_valid),
    .O_lp_en     (S_lp_en)
);

// ---- 颜色块检测阈值寄存器（串口在线可调）----
// 命令：R### 红下限  r### 红比G/B高的量  G### 绿下限  g### 绿比R/B高的量
//       Y### 黄下限  y### 黄比蓝高的量  B### 黄里蓝上限
//       b##  块内像素下限(0..15)      m##  区域块数下限(0..15)
// 例：  R120  G90  y45  b40  m04
wire [7:0] S_thr_red_min, S_thr_red_gap, S_thr_grn_min, S_thr_grn_gap;
wire [7:0] S_thr_yel_min, S_thr_yel_bdiff, S_thr_yel_bmax;
wire [3:0] S_thr_blk_pix, S_thr_min_blk;
wire       S_thr_tog;
thr_regs u_thr_regs (
    .I_clk       (S_hdmi_pixel_clk),
    .I_rst_n     (S_hdmi_rst_n),
    .I_byte      (S_cmd_byte),
    .I_byte_valid(S_cmd_byte_valid),
    .O_red_min   (S_thr_red_min),
    .O_red_gap   (S_thr_red_gap),
    .O_grn_min   (S_thr_grn_min),
    .O_grn_gap   (S_thr_grn_gap),
    .O_yel_min   (S_thr_yel_min),
    .O_yel_bdiff (S_thr_yel_bdiff),
    .O_yel_bmax  (S_thr_yel_bmax),
    .O_blk_pix   (S_thr_blk_pix),
    .O_min_blk   (S_thr_min_blk),
    .O_tog       (S_thr_tog)
);

// 阈值跨到 ISP 域：toggle 握手，整组一起锁存（避免多比特裸同步）
reg       S_thr_tog_h1, S_thr_tog_h2, S_thr_tog_h3;
reg [7:0] S_thr_red_min_isp, S_thr_red_gap_isp;
reg [7:0] S_thr_grn_min_isp, S_thr_grn_gap_isp;
reg [7:0] S_thr_yel_min_isp, S_thr_yel_bdiff_isp, S_thr_yel_bmax_isp;
reg [3:0] S_thr_blk_pix_isp, S_thr_min_blk_isp;
always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
    if (!S_rst_n) begin
        S_thr_tog_h1 <= 1'b0;
        S_thr_tog_h2 <= 1'b0;
        S_thr_tog_h3 <= 1'b0;
        // 2026-10-13 上板标定后的定值（原来 110/70 90/40 110/45/180 12/4）
        // 注：b 是 4bit，上限 15，上面发的 b40 被 thr_regs 钳到 15
        S_thr_red_min_isp   <= 8'd28;
        S_thr_red_gap_isp   <= 8'd6;
        S_thr_grn_min_isp   <= 8'd28;
        S_thr_grn_gap_isp   <= 8'd6;
        S_thr_yel_min_isp   <= 8'd28;
        S_thr_yel_bdiff_isp <= 8'd6;
        S_thr_yel_bmax_isp  <= 8'd255;
        S_thr_blk_pix_isp   <= 4'd15;
        S_thr_min_blk_isp   <= 4'd8;
    end
    else begin
        S_thr_tog_h1 <= S_thr_tog;
        S_thr_tog_h2 <= S_thr_tog_h1;
        S_thr_tog_h3 <= S_thr_tog_h2;
        if (S_thr_tog_h2 ^ S_thr_tog_h3) begin
            S_thr_red_min_isp   <= S_thr_red_min;
            S_thr_red_gap_isp   <= S_thr_red_gap;
            S_thr_grn_min_isp   <= S_thr_grn_min;
            S_thr_grn_gap_isp   <= S_thr_grn_gap;
            S_thr_yel_min_isp   <= S_thr_yel_min;
            S_thr_yel_bdiff_isp <= S_thr_yel_bdiff;
            S_thr_yel_bmax_isp  <= S_thr_yel_bmax;
            S_thr_blk_pix_isp   <= S_thr_blk_pix;
            S_thr_min_blk_isp   <= S_thr_min_blk;
        end
    end
end

// ---- 批 3：低功耗降频开关跨到 ISP 域 ----
// 单 bit 配置、变化极慢，两级同步就够（和 S_awb_h1/h2 那些配置位做法一致）。
// 上电默认开，和 uart_cmd 里 O_lp_en 的复位值保持一致。
reg S_lp_en_i1, S_lp_en_i2;
always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
    if (!S_rst_n) begin
        S_lp_en_i1 <= 1'b1;
        S_lp_en_i2 <= 1'b1;
    end
    else begin
        S_lp_en_i1 <= S_lp_en;
        S_lp_en_i2 <= S_lp_en_i1;
    end
end

// 按键按下时交还手动控制；上位机发 M0~M5 时接管
reg [1:0] S_btn_d;
reg       S_uart_mode_valid_r;
always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
    if (!S_hdmi_rst_n) begin
        S_btn_d <= 2'b11;
        S_uart_mode_valid_r <= 1'b0;
    end
    else begin
        S_btn_d <= I_button;
        if ((S_btn_d[0] & ~I_button[0]) || (S_btn_d[1] & ~I_button[1]))
            S_uart_mode_valid_r <= 1'b0;
        else if (S_uart_mode_valid)
            S_uart_mode_valid_r <= 1'b1;
    end
end

wire [2:0] S_final_disp_mode = S_uart_mode_valid_r ? S_uart_mode : S_disp_mode;

// 目标跟踪跨时钟域寄存器：提前声明，供数码管/蜂鸣器/串口使用
reg        S_t0_valid_h1, S_t0_valid_h2, S_t1_valid_h1, S_t1_valid_h2;
reg [52:0] S_t0_h1, S_t0_h2, S_t1_h1, S_t1_h2;

// ---- 数码管 + 蜂鸣器 ----
// 交通灯类型编码 = 4 + 颜色（红4 黄5 绿6），车牌 1~3，无目标 0。
// 所以"是不是交通灯"必须判 type>=4；不能直接拿 O_t0_valid（那只是"跟踪槽0有目标"）。
// type 的低两位恰好等于颜色，所以颜色仍取 [49:48]。
wire       S_is_light_h2  = detect_status_h[7] && S_t0_valid_h2 && (S_t0_h2[50:48] >= 3'd4);
wire [1:0] S_light_col_h2 = S_t0_h2[49:48];
// ---- 场景闭环：识别结果 -> 决策 -> 板载执行器 ----
// 这几个量必须在这里先声明：S_mt_num_hd / S_conf_hd 的声明在文件更靠后
// 的位置，如果在下面直接使用，vlog 会当成隐式 1bit 网线并报重复声明。
wire [3:0] S_tgt_num;
wire [3:0] S_mot_num;
wire       S_alarm;
reg  [3:0] S_conf_hd;
// 交通灯相位倒计时（批 2）：数码管在这里就要用，真正的驱动在下面的 u_phase_timer
wire [3:0] S_cd_t;      // 剩余秒 十位（BCD）
wire [3:0] S_cd_u;      // 剩余秒 个位（BCD）
wire [1:0] S_cd_phase;  // 当前相位 0红 1黄 2绿
wire       S_cd_vld;    // 已锁定过相位

seg_scan u_seg_scan (
    .I_clk(S_hdmi_pixel_clk), .I_rst_n(S_hdmi_rst_n),
    .I_mode(S_final_disp_mode),
    .I_light_valid(S_is_light_h2),
    .I_light_color(S_light_col_h2),
    .I_tgt_num(S_tgt_num),
    .I_mot_num(S_mot_num),
    .I_conf(S_conf_hd),
    .I_alarm(S_alarm),
    .I_cd_t(S_cd_t), .I_cd_u(S_cd_u), .I_cd_vld(S_cd_vld),
    .O_seg(O_disp_seg), .O_sel(O_disp_sel)
);
buzzer_ctrl u_buzzer_ctrl (
    .I_clk(S_hdmi_pixel_clk), .I_rst_n(S_hdmi_rst_n && detect_status_h[7]),
    .I_light_valid(S_is_light_h2),
    .I_light_color(S_light_col_h2),
    .I_mot_num(S_mot_num),
    .O_buzzer(O_buzzer)
);

// ---- 交通灯相位倒计时（2026-10-15 批 2）----
// 输入就是"这一帧识别到的交通灯颜色"，输出两位 BCD 秒数 + 相位，给 OSD 和数码管。
// 注意它算的是**相位剩余时间的估计**（相位切换时按该相位的预设时长装载），
// 不是"读取真实信号机的倒计时" —— 报告里要如实这么写。
phase_timer u_phase_timer (
    .I_clk(S_hdmi_pixel_clk), .I_rst_n(S_hdmi_rst_n && detect_status_h[7]),
    .I_light_valid(S_is_light_h2),
    .I_light_color(S_light_col_h2),
    .O_sec_t(S_cd_t),
    .O_sec_u(S_cd_u),
    .O_phase(S_cd_phase),
    .O_vld(S_cd_vld)
);

// Display observations cross as a stable mailbox; retry changes while busy.
// Bits [4:0] = right, left, green, yellow, red. No detector decision is rerun here.
wire [4:0] display_signs_i,display_signs_h;wire display_signs_busy;
reg [4:0] display_signs_sent;
wire display_signs_send=(display_signs_i!=display_signs_sent)&&!display_signs_busy;
always @(posedge S_csi_rx_clk or negedge S_rst_n)begin
 if(!S_rst_n)display_signs_sent<=0;
 else if(display_signs_send)display_signs_sent<=display_signs_i;
end
arrow_mailbox #(.W(5)) display_signs_mailbox(
 .S_clk(S_csi_rx_clk),.S_rst_n(S_rst_n),.S_valid(display_signs_send),.S_data(display_signs_i),.S_busy(display_signs_busy),
 .D_clk(S_hdmi_pixel_clk),.D_rst_n(S_hdmi_rst_n),.D_valid(),.D_data(display_signs_h));
wire [1:0] visible_turn;
// A single five-class interface is published by the actual drawing paths.
// No enable/epoch/readiness/manual lease is allowed between it and the UART.
wire [2:0] visible_lamps;
wire [4:0] traffic_visible={visible_turn,visible_lamps}|display_signs_h;

// A box that appears (0->1) is ONE command event. Staying high is the same
// event and must not be re-sent: the car keeps a 2 s / 10 s event cooldown and
// would treat a replayed level as a new command. A box disappearing produces
// no action byte; only the 100 ms H heartbeat keeps the link alive.
// These three drawing paths are already in the HDMI pixel clock domain.
reg [4:0] traffic_visible_q;
always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n)begin
 if(!S_hdmi_rst_n)traffic_visible_q<=5'd0;
 else traffic_visible_q<=traffic_visible;
end
wire [4:0] traffic_event_bits=traffic_visible&~traffic_visible_q;
wire traffic_event_valid=|traffic_event_bits;

// Autonomous HC-05 car link, 9600 8N1; one-shot events become bytes.
// J1.9 / GPIOA_8 / R3 -> master HC-05 RXD. J1.11=5V, J1.12=GND.
// C+F/S/L/R is a queued one-shot byte; it cannot suppress visible events.
wire [7:0] S_bt_last;
wire S_bt_overflow; // delayed-delivery diagnostic; no spare board pin assigned
car_command_tx #(.CLK_FREQ(75000000)) u_car_command_tx (
 .I_clk(S_hdmi_pixel_clk),.I_rst_n(S_hdmi_rst_n),
 .I_event_classes(traffic_event_bits),.I_event_valid(traffic_event_valid),
 .I_manual_valid(S_bt_man_valid),.I_manual_byte(S_bt_man_byte),
 .O_last_cmd(S_bt_last),.O_tx(O_bt_tx),.O_overflow(S_bt_overflow));

// ---------------------------------------------------------------------------
// 板载 LED 状态指示（HX1P35B 板：LED1=D10  LED2=E10  LED3=C6  LED4=D6）
//   O_led[0] : 约 1.5Hz 心跳 —— 灯在闪就说明 FPGA 里跑的是本设计、时钟在动
//   O_led[1] : 画面里有颜色目标
//   O_led[2] : 画面里有运动目标
//   O_led[3] : 报警（红灯相位 + 有运动目标）
// 档位不在这里显示了 —— 数码管第 1 位本来就有模式号。
// 说明：LED 的"亮"对应 0 还是 1 取决于板上接法（常见为低电平点亮），
//       这里只保证"状态会变"，用来观察变化即可。
// ---------------------------------------------------------------------------
reg [25:0] S_led_cnt;
always @(posedge S_100m_clk or negedge S_rst_n) begin
    if(!S_rst_n) S_led_cnt <= 26'd0;
    else         S_led_cnt <= S_led_cnt + 26'd1;
end

assign O_led[0] = S_led_cnt[25];            // 心跳
assign O_led[1] = (S_tgt_num != 4'd0);      // 有颜色目标
assign O_led[2] = (S_mot_num != 4'd0);      // 有运动目标
assign O_led[3] = S_alarm;                  // 报警

// M2 颜色检测包围盒（ISP 时钟域）
wire        S_box_valid_isp;
wire [11:0] S_box_x0_isp, S_box_y0_isp, S_box_x1_isp, S_box_y1_isp;
wire        S_plate_valid_isp;
wire [11:0] S_plate_x0_isp, S_plate_y0_isp, S_plate_x1_isp, S_plate_y1_isp;
wire [1:0]  S_plate_type_isp;
wire [15:0] S_latency_isp;
wire        S_t0_valid_isp, S_t1_valid_isp;
wire [1:0]  S_t0_id_isp, S_t1_id_isp;
wire [2:0]  S_t0_type_isp, S_t1_type_isp;
wire [11:0] S_t0_x0_isp, S_t0_y0_isp, S_t0_x1_isp, S_t0_y1_isp;
wire [11:0] S_t1_x0_isp, S_t1_y0_isp, S_t1_x1_isp, S_t1_y1_isp;
wire         S_mt_tog_isp;
wire [15:0]  S_mt_num_isp;
wire [15:0]  S_mt_conf_isp;
wire [527:0] S_mt_tgt_isp;
// 批 3：低功耗降频的跨域快照 {sleep, 累计跳过帧数}
wire [16:0]  S_lp_xfer_isp;
wire         S_lp_tog_isp;

isp_top #(.ARROW_ENABLE(1),.TRAFFIC_DEFAULT_ON(1),
 .ARROW_DEFAULT_CFG({32'd1,12'd0,12'd0,12'd1279,12'd719,
                           12'd0,12'd0,12'd1279,12'd719})) u_isp_top (
.I_debug_view(camera_view_active),.I_camera_unstable(camera_unstable_sync[1]),
.O_camera_meter(camera_meter_i),.O_camera_meter_valid(camera_meter_iv),
.O_camera_snapshot(camera_snapshot_i),.O_camera_snapshot_valid(camera_snapshot_iv),
.I_arrow_cfg_valid(arrow_cfg_iv),.I_arrow_cfg(arrow_cfg_i),
.I_detect_cfg_valid(detect_cfg_iv),.I_detect_cfg(detect_cfg_i),.O_detect_status(detect_status_i),
.O_arrow_snapshot(arrow_snap_i),.O_arrow_snapshot_valid(arrow_snap_iv),
.O_visible_signs(display_signs_i),
.axi4s_video_aclk(S_csi_rx_clk),
.I_rst_n         (S_rst_n),
.I_tlast         (S_raw_tlast	),
.I_tuser         (S_raw_tuser	),
.I_tdata         (S_raw_tdata	),
.I_tvalid        (S_raw_tvalid	),
.I_tdest         (),
.O_tready        (S_ISP_O_tready),
.I_disp_mode     (S_final_disp_mode),
.O_tdata         (S_ISP_O_tdata ),
.O_tlast         (S_ISP_O_tlast ),
.O_tuser         (S_ISP_O_tuser ),
.O_tvalid        (S_ISP_O_tvalid),
        .I_tready        (),
        // 进-4：这条显示通路改接运动检测。
        //   O_box_* 在 isp_top 内部仍硬接 0（它被 target_tracker 当颜色目标输入用，
        //   本模块不再连接它，保持那一行为不变）。此处只把画框的源换成运动区域。
        .O_motion_valid  (S_box_valid_isp),
        .O_motion_x0     (S_box_x0_isp),
        .O_motion_y0     (S_box_y0_isp),
        .O_motion_x1     (S_box_x1_isp),
        .O_motion_y1     (S_box_y1_isp),
        .O_plate_valid   (S_plate_valid_isp),
        .O_plate_x0      (S_plate_x0_isp),
        .O_plate_y0      (S_plate_y0_isp),
        .O_plate_x1      (S_plate_x1_isp),
        .O_plate_y1      (S_plate_y1_isp),
        .O_plate_type    (S_plate_type_isp),
        .O_latency       (S_latency_isp),
        .O_t0_valid      (S_t0_valid_isp),
        .O_t0_id         (S_t0_id_isp),
        .O_t0_type       (S_t0_type_isp),
        .O_t0_x0         (S_t0_x0_isp),
        .O_t0_y0         (S_t0_y0_isp),
        .O_t0_x1         (S_t0_x1_isp),
        .O_t0_y1         (S_t0_y1_isp),
        .O_t1_valid      (S_t1_valid_isp),
        .O_t1_id         (S_t1_id_isp),
        .O_t1_type       (S_t1_type_isp),
        .O_t1_x0         (S_t1_x0_isp),
        .O_t1_y0         (S_t1_y0_isp),
        .O_t1_x1         (S_t1_x1_isp),
        .O_t1_y1         (S_t1_y1_isp),
        .O_mt_tog        (S_mt_tog_isp),
        .O_mt_num        (S_mt_num_isp),
        .O_mt_conf       (S_mt_conf_isp),
        .O_mt_tgt        (S_mt_tgt_isp),
        .O_lp_xfer       (S_lp_xfer_isp),
        .O_lp_tog        (S_lp_tog_isp),
        .I_thr_red_min   (S_thr_red_min_isp),
        .I_thr_red_gap   (S_thr_red_gap_isp),
        .I_thr_grn_min   (S_thr_grn_min_isp),
        .I_thr_grn_gap   (S_thr_grn_gap_isp),
        .I_thr_yel_min   (S_thr_yel_min_isp),
        .I_thr_yel_bdiff (S_thr_yel_bdiff_isp),
        .I_thr_yel_bmax  (S_thr_yel_bmax_isp),
        .I_thr_blk_pix   (S_thr_blk_pix_isp),
        .I_thr_min_blk   (S_thr_min_blk_isp),
        .I_lp_en         (S_lp_en_i2)
  );


    video_in u_video_in(
        .I_rst_n              (  S_rst_n             ),

        .I_camera_clk         ( S_csi_rx_clk          ),
        .I_camera_frame_start ( S_ISP_O_tuser  ),
        .I_camera_valid       ( S_ISP_O_tvalid ),
        .I_camera_data        ( S_ISP_O_tdata  ),
        .I_mipi_rx_error      ( 1'b0			      ),

        .I_ddr_clk            ( S_ddr_clk             ),
		.I_display_pause      ( 1'b0                  ),
        .I_video_out_rd_busy  ( S_video_out_rd_busy   ),
        .O_video_in_wr_busy   ( S_video_in_wr_busy    ),
        .O_video_out_rp       ( S_video_out_rp        ),

        .O_ddr_user_wr_en     ( S_vi_ddr_wr_en        ),
        .O_ddr_user_addr      ( S_vi_ddr_wr_addr      ),
        .O_ddr_user_wr_data   ( S_vi_ddr_wr_data      ),
        .I_ddr_user_ready     ( S_ddr_user_ready      )
    );



    assign S_ddr_user_wr_en    = S_vi_ddr_wr_en;

    assign S_ddr_user_rd_en    = S_vo_ddr_rd_en;

    assign S_ddr_user_addr     = S_vi_ddr_wr_en ? S_vi_ddr_wr_addr :
                                 S_vo_ddr_rd_en ? S_vo_ddr_rd_addr : 'd0;

    assign S_ddr_user_wr_data  = S_vi_ddr_wr_data;


    assign S_video_out_rst_n = S_hdmi_rst_n;

    video_out u_video_out(
        .I_rst_n             ( S_video_out_rst_n    ),
        .I_ddr_clk           ( S_ddr_clk            ),
 
        .O_video_out_rd_busy ( S_video_out_rd_busy  ),
        .I_video_in_wr_busy  ( S_video_in_wr_busy   ),
        .I_video_out_rp      ( S_video_out_rp       ),
 
        .O_ddr_user_rd_en    ( S_vo_ddr_rd_en       ),
        .O_ddr_user_addr     ( S_vo_ddr_rd_addr     ),
        .I_ddr_user_ready    ( S_ddr_user_ready     ),
        .I_ddr_user_rd_valid ( S_ddr_user_rd_valid  ),
        .I_ddr_user_rd_data  ( S_ddr_user_rd_data   ),

        .I_dsi_clk           ( S_hdmi_pixel_clk     ),
        .I_video_vsync       ( S_video_out_vsync    ),
        .I_video_rd_en       ( S_hdmi_window_rd_en  ),
        .O_vdieo_data        ( S_video_rd_data      )
    );



    mc_to_user_interface u_mc_to_user_interface(
        .I_clk                   ( S_ddr_clk               ),
        .I_rst_n                 ( S_rst_n                 ),

        .I_ddr_user_wr_en        ( S_ddr_user_wr_en        ),
        .I_ddr_user_rd_en        ( S_ddr_user_rd_en        ),
        .I_ddr_user_addr         ( S_ddr_user_addr         ),
        .I_ddr_user_wr_data      ( S_ddr_user_wr_data      ),
        .O_ddr_user_ready        ( S_ddr_user_ready        ),
        .O_ddr_user_rd_valid     ( S_ddr_user_rd_valid     ),
        .O_ddr_user_rd_data      ( S_ddr_user_rd_data      ),
            
        .O_mc_app_en             ( S_mc_app_en             ),
        .O_mc_app_addr           ( S_mc_app_addr           ),
        .O_mc_app_cmd            ( S_mc_app_cmd            ),
        .I_mc_app_rdy            ( S_mc_app_rdy            ),
        .O_mc_app_wdf_wren       ( S_mc_app_wdf_wren       ),
        .O_mc_app_wdf_data       ( S_mc_app_wdf_data       ),
        .O_mc_app_wdf_end        ( S_mc_app_wdf_end        ),
        .O_mc_app_wdf_mask       ( S_mc_app_wdf_mask       ),
        .I_mc_app_wdf_rdy        ( S_mc_app_wdf_rdy        ),
        .I_mc_app_rd_data        ( S_mc_app_rd_data        ),
        .I_mc_app_rd_data_end    ( S_mc_app_rd_data_end    ),
        .I_mc_app_rd_data_valid  ( S_mc_app_rd_data_valid  )
    );



    ph1p35_324_ddr_wrapper u_ph1p35_324_ddr_wrapper(
        .I_sys_clk              ( I_sys_clk              ),
        .I_sys_rst_n            ( S_rst_n                ),

        .O_ddr_clk              ( S_ddr_clk              ),
        .O_init_calib_complete  ( S_init_calib_complete  ),
        .I_mc_app_addr          ( S_mc_app_addr          ),
        .I_mc_app_cmd           ( S_mc_app_cmd           ),
        .I_mc_app_en            ( S_mc_app_en            ),
        .I_mc_app_wdf_data      ( S_mc_app_wdf_data      ),
        .I_mc_app_wdf_end       ( S_mc_app_wdf_end       ),
        .I_mc_app_wdf_mask      ( S_mc_app_wdf_mask      ),
        .I_mc_app_wdf_wren      ( S_mc_app_wdf_wren      ),
        .O_mc_app_rd_data       ( S_mc_app_rd_data       ),
        .O_mc_app_rd_data_end   ( S_mc_app_rd_data_end   ),
        .O_mc_app_rd_data_valid ( S_mc_app_rd_data_valid ),
        .O_mc_app_rdy           ( S_mc_app_rdy           ),
        .O_mc_app_wdf_rdy       ( S_mc_app_wdf_rdy       ),

        .ddr_addr               ( ddr_addr               ),
        .ddr_ba                 ( ddr_ba                 ),
        .ddr_cke                ( ddr_cke                ),
        .ddr_odt                ( ddr_odt                ),
        .ddr_cs_n               ( ddr_cs_n               ),
        .ddr_ras_n              ( ddr_ras_n              ),
        .ddr_cas_n              ( ddr_cas_n              ),
        .ddr_we_n               ( ddr_we_n               ),
        .ddr_ck_p               ( ddr_ck_p               ),
        .ddr_ck_n               ( ddr_ck_n               ),
        .ddr_dm                 ( ddr_dm                 ),
        .ddr_dq                 ( ddr_dq                 ),
        .ddr_dqs_p              ( ddr_dqs_p              ),
        .ddr_dqs_n              ( ddr_dqs_n              )
    );



    uivtc #(
        .H_ActiveSize ( 1280 ),
        .H_FrameSize  ( 1650 ),
        .H_SyncStart  ( 1390 ),
        .H_SyncEnd    ( 1430 ),
        .V_ActiveSize ( 720  ),
        .V_FrameSize  ( 750  ),
        .V_SyncStart  ( 725  ),
        .V_SyncEnd    ( 730  )
    )u_hdmi_vtc(
        .I_vtc_rstn    ( S_hdmi_rst_n    ),
        .I_vtc_clk     ( S_hdmi_pixel_clk ),
        .O_vtc_vs      ( S_hdmi_vsync     ),
        .O_vtc_hs      ( S_hdmi_hsync     ),
        .O_vtc_de_valid( S_hdmi_de        ),
        .O_vtc_user    ( S_hdmi_user      ),
        .O_vtc_last    ( S_hdmi_last      )
    );

    // 包围盒跨时钟域：ISP 域 -> HDMI 域（两级同步，每帧只变一次，够用）
reg        S_box_valid_h1, S_box_valid_h2;
reg [47:0] S_box_h1, S_box_h2;
always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
    if (!S_hdmi_rst_n) begin
        S_box_valid_h1 <= 1'b0;
        S_box_valid_h2 <= 1'b0;
        S_box_h1 <= 48'd0;
        S_box_h2 <= 48'd0;
    end
    else begin
        S_box_valid_h1 <= S_box_valid_isp;
        S_box_valid_h2 <= S_box_valid_h1;
        S_box_h1 <= {S_box_x0_isp, S_box_y0_isp, S_box_x1_isp, S_box_y1_isp};
        S_box_h2 <= S_box_h1;
    end
end


// 目标跟踪跨时钟域：ISP 域 -> HDMI 域（两级同步，每帧只变一次）
always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
    if (!S_hdmi_rst_n) begin
        S_t0_valid_h1 <= 1'b0; S_t0_valid_h2 <= 1'b0;
        S_t1_valid_h1 <= 1'b0; S_t1_valid_h2 <= 1'b0;
        S_t0_h1 <= 53'd0; S_t0_h2 <= 53'd0;
        S_t1_h1 <= 53'd0; S_t1_h2 <= 53'd0;
    end
    else begin
        S_t0_valid_h1 <= S_t0_valid_isp;
        S_t0_valid_h2 <= S_t0_valid_h1;
        S_t0_h1 <= {S_t0_id_isp, S_t0_type_isp, S_t0_x0_isp, S_t0_y0_isp, S_t0_x1_isp, S_t0_y1_isp};
        S_t0_h2 <= S_t0_h1;
        S_t1_valid_h1 <= S_t1_valid_isp;
        S_t1_valid_h2 <= S_t1_valid_h1;
        S_t1_h1 <= {S_t1_id_isp, S_t1_type_isp, S_t1_x0_isp, S_t1_y0_isp, S_t1_x1_isp, S_t1_y1_isp};
        S_t1_h2 <= S_t1_h1;
    end
end

// 阶段A：模式/延迟/AE/AWB 跨时钟域同步到 HDMI 像素域
reg [2:0]  S_style_h1, S_style_h2;
reg [15:0] S_latency_h1, S_latency_h2;
reg        S_ae_h1, S_ae_h2, S_awb_h1, S_awb_h2;
always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
    if (!S_hdmi_rst_n) begin
        S_style_h1   <= 3'd0;
        S_style_h2   <= 3'd0;
        S_latency_h1 <= 16'd0;
        S_latency_h2 <= 16'd0;
        S_ae_h1 <= 1'b0;
        S_ae_h2 <= 1'b0;
        S_awb_h1 <= 1'b0;
        S_awb_h2 <= 1'b0;
    end
    else begin
        S_style_h1   <= S_final_disp_mode;
        S_style_h2   <= S_style_h1;
        S_latency_h1 <= S_latency_isp;
        S_latency_h2 <= S_latency_h1;
        S_ae_h1 <= S_ae_cfg_done;
        S_ae_h2 <= S_ae_h1;
        S_awb_h1 <= S_cam_cfg_done;
        S_awb_h2 <= S_awb_h1;
    end
end

// ---- å¤ç®æ é¢è²åï¼ISP å -> HDMI åï¼æ´å¸§å¿«ç§ + 1 ä½ toggle æ¡æ ----
// 不逐位打两拍：588 位数据在上游每帧只更新一次、保持稳定，
// HDMI 域只在 toggle 跳变时整体锁存一次。既避免多比特跨域采到
// 半新半旧的值，也比逐位同步省一半触发器。
reg         S_mt_tog_h1, S_mt_tog_h2, S_mt_tog_h3;
reg [15:0]  S_mt_num_hd;
reg [15:0]  S_mt_conf_hd;
reg [527:0] S_mt_tgt_hd;
always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
    if (!S_hdmi_rst_n) begin
        S_mt_tog_h1  <= 1'b0;
        S_mt_tog_h2  <= 1'b0;
        S_mt_tog_h3  <= 1'b0;
        S_mt_num_hd  <= 16'd0;
        S_mt_conf_hd <= 16'd0;
        S_mt_tgt_hd  <= 528'd0;
    end
    else begin
        S_mt_tog_h1 <= S_mt_tog_isp;
        S_mt_tog_h2 <= S_mt_tog_h1;
        S_mt_tog_h3 <= S_mt_tog_h2;
        if (S_mt_tog_h2 ^ S_mt_tog_h3) begin
            S_mt_num_hd  <= S_mt_num_isp;
            S_mt_tgt_hd  <= S_mt_tgt_isp;
            S_mt_conf_hd <= S_mt_conf_isp;
        end
    end
end

// ---- 批 3：低功耗快照跨时钟域（同一套 toggle 握手）----
//   ISP 域只在"值变了"的时候换快照并翻 toggle；这里用 3 拍同步 + 边沿检测整组锁存，
//   不做逐位同步（17 位里 16 位是计数器，逐位同步会花）。快照在两个更新点之间
//   至少稳定一整帧（16.7ms），3 拍同步（40ns）远远够。
reg         S_lp_tog_h1, S_lp_tog_h2, S_lp_tog_h3;
reg [16:0]  S_lp_xfer_hd;
wire [15:0] S_skip_cnt_hd = S_lp_xfer_hd[15:0];   // 累计跳过的帧数
wire        S_lp_sleep_hd = S_lp_xfer_hd[16];     // 当前是否在降频
always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
    if (!S_hdmi_rst_n) begin
        S_lp_tog_h1  <= 1'b0;
        S_lp_tog_h2  <= 1'b0;
        S_lp_tog_h3  <= 1'b0;
        S_lp_xfer_hd <= 17'd0;
    end
    else begin
        S_lp_tog_h1 <= S_lp_tog_isp;
        S_lp_tog_h2 <= S_lp_tog_h1;
        S_lp_tog_h3 <= S_lp_tog_h2;
        if (S_lp_tog_h2 ^ S_lp_tog_h3) S_lp_xfer_hd <= S_lp_xfer_isp;
    end
end

// ---- 串口上报：把跟踪结果发送到上位机（UART_TX = H1）----
// 圆形目标个数：12 个目标槽（每槽 44bit，bit0 是形状位）按 valid 求和
integer d_circ_i, d_circ_j;
reg [4:0] S_circ_sum;
always @(*) begin
    S_circ_sum = 5'd0;
    for (d_circ_i = 0; d_circ_i < 4; d_circ_i = d_circ_i + 1)
        for (d_circ_j = 0; d_circ_j < 3; d_circ_j = d_circ_j + 1)
            S_circ_sum = S_circ_sum +
                {4'd0, (S_mt_tgt_hd[d_circ_i*132 + d_circ_j*44 + 43] &
                        S_mt_tgt_hd[d_circ_i*132 + d_circ_j*44 + 0])};
end
wire [3:0] S_circ_hd = (S_circ_sum > 5'd9) ? 4'd9 : S_circ_sum[3:0];

// ---- 场景闭环的决策逻辑 ----
// 目标总数 = 红 + 绿 + 黄（S_mt_num_hd 的 [3:0]/[7:4]/[11:8]），封顶 9
wire [5:0] S_tgt_sum = {2'd0, S_mt_num_hd[3:0]}
                     + {2'd0, S_mt_num_hd[7:4]}
                     + {2'd0, S_mt_num_hd[11:8]};
assign S_tgt_num = (S_tgt_sum > 6'd9) ? 4'd9 : S_tgt_sum[3:0];
assign S_mot_num = S_mt_num_hd[15:12];   // 运动目标数

// 报警条件：红灯相位 + 画面里有运动目标（当作"有车在动"）
//   为什么不用"红目标数 > 0"：交通灯的灯面本身就是画面里的一个红色
//   圆形目标，拿它当"有车"必然恒真、没有信息量。
assign S_alarm   = S_is_light_h2 && (S_light_col_h2 == 2'd0)
                 && (S_mot_num != 4'd0);

// 置信度：4 个类别取最大，OSD 和串口共用这一个值
integer d_conf_i;
always @(*) begin
    S_conf_hd = 4'd0;
    for (d_conf_i = 0; d_conf_i < 4; d_conf_i = d_conf_i + 1)
        if (S_mt_conf_hd[d_conf_i*4 +: 4] > S_conf_hd)
            S_conf_hd = S_mt_conf_hd[d_conf_i*4 +: 4];
end

uart_report #(
    .CLK_FREQ(75000000),.ARROW_FEATURE(1),.DETECT_FEATURE(1),.CAMERA_FEATURE(1)
) u_uart_report (
    .I_camera_snapshot({IQ_BUILD_ID,camera_control_h,camera_snapshot_h}),
    .I_arrow_valid(arrow_snap_hv),.I_arrow_snapshot(arrow_snap_h),.I_detect_status(detect_status_h),
    .I_clk     (S_hdmi_pixel_clk),
    .I_rst_n   (S_hdmi_rst_n),
    .I_valid   (S_t0_valid_h2 && detect_status_h[7]),
    .I_id      (S_t0_h2[52:51]),
    .I_type    (S_t0_h2[50:48]),
    .I_x       (S_t0_h2[47:36]),
    .I_y       (S_t0_h2[35:24]),
    .I_mt_num  (S_mt_num_hd[11:0]),
    .I_circ    (S_circ_hd),
    .I_conf    (S_conf_hd),
    .I_bt_cmd  (S_bt_last),
    .I_skip_cnt(S_skip_cnt_hd),
    .I_lp_active(S_lp_sleep_hd),
    .I_thr     ({S_thr_red_min, S_thr_red_gap, S_thr_grn_min, S_thr_grn_gap,
                 S_thr_yel_min, S_thr_yel_bdiff, S_thr_yel_bmax,
                 S_thr_blk_pix, S_thr_min_blk}),
    .O_uart_tx (O_uart_tx)
);

// ======================================================================
// 端到端延迟：相机帧首 -> HDMI 显示帧首
//   这个设计的帧缓存是"读指针 = 写指针 + 2 帧"（见 video_in.v 里的 case），
//   所以屏幕上这一帧其实是两帧之前拍的，端到端延迟约 2 个帧周期 ≈ 33 ms。
//   因为读指针是由写指针推导出来的，不存在两个时钟之间的拍频，
//   这个数字是稳定的，不会在 1 帧/2 帧之间来回跳。
//   时间基准：HDMI 像素时钟 74.25MHz 除以 74，约等于 1 微秒。
// ======================================================================
reg [6:0]  S_us_div;
reg [19:0] S_us_cnt;
always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
    if (!S_hdmi_rst_n) begin
        S_us_div <= 7'd0;
        S_us_cnt <= 20'd0;
    end
    else if (S_us_div == 7'd73) begin
        S_us_div <= 7'd0;
        S_us_cnt <= (S_us_cnt == 20'hFFFFF) ? 20'd0 : (S_us_cnt + 20'd1);
    end
    else begin
        S_us_div <= S_us_div + 7'd1;
    end
end

// 相机帧首（ISP 域）跨到 HDMI 域：toggle 握手，避免多比特裸同步
reg S_cam_fs_tog;
always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
    if (!S_rst_n)         S_cam_fs_tog <= 1'b0;
    else if (S_raw_tuser) S_cam_fs_tog <= ~S_cam_fs_tog;
end

reg        S_cam_fs_1, S_cam_fs_2, S_cam_fs_3;
reg [19:0] S_t_cam, S_t_disp, S_e2e_us;
always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
    if (!S_hdmi_rst_n) begin
        S_cam_fs_1 <= 1'b0;
        S_cam_fs_2 <= 1'b0;
        S_cam_fs_3 <= 1'b0;
        S_t_cam    <= 20'd0;
        S_t_disp   <= 20'd0;
        S_e2e_us   <= 20'd0;
    end
    else begin
        S_cam_fs_1 <= S_cam_fs_tog;
        S_cam_fs_2 <= S_cam_fs_1;
        S_cam_fs_3 <= S_cam_fs_2;
        if (S_cam_fs_2 ^ S_cam_fs_3) S_t_cam <= S_us_cnt;    // 记下最近一次相机帧首时刻
        if (S_hdmi_user) begin
            S_t_disp <= S_us_cnt;                            // 显示帧首时刻
            S_e2e_us <= S_us_cnt - S_t_cam;                  // 差值就是端到端延迟（微秒）
        end
    end
end

hdmi_mixer #(
        .H_OFFSET   ( 0    ),
        .V_OFFSET   ( 0    ),
        .IMG_WIDTH  ( 1280 ),
        .IMG_HEIGHT ( 720  ),
        .DEBUG_MODE ( 0    )
    )u_hdmi_mixer(
        .I_clk           ( S_hdmi_pixel_clk   ),
        .I_rst_n         ( S_hdmi_rst_n       ),
        .I_video_vsync   ( S_hdmi_vsync       ),
        .I_video_hsync   ( S_hdmi_hsync       ),
        .I_video_de      ( S_hdmi_de          ),
        .I_video_user    ( S_hdmi_user        ),
        .I_video_last    ( S_hdmi_last        ),
        .I_debug_status  ( S_hdmi_debug_status),
        .O_video_rd_en   ( S_hdmi_window_rd_en),
        .I_video_rd_data ( S_video_rd_data    ),
        .I_box_valid (1'b0), // V5 draws only the filtered region boxes
        .I_box_x0    (S_box_h2[47:36]),
        .I_box_y0    (S_box_h2[35:24]),
        .I_box_x1    (S_box_h2[23:12]),
                .I_box_y1    (S_box_h2[11: 0]),
        .I_style        (S_style_h2),
        .I_latency      (S_latency_h2),
        .I_ae_on        (S_ae_h2),
        .I_awb_on       (S_awb_h2),
        .I_t0_valid (S_t0_valid_h2 && detect_status_h[7]),
        .I_t0_id    (S_t0_h2[52:51]),
        .I_t0_type  (S_t0_h2[50:48]),
        .I_t0_x0    (S_t0_h2[47:36]),
        .I_t0_y0    (S_t0_h2[35:24]),
        .I_t0_x1    (S_t0_h2[23:12]),
        .I_t0_y1    (S_t0_h2[11: 0]),
        .I_t1_valid (S_t1_valid_h2 && detect_status_h[7]),
        .I_t1_id    (S_t1_h2[52:51]),
        .I_t1_type  (S_t1_h2[50:48]),
        .I_t1_x0    (S_t1_h2[47:36]),
        .I_t1_y0    (S_t1_h2[35:24]),
        .I_t1_x1    (S_t1_h2[23:12]),
        .I_t1_y1    (S_t1_h2[11: 0]),
        .I_mt_num   (S_mt_num_hd),
        .I_mt_tgt   (S_mt_tgt_hd),
    .I_conf     (S_conf_hd),
        .I_e2e_us   (S_e2e_us),
        .I_cd_t     (S_cd_t),
        .I_cd_u     (S_cd_u),
        .I_cd_phase (S_cd_phase),
        .I_cd_vld   (S_cd_vld),
        .I_lp_sleep (S_lp_sleep_hd),
        .O_hdmi_vsync    ( S_hdmi_out_vsync   ),
        .O_hdmi_hsync    ( S_hdmi_out_hsync   ),
        .O_hdmi_de       ( S_hdmi_out_de      ),
        .O_hdmi_data     ( S_hdmi_out_data    ),
        .O_visible_lamps(visible_lamps)
    );

    wire [23:0] arrow_display_data;
    arrow_overlay overlay(.I_clk(S_hdmi_pixel_clk),.I_rst_n(S_hdmi_rst_n),
      .I_vs(S_hdmi_out_vsync),.I_de(S_hdmi_out_de),.I_rgb(S_hdmi_out_data),
      .I_update(arrow_snap_hv|detect_status_iv),.I_snapshot(detect_status_h[7]?arrow_snap_h:192'd0),.O_rgb(arrow_display_data),.O_visible_turn(visible_turn));
    hdmi_tx u_hdmi_tx(
        .I_pixel_clk        ( S_hdmi_pixel_clk  ),
        .I_serial_clk       ( S_hdmi_serial_clk ),
        .I_rst              ( ~S_hdmi_rst_n     ),
        .I_key_in           ( 1'b0              ),
        .I_edid_read_trig   ( 1'b0              ),
        .O_edid_read_valid  (                   ),
        .O_edid_read_data   (                   ),
        .I_video_rgb_enable ( 1'b1              ),
        .I_video_in_vs      ( S_hdmi_out_vsync  ),
        .I_video_in_de      ( S_hdmi_out_de     ),
        .I_video_in_user    ( 1'b0              ),
        .I_video_in_valid   ( 1'b0              ),
        .I_video_in_last    ( 1'b0              ),
        .O_video_in_ready   (                   ),
        .I_video_in_data    ( arrow_display_data ),
        .I_audio_valid      ( 1'b0              ),
        .I_audio_left_data  ( 24'd0             ),
        .I_audio_right_data ( 24'd0             ),
        .I_i2s_BCLK         ( 1'b0              ),
        .I_i2s_LRCK         ( 1'b0              ),
        .I_i2s_DOUT         ( 1'b0              ),
        .O_ddc_scl          (                   ),
        .O_hdmi_clk_p       (                   ),
        .O_hdmi_tx_p        (                   ),
        .O_tmds_ch0_p       ( O_tmds_ch0_p      ),
        .O_tmds_ch1_p       ( O_tmds_ch1_p      ),
        .O_tmds_ch2_p       ( O_tmds_ch2_p      ),
        .O_tmds_clk_p       ( O_tmds_clk_p      )
    );


    
endmodule
