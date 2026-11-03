// Frame-stable white balance, four RGB888 pixels per beat.
// Statistics include only moderately bright near-neutral pixels. Saturated
// traffic-light colors cannot drive the gains toward gray. A complete frame
// is snapshotted once; the result is applied only at a later frame boundary.
// Gain defaults/ranges are engineering starting points, not color calibration.
module awb #(
    parameter IMG_HEIGHT = 1080,
    parameter IMG_WIDTH = 1920,
    parameter integer MIN_NEUTRAL_PIX = 1024,
    parameter [7:0] NEUTRAL_MIN = 8'd16,
    parameter [7:0] NEUTRAL_MAX = 8'd240,
    parameter [19:0] GAIN_MIN_Q16 = 20'h04000, // 0.25x
    parameter [19:0] GAIN_MAX_Q16 = 20'h40000  // 4.0x, accommodates warm-light blue loss
)(
    input wire I_clk,
    input wire I_rst_n,
    input wire I_tlast,
    input wire I_tuser,
    input wire [95:0] I_tdata,
    input wire I_tvalid,
    output wire I_tready,
    output reg O_tlast,
    output reg O_tuser,
    output reg [95:0] O_tdata,
    output reg O_tvalid,
    input wire O_tready,
    output wire [19:0] O_gain_r,O_gain_b,
    output reg [19:0] O_samples,
    output reg O_samples_valid,
    output reg O_frame_stats
);
    localparam integer FRAME_BEATS = (IMG_WIDTH / 4) * IMG_HEIGHT;
    localparam [19:0] UNITY = 20'h10000;

    // Whole pipeline has two registers and one common clock-enable. When
    // downstream stalls, pixels and ALL stream flags remain together.
    reg valid_s1, last_s1, user_s1;
    wire advance = !O_tvalid || O_tready;
    assign I_tready = advance;
    wire accept = advance && I_tvalid;
    reg user_prev;
    wire sof = advance && I_tuser && !user_prev;

    function is_neutral;
        input [23:0] pixel;
        reg [7:0] hi, lo, r, g, b;
        begin
            r = pixel[23:16]; g = pixel[15:8]; b = pixel[7:0];
            hi = (r > g) ? r : g;
            hi = (hi > b) ? hi : b;
            lo = (r < g) ? r : g;
            lo = (lo < b) ? lo : b;
            // Bootstrap on plausible warm/cool neutrals too. The old 1.5:1
            // gate rejected the very white points AWB needed to correct.
            // Keep strongly saturated lamps/signs out: R/G within 0.5..1.5,
            // B/G within 0.25..2, and max/min no greater than four.
            is_neutral = (lo >= NEUTRAL_MIN) && (hi <= NEUTRAL_MAX) &&
                         ({2'b0,hi} <= {lo,2'b00}) &&
                         ({1'b0,r} >= {2'b0,g[7:1]}) &&
                         ({1'b0,r} <= {1'b0,g}+{2'b0,g[7:1]}) &&
                         ({2'b0,b} >= {4'b0,g[7:2]}) &&
                         ({1'b0,b} <= {g,1'b0});
        end
    endfunction

    wire [23:0] px [0:3];
    wire [3:0] neutral;
    wire [7:0] sr [0:3], sg [0:3], sb [0:3];
    genvar n;
    generate for (n=0; n<4; n=n+1) begin: samples
        assign px[n] = I_tdata[n*24 +: 24];
        assign neutral[n] = is_neutral(px[n]);
        assign sr[n] = neutral[n] ? px[n][23:16] : 8'd0;
        assign sg[n] = neutral[n] ? px[n][15:8] : 8'd0;
        assign sb[n] = neutral[n] ? px[n][7:0] : 8'd0;
    end endgenerate
    wire [9:0] beat_r = {2'b0,sr[0]} + {2'b0,sr[1]} + {2'b0,sr[2]} + {2'b0,sr[3]};
    wire [9:0] beat_g = {2'b0,sg[0]} + {2'b0,sg[1]} + {2'b0,sg[2]} + {2'b0,sg[3]};
    wire [9:0] beat_b = {2'b0,sb[0]} + {2'b0,sb[1]} + {2'b0,sb[2]} + {2'b0,sb[3]};
    wire [2:0] beat_count = {2'b0,neutral[0]} + {2'b0,neutral[1]} +
                           {2'b0,neutral[2]} + {2'b0,neutral[3]};
    reg [31:0] sum_r, sum_g, sum_b, sample_count, frame_beats;
    reg frame_active;
    reg [31:0] snapshot_r, snapshot_g, snapshot_b;
    reg calc_start;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            user_prev <= 0;
            sum_r <= 0; sum_g <= 0; sum_b <= 0; sample_count <= 0;
            frame_beats <= 0; frame_active <= 0;
            snapshot_r <= 0; snapshot_g <= 0; snapshot_b <= 0;
            calc_start <= 0;
            O_samples<=0;O_samples_valid<=0;O_frame_stats<=0;
        end else begin
            calc_start <= 0;
            O_frame_stats<=0;
            if (advance) user_prev <= I_tuser;
            if (sof) begin
                sum_r <= accept ? beat_r : 0;
                sum_g <= accept ? beat_g : 0;
                sum_b <= accept ? beat_b : 0;
                sample_count <= accept ? beat_count : 0;
                frame_beats <= accept ? 1 : 0;
                frame_active <= 1;
            end else if (accept && frame_active) begin
                sum_r <= sum_r + beat_r;
                sum_g <= sum_g + beat_g;
                sum_b <= sum_b + beat_b;
                sample_count <= sample_count + beat_count;
                frame_beats <= frame_beats + 1;
                if (frame_beats == FRAME_BEATS-1) begin
                    frame_active <= 0;
                    O_samples<=sample_count+beat_count;
                    O_samples_valid<=I_tlast && sample_count+beat_count>=MIN_NEUTRAL_PIX;
                    O_frame_stats<=I_tlast;
                    // Never calculate from a truncated frame or a few pixels.
                    if (I_tlast && sample_count + beat_count >= MIN_NEUTRAL_PIX) begin
                        snapshot_r <= sum_r + beat_r;
                        snapshot_g <= sum_g + beat_g;
                        snapshot_b <= sum_b + beat_b;
                        calc_start <= 1;
                    end
                end
            end
        end
    end

    wire [19:0] ratio_r, ratio_b;
    wire done_r, done_b;
    div_q16_sat20 u_gain_r (.I_clk(I_clk), .I_rst_n(I_rst_n), .I_start(calc_start),
        .I_a(snapshot_g), .I_b(snapshot_r), .O_done(done_r), .O_q(ratio_r));
    div_q16_sat20 u_gain_b (.I_clk(I_clk), .I_rst_n(I_rst_n), .I_start(calc_start),
        .I_a(snapshot_g), .I_b(snapshot_b), .O_done(done_b), .O_q(ratio_b));

    function [19:0] smoothed_gain;
        input [19:0] current, ratio;
        reg [19:0] target;
        reg [20:0] delta;
        begin
            target = (ratio < GAIN_MIN_Q16) ? GAIN_MIN_Q16 :
                     (ratio > GAIN_MAX_Q16) ? GAIN_MAX_Q16 : ratio;
            if (target >= current) begin
                delta = {1'b0,target} - {1'b0,current} + 21'd4;
                smoothed_gain = current + (delta >> 3);
            end else begin
                delta = {1'b0,current} - {1'b0,target} + 21'd4;
                smoothed_gain = current - (delta >> 3);
            end
        end
    endfunction

    reg [19:0] active_r, active_b, pending_r, pending_b;
    assign O_gain_r=active_r;
    assign O_gain_b=active_b;
    reg pending_valid;
    wire [19:0] use_r = (sof && pending_valid) ? pending_r : active_r;
    wire [19:0] use_b = (sof && pending_valid) ? pending_b : active_b;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            active_r <= UNITY; active_b <= UNITY;
            pending_r <= UNITY; pending_b <= UNITY; pending_valid <= 0;
        end else begin
            if (sof && pending_valid) begin
                active_r <= pending_r; active_b <= pending_b;
                pending_valid <= 0;
            end
            if (done_r && done_b) begin
                pending_r <= smoothed_gain(active_r, ratio_r);
                pending_b <= smoothed_gain(active_b, ratio_b);
                pending_valid <= 1;
            end
        end
    end

    reg [27:0] product_r [0:3], product_b [0:3];
    reg [7:0] green_s1 [0:3];
    function [7:0] quantize;
        input [27:0] value;
        reg [28:0] rounded;
        begin
            rounded = {1'b0,value} + 29'd32768;
            quantize = (|rounded[28:24]) ? 8'hff : rounded[23:16];
        end
    endfunction
    integer i;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            valid_s1 <= 0; user_s1 <= 0; last_s1 <= 0;
            O_tvalid <= 0; O_tuser <= 0; O_tlast <= 0; O_tdata <= 0;
            for (i=0; i<4; i=i+1) begin
                product_r[i] <= 0; product_b[i] <= 0; green_s1[i] <= 0;
            end
        end else if (advance) begin
            valid_s1 <= I_tvalid; user_s1 <= I_tuser; last_s1 <= I_tlast;
            O_tvalid <= valid_s1; O_tuser <= user_s1; O_tlast <= last_s1;
            for (i=0; i<4; i=i+1) begin
                if (I_tvalid) begin
                    product_r[i] <= use_r * px[i][23:16];
                    product_b[i] <= use_b * px[i][7:0];
                    green_s1[i] <= px[i][15:8];
                end
                O_tdata[i*24 +: 24] <= valid_s1 ?
                    {quantize(product_r[i]), green_s1[i], quantize(product_b[i])} : 24'd0;
            end
        end
    end
endmodule

// Q16 unsigned ratio using ALL 32 dividend bits; no truncation above bit 27.
// One division per accepted frame, 48 cycles, saturated to the 20-bit result.
module div_q16_sat20 (
    input wire I_clk, I_rst_n, I_start,
    input wire [31:0] I_a, I_b,
    output reg O_done,
    output reg [19:0] O_q
);
    reg [47:0] dividend, quotient, remainder;
    reg [31:0] divisor;
    reg [5:0] count;
    reg busy;
    wire [47:0] shifted = {remainder[46:0], dividend[47]};
    wire ge = shifted >= {16'd0,divisor};
    wire [47:0] next_quotient = {quotient[46:0], ge};
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            dividend <= 0; quotient <= 0; remainder <= 0; divisor <= 0;
            count <= 0; busy <= 0; O_done <= 0; O_q <= 0;
        end else begin
            O_done <= 0;
            if (!busy && I_start) begin
                dividend <= {I_a,16'd0}; divisor <= I_b;
                quotient <= 0; remainder <= 0; count <= 48; busy <= 1;
            end else if (busy) begin
                remainder <= ge ? shifted - {16'd0,divisor} : shifted;
                dividend <= {dividend[46:0],1'b0};
                quotient <= next_quotient; count <= count - 1;
                if (count == 1) begin
                    busy <= 0; O_done <= 1;
                    O_q <= (|next_quotient[47:20]) ? 20'hfffff : next_quotient[19:0];
                end
            end
        end
    end
endmodule
