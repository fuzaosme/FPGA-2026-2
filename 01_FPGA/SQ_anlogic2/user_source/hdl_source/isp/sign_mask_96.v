`timescale 1ns/1ps
module sign_mask_96(input wire I_valid, input wire [95:0] I_rgb,
 input wire [11:0] I_x,I_y, input wire [47:0] I_roi, input wire [7:0] I_level,I_white_level, input wire I_color_mode,I_bright_mode,
 output wire [3:0] O_blue,O_white, output wire O_color);
assign O_color=lane[0].in_roi && lane[0].blue_pixel;
genvar p;
generate for(p=0;p<4;p=p+1) begin: lane
 // Recognition uses the representative second pixel of each beat.
 wire [7:0] r=I_rgb[71:64],g=I_rgb[63:56],b=I_rgb[55:48];
 wire [11:0] x=I_x+1;
 wire in_roi=I_valid && x>=I_roi[47:36] && x<=I_roi[23:12] && I_y>=I_roi[35:24] && I_y<=I_roi[11:0];
 wire [7:0] mx=(r>g)?((r>b)?r:b):((g>b)?g:b);
 wire [7:0] mn=(r<g)?((r<b)?r:b):((g<b)?g:b);
 wire [7:0] wgap=(mx>>2)>8'd16 ? (mx>>2):8'd16;
 // Broader exposure and colour-cast tolerance for printed / screen signs.
 wire blue_pixel=b>=8'd12 && {1'b0,b}>={1'b0,r}+{1'b0,(r>>1)}+9'd6 && {1'b0,b}>={1'b0,g}+{1'b0,(g>>2)}+9'd6;
 // Reuse the component finder for dark achromatic disk as well as blue.
 // I_level is derived from the preceding frame's white reference.
 wire dark_pixel=g<=I_level && g>=(I_level>>1);
 wire white_pixel=!blue_pixel && (I_color_mode || mn>I_white_level) && mn>=8'd12 && ({1'b0,mx}-{1'b0,mn})<={1'b0,wgap};
 // Very dark blue may be indistinguishable from the backing. In that case
 // localise the white ring/arrow itself instead of requiring a blue disk.
 wire bright_pixel=white_pixel && mn>={I_level[6:0],1'b0};
 assign O_blue[3-p]=in_roi && (I_color_mode ? blue_pixel : I_bright_mode ? bright_pixel : dark_pixel);
 assign O_white[3-p]=in_roi && white_pixel;
end endgenerate
endmodule
