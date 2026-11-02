`timescale 1ns/1ps
module status_tb;
reg clk=0,rst=1,vs=0,de=0,valid=0,alarm=0,enable=1,ready=0,update=0;
reg [3:0] tens=2,ones=5,boot=0;
always #5 clk=~clk;
wire [23:0] rgb,old_rgb;
osd_metro_chrome dut(.I_boot_status(boot),.I_media_ready(ready),.I_temperature_update(update),
 .I_temperature_valid(valid),.I_temperature_alarm(alarm),.I_temperature_tens(tens),.I_temperature_ones(ones),
 .I_media_index(3'd0),.I_media_auto(1'b0),.I_media_available(6'b011111),.I_scroll_speed(6'd60),.I_journey_stage(3'd0),
 .I_clk(clk),.I_rst(rst),.I_hs(1'b1),.I_vs(vs),.I_de(de),.I_enable(enable),.I_warn(1'b0),
 .I_fps_tenths(10'd595),.I_fps_valid(1'b1),.I_rgb(24'h182D40),.O_rgb(rgb));
osd_metro_chrome_before old(.I_temperature_valid(valid),.I_temperature_alarm(alarm),.I_temperature_tens(tens),.I_temperature_ones(ones),
 .I_media_index(3'd0),.I_media_auto(1'b0),.I_media_available(6'b011111),.I_scroll_speed(6'd60),.I_journey_stage(3'd0),
 .I_clk(clk),.I_rst(rst),.I_hs(1'b1),.I_vs(vs),.I_de(de),.I_enable(enable),.I_warn(1'b0),
 .I_fps_tenths(10'd595),.I_fps_valid(1'b1),.I_rgb(24'h182D40),.O_rgb(old_rgb));
integer checks=0,failures=0,x,y,i,differences=0;
task check(input c,input [767:0] why);begin checks=checks+1;if(c!==1'b1)begin failures=failures+1;$display("FAIL %0s",why);end end endtask
task ticks(input integer n);begin repeat(n)begin @(posedge clk);#1;end end endtask
task frame;begin @(negedge clk);vs=1;ticks(2);@(negedge clk);vs=0;ticks(4);end endtask
task point(input integer px,input integer py);begin
 force dut.pix_x=px;force dut.pix_y=py;force dut.I_de=1'b1;
 force old.pix_x=px;force old.pix_y=py;force old.I_de=1'b1;#1;
end endtask
task free_point;begin release dut.pix_x;release dut.pix_y;release dut.I_de;release old.pix_x;release old.pix_y;release old.I_de;end endtask
initial begin
 ticks(4);rst=0;ticks(4);frame;
 point(8,176);check(dut.base_rgb==24'hE3DDF6,"init box visible before first frame");ticks(4);check(rgb==24'hE3DDF6,"init box reaches registered RGB output");
 point(7,176);check(dut.base_rgb==24'h182D40,"diagnostic left boundary");
 point(104,176);check(dut.base_rgb==24'h182D40,"diagnostic right boundary");
 point(8,175);check(dut.base_rgb==24'h182D40,"diagnostic top boundary");
 point(8,202);check(dut.base_rgb==24'h182D40,"diagnostic bottom boundary");
 for(i=0;i<4;i=i+1)begin point(20+i*12,182);case(i)0,2:check(dut.tiny_char=="I","INIT I glyph");1:check(dut.tiny_char=="N","INIT N glyph");3:check(dut.tiny_char=="T","INIT T glyph");endcase end
 point(68,182);check(dut.tiny_char==" ","diagnostic separating space");
 for(i=0;i<8;i=i+1)begin boot=i;point(80,182);check(dut.tiny_enable && dut.tiny_char==48+i && dut.tiny_bits!=0,"all boot phase digits 0..7 exist");end
 ready=1;point(8,176);check(dut.base_rgb==24'h182D40 && !dut.tiny_enable,"diagnostics disappear once media is ready");
 ready=0;enable=0;point(8,176);check(dut.base_rgb==24'h182D40 && !dut.tiny_enable,"diagnostics do not overlay emergency page");
 enable=1;ready=1;point(344,450);check(dut.base_rgb==24'h71818C,"unavailable heartbeat gray");ticks(4);check(rgb==24'h71818C,"unavailable heartbeat at final RGB");free_point;
 valid=1;update=0;frame;point(344,450);check(dut.base_rgb==24'h91AAA4,"valid unchanged phase heartbeat dim");free_point;
 update=1;ticks(4);point(344,450);check(dut.base_rgb==24'h91AAA4,"mid-frame sample heartbeat does not tear");free_point;frame;
 point(344,450);check(dut.base_rgb==24'h16836F,"sample phase toggles heartbeat green at next frame");ticks(4);check(rgb==24'h16836F,"green heartbeat reaches registered RGB");
 point(343,450);check(dut.base_rgb==old.base_rgb,"heartbeat left gutter clear");
 point(348,450);check(dut.base_rgb==old.base_rgb,"heartbeat right gutter clear");
 point(344,449);check(dut.base_rgb==old.base_rgb,"heartbeat top gutter clear");
 point(344,454);check(dut.base_rgb==old.base_rgb,"heartbeat bottom gutter clear");
 point(8,436);check(dut.glyph_id==41 && dut.glyph_row==0,"station label unchanged");
 point(352,436);check(dut.glyph_id==2,"line label unchanged");
 point(472,436);check(dut.glyph_id==2,"clock label unchanged");
 point(240,444);check(dut.glyph_id==20 && dut.glyph_row==0,"temperature label unchanged");
 // Compare every footer coordinate against actual V93 renderer except the new dot.
 differences=0;
 for(y=424;y<480;y=y+1)for(x=0;x<640;x=x+1)begin
  point(x,y);
  if(!(x>=344 && x<348 && y>=450 && y<454))
   if({dut.base_rgb,dut.text_rgb,dut.glyph_id,dut.glyph_col,dut.glyph_row,dut.text_enable,dut.tiny_enable} !==
      {old.base_rgb,old.text_rgb,old.glyph_id,old.glyph_col,old.glyph_row,old.text_enable,old.tiny_enable})differences=differences+1;
 end
 check(differences==0,"all 35824 non-dot footer pixels and font coordinates match V93");free_point;
 valid=0;frame;point(344,450);check(dut.base_rgb==24'h71818C,"lost sensor returns heartbeat to gray");
 enable=0;point(344,450);check(dut.base_rgb==24'h182D40,"heartbeat does not overlay emergency poster");free_point;
 $display("STATUS_RESULT checks=%0d failures=%0d",checks,failures);if(failures)$fatal(1,"status renderer regression");$finish;
end
initial begin #5000000;$fatal(1,"timeout");end
endmodule
