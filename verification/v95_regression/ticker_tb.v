`timescale 1ns/1ps
module ticker_tb;
reg clk=0,rst=1,vs=0,de=0,enable=1;reg [2:0] stage=0;
always #5 clk=~clk;
wire [23:0] rgb;
osd_metro_chrome dut(.I_media_ready(1'b1),.I_boot_status(4'd0),.I_temperature_update(1'b0),.I_temperature_valid(1'b0),.I_temperature_alarm(1'b0),.I_temperature_tens(4'd0),.I_temperature_ones(4'd0),.I_media_index(3'd0),.I_media_auto(1'b0),.I_media_available(6'b011111),.I_scroll_speed(6'd60),.I_journey_stage(stage),.I_clk(clk),.I_rst(rst),.I_hs(1'b1),.I_vs(vs),.I_de(de),.I_enable(enable),.I_warn(1'b0),.I_fps_tenths(10'd595),.I_fps_valid(1'b1),.I_rgb(24'h182D40),.O_rgb(rgb));
integer checks=0,failures=0,i,j;reg [9:0] old_phase;
task check(input c,input [767:0] why);begin checks=checks+1;if(!c)begin failures=failures+1;$display("FAIL %0s",why);end end endtask
task ticks(input integer n);integer k;begin for(k=0;k<n;k=k+1)begin @(posedge clk);#1;end end endtask
task frame;begin @(negedge clk);vs=1;ticks(2);@(negedge clk);vs=0;ticks(4);end endtask
function [6:0] first;input integer s;begin case(s)0:first=49;1,3,4:first=54;2:first=33;5:first=36;endcase end endfunction
task probe_center(input integer s);
begin
 for(j=0;j<32;j=j+1)begin
  force dut.pix_x=244+j;force dut.pix_y=338;force dut.I_de=1'b1;#1;
  check(dut.text_enable && dut.glyph_id==first(s) && dut.glyph_col==j,"complete first glyph occupies center columns 244..275");
 end
 force dut.pix_x=4;#1;check(dut.glyph_id==0,"no previous caption tail appears left of the centered start");
 release dut.pix_x;release dut.pix_y;release dut.I_de;
end endtask
initial begin
 ticks(4);rst=0;ticks(4);check(dut.scroll_x==784,"initial caption starts in center");probe_center(0);
 for(i=1;i<=5;i=i+1)begin
  repeat(7)frame;old_phase=dut.scroll_x;
  force dut.pix_y=200;force dut.I_de=1'b1;
  @(negedge clk);stage=i;ticks(5);
  check(dut.journey_stage_frame==i-1 && dut.scroll_x==old_phase,"mid-frame caption change waits for blanking");
  release dut.pix_y;release dut.I_de;frame;
  check(dut.journey_stage_frame==i && dut.scroll_x==784 && dut.scroll_fraction==0,"each new caption restarts at center");probe_center(i);
 end
 repeat(240)frame;check(dut.scroll_x==0,"leftward scrolling crosses phase wrap continuously");
 enable=0;ticks(4);enable=1;ticks(4);check(dut.scroll_x==784,"returning from hidden page restarts centered");
 $display("TICKER_RESULT checks=%0d failures=%0d",checks,failures);if(failures)$fatal(1,"caption regression");$finish;
end
initial begin #1000000;$fatal(1,"timeout");end
endmodule
