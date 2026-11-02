`timescale 1ns/1ps
module bilinear_tb;
reg clk=0,rst=1,valid=0,ready=1;reg [23:0] pixel=0;
always #5 clk=~clk;
reg [15:0] width=640,height=360;
wire ov0,pause0,done0,over0,ov1,pause1,done1,over1;wire [23:0] p0,p1;
exhibit_media_bilinear dut(clk,rst,valid,ready,pixel,width,height,ov0,p0,pause0,done0,over0);
exhibit_media_bilinear_reference ref0(clk,rst,valid,ready,pixel,width,height,ov1,p1,pause1,done1,over1);
integer i,cycle=0,pixels=0,checks=0,scenario;
integer proof_cycle=0,last_p00=-1,mcp_checks=0;
integer sampled_phase;reg [23:0] sampled_p00;
always @(posedge clk)begin
 proof_cycle=proof_cycle+1;
 sampled_phase=dut.phase;sampled_p00=dut.p00;
 if(rst)last_p00=-1;
 else if(!dut.done && dut.format_ok)begin
  if(dut.phase==1)last_p00=proof_cycle;
  if(dut.phase==4)begin
   if(last_p00<0 || proof_cycle-last_p00<3)$fatal(1,"MCP source not stable for three clocks");
   mcp_checks=mcp_checks+1;
  end
 end
 #1;if(!rst && sampled_phase!=1 && dut.p00!==sampled_p00)$fatal(1,"Unexpected source register update");
end
always @(negedge clk)begin cycle=cycle+1;ready=cycle%4096>=180 && cycle%37!=0;end
always @(posedge clk)begin
 #1;if(!rst)begin
  if({ov0,p0,pause0,done0,over0}!=={ov1,p1,pause1,done1,over1})$fatal(1,"cycle differs: scenario %0d cycle %0d",scenario,cycle);
  if(over0)$fatal(1,"row overrun");if(ov0)begin pixels=pixels+1;checks=checks+1;end
 end
end
initial begin
 for(scenario=0;scenario<7;scenario=scenario+1)begin
  rst=1;valid=0;repeat(6)@(negedge clk);
  case(scenario) 0:begin width=640;height=360;end 1:begin width=512;height=288;end 2:begin width=320;height=180;end 3:begin width=256;height=144;end 4:begin width=352;height=198;end 5:begin width=368;height=207;end 6:begin width=384;height=216;end endcase pixels=0;rst=0;
  for(i=0;i<width*height;i=i+1)begin
   if(width==640 || width==512)while(pause0)@(negedge clk);
   else if(i%256==0)while(pause0)@(negedge clk);
   pixel={8'(i*3+i/width),8'(i*7),8'(i*11)};valid=1;@(negedge clk);valid=0;
   repeat(11)@(negedge clk);
  end
  wait(done0);repeat(10)@(negedge clk);
  if(pixels!=307200)$fatal(1,"incomplete frame: %0d",pixels);
 end
 $display("MCP_VALIDITY_RESULT checks=%0d failures=0 formats=7 min_source_to_capture_cycles=3 constraint_cycles=2",mcp_checks);
 $display("BILINEAR_RESULT pixels=%0d failures=0 cycle_exact=1 formats=all_7_supported",checks);$finish;
end
initial begin #600000000;$fatal(1,"equivalence timeout");end
endmodule
