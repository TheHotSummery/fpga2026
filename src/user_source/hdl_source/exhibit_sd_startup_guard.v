// Assert immediately; release on exactly the CYCLES-th clock after reset.
// A dedicated release register removes the startup-counter comparator from
// the high-fanout SD reset path. Default delay is 200 ms at 62.5 MHz.
module exhibit_sd_startup_guard #(parameter integer CYCLES=12500000)(
 input wire clk,rst,output wire reset_out);
generate if(CYCLES<=0)begin:g_bypass
 assign reset_out=rst;
end else begin:g_wait
 localparam integer WIDTH=(CYCLES<2)?1:$clog2(CYCLES+1);
 reg [WIDTH-1:0] count;
 reg holding;
 always @(posedge clk or posedge rst)begin
  if(rst)begin count<=0;holding<=1;end
  else if(count!=CYCLES)begin
   count<=count+1'b1;
   if(count==CYCLES-1)holding<=0;
  end
 end
 assign reset_out=rst|holding;
end endgenerate
endmodule
