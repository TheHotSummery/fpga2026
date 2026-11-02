// Force red poster (gallery 1 / 0006.BMP), count only displayed frames,
// then force the A/B map (gallery 0 / 0005.BMP). No timer starts during a load.
module exhibit_fire_sequence #(parameter integer RED_HOLD_FRAMES=180)(
 input wire clk,rst,trigger,frame_tick,emergency_active,page_ready,
 input wire [2:0] gallery_index,
 output wire force_enter,force_page,output wire [2:0] target_index,
 output wire [2:0] phase
);
localparam [2:0] IDLE=0,ENTER=1,WAIT_RED=2,HOLD_RED=3,WAIT_MAP=4,HOLD_MAP=5;
reg [2:0] state;
reg [7:0] frames;
assign force_enter=state==ENTER;
assign force_page=state!=IDLE;
assign target_index=(state==WAIT_MAP || state==HOLD_MAP)?3'd0:3'd1;
assign phase=state;
always @(posedge clk or posedge rst)begin
 if(rst)begin state<=IDLE;frames<=0;end
 else if(state!=IDLE && state!=ENTER && !emergency_active)begin state<=IDLE;frames<=0;end
 else case(state)
  IDLE:if(trigger)begin state<=ENTER;frames<=0;end
  ENTER:if(emergency_active)state<=WAIT_RED;
  WAIT_RED:if(page_ready && gallery_index==1)begin state<=HOLD_RED;frames<=0;end
  HOLD_RED:if(page_ready && gallery_index==1 && frame_tick)begin
   if(frames>=RED_HOLD_FRAMES-1)begin state<=WAIT_MAP;frames<=0;end
   else frames<=frames+1'b1;
  end
  WAIT_MAP:if(page_ready && gallery_index==0)state<=HOLD_MAP;
  HOLD_MAP:begin end
  default:state<=IDLE;
 endcase
end
endmodule
