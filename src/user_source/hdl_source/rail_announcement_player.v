// One-shot announcement player. Samples stay in the existing shared SDRAM.
// Commands use a held payload/toggle; old reads drain before both FIFO pointers reset.
module rail_announcement_player #(
 parameter [19:0] C1_START=0,C1_LENGTH=242523,C2_START=243003,C2_LENGTH=176013,
 parameter [19:0] C3_START=419496,C3_LENGTH=197970,C4_START=617946,C4_LENGTH=164441,
 parameter [19:0] C5_START=782867,C5_LENGTH=76493,
 parameter integer END_DRAIN_CYCLES=256
)(
 input wire sclk,mclk,aclk,rst,
 input wire [2:0] event_id,input wire ready_m,emergency_m,fault_m,mem_idle,
 input wire mem_request_issued,input wire sample_consumed,
 output wire reset_mem,reset_audio,allow_sample,
 output wire [19:0] sample_start,fetch_remaining,
 output reg done_pulse
);
reg [2:0] command_id;
reg command_toggle;
reg [2:0] cmd_sync;
reg cmd_seen,active_m;
reg [19:0] start_m,length_m,left_m;
reg [7:0] flush_m;
reg [2:0] done_m_sync,done_s_sync;
reg done_m_seen,done_s_seen;
reg done_a;
function [19:0] clip_start;
 input [2:0] n;
 begin case(n)1:clip_start=C1_START;2:clip_start=C2_START;3:clip_start=C3_START;4:clip_start=C4_START;5:clip_start=C5_START;default:clip_start=0;endcase end
endfunction
function [19:0] clip_length;
 input [2:0] n;
 begin case(n)1:clip_length=C1_LENGTH;2:clip_length=C2_LENGTH;3:clip_length=C3_LENGTH;4:clip_length=C4_LENGTH;5:clip_length=C5_LENGTH;default:clip_length=0;endcase end
endfunction
always @(posedge sclk or posedge rst)begin
 if(rst)begin command_id<=0;command_toggle<=0;done_s_sync<=0;done_s_seen<=0;done_pulse<=0;end
 else begin
  done_s_sync<={done_s_sync[1:0],done_a};done_pulse<=0;
  if(done_s_sync[2]!=done_s_seen)begin done_s_seen<=done_s_sync[2];done_pulse<=1;end
  if(event_id>=1 && event_id<=5)begin command_id<=event_id;command_toggle<=~command_toggle;end
 end
end
assign reset_mem=!ready_m || emergency_m || fault_m || !active_m || flush_m!=0;
assign sample_start=start_m;
assign fetch_remaining=left_m;
always @(posedge mclk or posedge rst)begin
 if(rst)begin cmd_sync<=0;cmd_seen<=0;active_m<=0;start_m<=0;length_m<=0;left_m<=0;flush_m<=0;done_m_sync<=0;done_m_seen<=0;end
 else begin
  cmd_sync<={cmd_sync[1:0],command_toggle};done_m_sync<={done_m_sync[1:0],done_a};
  if(done_m_sync[2]!=done_m_seen)begin done_m_seen<=done_m_sync[2];active_m<=0;end
  if(cmd_sync[2]!=cmd_seen)begin
   cmd_seen<=cmd_sync[2];start_m<=clip_start(command_id);length_m<=clip_length(command_id);
   left_m<=clip_length(command_id);active_m<=1;flush_m<=255;
  end else begin
   // No flush release until previously issued physical reads and their tail end.
   if(flush_m!=0 && mem_idle)flush_m<=flush_m-1'b1;
   if(reset_mem)left_m<=length_m;
   else if(mem_request_issued && left_m!=0)left_m<=left_m-1'b1;
  end
 end
end
reg [2:0] reset_a_sync,active_a_sync;
reg [19:0] played_a;
reg finished_a;
reg [8:0] tail_a;
assign reset_audio=reset_a_sync[2];
assign allow_sample=active_a_sync[2] && !reset_a_sync[2] && !finished_a;
always @(posedge aclk or posedge rst)begin
 if(rst)begin reset_a_sync<=7;active_a_sync<=0;played_a<=0;finished_a<=0;tail_a<=0;done_a<=0;end
 else begin
  reset_a_sync<={reset_a_sync[1:0],reset_mem};active_a_sync<={active_a_sync[1:0],active_m};
  if(reset_a_sync[2])begin played_a<=0;finished_a<=0;tail_a<=0;end
  else if(!finished_a && allow_sample && sample_consumed)begin
   played_a<=played_a+1'b1;
   if(played_a+1'b1>=length_m)begin finished_a<=1;tail_a<=END_DRAIN_CYCLES;end
  end else if(tail_a!=0)begin
   tail_a<=tail_a-1'b1;
   if(tail_a==1)done_a<=~done_a;
  end
 end
end
endmodule
