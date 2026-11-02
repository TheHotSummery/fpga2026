`timescale 1ns/1ps
module tb_abort_integrity;
 reg clk=0,rst=1,ir_sensor=1;
 always #5 clk=~clk;
 wire manager_req,manager_we,writer_ack,writer_finish,writer_we,fifo_aclr,wr_kill;
 wire [31:0] manager_data;
 wire [1:0] writer_page;
 wire [20:0] writer_addr;
 wire write_flip;
 reg finish_toggle=0;
 reg [2:0] kill_sync=0;
 reg [8:0] abort_count=0;
 wire writer_reset=rst || abort_count!=0;
 reg source_valid=0;
 reg [23:0] source_pixel=24'hFF0000;
 integer fifo_count=0,read_ptr=0,write_ptr=0;
 integer emitted_first=0,emitted_retry=0,ack_count=0,checks=0,failures=0;
 reg [1:0] fifo_tags[0:4095];
 reg [1:0] source_tag=1;
 reg ack_d=0;
 reg in_retry=0;
 integer retry_writes=0;
 reg [20:0] first_retry_addr=0;
 sd_card_bmp #(.CLK_FREQ_HZ(100000),.FRAME_PIXELS(512)) d(.clk(clk),.rst(rst),.ir_sensor(ir_sensor),
 .pip_read_req(1'b0),.pip_read_sector(32'd0),.ctrl_next(1'b0),.ctrl_prev(1'b0),.ctrl_auto(1'b0),.ctrl_emerg(1'b0),.ctrl_period(4'd10),
 .key_next(1'b1),.key_enc_sw(1'b1),.key_auto(1'b1),.key_period(1'b1),.emerg_n(1'b1),.vsync(1'b0),.fade_swap_ok(1'b0),
 .bmp_width(16'd640),.bmp_height(16'd480),.write_finish_toggle(finish_toggle),.wav_spi_hold(1'b0),.write_ready(1'b1),
 .flash_cap_rdy(1'b1),.flash_idle(1'b1),.flash_slot_ok(4'd15),.write_req_ack(writer_ack),.SD_MISO(1'b1),
 .write_req(manager_req),.write_en(manager_we),.write_data(manager_data),.write_buf_idx(writer_page),.write_flip(write_flip),.wr_kill_tgl(wr_kill));
 frame_fifo_write #(.FRAME_WIDTH(16),.FRAME_HEIGHT(32)) writer(.rst(writer_reset),.mem_clk(clk),
 .Sdr_init_done(1'b1),.Sdr_init_ref_vld(1'b0),.Sdr_busy(1'b0),.App_rd_busy(1'b0),.O_wr_busy(),
 .App_wr_en(writer_we),.App_wr_addr(writer_addr),.write_flip(write_flip),.write_req(manager_req),
 .write_req_ack(writer_ack),.write_finish(writer_finish),.write_addr_0(21'd0),.write_addr_1(21'd1024),
 .write_addr_2(21'd2048),.write_addr_3(21'd3072),.write_addr_index(writer_page),.write_len(21'd512),
 .fifo_aclr(fifo_aclr),.rdusedw(fifo_count[9:0]));
 // Match the production kill-toggle crossing and 255-memory-cycle abort reset.
 always @(posedge clk or posedge rst)begin
  if(rst)begin kill_sync<=0;abort_count<=0;finish_toggle<=0;end
  else begin
   kill_sync<={kill_sync[1:0],wr_kill};
   if(kill_sync[2]^kill_sync[1])abort_count<=255;
   else if(abort_count!=0)abort_count<=abort_count-1'b1;
   if(writer_finish)finish_toggle<=~finish_toggle;
  end
 end
 // Small FIFO scoreboard supplies actual readiness to the production writer.
 // TF parser and pixel normalizers are injected; write transaction is real.
 always @(posedge clk)begin
  if(writer_reset || fifo_aclr)begin fifo_count=0;read_ptr=0;write_ptr=0;end
  else begin
   if(manager_we)begin fifo_tags[write_ptr]=source_tag;write_ptr=(write_ptr+1)%4096;fifo_count=fifo_count+1;end
   if(writer_we)begin
    if(fifo_count<=0)$fatal(1,"FIFO underflow");
    if(fifo_tags[read_ptr]==1)emitted_first=emitted_first+1;
    if(fifo_tags[read_ptr]==2)emitted_retry=emitted_retry+1;
    read_ptr=(read_ptr+1)%4096;fifo_count=fifo_count-1;
    if(in_retry)begin
     if(retry_writes==0)first_retry_addr=writer_addr;
     retry_writes=retry_writes+1;
    end
   end
  end
  if(!rst && writer_ack && !ack_d)ack_count=ack_count+1;
  ack_d=writer_ack;
 end
 task tick;begin @(posedge clk);#1;@(negedge clk);end endtask
 task cycles(input integer n);integer k;begin for(k=0;k<n;k=k+1)tick;end endtask
 task check(input condition,input [799:0] label);begin
  checks=checks+1;if(condition)$display("PASS %0s",label);
  else begin failures=failures+1;$display("FAIL %0s",label);end
 end endtask
 task wait_launch;
 integer n;begin n=0;while(!d.load_start_pulse && n<10000)begin tick;n=n+1;end
 check(d.load_start_pulse && d.load_idx==4,"train launches intended video request");end endtask
 task wait_ack;
 integer n;begin n=0;while(!writer_ack && n<2000)begin tick;n=n+1;end end endtask
 reg kill_before;
 integer n;
 initial begin
  cycles(6);rst=0;cycles(20);
  force d.sd_init_done=1;force d.bmp_ready=1;force d.scan_done=1;force d.scan_kicked=1;
  force d.img_found_count=8;force d.pip_catalog_valid=1;force d.pip_sector0=500;
  force d.first_image_committed=1;force d.display_valid=1;
  force d.emerg_active=0;force d.emerg_pending=0;force d.emerg_req_pulse=0;
  force d.sd_frozen=0;force d.fill_active=0;force d.wav_load_busy=0;force d.loading_buf_c=0;
  force d.want_c_bmp=0;force d.want_c_scrub=0;force d.heal_after_emerg=0;force d.vs_rise=0;
  force d.wav_phase2_kicked=1;force d.wav_phase2_busy=0;force d.wav_found_count=0;
  force d.scan_found_valid=0;force d.wav_found_valid=0;force d.wav_load_done=0;force d.wav_scan_done_w=0;
  force d.load_fail=0;force d.video_last=0;force d.g_loading=0;
  force d.bmp_write_req=0;force d.bmp_data_wr_en=source_valid;force d.bmp_data=source_pixel;
  d.img_idx_r=0;d.disp_commit_pending=0;d.auto_play_en=0;
  ir_sensor=0;wait_launch;force d.bmp_ready=0;
  force d.bmp_write_req=1;wait_ack;check(writer_ack,"first transaction writer acknowledges");
  force d.bmp_write_req=0;cycles(8);
  source_valid=1;cycles(160);source_valid=0;cycles(180);
  check(writer.state==2 && writer.write_cnt==128 && emitted_first==128,"partial frame owns writer session and has written one burst");
  check(d.load_busy && d.load_source_started,"manager load is truly busy; load_busy was not forced");
  $display("EVIDENCE before_fail writer_state=%0d write_cnt=%0d fifo=%0d kill=%0d",writer.state,writer.write_cnt,fifo_count,wr_kill);
  kill_before=wr_kill;force d.load_fail=1;tick;force d.load_fail=0;force d.bmp_ready=1;
  check(wr_kill!=kill_before,"mid-stream parser failure cancels old SDRAM writer session");
  check(d.next_req_pending && !d.disp_commit_pending && d.img_idx_r==0,"failed hidden frame stays uncommitted and target retries");
  wait_launch;force d.bmp_ready=0;
  force d.bmp_write_req=1;wait_ack;tick; // scoreboard observes the registered ACK on its next clock
  $display("EVIDENCE retry writer_state=%0d write_cnt=%0d fifo=%0d ack_count=%0d abort_count=%0d",writer.state,writer.write_cnt,fifo_count,ack_count,abort_count);
  check(writer_ack && ack_count==2,"retry gets a fresh writer transaction instead of reusing orphan session");
  if(writer_ack)begin
   force d.bmp_write_req=0;cycles(8);in_retry=1;source_tag=2;source_pixel=24'h0000FF;
   source_valid=1;cycles(512);source_valid=0;
   n=0;while(!writer_finish && n<3000)begin tick;n=n+1;end
   check(writer_finish && retry_writes==512 && emitted_retry==512,"fresh retry writes every pixel exactly once");
   check(first_retry_addr==1024,"retry restarts at hidden page base address");
  end else begin
   check(0,"fresh retry writes every pixel exactly once");check(0,"retry restarts at hidden page base address");
  end
  $display("RESULT checks=%0d failures=%0d",checks,failures);$finish;
 end
 initial begin #5000000;$fatal(1,"abort integrity test timeout");end
endmodule
