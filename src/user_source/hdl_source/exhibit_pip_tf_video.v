// PVF4 version 6: native 256x144 RGB565, six HDMI frames per video frame.
// Header=512 bytes. Frame stride=73728 bytes; 73728 RGB565 bytes, no padding.
// Header width low byte 5 = 0, width high byte 13 = 1; height byte 6 = 144.
// SDRAM word map: 0x1e0000 / 0x1f0000, two RGB565 pixels per 32-bit word.
// Photo A/B/C and PCM end below 0x1da600; no addresses overlap.
module exhibit_pip_tf_video #(
 parameter [9:0] X=10'd256,Y=10'd132,
 parameter [9:0] W=10'd264,H=10'd158
)(input wire clk,sd_clk,mclk,rst,card_reset,vs,de,enable,input wire [6:0] status,
 input wire [92:0] cfg,input wire [23:0] irgb,output wire [23:0] rgb,output wire [9:0] panel_left,panel_top,
 input wire [1:0] catalog_valid,input wire [31:0] sector0,sector1,size0,size1,
 input wire allowed,output wire rd_req,output reg [31:0] rd_sector,
 input wire rd_ack,rd_valid,rd_end,input wire [7:0] rd_byte,
 input wire main_pending,input wire mem_free,input wire mem_busy,input wire mem_data_valid,input wire [31:0] mem_data,
 output wire mem_hold,mem_owner,output wire mem_rd,mem_wr,
 output wire [20:0] mem_addr,output wire [31:0] mem_wdata,
 output reg [15:0] line_miss);
wire [3:0] control_mode;wire signed [7:0] brightness;
wire [7:0] contrast,saturation,zoom;wire signed [4:0] tone;
wire signed [9:0] pan_x,pan_y;wire fit,blend,compare;
wire [6:0] opacity;wire [5:0] speed;wire [1:0] position,transition,direction,tuning;
wire [3:0] duration,period;
assign {control_mode,brightness,contrast,tone,saturation,zoom,pan_x,pan_y,fit,opacity,blend,speed,position,transition,duration,period,direction,compare,tuning}=cfg;
// Fixed top/right anchor. Native 256x144 media is resampled into a
// 192x108 base viewport, 96x54 at 50%, 384x216 at 200%.
// Zoom code 255 is MAX: cover the original photo rectangle [0,520)x[132,424).
// Geometry and inverse step are prepared off the pixel path (<=685 clocks).
reg [9:0] zoom_step,zoom_quotient,zoom_divisor;
reg [7:0] seen_zoom;
reg [16:0] zoom_remainder;
reg [15:0] width_remainder,height_remainder;
reg [9:0] width_quotient,height_quotient,body_width,body_height;
reg zoom_busy;
wire [7:0] bounded_zoom=zoom<50?8'd50:zoom;
wire [9:0] divisor_next={2'd0,bounded_zoom}+{1'd0,bounded_zoom,1'b0};
wire [15:0] zoom_wide={8'd0,bounded_zoom};
always @(posedge clk or posedge rst)begin
 if(rst)begin
  zoom_step<=341;seen_zoom<=100;zoom_divisor<=300;zoom_quotient<=0;zoom_remainder<=0;zoom_busy<=0;
  width_remainder<=0;height_remainder<=0;width_quotient<=0;height_quotient<=0;body_width<=192;body_height<=108;
 end else if(!zoom_busy)begin
  if(zoom!=seen_zoom)begin
   seen_zoom<=zoom;zoom_divisor<=divisor_next;
   zoom_remainder<=17'd102400+{8'd0,divisor_next[9:1]};zoom_quotient<=0;
   width_remainder<=(zoom_wide<<7)+(zoom_wide<<6)+16'd50;
   height_remainder<=(zoom_wide<<6)+(zoom_wide<<5)+(zoom_wide<<3)+(zoom_wide<<2)+16'd50;
   width_quotient<=0;height_quotient<=0;zoom_busy<=1;
  end
 end else begin
  if(zoom_remainder>={7'd0,zoom_divisor})begin zoom_remainder<=zoom_remainder-{7'd0,zoom_divisor};zoom_quotient<=zoom_quotient+1'b1;end
  if(width_remainder>=100)begin width_remainder<=width_remainder-16'd100;width_quotient<=width_quotient+1'b1;end
  if(height_remainder>=100)begin height_remainder<=height_remainder-16'd100;height_quotient<=height_quotient+1'b1;end
  if(zoom_remainder<{7'd0,zoom_divisor} && width_remainder<100 && height_remainder<100)begin
   zoom_step<=zoom_quotient;body_width<=width_quotient;body_height<=height_quotient;zoom_busy<=0;
  end
 end
end
wire full_area=!compare && zoom==8'd255;
wire [9:0] view_width=full_area?10'd520:compare?10'd192:body_width;
wire [9:0] view_height=full_area?10'd292:compare?10'd108:body_height;
wire [9:0] window_right=full_area?10'd520:X+W;
wire [9:0] window_top=full_area?10'd132:Y;
wire [9:0] body_right=full_area?window_right:window_right-10'd4;
wire [9:0] window_left=full_area?10'd0:body_right-view_width-10'd4;
wire [9:0] body_left=full_area?10'd0:window_left+10'd4;
wire [9:0] window_bottom=full_area?10'd424:Y+10'd14+view_height;
// A large video uses the right information column: x=524..635, y=132..171.
// Ordinary windows keep the compact panel on their left, separated by 8 pixels.
assign panel_left=window_left>=120?window_left-10'd120:10'd524;
assign panel_top=window_top;
wire [9:0] scale_step=full_area?10'd126:compare?10'd341:zoom_step;
wire signed [20:0] px_offset=compare?21'sd0:($signed({{11{pan_x[9]}},pan_x})<<<8);
wire signed [20:0] py_offset=compare?21'sd0:($signed({{11{pan_y[9]}},pan_y})<<<8);
wire signed [20:0] step_wide=$signed({11'd0,scale_step});
// Pixel-center nearest-neighbor mapping, with optional source-space pan.
wire signed [20:0] origin_x=(step_wide>>>1)+px_offset;
wire signed [20:0] origin_y=(step_wide>>>1)+py_offset;
reg signed [20:0] x_phase,prefetch_phase;
wire signed [12:0] source_x=x_phase>>>8;
wire signed [12:0] prefetch_source_y=prefetch_phase>>>8;
wire outside_x=source_x<0 || source_x>255;
reg [23:0] raw_rgb;reg color_enable;
exhibit_video_color u_video_color(.clk(clk),.rst(rst),.enable(color_enable),.cfg(cfg),.irgb(raw_rgb),.rgb(rgb));
reg request_toggle,request_clip,request_bank,outstanding;
reg done_toggle,done_ok,done_last,done_slow;
reg [1:0] done_format;
reg [2:0] request_sync,done_sync;
reg vd,dd,seen_step,clip,focus,auto_en;
reg display_bank,display_clip,have_frame,display_last,display_slow;
reg [1:0] display_format;
reg [9:0] px,py;
reg [2:0] age;
wire [2:0] frame_age_limit=3'd5;
reg error_visible;
reg [5:0] cooldown;
wire frame=vd && !vs;
wire manual_switch=status[5]!=seen_step;
wire auto_switch=enable && status[4] && auto_en && have_frame && display_clip==clip && display_last && age==frame_age_limit;
wire switch_clip=manual_switch || auto_switch;
wire desired_clip=clip ^ switch_clip;
wire ready_frame=outstanding && done_sync[2]==request_toggle;
wire hold_next=enable && status[4] && have_frame && display_clip==clip && display_last &&
 !switch_clip && request_clip!=clip && age!=frame_age_limit;
always @(posedge clk or posedge rst)begin
 if(rst)begin
  done_sync<=0;vd<=1;dd<=0;px<=0;py<=0;seen_step<=0;clip<=0;focus<=0;auto_en<=0;
  display_bank<=0;display_clip<=0;have_frame<=0;display_last<=0;display_slow<=0;display_format<=0;
  request_toggle<=0;request_clip<=0;request_bank<=1;outstanding<=0;
  age<=0;cooldown<=0;error_visible<=0;
 end else begin
  done_sync<={done_sync[1:0],done_toggle};vd<=vs;dd<=de;
  if(frame)py<=0;else if(dd && !de)py<=py+1'b1;
  if(!de)px<=0;else px<=px+1'b1;
  if(frame)begin
   focus<=status[6];auto_en<=status[4];
   if(enable)begin
    if(age!=frame_age_limit)age<=age+1'b1;
    if(cooldown!=0)cooldown<=cooldown-1'b1;
   end
   if(switch_clip)begin seen_step<=status[5];clip<=desired_clip;cooldown<=0;end
   if(enable && ready_frame)begin
    if(!done_ok)begin outstanding<=0;error_visible<=1;cooldown<=60;end
    else if(request_clip==desired_clip)begin
     if(!have_frame || display_clip!=desired_clip || age==frame_age_limit)begin
      // Publish a complete bank only at VS. Prefetch into the previous front bank.
      display_bank<=request_bank;display_clip<=request_clip;
      display_last<=done_last;display_slow<=done_slow;display_format<=done_format;have_frame<=1;
      age<=0;error_visible<=0;
      request_clip<=(status[4] && done_last)?~desired_clip:desired_clip;
      request_bank<=~request_bank;request_toggle<=~request_toggle;outstanding<=1;
     end
    end else if(!hold_next)begin
     // A manual command superseded this prefetch. Keep the front intact.
     request_clip<=desired_clip;request_bank<=~display_bank;
     request_toggle<=~request_toggle;outstanding<=1;
    end
   end else if(enable && !outstanding && (cooldown==0 || switch_clip))begin
    request_clip<=desired_clip;request_bank<=~display_bank;
    request_toggle<=~request_toggle;outstanding<=1;
   end
  end
 end
end



localparam [3:0] IDLE=0,HREQ=1,HREAD=2,HCHECK=3,DREQ=4,DREAD=5,COMMIT=6,COMPLETE=7;
reg [3:0] state;
reg guard,load_clip,load_bank,cache_valid,last_clip;
reg [15:0] frame_count,next_index,load_index;
reg [31:0] base,size_latched;
reg [9:0] byte_count;
reg [7:0] sector_index;
reg [23:0] sector_low;
reg header_good,transfer_good,cancelled;
reg write_toggle;
reg [20:0] write_base;
reg [2:0] write_ack_s;
reg write_ack;
wire [31:0] expected_size=32'd512+{frame_count,16'd0}+{3'd0,frame_count,13'd0};
assign rd_req=allowed && (state==HREQ || state==DREQ);
wire sector_we=state==DREAD && rd_valid && byte_count<512 && byte_count[1:0]==2'd3;
wire [31:0] sector_q;
wire [6:0] sector_ra;
EG_LOGIC_BRAM #(.DATA_WIDTH_A(32),.DATA_WIDTH_B(32),.ADDR_WIDTH_A(7),.ADDR_WIDTH_B(7),
 .DATA_DEPTH_A(128),.DATA_DEPTH_B(128),.MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
 .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.RESETMODE("SYNC"),.IMPLEMENT("9K"),.INIT_FILE("NONE"),.FILL_ALL("NONE")) sector_cache(
 .dia({rd_byte,sector_low}),.dib(32'd0),.addra(byte_count[8:2]),.addrb(sector_ra),
 .cea(1'b1),.ceb(1'b1),.ocea(1'b0),.oceb(1'b0),.clka(sd_clk),.clkb(mclk),
 .wea(sector_we),.web(1'b0),.bea(1'b0),.beb(1'b0),.rsta(1'b0),.rstb(1'b0),.doa(),.dob(sector_q));
always @(posedge sd_clk or posedge rst)begin
 if(rst)begin
  request_sync<=0;done_toggle<=0;done_ok<=0;done_last<=0;done_slow<=1;done_format<=0;
  state<=IDLE;guard<=0;load_clip<=0;load_bank<=1;cache_valid<=0;last_clip<=0;
  frame_count<=0;next_index<=0;load_index<=0;base<=0;size_latched<=0;rd_sector<=0;
  byte_count<=0;sector_index<=0;sector_low<=0;header_good<=0;transfer_good<=0;
  write_toggle<=0;write_base<=0;write_ack_s<=0;cancelled<=0;
 end else begin
  request_sync<={request_sync[1:0],request_toggle};
  write_ack_s<={write_ack_s[1:0],write_ack};
  if(card_reset)cancelled<=1;
  // Bus reset invalidates the SD transaction only. Never cancel a physical
  // memory burst or reuse its sector buffer until the memory side acknowledges.
  if(card_reset && state!=COMMIT)begin
   cache_valid<=0;transfer_good<=0;state<=COMPLETE;
  end else case(state)
   IDLE:begin
    if(request_sync[2]!=done_toggle && allowed)begin
     if(!guard)guard<=1;
     else begin
      guard<=0;load_clip<=request_clip;load_bank<=request_bank;
      base<=request_clip?sector1:sector0;size_latched<=request_clip?size1:size0;
      transfer_good<=1;cancelled<=0;done_ok<=0;sector_index<=0;
      if(!catalog_valid[request_clip])begin transfer_good<=0;state<=COMPLETE;end
      else if(!cache_valid || request_clip!=last_clip)begin
       rd_sector<=request_clip?sector1:sector0;state<=HREQ;
      end else begin
       load_index<=next_index;rd_sector<=(request_clip?sector1:sector0)+32'd1+({9'd0,next_index,7'd0}+{12'd0,next_index,4'd0});state<=DREQ;
      end
     end
    end else guard<=0;
   end
   HREQ:if(rd_ack)begin state<=HREAD;byte_count<=0;header_good<=1;frame_count<=0;end
   HREAD:begin
    if(rd_valid)begin
     byte_count<=byte_count+1'b1;
     case(byte_count)
      0:if(rd_byte!="P")header_good<=0;
      1:if(rd_byte!="V")header_good<=0;
      2:if(rd_byte!="F")header_good<=0;
      3:if(rd_byte!="4")header_good<=0;
      4:if(rd_byte!=6)header_good<=0;
      5:if(rd_byte!=0)header_good<=0;
      6:if(rd_byte!=144)header_good<=0;
      7:if(rd_byte!=6)header_good<=0;
      8:frame_count[7:0]<=rd_byte;
      9:frame_count[15:8]<=rd_byte;
      10:if(rd_byte!=144)header_good<=0;
      11:if(rd_byte!=16)header_good<=0;
      12:if(rd_byte!={7'd0,load_clip})header_good<=0;
      13:if(rd_byte!=1)header_good<=0;
      default:;
     endcase
    end
    if(rd_end)begin state<=HCHECK;if(byte_count!=512)header_good<=0;end
   end
   HCHECK:begin
    if(header_good && frame_count!=0 && frame_count<=16'd4096 && size_latched==expected_size)begin
     last_clip<=load_clip;cache_valid<=1;load_index<=0;next_index<=0;
     rd_sector<=base+1;state<=DREQ;
    end else begin cache_valid<=0;transfer_good<=0;state<=COMPLETE;end
   end
   DREQ:if(rd_ack)begin state<=DREAD;byte_count<=0;end
   DREAD:begin
    if(rd_valid)begin
     byte_count<=byte_count+1'b1;
     case(byte_count[1:0])
      0:sector_low[7:0]<=rd_byte;
      1:sector_low[15:8]<=rd_byte;
      2:sector_low[23:16]<=rd_byte;
      default:;
     endcase
    end
    if(rd_end)begin
     if(byte_count!=512)begin transfer_good<=0;state<=COMPLETE;end
     else begin
      write_base<=21'h1e0000+(load_bank?21'd65536:21'd0)+{6'd0,sector_index,7'd0};
      write_toggle<=~write_toggle;state<=COMMIT;
     end
    end
   end
   COMMIT:if(write_ack_s[2]==write_toggle)begin
    if(card_reset || cancelled)begin cache_valid<=0;transfer_good<=0;state<=COMPLETE;end
    else if(sector_index==143)state<=COMPLETE;
    else begin sector_index<=sector_index+1'b1;rd_sector<=rd_sector+1'b1;state<=DREQ;end
   end
   COMPLETE:begin
    done_ok<=transfer_good && !cancelled;done_last<=transfer_good && !cancelled && load_index+1'b1>=frame_count;
    done_toggle<=request_sync[2];
    if(transfer_good && !cancelled)next_index<=load_index+1'b1>=frame_count?16'd0:load_index+1'b1;
    else cache_valid<=0;
    state<=IDLE;
   end
   default:state<=IDLE;
  endcase
 end
end

// Four independent scanline slots; prefetch four lines ahead. Descriptor
// fields remain held until acknowledged across clocks. Only complete rows
// become visible, and readiness is frozen for the entire displayed line.
wire [9:0] VIDEO_Y=full_area?10'd132:Y+10'd13;
reg line_toggle,line_ack;
reg [2:0] line_ack_s,line_done_s;
reg [9:0] line_y,done_y;
reg line_front,line_source_valid;reg [7:0] line_source_y;
reg [1:0] line_slot,done_slot,done_physical;
reg [1:0] physical0,physical1,physical2,physical3;
reg done_line_toggle,done_line_ok,done_line_blank;
reg line_done_seen;
reg [3:0] line_ready,line_blank;
reg [9:0] tag0,tag1,tag2,tag3;
reg line_ok,display_blank;
wire [9:0] target_y=py+10'd4;
wire cached_ok=line_ready[py[1:0]] && (py[1:0]==0?tag0:py[1:0]==1?tag1:py[1:0]==2?tag2:tag3)==py;
wire line_start=de && !dd;
wire row_valid=line_start?cached_ok:line_ok;
wire blank_row=line_start?line_blank[py[1:0]]:display_blank;
wire video_row=py>=VIDEO_Y && py<VIDEO_Y+view_height;
always @(posedge clk or posedge rst)begin
 if(rst)begin
  line_toggle<=0;line_y<=0;line_front<=0;line_slot<=0;line_ack_s<=0;line_done_s<=0;line_done_seen<=0;
  physical0<=0;physical1<=0;physical2<=0;physical3<=0;
  line_ready<=0;line_blank<=0;display_blank<=0;line_source_valid<=0;line_source_y<=0;x_phase<=0;prefetch_phase<=0;tag0<=0;tag1<=0;tag2<=0;tag3<=0;line_ok<=0;line_miss<=0;
 end else begin
  line_ack_s<={line_ack_s[1:0],line_ack};line_done_s<={line_done_s[1:0],done_line_toggle};
  if(line_done_s[2]!=line_done_seen)begin
   line_done_seen<=line_done_s[2];line_ready[done_slot]<=done_line_ok;line_blank[done_slot]<=done_line_blank;
   case(done_slot)
    0:begin tag0<=done_y;physical0<=done_physical;end
    1:begin tag1<=done_y;physical1<=done_physical;end
    2:begin tag2<=done_y;physical2<=done_physical;end
    3:begin tag3<=done_y;physical3<=done_physical;end
   endcase
  end
  if(!de)x_phase<=origin_x;
  else if(px>=body_left && px<body_right)x_phase<=x_phase+step_wide;
  if(frame || py<VIDEO_Y-4)prefetch_phase<=origin_y;
  else if(dd && !de && target_y>=VIDEO_Y && target_y<VIDEO_Y+view_height)prefetch_phase<=prefetch_phase+step_wide;
  if(frame)line_ready<=0;
  if(line_start)begin
   line_ok<=cached_ok;display_blank<=line_blank[py[1:0]];
   if(enable && have_frame && video_row && !cached_ok && line_miss!=65535)line_miss<=line_miss+1'b1;
  end
  if(enable && have_frame && dd && !de && target_y>=VIDEO_Y && target_y<VIDEO_Y+view_height && line_ack_s[2]==line_toggle)begin
   line_y<=target_y;line_front<=display_bank;line_slot<=target_y[1:0];line_toggle<=~line_toggle;
   line_source_valid<=prefetch_source_y>=0 && prefetch_source_y<144;
   line_source_y<=prefetch_source_y[7:0];
  end
 end
end

// SDRAM arbitration: advertise pending work before acquisition so other bus
// clients stop starting new sessions; allow their issued beats to drain first.
// mem_owner is asserted only after that drain, never for a waiting descriptor.
localparam [2:0] M_IDLE=0,M_READ=1,M_DRAIN=2,M_PRIME=3,M_WRITE=4,M_WDRAIN=5;
reg [2:0] ms,wr_s,lr_s;
reg [2:0] completion_ack_s;
reg [8:0] issued,received;
reg [20:0] burst_base;
reg [1:0] slot_m,destination_m;
reg [7:0] source_m;reg front_m;
// Four physical source rows, referenced by four destination-line descriptors.
// A destination slot is retired only after its active line has finished.
// Allocation excludes the other three live references, so repeated source
// rows can be shared safely at every zoom ratio, including downscaling.
reg [3:0] row_cache_valid,cache_bank,map_valid;
reg [7:0] cache_y0,cache_y1,cache_y2,cache_y3;
reg [1:0] map0,map1,map2,map3;
wire [3:0] cache_match={
 row_cache_valid[3] && cache_bank[3]==line_front && cache_y3==line_source_y,
 row_cache_valid[2] && cache_bank[2]==line_front && cache_y2==line_source_y,
 row_cache_valid[1] && cache_bank[1]==line_front && cache_y1==line_source_y,
 row_cache_valid[0] && cache_bank[0]==line_front && cache_y0==line_source_y};
wire cache_hit=|cache_match;
wire [1:0] hit_physical=cache_match[0]?2'd0:cache_match[1]?2'd1:cache_match[2]?2'd2:2'd3;
wire [3:0] pinned=
 ((map_valid[0] && line_slot!=0)?(4'b0001<<map0):4'd0) |
 ((map_valid[1] && line_slot!=1)?(4'b0001<<map1):4'd0) |
 ((map_valid[2] && line_slot!=2)?(4'b0001<<map2):4'd0) |
 ((map_valid[3] && line_slot!=3)?(4'b0001<<map3):4'd0);
wire [1:0] free_physical=!pinned[0]?2'd0:!pinned[1]?2'd1:!pinned[2]?2'd2:2'd3;
task map_destination;
 input [1:0] destination,physical;
 begin
  map_valid[destination]<=1;
  case(destination)0:map0<=physical;1:map1<=physical;2:map2<=physical;3:map3<=physical;endcase
 end
endtask
reg [9:0] y_m;
reg [5:0] quiet;
reg [11:0] watchdog;
reg failed;
wire write_pending=wr_s[2]!=write_ack;
wire line_pending=lr_s[2]!=line_ack;
// Cache hits may complete on adjacent memory clocks. Hold completion payload
// until the pixel domain has captured it; otherwise two toggles can be lost.
wire line_work=line_pending && completion_ack_s[2]==done_line_toggle;
// Complete the current PIP burst, then yield to any waiting photo row.
// Chaining a line fill and a sector write ahead of a queued crossfade row
// consumed its prefetch lead. New PIP work uses gaps between photo sessions.
assign mem_hold=ms!=M_IDLE || ((write_pending || line_work) && !main_pending);
assign mem_owner=ms!=M_IDLE;
assign mem_rd=ms==M_READ && issued<128 && issued-received<32;
assign mem_wr=ms==M_WRITE;
assign mem_addr=burst_base+{12'd0,issued};
assign mem_wdata=sector_q;
assign sector_ra=ms==M_WRITE?issued[6:0]+7'd1:7'd0;
wire [8:0] line_write_addr={slot_m,received[6:0]};
wire [9:0] dx=px-window_left,dy=py-Y;
wire [9:0] lx=dx,ly=dy;
wire [9:0] inner_x=lx-10'd4;
wire [1:0] pixel_physical=py[1:0]==0?physical0:py[1:0]==1?physical1:py[1:0]==2?physical2:physical3;
wire [8:0] line_read_addr={pixel_physical,source_x[7:1]};
wire [31:0] line_q;
reg pixel_odd_q;
always @(posedge clk)pixel_odd_q<=source_x[0];
wire [15:0] color565=pixel_odd_q?line_q[31:16]:line_q[15:0];
EG_LOGIC_BRAM #(.DATA_WIDTH_A(32),.DATA_WIDTH_B(32),.ADDR_WIDTH_A(9),.ADDR_WIDTH_B(9),
 .DATA_DEPTH_A(512),.DATA_DEPTH_B(512),.MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
 .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.RESETMODE("SYNC"),.IMPLEMENT("9K"),.INIT_FILE("NONE"),.FILL_ALL("NONE")) line_cache(
 .dia(mem_data),.dib(32'd0),.addra(line_write_addr),.addrb(line_read_addr),
 .cea(1'b1),.ceb(1'b1),.ocea(1'b0),.oceb(1'b0),.clka(mclk),.clkb(clk),
 .wea(mem_data_valid && ms==M_READ && received<128),.web(1'b0),
 .bea(1'b0),.beb(1'b0),.rsta(1'b0),.rstb(1'b0),.doa(),.dob(line_q));
always @(posedge mclk or posedge rst)begin
 if(rst)begin
  ms<=M_IDLE;wr_s<=0;lr_s<=0;completion_ack_s<=0;write_ack<=0;line_ack<=0;issued<=0;received<=0;
  burst_base<=0;slot_m<=0;destination_m<=0;source_m<=0;front_m<=0;y_m<=0;
  row_cache_valid<=0;cache_bank<=0;cache_y0<=0;cache_y1<=0;cache_y2<=0;cache_y3<=0;
  map_valid<=0;map0<=0;map1<=0;map2<=0;map3<=0;done_physical<=0;quiet<=0;watchdog<=0;failed<=0;
  done_line_toggle<=0;done_line_ok<=0;done_line_blank<=0;done_y<=0;done_slot<=0;
 end else begin
  wr_s<={wr_s[1:0],write_toggle};lr_s<={lr_s[1:0],line_toggle};
  completion_ack_s<={completion_ack_s[1:0],line_done_seen};
  case(ms)
   M_IDLE:begin
    if(!mem_free)quiet<=0;else if(quiet!=63)quiet<=quiet+1'b1;
    if(line_work && !line_source_valid)begin
     line_ack<=lr_s[2];done_y<=line_y;done_slot<=line_slot;done_physical<=0;done_line_ok<=1;done_line_blank<=1;
     map_valid[line_slot]<=0;
     done_line_toggle<=~done_line_toggle;
    end else if(line_work && cache_hit)begin
     // Source row already resident: publish a reference without a bus session.
     line_ack<=lr_s[2];done_y<=line_y;done_slot<=line_slot;done_physical<=hit_physical;
     done_line_ok<=1;done_line_blank<=0;done_line_toggle<=~done_line_toggle;
     map_destination(line_slot,hit_physical);
    end else if(mem_free && !main_pending && quiet>=8 && (write_pending || line_work))begin
     issued<=0;received<=0;quiet<=0;watchdog<=0;failed<=0;
     if(line_work)begin
      line_ack<=lr_s[2];slot_m<=free_physical;destination_m<=line_slot;y_m<=line_y;
      source_m<=line_source_y;front_m<=line_front;
      row_cache_valid[free_physical]<=0;map_destination(line_slot,free_physical);
      burst_base<=21'h1e0000+(line_front?21'd65536:21'd0)+
       {6'd0,line_source_y,7'd0};
      ms<=M_READ;
     end else begin burst_base<=write_base;ms<=M_PRIME;end
    end
   end
   M_PRIME:begin
    // A back-bank write invalidates only cache rows from that bank. Front-bank
    // rows stay reusable while the next video frame is loaded in the background.
    row_cache_valid<=row_cache_valid & (cache_bank ^ {4{write_base[16]}});
    ms<=M_WRITE;
   end
   M_WRITE:begin
    if(issued==127)begin ms<=M_WDRAIN;quiet<=0;end
    else issued<=issued+1'b1;
   end
   M_WDRAIN:begin
    if(mem_busy || mem_data_valid)quiet<=0;
    else if(quiet==31)begin write_ack<=wr_s[2];ms<=M_IDLE;quiet<=0;end
    else quiet<=quiet+1'b1;
   end
   M_READ:begin
    if(mem_rd)issued<=issued+1'b1;
    if(mem_rd || mem_data_valid)watchdog<=0;
    else if(watchdog==2047)begin failed<=1;ms<=M_DRAIN;quiet<=0;end
    else watchdog<=watchdog+1'b1;
    if(mem_data_valid && received<128)begin
     if(received==127)begin ms<=M_DRAIN;quiet<=0;end
     received<=received+1'b1;
    end
   end
   M_DRAIN:begin
    if(mem_busy || mem_data_valid)quiet<=0;
    else if(quiet==15)begin
     done_y<=y_m;done_slot<=destination_m;done_physical<=slot_m;done_line_ok<=!failed;done_line_blank<=0;
     row_cache_valid[slot_m]<=!failed;cache_bank[slot_m]<=front_m;
     case(slot_m)0:cache_y0<=source_m;1:cache_y1<=source_m;2:cache_y2<=source_m;3:cache_y3<=source_m;endcase
     done_line_toggle<=~done_line_toggle;ms<=M_IDLE;quiet<=0;
    end else quiet<=quiet+1'b1;
   end
   default:ms<=M_IDLE;
  endcase
 end
end

wire in_window=enable && have_frame && de && px>=window_left && px<window_right && py>=window_top && py<window_bottom;
reg mask_q,de_q,border_q,header_q,mark_q,auto_mark_q,focus_q,valid_q,error_q,side_q;
reg [23:0] bg_q;
wire [23:0] pixel_color={color565[15:11],color565[15:13],color565[10:5],color565[10:9],color565[4:0],color565[4:2]};
always @(posedge clk or posedge rst)begin
 if(rst)begin
  raw_rgb<=0;color_enable<=0;mask_q<=0;de_q<=0;bg_q<=0;focus_q<=0;valid_q<=0;error_q<=0;side_q<=0;
  border_q<=0;header_q<=0;mark_q<=0;auto_mark_q<=0;
 end else begin
  color_enable<=de_q && mask_q && !border_q && !header_q && valid_q && !side_q;
  mask_q<=in_window;de_q<=de;bg_q<=irgb;focus_q<=focus;valid_q<=have_frame && row_valid && !blank_row && !outside_x;
  error_q<=error_visible;side_q<=!full_area && (lx<4 || px>=body_right);
  border_q<=!full_area && (px==window_left || px==window_right-1 || ly==0 || py==window_bottom-1);
  header_q<=!full_area && ly<13;
  mark_q<=ly>=4 && ly<9 && ((lx>=5 && lx<13) || (px>=window_right-22 && px<window_right-18) || (clip && px>=window_right-15 && px<window_right-11));
  auto_mark_q<=auto_en && ly>=4 && ly<9 && px>=window_right-7 && px<window_right-3;
  if(!de_q)raw_rgb<=0;
  else if(!mask_q)raw_rgb<=bg_q;
  else if(border_q)raw_rgb<=focus_q?24'h64E6A6:24'hD8E3EA;
  else if(header_q)raw_rgb<=mark_q?24'hFFFFFF:auto_mark_q?24'hFCE67A:error_q?24'h982C36:focus_q?24'h124F39:24'h16354D;
  else raw_rgb<=(!valid_q || side_q)?24'h000000:pixel_color;
 end
end
endmodule

// Four-clock color path. Saturation uses shift/add products. A contrast table
// is rebuilt only when contrast/brightness changes, during vertical blank.
// Two small dual-port BRAMs provide three parallel channel lookups.
module exhibit_video_color(input wire clk,rst,enable,
 input wire [92:0] cfg,input wire [23:0] irgb,output reg [23:0] rgb);
wire [3:0] mode;wire signed [7:0] brightness;
wire [7:0] contrast,saturation,zoom;wire signed [4:0] tone;
wire signed [9:0] pan_x,pan_y;wire fit,blend,compare;
wire [6:0] opacity;wire [5:0] speed;wire [1:0] position,transition,direction,tuning;
wire [3:0] duration,period;
assign {mode,brightness,contrast,tone,saturation,zoom,pan_x,pan_y,fit,opacity,blend,speed,position,transition,duration,period,direction,compare,tuning}=cfg;
reg [7:0] sat_gain,seen_sat,sat_quotient;
reg [14:0] sat_remainder;reg sat_busy;
always @(posedge clk or posedge rst)begin
 if(rst)begin sat_gain<=128;seen_sat<=100;sat_quotient<=0;sat_remainder<=0;sat_busy<=0;end
 else if(!sat_busy)begin
  if(saturation!=seen_sat)begin seen_sat<=saturation;sat_remainder<={saturation,7'd0};sat_quotient<=0;sat_busy<=1;end
 end else if(sat_remainder>=100)begin sat_remainder<=sat_remainder-15'd100;sat_quotient<=sat_quotient+1'b1;end
 else begin sat_gain<=sat_quotient;sat_busy<=0;end
end
reg [1:0] table_state;reg table_ready;
reg [7:0] seen_con,table_gain,table_quotient,table_index;
reg signed [7:0] seen_bright;
reg [14:0] table_remainder;
reg signed [17:0] table_phase;
wire table_we=table_state==3;
wire signed [17:0] table_value=18'sd128+(table_phase>>>7)+$signed({{10{seen_bright[7]}},seen_bright});
// Biased, clipped 9-bit storage is exact after final clamping for tone offsets up to +/-32.
wire [8:0] table_data=(table_value < -18'sd32) ? 9'd0 : (table_value > 18'sd287) ? 9'd319 : table_value[8:0]+9'd32;
always @(posedge clk or posedge rst)begin
 if(rst)begin table_state<=0;table_ready<=0;seen_con<=100;seen_bright<=0;table_gain<=128;table_quotient<=0;table_index<=0;table_remainder<=0;table_phase<=0;end
 else case(table_state)
  0:if(!table_ready || contrast!=seen_con || brightness!=seen_bright)begin
   seen_con<=contrast;seen_bright<=brightness;table_remainder<={contrast,7'd0};table_quotient<=0;table_state<=1;
  end
  1:if(table_remainder>=100)begin table_remainder<=table_remainder-15'd100;table_quotient<=table_quotient+1'b1;end
    else begin table_gain<=table_quotient;table_state<=2;end
  2:begin table_phase<=-$signed({3'd0,table_gain,7'd0});table_index<=0;table_state<=3;end
  3:begin
   table_phase<=table_phase+$signed({10'd0,table_gain});table_index<=table_index+1'b1;
   if(table_index==255)begin table_ready<=1;table_state<=0;end
  end
 endcase
end
function signed [18:0] product;
 input signed [9:0] value;input [7:0] gain;
 reg signed [18:0] v,p0,p1,p2,p3,p4,p5,p6,p7;
 begin
  v={{9{value[9]}},value};
  p0=gain[0]?v:19'sd0;p1=gain[1]?(v<<<1):19'sd0;
  p2=gain[2]?(v<<<2):19'sd0;p3=gain[3]?(v<<<3):19'sd0;
  p4=gain[4]?(v<<<4):19'sd0;p5=gain[5]?(v<<<5):19'sd0;
  p6=gain[6]?(v<<<6):19'sd0;p7=gain[7]?(v<<<7):19'sd0;
  product=((p0+p1)+(p2+p3))+((p4+p5)+(p6+p7));
 end
endfunction
function [7:0] clamp;
 input signed [20:0] value;
 begin clamp=value<0?0:value>255?255:value[7:0];end
endfunction
reg [3:0] bypass;
reg [23:0] original0,original1,original2;
reg [7:0] gray;reg signed [9:0] dr,dg,db;
reg signed [7:0] cr,cg,cb;
wire [8:0] qr,qg,qb;
wire [7:0] ar={~cr[7],cr[6:0]},ag={~cg[7],cg[6:0]},ab={~cb[7],cb[6:0]};
EG_LOGIC_BRAM #(.DATA_WIDTH_A(9),.DATA_WIDTH_B(9),.ADDR_WIDTH_A(8),.ADDR_WIDTH_B(8),
 .DATA_DEPTH_A(256),.DATA_DEPTH_B(256),.MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
 .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.RESETMODE("SYNC"),.IMPLEMENT("9K"),.INIT_FILE("NONE"),.FILL_ALL("NONE")) contrast_rg(
 .dia(table_data),.dib(9'd0),.addra(table_we?table_index:ar),.addrb(ag),
 .cea(1'b1),.ceb(1'b1),.ocea(1'b0),.oceb(1'b0),.clka(clk),.clkb(clk),.wea(table_we),.web(1'b0),
 .bea(1'b0),.beb(1'b0),.rsta(1'b0),.rstb(1'b0),.doa(qr),.dob(qg));
EG_LOGIC_BRAM #(.DATA_WIDTH_A(9),.DATA_WIDTH_B(9),.ADDR_WIDTH_A(8),.ADDR_WIDTH_B(8),
 .DATA_DEPTH_A(256),.DATA_DEPTH_B(256),.MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
 .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.RESETMODE("SYNC"),.IMPLEMENT("9K"),.INIT_FILE("NONE"),.FILL_ALL("NONE")) contrast_b(
 .dia(table_data),.dib(9'd0),.addra(table_we?table_index:ab),.addrb(8'd0),
 .cea(1'b1),.ceb(1'b0),.ocea(1'b0),.oceb(1'b0),.clka(clk),.clkb(clk),.wea(table_we),.web(1'b0),
 .bea(1'b0),.beb(1'b0),.rsta(1'b0),.rstb(1'b0),.doa(qb),.dob());
wire [9:0] gray_sum={2'b0,irgb[23:16]}+{1'b0,irgb[15:8],1'b0}+{2'b0,irgb[7:0]};
// Infer R/G multipliers; blue uses an explicit shift/add tree.
wire signed [18:0] sr=$signed(dr)*$signed({1'b0,sat_gain});
wire signed [18:0] sg=$signed(dg)*$signed({1'b0,sat_gain});
wire signed [18:0] sb=product(db,sat_gain);
always @(posedge clk or posedge rst)begin
 if(rst)begin bypass<=15;original0<=0;original1<=0;original2<=0;gray<=0;dr<=0;dg<=0;db<=0;cr<=0;cg<=0;cb<=0;rgb<=0;end
 else begin
  bypass<={bypass[2:0],(!enable || compare || !table_ready)};
  original0<=irgb;original1<=original0;original2<=original1;
  if(enable && !compare && table_ready)begin
  gray<=gray_sum[9:2];
  dr<=$signed({2'b0,irgb[23:16]})-$signed({2'b0,gray_sum[9:2]});
  dg<=$signed({2'b0,irgb[15:8]})-$signed({2'b0,gray_sum[9:2]});
  db<=$signed({2'b0,irgb[7:0]})-$signed({2'b0,gray_sum[9:2]});
  end
  if(!bypass[0])begin
  cr<=clamp($signed({1'b0,gray})+(sr>>>7))-8'd128;
  cg<=clamp($signed({1'b0,gray})+(sg>>>7))-8'd128;
  cb<=clamp($signed({1'b0,gray})+(sb>>>7))-8'd128;
  end
  rgb<=bypass[2]?original2:{
   clamp(($signed({12'd0,qr})-21'sd32)+($signed(tone)<<<1)),
   clamp(($signed({12'd0,qg})-21'sd32)),
   clamp(($signed({12'd0,qb})-21'sd32)-($signed(tone)<<<1))};
 end
end
endmodule
