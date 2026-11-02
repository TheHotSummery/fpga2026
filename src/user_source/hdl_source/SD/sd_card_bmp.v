
module sd_card_bmp #(
    parameter integer CLK_FREQ_HZ       = 100_000_000,
    parameter [31:0]  SCAN_START_SECTOR = 32'd0,
    parameter [31:0]  SCAN_MAX_SECTOR   = 32'd131071,
    parameter [3:0]   SCAN_TARGET_COUNT = 4'd8,
    parameter [23:0]  FRAME_PIXELS      = 24'd307200,
    parameter         IR_ACTIVE_LOW     = 1'b1
)(
    input                       clk,
    input                       rst,

    input pip_read_req,input [31:0] pip_read_sector,
    output pip_read_ack,pip_read_valid,pip_read_end,output [7:0] pip_read_byte,
    output [1:0] pip_catalog_valid,output [31:0] pip_sector0,pip_sector1,pip_size0,pip_size1,
    output pip_allowed,pip_card_reset,
    input write_ready,
    output reg [2:0] buffer_video,
    output wire interpolate_ready,output write_flip,output [2:0] media_index,output media_auto,output [5:0] media_available,
    input ctrl_next,ctrl_prev,ctrl_auto,ctrl_emerg,
    input [3:0] ctrl_period,
    output [1:0] pending_display,
    input                       key_next,
    input                       key_enc_sw,     // EC11 SW, active-low; own debounce (may sit low)
    input                       key_auto,
    input                       key_period,     // short: period; long: emergency toggle
    input                       emerg_n,        // external active-low emergency (J4), pullup
    input wire fire_trigger,
    input                       ir_sensor,      // J1-2 / FPGA G11 / GPIO1
    input                       vsync,          // video vs, commit display buffer on rising edge
    input                       fade_swap_ok,   // video domain: fade-out done, may swap
    output reg                  fade_out_req,   // ask video to fade out before swap
    output wire [3:0]           boot_status, // 0 init, 1 catalog, 2 missing BMP, 3 first frame, 4 PCM, 5 ready, 6 recovery
    output [3:0]                state_code,
    output [3:0]                period_sec,     // 1/2/3/5 for 7-seg
    output [1:0]                img_idx,        // current displayed image (slice 4 audio bind)
    input  [15:0]               bmp_width,
    input  [15:0]               bmp_height,
    output reg                  display_valid,
    input wire train_audio_available,train_audio_done,
    output wire rail_audio_present,
    output wire [2:0]           train_speech_event, // one-cycle future audio trigger: 1..5
    output [3:0]                train_phase,        // 0 idle, 1..11 automated railway sequence
    output [2:0]                train_journey_stage, // committed subtitle/station state; persists while idle

    input                       write_finish_toggle,
    input                       wav_spi_hold,
    input                       flash_cap_rdy,
    input                       flash_idle,
    input  [3:0]                flash_slot_ok,
    output reg [1:0]            write_buf_idx,
    output reg [1:0]            disp_buf_idx,

    // Slice 7 status
    output reg                  emerg_active,   // Lv.2 showing Buffer C
    output reg                  emerg_trig,     // sticky high while pending/active (scope)
    output reg [7:0]            lat_ms,         // request -> VSYNC commit (software estimate)
    output reg                  sd_frozen,      // TF timeout: freeze last frame
    output reg                  buf_c_ready,    // Buffer C filled
    output [1:0]                c_stat,         // UART: 0 idle, 1 ready, 2 fail, 3 loading
    output                      wav_pcm_we,
    output [19:0]               wav_pcm_addr,
    output [15:0]               wav_pcm_data,
    output reg                  wav_ready,
    output reg                  wav_load_busy,
    output [2:0]                wav_found_n,
    output [3:0]                wav_dbg,
    output [2:0]                bmp_found_n,
    output                      pic_wr_busy,

    output                      write_req,
    input                       write_req_ack,
    output                      write_en,
    output [31:0]               write_data,
    // Toggle in sd clk: abort left frame_fifo_write mid-session → no more ACKs (S3 forever).
    output reg                  wr_kill_tgl,
    output                      SD_nCS,
    output                      SD_DCLK,
    output                      SD_MOSI,
    input                       SD_MISO
);

wire key_next_press=ctrl_next;
wire key_enc_press=1'b0;
wire key_auto_press=ctrl_auto;
wire key_period_press=ctrl_prev;      // short press on KEY3 release
wire key_emerg_press=ctrl_emerg;       // long press on KEY3
wire key3_held=1'b0;             // KEY3 currently down (suppress next/auto)

wire             sd_sec_read;
wire [31:0]      sd_sec_read_addr;
wire [7:0]       sd_sec_read_data;
wire             sd_sec_read_data_valid;
wire             sd_sec_read_end;
wire bmp_data_wr_en,photo_pixel_valid;wire [23:0] photo_pixel;
wire raw_bmp_valid;
wire [23:0] raw_bmp_pixel;
wire [15:0] source_width,source_height;
wire normalize_overrun;
wire [23:0]      bmp_data;
wire             sd_init_done;
wire             bmp_ready_raw;
reg              bmp_ready;
// Keep parser state decoding out of the wide sector-selection/launch cone.
// Completion already requires observing source busy before accepting ready.
always @(posedge clk or posedge rst)begin
 if(rst)bmp_ready<=1'b0;else bmp_ready<=bmp_ready_raw;
end
wire             scan_done;
wire             scan_found_valid;
wire [31:0]      scan_found_sector;
wire [3:0]       scan_found_total;
wire             bmp_write_req;
wire             wav_found_valid;
wire [31:0]      wav_found_sector;
wire [2:0]       wav_found_total;
wire             wav_pcm_we_w;
wire [19:0]      wav_pcm_addr_w;
wire [15:0]      wav_pcm_data_w;
wire             wav_load_done;
wire             wav_load_ok;
wire             load_fail;
wire             wav_scan_done_w;
wire             wav_catalog_complete;
reg              load_source_started;
wire [31:0]      wav_resume_sector;

reg              scan_start_pulse;
reg              load_start_pulse;
reg              load_abort_pulse;
reg              orphan_abort_sent;
reg              wav_load_start_pulse;
reg              wav_scan_start_pulse;
reg              wav_phase2_kicked;
reg              wav_phase2_busy;
reg [1:0]        wav_playing_idx;
reg [31:0]       load_sector;
reg [31:0]       wav_load_sector_r;
reg [19:0]       wav_pcm_base_r;
reg [19:0]       wav_drain;
reg              wav_await_wrfin;
reg              wav_ok_hold;
reg              wav_saw_wrfin;
reg [$clog2(CLK_FREQ_HZ*30+1)-1:0] wav_await_cnt;
reg              wav_wr_req;
reg [1:0]        wav_ph;
reg [31:0]       wav_watch;
reg [2:0]        wav_found_count;
reg [1:0]        wav_load_idx;
reg [31:0]       wav_sector0, wav_sector1, wav_sector2, wav_sector3;
reg              scan_kicked;
reg              first_image_committed;
reg              auto_play_en;
reg              train_saved_auto;
reg [$clog2(CLK_FREQ_HZ*10+1)-1:0] auto_cnt;
reg [3:0]        img_found_count;
reg [2:0]        img_idx_r;
reg [2:0]        load_idx;
assign img_idx = img_idx_r[1:0];
assign media_index=img_idx_r;assign media_auto=auto_play_en;
reg [2:0] requested_media;
reg [15:0] load_frame,video_next_frame;
reg video_continuation,load_video_active;
reg [2:0] video_age;
reg [1:0]        pending_buf_idx;
reg [31:0]       img_sector0;
reg [31:0]       img_sector1;
reg [31:0]       img_sector2;
reg [31:0]       img_sector3;
reg [31:0]       img_sector4, img_sector5, img_sector6, img_sector7;
reg              loading_buf_c;
reg              buf_c_bmp_tried; // one-shot 5th-BMP attempt; fail → give_up
reg              buf_c_scrubbed;  // solid-red scrub finished before BMP (kills virgin-RAM snow)
reg              buf_c_give_up;   // fill/C abandoned → emerg stays solid red, never fake-ready
reg              load_saw_wrfin;
reg              c_src_done;      // C BMP: only accept write_finish after SD source done
reg [23:0]       c_pix_cnt;       // BMP write_en beats while loading C
reg [$clog2(CLK_FREQ_HZ/50+1)-1:0] c_drain_cnt;
reg [$clog2(CLK_FREQ_HZ/10+1)-1:0] c_quiet_cnt;
reg              c_audio_quiet;
reg [$clog2(CLK_FREQ_HZ/100+1)-1:0] c_post_hold;
localparam integer C_DRAIN_CYCLES = CLK_FREQ_HZ / 50; // 20 ms FIFO→SDRAM drain
reg              next_req_pending;
reg prev_request;
reg              load_busy;
reg              disp_commit_pending;
reg [1:0]        period_sel;
reg [2:0]        vs_sync;
reg              source_done_seen;
reg [2:0]        wrfin_tgl_sync;
wire             write_finish_pulse;

// Slice 7: arbitration / latency / BUF C / SD heal
reg              emerg_pending;
reg              heal_after_emerg; // exit Lv.2: reload BMP+WAV (heal any spray)
reg [1:0]        saved_disp_buf;
reg [2:0]        saved_img_idx;
// Independent emergency gallery; normal A/B front and PCM are protected.
reg [2:0] g_idx,g_last_idx,g_c_idx,g_wanted,g_load_idx;
reg [15:0] saved_video_frame;
reg g_c_valid,g_request,g_loading,g_commit,g_auto;
reg [$clog2(CLK_FREQ_HZ*10+1)-1:0] g_auto_cnt;
wire [2:0] g_photo_count = img_found_count > 4 ? img_found_count - 4'd4 : 3'd0;
wire [2:0] g_count = g_photo_count + {2'b0,pip_catalog_valid[0]};
wire g_video_selected = pip_catalog_valid[0] && g_idx == g_photo_count;
wire [1:0] g_back = !buf_c_ready ? 2'd2 :
                        disp_buf_idx == 2'd2 ? (saved_disp_buf ^ 2'd1) : 2'd2;
wire gallery_write_ok = emerg_active && g_loading && !emerg_pending &&
                       (write_buf_idx != saved_disp_buf) && (write_buf_idx != 2'd3);

reg              lat_running;
reg [$clog2(CLK_FREQ_HZ/1000+1)-1:0] lat_ms_div;
reg [2:0]        emerg_n_sync;
wire             emerg_ext_raw = ~emerg_n_sync[2]; // active-low pin, raw asserted
reg              emerg_ext_lvl;                    // debounced asserted
reg [$clog2(CLK_FREQ_HZ/50+1)-1:0] emerg_ext_db;
reg              emerg_ext_d;
wire             emerg_ext_rise = emerg_ext_lvl & ~emerg_ext_d;
wire             emerg_req_pulse;
localparam integer EMERG_N_DEB_CYCLES = CLK_FREQ_HZ / 50; // 20 ms, ignore scope finger noise

reg [1:0]        fill_state;     // 0 idle, 1 req, 2 stream
reg              fill_write_req;
reg              fill_write_en;
reg [1:0]        fill_div;
reg [23:0]       fill_cnt;
reg [31:0]       fill_watch;

reg              sd_bus_rst;
reg [$clog2(CLK_FREQ_HZ*6+1)-1:0] sd_watchdog;
reg [$clog2(CLK_FREQ_HZ*3+1)-1:0] heal_timer;
localparam integer INIT_TIMEOUT_CYCLES = CLK_FREQ_HZ * 8;
localparam integer CATALOG_IDLE_CYCLES = CLK_FREQ_HZ * 6;
reg [$clog2(INIT_TIMEOUT_CYCLES+1)-1:0] init_watch;
reg [$clog2(CATALOG_IDLE_CYCLES+1)-1:0] catalog_idle_watch;
reg catalog_stall_q;
reg [2:0]        sd_init_sync;
wire             sd_init_rise = sd_init_sync[1] && !sd_init_sync[2];

localparam [1:0] FILL_IDLE   = 2'd0;
localparam [1:0] FILL_REQ    = 2'd1;
localparam [1:0] FILL_STREAM = 2'd2;
localparam [1:0] WAV_PH_IDLE = 2'd0;
localparam [1:0] WAV_PH_REQ  = 2'd1;
localparam [1:0] WAV_PH_RUN  = 2'd2;
localparam integer LOAD_TIMEOUT_SD     = CLK_FREQ_HZ * 2;      // 2 s: BMP still coming from TF
localparam integer LOAD_TIMEOUT_WR     = CLK_FREQ_HZ * 6;      // 6 s: SD done, SDRAM write contended
localparam [31:0] WAV_TIMEOUT_CYCLES   = CLK_FREQ_HZ * 40;   // 40 s at the actual manager clock
wire wav_session_timeout=wav_load_busy && (wav_watch>=WAV_TIMEOUT_CYCLES-1);
localparam integer WAV_AWAIT_TIMEOUT   = CLK_FREQ_HZ * 30;     // 30 s: 960k PCM + video contention
localparam integer C_QUIET_CYCLES      = CLK_FREQ_HZ / 10;     // 100 ms steal/WAV drain before C
localparam integer C_POST_HOLD_CYCLES  = CLK_FREQ_HZ / 100;    // 10 ms after C before steal
localparam integer HEAL_PERIOD_CYCLES  = CLK_FREQ_HZ * 3;      // retry every 3 s
localparam integer HEAL_RST_CYCLES     = CLK_FREQ_HZ / 100;    // 10 ms reset pulse
localparam integer MS_DIV_MAX          = CLK_FREQ_HZ / 1000;

wire load_is_video=!loading_buf_c && (g_loading?(pip_catalog_valid[0] && g_load_idx==g_photo_count):load_idx[2]);
wire video_last;
wire video_pause,video_done,video_overrun,video_pixel_valid;
wire [23:0] video_pixel;
assign write_flip=!load_video_active;
wire [5:0] media_mask={1'b0,pip_catalog_valid[0],img_found_count>=4,img_found_count>=3,img_found_count>=2,img_found_count>=1};
assign interpolate_ready=disp_commit_pending && video_continuation && video_age>=4 && !emerg_active && !emerg_pending;
assign media_available=media_mask;
wire media_multiple=(media_mask & (media_mask-6'd1))!=0;
wire [2:0] request_base=next_req_pending?requested_media:(load_busy || disp_commit_pending || fade_out_req)?load_idx:img_idx_r;
always @(posedge clk or posedge rst)begin
 if(rst)begin load_video_active<=0;buffer_video<=0;end
 else if(load_start_pulse)begin
  load_video_active<=load_is_video;
  if(write_buf_idx<3)buffer_video[write_buf_idx]<=load_is_video;
 end
end

wire auto_tick;
wire [2:0] next_from_current;
wire       vs_rise = (vs_sync[2] == 1'b0) && (vs_sync[1] == 1'b1);
wire       media_commit_pulse =
    (disp_commit_pending && vs_rise && (!video_continuation || video_age>=5) && !emerg_active && !emerg_pending) ||
    (fade_out_req && fade_swap_ok && !load_busy && vs_rise && !emerg_active && !emerg_pending);
wire       train_video_done = media_commit_pulse && load_idx[2] && video_last;
wire       train_start_pulse,train_request_pulse,train_busy,train_finish_pulse;
wire [2:0] train_request_media,train_event_wire;
wire [3:0] train_phase_wire;
wire       audio_settled = wav_ready ||
                           (wav_phase2_kicked && !wav_phase2_busy && wav_catalog_complete && (wav_found_count == 3'd0));
rail_train_flow #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.IR_ACTIVE_LOW(IR_ACTIVE_LOW)) train_flow(
    .clk(clk),.rst(rst),.ir_sensor(ir_sensor),.frame_tick(vs_rise),
    .system_ready(display_valid && first_image_committed && scan_done && sd_init_done && audio_settled),
    .emergency(emerg_active || emerg_pending || emerg_req_pulse),
    .video_available(pip_catalog_valid[0]),.video_done(train_video_done),
    .audio_available(train_audio_available),.audio_done(train_audio_done),
    .media_commit(media_commit_pulse),.committed_media(load_idx),
    .photo_count(img_found_count>=4 ? 3'd4 : img_found_count[2:0]),
    .start_pulse(train_start_pulse),.request_pulse(train_request_pulse),
    .request_media(train_request_media),.speech_event(train_event_wire),
    .phase(train_phase_wire),.journey_stage(train_journey_stage),
    .busy(train_busy),.finish_pulse(train_finish_pulse)
);
assign train_speech_event=train_event_wire;
assign train_phase=train_phase_wire;
wire       fill_active = (fill_state != FILL_IDLE);
// Compact status uses only committed manager state; no media selection changes.
assign boot_status=sd_bus_rst ? 4'd6 : !sd_init_done ? 4'd0 :
 !scan_done ? 4'd1 : img_found_count==0 ? 4'd2 :
 !first_image_committed ? 4'd3 :
 (wav_load_busy || wav_phase2_busy || wav_await_wrfin ||
  (wav_found_count!=0 && !wav_ready)) ? 4'd4 : 4'd5;
// Watch inactivity, not total scan duration: valid sectors can continue
// for an arbitrarily large card without causing a reset. Register the
// deadline before it controls the high-fanout card-reset signal.
always @(posedge clk or posedge rst)begin
 if(rst)begin catalog_idle_watch<=0;catalog_stall_q<=0;end
 else begin
  catalog_stall_q<=0;
  // The resumed audio catalog is still a sector scan even when the initial
  // BMP scan has completed. Both scans need the same no-progress recovery.
  if((!(scan_kicked && !scan_done) && !wav_phase2_busy) || !sd_init_done || sd_bus_rst ||
     scan_start_pulse || wav_scan_start_pulse || sd_sec_read_data_valid || sd_sec_read_end)catalog_idle_watch<=0;
  else if(catalog_idle_watch>=CATALOG_IDLE_CYCLES-1)begin
   catalog_idle_watch<=0;catalog_stall_q<=1;
  end else catalog_idle_watch<=catalog_idle_watch+1'b1;
 end
end
// Preload C before the WAV session at cold boot. If WAV is already ready,
// retain the quiet guard. C and WAV may never share a write session.
// 5th BMP → C after WAV quiet. No solid scrub (scrub timeout was give_up → forever
// mux-red). Snow guard = quiet + no WAV mux + steal arm, not pixel-count gate.
wire       c_slot_ok = (img_found_count >= 4'd5);
wire       want_c_scrub = 1'b0;
wire       want_c_bmp =
    first_image_committed && c_slot_ok && !buf_c_ready &&
    !buf_c_bmp_tried && !buf_c_give_up &&
    !load_busy && !loading_buf_c && !fill_active && !fade_out_req && (c_post_hold==0) &&
    !disp_commit_pending && !emerg_active && !emerg_pending &&
    bmp_ready && !sd_frozen && scan_done &&
    !wav_load_busy && !wav_await_wrfin && !wav_phase2_busy &&
    (!wav_ready || c_audio_quiet);
wire pip_master_busy;
reg [$clog2(CLK_FREQ_HZ*2+1)-1:0] pip_watch;
reg [$clog2(CLK_FREQ_HZ/100+1)-1:0] pip_reset_count;
always @(posedge clk or posedge rst)begin
 if(rst)begin pip_watch<=0;pip_reset_count<=0;end
 else begin
  if(pip_reset_count!=0)pip_reset_count<=pip_reset_count-1'b1;
  if(!pip_master_busy)pip_watch<=0;
  else if(pip_watch>=CLK_FREQ_HZ*2-1)begin pip_watch<=0;pip_reset_count<=CLK_FREQ_HZ/100;end
  else pip_watch<=pip_watch+1'b1;
 end
end
wire sd_rst = rst | sd_bus_rst | (pip_reset_count!=0);
assign pip_card_reset=sd_rst;
// Ordinary BMP loads can lend the SPI port at sector boundaries. Emergency and WAV have priority.
assign pip_allowed=display_valid && scan_done && !sd_frozen && !emerg_active && !emerg_pending &&
 !emerg_trig && !wav_load_busy && !wav_phase2_busy && !wav_await_wrfin && !loading_buf_c &&
 !fill_active && !g_request && !g_loading && !g_commit && !ctrl_emerg;
// Never mux PCM onto the bus while C scrub/BMP is active (glitch → snow in C).
wire       use_wav_wr = wav_load_busy && !loading_buf_c && !fill_active;

// Hard-silence App_wr during Lv.2 (orphan bmp/fill must not spray A/B/PCM).
assign write_req  = (emerg_pending || (emerg_active && !gallery_write_ok)) ? 1'b0 :
                    fill_active ? fill_write_req :
                    (use_wav_wr && wav_wr_req) ? wav_wr_req : bmp_write_req;
assign write_en   = (emerg_pending || (emerg_active && !gallery_write_ok)) ? 1'b0 :
                    fill_active ? fill_write_en  :
                    use_wav_wr  ? wav_pcm_we_w   : bmp_data_wr_en;
assign write_data = fill_active ? 32'h0000_F800 :
                    use_wav_wr  ? {16'h0000, wav_pcm_data_w} :
                    {16'h0000, bmp_data[23:19], bmp_data[15:10], bmp_data[7:3]};
assign next_from_current=playlist_step(img_idx_r,prev_request);
assign write_finish_pulse = wrfin_tgl_sync[2] ^ wrfin_tgl_sync[1];
wire fire_enter,fire_force;
wire [2:0] fire_target,fire_phase;
exhibit_fire_sequence u_fire_sequence(.clk(clk),.rst(rst),.trigger(fire_trigger),.frame_tick(vs_rise),
 .emergency_active(emerg_active),.page_ready(buf_c_ready && !g_loading && !g_commit && !load_busy),
 .gallery_index(g_idx),.force_enter(fire_enter),.force_page(fire_force),.target_index(fire_target),.phase(fire_phase));
assign emerg_req_pulse = key_emerg_press | emerg_ext_rise | (fire_enter && !emerg_active && !emerg_pending);
assign wav_pcm_we   = wav_pcm_we_w;
assign wav_pcm_addr = wav_pcm_addr_w;
assign wav_pcm_data = wav_pcm_data_w;
assign wav_found_n  = wav_found_count;
assign wav_dbg      = {wav_load_busy, display_valid, first_image_committed, wav_ready};
assign bmp_found_n  = img_found_count >= 8 ? 3'd7 : img_found_count[2:0]; // legacy UART saturates at 7
assign c_stat       = buf_c_ready ? 2'd1 :
                      loading_buf_c ? 2'd3 :
                      (buf_c_give_up || buf_c_bmp_tried) ? 2'd2 : 2'd0;
// Continuous video writes arbitrate per SDRAM burst; they must not mute or starve PCM.
assign pic_wr_busy  = (load_busy && !load_video_active) | fill_active | loading_buf_c | want_c_bmp |
                      want_c_scrub | (c_post_hold != 24'd0) |
                      (wav_ready && !c_audio_quiet);

wire key_next_any=key_next_press;
assign pending_display=pending_buf_idx;
// Register the stable carousel deadline before the counter/FSM decision.
// This cuts the UI-period -> multiply -> compare -> sector-mux timing path.
reg [$clog2(CLK_FREQ_HZ*10+1)-1:0] auto_period_m1;
always @(posedge clk or posedge rst)begin
    if(rst)auto_period_m1<=CLK_FREQ_HZ*3-1;
    else auto_period_m1<=CLK_FREQ_HZ*((ctrl_period>=1 && ctrl_period<=10)?ctrl_period:4'd3)-1;
end
// Byte-parallel unsigned comparison; identical to >= without a 32-bit
// serial comparison cone feeding the high-fanout load enable.
function ge32;
 input [31:0] a,b;
 begin
  ge32=(a[31:24]>b[31:24]) ||
       ((a[31:24]==b[31:24]) && (a[23:16]>b[23:16])) ||
       ((a[31:24]==b[31:24]) && (a[23:16]==b[23:16]) && (a[15:8]>b[15:8])) ||
       ((a[31:24]==b[31:24]) && (a[23:16]==b[23:16]) && (a[15:8]==b[15:8]) && (a[7:0]>=b[7:0]));
 end
endfunction
// Separate deadline comparison from the high-fanout sector/load enable.
// A self-clearing pulse also prevents a duplicate tick when a counter resets.
reg auto_tick_q,g_auto_tick_q;
always @(posedge clk or posedge rst)begin
 if(rst)begin auto_tick_q<=0;g_auto_tick_q<=0;end
 else begin
  if(auto_tick_q)auto_tick_q<=0;
  else auto_tick_q<=ge32(auto_cnt,auto_period_m1);
  if(g_auto_tick_q || !emerg_active || fire_force)g_auto_tick_q<=0;
  else g_auto_tick_q<=ge32(g_auto_cnt,auto_period_m1);
 end
end
assign auto_tick=auto_tick_q;
assign period_sec = (ctrl_period>=1 && ctrl_period<=10)?ctrl_period:4'd3;


function [2:0] gallery_step;
    input [2:0] cur;input [2:0] count;input backwards;
    begin
        if(count<=1)gallery_step=0;
        else if(backwards)gallery_step=cur==0?count-1'b1:cur-1'b1;
        else gallery_step={1'b0,cur}+4'd1>={1'b0,count}?3'd0:cur+1'b1;
    end
endfunction
function [31:0] gallery_sector;
    input [2:0] idx;
    begin if(pip_catalog_valid[0] && idx==g_photo_count)gallery_sector=pip_sector0;else case(idx)
        0:gallery_sector=img_sector4;1:gallery_sector=img_sector5;
        2:gallery_sector=img_sector6;default:gallery_sector=img_sector7;
    endcase end
endfunction

function [2:0] playlist_step;
 input [2:0] cur;input backwards;
 integer j;reg [2:0] candidate;reg found;
 begin
  candidate=cur;found=0;playlist_step=cur;
  for(j=0;j<6;j=j+1)begin
   candidate=backwards?(candidate==0?3'd5:candidate-1'b1):(candidate>=5?3'd0:candidate+1'b1);
   if(!found && media_mask[candidate])begin playlist_step=candidate;found=1;end
  end
 end
endfunction
function [2:0] next_index_limited;
 input [2:0] cur;input [3:0] count;
 begin next_index_limited=playlist_step(cur,1'b0);end
endfunction
function [31:0] sector_lut;
 input [2:0] idx;
 begin case(idx)
  0:sector_lut=img_sector0;1:sector_lut=img_sector1;2:sector_lut=img_sector2;3:sector_lut=img_sector3;
  4:sector_lut=pip_sector0;5:sector_lut=pip_sector1;default:sector_lut=img_sector0;
 endcase end
endfunction

function [31:0] wav_sector_lut;
    input [1:0] idx;
    begin
        case (idx)
            2'd0: wav_sector_lut = wav_sector0;
            2'd1: wav_sector_lut = wav_sector1;
            2'd2: wav_sector_lut = wav_sector2;
            2'd3: wav_sector_lut = wav_sector3;
            default: wav_sector_lut = wav_sector0;
        endcase
    end
endfunction

function [1:0] next_buf_lut;
    input [1:0] cur_disp_buf;
    input       valid_now;
    begin
        if (!valid_now)
            next_buf_lut = 2'd0;
        else if (cur_disp_buf == 2'd0)
            next_buf_lut = 2'd1;
        else
            next_buf_lut = 2'd0; // never pick Buffer C for carousel
    end
endfunction

always @(posedge clk or posedge rst) begin
    if (rst) begin
        wrfin_tgl_sync        <= 3'b000;
        scan_start_pulse      <= 1'b0;
        load_start_pulse      <= 1'b0;
        load_abort_pulse      <= 1'b0;
        orphan_abort_sent     <= 1'b0;
        wr_kill_tgl           <= 1'b0;
        wav_load_start_pulse  <= 1'b0;
        wav_scan_start_pulse  <= 1'b0;
        wav_phase2_kicked     <= 1'b0;
        wav_phase2_busy       <= 1'b0;
        wav_playing_idx       <= 2'd0;
        load_sector           <= 32'd0;
        scan_kicked           <= 1'b0;
        load_source_started  <= 1'b0;
        first_image_committed <= 1'b0;
        auto_play_en          <= 1'b0;
        train_saved_auto       <= 1'b0;
        auto_cnt              <= 32'd0;
        img_found_count       <= 4'd0;
        requested_media<=0;load_frame<=0;video_next_frame<=0;video_continuation<=0;video_age<=0;saved_video_frame<=0;
        img_idx_r             <= 2'd0;
        load_idx              <= 2'd0;
        pending_buf_idx       <= 2'd0;
        write_buf_idx         <= 2'd0;
        disp_buf_idx          <= 2'd0;
        img_sector0           <= 32'd0;
        img_sector1           <= 32'd0;
        img_sector2           <= 32'd0;
        img_sector3           <= 32'd0;
        img_sector4           <= 32'd0;
        img_sector5<=0;img_sector6<=0;img_sector7<=0;
        g_idx<=0;g_last_idx<=0;g_c_idx<=0;g_wanted<=0;g_load_idx<=0;
        g_c_valid<=0;g_request<=0;g_loading<=0;g_commit<=0;g_auto<=0;g_auto_cnt<=0;
        loading_buf_c         <= 1'b0;
        buf_c_give_up         <= 1'b0;
        buf_c_scrubbed        <= 1'b0;
        load_saw_wrfin        <= 1'b0;
        c_src_done            <= 1'b0;
        c_pix_cnt             <= 24'd0;
        c_drain_cnt           <= 32'd0;
        c_quiet_cnt           <= 32'd0;
        c_audio_quiet         <= 1'b0;
        c_post_hold           <= 24'd0;
        next_req_pending      <= 1'b0;
        prev_request<=0;
        load_busy             <= 1'b0;
        source_done_seen      <= 1'b0;
        display_valid         <= 1'b0;
        disp_commit_pending   <= 1'b0;
        fade_out_req          <= 1'b0;
        period_sel            <= 2'd0;
        vs_sync               <= 3'b000;
        emerg_active          <= 1'b0;
        emerg_pending         <= 1'b0;
        emerg_trig            <= 1'b0;
        heal_after_emerg      <= 1'b0;
        lat_ms                <= 8'd0;
        lat_running           <= 1'b0;
        lat_ms_div            <= 17'd0;
        saved_disp_buf        <= 2'd0;
        saved_img_idx         <= 2'd0;
        emerg_n_sync          <= 3'b111;
        emerg_ext_lvl         <= 1'b0;
        emerg_ext_db          <= 32'd0;
        emerg_ext_d           <= 1'b0;
        fill_state            <= FILL_IDLE;
        fill_write_req        <= 1'b0;
        fill_write_en         <= 1'b0;
        fill_div              <= 2'd0;
        fill_cnt              <= 24'd0;
        fill_watch            <= 32'd0;
        buf_c_ready           <= 1'b0;
        buf_c_bmp_tried       <= 1'b0;
        buf_c_give_up         <= 1'b0;
        sd_frozen             <= 1'b0;
        sd_bus_rst            <= 1'b0;
        sd_watchdog           <= 32'd0;
        heal_timer            <= 32'd0;
        init_watch            <= 32'd0;
        sd_init_sync          <= 3'b000;
        wav_load_busy         <= 1'b0;
        wav_drain             <= 20'd0;
        wav_await_wrfin       <= 1'b0;
        wav_ok_hold           <= 1'b0;
        wav_saw_wrfin         <= 1'b0;
        wav_await_cnt         <= 32'd0;
        wav_wr_req            <= 1'b0;
        wav_ph                <= 2'd0;
        wav_watch             <= 32'd0;
        wav_found_count       <= 3'd0;
        wav_load_idx          <= 2'd0;
        wav_sector0           <= 32'd0;
        wav_sector1           <= 32'd0;
        wav_sector2           <= 32'd0;
        wav_sector3           <= 32'd0;
        wav_load_sector_r     <= 32'd0;
        wav_pcm_base_r        <= 20'd0;
        wav_ready             <= 1'b0;
    end else begin
        wrfin_tgl_sync   <= {wrfin_tgl_sync[1:0], write_finish_toggle};
        vs_sync          <= {vs_sync[1:0], vsync};
        emerg_n_sync     <= {emerg_n_sync[1:0], emerg_n};
        // Debounce external emerg_n (J1-10): probing nearby pins used to glitch
        // mid Buffer-C load → abort, bmp_tried stuck, forever mux-red.
        if (emerg_ext_raw == emerg_ext_lvl)
            emerg_ext_db <= 32'd0;
        else if (emerg_ext_db >= EMERG_N_DEB_CYCLES - 1) begin
            emerg_ext_lvl <= emerg_ext_raw;
            emerg_ext_db  <= 32'd0;
        end else
            emerg_ext_db <= emerg_ext_db + 32'd1;
        emerg_ext_d      <= emerg_ext_lvl;
        sd_init_sync     <= {sd_init_sync[1:0], sd_init_done};
        scan_start_pulse <= 1'b0;
        if(vs_rise && (emerg_active?g_video_selected:img_idx_r[2]) && video_age<7)video_age<=video_age+1'b1;
        load_start_pulse <= 1'b0;
        load_abort_pulse <= 1'b0;
        wav_load_start_pulse <= 1'b0;
        wav_scan_start_pulse <= 1'b0;
        fill_write_en    <= 1'b0;

        // After wav_ready: drain steal/WAV bus before any C scrub/BMP.
        if (!wav_ready) begin
            c_audio_quiet <= 1'b0;
            c_quiet_cnt   <= 32'd0;
        end else if (!c_audio_quiet) begin
            if (c_quiet_cnt >= C_QUIET_CYCLES - 1)
                c_audio_quiet <= 1'b1;
            else
                c_quiet_cnt <= c_quiet_cnt + 32'd1;
        end
        if (c_post_hold != 24'd0)
            c_post_hold <= c_post_hold - 24'd1;

        // Orphan: load_busy cleared (emerg/watchdog) but bmp_read still in LOAD_WAIT (S3)
        // → bmp_ready=0 forever → K2/旋钮切图全死。One-shot (not every cycle).
        if (!load_busy && !wav_load_busy &&
            ((state_code == 4'd3) || (state_code == 4'd4))) begin
            if (!orphan_abort_sent) begin
                load_abort_pulse  <= 1'b1;
                wr_kill_tgl       <= ~wr_kill_tgl;
                orphan_abort_sent <= 1'b1;
            end
        end else
            orphan_abort_sent <= 1'b0;

        // ---- software latency counter (ms) ----
        if (lat_running) begin
            if (lat_ms_div >= MS_DIV_MAX - 1) begin
                lat_ms_div <= 17'd0;
                if (lat_ms != 8'd255)
                    lat_ms <= lat_ms + 8'd1;
            end else begin
                lat_ms_div <= lat_ms_div + 17'd1;
            end
        end

        // ---- SD soft-reset heal pulse ----
        if (sd_bus_rst) begin
            if (heal_timer >= HEAL_RST_CYCLES - 1) begin
                sd_bus_rst <= 1'b0;
                heal_timer <= 32'd0;
            end else begin
                heal_timer <= heal_timer + 32'd1;
            end
        end else if (catalog_stall_q) begin
            // An initialized card can stop answering its first FAT sector.
            // The picture-load watchdog cannot see this state. Restart the
            // card/parser together and let the ordinary catalog path rescan.
            sd_bus_rst <= 1'b1;
            heal_timer <= 32'd0;
            init_watch <= 0;
            scan_kicked <= 1'b0;
            if(display_valid)sd_frozen <= 1'b1;
        end else if (!sd_init_done && !wav_load_busy) begin
            // Leave enough time for a slow cold card to finish ACMD41.
            // A frozen picture must not shorten this initialization deadline.
            heal_timer <= 32'd0;
            if (init_watch >= INIT_TIMEOUT_CYCLES - 1) begin
                init_watch <= 32'd0;
                sd_bus_rst <= 1'b1;
            end else begin
                init_watch <= init_watch + 32'd1;
            end
        end else begin
            heal_timer <= 32'd0;
            init_watch <= 32'd0;
        end

        // ---- watchdog: only a stuck BMP *load* (not the 64MB sector scan).
        // After TF bytes are in, SDRAM write can exceed 2s under video contention:
        // soft-abort + retry, do NOT freeze (freeze used to clear wav_ready → 20s WAV reload → I stuck).
        if (load_busy && !emerg_pending && !emerg_req_pulse && !sd_frozen && !fill_active) begin
            if (sd_watchdog >= ((source_done_seen ? LOAD_TIMEOUT_WR : LOAD_TIMEOUT_SD) - 1)) begin
                sd_watchdog         <= 32'd0;
                load_busy           <= 1'b0;
                source_done_seen    <= 1'b0;
                load_abort_pulse    <= 1'b1;
                wr_kill_tgl         <= ~wr_kill_tgl;
                fade_out_req        <= 1'b0;
                disp_commit_pending <= 1'b0;
                if (g_loading) begin
                    g_loading<=0;g_commit<=0;g_request<=0;g_auto<=0;g_auto_cnt<=0;
                    c_post_hold<=C_POST_HOLD_CYCLES[23:0];
                    // Front stays valid. Partial back page is never committed.
                end else if (loading_buf_c) begin
                    loading_buf_c    <= 1'b0;
                    buf_c_give_up    <= 1'b1; // partial C stays unread; Lv.2 = solid red
                    write_buf_idx    <= 2'd0; // was 2 → widx2 held steal forever (U000)
                    c_post_hold      <= C_POST_HOLD_CYCLES[23:0];
                    next_req_pending <= 1'b0;
                end else if (display_valid && source_done_seen) begin
                    // write stalled: keep audio, retry same next index
                    next_req_pending <= 1'b1;
                end else if (display_valid) begin
                    next_req_pending <= 1'b1;
                    sd_frozen        <= 1'b1;
                    auto_play_en     <= 1'b0;
                    sd_bus_rst       <= 1'b1;
                    heal_timer       <= 32'd0;
                end else
                    next_req_pending <= 1'b0;
            end else begin
                sd_watchdog <= sd_watchdog + 32'd1;
            end
        end else begin
            sd_watchdog <= 32'd0;
        end

        // SD lost while displayed: keep pixels, arm rescan
        if (!sd_init_done && !wav_load_busy) begin
            load_busy           <= 1'b0;
            source_done_seen    <= 1'b0;
            fade_out_req        <= 1'b0;
            disp_commit_pending <= 1'b0;
            fill_state          <= FILL_IDLE;
            fill_write_req      <= 1'b0;
            loading_buf_c       <= 1'b0;
            g_loading<=0;g_commit<=0;g_request<=0;g_auto<=0;
            if (!display_valid) begin
                scan_kicked           <= 1'b0;
                first_image_committed <= 1'b0;
                auto_play_en          <= 1'b0;
                auto_cnt              <= 32'd0;
                img_found_count       <= 4'd0;
                img_idx_r             <= 2'd0;
                load_idx              <= 2'd0;
                pending_buf_idx       <= 2'd0;
                write_buf_idx         <= 2'd0;
                disp_buf_idx          <= 2'd0;
                img_sector0           <= 32'd0;
                img_sector1           <= 32'd0;
                img_sector2           <= 32'd0;
                img_sector3           <= 32'd0;
                img_sector4           <= 32'd0;
        img_sector5<=0;img_sector6<=0;img_sector7<=0;
        g_idx<=0;g_last_idx<=0;g_c_idx<=0;g_wanted<=0;g_load_idx<=0;
        g_c_valid<=0;g_request<=0;g_loading<=0;g_commit<=0;g_auto<=0;g_auto_cnt<=0;
                loading_buf_c         <= 1'b0;
                next_req_pending      <= 1'b0;
                display_valid         <= 1'b0;
                buf_c_ready           <= 1'b0;
                buf_c_bmp_tried       <= 1'b0;
                buf_c_scrubbed        <= 1'b0;
                buf_c_give_up         <= 1'b0;
                c_src_done            <= 1'b0;
                emerg_active          <= 1'b0;
                emerg_pending         <= 1'b0;
                emerg_trig            <= 1'b0;
                lat_running           <= 1'b0;
                sd_frozen             <= 1'b0;
            end else begin
                // freeze path: keep pixels + PCM already in SDRAM (do not force WAV reload)
                scan_kicked      <= 1'b0;
                img_found_count  <= 4'd0;
                auto_play_en     <= 1'b0;
                wav_load_busy    <= 1'b0;
                wav_drain        <= 20'd0;
                wav_await_wrfin  <= 1'b0;
                wav_ph           <= WAV_PH_IDLE;
                // keep wav_ready / next_req_pending
                sd_frozen        <= 1'b1;
            end
        end else begin
            // Loss of SD initialization already armed a fresh catalog above.
            // Do not clear scan_kicked on the delayed synchronized rise:
            // an immediately ready parser may already be scanning by then.

            if (scan_found_valid) begin
                case (img_found_count)
                    3'd0: img_sector0 <= scan_found_sector;
                    3'd1: img_sector1 <= scan_found_sector;
                    3'd2: img_sector2 <= scan_found_sector;
                    3'd3: img_sector3 <= scan_found_sector;
                    4'd4: img_sector4 <= scan_found_sector;
                    4'd5: img_sector5 <= scan_found_sector;
                    4'd6: img_sector6 <= scan_found_sector;
                    4'd7: img_sector7 <= scan_found_sector;
                    default: ;
                endcase
                if (img_found_count < 4'd8)
                    img_found_count <= img_found_count + 4'd1;
            end

            if (wav_found_valid) begin
                case (wav_found_count)
                    3'd0: wav_sector0 <= wav_found_sector;
                    3'd1: wav_sector1 <= wav_found_sector;
                    3'd2: wav_sector2 <= wav_found_sector;
                    3'd3: wav_sector3 <= wav_found_sector;
                    default: ;
                endcase
                if (wav_found_count < 3'd4)
                    wav_found_count <= wav_found_count + 3'd1;
            end

            if (!load_busy || load_start_pulse)
                load_source_started <= 1'b0;
            else if (!bmp_ready)
                load_source_started <= 1'b1;
            if (load_busy && load_source_started && bmp_ready && !load_start_pulse)
                source_done_seen <= 1'b1;
            if (load_busy && load_source_started && write_finish_pulse)
                load_saw_wrfin <= 1'b1;
            if (loading_buf_c && load_busy && source_done_seen)
                c_src_done <= 1'b1;
            if ((loading_buf_c || g_loading) && write_en)
                c_pix_cnt <= c_pix_cnt + 24'd1;
            // Keep write target glued to C for the whole session (heal/WAV must not steal idx).
            if (loading_buf_c)
                write_buf_idx <= 2'd2;

            // A/B: sticky write_finish OK.
            // C: require the exact pixel count and a completed write session.
            // A drain timeout aborts C and keeps the solid-red fallback.
            if (!fill_active && load_busy) begin
                if (g_loading) begin
                    if (source_done_seen && (c_pix_cnt == FRAME_PIXELS) &&
                        (write_finish_pulse || load_saw_wrfin)) begin
                        load_busy<=0;g_loading<=0;g_commit<=1;
                        if(buf_c_ready && !video_continuation)fade_out_req<=1;
                        if(load_video_active)video_next_frame<=video_last?16'd0:load_frame+1'b1;
                        source_done_seen<=0;load_saw_wrfin<=0;
                        c_post_hold<=load_video_active?0:C_POST_HOLD_CYCLES[23:0];
                        if(pending_buf_idx==2'd2)begin g_c_valid<=!load_video_active;g_c_idx<=g_load_idx;end
                    end
                end else if (loading_buf_c) begin
                    if (source_done_seen && (c_pix_cnt == FRAME_PIXELS) &&
                        (write_finish_pulse || load_saw_wrfin)) begin
                        load_busy        <= 1'b0;
                        source_done_seen <= 1'b0;
                        load_saw_wrfin   <= 1'b0;
                        c_src_done       <= 1'b0;
                        c_drain_cnt      <= 32'd0;
                        loading_buf_c    <= 1'b0;
                        buf_c_ready      <= 1'b1;
                        g_c_valid<=1;g_c_idx<=0;
                        buf_c_give_up    <= 1'b0;
                        write_buf_idx    <= 2'd0;
                        c_post_hold      <= C_POST_HOLD_CYCLES[23:0];
                        heal_after_emerg <= 1'b0;
                    end else if (source_done_seen && (c_pix_cnt >= FRAME_PIXELS)) begin
                        if (c_drain_cnt >= C_DRAIN_CYCLES - 1) begin
                            wr_kill_tgl      <= ~wr_kill_tgl;
                            load_busy        <= 1'b0;
                            source_done_seen <= 1'b0;
                            load_saw_wrfin   <= 1'b0;
                            c_src_done       <= 1'b0;
                            c_drain_cnt      <= 32'd0;
                            loading_buf_c    <= 1'b0;
                            buf_c_ready      <= 1'b0;
                            buf_c_give_up    <= 1'b1;
                            write_buf_idx    <= 2'd0;
                            c_post_hold      <= C_POST_HOLD_CYCLES[23:0];
                            heal_after_emerg <= 1'b0;
                        end else
                            c_drain_cnt <= c_drain_cnt + 32'd1;
                    end else
                        c_drain_cnt <= 32'd0;
                end else if (source_done_seen && (write_finish_pulse || load_saw_wrfin)) begin
                    load_busy        <= 1'b0;
                    source_done_seen <= 1'b0;
                    load_saw_wrfin   <= 1'b0;
                    if(load_video_active)video_next_frame<=video_last?16'd0:load_frame+1'b1;
                    if (!first_image_committed || video_continuation) begin
                        disp_commit_pending <= 1'b1;
                    end
                    else fade_out_req<=1'b1; // new image is now fully written
                end
            end

            if (disp_commit_pending && vs_rise && (!video_continuation || video_age>=5) && !emerg_active && !emerg_pending) begin
                img_idx_r<=load_idx;video_age<=0;
                disp_buf_idx          <= pending_buf_idx;
                display_valid         <= 1'b1;
                first_image_committed <= 1'b1;
                disp_commit_pending   <= 1'b0;
            end

            // Early fade may reach black before BMP write finishes — wait !load_busy.
            if (fade_out_req && fade_swap_ok && !load_busy && vs_rise &&
                !emerg_active && !emerg_pending) begin
                disp_buf_idx          <= pending_buf_idx;
                img_idx_r             <= load_idx;video_age<=0;
                display_valid         <= 1'b1;
                first_image_committed <= 1'b1;
                fade_out_req          <= 1'b0;
            end

            // ---- Lv.2 emergency: request -> next VSYNC commit ----
            if (emerg_req_pulse && display_valid && !emerg_pending) begin
                if (!emerg_active) begin
                    emerg_pending  <= 1'b1;
                    emerg_trig     <= 1'b1;
                    saved_disp_buf <= (disp_buf_idx == 2'd2) ? saved_disp_buf : disp_buf_idx;
                    saved_img_idx  <= img_idx_r;
                    saved_video_frame<=video_next_frame;
                    lat_ms         <= 8'd0;
                    lat_ms_div     <= 17'd0;
                    lat_running    <= 1'b1;
                    fade_out_req   <= 1'b0; // preempt fade
                    // Normal autoplay is paused, not changed by emergency controls.
                    next_req_pending <= 1'b0;
                end else begin
                    // exit Lv.2 on next VSYNC
                    emerg_pending <= 1'b1;
                    emerg_trig    <= 1'b1;
                    lat_ms        <= 8'd0;
                    lat_ms_div    <= 17'd0;
                    lat_running   <= 1'b1;
                end
            end

            if (emerg_pending && vs_rise) begin
                if (!emerg_active) begin
                    // Point reader at C when ready (pixel mux falls back to red if not).
                    if (g_c_valid)disp_buf_idx<=2'd2;
                    buf_c_ready<=g_c_valid && (!fire_force || g_c_idx==fire_target);
                    g_idx<=g_c_idx;g_wanted<=fire_force?fire_target:g_last_idx;
                    g_request<=fire_force ? ((g_photo_count>fire_target) && (!g_c_valid || g_c_idx!=fire_target)) : (!g_c_valid || (g_c_idx!=g_last_idx));
                    g_loading<=0;g_commit<=0;g_auto_cnt<=0;video_age<=0;video_continuation<=0;
                    c_post_hold<=C_POST_HOLD_CYCLES[23:0];
                    emerg_active  <= 1'b1;
                    img_idx_r     <= 2'd0;
                    emerg_pending <= 1'b0;
                    lat_running   <= 1'b0;
                    // Abort picture/fill traffic only — do NOT sd_bus_rst (that
                    // wiped FAT counts → UART B0/W0/S7 blips and rescan storms).
                    load_busy           <= 1'b0;
                    source_done_seen    <= 1'b0;
                    load_saw_wrfin      <= 1'b0;
                    load_abort_pulse    <= 1'b1; // unstick S3 LOAD_WAIT (write_req was gated off)
                    wr_kill_tgl         <= ~wr_kill_tgl;
                    c_src_done          <= 1'b0;
                    fade_out_req        <= 1'b0;
                    disp_commit_pending <= 1'b0;
                    // Aborting C mid-load must allow retry after exit (else forever red).
                    if (loading_buf_c || (!buf_c_ready && buf_c_bmp_tried)) begin
                        buf_c_bmp_tried <= 1'b0;
                        buf_c_give_up   <= 1'b0;
                    end
                    loading_buf_c       <= 1'b0;
                    write_buf_idx       <= 2'd0;
                    next_req_pending    <= 1'b0;
                    fill_state          <= FILL_IDLE;
                    fill_write_req      <= 1'b0;
                    fill_write_en       <= 1'b0;
                    // A partially loaded WAV must be aborted, not paused forever.
                    // Completed PCM stays untouched; normal image/audio binding is retained.
                    img_idx_r<=saved_img_idx;
                    wav_load_busy<=0;wav_await_wrfin<=0;wav_drain<=0;wav_ph<=WAV_PH_IDLE;
                    wav_wr_req<=0;wav_watch<=0;wav_await_cnt<=0;wav_saw_wrfin<=0;
                    wav_load_start_pulse<=0;wav_phase2_busy<=0;
                end else begin
                    disp_buf_idx     <= saved_disp_buf;
                    img_idx_r        <= saved_img_idx;
                    video_next_frame<=saved_video_frame;video_age<=0;video_continuation<=0;
                    emerg_active     <= 1'b0;
                    emerg_trig       <= 1'b0;
                    emerg_pending    <= 1'b0;
                    lat_running      <= 1'b0;
                    load_abort_pulse <= 1'b1;
                    wr_kill_tgl      <= ~wr_kill_tgl;
                    // Heal A/B only if C already OK; else retry C first (avoid idx clash).
                    if (g_c_valid)
                        heal_after_emerg <= 1'b0;
                    else begin
                        heal_after_emerg <= 1'b0;
                        buf_c_bmp_tried  <= 1'b0;
                        buf_c_give_up    <= 1'b0;
                    end
                    fade_out_req     <= 1'b0;
                    load_busy<=0;source_done_seen<=0;load_saw_wrfin<=0;loading_buf_c<=0;
                    g_loading<=0;g_commit<=0;g_request<=0;g_auto_cnt<=0;
                    buf_c_ready<=g_c_valid;write_buf_idx<=0;
                    c_post_hold<=C_POST_HOLD_CYCLES[23:0];
                end
            end

            if(fire_trigger && emerg_active && !emerg_pending)begin
                buf_c_ready<=0;g_auto_cnt<=0;
            end

            // Full-screen gallery: load hidden C / spare A-B, then swap at VS.
            // Never target saved_disp_buf or audio slot 3.
            if(emerg_active && !emerg_pending && !emerg_req_pulse)begin
                if(sd_frozen && scan_done && sd_init_done)sd_frozen<=0;
                if(g_commit && vs_rise && !fire_trigger && (video_continuation?(video_age>=5):(!buf_c_ready || (fade_out_req && fade_swap_ok))))begin
                    if(!fire_force || g_load_idx==fire_target)begin
                        disp_buf_idx<=pending_buf_idx;g_idx<=g_load_idx;g_last_idx<=g_load_idx;buf_c_ready<=1;
                    end
                    g_commit<=0;fade_out_req<=0;video_age<=0;
                    if(!video_continuation)g_auto_cnt<=0;
                end
                if(!fire_force && ctrl_auto && g_count>1)begin g_auto<=~g_auto;g_auto_cnt<=0;end
                // Accept navigation even while a poster is loading/awaiting VS.
                // Accumulate a desired index; do not discard a short K3 press.
                if(fire_force && !g_request && !g_loading && !g_commit && !load_busy &&
                   g_photo_count>fire_target && (!buf_c_ready || g_idx!=fire_target))begin
                    g_wanted<=fire_target;g_request<=1;g_auto_cnt<=0;
                end else if(!fire_force && (ctrl_next || ctrl_prev) && g_count>0)begin
                    g_wanted<=gallery_step(g_request?g_wanted:(g_loading || g_commit)?g_load_idx:g_idx,g_count,ctrl_prev);
                    g_request<=1;g_auto_cnt<=0;
                end else if(!fire_force && (g_video_selected || (!load_busy && !g_commit)) && !g_request && g_count>1 && g_auto)begin
                    if(g_auto_tick_q)begin
                        g_wanted<=gallery_step(g_idx,g_count,1'b0);g_request<=1;g_auto_cnt<=0;
                    end else g_auto_cnt<=g_auto_cnt+1'b1;
                end
                if(g_request && !ctrl_next && !ctrl_prev && g_count>0 && (!fire_force || g_photo_count>fire_target) && scan_done && bmp_ready && !load_abort_pulse &&
                   !load_busy && !g_commit && !wav_load_busy && !wav_phase2_busy &&
                   !sd_frozen && c_post_hold==0)begin
                    g_request<=0;g_loading<=1;g_load_idx<=fire_force?fire_target:g_wanted;load_frame<=0;video_continuation<=0;
                    load_sector<=gallery_sector(fire_force?fire_target:g_wanted);pending_buf_idx<=g_back;write_buf_idx<=g_back;
                    load_start_pulse<=1;load_busy<=1;source_done_seen<=0;load_saw_wrfin<=0;
                    c_pix_cnt<=0;c_drain_cnt<=0;
                    if(g_back==2'd2)g_c_valid<=0;
                end else if(!fire_force && g_video_selected && !g_request && !ctrl_next && !ctrl_prev && scan_done && bmp_ready && !load_abort_pulse &&
                   !load_busy && !g_commit && !wav_load_busy && !wav_phase2_busy && !sd_frozen && c_post_hold==0)begin
                    g_loading<=1;g_load_idx<=g_idx;load_sector<=pip_sector0;
                    load_frame<=video_next_frame;video_continuation<=1;
                    pending_buf_idx<=g_back;write_buf_idx<=g_back;
                    load_start_pulse<=1;load_busy<=1;source_done_seen<=0;load_saw_wrfin<=0;
                    c_pix_cnt<=0;c_drain_cnt<=0;if(g_back==2'd2)g_c_valid<=0;
                end
            end

            // ---- Buffer C: solid scrub first, then 5th BMP (pixel-count gated) ----
            if (want_c_bmp && !emerg_req_pulse && !load_abort_pulse && c_post_hold==0) begin
                write_buf_idx    <= 2'd2;
                load_sector      <= img_sector4;
                load_start_pulse <= 1'b1;
                load_busy        <= 1'b1;
                source_done_seen <= 1'b0;
                load_saw_wrfin   <= 1'b0;
                c_src_done       <= 1'b0;
                c_pix_cnt        <= 24'd0;
                c_drain_cnt      <= 32'd0;
                loading_buf_c    <= 1'b1;
                buf_c_bmp_tried  <= 1'b1;
            end else begin
            case (fill_state)
                FILL_IDLE: begin
                    fill_write_req <= 1'b0;
                    fill_watch     <= 32'd0;
                    // Solid scrub disabled (want_c_scrub=0); kept for emerg abort paths.
                end
                FILL_REQ: begin
                    fill_write_req <= 1'b1;
                    fill_watch     <= fill_watch + 32'd1;
                    if (emerg_active) begin
                        fill_write_req <= 1'b0;
                        fill_state     <= FILL_IDLE;
                        write_buf_idx  <= 2'd0;
                        wr_kill_tgl    <= ~wr_kill_tgl;
                        buf_c_give_up  <= 1'b1;
                        c_post_hold    <= C_POST_HOLD_CYCLES[23:0];
                    end else if (fill_watch >= LOAD_TIMEOUT_WR - 1) begin
                        fill_write_req <= 1'b0;
                        fill_state     <= FILL_IDLE;
                        buf_c_give_up  <= 1'b1;
                        write_buf_idx  <= 2'd0;
                        wr_kill_tgl    <= ~wr_kill_tgl;
                        c_post_hold    <= C_POST_HOLD_CYCLES[23:0];
                    end else if (write_req_ack) begin
                        fill_write_req <= 1'b0;
                        fill_state     <= FILL_STREAM;
                    end
                end
                FILL_STREAM: begin
                    fill_watch <= fill_watch + 32'd1;
                    if (emerg_active) begin
                        fill_write_en  <= 1'b0;
                        fill_write_req <= 1'b0;
                        fill_state     <= FILL_IDLE;
                        write_buf_idx  <= 2'd0;
                        wr_kill_tgl    <= ~wr_kill_tgl;
                        buf_c_give_up  <= 1'b1;
                        c_post_hold    <= C_POST_HOLD_CYCLES[23:0];
                    end else if (fill_watch >= LOAD_TIMEOUT_WR - 1) begin
                        fill_write_en  <= 1'b0;
                        fill_write_req <= 1'b0;
                        fill_state     <= FILL_IDLE;
                        buf_c_give_up  <= 1'b1;
                        write_buf_idx  <= 2'd0;
                        wr_kill_tgl    <= ~wr_kill_tgl;
                        c_post_hold    <= C_POST_HOLD_CYCLES[23:0];
                    end else begin
                        fill_div <= fill_div + 2'd1;
                        if (fill_div == 2'd0) begin
                            fill_write_en <= 1'b1;
                            fill_cnt      <= fill_cnt + 24'd1;
                        end
                        // Scrub only — do NOT set buf_c_ready (BMP comes next).
                        // write_len=FRAME_PIXELS so finish means a full red frame.
                        if (write_finish_pulse) begin
                            fill_state     <= FILL_IDLE;
                            buf_c_scrubbed <= 1'b1;
                            write_buf_idx  <= 2'd0;
                            c_post_hold    <= C_POST_HOLD_CYCLES[23:0];
                        end
                    end
                end
                default: fill_state <= FILL_IDLE;
            endcase
            end

            if (load_fail) begin
                load_busy        <= 1'b0;
                source_done_seen <= 1'b0;
                c_src_done       <= 1'b0;
                // A parser error can stop pixels after the writer accepted a
                // frame. Cancel that partial session before retrying; otherwise
                // CHECK_FIFO waits forever for the old frame's missing pixels.
                // Watchdog/emergency cancellation already clears load_busy, so
                // its delayed load_fail must not toggle the kill a second time.
                if(load_busy && !g_loading && !loading_buf_c &&
                   !emerg_active && !emerg_pending) begin
                    wr_kill_tgl<=~wr_kill_tgl;
                    load_abort_pulse<=1'b1;
                    load_saw_wrfin<=1'b0;
                    disp_commit_pending<=1'b0;
                    fade_out_req<=1'b0;
                    c_post_hold<=(C_POST_HOLD_CYCLES<1)?1:C_POST_HOLD_CYCLES;
                end
                // Keep the automatic target pending after a parser/read failure.
                // A partial back page is never committed; retry once ready again.
                if(train_busy && !g_loading && !loading_buf_c && !emerg_active && !emerg_pending) begin
                    requested_media<=load_idx;
                    next_req_pending<=1'b1;
                end
                if(g_loading)begin
                    g_loading<=0;g_request<=0;g_commit<=0;g_auto<=0;
                    c_post_hold<=C_POST_HOLD_CYCLES[23:0];
                    wr_kill_tgl<=~wr_kill_tgl;
                end
                // A spontaneous parser failure can also interrupt background
                // C preload. Retire its writer now; after a watchdog/abort,
                // load_busy is already clear, so its delayed error cannot
                // toggle the cross-clock cancellation a second time.
                if (loading_buf_c) begin
                    if(load_busy)begin
                        wr_kill_tgl<=~wr_kill_tgl;
                        load_abort_pulse<=1'b1;
                        load_saw_wrfin<=1'b0;
                    end
                    loading_buf_c <= 1'b0;
                    buf_c_give_up <= 1'b1; // never promote partial C
                    write_buf_idx <= 2'd0;
                    c_post_hold   <= C_POST_HOLD_CYCLES[23:0];
                end
            end

            // Safety: never leave write target parked on C after the C session ends
            // (widx2 pauses audio steal → U000 with D7 / good poster).
            if (!load_busy && !loading_buf_c && !wav_load_busy && !fill_active &&
                (write_buf_idx == 2'd2))
                write_buf_idx <= 2'd0;

            if (wav_scan_done_w)
                wav_phase2_busy <= 1'b0;

            wav_wr_req <= 1'b0;

            if(!emerg_active && !emerg_pending && !emerg_req_pulse)begin
            if (wav_load_busy) begin
                write_buf_idx <= 2'd3;
                // Latch finish for the whole WAV session (may arrive before wav_load_done).
                if (write_finish_pulse)
                    wav_saw_wrfin <= 1'b1;
                if (wav_session_timeout) begin
                    wav_load_busy        <= 1'b0;
                    wav_drain            <= 20'd0;
                    wav_await_wrfin      <= 1'b0;
                    wav_await_cnt        <= 32'd0;
                    wav_wr_req           <= 1'b0;
                    wav_ph               <= WAV_PH_IDLE;
                    wav_watch            <= 32'd0;
                    wav_ready            <= 1'b0;
                    wav_load_start_pulse <= 1'b0;
                    wav_saw_wrfin        <= 1'b0;
                    // Stop the physical partial write and the parser together.
                    // A fresh card reset also ends any unreturned sector read;
                    // the existing warm path keeps the displayed front frame.
                    wr_kill_tgl          <= ~wr_kill_tgl;
                    load_abort_pulse     <= 1'b1;
                    sd_bus_rst           <= 1'b1;
                    heal_timer           <= 32'd0;
                    write_buf_idx        <= 2'd0;
                end else
                    wav_watch <= wav_watch + 32'd1;
            end else
                wav_watch <= 32'd0;

            // Deadline cancellation has priority over every phase below,
            // including a late ACK/finish on the exact expiry clock.
            if(!wav_session_timeout)begin
            if (wav_load_busy && (wav_ph == WAV_PH_IDLE) && flash_cap_rdy &&
                !wav_ready && !wav_await_wrfin && (wav_drain == 20'd0)) begin
                wav_ph <= WAV_PH_REQ;
            end

            if (wav_ph == WAV_PH_REQ) begin
                wav_wr_req <= 1'b1;
                write_buf_idx <= 2'd3;
                if (write_req_ack) begin
                    wav_wr_req           <= 1'b0;
                    wav_load_start_pulse <= 1'b1;
                    wav_ph               <= WAV_PH_RUN;
                    wav_saw_wrfin        <= 1'b0; // arm sticky finish from this write session
                end
            end else if (wav_ph == WAV_PH_RUN) begin
                // Parser returns IDLE with wav_load_done on the same clock.
                // Re-kicking there creates an unowned second PCM transaction.
                if (state_code == 4'd1 && !wav_load_done)
                    wav_load_start_pulse <= 1'b1;
                // TF done: wait for SDRAM write_finish (already latched) or 200ms timeout.
                // Do NOT clear wav_saw_wrfin here — finish often beats wav_load_done.
                if (wav_load_done) begin
                    wav_ph          <= WAV_PH_IDLE;
                    wav_ok_hold     <= wav_load_ok;
                    wav_drain       <= 20'd0;
                    wav_await_cnt   <= 32'd0;
                    if (wav_saw_wrfin || write_finish_pulse) begin
                        wav_await_wrfin <= 1'b0;
                        wav_load_busy   <= 1'b0;
                        wav_ready       <= wav_load_ok;
                        write_buf_idx   <= 2'd0;
                        if (wav_load_ok)
                            wav_playing_idx <= img_idx_r;
                    end else
                        wav_await_wrfin <= 1'b1;
                end
            end else if (wav_await_wrfin) begin
                write_buf_idx <= 2'd3;
                wav_load_busy <= 1'b1;
                wav_await_cnt <= wav_await_cnt + 32'd1;
                if (wav_saw_wrfin || write_finish_pulse) begin
                    wav_await_wrfin <= 1'b0;
                    wav_await_cnt   <= 32'd0;
                    wav_load_busy   <= 1'b0;
                    wav_ready       <= wav_ok_hold;
                    write_buf_idx   <= 2'd0;
                    if (wav_ok_hold)
                        wav_playing_idx <= img_idx_r;
                end else if (wav_await_cnt >= WAV_AWAIT_TIMEOUT - 1) begin
                    // Fail closed: never claim ready with a half-written AUDIO_LEN
                    // (old 200ms timeout → wr_kill + D7 + noise / C start too early).
                    wr_kill_tgl     <= ~wr_kill_tgl;
                    wav_await_wrfin <= 1'b0;
                    wav_await_cnt   <= 32'd0;
                    wav_load_busy   <= 1'b0;
                    wav_ready       <= 1'b0;
                    write_buf_idx   <= 2'd0;
                end
            end else if (wav_drain != 20'd0) begin
                write_buf_idx <= 2'd3;
                wav_load_busy <= 1'b1;
                if (wav_drain == 20'd1) begin
                    wav_drain     <= 20'd0;
                    wav_load_busy <= 1'b0;
                end else
                    wav_drain <= wav_drain - 20'd1;
            end else if (first_image_committed && scan_done && bmp_ready && !load_busy &&
                         !wav_load_busy && !wav_await_wrfin && !sd_frozen && !wav_phase2_busy &&
                         !loading_buf_c && !want_c_bmp && !want_c_scrub &&
                         !emerg_active && !emerg_pending &&
                         !heal_after_emerg && !fade_out_req) begin
                // A reset or emergency can interrupt a resumed directory scan.
                // Re-arm incomplete discovery even when its old kick survived;
                // only a completed catalog can establish that audio is absent.
                if (!wav_phase2_kicked || (!wav_ready && !wav_catalog_complete && wav_found_count<3'd4)) begin
                    wav_phase2_kicked <= 1'b1;
                    if (!wav_catalog_complete && (wav_found_count < 3'd4)) begin
                        wav_scan_start_pulse <= 1'b1;
                        wav_phase2_busy      <= 1'b1;
                    end
                end else if ((wav_found_count != 3'd0) && !wav_ready) begin
                    if (flash_slot_ok[img_idx_r[1:0]]) begin
                        wav_ready       <= 1'b1;
                        wav_playing_idx <= img_idx_r;
                    end else begin
                        write_buf_idx        <= 2'd3;
                        wav_wr_req           <= 1'b1;
                        wav_ph               <= WAV_PH_REQ;
                        wav_load_busy        <= 1'b1;
                        wav_await_wrfin      <= 1'b0;
                        wav_saw_wrfin        <= 1'b0;
                        wav_await_cnt        <= 32'd0;
                        wav_watch            <= 32'd0;
                        wav_ready            <= 1'b0;
                        wav_load_sector_r    <= wav_sector_lut(img_idx_r);
                        wav_pcm_base_r       <= 20'd0;
                    end
                end
            end

            end // no expired WAV session may re-launch or become ready
            end // normal audio loader paused/cancelled during emergency

            // ---- scan / carousel (Lv.0); blocked in Lv.2 / frozen / filling / wav load ----
            if (!scan_kicked && bmp_ready && !wav_load_busy) begin
                scan_start_pulse <= 1'b1;
                scan_kicked      <= 1'b1;
                img_found_count  <= 4'd0;
                wav_found_count  <= 3'd0;
                wav_load_idx     <= 2'd0;
                wav_load_busy    <= 1'b0;
                wav_drain        <= 20'd0;
                wav_await_wrfin  <= 1'b0;
                wav_ph           <= WAV_PH_IDLE;
                wav_phase2_busy  <= 1'b0;
                load_busy        <= 1'b0;
                source_done_seen <= 1'b0;
                loading_buf_c    <= 1'b0;
                if (!display_valid) begin
                    // cold boot / blank: full reset including audio
                    wav_ready             <= 1'b0;
                    wav_phase2_kicked     <= 1'b0;
                    next_req_pending      <= 1'b0;
                    first_image_committed <= 1'b0;
                    auto_play_en          <= 1'b0;
                    auto_cnt              <= 32'd0;
                    img_idx_r             <= 2'd0;
                    load_idx              <= 2'd0;
                    pending_buf_idx       <= 2'd0;
                    write_buf_idx         <= 2'd0;
                    disp_buf_idx          <= 2'd0;
                    disp_commit_pending   <= 1'b0;
                    fade_out_req          <= 1'b0;
                    buf_c_ready           <= 1'b0;
                    buf_c_bmp_tried       <= 1'b0;
                    buf_c_scrubbed        <= 1'b0;
                    buf_c_give_up         <= 1'b0;
                    loading_buf_c         <= 1'b0;
                    c_src_done            <= 1'b0;
                    c_audio_quiet         <= 1'b0;
                    c_quiet_cnt           <= 32'd0;
                    c_post_hold           <= 24'd0;
                end else begin
                    // heal rescan while picture still up: keep PCM + pending next
                    wav_phase2_kicked <= 1'b1;
                end
            end else if (!emerg_active && !emerg_pending && !emerg_req_pulse) begin


                if (sd_frozen && scan_done && (img_found_count != 3'd0) && sd_init_done)
                    sd_frozen <= 1'b0;

                if (!train_busy && (key_next_any || ctrl_prev) && !key3_held && scan_done && (img_found_count > 3'd0) && sd_init_done) begin
                    next_req_pending <= 1'b1;
                    requested_media<=playlist_step(request_base,ctrl_prev);
                    prev_request<=ctrl_prev;
                    sd_frozen        <= 1'b0;
                end

                if (!train_busy && key_auto_press && !key3_held && scan_done && media_multiple && sd_init_done) begin
                    auto_play_en <= ~auto_play_en;
                    auto_cnt     <= 32'd0;
                    sd_frozen    <= 1'b0;
                end

                if (!sd_frozen && !fill_active && !wav_load_busy && !loading_buf_c &&
                    !want_c_bmp && !want_c_scrub && (c_post_hold == 24'd0)) begin

                    if (scan_done && auto_play_en && display_valid && (!load_busy || img_idx_r[2]) && (!disp_commit_pending || video_continuation) && !fade_out_req && first_image_committed && media_multiple) begin
                        if (auto_tick) begin auto_cnt<=auto_cnt;prev_request<=0;end
                        else
                            auto_cnt <= auto_cnt + 32'd1;
                    end else begin
                        auto_cnt <= 32'd0;
                    end

                    if (scan_done && !first_image_committed && bmp_ready && !load_busy && !disp_commit_pending && !fade_out_req && (img_found_count != 3'd0)) begin
                        load_frame<=0;video_continuation<=0;
                        load_idx         <= 2'd0;
                        load_sector      <= img_sector0;
                        pending_buf_idx  <= 2'd0;
                        write_buf_idx    <= 2'd0;
                        load_start_pulse <= 1'b1;
                        load_busy        <= 1'b1;
                        source_done_seen <= 1'b0;
                        load_saw_wrfin   <= 1'b0;
                        next_req_pending <= 1'b0;
                        auto_cnt         <= 32'd0;
                    end
                    else if (heal_after_emerg && scan_done && bmp_ready && display_valid &&
                             !load_busy && !disp_commit_pending && !fade_out_req &&
                             (img_found_count != 3'd0)) begin
                        // Same index into the other A/B buffer, then fade+swap
                        load_frame<=0;video_continuation<=0;
                        load_idx         <= img_idx_r;
                        load_sector      <= sector_lut(img_idx_r);
                        pending_buf_idx  <= next_buf_lut(disp_buf_idx, display_valid);
                        write_buf_idx    <= next_buf_lut(disp_buf_idx, display_valid);
                        load_start_pulse <= 1'b1;
                        load_busy        <= 1'b1;
                        source_done_seen <= 1'b0;
                        load_saw_wrfin   <= 1'b0;
                        heal_after_emerg <= 1'b0;
                        next_req_pending <= 1'b0;
                        auto_cnt         <= 32'd0;
                        fade_out_req     <= 1'b0;
                        // keep wav_ready — emerg path no longer sprays PCM into A/B
                    end
                    else if (scan_done && audio_settled && bmp_ready && display_valid && !load_busy && !disp_commit_pending && !fade_out_req && next_req_pending && (train_busy || (!ctrl_next && !ctrl_prev)) && (img_found_count != 3'd0)) begin
                        load_frame<=0;video_continuation<=0;
                        load_idx         <= requested_media;
                        load_sector      <= sector_lut(requested_media);
                        pending_buf_idx  <= next_buf_lut(disp_buf_idx, display_valid);
                        write_buf_idx    <= next_buf_lut(disp_buf_idx, display_valid);
                        load_start_pulse <= 1'b1;
                        load_busy        <= 1'b1;
                        source_done_seen <= 1'b0;
                        load_saw_wrfin   <= 1'b0;
                        next_req_pending <= 1'b0;
                        auto_cnt         <= 32'd0;
                        // Fade while hidden buffer fills — covers SDRAM write wait
                        fade_out_req     <= 1'b0;
                    end
                    else if (scan_done && audio_settled && bmp_ready && display_valid && !load_busy && !disp_commit_pending && !fade_out_req && auto_play_en && auto_tick && media_multiple) begin
                        load_frame<=0;video_continuation<=0;
                        load_idx         <= next_index_limited(img_idx_r, img_found_count);
                        load_sector      <= sector_lut(next_index_limited(img_idx_r, img_found_count));
                        pending_buf_idx  <= next_buf_lut(disp_buf_idx, display_valid);
                        write_buf_idx    <= next_buf_lut(disp_buf_idx, display_valid);
                        load_start_pulse <= 1'b1;
                        load_busy        <= 1'b1;
                        source_done_seen <= 1'b0;
                        load_saw_wrfin   <= 1'b0;
                        auto_cnt         <= 32'd0;
                        fade_out_req     <= 1'b0;
                    end else if(scan_done && audio_settled && bmp_ready && display_valid && img_idx_r[2] &&
                                !load_busy && !disp_commit_pending && !fade_out_req && !next_req_pending &&
                                (train_busy ? (train_phase_wire==4'd2 && !train_request_pulse) : (!ctrl_next && !ctrl_prev)))begin
                        load_idx<=img_idx_r;load_sector<=sector_lut(img_idx_r);load_frame<=video_next_frame;video_continuation<=1;
                        pending_buf_idx<=next_buf_lut(disp_buf_idx,display_valid);write_buf_idx<=next_buf_lut(disp_buf_idx,display_valid);
                        load_start_pulse<=1;load_busy<=1;source_done_seen<=0;load_saw_wrfin<=0;
                    end
                end
            end
            if (train_start_pulse) begin
                train_saved_auto      <= auto_play_en;
                auto_play_en          <= 1'b0;
                auto_cnt              <= 32'd0;
                next_req_pending      <= 1'b0;
                prev_request          <= 1'b0;
                sd_frozen             <= 1'b0;
            end
            if (train_request_pulse) begin
                requested_media       <= train_request_media;
                next_req_pending      <= 1'b1;
                prev_request          <= 1'b0;
                sd_frozen             <= 1'b0;
            end
            if (train_finish_pulse) begin
                auto_play_en <= train_saved_auto;
                auto_cnt     <= 32'd0;
            end
        end
    end
end

bmp_read bmp_read_m0(
    .load_video(load_is_video),.load_video_clip(1'b0),.load_video_frame(load_frame),
    .load_video_size(pip_size0),.load_video_last(video_last),
    .normalize_pause(video_pause),.normalize_done(video_done),
    .pip_req(pip_read_req && pip_allowed),.pip_sector(pip_read_sector),
    .pip_ack(pip_read_ack),.pip_valid(pip_read_valid),.pip_end(pip_read_end),.pip_byte(pip_read_byte),
    .pip_bus_busy(pip_master_busy),.pip_catalog_valid(pip_catalog_valid),
    .pip_sector0(pip_sector0),.pip_sector1(pip_sector1),.pip_size0(pip_size0),.pip_size1(pip_size1),
    .clk                    (clk),
    .rst                    (sd_rst),
    .ready                  (bmp_ready_raw),

    .scan_start             (scan_start_pulse),
    .scan_start_sector      (SCAN_START_SECTOR),
    .scan_max_sector        (SCAN_MAX_SECTOR),
    .scan_target_count      (SCAN_TARGET_COUNT),
    .scan_done              (scan_done),
    .scan_found_valid       (scan_found_valid),
    .scan_found_sector      (scan_found_sector),
    .scan_found_total       (scan_found_total),
    .wav_found_valid        (wav_found_valid),
    .wav_found_sector       (wav_found_sector),
    .wav_found_total        (wav_found_total),
    .wav_scan_start         (wav_scan_start_pulse),
    .wav_resume_sector      (wav_resume_sector),
    .wav_scan_done          (wav_scan_done_w),
    .wav_catalog_complete   (wav_catalog_complete),

    .load_start             (load_start_pulse),
    .load_abort             (load_abort_pulse),
    .load_sector            (load_sector),

    .wav_load_start         (wav_load_start_pulse),
    .wav_spi_hold           (wav_spi_hold),
    .wav_load_sector        (wav_load_sector_r),
    .wav_pcm_base           (wav_pcm_base_r),
    .wav_pcm_we             (wav_pcm_we_w),
    .wav_pcm_addr           (wav_pcm_addr_w),
    .wav_pcm_data           (wav_pcm_data_w),
    .wav_load_done          (wav_load_done),
    .wav_load_ok            (wav_load_ok),.wav_is_rail(rail_audio_present),
    .load_fail              (load_fail),

    .sd_init_done           (sd_init_done),
    .state_code             (state_code),
    .bmp_width              (bmp_width),
    .bmp_height             (bmp_height),
    .write_req              (bmp_write_req),
    .write_req_ack          (write_req_ack),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_end        (sd_sec_read_end),
    .source_width(source_width),.source_height(source_height),
    .bmp_data_wr_en         (raw_bmp_valid),
    .bmp_data               (raw_bmp_pixel)
);

bmp_normalize u_normalize(.clk(clk),.rst(sd_rst|load_abort_pulse|bmp_write_req),
    .valid(raw_bmp_valid && !load_video_active),.pixel(raw_bmp_pixel),.width(source_width),.height(source_height),
    .out_valid(photo_pixel_valid),.out_pixel(photo_pixel),.overrun(normalize_overrun));
exhibit_media_bilinear u_video_reconstruct(.clk(clk),.rst(sd_rst|load_abort_pulse|bmp_write_req),
    .valid(raw_bmp_valid && load_video_active),.out_ready(write_ready),.pixel(raw_bmp_pixel),.width(source_width),.height(source_height),
    .out_valid(video_pixel_valid),.out_pixel(video_pixel),.pause_source(video_pause),.done(video_done),.overrun(video_overrun));
assign bmp_data_wr_en=load_video_active?video_pixel_valid:photo_pixel_valid;
assign bmp_data=load_video_active?video_pixel:photo_pixel;

sd_card_top sd_card_top_m0(
    .clk                    (clk),
    .rst                    (sd_rst),
    .SD_nCS                 (SD_nCS),
    .SD_DCLK                (SD_DCLK),
    .SD_MOSI                (SD_MOSI),
    .SD_MISO                (SD_MISO),
    .sd_init_done           (sd_init_done),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_end        (sd_sec_read_end),
    .sd_sec_write           (1'b0),
    .sd_sec_write_addr      (32'd0),
    .sd_sec_write_data      (),
    .sd_sec_write_data_req  (),
    .sd_sec_write_end       ()
);

endmodule

module key_press_debounce #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer DEBOUNCE_MS = 20
)(
    input  wire clk,
    input  wire rst,
    input  wire button_in,
    output reg  press_pulse
);

localparam integer DEBOUNCE_CYCLES = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;

reg button_sync0;
reg button_sync1;
reg button_stable;
reg [31:0] cnt;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        button_sync0  <= 1'b1;
        button_sync1  <= 1'b1;
        button_stable <= 1'b1;
        cnt           <= 32'd0;
        press_pulse   <= 1'b0;
    end else begin
        button_sync0 <= button_in;
        button_sync1 <= button_sync0;
        press_pulse  <= 1'b0;

        if (button_sync1 == button_stable) begin
            cnt <= 32'd0;
        end else begin
            if (cnt >= DEBOUNCE_CYCLES - 1) begin
                if (button_stable && !button_sync1)
                    press_pulse <= 1'b1;
                button_stable <= button_sync1;
                cnt           <= 32'd0;
            end else begin
                cnt <= cnt + 32'd1;
            end
        end
    end
end

endmodule

// Active-low: short pulse on confirmed release if hold < LONG; long pulse once at LONG.
// Release must stay high for RELEASE_MS so bounce during a hold does not look like a tap.
module key_short_long #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer DEBOUNCE_MS = 20,
    parameter integer RELEASE_MS  = 50,
    parameter integer LONG_MS     = 1000
)(
    input  wire clk,
    input  wire rst,
    input  wire button_in,
    output reg  held,
    output reg  short_pulse,
    output reg  long_pulse
);
localparam integer DEBOUNCE_CYCLES = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;
localparam integer RELEASE_CYCLES  = (CLK_FREQ_HZ / 1000) * RELEASE_MS;
localparam integer LONG_CYCLES     = (CLK_FREQ_HZ / 1000) * LONG_MS;

reg        sync0, sync1;
reg        raw_low;
reg [31:0] db_cnt;
reg [31:0] hold_cnt;
reg [31:0] rel_cnt;
reg        long_fired;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        sync0 <= 1'b1;
        sync1 <= 1'b1;
        raw_low <= 1'b0;
        db_cnt <= 32'd0;
        hold_cnt <= 32'd0;
        rel_cnt <= 32'd0;
        long_fired <= 1'b0;
        held <= 1'b0;
        short_pulse <= 1'b0;
        long_pulse <= 1'b0;
    end else begin
        sync0 <= button_in;
        sync1 <= sync0;
        short_pulse <= 1'b0;
        long_pulse  <= 1'b0;

        if (sync1 == (raw_low ? 1'b0 : 1'b1))
            db_cnt <= 32'd0;
        else if (db_cnt >= DEBOUNCE_CYCLES - 1) begin
            raw_low <= ~sync1; // sync1=0 => pressed
            db_cnt <= 32'd0;
        end else
            db_cnt <= db_cnt + 32'd1;

        if (raw_low) begin
            rel_cnt <= 32'd0;
            held <= 1'b1;
            if (!long_fired) begin
                if (hold_cnt >= LONG_CYCLES - 1) begin
                    long_pulse <= 1'b1;
                    long_fired <= 1'b1;
                end else begin
                    hold_cnt <= hold_cnt + 32'd1;
                end
            end
        end else begin
            if (held) begin
                if (rel_cnt >= RELEASE_CYCLES - 1) begin
                    if (!long_fired && (hold_cnt > DEBOUNCE_CYCLES))
                        short_pulse <= 1'b1;
                    held <= 1'b0;
                    hold_cnt <= 32'd0;
                    long_fired <= 1'b0;
                    rel_cnt <= 32'd0;
                end else begin
                    rel_cnt <= rel_cnt + 32'd1;
                end
            end else begin
                hold_cnt <= 32'd0;
                long_fired <= 1'b0;
                rel_cnt <= 32'd0;
            end
        end
    end
end

endmodule

// Accepted 24-bit bottom-up BMP sizes have no row padding: 320/640/800.
// Normalize to 640x480 entirely in the FPGA before the existing flipped writer.
module bmp_normalize (
    input wire clk,rst,valid,
    input wire [23:0] pixel,
    input wire [15:0] width,height,
    output reg out_valid,output reg [23:0] out_pixel,
    output reg overrun
);
reg [9:0] in_x,in_y;
reg bank;
reg [2:0] row_mod;
reg pending,emit_bank,emit_repeat;
reg [9:0] out_x;
reg [11:0] source_q;
reg [3:0] pace;
wire [9:0] source_x=width==320 ? {1'b0,out_x[9:1]} : width==800 ? source_q[11:2] : out_x;
wire [10:0] wa={bank,in_x};
wire [10:0] ra={emit_bank,source_x};
wire [15:0] q;
wire [15:0] p565={pixel[23:19],pixel[15:10],pixel[7:3]};
EG_LOGIC_BRAM #(.DATA_WIDTH_A(16),.DATA_WIDTH_B(16),.ADDR_WIDTH_A(11),.ADDR_WIDTH_B(11),
    .DATA_DEPTH_A(2048),.DATA_DEPTH_B(2048),.MODE("DP"),.REGMODE_A("NOREG"),
    .REGMODE_B("NOREG"),.WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),
    .RESETMODE("SYNC"),.IMPLEMENT("9K"),.INIT_FILE("NONE"),.FILL_ALL("NONE")) lines(
    .dia(p565),.dib(16'd0),.addra(wa),.addrb(ra),.cea(1'b1),.ceb(1'b1),
    .ocea(1'b0),.oceb(1'b0),.clka(clk),.clkb(clk),.wea(valid),.web(1'b0),
    .bea(1'b0),.beb(1'b0),.rsta(1'b0),.rstb(1'b0),.doa(),.dob(q));
always @(posedge clk or posedge rst) begin
    if(rst) begin
        in_x<=0;in_y<=0;bank<=0;row_mod<=0;pending<=0;emit_bank<=0;emit_repeat<=0;
        out_x<=0;source_q<=0;pace<=0;out_valid<=0;out_pixel<=0;overrun<=0;
    end else begin
        out_valid<=0;
        if(valid) begin
            if(in_x==width-1) begin
                in_x<=0;bank<=~bank;in_y<=in_y+1;
                if(width==800) row_mod<=row_mod==4?0:row_mod+1;
                // floor(y * 5/4): source rows 0,1,2,3,5,6,7,8,...
                if(width!=800 || row_mod!=4) begin
                    if(pending) overrun<=1;
                    else begin pending<=1;emit_bank<=bank;emit_repeat<=0;out_x<=0;source_q<=0;pace<=0;end
                end
            end else in_x<=in_x+1;
        end
        if(pending) begin
            pace<=pace+1;
            if(pace==15) begin
                out_valid<=1;
                out_pixel<={q[15:11],q[15:13],q[10:5],q[10:9],q[4:0],q[4:2]};
                if(out_x==639) begin
                    out_x<=0;source_q<=0;
                    if(width==320 && !emit_repeat)emit_repeat<=1;
                    else begin pending<=0;emit_repeat<=0;end
                end else begin out_x<=out_x+1;source_q<=source_q+5;end
            end
        end
    end
end
endmodule

// Sensor-driven railway demonstration sequencer. Audio is intentionally left
// outside this module: speech_event is a one-cycle hook for a later audio player.
module rail_train_flow #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer ANNOUNCE_FRAMES = 180,
    parameter         IR_ACTIVE_LOW = 1'b1
)(
    input clk,rst,
    input ir_sensor,
    input frame_tick,
    input system_ready,
    input emergency,
    input video_available,
    input video_done,
    input audio_available,audio_done,
    input media_commit,
    input [2:0] committed_media,
    input [2:0] photo_count,
    output reg start_pulse,
    output reg request_pulse,
    output reg [2:0] request_media,
    output reg [2:0] speech_event,
    output reg [2:0] journey_stage,
    output [3:0] phase,
    output busy,
    output reg finish_pulse
);
    localparam integer IR_DEBOUNCE_CYCLES = (CLK_FREQ_HZ/50 < 1) ? 1 : CLK_FREQ_HZ/50;
    localparam integer IR_COUNT_W = $clog2(IR_DEBOUNCE_CYCLES+1);
    localparam integer ANNOUNCE_COUNT_W = (ANNOUNCE_FRAMES < 2) ? 1 : $clog2(ANNOUNCE_FRAMES+1);
    localparam [3:0] S_IDLE=0, S_WAIT_VIDEO=1, S_VIDEO=2,
                     S_WAIT_ARRIVAL=3, S_ARRIVAL=4,
                     S_WAIT_BOARD=5, S_BOARD=6,
                     S_WAIT_PREP=7, S_PREP=8, S_WAIT_CLEAR=9,
                     S_WAIT_NEXT=10, S_NEXT=11, S_WAIT_VIDEO_AUDIO=12;
    reg [3:0] state;
    reg [2:0] ir_sync;
    reg ir_present;
    reg [IR_COUNT_W-1:0] ir_db_count;
    reg [ANNOUNCE_COUNT_W-1:0] announce_count;
    reg [2:0] stage_photo;
    reg emergency_d;
    reg use_audio,audio_completed;
    wire ir_raw = IR_ACTIVE_LOW ? ~ir_sync[2] : ir_sync[2];
    assign phase=state;
    assign busy=(state!=S_IDLE);

    function [2:0] next_photo;
        input [2:0] current;
        input [2:0] count;
        reg [2:0] usable;
        begin
            usable=(count>4)?3'd4:count;
            if(usable<=1) next_photo=0;
            else if(current+1'b1>=usable) next_photo=0;
            else next_photo=current+1'b1;
        end
    endfunction

    always @(posedge clk or posedge rst) begin
        if(rst) begin
            ir_sync<=3'b111;
            ir_present<=1'b0;
            ir_db_count<=0;
            announce_count<=0;
            stage_photo<=0;
            emergency_d<=0;use_audio<=0;audio_completed<=0;
            state<=S_IDLE;
            start_pulse<=0;
            request_pulse<=0;
            request_media<=0;
            speech_event<=0;
            journey_stage<=0;
            finish_pulse<=0;
        end else begin
            ir_sync<={ir_sync[1:0],ir_sensor};
            emergency_d<=emergency;
            start_pulse<=0;
            request_pulse<=0;
            speech_event<=0;
            finish_pulse<=0;
            if(audio_done && state!=S_IDLE)audio_completed<=1;

            if(ir_raw==ir_present) ir_db_count<=0;
            else if(ir_db_count>=IR_DEBOUNCE_CYCLES-1) begin
                ir_present<=ir_raw;
                ir_db_count<=0;
            end else ir_db_count<=ir_db_count+1'b1;

            if(!emergency) begin
                // Emergency entry aborts outstanding media loads. Re-submit the
                // held target once on return; otherwise WAIT states can stall.
                if(emergency_d && (state==S_WAIT_VIDEO || state==S_WAIT_ARRIVAL ||
                   state==S_WAIT_BOARD || state==S_WAIT_PREP || state==S_WAIT_NEXT)) begin
                    request_pulse<=1;
                end else begin
                case(state)
                    S_IDLE: begin
                        announce_count<=0;
                        if(system_ready && ir_present) begin
                            start_pulse<=1;use_audio<=(audio_available===1'b1);audio_completed<=0;
                            journey_stage<=0;
                            stage_photo<=0;
                            if(video_available) begin
                                request_media<=3'd4;
                                request_pulse<=1;
                                state<=S_WAIT_VIDEO;
                            end else begin
                                request_media<=3'd0;
                                request_pulse<=1;
                                state<=S_WAIT_ARRIVAL;
                            end
                        end
                    end
                    S_WAIT_VIDEO: if(media_commit && committed_media==3'd4) begin
                        speech_event<=3'd1;audio_completed<=0;
                        journey_stage<=3'd1;
                        state<=S_VIDEO;
                    end
                    S_VIDEO: if(video_done) begin
                        if(!use_audio || audio_completed || audio_done)begin
                            request_media<=3'd0;request_pulse<=1;state<=S_WAIT_ARRIVAL;
                        end else state<=S_WAIT_VIDEO_AUDIO;
                    end
                    S_WAIT_VIDEO_AUDIO: if(audio_completed || audio_done)begin
                        request_media<=3'd0;request_pulse<=1;state<=S_WAIT_ARRIVAL;
                    end
                    S_WAIT_ARRIVAL: if(media_commit && committed_media==3'd0) begin
                        speech_event<=3'd2;audio_completed<=0;
                        journey_stage<=3'd2;
                        announce_count<=0;
                        state<=S_ARRIVAL;
                    end
                    S_ARRIVAL: if(use_audio ? (audio_completed || audio_done) : (frame_tick && announce_count>=ANNOUNCE_FRAMES-1)) begin
                        announce_count<=0;
                        stage_photo<=next_photo(3'd0,photo_count);
                        request_media<=next_photo(3'd0,photo_count);
                        request_pulse<=1;
                        state<=S_WAIT_BOARD;
                    end else if(frame_tick) announce_count<=announce_count+1'b1;
                    S_WAIT_BOARD: if(media_commit && committed_media==stage_photo) begin
                        speech_event<=3'd3;audio_completed<=0;
                        journey_stage<=3'd3;
                        announce_count<=0;
                        state<=S_BOARD;
                    end
                    S_BOARD: if(use_audio ? (audio_completed || audio_done) : (frame_tick && announce_count>=ANNOUNCE_FRAMES-1)) begin
                        announce_count<=0;
                        stage_photo<=next_photo(stage_photo,photo_count);
                        request_media<=next_photo(stage_photo,photo_count);
                        request_pulse<=1;
                        state<=S_WAIT_PREP;
                    end else if(frame_tick) announce_count<=announce_count+1'b1;
                    S_WAIT_PREP: if(media_commit && committed_media==stage_photo) begin
                        speech_event<=3'd4;audio_completed<=0;
                        journey_stage<=3'd4;
                        announce_count<=0;
                        state<=S_PREP;
                    end
                    S_PREP: if(use_audio ? (audio_completed || audio_done) : (frame_tick && announce_count>=ANNOUNCE_FRAMES-1)) begin
                        announce_count<=0;
                        state<=S_WAIT_CLEAR;
                    end else if(frame_tick) announce_count<=announce_count+1'b1;
                    S_WAIT_CLEAR: if(!ir_present) begin
                        stage_photo<=next_photo(stage_photo,photo_count);
                        request_media<=next_photo(stage_photo,photo_count);
                        request_pulse<=1;
                        state<=S_WAIT_NEXT;
                    end
                    S_WAIT_NEXT: if(media_commit && committed_media==stage_photo) begin
                        speech_event<=3'd5;audio_completed<=0;
                        journey_stage<=3'd5;
                        announce_count<=0;
                        state<=S_NEXT;
                    end
                    S_NEXT: if(use_audio ? (audio_completed || audio_done) : (frame_tick && announce_count>=ANNOUNCE_FRAMES-1)) begin
                        announce_count<=0;
                        state<=S_IDLE;
                        finish_pulse<=1;
                    end else if(frame_tick) announce_count<=announce_count+1'b1;
                    default: state<=S_IDLE;
                endcase
                end
            end
        end
    end
endmodule
