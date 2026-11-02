
module top(
    inout wire dht_data, // DHT11 DATA: J1-1 / D14, open drain
    input                       clk,
    input                       rst_n,
    input                       key1,           // 手动下一张  B2 / 丝印K2
    input                       key2,           // 自动播放    C1 / 丝印K4
    input [3:0]                 sw, // SW4..SW1: C5/C6/C7/C8
    input                       key3,           // 短按周期 / 长按应急  B1
    input                       emerg_n,        // 外部应急（低有效），J4/L14，内部上拉
    input                       ir_sensor,      // 红外 OUT，J1-2 / FPGA G11 / GPIO1

    // EC11 rotary on expansion P1IO (not VGA): idle high, SW active-low
    input                       enc_clk,        // module CLK  → H13 (P1IO[4])
    input                       enc_dt,         // module DT   → H14 (P1IO[5])
    input                       enc_sw,         // module SW   → J14 (P1IO[6])

    output [5:0]                seg_sel,
    output [7:0]                seg_data,

    // Slice 7: scope hooks (J4) — software lat_ms on 7-seg/OSD until scope returns
    output                      scope_trig,     // J13: emerg request / active
    output                      scope_vsync,    // F13: video vsync

    // Slice 9: USB-TTL on VGA header (away from HDMI Type-C / CH340)
    output                      uart_tx,        // E7 VGA_HS  -> USB-TTL RXD
    input                       uart_rx,        // F7 VGA_VS  <- USB-TTL TXD

    // HDMI TMDS
    output                      HDMI_CLK_P,
    output                      HDMI_D2_P,
    output                      HDMI_D1_P,
    output                      HDMI_D0_P,

    // HDMI DDC
    output                      HDMI_DDC_SCL,
    inout                       HDMI_DDC_SDA,

    // TF card SPI
    output                      sd_ncs,
    output                      sd_dclk,
    output                      sd_mosi,
    input                       sd_miso,

    output                      spk,            // H11 onboard piezo, Lv.2 only

    // U33 W25X64: CS P8, SDO N8, WP P9, HOLD R9, SCLK M9, SDI P7
    output                      flash_cs,
    input                       flash_sdo,
    output                      flash_wp,
    output                      flash_hold,
    output                      flash_sclk,
    output                      flash_sdi
);

parameter MEM_DATA_BITS = 32;
parameter ADDR_BITS     = 21;
parameter BUSRT_BITS    = 10;
parameter FRAME_PIXELS  = 24'd307200;   // 640*480
parameter BUF0_ADDR     = 24'd0;
parameter BUF1_ADDR     = FRAME_PIXELS;
parameter BUF2_ADDR     = FRAME_PIXELS + FRAME_PIXELS; // Buffer C emergency
parameter AUDIO_BASE    = 24'd983040;   // after A/B/C plus 64K guard
parameter AUDIO_LEN     = 24'd960000;   // 20 s * 48 kHz mono

wire Sdr_init_done;
wire Sdr_init_ref_vld;
wire Sdr_busy;

wire sd_card_clk;
wire ext_mem_clk;
wire ext_mem_clk_sft;
wire video_clk;
wire hdmi_5x_clk;

wire hs;
wire vs;
wire de;

wire [23:0] vout_data_raw;
wire [23:0] vout_data_pre;
wire [23:0] vout_data_faded;
wire [23:0] vout_data_metro;
wire [23:0] vout_data_wave;
wire [23:0] vout_data_rail;
wire [23:0] vout_data_fps;
wire [23:0] vout_data_emerg;
wire        warn_lv1;
reg  [2:0]  warn_lv1_v_sync;
reg  [2:0]  pic_wr_v_sync;
wire [23:0] vout_data;
wire        display_valid;
wire        fade_out_req;
wire        fade_swap_ok_v;
reg  [2:0]  fade_swap_ok_sd_sync;
reg  [2:0]  fade_req_v_sync;
reg  [2:0]  display_valid_v_sync;
reg  [2:0]  emerg_active_v_sync;
reg  [2:0]  buf_c_ready_v_sync;
wire [7:0]  fade_alpha;
wire        fade_wipe_en;
wire [9:0]  fade_wipe_x;
wire [7:0]  fade_r;
wire [7:0]  fade_g;
wire [7:0]  fade_b;
wire [23:0] vout_data_wipe;
reg         de_d_wipe;
reg  [9:0]  pix_x_wipe;

wire [3:0]  state_code;
wire [3:0] media_boot_status_sd;
wire [3:0]  period_sec;
wire [1:0]  img_idx;
wire        emerg_active;
wire        emerg_trig;
wire [7:0]  lat_ms;
wire        sd_frozen;
wire        buf_c_ready;
wire [1:0]  c_stat;
wire        wav_pcm_we;
wire [19:0] wav_pcm_addr;
wire [15:0] wav_pcm_data;
wire        wav_ready;
wire        wav_load_busy;
wire        pic_wr_busy;
wire        wav_spi_hold;
wire        flash_cap_rdy;
wire        flash_idle;
wire [3:0]  flash_slot_ok;
wire [2:0]  wav_found_n;
wire [3:0]  wav_dbg;
wire [2:0]  bmp_found_n;
wire [6:0]  seg_data_0;
wire [6:0]  seg_data_period;
wire [6:0]  seg_data_lat_hi;
wire [6:0]  seg_data_lat_lo;
wire [6:0]  seg_data_vol;
wire [3:0] vol_lvl;
wire [3:0] volume_sd;
wire volume_gesture;
reg [4:0] uart_volume_request;
wire [4:0] uart_volume_sd;
wire [3:0]  vol_cmd;
wire        vol_cmd_stb;
wire [23:0] audio_left_raw;
wire [23:0] audio_right_raw;
wire [23:0] audio_left_data;
wire [23:0] audio_right_data;
reg  [1:0]  img_idx_amclk_d0;
reg  [1:0]  img_idx_amclk_d1;

wire        video_read_req;
wire        video_read_req_ack;
wire        video_read_en;
wire [31:0] video_read_data;

wire        sd_card_write_en;
wire [31:0] sd_card_write_data;
wire        sd_card_write_req;
wire        sd_card_write_req_ack;
wire        sd_wr_kill_tgl;
wire        frame_write_finish;
reg         frame_write_toggle_mem;

wire [1:0]  write_buf_idx;
wire [1:0]  disp_buf_idx;

wire App_rd_en;
wire [ADDR_BITS-1:0] App_rd_addr;
wire vid_rd_en;
wire [ADDR_BITS-1:0] vid_rd_addr;
wire vid_rd_busy;
wire vid_rd_active;
wire vid_rd_recover;
wire Sdr_rd_en;
wire [MEM_DATA_BITS-1:0] Sdr_rd_dout;
wire App_wr_en;
wire [ADDR_BITS-1:0] App_wr_addr;
wire [MEM_DATA_BITS-1:0] App_wr_din;
wire [3:0] App_wr_dm;
wire        wr_mem_busy;
reg         wr_mem_busy_q;

reg         aud_rd_en;
reg         aud_lock;
reg [4:0]   aud_issue;
reg [4:0]   aud_need;
reg [4:0]   aud_tail;
reg [17:0]  aud_arm;
reg [2:0]   wload_m, picw_m, bufc_m, emerg_m, widx2_m;
reg [9:0]   vid_os;
reg [5:0]   aud_gap;
reg [2:0]   de_m;
reg [ADDR_BITS-1:0] aud_rd_addr;
wire [15:0] pcm_bram_q;
wire [15:0] pcm_doa_unused;
reg [10:0]  pcm_wptr;
reg [10:0]  pcm_rptr;
reg [10:0]  pcm_wptr_g;
reg [10:0]  pcm_rptr_g;
reg [10:0]  pcm_wptr_g_m0, pcm_wptr_g_m1;
reg [10:0]  pcm_rptr_g_m0, pcm_rptr_g_m1;
reg [2:0]   wav_rdy_m;
reg [1:0]   wav_rdy_a;
reg         pcm_armed;
reg [15:0]  pcm_rd;
reg [7:0]   pcm_under;
reg [2:0]   emerg_a_sync;
reg [2:0] pcm_fault_a;
wire        emerg_a;
wire        pcm_pop;
wire        pcm_empty;
wire [10:0] pcm_wptr_s;
wire [10:0] pcm_rptr_s;
wire [10:0] pcm_wused;
wire [10:0] pcm_rused;
wire        pcm_wfull;
reg         pcm_has_space_q;
reg [5:0]   bgm_flush_m;
wire        pcm_rst_wr;

wire pip_mem_hold,pip_mem_owner,pip_mem_rd,pip_mem_wr;
wire [20:0] pip_mem_addr;
wire [31:0] pip_mem_wdata;
wire [15:0] pip_line_miss;
wire photo_wr_en;
wire [20:0] photo_wr_addr;
wire [31:0] photo_wr_din;
wire [3:0] photo_wr_dm;
wire pip_mem_free=Sdr_init_done && !Sdr_busy && !Sdr_rd_en &&
 !vid_rd_active && !wr_mem_busy && !photo_wr_en && !aud_hold_ext;
assign App_wr_en=photo_wr_en | pip_mem_wr;
assign App_wr_addr=pip_mem_wr?pip_mem_addr:photo_wr_addr;
assign App_wr_din=pip_mem_wr?pip_mem_wdata:photo_wr_din;
assign App_wr_dm=pip_mem_wr?4'b0000:photo_wr_dm;
assign App_rd_en   = vid_rd_en | aud_rd_en | pip_mem_rd;
assign App_rd_addr = pip_mem_rd?pip_mem_addr:vid_rd_en ? vid_rd_addr : aud_rd_addr;
wire        aud_hold = aud_lock || (aud_issue != 5'd0) || (aud_need != 5'd0) || (aud_tail != 5'd0) || aud_rd_en;
reg  [1:0]  aud_hold_d;
wire        aud_hold_ext = aud_hold || (aud_hold_d != 2'b00);
// Late PHY beats after steal must not count as pixels (aud_tail/hold still high).
reg vid_response_enable;
always @(posedge ext_mem_clk or posedge rst_all)begin
    if(rst_all)vid_response_enable<=0;
    else vid_response_enable<=!aud_hold;
end
wire vid_data=Sdr_rd_en && vid_response_enable && !aud_lock && !pip_mem_owner;
wire        aud_data = Sdr_rd_en && aud_lock && (aud_need != 5'd0);

assign pcm_wptr_s[10] = pcm_wptr_g_m1[10];
assign pcm_wptr_s[9]  = pcm_wptr_s[10] ^ pcm_wptr_g_m1[9];
assign pcm_wptr_s[8]  = pcm_wptr_s[9]  ^ pcm_wptr_g_m1[8];
assign pcm_wptr_s[7]  = pcm_wptr_s[8]  ^ pcm_wptr_g_m1[7];
assign pcm_wptr_s[6]  = pcm_wptr_s[7]  ^ pcm_wptr_g_m1[6];
assign pcm_wptr_s[5]  = pcm_wptr_s[6]  ^ pcm_wptr_g_m1[5];
assign pcm_wptr_s[4]  = pcm_wptr_s[5]  ^ pcm_wptr_g_m1[4];
assign pcm_wptr_s[3]  = pcm_wptr_s[4]  ^ pcm_wptr_g_m1[3];
assign pcm_wptr_s[2]  = pcm_wptr_s[3]  ^ pcm_wptr_g_m1[2];
assign pcm_wptr_s[1]  = pcm_wptr_s[2]  ^ pcm_wptr_g_m1[1];
assign pcm_wptr_s[0]  = pcm_wptr_s[1]  ^ pcm_wptr_g_m1[0];
assign pcm_rptr_s[10] = pcm_rptr_g_m1[10];
assign pcm_rptr_s[9]  = pcm_rptr_s[10] ^ pcm_rptr_g_m1[9];
assign pcm_rptr_s[8]  = pcm_rptr_s[9]  ^ pcm_rptr_g_m1[8];
assign pcm_rptr_s[7]  = pcm_rptr_s[8]  ^ pcm_rptr_g_m1[7];
assign pcm_rptr_s[6]  = pcm_rptr_s[7]  ^ pcm_rptr_g_m1[6];
assign pcm_rptr_s[5]  = pcm_rptr_s[6]  ^ pcm_rptr_g_m1[5];
assign pcm_rptr_s[4]  = pcm_rptr_s[5]  ^ pcm_rptr_g_m1[4];
assign pcm_rptr_s[3]  = pcm_rptr_s[4]  ^ pcm_rptr_g_m1[3];
assign pcm_rptr_s[2]  = pcm_rptr_s[3]  ^ pcm_rptr_g_m1[2];
assign pcm_rptr_s[1]  = pcm_rptr_s[2]  ^ pcm_rptr_g_m1[1];
assign pcm_rptr_s[0]  = pcm_rptr_s[1]  ^ pcm_rptr_g_m1[0];
assign pcm_wused  = pcm_wptr - pcm_rptr_s;
assign pcm_rused  = pcm_wptr_s - pcm_rptr;
assign pcm_wfull  = (pcm_wused >= 11'd1000);
assign emerg_a    = emerg_a_sync[2];
// Lv.2: freeze PCM consumer so long stays do not empty BRAM (exit steal-storm → snow).
assign pcm_empty  = (!pcm_armed) || (pcm_rptr == pcm_wptr_s) || emerg_a || (rail_audio_a[2] && !announce_allow_a);
assign pcm_rst_wr = ~wav_rdy_m[2] || emerg_m[2] || (pcm_fault_hold!=0) || (rail_audio_m[2] ? announce_reset_m : (bgm_flush_m!=0));

wire hs_0;
wire vs_0;
wire de_0;

// HDMI 1.4b 音频发射相关
wire        audio_pll_lock;
wire        audio_mclk;
wire        audio_i2s_bclk;
wire        audio_i2s_lrck;
wire        audio_i2s_dout;
wire        audio_valid;
wire        acr_valid;
wire [19:0] acr_cts;
wire [19:0] acr_n;

wire        axis_s_user;
wire        axis_s_valid;
wire        axis_s_last;
wire [23:0] axis_s_data;
wire        axis_s_ready;

wire        edid_trig;
wire        edid_valid;
wire [7:0]  edid_data;

wire [9:0]  tmds_ch0_data;
wire [9:0]  tmds_ch1_data;
wire [9:0]  tmds_ch2_data;
wire [9:0]  tmds_clk_data;

// 统一复位：TF 图像链路 + HDMI 音频链路
wire rst_all;
assign rst_all = ~rst_n | ~audio_pll_lock;

// After bitstream / PLL lock, wait 200ms before SD init (avoids black screen until K1).
wire rst_sd;
exhibit_sd_startup_guard #(.CYCLES(12500000)) u_sd_startup_guard(
    .clk(sd_card_clk),.rst(rst_all),.reset_out(rst_sd));

// 保持你原来的 TF / SDRAM / video 时钟
sys_pll sys_pll_m0(
    .refclk     (clk),
    .clk0_out   (sd_card_clk),
    .clk1_out   (ext_mem_clk),
    .clk2_out   (ext_mem_clk_sft),
    .reset      (1'b0)
);

video_pll video_pll_m0(
    .refclk     (clk),
    .clk0_out   (video_clk),
    .clk1_out   (hdmi_5x_clk),
    .reset      (1'b0)
);

// 复用你 I2S->HDMI 工程里的 PLL，只取 12.288MHz 音频主时钟
PLL_HDMI_AUDIO u_audio_pll(
    .refclk     (clk),
    .reset      (1'b0),
    .extlock    (audio_pll_lock),
    .clk0_out   (),
    .clk1_out   (),
    .clk2_out   (audio_mclk)
);

// mem_clk 域把 write_finish 单拍转成 toggle，供 sd_card_clk 域可靠同步
always @(posedge ext_mem_clk or posedge rst_all) begin
    if (rst_all)
        frame_write_toggle_mem <= 1'b0;
    else if (frame_write_finish)
        frame_write_toggle_mem <= ~frame_write_toggle_mem;
end

// ===================== TF 多图扫描与缓存（双缓冲） =====================
wire        enc_vol_up;
wire        enc_vol_dn;
wire        enc_sw_n; // debounced SW, active-low

ec11_encoder #(
    .CLK_HZ (62_500_000)
) u_ec11_encoder (
    .I_clk     (sd_card_clk),
    .I_rst     (rst_sd),
    .I_enc_clk (enc_clk),
    .I_enc_dt  (enc_dt),
    .I_enc_sw  (enc_sw),
    .O_vol_up  (enc_vol_up),
    .O_vol_dn  (enc_vol_dn),
    .O_sw_n    (enc_sw_n)
);

wire ui_next,ui_prev,ui_auto,ui_emerg,ui_req,ui_ack,ui_panel;
wire pip_focus,pip_started;
wire [9:0] pip_panel_left,pip_panel_top;
wire [23:0] vout_data_pip;
wire pip_read_req,pip_read_ack,pip_read_valid,pip_read_end,pip_allowed,pip_card_reset;
wire [31:0] pip_read_sector,pip_sector0,pip_sector1,pip_size0,pip_size1;
wire [7:0] pip_read_byte;
wire [1:0] pip_catalog_valid;
wire [2:0] media_index_sd,buffer_video_sd;wire [5:0] media_available_sd;
wire rail_audio_present_sd,train_audio_done_sd;
wire announce_reset_m,announce_reset_a,announce_allow_a;
wire [19:0] announce_start_m,announce_left_m;
reg [2:0] rail_audio_m,rail_audio_a,rail_present_m,bgm_rst_a;
reg [1:0] bgm_idx_s0,bgm_idx_s1,bgm_idx_m;
wire [2:0] train_speech_event_sd,train_journey_stage_sd;wire [3:0] train_phase_sd;
wire train_voice_mode_sd = rail_audio_present_sd && (train_phase_sd != 4'd0);
wire media_auto_sd,media_write_flip,media_write_ready;
assign pip_read_req=1'b0;assign pip_read_sector=32'd0;
assign pip_mem_hold=1'b0;assign pip_mem_owner=1'b0;assign pip_mem_rd=1'b0;assign pip_mem_wr=1'b0;
assign pip_mem_addr=0;assign pip_mem_wdata=0;assign pip_line_miss=0;
wire [92:0] ui_mailbox,ui_cfg,video_mailbox,video_cfg;
wire video_cfg_req,video_cfg_ack,video_panel,normal_panel;wire [92:0] normal_cfg;
assign ui_cfg=video_emerg?video_cfg:normal_cfg;
assign ui_panel=video_emerg?video_panel:normal_panel;
exhibit_controls #(.CLK_HZ(62_500_000),.DEBOUNCE(1_250_000),.LONG_TIME(62_500_000)) u_controls(.clk(sd_card_clk),.rst(rst_sd),.sw(sw),
    .k2(key1),.k3(key3),.k4(key2),.enc_sw(enc_sw_n),
    .enc_up(enc_vol_up),.enc_dn(enc_vol_dn),.emerg(emerg_active),
    .cfg_ack(ui_ack),.cfg_req(ui_req),.cfg_mailbox(ui_mailbox),
    .uart_volume(uart_volume_sd),.volume(volume_sd),.volume_gesture(volume_gesture),
    .video_cfg_ack(video_cfg_ack),.video_cfg_req(video_cfg_req),.video_cfg_mailbox(video_mailbox),
    .next_p(ui_next),.prev_p(ui_prev),.auto_p(ui_auto),.emerg_p(ui_emerg),.media_focus(pip_focus),.video_started(pip_started));
exhibit_config_latch u_config(.clk(video_clk),.rst(rst_all),.vs(vs_0),
    .req(ui_req),.mailbox(ui_mailbox),.ack(ui_ack),.cfg(normal_cfg),.panel_visible(normal_panel));
exhibit_config_latch u_emergency_config(.clk(video_clk),.rst(rst_all),.vs(vs_0),
    .req(video_cfg_req),.mailbox(video_mailbox),.ack(video_cfg_ack),.cfg(video_cfg),.panel_visible(video_panel));
wire dht_valid,dht_error,fire_level,temperature_baseline_ready;
wire [7:0] temperature_c,temperature_baseline_c;
exhibit_dht11_reader #(.CLK_HZ(50_000_000),.POLL_US(2_100_000)) u_dht11(.clk(clk),.rst(rst_all),.data(dht_data),
 .sample_valid(dht_valid),.sample_error(dht_error),.temperature_c(temperature_c));
exhibit_temperature_alarm u_temperature_alarm(.clk(clk),.rst(rst_all),.sample_valid(dht_valid),
 .sample_error(dht_error),.temperature_c(temperature_c),.alarm_level(fire_level),
 .baseline_ready(temperature_baseline_ready),.baseline_c(temperature_baseline_c));
reg [2:0] fire_sync_sd;reg fire_seen_sd;
always @(posedge sd_card_clk or posedge rst_sd)begin
 if(rst_sd)begin fire_sync_sd<=0;fire_seen_sd<=0;end
 else begin fire_sync_sd<={fire_sync_sd[1:0],fire_level};fire_seen_sd<=fire_sync_sd[2];end
end
wire fire_trigger_sd=fire_sync_sd[2] && !fire_seen_sd;
wire [1:0] pending_display;

sd_card_bmp #(
    .CLK_FREQ_HZ       (62_500_000),
    .SCAN_START_SECTOR (32'd0),
    .SCAN_MAX_SECTOR   (32'd131071),
    .SCAN_TARGET_COUNT (4'd8),
    .FRAME_PIXELS      (FRAME_PIXELS)
) sd_card_bmp_m0(
    .pip_read_req(pip_read_req),.pip_read_sector(pip_read_sector),.pip_read_ack(pip_read_ack),
    .pip_read_valid(pip_read_valid),.pip_read_end(pip_read_end),.pip_read_byte(pip_read_byte),
    .pip_catalog_valid(pip_catalog_valid),.pip_sector0(pip_sector0),.pip_sector1(pip_sector1),
    .pip_size0(pip_size0),.pip_size1(pip_size1),.pip_allowed(pip_allowed),.pip_card_reset(pip_card_reset),
    .clk               (sd_card_clk),
    .rst               (rst_sd),
    .key_next          (key1),
    .key_enc_sw        (1'b1),
    .interpolate_ready(interpolate_ready_sd),.write_ready(media_write_ready),.buffer_video(buffer_video_sd),.write_flip(media_write_flip),.media_index(media_index_sd),.media_auto(media_auto_sd),.media_available(media_available_sd),
    .ctrl_next(ui_next),.ctrl_prev(ui_prev),.ctrl_auto(ui_auto),.ctrl_emerg(ui_emerg),
    .fire_trigger(fire_trigger_sd),
    .ctrl_period(emerg_active?video_mailbox[8:5]:ui_mailbox[8:5]),.pending_display(pending_display),
    .key_auto          (key2),
    .key_period        (key3),
    .emerg_n           (emerg_n),
    .ir_sensor         (ir_sensor),
    .vsync             (vs_0),
    .fade_swap_ok      (fade_swap_ok_sd_sync[2]),
    .fade_out_req      (fade_out_req),
    .state_code        (state_code),
    .boot_status       (media_boot_status_sd),
    .period_sec        (period_sec),
    .img_idx           (img_idx),
    .bmp_width         (16'd640),
    .bmp_height        (16'd480),
    .display_valid     (display_valid),

    .write_finish_toggle(frame_write_toggle_mem),
    .wav_spi_hold      (wav_spi_hold),
    .flash_cap_rdy     (flash_cap_rdy),
    .flash_idle        (flash_idle),
    .flash_slot_ok     (flash_slot_ok),
    .write_buf_idx     (write_buf_idx),
    .disp_buf_idx      (disp_buf_idx),

    .emerg_active      (emerg_active),
    .train_audio_available(wav_ready && rail_audio_present_sd),.train_audio_done(train_audio_done_sd),
    .rail_audio_present(rail_audio_present_sd),.train_speech_event(train_speech_event_sd),
    .train_phase       (train_phase_sd),
    .train_journey_stage(train_journey_stage_sd),
    .emerg_trig        (emerg_trig),
    .lat_ms            (lat_ms),
    .sd_frozen         (sd_frozen),
    .buf_c_ready       (buf_c_ready),
    .c_stat            (c_stat),
    .wav_pcm_we        (wav_pcm_we),
    .wav_pcm_addr      (wav_pcm_addr),
    .wav_pcm_data      (wav_pcm_data),
    .wav_ready         (wav_ready),
    .wav_load_busy     (wav_load_busy),
    .pic_wr_busy       (pic_wr_busy),
    .wav_found_n       (wav_found_n),
    .wav_dbg           (wav_dbg),
    .bmp_found_n       (bmp_found_n),

    .write_req         (sd_card_write_req),
    .write_req_ack     (sd_card_write_req_ack),
    .write_en          (sd_card_write_en),
    .write_data        (sd_card_write_data),
    .wr_kill_tgl       (sd_wr_kill_tgl),
    .SD_nCS            (sd_ncs),
    .SD_DCLK           (sd_dclk),
    .SD_MOSI           (sd_mosi),
    .SD_MISO           (sd_miso)
);

reg [2:0] emerg_clk_sync;
always @(posedge clk or posedge rst_all) begin
    if (rst_all)
        emerg_clk_sync <= 3'b000;
    else
        emerg_clk_sync <= {emerg_clk_sync[1:0], emerg_active};
end

emerg_buzzer u_emerg_buzzer (
    .I_clk    (clk),
    .I_rst    (rst_all),
    .I_enable (emerg_clk_sync[2]),
    .O_spk    (spk)
);

seg_decoder seg_decoder_m0(
    .bin_data          (state_clk),
    .seg_data          (seg_data_0)
);

seg_decoder seg_decoder_period(
    .bin_data          (period_clk),
    .seg_data          (seg_data_period)
);

// Software latency ms (0..99) on digits when Lv.2 active; scope pins for later
wire [3:0] lat_tens =
    (lat_clk >= 8'd90) ? 4'd9 : (lat_clk >= 8'd80) ? 4'd8 :
    (lat_clk >= 8'd70) ? 4'd7 : (lat_clk >= 8'd60) ? 4'd6 :
    (lat_clk >= 8'd50) ? 4'd5 : (lat_clk >= 8'd40) ? 4'd4 :
    (lat_clk >= 8'd30) ? 4'd3 : (lat_clk >= 8'd20) ? 4'd2 :
    (lat_clk >= 8'd10) ? 4'd1 : 4'd0;
wire [7:0] lat_ones_w = lat_clk - {1'b0, lat_tens, 3'b0} - {5'b0, lat_tens, 1'b0}; // -10*tens
wire [3:0] lat_ones = lat_ones_w[3:0];

seg_decoder seg_decoder_lat_tens(
    .bin_data          (lat_tens),
    .seg_data          (seg_data_lat_hi)
);

seg_decoder seg_decoder_lat_ones(
    .bin_data          (lat_ones),
    .seg_data          (seg_data_lat_lo)
);

seg_decoder seg_decoder_vol(
    .bin_data          (vol_lvl),
    .seg_data          (seg_data_vol)
);

assign scope_trig  = emerg_trig;
assign scope_vsync = vs_0;

seg_scan seg_scan_m0(
    .clk               (clk),
    .rst_n             (rst_n),
    .seg_sel           (seg_sel),
    .seg_data          (seg_data),
    .seg_data_0        ({1'b1,7'b1111_111}),
    .seg_data_1        ({1'b1,7'b1111_111}),
    .seg_data_2        (emerg_clk ? {1'b1, seg_data_lat_hi} : {1'b1,7'b1111_111}),
    .seg_data_3        (emerg_clk ? {1'b1, seg_data_lat_lo} : {1'b1, seg_data_vol}),
    .seg_data_4        ({1'b1, seg_data_period}),
    .seg_data_5        ({1'b1, frozen_clk ? 7'b0001110 : seg_data_0})
);

// The UI domain owns volume. UART sends absolute commands through a
// coherent mailbox; encoder steps operate on that same stored value.
always @(posedge clk or posedge rst_all) begin
    if(rst_all)uart_volume_request<=0;
    else if(vol_cmd_stb)uart_volume_request<={~uart_volume_request[4],vol_cmd};
end
exhibit_cdc_mailbox #(.WIDTH(5)) u_uart_volume_sd(
    .src_clk(clk),.dst_clk(sd_card_clk),.rst(rst_all),
    .src_data(uart_volume_request),.dst_data(uart_volume_sd));
exhibit_cdc_mailbox #(.WIDTH(4)) u_volume_feedback(
    .src_clk(sd_card_clk),.dst_clk(clk),.rst(rst_all),
    .src_data(volume_sd),.dst_data(vol_lvl));

wire [3:0] state_clk,period_clk;
wire [1:0] img_clk,cstat_clk;
wire [7:0] lat_clk,under_c1;
wire emerg_clk,frozen_clk;
wire [2:0] wav_n_clk,bmp_n_clk;
wire [3:0] dbg_clk;
wire [23:0] jedec_c1;
wire [2:0] flst_c1;
wire [11:0] used_c1;
exhibit_cdc_mailbox #(.WIDTH(59)) u_status_clk(
    .src_clk(sd_card_clk),.dst_clk(clk),.rst(rst_all),
    .src_data({state_code,period_sec,img_idx,lat_ms,emerg_active,sd_frozen,
               wav_found_n,wav_dbg,bmp_found_n,c_stat,jedec_id,fl_state}),
    .dst_data({state_clk,period_clk,img_clk,lat_clk,emerg_clk,frozen_clk,
               wav_n_clk,dbg_clk,bmp_n_clk,cstat_clk,jedec_c1,flst_c1}));
exhibit_cdc_mailbox #(.WIDTH(20)) u_pcm_status_clk(
    .src_clk(audio_mclk),.dst_clk(clk),.rst(rst_all),
    .src_data({1'b0,pcm_rused,pcm_under}),.dst_data({used_c1,under_c1}));
wire [3:0] vol_v;
wire [4:0] volume_status_v;
wire [3:0] volume_panel_value;
wire volume_panel_visible;
exhibit_cdc_mailbox #(.WIDTH(5)) u_volume_status_video(
    .src_clk(sd_card_clk),.dst_clk(video_clk),.rst(rst_all),
    .src_data({volume_gesture,volume_sd}),.dst_data(volume_status_v));
exhibit_volume_latch u_volume_panel_latch(.clk(video_clk),.rst(rst_all),
    .vs(vs_0),.status(volume_status_v),.picture_ack(video_emerg?video_cfg_ack:ui_ack),
    .volume(volume_panel_value),.visible(volume_panel_visible));
exhibit_cdc_mailbox #(.WIDTH(4)) u_volume_video(
    .src_clk(clk),.dst_clk(video_clk),.rst(rst_all),
    .src_data(vol_lvl),.dst_data(vol_v));
wire [1:0] display_v,pending_v;
wire [7:0] lat_v;
wire wav_ready_v;
wire interpolate_ready_sd;wire [39:0] video_status;wire [5:0] media_available_v;
// Synchronize memory initialization before adding it to the coherent SD descriptor.
(* ASYNC_REG="TRUE" *) reg [1:0] memory_init_sd;
always @(posedge sd_card_clk or posedge rst_sd)begin
 if(rst_sd)memory_init_sd<=0;else memory_init_sd<={memory_init_sd[0],Sdr_init_done};
end
wire [2:0] buffer_video_v,media_index_v;wire media_auto_v;
reg [39:0] video_status_frame;reg [4:0] status_guard;
wire [2:0] train_journey_stage_v=video_status_frame[34:32];
reg video_status_vs;
wire video_emerg,video_valid,video_c_ready,video_pic_wr,video_transition;
exhibit_cdc_mailbox #(.WIDTH(40)) u_status_video(
    .src_clk(sd_card_clk),.dst_clk(video_clk),.rst(rst_all),
    .src_data({memory_init_sd[1],media_boot_status_sd,train_journey_stage_sd,interpolate_ready_sd,media_available_sd,buffer_video_sd,media_index_sd,media_auto_sd,disp_buf_idx,pending_display,lat_ms,wav_ready,emerg_active,
               display_valid,buf_c_ready,pic_wr_busy,fade_out_req}),.dst_data(video_status));
// Capture the coherent descriptor after the manager commits on VS rising.
// The reader starts after an additional guard and primes rows in blanking.
always @(posedge video_clk or posedge rst_all)begin
    if(rst_all)begin video_status_frame<=0;video_status_vs<=0;status_guard<=0;end
    else begin
        video_status_vs<=vs_0;
        // SDRAM manager commits on VS rising. Wait for its coherent CDC payload
        // before freezing the page descriptor; no old front is read after reuse.
        if(vs_0 && !video_status_vs)status_guard<=1;
        else if(status_guard!=0)begin
            if(status_guard==16)begin video_status_frame<=video_status;status_guard<=0;end
            else status_guard<=status_guard+1'b1;
        end
    end
end
assign {display_v,pending_v,lat_v,wav_ready_v,video_emerg,
        video_valid,video_c_ready,video_pic_wr,video_transition}=video_status_frame[17:0];
assign {buffer_video_v,media_index_v,media_auto_v}=video_status_frame[24:18];
assign media_available_v=video_status_frame[30:25];

uart_console #(
    .CLK_HZ        (50_000_000),
    .BAUD          (115200)
) u_uart_console (
    .I_clk         (clk),
    .I_rst         (rst_all),
    .I_rx          (uart_rx),
    .O_tx          (uart_tx),
    .I_img         (img_clk),
    .I_period      (period_clk),
    .I_vol         (vol_lvl),
    .I_lat_ms      (lat_clk),
    .I_emerg       (emerg_clk),
    .I_frozen      (frozen_clk),
    .I_wav_n       (wav_n_clk),
    .I_dbg         (dbg_clk),
    .I_bmp_n       (bmp_n_clk),
    .I_st          (state_clk),
    .I_cstat       (cstat_clk),
    .I_jedec       (jedec_c1),
    .I_used        (used_c1),
    .I_under       (under_c1),
    .I_line_miss(line_c1),
    .O_vol         (vol_cmd),
    .O_vol_stb     (vol_cmd_stb),
    .O_warn        (warn_lv1)
);

// ===================== 原图像时序与帧缓存 =====================
video_timing_data video_timing_data_m0(
    .video_clk         (video_clk),
    .rst               (rst_all),
    .read_req          (video_read_req),
    .read_req_ack      (video_read_req_ack),
    .hs                (hs_0),
    .vs                (vs_0),
    .de                (de_0)
);

wire reader_hs,reader_vs,reader_de,color_hs,color_vs,color_de;
wire [23:0] reader_rgb,color_rgb;
wire [15:0] line_miss;
reg [15:0] line_gray;
always @(posedge video_clk or posedge rst_all)begin
    if(rst_all)line_gray<=0;else line_gray<=(line_miss>>1)^line_miss;
end
reg [15:0] line_g0,line_g1;
wire [15:0] line_c1;
assign line_c1[15]=line_g1[15];
genvar lg;
generate for(lg=0;lg<15;lg=lg+1)begin:line_decode
    assign line_c1[lg]=line_c1[lg+1]^line_g1[lg];
end endgenerate
always @(posedge clk or posedge rst_all)begin
    if(rst_all)begin line_g0<=0;line_g1<=0;end
    else begin line_g0<=line_gray;line_g1<=line_g0;end
end
exhibit_reader u_reader(.vclk(video_clk),.mclk(ext_mem_clk),.rst(rst_all),
    .ihs(hs_0),.ivs(vs_0),.ide(de_0),.cfg(ui_cfg),
    .display_buf(display_v),.pending_buf(pending_v),.buffer_video(buffer_video_v),
    .temporal_blend(video_status_frame[31]),.transition_req(video_transition),.emergency(video_emerg),
    .display_valid(video_valid),.mem_init(Sdr_init_done),.mem_busy(Sdr_busy),
    .write_busy(wr_mem_busy | App_wr_en),.audio_hold(aud_hold_ext | pip_mem_hold),.data_valid(vid_data),.data(Sdr_rd_dout),
    .rd_en(vid_rd_en),.rd_addr(vid_rd_addr),.rd_busy(vid_rd_busy),.rd_active(vid_rd_active),.rd_recover(vid_rd_recover),
    .swap_ok(fade_swap_ok_v),.ohs(reader_hs),.ovs(reader_vs),.ode(reader_de),
    .rgb(reader_rgb),.line_miss(line_miss));
exhibit_color u_color(.clk(video_clk),.rst(rst_all),.ihs(reader_hs),.ivs(reader_vs),.ide(reader_de),
    .enable(!video_emerg || video_c_ready),.cfg(ui_cfg),.irgb(reader_rgb),
    .ohs(color_hs),.ovs(color_vs),.ode(color_de),.rgb(color_rgb));
exhibit_layer u_layer(.clk(video_clk),.rst(rst_all),.ihs(color_hs),.ivs(color_vs),.ide(color_de),
    .enable(!video_emerg || video_c_ready),.fullscreen(video_emerg),.cfg(ui_cfg),.irgb(color_rgb),
    .ohs(hs),.ovs(vs),.ode(de),.rgb(vout_data_raw));
assign video_read_req_ack=1'b1;
// Slice 5: fade-to-black / fade-in around buffer swap + bottom OSD ticker
always @(posedge sd_card_clk or posedge rst_all) begin
    if (rst_all)
        fade_swap_ok_sd_sync <= 3'b000;
    else
        fade_swap_ok_sd_sync <= {fade_swap_ok_sd_sync[1:0], fade_swap_ok_v};
end

always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        fade_req_v_sync       <= 3'b000;
        display_valid_v_sync  <= 3'b000;
        emerg_active_v_sync   <= 3'b000;
        buf_c_ready_v_sync    <= 3'b000;
        warn_lv1_v_sync       <= 3'b000;
        pic_wr_v_sync         <= 3'b000;
    end else begin
        fade_req_v_sync      <= {fade_req_v_sync[1:0], fade_out_req};
        display_valid_v_sync <= {display_valid_v_sync[1:0], display_valid};
        emerg_active_v_sync  <= {emerg_active_v_sync[1:0], emerg_active};
        buf_c_ready_v_sync   <= {buf_c_ready_v_sync[1:0], buf_c_ready};
        warn_lv1_v_sync      <= {warn_lv1_v_sync[1:0], warn_lv1};
        pic_wr_v_sync        <= {pic_wr_v_sync[1:0], pic_wr_busy};
    end
end

// No fade-to-black: the scanline reader mixes old and new images.
assign vout_data_wipe=(video_emerg && !video_c_ready) ?
    24'hCC2200 : vout_data_raw;
// Eight registered OSD stages: Chinese BRAM renderer adds two pixel clocks.
reg [16:0] osd_hs_pipe, osd_vs_pipe, osd_de_pipe;
wire [9:0] metro_fps_tenths;
wire metro_fps_valid;
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        osd_hs_pipe <= 17'd0;
        osd_vs_pipe <= 17'd0;
        osd_de_pipe <= 17'd0;
    end else begin
        osd_hs_pipe <= {osd_hs_pipe[15:0], hs};
        osd_vs_pipe <= {osd_vs_pipe[15:0], vs};
        osd_de_pipe <= {osd_de_pipe[15:0], de};
    end
end

// Atomic held-data transfer: never synchronize temperature bits independently.
wire [3:0] temperature_tens_v,temperature_ones_v;
wire temperature_valid_v,temperature_alarm_v,temperature_update_v;
exhibit_temperature_display_bridge u_temperature_display(
 .sensor_clk(clk),.video_clk(video_clk),.rst(rst_all),
 .sample_valid(dht_valid),.temperature_c(temperature_c),.alarm_level(fire_level),
 .display_valid(temperature_valid_v),.display_alarm(temperature_alarm_v),.display_update(temperature_update_v),
 .display_tens(temperature_tens_v),.display_ones(temperature_ones_v));
// Wave sits over the picture after chrome. PCM tap only, no SDRAM access.
osd_metro_chrome u_osd_metro_chrome (
    .I_boot_status(video_status_frame[39]?video_status_frame[38:35]:4'd7),
    .I_media_ready(video_valid),.I_temperature_update(temperature_update_v),
    .I_temperature_valid(temperature_valid_v),.I_temperature_alarm(temperature_alarm_v),
    .I_temperature_tens(temperature_tens_v),.I_temperature_ones(temperature_ones_v),
    .I_journey_stage(train_journey_stage_v),
    .I_media_index(media_index_v),.I_media_auto(media_auto_v),.I_media_available(media_available_v),
    .I_scroll_speed(ui_cfg[22:17]),
    .I_clk    (video_clk),
    .I_rst    (rst_all),
    .I_hs     (hs),
    .I_vs     (vs),
    .I_de     (de),
    .I_enable (!video_emerg),
    .I_warn   (warn_lv1_v_sync[2] && !video_emerg),
    .I_fps_tenths(metro_fps_tenths), .I_fps_valid(metro_fps_valid),
    .I_rgb    (vout_data_wipe),
    .O_rgb    (vout_data_metro)
);

// Preserve the original right-rail trace, in addition to the bottom envelope.
osd_audio_wave_rail u_osd_audio_wave_rail (
    .I_clk(video_clk), .I_rst(rst_all),
    .I_hs(osd_hs_pipe[2]), .I_vs(osd_vs_pipe[2]), .I_de(osd_de_pipe[2]),
    .I_enable(!video_emerg),
    .I_audio_valid(audio_valid), .I_pcm(audio_left_data),
    .I_rgb(vout_data_metro), .O_rgb(vout_data_rail)
);

osd_audio_wave u_osd_audio_wave (
    .I_clk         (video_clk),
    .I_rst         (rst_all),
    .I_hs          (osd_hs_pipe[3]),
    .I_vs          (osd_vs_pipe[3]),
    .I_de          (osd_de_pipe[3]),
    .I_enable      (!video_emerg),
    .I_audio_valid (audio_valid),
    .I_pcm         (audio_left_data),
    .I_rgb         (vout_data_rail),
    .O_rgb         (vout_data_wave)
);

// Measure HDMI output VS periods; text is inside the picture's top-left.
osd_video_fps #(.CLOCK_HZ(25'd25000000)) u_osd_video_fps (
    .I_clk(video_clk), .I_rst(rst_all),
    .I_hs(osd_hs_pipe[4]), .I_vs(osd_vs_pipe[4]), .I_de(osd_de_pipe[4]),
    .I_enable(1'b0),
    .I_rgb(vout_data_wave), .O_rgb(vout_data_fps),
    .O_fps_tenths(metro_fps_tenths), .O_fps_valid(metro_fps_valid)
);

osd_emerg_banner u_osd_emerg_banner (
    .I_clk      (video_clk),
    .I_rst      (rst_all),
    .I_hs       (osd_hs_pipe[5]),
    .I_vs       (osd_vs_pipe[5]),
    .I_de       (osd_de_pipe[5]),
    .I_enable   (video_emerg && !video_c_ready),
    .I_lat_ms   (lat_v),
    .I_rgb      (vout_data_fps),
    .O_rgb      (vout_data_emerg)
);

wire [23:0] vout_data_panel_in;
osd_ticker_bar u_osd_ticker_bar (
    .I_clk     (video_clk),
    .I_rst     (rst_all),
    .I_hs      (osd_hs_pipe[6]),
    .I_vs      (osd_vs_pipe[6]),
    .I_de      (osd_de_pipe[6]),
    .I_enable  (!video_emerg),
    .I_rgb     (vout_data_emerg),
    .O_rgb     (vout_data_panel_in)
);

exhibit_panel u_panel(.clk(video_clk),.rst(rst_all),
    .vs(osd_vs_pipe[7]),.de(osd_de_pipe[7]),.irgb(vout_data_panel_in),
    .enable((ui_panel || volume_panel_visible) && (!video_emerg || video_c_ready)),.cfg(ui_cfg),.media_video(1'b1),.video_panel_x(10'd524),.video_panel_y(video_emerg?10'd88:10'd132),
    .volume_show(volume_panel_visible),.volume(volume_panel_value),.rgb(vout_data));

// Lv.2: pulse-reset mem path on enter only (clear stuck writer). Do NOT hold reset
// for the whole Lv.2 stay — that blanked Buffer C reads (black emergency screen).
// Also pulse-reset when sd_card_bmp aborts a write session (else S3 waits forever).
reg [2:0]  emerg_mem_m;
reg [8:0]  emerg_mem_kill;
reg [2:0]  wr_kill_sync;
reg [8:0]  wr_session_kill;
always @(posedge ext_mem_clk or posedge rst_all) begin
    if (rst_all) begin
        emerg_mem_m     <= 3'b000;
        emerg_mem_kill  <= 9'd0;
        wr_kill_sync    <= 3'b000;
        wr_session_kill <= 9'd0;
    end else begin
        emerg_mem_m  <= {emerg_mem_m[1:0], emerg_active};
        wr_kill_sync <= {wr_kill_sync[1:0], sd_wr_kill_tgl};
        if (emerg_mem_m[1] && !emerg_mem_m[2])
            emerg_mem_kill <= 9'd255;
        else if (emerg_mem_kill != 9'd0)
            emerg_mem_kill <= emerg_mem_kill - 9'd1;
        if (wr_kill_sync[2] ^ wr_kill_sync[1])
            wr_session_kill <= 9'd255;
        else if (wr_session_kill != 9'd0)
            wr_session_kill <= wr_session_kill - 9'd1;
    end
end

frame_read_write #(
    .WRITE_V_FLIP     (1),
    .FRAME_WIDTH      (640),
    .FRAME_HEIGHT     (480)
) frame_read_write_m0(
    .mem_clk           (ext_mem_clk),
    .rst               (rst_all),
    .write_abort_rst   ((emerg_mem_kill != 9'd0) | (wr_session_kill != 9'd0)),
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),

    .App_rd_en         (),
    .App_rd_addr       (),
    .O_app_rd_busy     (),
    .O_vid_rd_active   (),
    .App_ext_hold      (aud_hold_ext | vid_rd_active | pip_mem_hold),
    .Sdr_rd_en         (Sdr_rd_en & ~aud_hold_ext),
    .Sdr_rd_dout       (Sdr_rd_dout),

    .read_clk          (video_clk),
    .read_req          (1'b0),
    .read_req_ack      (),
    .read_finish       (),
    .read_addr_0       (BUF0_ADDR),
    .read_addr_1       (BUF1_ADDR),
    .read_addr_2       (BUF2_ADDR),
    .read_addr_3       (24'd0),
    .read_addr_index   (disp_buf_idx),
    .read_len          (FRAME_PIXELS),
    .read_en           (video_read_en),
    .read_data         (video_read_data),

    .App_wr_en         (photo_wr_en),
    .App_wr_addr       (photo_wr_addr),
    .App_wr_din        (photo_wr_din),
    .App_wr_dm         (photo_wr_dm),
    .O_wr_busy         (wr_mem_busy),

    .write_ready(media_write_ready),.write_flip(media_write_flip),
    .write_clk         (sd_card_clk),
    .write_req         (sd_card_write_req),
    .write_req_ack     (sd_card_write_req_ack),
    .write_finish      (frame_write_finish),
    .write_addr_0      (BUF0_ADDR),
    .write_addr_1      (BUF1_ADDR),
    .write_addr_2      (BUF2_ADDR),
    .write_addr_3      (AUDIO_BASE),
    .write_addr_index  (write_buf_idx),
    // Never latch AUDIO_LEN for Buffer C (idx==2); that hung C until 6s watchdog.
    .write_len         ((write_buf_idx == 2'd2) ? FRAME_PIXELS :
                        (wav_load_busy || (write_buf_idx == 2'd3)) ? AUDIO_LEN :
                        FRAME_PIXELS),
    .write_en          (sd_card_write_en),
    .write_data        (sd_card_write_data)
);

sdram U3(
    .Clk               (ext_mem_clk),
    .Clk_sft           (ext_mem_clk_sft),
    .Rst               (rst_all),
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),
    .App_wr_en         (App_wr_en),
    .App_wr_addr       (App_wr_addr),
    .App_wr_dm         (App_wr_dm),
    .App_wr_din        (App_wr_din),
    .App_rd_en         (App_rd_en),
    .App_rd_addr       (App_rd_addr),
    .Sdr_rd_en         (Sdr_rd_en),
    .Sdr_rd_dout       (Sdr_rd_dout)
);

EG_LOGIC_BRAM #(
    .DATA_WIDTH_A (16),
    .DATA_WIDTH_B (16),
    .ADDR_WIDTH_A (10),
    .ADDR_WIDTH_B (10),
    .DATA_DEPTH_A (1024),
    .DATA_DEPTH_B (1024),
    .MODE         ("DP"),
    .REGMODE_A    ("NOREG"),
    .REGMODE_B    ("NOREG"),
    .WRITEMODE_A  ("NORMAL"),
    .WRITEMODE_B  ("NORMAL"),
    .RESETMODE    ("SYNC"),
    .IMPLEMENT    ("9K"),
    .INIT_FILE    ("NONE"),
    .FILL_ALL     ("NONE")
) u_pcm_bram (
    .dia   (Sdr_rd_dout[15:0]),
    .dib   (16'd0),
    .addra (pcm_wptr[9:0]),
    .addrb (pcm_rptr[9:0]),
    .cea   (1'b1),
    .ceb   (1'b1),
    .ocea  (1'b0),
    .oceb  (1'b0),
    .clka  (ext_mem_clk),
    .clkb  (audio_mclk),
    .wea   (aud_data && !pcm_rst_wr),
    .web   (1'b0),
    .bea   (1'b0),
    .beb   (1'b0),
    .rsta  (1'b0),
    .rstb  (1'b0),
    .doa   (pcm_doa_unused),
    .dob   (pcm_bram_q)
);

// V6 PCM ownership: never discard an outstanding physical read on emergency.
// Audio and video responses cannot share a session. A reset flushes both FIFO
// pointers; after exit the source restarts at AUDIO_BASE and prefills 768 samples.
reg [15:0] aud_watch;
reg [2:0] aud_watch_div;
reg aud_timeout;
reg [15:0] pcm_fault_hold;
wire [19:0] bgm_start_m = (bgm_idx_m == 2'd0) ? 20'd860160 :
                          (bgm_idx_m == 2'd1) ? 20'd885120 :
                          (bgm_idx_m == 2'd2) ? 20'd910080 : 20'd935040;
wire [19:0] bgm_end_m   = (bgm_idx_m == 2'd0) ? 20'd885119 :
                          (bgm_idx_m == 2'd1) ? 20'd910079 :
                          (bgm_idx_m == 2'd2) ? 20'd935039 : 20'd959999;
wire [20:0] loop_start_addr = AUDIO_BASE[ADDR_BITS-1:0] + (rail_present_m[2] ? {1'b0, bgm_start_m} : 21'd0);
wire [20:0] loop_end_addr   = AUDIO_BASE[ADDR_BITS-1:0] + (rail_present_m[2] ? {1'b0, bgm_end_m}   : (AUDIO_LEN[ADDR_BITS-1:0] - 21'd1));
always @(posedge ext_mem_clk or posedge rst_all)begin
 if(rst_all)begin
  rail_audio_m<=0;rail_present_m<=0;
  bgm_idx_s0<=0;bgm_idx_s1<=0;bgm_idx_m<=0;bgm_flush_m<=0;
 end else begin
  rail_audio_m<={rail_audio_m[1:0],train_voice_mode_sd};
  rail_present_m<={rail_present_m[1:0],rail_audio_present_sd};
  bgm_idx_s0<=img_idx;bgm_idx_s1<=bgm_idx_s0;
  if(bgm_idx_s1==bgm_idx_s0 && bgm_idx_m!=bgm_idx_s1)begin
   bgm_idx_m<=bgm_idx_s1;
   if(!rail_audio_m[2])bgm_flush_m<=6'd63;
  end else if(rail_audio_m[2] && !rail_audio_m[1])begin
   bgm_flush_m<=6'd63;
  end else if(bgm_flush_m!=0 && !aud_hold_ext)begin
   bgm_flush_m<=bgm_flush_m-1'b1;
  end
 end
end
always @(posedge audio_mclk or posedge rst_all)begin
 if(rst_all)begin rail_audio_a<=0;bgm_rst_a<=0;end
 else begin
  rail_audio_a<={rail_audio_a[1:0],train_voice_mode_sd};
  bgm_rst_a<={bgm_rst_a[1:0],(bgm_flush_m!=0)};
 end
end
rail_announcement_player u_rail_voice(
 .sclk(sd_card_clk),.mclk(ext_mem_clk),.aclk(audio_mclk),.rst(rst_all),
 .event_id(train_speech_event_sd),.ready_m(wav_rdy_m[2] && rail_present_m[2]),
 .emergency_m(emerg_m[2]),.fault_m(pcm_fault_hold!=0),.mem_idle(!aud_hold_ext),
 .mem_request_issued(aud_rd_en),.sample_consumed(pcm_pop && !pcm_empty),
 .reset_mem(announce_reset_m),.reset_audio(announce_reset_a),.allow_sample(announce_allow_a),
 .sample_start(announce_start_m),.fetch_remaining(announce_left_m),.done_pulse(train_audio_done_sd));

always @(posedge ext_mem_clk or posedge rst_all) begin
    if(rst_all) begin
        wav_rdy_m<=0;de_m<=0;aud_gap<=0;aud_rd_en<=0;aud_lock<=0;
        aud_issue<=0;aud_need<=0;aud_tail<=0;aud_hold_d<=0;vid_os<=0;
        aud_rd_addr<=AUDIO_BASE[ADDR_BITS-1:0];pcm_wptr<=0;pcm_wptr_g<=0;
        pcm_rptr_g_m0<=0;pcm_rptr_g_m1<=0;
        wload_m<=0;picw_m<=0;bufc_m<=0;emerg_m<=0;widx2_m<=0;
        aud_arm<=18'd80000;wr_mem_busy_q<=0;pcm_has_space_q<=0;aud_watch<=0;aud_watch_div<=0;aud_timeout<=0;pcm_fault_hold<=0;
    end else begin
        wav_rdy_m<={wav_rdy_m[1:0],wav_ready};de_m<={de_m[1:0],de_0};
        wload_m<={wload_m[1:0],wav_load_busy};picw_m<={picw_m[1:0],pic_wr_busy};
        bufc_m<={bufc_m[1:0],buf_c_ready};emerg_m<={emerg_m[1:0],emerg_active};
        widx2_m<={widx2_m[1:0],write_buf_idx==2'd2};
        pcm_rptr_g_m0<=pcm_rptr_g;pcm_rptr_g_m1<=pcm_rptr_g_m0;
        wr_mem_busy_q<=wr_mem_busy;pcm_has_space_q<=pcm_wused<960;
        aud_hold_d<={aud_hold_d[0],aud_hold};aud_rd_en<=0;
        if(pcm_fault_hold!=0)pcm_fault_hold<=pcm_fault_hold-1;
        aud_timeout<=(aud_watch==16'd24999);
        if(picw_m[2] || wload_m[2] || emerg_m[2] || widx2_m[2] || !wav_rdy_m[2])
            aud_arm<=80000;
        else if(aud_arm!=0)aud_arm<=aud_arm-1;
        case({vid_rd_en,vid_data})
            2'b10:vid_os<=vid_os+1;
            2'b01:vid_os<=vid_os==0?0:vid_os-1;
        endcase
        if(vid_rd_recover)vid_os<=0;
        if(vid_rd_active || App_wr_en || wr_mem_busy || Sdr_busy || Sdr_rd_en)
            aud_gap<=0;
        else if(aud_gap!=63)aud_gap<=aud_gap+1;
        if(aud_tail!=0)aud_tail<=aud_tail-1;
        // Count requests at the actual controller interface, not at scheduling.
        case({aud_rd_en,aud_data})
            2'b10:aud_need<=aud_need+1;
            2'b01:aud_need<=aud_need-1;
        endcase
        if(aud_rd_en) begin
            if(!rail_audio_m[2] && aud_rd_addr==loop_end_addr)
                aud_rd_addr<=loop_start_addr;
            else aud_rd_addr<=aud_rd_addr+1;
        end
        if(pcm_rst_wr) begin
            pcm_wptr<=0;pcm_wptr_g<=0;
            // Wait for the last issued address to leave the physical interface.
            if(!aud_rd_en)aud_rd_addr<=rail_audio_m[2]?(AUDIO_BASE[ADDR_BITS-1:0]+{1'b0,announce_start_m}):loop_start_addr;
        end else if(aud_data) begin
            pcm_wptr<=pcm_wptr+1;
            pcm_wptr_g<=(pcm_wptr+11'd1)^((pcm_wptr+11'd1)>>1);
        end
        if(aud_lock) begin
            if(pcm_rst_wr || picw_m[2] || wload_m[2] || widx2_m[2])
                aud_issue<=0; // cancel only requests that have not been issued
            else if(aud_issue!=0 && !vid_rd_en) begin
                aud_rd_en<=1;aud_issue<=aud_issue-1;
            end
            if(aud_issue==0 && aud_need==0 && !aud_rd_en) begin
                aud_lock<=0;aud_tail<=31;aud_watch<=0;aud_watch_div<=0;
            end else if(aud_data || aud_rd_en)begin aud_watch<=0;aud_watch_div<=0;end
            else if(!aud_timeout)begin
                aud_watch_div<=aud_watch_div+1;
                if(aud_watch_div==7 && aud_watch<16'd24999)aud_watch<=aud_watch+1;
            end
            else if(!Sdr_busy && !Sdr_rd_en) begin
                // A controller that lost a response must not latch the bus forever.
                // Reset ownership only after 4ms without any physical data.
                aud_lock<=0;aud_issue<=0;aud_need<=0;aud_tail<=31;
                pcm_wptr<=0;pcm_wptr_g<=0;aud_rd_addr<=AUDIO_BASE[ADDR_BITS-1:0];
                aud_watch<=0;pcm_fault_hold<=65535;
            end
        end else if(!pcm_rst_wr && aud_tail==0 && aud_hold_d==0 &&
            !pip_mem_hold && !vid_rd_active && !vid_rd_en && !vid_rd_busy && vid_os==0 &&
            !App_wr_en && !wr_mem_busy && !Sdr_busy && Sdr_init_done &&
            !picw_m[2] && !wload_m[2] && !widx2_m[2] &&
            aud_arm==0 && pcm_has_space_q && aud_gap>=8 && (!rail_audio_m[2] || announce_left_m!=0)) begin
            aud_lock<=1;aud_issue<=(rail_audio_m[2] && announce_left_m<16)?announce_left_m[4:0]:5'd16;aud_watch<=0;
        end
    end
end

reg        pcm_flush_tog;
reg [2:0]  pcm_flush_s;

// U33: JEDEC constant (W25Q64 EF4017); frees ~120 LUTs for MSlice budget.
wire [2:0]  fl_state = 3'd2;
wire [23:0] jedec_id = 24'hEF4017;
reg         flash_sdo_keep;
always @(posedge sd_card_clk or posedge rst_all) begin
    if (rst_all) flash_sdo_keep <= 1'b0;
    else         flash_sdo_keep <= flash_sdo;
end
assign flash_cs       = 1'b1;
assign flash_sclk     = 1'b0;
assign flash_sdi      = flash_sdo_keep & 1'b0;
assign flash_wp       = 1'b1;
assign flash_hold     = 1'b1;
assign flash_slot_ok  = 4'd0;
assign flash_cap_rdy  = 1'b1;
assign flash_idle     = 1'b1;
assign wav_spi_hold   = 1'b0;

always @(posedge audio_mclk or posedge rst_all) begin
    if (rst_all) begin
        pcm_rptr      <= 11'd0;
        pcm_rptr_g    <= 11'd0;
        pcm_wptr_g_m0 <= 11'd0;
        pcm_wptr_g_m1 <= 11'd0;
        pcm_rd        <= 16'd0;
        wav_rdy_a     <= 2'b00;
        pcm_armed     <= 1'b0;
        pcm_under     <= 8'd0;
        pcm_flush_s   <= 3'b000;
        emerg_a_sync  <= 3'b000;
        pcm_fault_a<=0;
    end else begin
        emerg_a_sync  <= {emerg_a_sync[1:0], emerg_active};
        pcm_fault_a<={pcm_fault_a[1:0],pcm_fault_hold!=0};
        pcm_wptr_g_m0 <= pcm_wptr_g;
        pcm_wptr_g_m1 <= pcm_wptr_g_m0;
        wav_rdy_a     <= {wav_rdy_a[0], wav_ready};
        if (!wav_rdy_a[1] || emerg_a || pcm_fault_a[2] || (rail_audio_a[2] ? announce_reset_a : bgm_rst_a[2])) begin
            pcm_rptr   <= 11'd0;
            pcm_rptr_g <= 11'd0;
            pcm_rd     <= 16'd0;
            pcm_armed  <= 1'b0;
            pcm_under  <= 8'd0;
        end else begin
            if (!pcm_armed && pcm_rused >= 11'd768) pcm_armed<=1'b1;
            if (pcm_armed && pcm_rptr==pcm_wptr_s) begin
                pcm_armed<=1'b0;
                if(pcm_under!=8'hFF)pcm_under<=pcm_under+8'd1;
            end
            if (pcm_pop && !pcm_empty) begin
                pcm_rptr   <= pcm_rptr + 11'd1;
                pcm_rptr_g <= (pcm_rptr + 11'd1) ^ ((pcm_rptr + 11'd1) >> 1);
            end
            pcm_rd <= pcm_empty ? 16'd0 : pcm_bram_q;
        end
    end
end

// ===================== 音频：图绑定音高（第4刀，非例程固定音阶） =====================
wire [1:0] img_audio;
exhibit_cdc_mailbox #(.WIDTH(2)) u_image_audio(
    .src_clk(sd_card_clk),.dst_clk(audio_mclk),.rst(rst_all),
    .src_data(img_idx),.dst_data(img_audio));

hdmi_audio_tone_i2s_64fs #(
    .PHASE_INC (32'd39370534),
    .AMP       (24'sd2000000)
) u_hdmi_audio_tone_i2s_64fs (
    .I_mclk      (audio_mclk),
    .I_rst       (rst_all),
    .I_img_idx   (img_audio),
    .I_wav_clk   (sd_card_clk),
    .I_wav_we    (1'b0),
    .I_wav_addr  (15'd0),
    .I_wav_data  (16'd0),
    .I_wav_ready (wav_ready),
    .I_pcm       (pcm_rd),
    .I_pcm_empty (pcm_empty),
    .O_pcm_pop   (pcm_pop),
    .O_i2s_BCLK  (audio_i2s_bclk),
    .O_i2s_LRCK  (audio_i2s_lrck),
    .O_i2s_DOUT  (audio_i2s_dout)
);

I2S_receiver u_I2S_receiver(
    .I_clk              (video_clk),
    .I_rst              (rst_all),
    .I_i2s_BCLK         (audio_i2s_bclk),
    .I_i2s_LRCK         (audio_i2s_lrck),
    .I_i2s_DOUT         (audio_i2s_dout),
    .O_audio_valid      (audio_valid),
    .O_audio_left_data  (audio_left_raw),
    .O_audio_right_data (audio_right_raw)
);

// Slice 9: volume 0..8. Shift-only (no 28-bit add tree on video_clk).
reg signed [23:0] vol_l_q;
reg signed [23:0] vol_r_q;
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        vol_l_q <= 24'sd0;
        vol_r_q <= 24'sd0;
    end else begin
        if (audio_valid) begin
            case (vol_v)
                4'd0: begin vol_l_q <= 24'sd0; vol_r_q <= 24'sd0; end
                4'd1: begin
                    vol_l_q <= {{3{audio_left_raw[23]}},  audio_left_raw[23:3]};
                    vol_r_q <= {{3{audio_right_raw[23]}}, audio_right_raw[23:3]};
                end
                4'd2: begin
                    vol_l_q <= {{2{audio_left_raw[23]}},  audio_left_raw[23:2]};
                    vol_r_q <= {{2{audio_right_raw[23]}}, audio_right_raw[23:2]};
                end
                4'd3: begin
                    vol_l_q <= {{2{audio_left_raw[23]}},  audio_left_raw[23:2]}
                               + {{3{audio_left_raw[23]}}, audio_left_raw[23:3]};
                    vol_r_q <= {{2{audio_right_raw[23]}}, audio_right_raw[23:2]}
                               + {{3{audio_right_raw[23]}}, audio_right_raw[23:3]};
                end
                4'd4: begin
                    vol_l_q <= {audio_left_raw[23],  audio_left_raw[23:1]};
                    vol_r_q <= {audio_right_raw[23], audio_right_raw[23:1]};
                end
                4'd5: begin
                    vol_l_q <= {audio_left_raw[23],  audio_left_raw[23:1]}
                               + {{3{audio_left_raw[23]}}, audio_left_raw[23:3]};
                    vol_r_q <= {audio_right_raw[23], audio_right_raw[23:1]}
                               + {{3{audio_right_raw[23]}}, audio_right_raw[23:3]};
                end
                4'd6: begin
                    vol_l_q <= {audio_left_raw[23],  audio_left_raw[23:1]}
                               + {{2{audio_left_raw[23]}}, audio_left_raw[23:2]};
                    vol_r_q <= {audio_right_raw[23], audio_right_raw[23:1]}
                               + {{2{audio_right_raw[23]}}, audio_right_raw[23:2]};
                end
                4'd7: begin
                    vol_l_q <= audio_left_raw
                               - {{3{audio_left_raw[23]}}, audio_left_raw[23:3]};
                    vol_r_q <= audio_right_raw
                               - {{3{audio_right_raw[23]}}, audio_right_raw[23:3]};
                end
                default: begin
                    vol_l_q <= audio_left_raw;
                    vol_r_q <= audio_right_raw;
                end
            endcase
        end
    end
end
reg [2:0] pcm_play_s;
reg [2:0] siren_v_sync;
reg [8:0] resume_gain;
wire mute_pcm=emerg_active_v_sync[2] || pic_wr_v_sync[2] || (wav_ready_v && !pcm_play_s[2]);
always @(posedge video_clk or posedge rst_all) begin
    if(rst_all)begin pcm_play_s<=0;siren_v_sync<=0;resume_gain<=0;end
    else begin
        pcm_play_s<={pcm_play_s[1:0],pcm_armed};
        siren_v_sync<={siren_v_sync[1:0],spk};
        if(mute_pcm)resume_gain<=0;
        else if(audio_valid && resume_gain<256)resume_gain<=resume_gain+1;
    end
end
wire signed [33:0] scaled_l=$signed(vol_l_q)*$signed({1'b0,resume_gain});
wire signed [33:0] scaled_r=$signed(vol_r_q)*$signed({1'b0,resume_gain});
wire signed [23:0] siren_pcm=(vol_v==4'd0)?24'sd0:(siren_v_sync[2]?24'sd2097152:-24'sd2097152);
assign audio_left_data=emerg_active_v_sync[2]?siren_pcm:(mute_pcm?24'd0:scaled_l[31:8]);
assign audio_right_data=emerg_active_v_sync[2]?siren_pcm:(mute_pcm?24'd0:scaled_r[31:8]);

audio_arc_calculate #(
    .ACR_N         (6144)
) u_audio_arc_calculate (
    .I_clk         (video_clk),
    .I_rst         (rst_all),
    .I_audio_valid (audio_valid),
    .O_acr_valid   (acr_valid),
    .O_acr_cts     (acr_cts),
    .O_acr_n       (acr_n)
);

// ===================== RGB/DE 转 AXIS 视频 =====================
video_rgb_to_axis_640x480 u_video_rgb_to_axis_640x480(
    .I_clk         (video_clk),
    .I_rst         (rst_all),
    .I_vs          (osd_vs_pipe[10]),
    .I_de          (osd_de_pipe[10]),
    .I_rgb         (vout_data),
    .O_video_user  (axis_s_user),
    .O_video_valid (axis_s_valid),
    .O_video_last  (axis_s_last),
    .O_video_data  (axis_s_data)
);

// 上电后自动打一拍，触发一次 EDID 读取
startup_pulse #(
    .CNT_MAX(20'd100000)
) u_startup_pulse (
    .I_clk   (video_clk),
    .I_rst   (rst_all),
    .O_pulse (edid_trig)
);

// ===================== 带音频的 HDMI 1.4b 发射 =====================
hdmi_1_4b_transmitter_core_wrapper #(
    .DEVICE                 ( "EG"       ),
    .HTOTAL                 ( 800        ),
    .HSA                    ( 96         ),
    .HFP                    ( 16         ),
    .HBP                    ( 48         ),
    .HACTIVE                ( 640        ),
    .VTOTAL                 ( 525        ),
    .VSA                    ( 2          ),
    .VFP                    ( 10         ),
    .VBP                    ( 33         ),
    .VACTIVE                ( 480        ),
    .VIDEO_VIC              ( 1          ),
    .VIDEO_TPG              ( "Disable"  ),
    .VIDEO_FORMAT           ( "RGB"      ),
    .AUDIO_SAMPLE_RATE      ( "48K"      ),
    .IIC_SCL_DIV            ( 250        )
) u_hdmi_1_4b_transmitter_core_wrapper(
    .I_pixel_clk        (video_clk),
    .I_rst              (rst_all),
    .I_edid_read_trig   (edid_trig),
    .O_edid_read_valid  (edid_valid),
    .O_edid_read_data   (edid_data),

    .I_axis_s_user      (axis_s_user),
    .I_axis_s_valid     (axis_s_valid),
    .I_axis_s_last      (axis_s_last),
    .I_axis_s_data      (axis_s_data),
    .O_axis_s_ready     (axis_s_ready),

    .I_audio_valid      (audio_valid),
    .I_audio_left_data  (audio_left_data),
    .I_audio_right_data (audio_right_data),
    .I_acr_valid        (acr_valid),
    .I_acr_cts          (acr_cts),
    .I_acr_n            (acr_n),

    .O_video_locked     (),
    .O_ddc_scl          (HDMI_DDC_SCL),
    .IO_ddc_sda         (HDMI_DDC_SDA),

    .O_ch0_tmds_data    (tmds_ch0_data),
    .O_ch1_tmds_data    (tmds_ch1_data),
    .O_ch2_tmds_data    (tmds_ch2_data),
    .O_clk_tmds_data    (tmds_clk_data)
);

hdmi_phy_wrapper #(
    .DEVICE ( "EG" )
) u_hdmi2phy_wrapper(
    .I_pixel_clk        (video_clk),
    .I_serial_clk       (hdmi_5x_clk),
    .I_rst              (rst_all),
    .I_tmds_channel_0   (tmds_ch0_data),
    .I_tmds_channel_1   (tmds_ch1_data),
    .I_tmds_channel_2   (tmds_ch2_data),
    .I_tmds_channel_clk (tmds_clk_data),
    .O_tmds_ch0_p       (HDMI_D0_P),
    .O_tmds_ch1_p       (HDMI_D1_P),
    .O_tmds_ch2_p       (HDMI_D2_P),
    .O_tmds_clk_p       (HDMI_CLK_P)
);

endmodule

// ---------------------------------------------------------------------------
// Slice 5 helpers (kept in this file to avoid TD black-box of new .v files)
// ---------------------------------------------------------------------------

module fade_transition_ctrl (
    input  wire       I_clk,
    input  wire       I_rst,
    input  wire       I_vs,
    input  wire       I_fade_out_req,
    output reg        O_fade_swap_ok,
    output reg [7:0]  O_alpha,
    output reg        O_wipe_en,
    output reg [9:0]  O_wipe_x
);
localparam ST_IDLE      = 2'd0;
localparam ST_FADE_OUT  = 2'd1;
localparam ST_WAIT_SWAP = 2'd2;
localparam ST_WIPE_IN   = 2'd3; // slice 18: left unveil instead of fade-in
localparam [7:0] ALPHA_STEP = 8'd21; // ~12 frames fade-out @ 60Hz
localparam [9:0] WIPE_STEP  = 10'd54; // ~12 frames to cover 640

reg [1:0] state;
reg [2:0] req_sync;
reg [2:0] vs_sync;
wire      req_lvl  = req_sync[2];
wire      vs_rise  = (vs_sync[2] == 1'b0) && (vs_sync[1] == 1'b1);

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        state          <= ST_IDLE;
        O_alpha        <= 8'd255;
        O_fade_swap_ok <= 1'b0;
        O_wipe_en      <= 1'b0;
        O_wipe_x       <= 10'd640;
        req_sync       <= 3'b000;
        vs_sync        <= 3'b000;
    end else begin
        req_sync <= {req_sync[1:0], I_fade_out_req};
        vs_sync  <= {vs_sync[1:0], I_vs};

        case (state)
            ST_IDLE: begin
                O_fade_swap_ok <= 1'b0;
                O_alpha        <= 8'd255;
                O_wipe_en      <= 1'b0;
                O_wipe_x       <= 10'd640;
                if (req_lvl)
                    state <= ST_FADE_OUT;
            end
            ST_FADE_OUT: begin
                O_fade_swap_ok <= 1'b0;
                O_wipe_en      <= 1'b0;
                if (vs_rise) begin
                    if (O_alpha <= ALPHA_STEP) begin
                        O_alpha        <= 8'd0;
                        O_fade_swap_ok <= 1'b1;
                        state          <= ST_WAIT_SWAP;
                    end else begin
                        O_alpha <= O_alpha - ALPHA_STEP;
                    end
                end
            end
            ST_WAIT_SWAP: begin
                O_fade_swap_ok <= 1'b1;
                O_alpha        <= 8'd0;
                O_wipe_en      <= 1'b0;
                O_wipe_x       <= 10'd0;
                if (!req_lvl)
                    state <= ST_WIPE_IN;
            end
            ST_WIPE_IN: begin
                // Full brightness on new buffer; unveil left->right (single SDRAM read)
                O_fade_swap_ok <= 1'b0;
                O_alpha        <= 8'd255;
                O_wipe_en      <= 1'b1;
                if (vs_rise) begin
                    if (O_wipe_x >= (10'd640 - WIPE_STEP)) begin
                        O_wipe_x  <= 10'd640;
                        O_wipe_en <= 1'b0;
                        state     <= ST_IDLE;
                    end else begin
                        O_wipe_x <= O_wipe_x + WIPE_STEP;
                    end
                end
            end
            default: state <= ST_IDLE;
        endcase
    end
end
endmodule

// Slice 13: MTR-style header / warn / countdown rail / station bar (ASCII only).
module osd_metro_chrome (
    input wire [3:0] I_boot_status,
    input wire I_media_ready,I_temperature_update,
    input wire I_temperature_valid,I_temperature_alarm,
    input wire [3:0] I_temperature_tens,I_temperature_ones,
    input wire [2:0] I_media_index,input wire I_media_auto,input wire [5:0] I_media_available,
    input wire [5:0] I_scroll_speed,
    input wire [2:0] I_journey_stage,
    input wire I_clk, I_rst, I_hs, I_vs, I_de, I_enable, I_warn,
    input wire [9:0] I_fps_tenths,
    input wire I_fps_valid,
    input wire [23:0] I_rgb,
    output reg [23:0] O_rgb
);
// FONT32_TEXT_BEGIN
localparam integer FONT_ADDR_BITS = 12;

function [6:0] text_title;
    input [4:0] idx;
    begin
        case (idx)
            5'd0: text_title = 7'd14; // 城
            5'd1: text_title = 7'd15; // 市
            5'd2: text_title = 7'd16; // 轨
            5'd3: text_title = 7'd17; // 道
            5'd4: text_title = 7'd18; // 交
            5'd5: text_title = 7'd19; // 通
            default: text_title = 7'd0;
        endcase
    end
endfunction

function [6:0] text_warning;
    input [4:0] idx;
    begin
        case (idx)
            5'd0: text_warning = 7'd20; // 温
            5'd1: text_warning = 7'd21; // 度
            5'd2: text_warning = 7'd22; // 偏
            5'd3: text_warning = 7'd23; // 高
            default: text_warning = 7'd0;
        endcase
    end
endfunction

function [6:0] text_line;
    input [4:0] idx;
    begin
        case (idx)
            5'd0: text_line = 7'd2; // 1
            5'd1: text_line = 7'd24; // 号
            5'd2: text_line = 7'd25; // 线
            default: text_line = 7'd0;
        endcase
    end
endfunction

function [6:0] text_line_header;
    input [4:0] idx;
    begin
        case (idx)
            5'd0: text_line_header = 7'd25; // 线
            5'd1: text_line_header = 7'd26; // 路
            default: text_line_header = 7'd0;
        endcase
    end
endfunction

function [6:0] text_destination_header;
    input [4:0] idx;
    begin
        case (idx)
            5'd0: text_destination_header = 7'd27; // 开
            5'd1: text_destination_header = 7'd28; // 往
            default: text_destination_header = 7'd0;
        endcase
    end
endfunction

function [6:0] text_eta_header;
    input [4:0] idx;
    begin
        case (idx)
            5'd0: text_eta_header = 7'd29; // 预
            5'd1: text_eta_header = 7'd30; // 计
            5'd2: text_eta_header = 7'd31; // 到
            5'd3: text_eta_header = 7'd32; // 达
            default: text_eta_header = 7'd0;
        endcase
    end
endfunction

function [6:0] text_destination;
    input [4:0] idx;
    begin
        case (idx)
            5'd0: text_destination = 7'd33; // 科
            5'd1: text_destination = 7'd34; // 技
            5'd2: text_destination = 7'd35; // 园
            default: text_destination = 7'd0;
        endcase
    end
endfunction

function [6:0] text_next_train;
    input [4:0] idx;
    begin
        case (idx)
            5'd0: text_next_train = 7'd36; // 下
            5'd1: text_next_train = 7'd37; // 一
            5'd2: text_next_train = 7'd38; // 班
            default: text_next_train = 7'd0;
        endcase
    end
endfunction

function [6:0] text_unit;
    input [4:0] idx;
    begin
        case (idx)
            5'd0: text_unit = 7'd39; // 分
            5'd1: text_unit = 7'd40; // 钟
            default: text_unit = 7'd0;
        endcase
    end
endfunction

function [6:0] text_station;
    input [4:0] idx;
    begin
        case (idx)
            5'd0: text_station = 7'd41; // 当
            5'd1: text_station = 7'd42; // 前
            5'd2: text_station = 7'd43; // 站
            5'd3: text_station = 7'd44; // ：
            5'd4: text_station = 7'd45; // 中
            5'd5: text_station = 7'd46; // 心
            5'd6: text_station = 7'd43; // 站
            default: text_station = 7'd0;
        endcase
    end
endfunction

function [6:0] text_fps;
    input [4:0] idx;
    begin
        case (idx)
            5'd0: text_fps = 7'd47; // 帧
            5'd1: text_fps = 7'd48; // 率
            default: text_fps = 7'd0;
        endcase
    end
endfunction

function [6:0] text_ticker;
    input [4:0] idx;
    begin
        case (idx)
            5'd0: text_ticker = 7'd49; // 各
            5'd1: text_ticker = 7'd50; // 位
            5'd2: text_ticker = 7'd51; // 乘
            5'd3: text_ticker = 7'd52; // 客
            5'd4: text_ticker = 7'd53; // ，
            5'd5: text_ticker = 7'd54; // 列
            5'd6: text_ticker = 7'd55; // 车
            5'd7: text_ticker = 7'd56; // 即
            5'd8: text_ticker = 7'd57; // 将
            5'd9: text_ticker = 7'd58; // 进
            5'd10: text_ticker = 7'd43; // 站
            5'd11: text_ticker = 7'd53; // ，
            5'd12: text_ticker = 7'd59; // 请
            5'd13: text_ticker = 7'd43; // 站
            5'd14: text_ticker = 7'd60; // 在
            5'd15: text_ticker = 7'd61; // 黄
            5'd16: text_ticker = 7'd62; // 色
            5'd17: text_ticker = 7'd63; // 安
            5'd18: text_ticker = 7'd64; // 全
            5'd19: text_ticker = 7'd25; // 线
            5'd20: text_ticker = 7'd65; // 内
            5'd21: text_ticker = 7'd66; // 候
            5'd22: text_ticker = 7'd55; // 车
            5'd23: text_ticker = 7'd67; // 。
            5'd24: text_ticker = 7'd0; //  
            5'd25: text_ticker = 7'd0; //  
            5'd26: text_ticker = 7'd0; //  
            5'd27: text_ticker = 7'd0; //  
            5'd28: text_ticker = 7'd0; //  
            5'd29: text_ticker = 7'd0; //  
            5'd30: text_ticker = 7'd0; //  
            5'd31: text_ticker = 7'd0; //  
            default: text_ticker = 7'd0;
        endcase
    end
endfunction
function [6:0] journey_site_0;
    input [4:0] idx;
    begin
        case(idx)
            5'd0:journey_site_0=7'd33; // 科
            5'd1:journey_site_0=7'd34; // 技
            5'd2:journey_site_0=7'd35; // 园
            default:journey_site_0=0;
        endcase
    end
endfunction

function [6:0] journey_site_1;
    input [4:0] idx;
    begin
        case(idx)
            5'd0:journey_site_1=7'd68; // 大
            5'd1:journey_site_1=7'd69; // 学
            5'd2:journey_site_1=7'd14; // 城
            default:journey_site_1=0;
        endcase
    end
endfunction

function [6:0] journey_site_2;
    input [4:0] idx;
    begin
        case(idx)
            5'd0:journey_site_2=7'd70; // 美
            5'd1:journey_site_2=7'd71; // 食
            5'd2:journey_site_2=7'd72; // 街
            default:journey_site_2=0;
        endcase
    end
endfunction

function [6:0] journey_site;
 input [1:0] site;input [4:0] idx;
 begin case(site)
  0:journey_site=journey_site_0(idx);
  1:journey_site=journey_site_1(idx);
  default:journey_site=journey_site_2(idx);
 endcase end
endfunction

function [6:0] journey_arrived;
    input [4:0] idx;
    begin
        case(idx)
            5'd0:journey_arrived=7'd31; // 到
            5'd1:journey_arrived=7'd73; // 了
            default:journey_arrived=0;
        endcase
    end
endfunction

function [6:0] journey_soon;
    input [4:0] idx;
    begin
        case(idx)
            5'd0:journey_soon=7'd56; // 即
            5'd1:journey_soon=7'd57; // 将
            default:journey_soon=0;
        endcase
    end
endfunction

function [6:0] journey_departing;
    input [4:0] idx;
    begin
        case(idx)
            5'd0:journey_departing=7'd82; // 正
            5'd1:journey_departing=7'd60; // 在
            default:journey_departing=0;
        endcase
    end
endfunction

function [6:0] journey_exit;
    input [4:0] idx;
    begin
        case(idx)
            5'd0:journey_exit=7'd77; // 出
            5'd1:journey_exit=7'd43; // 站
            default:journey_exit=0;
        endcase
    end
endfunction

function [6:0] journey_next_site;
    input [4:0] idx;
    begin
        case(idx)
            5'd0:journey_next_site=7'd36; // 下
            5'd1:journey_next_site=7'd37; // 一
            5'd2:journey_next_site=7'd43; // 站
            default:journey_next_site=0;
        endcase
    end
endfunction

function [6:0] journey_ticker_1;
    input [4:0] idx;
    begin
        case(idx)
            5'd0:journey_ticker_1=7'd54; // 列
            5'd1:journey_ticker_1=7'd55; // 车
            5'd2:journey_ticker_1=7'd56; // 即
            5'd3:journey_ticker_1=7'd57; // 将
            5'd4:journey_ticker_1=7'd58; // 进
            5'd5:journey_ticker_1=7'd43; // 站
            default:journey_ticker_1=0;
        endcase
    end
endfunction

function [6:0] journey_ticker_2;
    input [4:0] idx;
    begin
        case(idx)
            5'd0:journey_ticker_2=7'd33; // 科
            5'd1:journey_ticker_2=7'd34; // 技
            5'd2:journey_ticker_2=7'd35; // 园
            5'd3:journey_ticker_2=7'd31; // 到
            5'd4:journey_ticker_2=7'd73; // 了
            5'd5:journey_ticker_2=7'd53; // ，
            5'd6:journey_ticker_2=7'd59; // 请
            5'd7:journey_ticker_2=7'd49; // 各
            5'd8:journey_ticker_2=7'd50; // 位
            5'd9:journey_ticker_2=7'd51; // 乘
            5'd10:journey_ticker_2=7'd52; // 客
            5'd11:journey_ticker_2=7'd74; // 有
            5'd12:journey_ticker_2=7'd75; // 序
            5'd13:journey_ticker_2=7'd76; // 上
            5'd14:journey_ticker_2=7'd36; // 下
            5'd15:journey_ticker_2=7'd55; // 车
            default:journey_ticker_2=0;
        endcase
    end
endfunction

function [6:0] journey_ticker_3;
    input [4:0] idx;
    begin
        case(idx)
            5'd0:journey_ticker_3=7'd54; // 列
            5'd1:journey_ticker_3=7'd55; // 车
            5'd2:journey_ticker_3=7'd56; // 即
            5'd3:journey_ticker_3=7'd57; // 将
            5'd4:journey_ticker_3=7'd77; // 出
            5'd5:journey_ticker_3=7'd43; // 站
            5'd6:journey_ticker_3=7'd53; // ，
            5'd7:journey_ticker_3=7'd59; // 请
            5'd8:journey_ticker_3=7'd78; // 未
            5'd9:journey_ticker_3=7'd76; // 上
            5'd10:journey_ticker_3=7'd55; // 车
            5'd11:journey_ticker_3=7'd79; // 的
            5'd12:journey_ticker_3=7'd51; // 乘
            5'd13:journey_ticker_3=7'd52; // 客
            5'd14:journey_ticker_3=7'd80; // 尽
            5'd15:journey_ticker_3=7'd81; // 快
            5'd16:journey_ticker_3=7'd76; // 上
            5'd17:journey_ticker_3=7'd55; // 车
            default:journey_ticker_3=0;
        endcase
    end
endfunction

function [6:0] journey_ticker_4;
    input [4:0] idx;
    begin
        case(idx)
            5'd0:journey_ticker_4=7'd54; // 列
            5'd1:journey_ticker_4=7'd55; // 车
            5'd2:journey_ticker_4=7'd82; // 正
            5'd3:journey_ticker_4=7'd60; // 在
            5'd4:journey_ticker_4=7'd77; // 出
            5'd5:journey_ticker_4=7'd43; // 站
            5'd6:journey_ticker_4=7'd53; // ，
            5'd7:journey_ticker_4=7'd59; // 请
            5'd8:journey_ticker_4=7'd83; // 勿
            5'd9:journey_ticker_4=7'd43; // 站
            5'd10:journey_ticker_4=7'd60; // 在
            5'd11:journey_ticker_4=7'd43; // 站
            5'd12:journey_ticker_4=7'd84; // 台
            5'd13:journey_ticker_4=7'd61; // 黄
            5'd14:journey_ticker_4=7'd25; // 线
            5'd15:journey_ticker_4=7'd65; // 内
            default:journey_ticker_4=0;
        endcase
    end
endfunction

function [6:0] journey_ticker_5;
    input [4:0] idx;
    begin
        case(idx)
            5'd0:journey_ticker_5=7'd36; // 下
            5'd1:journey_ticker_5=7'd37; // 一
            5'd2:journey_ticker_5=7'd43; // 站
            5'd3:journey_ticker_5=7'd53; // ，
            5'd4:journey_ticker_5=7'd68; // 大
            5'd5:journey_ticker_5=7'd69; // 学
            5'd6:journey_ticker_5=7'd14; // 城
            default:journey_ticker_5=0;
        endcase
    end
endfunction

function [6:0] journey_ticker;
 input [2:0] stage;input [4:0] idx;
 begin case(stage)
  1:journey_ticker=journey_ticker_1(idx);
  2:journey_ticker=journey_ticker_2(idx);
  3:journey_ticker=journey_ticker_3(idx);
  4:journey_ticker=journey_ticker_4(idx);
  5:journey_ticker=journey_ticker_5(idx);
  default:journey_ticker=text_ticker(idx);
 endcase end
endfunction
// FONT32_TEXT_END
reg [2:0] journey_stage_frame;
// Caption viewport is x=4..515. A 32-pixel first glyph is centered at x=260.
// Initial phase 784 = 1024 - (244 - 4), so glyph zero begins at x=244.
localparam [9:0] TICKER_START_PHASE=10'd784;
reg ticker_enabled_frame;
// Announcement text must reach its final characters during the 3 s hold.
// Preserve the selected speed in normal mode; automatic stages use >=32 px/s.
wire [5:0] journey_scroll_speed=(journey_stage_frame!=0 && I_scroll_speed<6'd32)?6'd32:I_scroll_speed;
reg temperature_valid_frame,temperature_alarm_frame,temperature_update_frame;
reg [3:0] temperature_tens_frame,temperature_ones_frame;
reg de_d, vs_d;
reg [9:0] pix_x, pix_y;
wire frame_start = vs_d && !I_vs;
reg [24:0] sec_div;
reg [5:0] second, minute;
reg [4:0] hour;
reg [2:0] eta;
reg [5:0] minute_frame;
reg [4:0] hour_frame;
reg [2:0] eta_frame;
reg [9:0] fps_frame;
reg fps_valid_frame, scroll_half;
reg [9:0] scroll_x;
reg [6:0] scroll_fraction;
// Simulated clock and arrivals. One displayed minute is one actual minute.
always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        journey_stage_frame<=0;ticker_enabled_frame<=0;
        temperature_valid_frame<=0;temperature_alarm_frame<=0;temperature_update_frame<=0;
        temperature_tens_frame<=0;temperature_ones_frame<=0;
        de_d<=0; vs_d<=0; pix_x<=0; pix_y<=0;
        sec_div<=0; second<=0; minute<=6'd3; hour<=5'd15; eta<=3'd5;
        minute_frame<=6'd3; hour_frame<=5'd15; eta_frame<=3'd5;
        fps_frame<=0; fps_valid_frame<=0; scroll_half<=0; scroll_x<=TICKER_START_PHASE; scroll_fraction<=0;
    end else begin
        de_d<=I_de; vs_d<=I_vs;
        if (frame_start) pix_y<=0;
        else if (de_d && !I_de) pix_y<=pix_y+10'd1;
        if (!I_de) pix_x<=0; else pix_x<=pix_x+10'd1;
        if (sec_div==25'd24999999) begin
            sec_div<=0;
            if (second==6'd59) begin
                second<=0;
                if (minute==6'd59) begin
                    minute<=0;
                    if (hour==5'd23) hour<=0; else hour<=hour+5'd1;
                end else minute<=minute+6'd1;
                if (eta==0) eta<=3'd5; else eta<=eta-3'd1;
            end else second<=second+6'd1;
        end else sec_div<=sec_div+25'd1;
        if (frame_start) begin
            // Latch all dynamic digits in blanking: no split-frame text update.
            temperature_valid_frame<=I_temperature_valid;temperature_alarm_frame<=I_temperature_alarm;
            temperature_update_frame<=I_temperature_update;
            temperature_tens_frame<=I_temperature_tens;temperature_ones_frame<=I_temperature_ones;
            minute_frame<=minute; hour_frame<=hour; eta_frame<=eta;
            fps_frame<=I_fps_tenths; fps_valid_frame<=I_fps_valid;
            if (I_enable) begin
                if(scroll_fraction+journey_scroll_speed>=60) begin
                    scroll_fraction<=scroll_fraction+journey_scroll_speed-60;scroll_x<=scroll_x+1;
                end else scroll_fraction<=scroll_fraction+journey_scroll_speed;
            end
        end
        // The top descriptor arrives after VS; capture throughout vertical
        // blanking so the stage is ready for this frame's first active pixel.
        if(!I_de && pix_y==0)begin
            journey_stage_frame<=I_journey_stage;
            ticker_enabled_frame<=I_enable;
            if(journey_stage_frame!=I_journey_stage || (!ticker_enabled_frame && I_enable))begin
                scroll_x<=TICKER_START_PHASE;scroll_fraction<=0;
            end
        end
    end
end
function [6:0] digit_id;
    input [9:0] digit;
    begin digit_id=7'd1+{3'd0,digit[3:0]}; end
endfunction
function [6:0] clock_id;
    input [4:0] idx;
    begin
        case (idx)
            0: clock_id=digit_id({5'd0,hour_frame}/10'd10);
            1: clock_id=digit_id({5'd0,hour_frame}%10'd10);
            2: clock_id=7'd11; // colon
            3: clock_id=digit_id({4'd0,minute_frame}/10'd10);
            4: clock_id=digit_id({4'd0,minute_frame}%10'd10);
            default: clock_id=0;
        endcase
    end
endfunction
function [6:0] eta_id;
    input [4:0] idx;
    input [9:0] minutes;
    begin
        case (idx)
            0: eta_id=digit_id(minutes/10'd10);
            1: eta_id=digit_id(minutes%10'd10);
            2: eta_id=text_unit(0);
            3: eta_id=text_unit(1);
            default: eta_id=0;
        endcase
    end
endfunction
function [6:0] fps_id;
    input [4:0] idx;
    begin
        case (idx)
            0: fps_id=text_fps(0);
            1: fps_id=text_fps(1);
            2: fps_id=fps_valid_frame ? digit_id(fps_frame/10'd100) : 7'd13;
            3: fps_id=fps_valid_frame ? digit_id((fps_frame/10'd10)%10'd10) : 7'd13;
            4: fps_id=7'd12; // decimal point
            5: fps_id=fps_valid_frame ? digit_id(fps_frame%10'd10) : 7'd13;
            default: fps_id=0;
        endcase
    end
endfunction
wire [9:0] title_x=pix_x-10'd8;
wire [9:0] header_line_x=pix_x-10'd240;
wire [9:0] clock_x=pix_x-10'd472;
wire [9:0] line_x=pix_x-10'd16;
wire [9:0] dest_x=pix_x-10'd160;
wire [9:0] eta_x=pix_x-10'd384;
wire [9:0] rail_x=pix_x-10'd528;
wire [9:0] rail_eta_x=pix_x-10'd548;
wire [9:0] station_line_x=pix_x-10'd352;
wire [9:0] temperature_x=pix_x-10'd240;
wire [9:0] temperature_y=pix_y-10'd444;
wire temperature_degree=(pix_x>=322 && pix_x<326 && pix_y>=445 && pix_y<449) &&
 ((pix_x==322 || pix_x==325) ? (pix_y==446 || pix_y==447) : (pix_y==445 || pix_y==448));
wire temperature_celsius=(pix_x>=330 && pix_x<340 && pix_y>=446 && pix_y<458) &&
 ((pix_y==446 || pix_y==447 || pix_y==456 || pix_y==457) ? (pix_x>=332) : (pix_x<332));
wire [9:0] fps_x=pix_x-10'd8;
wire [9:0] ticker_x=(pix_x-10'd4)+scroll_x; // wraps at 32*32=1024
reg [6:0] glyph_id;
reg [4:0] glyph_row, glyph_col;
reg text_enable;
function [34:0] tiny_font;
 input [7:0] ch;
 begin case(ch)
  8'd32:tiny_font=35'b00000000000000000000000000000000000;
  8'd65:tiny_font=35'b01110100011000111111100011000110001;
  8'd68:tiny_font=35'b11110100011000110001100011000111110;
  8'd69:tiny_font=35'b11111100001000011110100001000011111;
  8'd72:tiny_font=35'b10001100011000111111100011000110001;
  8'd73:tiny_font=35'b11111001000010000100001000010011111;
  8'd76:tiny_font=35'b10000100001000010000100001000011111;
  8'd77:tiny_font=35'b10001110111010110101100011000110001;
  8'd78:tiny_font=35'b10001110011100110101100111001110001;
  8'd79:tiny_font=35'b01110100011000110001100011000101110;
  8'd80:tiny_font=35'b11110100011000111110100001000010000;
  8'd84:tiny_font=35'b11111001000010000100001000010000100;
  8'd85:tiny_font=35'b10001100011000110001100011000101110;
  8'd86:tiny_font=35'b10001100011000110001100010101000100;
  8'd48:tiny_font=35'b01110100011001110101110011000101110;
  8'd49:tiny_font=35'b00100011000010000100001000010001110;
  8'd50:tiny_font=35'b01110100010000100010001000100011111;
  8'd51:tiny_font=35'b11110000010000101110000010000111110;
  8'd52:tiny_font=35'b00010001100101010010111110001000010;
  8'd53:tiny_font=35'b11111100001000011110000010000111110;
  8'd54:tiny_font=35'b01110100001000011110100011000101110;
  8'd55:tiny_font=35'b11111000010001000100010000100001000;
  default:tiny_font=0;
 endcase end
endfunction
function [7:0] media_letter;
 input [3:0] index;
 begin case(index)
  0:media_letter=I_media_index[2]?"V":"P";
  1:media_letter=I_media_index[2]?"I":"H";
  2:media_letter=I_media_index[2]?"D":"O";
  3:media_letter=I_media_index[2]?"E":"T";
  4:media_letter="O";5:media_letter=" ";6:media_letter="0";
  7:media_letter=8'd49+{6'd0,I_media_index[1:0]};default:media_letter=" ";
 endcase end
endfunction
function [7:0] mode_letter;
 input [3:0] index;
 begin
  if(I_media_auto)case(index)0:mode_letter="A";1:mode_letter="U";2:mode_letter="T";3:mode_letter="O";default:mode_letter=" ";endcase
  else case(index)0:mode_letter="M";1:mode_letter="A";2:mode_letter="N";3:mode_letter="U";4:mode_letter="A";5:mode_letter="L";default:mode_letter=" ";endcase
 end
endfunction
reg tiny_enable;reg [6:0] tiny_x;reg [3:0] tiny_y;reg [7:0] tiny_char;
wire [34:0] tiny_bits=tiny_font(tiny_char);
wire tiny_pixel=tiny_enable && tiny_x%12<10 && tiny_bits[(6-tiny_y[3:1])*5+(4-(tiny_x%12)/2)];
integer pager;

reg [23:0] base_rgb, text_rgb;
wire [23:0] dark_rgb={1'b0,I_rgb[23:17],1'b0,I_rgb[15:9],1'b0,I_rgb[7:1]};
always @(*) begin
    base_rgb=I_rgb; text_rgb=24'h233D52;
    tiny_enable=0;tiny_x=0;tiny_y=0;tiny_char=" ";
    glyph_id=0; glyph_row=0; glyph_col=0; text_enable=0;
    if (I_enable && I_de) begin
        if (pix_y<32) begin
            base_rgb=I_warn ? 24'hA85D16 : 24'h142D45;
            text_rgb=24'hFFFFFF;
            if (pix_x>=8 && pix_x<200) begin
                text_enable=1; glyph_id=I_warn ? text_warning(title_x[9:5]) : text_title(title_x[9:5]);
                glyph_col=title_x[4:0]; glyph_row=pix_y[4:0];
            end else if (pix_x>=240 && pix_x<336) begin
                text_enable=1; glyph_id=text_line(header_line_x[9:5]);
                glyph_col=header_line_x[4:0]; glyph_row=pix_y[4:0];
            end else if (pix_x>=472 && pix_x<632) begin
                text_enable=1; glyph_id=clock_id(clock_x[9:5]); text_rgb=24'hF5BD53;
                glyph_col=clock_x[4:0]; glyph_row=pix_y[4:0];
            end
        end else if (pix_y<132 && pix_x<520) begin
            if (pix_y<64)begin base_rgb=24'h34566B;text_rgb=24'hEAF2F5;end
            else if (pix_y<96)base_rgb=24'hFFF4E5;
            else base_rgb=24'hE5F1EC;
            // Three native 32-pixel rows occupy y=32..127; y=128..131 is padding.
            if (pix_y<128) begin
                glyph_row=pix_y[4:0];
                if (pix_x>=16 && pix_x<112) begin
                    text_enable=1; glyph_col=line_x[4:0];
                    glyph_id=(pix_y<64) ? text_line_header(line_x[9:5]) : text_line(line_x[9:5]);
                end else if (pix_x>=160 && pix_x<256) begin
                    text_enable=1; glyph_col=dest_x[4:0];
                    glyph_id=(pix_y<64) ? text_destination_header(dest_x[9:5]) : journey_site(journey_stage_frame==5?((pix_y<96)?2'd1:2'd2):((pix_y<96)?2'd0:2'd1),dest_x[9:5]);
                end else if (pix_x>=384 && pix_x<512) begin
                    text_enable=1; glyph_col=eta_x[4:0];
                    if (pix_y<64) glyph_id=text_eta_header(eta_x[9:5]);
                    else begin glyph_id=eta_id(eta_x[9:5],journey_stage_frame==5?((pix_y<96)?10'd4:10'd10):((pix_y<96)?10'd1:10'd5));text_rgb=24'hA65C22;end
                end
            end
        end else if (pix_y>=424 && pix_y<480) begin
            base_rgb=24'hDFEBF1;
            if (pix_y>=436 && pix_y<468) begin
                glyph_row=pix_y-10'd436;
                if (pix_x>=8 && pix_x<232) begin
                    text_enable=1; glyph_id=text_station(title_x[9:5]); glyph_col=title_x[4:0];
                end else if (pix_x>=352 && pix_x<448) begin
                    text_enable=1; glyph_id=text_line(station_line_x[9:5]); glyph_col=station_line_x[4:0];
                end else if (pix_x>=472 && pix_x<632) begin
                    text_enable=1; glyph_id=clock_id(clock_x[9:5]); glyph_col=clock_x[4:0];
                end
            end
            // Existing font ROM at half size. Fixed 16-pixel cells fit the
            // unused center of the footer; main station/line/clock stay intact.
            if(pix_x>=240 && pix_x<320 && pix_y>=444 && pix_y<460)begin
                text_enable=1;glyph_col={temperature_x[3:0],1'b0};glyph_row={temperature_y[3:0],1'b0};
                text_rgb=!temperature_valid_frame?24'h71818C:temperature_alarm_frame?24'hB51F2A:24'h1C6573;
                case(temperature_x[6:4])
                 0:glyph_id=7'd20; // 温
                 1:glyph_id=7'd21; // 度
                 2:glyph_id=7'd11; // :
                 3:glyph_id=temperature_valid_frame?digit_id({6'd0,temperature_tens_frame}):7'd13;
                 4:glyph_id=temperature_valid_frame?digit_id({6'd0,temperature_ones_frame}):7'd13;
                 default:glyph_id=0;
                endcase
            end
            if(temperature_degree || temperature_celsius)
                base_rgb=!temperature_valid_frame?24'h71818C:temperature_alarm_frame?24'hB51F2A:24'h1C6573;
            // Each valid sensor transaction changes this dot, even if the integer
            // temperature is unchanged. Commit its phase with the footer digits.
            if(pix_x>=344 && pix_x<348 && pix_y>=450 && pix_y<454)
                base_rgb=!temperature_valid_frame?24'h71818C:temperature_update_frame?24'h16836F:24'h91AAA4;
        end else if (pix_x>=520 && pix_y<424) begin
            base_rgb=(pix_x==520) ? 24'hD5A552 : 24'hF5F0E8;
            if (pix_x>=528 && pix_x<624 && pix_y>=152 && pix_y<184) begin
                text_enable=1;glyph_col=rail_x[4:0];glyph_row=pix_y-10'd152;
                case(journey_stage_frame)
                 2:glyph_id=journey_site(0,rail_x[9:5]);
                 3:glyph_id=journey_soon(rail_x[9:5]);
                 4:glyph_id=journey_departing(rail_x[9:5]);
                 5:glyph_id=journey_next_site(rail_x[9:5]);
                 default:glyph_id=text_next_train(rail_x[9:5]);
                endcase
            end else if (pix_x>=528 && pix_x<624 && pix_y>=192 && pix_y<224) begin
                text_enable=1;glyph_col=rail_x[4:0];glyph_row=pix_y-10'd192;
                case(journey_stage_frame)
                 2:glyph_id=journey_arrived(rail_x[9:5]);
                 3,4:glyph_id=journey_exit(rail_x[9:5]);
                 5:glyph_id=journey_site(1,rail_x[9:5]);
                 default:glyph_id=journey_site(0,rail_x[9:5]);
                endcase
            end else if (journey_stage_frame>=3 && journey_stage_frame<=4 && pix_x>=528 && pix_x<624 && pix_y>=240 && pix_y<272)begin
                text_enable=1;glyph_id=journey_site(0,rail_x[9:5]);glyph_col=rail_x[4:0];glyph_row=pix_y-10'd240;
            end else if ((journey_stage_frame<2 || journey_stage_frame==5) && pix_x>=548 && pix_x<612 && pix_y>=240 && pix_y<272) begin
                text_enable=1;glyph_id=(rail_eta_x[9:5]==0)?digit_id(10'd0):digit_id(journey_stage_frame==5?10'd4:10'd1);
                glyph_col=rail_eta_x[4:0];glyph_row=pix_y-10'd240;text_rgb=24'hA65C22;
            end else if ((journey_stage_frame<2 || journey_stage_frame==5) && pix_x>=548 && pix_x<612 && pix_y>=280 && pix_y<312) begin
                text_enable=1;glyph_id=text_unit(rail_eta_x[9:5]);glyph_col=rail_eta_x[4:0];glyph_row=pix_y-10'd280;
            end
        end else if (1'b0 && pix_x>=4 && pix_x<204 && pix_y>=136 && pix_y<172) begin
            base_rgb=dark_rgb; text_rgb=24'hFFFFFF;
            if (pix_x>=8 && pix_x<200 && pix_y>=138 && pix_y<170) begin
                text_enable=1; glyph_id=fps_id(fps_x[9:5]); glyph_col=fps_x[4:0]; glyph_row=pix_y-10'd138;
            end
        end else if (pix_x>=4 && pix_x<516 && pix_y>=336 && pix_y<372) begin
            base_rgb=dark_rgb; text_rgb=24'hF5BD53;
            if (pix_y>=338 && pix_y<370) begin
                text_enable=1; glyph_id=journey_ticker(journey_stage_frame,ticker_x[9:5]); glyph_col=ticker_x[4:0]; glyph_row=pix_y-10'd338;
            end
        end
        if(pix_y==30 || pix_y==31 || pix_y==130 || pix_y==131 || pix_y==424 || pix_y==425)begin
            base_rgb=24'hD5A552;text_enable=0;
        end
        if(pix_x>=232 && pix_x<344 && pix_y<30)begin
            base_rgb=24'h16836F;
        end
        if(pix_x<520 && (pix_y==63 || pix_y==95 || pix_y==127))begin
            base_rgb=24'hBED4D7;text_enable=0;
        end
        if(pix_x>=528 && pix_x<632 && pix_y>=42 && pix_y<68)base_rgb=24'hE3DDF6;
        if(pix_x>=532 && pix_x<628 && pix_y>=48 && pix_y<62)begin
            tiny_enable=1;tiny_x=pix_x-532;tiny_y=pix_y-48;tiny_char=media_letter(tiny_x/12);
        end
        if(pix_x>=532 && pix_x<628 && pix_y>=86 && pix_y<100)begin
            tiny_enable=1;tiny_x=pix_x-532;tiny_y=pix_y-86;tiny_char=mode_letter(tiny_x/12);
        end
        for(pager=0;pager<5;pager=pager+1)begin
            if(pix_x>=534+pager*15 && pix_x<541+pager*15 && pix_y>=114 && pix_y<118)
                base_rgb=I_media_index==pager?24'hE87959:I_media_available[pager]?24'h8098A5:24'hD9DFDE;
        end
        if(pix_x==527 && pix_y>=144 && pix_y<320)base_rgb=24'hD5A552;
        if(pix_y==326 && pix_x>=528 && pix_x<632)base_rgb=24'hD5A552;
        // Show a compact progress code only before the first valid media frame.
        // The normal page and emergency poster remain free of diagnostics.
        if(!I_media_ready && pix_x>=8 && pix_x<104 && pix_y>=176 && pix_y<202)begin
            base_rgb=24'hE3DDF6;text_enable=0;
            if(pix_x>=20 && pix_x<92 && pix_y>=182 && pix_y<196)begin
                tiny_enable=1;tiny_x=pix_x-20;tiny_y=pix_y-182;
                case(tiny_x/12)
                 0:tiny_char="I";1:tiny_char="N";2:tiny_char="I";3:tiny_char="T";
                 5:tiny_char=8'd48+I_boot_status;default:tiny_char=" ";
                endcase
            end
        end
        if(tiny_pixel)base_rgb=24'h59438C;
    end
end
// Three registered stages: address/metadata, synchronous BRAM row, RGB mix.
// Background, enable, column and color follow exactly the same pixel as the ROM.
// Lossless row dictionary uses the existing address stage, no extra latency.
(* rom_style="bram" *) reg [9:0] font_index[0:4095];
initial begin
font_index[0]=10'd0;
font_index[1]=10'd0;
font_index[2]=10'd0;
font_index[3]=10'd0;
font_index[4]=10'd0;
font_index[5]=10'd0;
font_index[6]=10'd0;
font_index[7]=10'd0;
font_index[8]=10'd0;
font_index[9]=10'd0;
font_index[10]=10'd0;
font_index[11]=10'd0;
font_index[12]=10'd0;
font_index[13]=10'd0;
font_index[14]=10'd0;
font_index[15]=10'd0;
font_index[16]=10'd0;
font_index[17]=10'd0;
font_index[18]=10'd0;
font_index[19]=10'd0;
font_index[20]=10'd0;
font_index[21]=10'd0;
font_index[22]=10'd0;
font_index[23]=10'd0;
font_index[24]=10'd0;
font_index[25]=10'd0;
font_index[26]=10'd0;
font_index[27]=10'd0;
font_index[28]=10'd0;
font_index[29]=10'd0;
font_index[30]=10'd0;
font_index[31]=10'd0;
font_index[32]=10'd0;
font_index[33]=10'd0;
font_index[34]=10'd0;
font_index[35]=10'd0;
font_index[36]=10'd1;
font_index[37]=10'd2;
font_index[38]=10'd3;
font_index[39]=10'd4;
font_index[40]=10'd5;
font_index[41]=10'd6;
font_index[42]=10'd7;
font_index[43]=10'd8;
font_index[44]=10'd8;
font_index[45]=10'd8;
font_index[46]=10'd8;
font_index[47]=10'd8;
font_index[48]=10'd8;
font_index[49]=10'd8;
font_index[50]=10'd8;
font_index[51]=10'd9;
font_index[52]=10'd5;
font_index[53]=10'd10;
font_index[54]=10'd3;
font_index[55]=10'd2;
font_index[56]=10'd11;
font_index[57]=10'd0;
font_index[58]=10'd0;
font_index[59]=10'd0;
font_index[60]=10'd0;
font_index[61]=10'd0;
font_index[62]=10'd0;
font_index[63]=10'd0;
font_index[64]=10'd0;
font_index[65]=10'd0;
font_index[66]=10'd0;
font_index[67]=10'd0;
font_index[68]=10'd12;
font_index[69]=10'd13;
font_index[70]=10'd14;
font_index[71]=10'd15;
font_index[72]=10'd15;
font_index[73]=10'd16;
font_index[74]=10'd17;
font_index[75]=10'd18;
font_index[76]=10'd18;
font_index[77]=10'd18;
font_index[78]=10'd18;
font_index[79]=10'd18;
font_index[80]=10'd18;
font_index[81]=10'd18;
font_index[82]=10'd18;
font_index[83]=10'd18;
font_index[84]=10'd18;
font_index[85]=10'd18;
font_index[86]=10'd18;
font_index[87]=10'd18;
font_index[88]=10'd18;
font_index[89]=10'd0;
font_index[90]=10'd0;
font_index[91]=10'd0;
font_index[92]=10'd0;
font_index[93]=10'd0;
font_index[94]=10'd0;
font_index[95]=10'd0;
font_index[96]=10'd0;
font_index[97]=10'd0;
font_index[98]=10'd0;
font_index[99]=10'd0;
font_index[100]=10'd14;
font_index[101]=10'd19;
font_index[102]=10'd20;
font_index[103]=10'd21;
font_index[104]=10'd22;
font_index[105]=10'd23;
font_index[106]=10'd23;
font_index[107]=10'd23;
font_index[108]=10'd24;
font_index[109]=10'd25;
font_index[110]=10'd26;
font_index[111]=10'd1;
font_index[112]=10'd11;
font_index[113]=10'd27;
font_index[114]=10'd28;
font_index[115]=10'd29;
font_index[116]=10'd30;
font_index[117]=10'd31;
font_index[118]=10'd31;
font_index[119]=10'd31;
font_index[120]=10'd31;
font_index[121]=10'd0;
font_index[122]=10'd0;
font_index[123]=10'd0;
font_index[124]=10'd0;
font_index[125]=10'd0;
font_index[126]=10'd0;
font_index[127]=10'd0;
font_index[128]=10'd0;
font_index[129]=10'd0;
font_index[130]=10'd0;
font_index[131]=10'd0;
font_index[132]=10'd14;
font_index[133]=10'd32;
font_index[134]=10'd33;
font_index[135]=10'd3;
font_index[136]=10'd24;
font_index[137]=10'd23;
font_index[138]=10'd23;
font_index[139]=10'd23;
font_index[140]=10'd25;
font_index[141]=10'd34;
font_index[142]=10'd14;
font_index[143]=10'd2;
font_index[144]=10'd35;
font_index[145]=10'd36;
font_index[146]=10'd37;
font_index[147]=10'd37;
font_index[148]=10'd38;
font_index[149]=10'd39;
font_index[150]=10'd21;
font_index[151]=10'd20;
font_index[152]=10'd40;
font_index[153]=10'd0;
font_index[154]=10'd0;
font_index[155]=10'd0;
font_index[156]=10'd0;
font_index[157]=10'd0;
font_index[158]=10'd0;
font_index[159]=10'd0;
font_index[160]=10'd0;
font_index[161]=10'd0;
font_index[162]=10'd0;
font_index[163]=10'd0;
font_index[164]=10'd23;
font_index[165]=10'd24;
font_index[166]=10'd41;
font_index[167]=10'd41;
font_index[168]=10'd35;
font_index[169]=10'd42;
font_index[170]=10'd43;
font_index[171]=10'd44;
font_index[172]=10'd45;
font_index[173]=10'd46;
font_index[174]=10'd47;
font_index[175]=10'd48;
font_index[176]=10'd49;
font_index[177]=10'd50;
font_index[178]=10'd50;
font_index[179]=10'd50;
font_index[180]=10'd23;
font_index[181]=10'd23;
font_index[182]=10'd23;
font_index[183]=10'd23;
font_index[184]=10'd23;
font_index[185]=10'd0;
font_index[186]=10'd0;
font_index[187]=10'd0;
font_index[188]=10'd0;
font_index[189]=10'd0;
font_index[190]=10'd0;
font_index[191]=10'd0;
font_index[192]=10'd0;
font_index[193]=10'd0;
font_index[194]=10'd0;
font_index[195]=10'd0;
font_index[196]=10'd51;
font_index[197]=10'd51;
font_index[198]=10'd51;
font_index[199]=10'd3;
font_index[200]=10'd52;
font_index[201]=10'd52;
font_index[202]=10'd52;
font_index[203]=10'd52;
font_index[204]=10'd32;
font_index[205]=10'd33;
font_index[206]=10'd3;
font_index[207]=10'd53;
font_index[208]=10'd36;
font_index[209]=10'd37;
font_index[210]=10'd37;
font_index[211]=10'd37;
font_index[212]=10'd54;
font_index[213]=10'd55;
font_index[214]=10'd3;
font_index[215]=10'd33;
font_index[216]=10'd11;
font_index[217]=10'd0;
font_index[218]=10'd0;
font_index[219]=10'd0;
font_index[220]=10'd0;
font_index[221]=10'd0;
font_index[222]=10'd0;
font_index[223]=10'd0;
font_index[224]=10'd0;
font_index[225]=10'd0;
font_index[226]=10'd0;
font_index[227]=10'd0;
font_index[228]=10'd56;
font_index[229]=10'd57;
font_index[230]=10'd58;
font_index[231]=10'd59;
font_index[232]=10'd60;
font_index[233]=10'd52;
font_index[234]=10'd52;
font_index[235]=10'd30;
font_index[236]=10'd61;
font_index[237]=10'd62;
font_index[238]=10'd63;
font_index[239]=10'd64;
font_index[240]=10'd65;
font_index[241]=10'd66;
font_index[242]=10'd66;
font_index[243]=10'd66;
font_index[244]=10'd67;
font_index[245]=10'd53;
font_index[246]=10'd68;
font_index[247]=10'd69;
font_index[248]=10'd1;
font_index[249]=10'd0;
font_index[250]=10'd0;
font_index[251]=10'd0;
font_index[252]=10'd0;
font_index[253]=10'd0;
font_index[254]=10'd0;
font_index[255]=10'd0;
font_index[256]=10'd0;
font_index[257]=10'd0;
font_index[258]=10'd0;
font_index[259]=10'd0;
font_index[260]=10'd70;
font_index[261]=10'd70;
font_index[262]=10'd70;
font_index[263]=10'd70;
font_index[264]=10'd37;
font_index[265]=10'd23;
font_index[266]=10'd24;
font_index[267]=10'd71;
font_index[268]=10'd25;
font_index[269]=10'd72;
font_index[270]=10'd73;
font_index[271]=10'd73;
font_index[272]=10'd13;
font_index[273]=10'd13;
font_index[274]=10'd74;
font_index[275]=10'd75;
font_index[276]=10'd75;
font_index[277]=10'd75;
font_index[278]=10'd76;
font_index[279]=10'd76;
font_index[280]=10'd76;
font_index[281]=10'd0;
font_index[282]=10'd0;
font_index[283]=10'd0;
font_index[284]=10'd0;
font_index[285]=10'd0;
font_index[286]=10'd0;
font_index[287]=10'd0;
font_index[288]=10'd0;
font_index[289]=10'd0;
font_index[290]=10'd0;
font_index[291]=10'd0;
font_index[292]=10'd13;
font_index[293]=10'd2;
font_index[294]=10'd3;
font_index[295]=10'd10;
font_index[296]=10'd5;
font_index[297]=10'd5;
font_index[298]=10'd5;
font_index[299]=10'd77;
font_index[300]=10'd3;
font_index[301]=10'd2;
font_index[302]=10'd2;
font_index[303]=10'd3;
font_index[304]=10'd5;
font_index[305]=10'd8;
font_index[306]=10'd8;
font_index[307]=10'd8;
font_index[308]=10'd78;
font_index[309]=10'd79;
font_index[310]=10'd63;
font_index[311]=10'd3;
font_index[312]=10'd80;
font_index[313]=10'd0;
font_index[314]=10'd0;
font_index[315]=10'd0;
font_index[316]=10'd0;
font_index[317]=10'd0;
font_index[318]=10'd0;
font_index[319]=10'd0;
font_index[320]=10'd0;
font_index[321]=10'd0;
font_index[322]=10'd0;
font_index[323]=10'd0;
font_index[324]=10'd11;
font_index[325]=10'd32;
font_index[326]=10'd20;
font_index[327]=10'd21;
font_index[328]=10'd81;
font_index[329]=10'd82;
font_index[330]=10'd82;
font_index[331]=10'd82;
font_index[332]=10'd9;
font_index[333]=10'd83;
font_index[334]=10'd63;
font_index[335]=10'd53;
font_index[336]=10'd84;
font_index[337]=10'd37;
font_index[338]=10'd85;
font_index[339]=10'd23;
font_index[340]=10'd24;
font_index[341]=10'd86;
font_index[342]=10'd19;
font_index[343]=10'd15;
font_index[344]=10'd87;
font_index[345]=10'd0;
font_index[346]=10'd0;
font_index[347]=10'd0;
font_index[348]=10'd0;
font_index[349]=10'd0;
font_index[350]=10'd0;
font_index[351]=10'd0;
font_index[352]=10'd0;
font_index[353]=10'd0;
font_index[354]=10'd0;
font_index[355]=10'd0;
font_index[356]=10'd0;
font_index[357]=10'd0;
font_index[358]=10'd0;
font_index[359]=10'd0;
font_index[360]=10'd0;
font_index[361]=10'd0;
font_index[362]=10'd88;
font_index[363]=10'd75;
font_index[364]=10'd75;
font_index[365]=10'd75;
font_index[366]=10'd88;
font_index[367]=10'd0;
font_index[368]=10'd0;
font_index[369]=10'd0;
font_index[370]=10'd0;
font_index[371]=10'd0;
font_index[372]=10'd88;
font_index[373]=10'd75;
font_index[374]=10'd75;
font_index[375]=10'd75;
font_index[376]=10'd88;
font_index[377]=10'd0;
font_index[378]=10'd0;
font_index[379]=10'd0;
font_index[380]=10'd0;
font_index[381]=10'd0;
font_index[382]=10'd0;
font_index[383]=10'd0;
font_index[384]=10'd0;
font_index[385]=10'd0;
font_index[386]=10'd0;
font_index[387]=10'd0;
font_index[388]=10'd0;
font_index[389]=10'd0;
font_index[390]=10'd0;
font_index[391]=10'd0;
font_index[392]=10'd0;
font_index[393]=10'd0;
font_index[394]=10'd0;
font_index[395]=10'd0;
font_index[396]=10'd0;
font_index[397]=10'd0;
font_index[398]=10'd0;
font_index[399]=10'd0;
font_index[400]=10'd0;
font_index[401]=10'd0;
font_index[402]=10'd0;
font_index[403]=10'd0;
font_index[404]=10'd88;
font_index[405]=10'd75;
font_index[406]=10'd75;
font_index[407]=10'd75;
font_index[408]=10'd88;
font_index[409]=10'd0;
font_index[410]=10'd0;
font_index[411]=10'd0;
font_index[412]=10'd0;
font_index[413]=10'd0;
font_index[414]=10'd0;
font_index[415]=10'd0;
font_index[416]=10'd0;
font_index[417]=10'd0;
font_index[418]=10'd0;
font_index[419]=10'd0;
font_index[420]=10'd0;
font_index[421]=10'd0;
font_index[422]=10'd0;
font_index[423]=10'd0;
font_index[424]=10'd0;
font_index[425]=10'd0;
font_index[426]=10'd0;
font_index[427]=10'd0;
font_index[428]=10'd0;
font_index[429]=10'd0;
font_index[430]=10'd0;
font_index[431]=10'd34;
font_index[432]=10'd34;
font_index[433]=10'd34;
font_index[434]=10'd0;
font_index[435]=10'd0;
font_index[436]=10'd0;
font_index[437]=10'd0;
font_index[438]=10'd0;
font_index[439]=10'd0;
font_index[440]=10'd0;
font_index[441]=10'd0;
font_index[442]=10'd0;
font_index[443]=10'd0;
font_index[444]=10'd0;
font_index[445]=10'd0;
font_index[446]=10'd0;
font_index[447]=10'd0;
font_index[448]=10'd0;
font_index[449]=10'd89;
font_index[450]=10'd90;
font_index[451]=10'd91;
font_index[452]=10'd92;
font_index[453]=10'd93;
font_index[454]=10'd94;
font_index[455]=10'd94;
font_index[456]=10'd95;
font_index[457]=10'd96;
font_index[458]=10'd96;
font_index[459]=10'd96;
font_index[460]=10'd97;
font_index[461]=10'd98;
font_index[462]=10'd98;
font_index[463]=10'd99;
font_index[464]=10'd100;
font_index[465]=10'd100;
font_index[466]=10'd101;
font_index[467]=10'd102;
font_index[468]=10'd103;
font_index[469]=10'd104;
font_index[470]=10'd105;
font_index[471]=10'd106;
font_index[472]=10'd107;
font_index[473]=10'd108;
font_index[474]=10'd109;
font_index[475]=10'd110;
font_index[476]=10'd111;
font_index[477]=10'd112;
font_index[478]=10'd0;
font_index[479]=10'd0;
font_index[480]=10'd0;
font_index[481]=10'd0;
font_index[482]=10'd113;
font_index[483]=10'd75;
font_index[484]=10'd75;
font_index[485]=10'd75;
font_index[486]=10'd114;
font_index[487]=10'd114;
font_index[488]=10'd114;
font_index[489]=10'd114;
font_index[490]=10'd74;
font_index[491]=10'd74;
font_index[492]=10'd115;
font_index[493]=10'd115;
font_index[494]=10'd115;
font_index[495]=10'd115;
font_index[496]=10'd116;
font_index[497]=10'd116;
font_index[498]=10'd116;
font_index[499]=10'd116;
font_index[500]=10'd116;
font_index[501]=10'd116;
font_index[502]=10'd117;
font_index[503]=10'd118;
font_index[504]=10'd119;
font_index[505]=10'd120;
font_index[506]=10'd121;
font_index[507]=10'd74;
font_index[508]=10'd74;
font_index[509]=10'd0;
font_index[510]=10'd0;
font_index[511]=10'd0;
font_index[512]=10'd0;
font_index[513]=10'd0;
font_index[514]=10'd122;
font_index[515]=10'd123;
font_index[516]=10'd123;
font_index[517]=10'd124;
font_index[518]=10'd124;
font_index[519]=10'd125;
font_index[520]=10'd126;
font_index[521]=10'd127;
font_index[522]=10'd127;
font_index[523]=10'd128;
font_index[524]=10'd129;
font_index[525]=10'd130;
font_index[526]=10'd130;
font_index[527]=10'd131;
font_index[528]=10'd132;
font_index[529]=10'd132;
font_index[530]=10'd133;
font_index[531]=10'd134;
font_index[532]=10'd135;
font_index[533]=10'd136;
font_index[534]=10'd137;
font_index[535]=10'd138;
font_index[536]=10'd139;
font_index[537]=10'd139;
font_index[538]=10'd140;
font_index[539]=10'd141;
font_index[540]=10'd142;
font_index[541]=10'd0;
font_index[542]=10'd0;
font_index[543]=10'd0;
font_index[544]=10'd0;
font_index[545]=10'd0;
font_index[546]=10'd143;
font_index[547]=10'd144;
font_index[548]=10'd145;
font_index[549]=10'd146;
font_index[550]=10'd147;
font_index[551]=10'd148;
font_index[552]=10'd149;
font_index[553]=10'd150;
font_index[554]=10'd151;
font_index[555]=10'd151;
font_index[556]=10'd152;
font_index[557]=10'd153;
font_index[558]=10'd152;
font_index[559]=10'd152;
font_index[560]=10'd154;
font_index[561]=10'd155;
font_index[562]=10'd155;
font_index[563]=10'd156;
font_index[564]=10'd156;
font_index[565]=10'd154;
font_index[566]=10'd156;
font_index[567]=10'd157;
font_index[568]=10'd158;
font_index[569]=10'd159;
font_index[570]=10'd160;
font_index[571]=10'd161;
font_index[572]=10'd162;
font_index[573]=10'd0;
font_index[574]=10'd0;
font_index[575]=10'd0;
font_index[576]=10'd0;
font_index[577]=10'd0;
font_index[578]=10'd113;
font_index[579]=10'd75;
font_index[580]=10'd75;
font_index[581]=10'd114;
font_index[582]=10'd114;
font_index[583]=10'd114;
font_index[584]=10'd114;
font_index[585]=10'd163;
font_index[586]=10'd164;
font_index[587]=10'd165;
font_index[588]=10'd166;
font_index[589]=10'd167;
font_index[590]=10'd168;
font_index[591]=10'd169;
font_index[592]=10'd170;
font_index[593]=10'd171;
font_index[594]=10'd172;
font_index[595]=10'd33;
font_index[596]=10'd34;
font_index[597]=10'd14;
font_index[598]=10'd34;
font_index[599]=10'd63;
font_index[600]=10'd173;
font_index[601]=10'd174;
font_index[602]=10'd175;
font_index[603]=10'd176;
font_index[604]=10'd177;
font_index[605]=10'd0;
font_index[606]=10'd0;
font_index[607]=10'd0;
font_index[608]=10'd0;
font_index[609]=10'd0;
font_index[610]=10'd178;
font_index[611]=10'd179;
font_index[612]=10'd180;
font_index[613]=10'd181;
font_index[614]=10'd182;
font_index[615]=10'd183;
font_index[616]=10'd184;
font_index[617]=10'd151;
font_index[618]=10'd151;
font_index[619]=10'd151;
font_index[620]=10'd185;
font_index[621]=10'd152;
font_index[622]=10'd152;
font_index[623]=10'd186;
font_index[624]=10'd187;
font_index[625]=10'd156;
font_index[626]=10'd156;
font_index[627]=10'd188;
font_index[628]=10'd189;
font_index[629]=10'd190;
font_index[630]=10'd191;
font_index[631]=10'd192;
font_index[632]=10'd193;
font_index[633]=10'd114;
font_index[634]=10'd160;
font_index[635]=10'd161;
font_index[636]=10'd194;
font_index[637]=10'd0;
font_index[638]=10'd0;
font_index[639]=10'd0;
font_index[640]=10'd0;
font_index[641]=10'd0;
font_index[642]=10'd178;
font_index[643]=10'd195;
font_index[644]=10'd196;
font_index[645]=10'd197;
font_index[646]=10'd198;
font_index[647]=10'd199;
font_index[648]=10'd200;
font_index[649]=10'd201;
font_index[650]=10'd202;
font_index[651]=10'd203;
font_index[652]=10'd204;
font_index[653]=10'd205;
font_index[654]=10'd206;
font_index[655]=10'd207;
font_index[656]=10'd0;
font_index[657]=10'd208;
font_index[658]=10'd209;
font_index[659]=10'd210;
font_index[660]=10'd211;
font_index[661]=10'd211;
font_index[662]=10'd211;
font_index[663]=10'd211;
font_index[664]=10'd212;
font_index[665]=10'd213;
font_index[666]=10'd213;
font_index[667]=10'd213;
font_index[668]=10'd178;
font_index[669]=10'd0;
font_index[670]=10'd0;
font_index[671]=10'd0;
font_index[672]=10'd0;
font_index[673]=10'd18;
font_index[674]=10'd18;
font_index[675]=10'd73;
font_index[676]=10'd214;
font_index[677]=10'd214;
font_index[678]=10'd214;
font_index[679]=10'd215;
font_index[680]=10'd216;
font_index[681]=10'd214;
font_index[682]=10'd214;
font_index[683]=10'd214;
font_index[684]=10'd216;
font_index[685]=10'd216;
font_index[686]=10'd217;
font_index[687]=10'd217;
font_index[688]=10'd217;
font_index[689]=10'd215;
font_index[690]=10'd218;
font_index[691]=10'd218;
font_index[692]=10'd218;
font_index[693]=10'd219;
font_index[694]=10'd220;
font_index[695]=10'd221;
font_index[696]=10'd222;
font_index[697]=10'd223;
font_index[698]=10'd224;
font_index[699]=10'd225;
font_index[700]=10'd226;
font_index[701]=10'd0;
font_index[702]=10'd0;
font_index[703]=10'd0;
font_index[704]=10'd0;
font_index[705]=10'd0;
font_index[706]=10'd227;
font_index[707]=10'd228;
font_index[708]=10'd229;
font_index[709]=10'd230;
font_index[710]=10'd231;
font_index[711]=10'd156;
font_index[712]=10'd232;
font_index[713]=10'd233;
font_index[714]=10'd234;
font_index[715]=10'd234;
font_index[716]=10'd235;
font_index[717]=10'd236;
font_index[718]=10'd237;
font_index[719]=10'd238;
font_index[720]=10'd239;
font_index[721]=10'd239;
font_index[722]=10'd240;
font_index[723]=10'd241;
font_index[724]=10'd241;
font_index[725]=10'd242;
font_index[726]=10'd242;
font_index[727]=10'd243;
font_index[728]=10'd244;
font_index[729]=10'd245;
font_index[730]=10'd246;
font_index[731]=10'd247;
font_index[732]=10'd248;
font_index[733]=10'd0;
font_index[734]=10'd0;
font_index[735]=10'd0;
font_index[736]=10'd0;
font_index[737]=10'd0;
font_index[738]=10'd74;
font_index[739]=10'd74;
font_index[740]=10'd159;
font_index[741]=10'd159;
font_index[742]=10'd114;
font_index[743]=10'd0;
font_index[744]=10'd173;
font_index[745]=10'd173;
font_index[746]=10'd249;
font_index[747]=10'd250;
font_index[748]=10'd173;
font_index[749]=10'd173;
font_index[750]=10'd0;
font_index[751]=10'd251;
font_index[752]=10'd251;
font_index[753]=10'd251;
font_index[754]=10'd252;
font_index[755]=10'd253;
font_index[756]=10'd253;
font_index[757]=10'd254;
font_index[758]=10'd254;
font_index[759]=10'd253;
font_index[760]=10'd253;
font_index[761]=10'd255;
font_index[762]=10'd256;
font_index[763]=10'd257;
font_index[764]=10'd258;
font_index[765]=10'd0;
font_index[766]=10'd0;
font_index[767]=10'd0;
font_index[768]=10'd0;
font_index[769]=10'd0;
font_index[770]=10'd0;
font_index[771]=10'd173;
font_index[772]=10'd173;
font_index[773]=10'd173;
font_index[774]=10'd250;
font_index[775]=10'd250;
font_index[776]=10'd250;
font_index[777]=10'd173;
font_index[778]=10'd173;
font_index[779]=10'd173;
font_index[780]=10'd0;
font_index[781]=10'd0;
font_index[782]=10'd259;
font_index[783]=10'd259;
font_index[784]=10'd259;
font_index[785]=10'd260;
font_index[786]=10'd261;
font_index[787]=10'd173;
font_index[788]=10'd173;
font_index[789]=10'd173;
font_index[790]=10'd262;
font_index[791]=10'd262;
font_index[792]=10'd263;
font_index[793]=10'd264;
font_index[794]=10'd265;
font_index[795]=10'd266;
font_index[796]=10'd69;
font_index[797]=10'd0;
font_index[798]=10'd0;
font_index[799]=10'd0;
font_index[800]=10'd0;
font_index[801]=10'd0;
font_index[802]=10'd267;
font_index[803]=10'd268;
font_index[804]=10'd269;
font_index[805]=10'd270;
font_index[806]=10'd271;
font_index[807]=10'd272;
font_index[808]=10'd273;
font_index[809]=10'd274;
font_index[810]=10'd275;
font_index[811]=10'd276;
font_index[812]=10'd277;
font_index[813]=10'd278;
font_index[814]=10'd279;
font_index[815]=10'd280;
font_index[816]=10'd281;
font_index[817]=10'd282;
font_index[818]=10'd283;
font_index[819]=10'd284;
font_index[820]=10'd285;
font_index[821]=10'd286;
font_index[822]=10'd287;
font_index[823]=10'd288;
font_index[824]=10'd289;
font_index[825]=10'd290;
font_index[826]=10'd291;
font_index[827]=10'd292;
font_index[828]=10'd293;
font_index[829]=10'd0;
font_index[830]=10'd0;
font_index[831]=10'd0;
font_index[832]=10'd0;
font_index[833]=10'd0;
font_index[834]=10'd71;
font_index[835]=10'd71;
font_index[836]=10'd294;
font_index[837]=10'd295;
font_index[838]=10'd296;
font_index[839]=10'd214;
font_index[840]=10'd297;
font_index[841]=10'd298;
font_index[842]=10'd299;
font_index[843]=10'd300;
font_index[844]=10'd301;
font_index[845]=10'd301;
font_index[846]=10'd302;
font_index[847]=10'd303;
font_index[848]=10'd304;
font_index[849]=10'd305;
font_index[850]=10'd306;
font_index[851]=10'd307;
font_index[852]=10'd308;
font_index[853]=10'd308;
font_index[854]=10'd309;
font_index[855]=10'd310;
font_index[856]=10'd311;
font_index[857]=10'd312;
font_index[858]=10'd313;
font_index[859]=10'd314;
font_index[860]=10'd315;
font_index[861]=10'd315;
font_index[862]=10'd0;
font_index[863]=10'd0;
font_index[864]=10'd0;
font_index[865]=10'd0;
font_index[866]=10'd0;
font_index[867]=10'd214;
font_index[868]=10'd214;
font_index[869]=10'd214;
font_index[870]=10'd251;
font_index[871]=10'd8;
font_index[872]=10'd8;
font_index[873]=10'd8;
font_index[874]=10'd8;
font_index[875]=10'd8;
font_index[876]=10'd8;
font_index[877]=10'd8;
font_index[878]=10'd259;
font_index[879]=10'd259;
font_index[880]=10'd259;
font_index[881]=10'd114;
font_index[882]=10'd8;
font_index[883]=10'd316;
font_index[884]=10'd316;
font_index[885]=10'd317;
font_index[886]=10'd318;
font_index[887]=10'd318;
font_index[888]=10'd319;
font_index[889]=10'd320;
font_index[890]=10'd321;
font_index[891]=10'd322;
font_index[892]=10'd323;
font_index[893]=10'd324;
font_index[894]=10'd0;
font_index[895]=10'd0;
font_index[896]=10'd0;
font_index[897]=10'd0;
font_index[898]=10'd325;
font_index[899]=10'd326;
font_index[900]=10'd327;
font_index[901]=10'd328;
font_index[902]=10'd329;
font_index[903]=10'd330;
font_index[904]=10'd331;
font_index[905]=10'd332;
font_index[906]=10'd333;
font_index[907]=10'd334;
font_index[908]=10'd327;
font_index[909]=10'd335;
font_index[910]=10'd336;
font_index[911]=10'd337;
font_index[912]=10'd338;
font_index[913]=10'd338;
font_index[914]=10'd338;
font_index[915]=10'd339;
font_index[916]=10'd340;
font_index[917]=10'd341;
font_index[918]=10'd342;
font_index[919]=10'd342;
font_index[920]=10'd342;
font_index[921]=10'd343;
font_index[922]=10'd343;
font_index[923]=10'd343;
font_index[924]=10'd344;
font_index[925]=10'd345;
font_index[926]=10'd0;
font_index[927]=10'd0;
font_index[928]=10'd0;
font_index[929]=10'd0;
font_index[930]=10'd0;
font_index[931]=10'd114;
font_index[932]=10'd114;
font_index[933]=10'd114;
font_index[934]=10'd78;
font_index[935]=10'd346;
font_index[936]=10'd347;
font_index[937]=10'd348;
font_index[938]=10'd349;
font_index[939]=10'd350;
font_index[940]=10'd351;
font_index[941]=10'd352;
font_index[942]=10'd353;
font_index[943]=10'd354;
font_index[944]=10'd355;
font_index[945]=10'd356;
font_index[946]=10'd356;
font_index[947]=10'd356;
font_index[948]=10'd357;
font_index[949]=10'd358;
font_index[950]=10'd359;
font_index[951]=10'd360;
font_index[952]=10'd361;
font_index[953]=10'd362;
font_index[954]=10'd363;
font_index[955]=10'd364;
font_index[956]=10'd365;
font_index[957]=10'd0;
font_index[958]=10'd0;
font_index[959]=10'd0;
font_index[960]=10'd0;
font_index[961]=10'd0;
font_index[962]=10'd366;
font_index[963]=10'd367;
font_index[964]=10'd368;
font_index[965]=10'd369;
font_index[966]=10'd328;
font_index[967]=10'd370;
font_index[968]=10'd371;
font_index[969]=10'd36;
font_index[970]=10'd372;
font_index[971]=10'd373;
font_index[972]=10'd373;
font_index[973]=10'd374;
font_index[974]=10'd375;
font_index[975]=10'd376;
font_index[976]=10'd376;
font_index[977]=10'd376;
font_index[978]=10'd376;
font_index[979]=10'd376;
font_index[980]=10'd377;
font_index[981]=10'd378;
font_index[982]=10'd379;
font_index[983]=10'd379;
font_index[984]=10'd379;
font_index[985]=10'd380;
font_index[986]=10'd369;
font_index[987]=10'd376;
font_index[988]=10'd381;
font_index[989]=10'd36;
font_index[990]=10'd0;
font_index[991]=10'd0;
font_index[992]=10'd0;
font_index[993]=10'd0;
font_index[994]=10'd382;
font_index[995]=10'd383;
font_index[996]=10'd383;
font_index[997]=10'd384;
font_index[998]=10'd385;
font_index[999]=10'd386;
font_index[1000]=10'd387;
font_index[1001]=10'd388;
font_index[1002]=10'd389;
font_index[1003]=10'd384;
font_index[1004]=10'd384;
font_index[1005]=10'd384;
font_index[1006]=10'd390;
font_index[1007]=10'd391;
font_index[1008]=10'd391;
font_index[1009]=10'd391;
font_index[1010]=10'd392;
font_index[1011]=10'd392;
font_index[1012]=10'd392;
font_index[1013]=10'd393;
font_index[1014]=10'd391;
font_index[1015]=10'd394;
font_index[1016]=10'd395;
font_index[1017]=10'd396;
font_index[1018]=10'd397;
font_index[1019]=10'd291;
font_index[1020]=10'd398;
font_index[1021]=10'd399;
font_index[1022]=10'd0;
font_index[1023]=10'd0;
font_index[1024]=10'd0;
font_index[1025]=10'd0;
font_index[1026]=10'd400;
font_index[1027]=10'd401;
font_index[1028]=10'd402;
font_index[1029]=10'd403;
font_index[1030]=10'd149;
font_index[1031]=10'd404;
font_index[1032]=10'd405;
font_index[1033]=10'd406;
font_index[1034]=10'd407;
font_index[1035]=10'd25;
font_index[1036]=10'd408;
font_index[1037]=10'd408;
font_index[1038]=10'd409;
font_index[1039]=10'd410;
font_index[1040]=10'd411;
font_index[1041]=10'd412;
font_index[1042]=10'd413;
font_index[1043]=10'd414;
font_index[1044]=10'd415;
font_index[1045]=10'd416;
font_index[1046]=10'd417;
font_index[1047]=10'd418;
font_index[1048]=10'd419;
font_index[1049]=10'd114;
font_index[1050]=10'd420;
font_index[1051]=10'd421;
font_index[1052]=10'd422;
font_index[1053]=10'd0;
font_index[1054]=10'd0;
font_index[1055]=10'd0;
font_index[1056]=10'd0;
font_index[1057]=10'd0;
font_index[1058]=10'd423;
font_index[1059]=10'd424;
font_index[1060]=10'd425;
font_index[1061]=10'd426;
font_index[1062]=10'd427;
font_index[1063]=10'd428;
font_index[1064]=10'd429;
font_index[1065]=10'd430;
font_index[1066]=10'd430;
font_index[1067]=10'd431;
font_index[1068]=10'd432;
font_index[1069]=10'd433;
font_index[1070]=10'd434;
font_index[1071]=10'd435;
font_index[1072]=10'd436;
font_index[1073]=10'd437;
font_index[1074]=10'd438;
font_index[1075]=10'd439;
font_index[1076]=10'd440;
font_index[1077]=10'd339;
font_index[1078]=10'd441;
font_index[1079]=10'd442;
font_index[1080]=10'd250;
font_index[1081]=10'd250;
font_index[1082]=10'd250;
font_index[1083]=10'd250;
font_index[1084]=10'd250;
font_index[1085]=10'd0;
font_index[1086]=10'd0;
font_index[1087]=10'd0;
font_index[1088]=10'd0;
font_index[1089]=10'd0;
font_index[1090]=10'd443;
font_index[1091]=10'd443;
font_index[1092]=10'd443;
font_index[1093]=10'd443;
font_index[1094]=10'd444;
font_index[1095]=10'd114;
font_index[1096]=10'd114;
font_index[1097]=10'd445;
font_index[1098]=10'd446;
font_index[1099]=10'd443;
font_index[1100]=10'd447;
font_index[1101]=10'd447;
font_index[1102]=10'd448;
font_index[1103]=10'd449;
font_index[1104]=10'd450;
font_index[1105]=10'd451;
font_index[1106]=10'd452;
font_index[1107]=10'd453;
font_index[1108]=10'd454;
font_index[1109]=10'd455;
font_index[1110]=10'd456;
font_index[1111]=10'd457;
font_index[1112]=10'd458;
font_index[1113]=10'd459;
font_index[1114]=10'd460;
font_index[1115]=10'd461;
font_index[1116]=10'd462;
font_index[1117]=10'd463;
font_index[1118]=10'd0;
font_index[1119]=10'd0;
font_index[1120]=10'd0;
font_index[1121]=10'd0;
font_index[1122]=10'd0;
font_index[1123]=10'd114;
font_index[1124]=10'd114;
font_index[1125]=10'd114;
font_index[1126]=10'd464;
font_index[1127]=10'd464;
font_index[1128]=10'd465;
font_index[1129]=10'd465;
font_index[1130]=10'd465;
font_index[1131]=10'd464;
font_index[1132]=10'd464;
font_index[1133]=10'd466;
font_index[1134]=10'd466;
font_index[1135]=10'd466;
font_index[1136]=10'd467;
font_index[1137]=10'd468;
font_index[1138]=10'd469;
font_index[1139]=10'd470;
font_index[1140]=10'd471;
font_index[1141]=10'd472;
font_index[1142]=10'd473;
font_index[1143]=10'd474;
font_index[1144]=10'd114;
font_index[1145]=10'd114;
font_index[1146]=10'd114;
font_index[1147]=10'd464;
font_index[1148]=10'd0;
font_index[1149]=10'd0;
font_index[1150]=10'd0;
font_index[1151]=10'd0;
font_index[1152]=10'd0;
font_index[1153]=10'd0;
font_index[1154]=10'd0;
font_index[1155]=10'd114;
font_index[1156]=10'd114;
font_index[1157]=10'd114;
font_index[1158]=10'd114;
font_index[1159]=10'd475;
font_index[1160]=10'd475;
font_index[1161]=10'd475;
font_index[1162]=10'd476;
font_index[1163]=10'd44;
font_index[1164]=10'd58;
font_index[1165]=10'd266;
font_index[1166]=10'd477;
font_index[1167]=10'd478;
font_index[1168]=10'd479;
font_index[1169]=10'd480;
font_index[1170]=10'd481;
font_index[1171]=10'd482;
font_index[1172]=10'd475;
font_index[1173]=10'd475;
font_index[1174]=10'd475;
font_index[1175]=10'd475;
font_index[1176]=10'd475;
font_index[1177]=10'd475;
font_index[1178]=10'd475;
font_index[1179]=10'd475;
font_index[1180]=10'd475;
font_index[1181]=10'd0;
font_index[1182]=10'd0;
font_index[1183]=10'd0;
font_index[1184]=10'd0;
font_index[1185]=10'd0;
font_index[1186]=10'd0;
font_index[1187]=10'd0;
font_index[1188]=10'd0;
font_index[1189]=10'd0;
font_index[1190]=10'd0;
font_index[1191]=10'd0;
font_index[1192]=10'd0;
font_index[1193]=10'd0;
font_index[1194]=10'd0;
font_index[1195]=10'd0;
font_index[1196]=10'd0;
font_index[1197]=10'd114;
font_index[1198]=10'd114;
font_index[1199]=10'd114;
font_index[1200]=10'd114;
font_index[1201]=10'd0;
font_index[1202]=10'd0;
font_index[1203]=10'd0;
font_index[1204]=10'd0;
font_index[1205]=10'd0;
font_index[1206]=10'd0;
font_index[1207]=10'd0;
font_index[1208]=10'd0;
font_index[1209]=10'd0;
font_index[1210]=10'd0;
font_index[1211]=10'd0;
font_index[1212]=10'd0;
font_index[1213]=10'd0;
font_index[1214]=10'd0;
font_index[1215]=10'd0;
font_index[1216]=10'd0;
font_index[1217]=10'd0;
font_index[1218]=10'd12;
font_index[1219]=10'd483;
font_index[1220]=10'd483;
font_index[1221]=10'd483;
font_index[1222]=10'd484;
font_index[1223]=10'd485;
font_index[1224]=10'd485;
font_index[1225]=10'd485;
font_index[1226]=10'd485;
font_index[1227]=10'd485;
font_index[1228]=10'd485;
font_index[1229]=10'd486;
font_index[1230]=10'd487;
font_index[1231]=10'd487;
font_index[1232]=10'd488;
font_index[1233]=10'd485;
font_index[1234]=10'd485;
font_index[1235]=10'd489;
font_index[1236]=10'd490;
font_index[1237]=10'd490;
font_index[1238]=10'd491;
font_index[1239]=10'd492;
font_index[1240]=10'd493;
font_index[1241]=10'd494;
font_index[1242]=10'd495;
font_index[1243]=10'd496;
font_index[1244]=10'd60;
font_index[1245]=10'd497;
font_index[1246]=10'd0;
font_index[1247]=10'd0;
font_index[1248]=10'd0;
font_index[1249]=10'd0;
font_index[1250]=10'd0;
font_index[1251]=10'd498;
font_index[1252]=10'd46;
font_index[1253]=10'd77;
font_index[1254]=10'd83;
font_index[1255]=10'd164;
font_index[1256]=10'd499;
font_index[1257]=10'd500;
font_index[1258]=10'd501;
font_index[1259]=10'd502;
font_index[1260]=10'd503;
font_index[1261]=10'd114;
font_index[1262]=10'd504;
font_index[1263]=10'd505;
font_index[1264]=10'd506;
font_index[1265]=10'd507;
font_index[1266]=10'd507;
font_index[1267]=10'd507;
font_index[1268]=10'd508;
font_index[1269]=10'd509;
font_index[1270]=10'd510;
font_index[1271]=10'd165;
font_index[1272]=10'd511;
font_index[1273]=10'd512;
font_index[1274]=10'd513;
font_index[1275]=10'd514;
font_index[1276]=10'd515;
font_index[1277]=10'd0;
font_index[1278]=10'd0;
font_index[1279]=10'd0;
font_index[1280]=10'd0;
font_index[1281]=10'd0;
font_index[1282]=10'd516;
font_index[1283]=10'd517;
font_index[1284]=10'd517;
font_index[1285]=10'd518;
font_index[1286]=10'd519;
font_index[1287]=10'd214;
font_index[1288]=10'd520;
font_index[1289]=10'd520;
font_index[1290]=10'd521;
font_index[1291]=10'd522;
font_index[1292]=10'd522;
font_index[1293]=10'd523;
font_index[1294]=10'd524;
font_index[1295]=10'd521;
font_index[1296]=10'd521;
font_index[1297]=10'd525;
font_index[1298]=10'd526;
font_index[1299]=10'd527;
font_index[1300]=10'd528;
font_index[1301]=10'd528;
font_index[1302]=10'd529;
font_index[1303]=10'd530;
font_index[1304]=10'd518;
font_index[1305]=10'd518;
font_index[1306]=10'd531;
font_index[1307]=10'd516;
font_index[1308]=10'd532;
font_index[1309]=10'd0;
font_index[1310]=10'd0;
font_index[1311]=10'd0;
font_index[1312]=10'd0;
font_index[1313]=10'd0;
font_index[1314]=10'd74;
font_index[1315]=10'd74;
font_index[1316]=10'd533;
font_index[1317]=10'd534;
font_index[1318]=10'd535;
font_index[1319]=10'd536;
font_index[1320]=10'd537;
font_index[1321]=10'd538;
font_index[1322]=10'd539;
font_index[1323]=10'd74;
font_index[1324]=10'd251;
font_index[1325]=10'd251;
font_index[1326]=10'd251;
font_index[1327]=10'd540;
font_index[1328]=10'd382;
font_index[1329]=10'd382;
font_index[1330]=10'd115;
font_index[1331]=10'd115;
font_index[1332]=10'd115;
font_index[1333]=10'd540;
font_index[1334]=10'd382;
font_index[1335]=10'd382;
font_index[1336]=10'd504;
font_index[1337]=10'd504;
font_index[1338]=10'd504;
font_index[1339]=10'd540;
font_index[1340]=10'd382;
font_index[1341]=10'd0;
font_index[1342]=10'd0;
font_index[1343]=10'd0;
font_index[1344]=10'd0;
font_index[1345]=10'd0;
font_index[1346]=10'd541;
font_index[1347]=10'd542;
font_index[1348]=10'd164;
font_index[1349]=10'd259;
font_index[1350]=10'd259;
font_index[1351]=10'd259;
font_index[1352]=10'd0;
font_index[1353]=10'd543;
font_index[1354]=10'd544;
font_index[1355]=10'd544;
font_index[1356]=10'd545;
font_index[1357]=10'd545;
font_index[1358]=10'd544;
font_index[1359]=10'd544;
font_index[1360]=10'd544;
font_index[1361]=10'd545;
font_index[1362]=10'd545;
font_index[1363]=10'd545;
font_index[1364]=10'd544;
font_index[1365]=10'd544;
font_index[1366]=10'd544;
font_index[1367]=10'd545;
font_index[1368]=10'd546;
font_index[1369]=10'd546;
font_index[1370]=10'd547;
font_index[1371]=10'd547;
font_index[1372]=10'd548;
font_index[1373]=10'd549;
font_index[1374]=10'd0;
font_index[1375]=10'd0;
font_index[1376]=10'd0;
font_index[1377]=10'd0;
font_index[1378]=10'd550;
font_index[1379]=10'd551;
font_index[1380]=10'd552;
font_index[1381]=10'd553;
font_index[1382]=10'd553;
font_index[1383]=10'd554;
font_index[1384]=10'd554;
font_index[1385]=10'd554;
font_index[1386]=10'd555;
font_index[1387]=10'd556;
font_index[1388]=10'd557;
font_index[1389]=10'd558;
font_index[1390]=10'd559;
font_index[1391]=10'd560;
font_index[1392]=10'd560;
font_index[1393]=10'd560;
font_index[1394]=10'd561;
font_index[1395]=10'd562;
font_index[1396]=10'd563;
font_index[1397]=10'd562;
font_index[1398]=10'd564;
font_index[1399]=10'd565;
font_index[1400]=10'd566;
font_index[1401]=10'd567;
font_index[1402]=10'd568;
font_index[1403]=10'd569;
font_index[1404]=10'd570;
font_index[1405]=10'd0;
font_index[1406]=10'd0;
font_index[1407]=10'd0;
font_index[1408]=10'd0;
font_index[1409]=10'd0;
font_index[1410]=10'd0;
font_index[1411]=10'd0;
font_index[1412]=10'd0;
font_index[1413]=10'd0;
font_index[1414]=10'd0;
font_index[1415]=10'd0;
font_index[1416]=10'd0;
font_index[1417]=10'd0;
font_index[1418]=10'd571;
font_index[1419]=10'd260;
font_index[1420]=10'd260;
font_index[1421]=10'd260;
font_index[1422]=10'd571;
font_index[1423]=10'd0;
font_index[1424]=10'd0;
font_index[1425]=10'd0;
font_index[1426]=10'd0;
font_index[1427]=10'd0;
font_index[1428]=10'd571;
font_index[1429]=10'd260;
font_index[1430]=10'd260;
font_index[1431]=10'd260;
font_index[1432]=10'd571;
font_index[1433]=10'd0;
font_index[1434]=10'd0;
font_index[1435]=10'd0;
font_index[1436]=10'd0;
font_index[1437]=10'd0;
font_index[1438]=10'd0;
font_index[1439]=10'd0;
font_index[1440]=10'd0;
font_index[1441]=10'd0;
font_index[1442]=10'd74;
font_index[1443]=10'd74;
font_index[1444]=10'd74;
font_index[1445]=10'd74;
font_index[1446]=10'd74;
font_index[1447]=10'd251;
font_index[1448]=10'd251;
font_index[1449]=10'd251;
font_index[1450]=10'd251;
font_index[1451]=10'd572;
font_index[1452]=10'd572;
font_index[1453]=10'd572;
font_index[1454]=10'd572;
font_index[1455]=10'd572;
font_index[1456]=10'd251;
font_index[1457]=10'd251;
font_index[1458]=10'd251;
font_index[1459]=10'd251;
font_index[1460]=10'd572;
font_index[1461]=10'd572;
font_index[1462]=10'd74;
font_index[1463]=10'd74;
font_index[1464]=10'd74;
font_index[1465]=10'd74;
font_index[1466]=10'd74;
font_index[1467]=10'd74;
font_index[1468]=10'd74;
font_index[1469]=10'd0;
font_index[1470]=10'd0;
font_index[1471]=10'd0;
font_index[1472]=10'd0;
font_index[1473]=10'd0;
font_index[1474]=10'd573;
font_index[1475]=10'd475;
font_index[1476]=10'd75;
font_index[1477]=10'd13;
font_index[1478]=10'd13;
font_index[1479]=10'd73;
font_index[1480]=10'd61;
font_index[1481]=10'd574;
font_index[1482]=10'd575;
font_index[1483]=10'd576;
font_index[1484]=10'd577;
font_index[1485]=10'd578;
font_index[1486]=10'd579;
font_index[1487]=10'd580;
font_index[1488]=10'd580;
font_index[1489]=10'd581;
font_index[1490]=10'd582;
font_index[1491]=10'd583;
font_index[1492]=10'd584;
font_index[1493]=10'd585;
font_index[1494]=10'd586;
font_index[1495]=10'd66;
font_index[1496]=10'd65;
font_index[1497]=10'd587;
font_index[1498]=10'd587;
font_index[1499]=10'd53;
font_index[1500]=10'd80;
font_index[1501]=10'd0;
font_index[1502]=10'd0;
font_index[1503]=10'd0;
font_index[1504]=10'd0;
font_index[1505]=10'd0;
font_index[1506]=10'd588;
font_index[1507]=10'd588;
font_index[1508]=10'd589;
font_index[1509]=10'd589;
font_index[1510]=10'd590;
font_index[1511]=10'd591;
font_index[1512]=10'd591;
font_index[1513]=10'd592;
font_index[1514]=10'd593;
font_index[1515]=10'd593;
font_index[1516]=10'd594;
font_index[1517]=10'd595;
font_index[1518]=10'd595;
font_index[1519]=10'd595;
font_index[1520]=10'd595;
font_index[1521]=10'd595;
font_index[1522]=10'd595;
font_index[1523]=10'd596;
font_index[1524]=10'd597;
font_index[1525]=10'd597;
font_index[1526]=10'd598;
font_index[1527]=10'd599;
font_index[1528]=10'd600;
font_index[1529]=10'd601;
font_index[1530]=10'd602;
font_index[1531]=10'd603;
font_index[1532]=10'd604;
font_index[1533]=10'd0;
font_index[1534]=10'd0;
font_index[1535]=10'd0;
font_index[1536]=10'd0;
font_index[1537]=10'd0;
font_index[1538]=10'd75;
font_index[1539]=10'd74;
font_index[1540]=10'd114;
font_index[1541]=10'd114;
font_index[1542]=10'd114;
font_index[1543]=10'd605;
font_index[1544]=10'd606;
font_index[1545]=10'd607;
font_index[1546]=10'd608;
font_index[1547]=10'd609;
font_index[1548]=10'd610;
font_index[1549]=10'd611;
font_index[1550]=10'd612;
font_index[1551]=10'd613;
font_index[1552]=10'd614;
font_index[1553]=10'd615;
font_index[1554]=10'd616;
font_index[1555]=10'd617;
font_index[1556]=10'd74;
font_index[1557]=10'd259;
font_index[1558]=10'd259;
font_index[1559]=10'd259;
font_index[1560]=10'd74;
font_index[1561]=10'd74;
font_index[1562]=10'd74;
font_index[1563]=10'd74;
font_index[1564]=10'd74;
font_index[1565]=10'd0;
font_index[1566]=10'd0;
font_index[1567]=10'd0;
font_index[1568]=10'd0;
font_index[1569]=10'd0;
font_index[1570]=10'd618;
font_index[1571]=10'd619;
font_index[1572]=10'd620;
font_index[1573]=10'd621;
font_index[1574]=10'd622;
font_index[1575]=10'd623;
font_index[1576]=10'd624;
font_index[1577]=10'd625;
font_index[1578]=10'd626;
font_index[1579]=10'd627;
font_index[1580]=10'd628;
font_index[1581]=10'd629;
font_index[1582]=10'd114;
font_index[1583]=10'd174;
font_index[1584]=10'd630;
font_index[1585]=10'd631;
font_index[1586]=10'd173;
font_index[1587]=10'd173;
font_index[1588]=10'd173;
font_index[1589]=10'd250;
font_index[1590]=10'd250;
font_index[1591]=10'd250;
font_index[1592]=10'd250;
font_index[1593]=10'd173;
font_index[1594]=10'd173;
font_index[1595]=10'd173;
font_index[1596]=10'd250;
font_index[1597]=10'd0;
font_index[1598]=10'd0;
font_index[1599]=10'd0;
font_index[1600]=10'd0;
font_index[1601]=10'd0;
font_index[1602]=10'd325;
font_index[1603]=10'd632;
font_index[1604]=10'd632;
font_index[1605]=10'd633;
font_index[1606]=10'd344;
font_index[1607]=10'd344;
font_index[1608]=10'd634;
font_index[1609]=10'd156;
font_index[1610]=10'd635;
font_index[1611]=10'd636;
font_index[1612]=10'd637;
font_index[1613]=10'd638;
font_index[1614]=10'd639;
font_index[1615]=10'd640;
font_index[1616]=10'd641;
font_index[1617]=10'd642;
font_index[1618]=10'd643;
font_index[1619]=10'd644;
font_index[1620]=10'd644;
font_index[1621]=10'd645;
font_index[1622]=10'd646;
font_index[1623]=10'd647;
font_index[1624]=10'd648;
font_index[1625]=10'd649;
font_index[1626]=10'd649;
font_index[1627]=10'd649;
font_index[1628]=10'd634;
font_index[1629]=10'd650;
font_index[1630]=10'd0;
font_index[1631]=10'd0;
font_index[1632]=10'd0;
font_index[1633]=10'd0;
font_index[1634]=10'd651;
font_index[1635]=10'd251;
font_index[1636]=10'd251;
font_index[1637]=10'd652;
font_index[1638]=10'd74;
font_index[1639]=10'd74;
font_index[1640]=10'd114;
font_index[1641]=10'd114;
font_index[1642]=10'd114;
font_index[1643]=10'd653;
font_index[1644]=10'd654;
font_index[1645]=10'd654;
font_index[1646]=10'd655;
font_index[1647]=10'd656;
font_index[1648]=10'd657;
font_index[1649]=10'd658;
font_index[1650]=10'd659;
font_index[1651]=10'd660;
font_index[1652]=10'd661;
font_index[1653]=10'd662;
font_index[1654]=10'd663;
font_index[1655]=10'd664;
font_index[1656]=10'd665;
font_index[1657]=10'd666;
font_index[1658]=10'd667;
font_index[1659]=10'd668;
font_index[1660]=10'd74;
font_index[1661]=10'd0;
font_index[1662]=10'd0;
font_index[1663]=10'd0;
font_index[1664]=10'd0;
font_index[1665]=10'd0;
font_index[1666]=10'd74;
font_index[1667]=10'd13;
font_index[1668]=10'd114;
font_index[1669]=10'd114;
font_index[1670]=10'd114;
font_index[1671]=10'd669;
font_index[1672]=10'd670;
font_index[1673]=10'd671;
font_index[1674]=10'd672;
font_index[1675]=10'd506;
font_index[1676]=10'd673;
font_index[1677]=10'd674;
font_index[1678]=10'd675;
font_index[1679]=10'd2;
font_index[1680]=10'd676;
font_index[1681]=10'd677;
font_index[1682]=10'd678;
font_index[1683]=10'd679;
font_index[1684]=10'd680;
font_index[1685]=10'd173;
font_index[1686]=10'd173;
font_index[1687]=10'd250;
font_index[1688]=10'd250;
font_index[1689]=10'd173;
font_index[1690]=10'd173;
font_index[1691]=10'd173;
font_index[1692]=10'd250;
font_index[1693]=10'd0;
font_index[1694]=10'd0;
font_index[1695]=10'd0;
font_index[1696]=10'd0;
font_index[1697]=10'd0;
font_index[1698]=10'd0;
font_index[1699]=10'd0;
font_index[1700]=10'd0;
font_index[1701]=10'd0;
font_index[1702]=10'd0;
font_index[1703]=10'd0;
font_index[1704]=10'd0;
font_index[1705]=10'd0;
font_index[1706]=10'd0;
font_index[1707]=10'd0;
font_index[1708]=10'd0;
font_index[1709]=10'd0;
font_index[1710]=10'd0;
font_index[1711]=10'd0;
font_index[1712]=10'd0;
font_index[1713]=10'd0;
font_index[1714]=10'd0;
font_index[1715]=10'd0;
font_index[1716]=10'd0;
font_index[1717]=10'd0;
font_index[1718]=10'd261;
font_index[1719]=10'd261;
font_index[1720]=10'd571;
font_index[1721]=10'd345;
font_index[1722]=10'd345;
font_index[1723]=10'd650;
font_index[1724]=10'd650;
font_index[1725]=10'd0;
font_index[1726]=10'd0;
font_index[1727]=10'd0;
font_index[1728]=10'd0;
font_index[1729]=10'd0;
font_index[1730]=10'd382;
font_index[1731]=10'd383;
font_index[1732]=10'd383;
font_index[1733]=10'd681;
font_index[1734]=10'd682;
font_index[1735]=10'd683;
font_index[1736]=10'd684;
font_index[1737]=10'd685;
font_index[1738]=10'd685;
font_index[1739]=10'd686;
font_index[1740]=10'd686;
font_index[1741]=10'd687;
font_index[1742]=10'd688;
font_index[1743]=10'd689;
font_index[1744]=10'd690;
font_index[1745]=10'd691;
font_index[1746]=10'd692;
font_index[1747]=10'd693;
font_index[1748]=10'd694;
font_index[1749]=10'd695;
font_index[1750]=10'd696;
font_index[1751]=10'd697;
font_index[1752]=10'd698;
font_index[1753]=10'd699;
font_index[1754]=10'd700;
font_index[1755]=10'd701;
font_index[1756]=10'd702;
font_index[1757]=10'd703;
font_index[1758]=10'd0;
font_index[1759]=10'd0;
font_index[1760]=10'd0;
font_index[1761]=10'd0;
font_index[1762]=10'd52;
font_index[1763]=10'd52;
font_index[1764]=10'd29;
font_index[1765]=10'd504;
font_index[1766]=10'd504;
font_index[1767]=10'd504;
font_index[1768]=10'd620;
font_index[1769]=10'd704;
font_index[1770]=10'd705;
font_index[1771]=10'd706;
font_index[1772]=10'd707;
font_index[1773]=10'd707;
font_index[1774]=10'd708;
font_index[1775]=10'd708;
font_index[1776]=10'd708;
font_index[1777]=10'd18;
font_index[1778]=10'd18;
font_index[1779]=10'd18;
font_index[1780]=10'd114;
font_index[1781]=10'd114;
font_index[1782]=10'd114;
font_index[1783]=10'd18;
font_index[1784]=10'd18;
font_index[1785]=10'd18;
font_index[1786]=10'd18;
font_index[1787]=10'd18;
font_index[1788]=10'd18;
font_index[1789]=10'd18;
font_index[1790]=10'd0;
font_index[1791]=10'd0;
font_index[1792]=10'd0;
font_index[1793]=10'd0;
font_index[1794]=10'd0;
font_index[1795]=10'd709;
font_index[1796]=10'd709;
font_index[1797]=10'd709;
font_index[1798]=10'd710;
font_index[1799]=10'd711;
font_index[1800]=10'd711;
font_index[1801]=10'd712;
font_index[1802]=10'd712;
font_index[1803]=10'd712;
font_index[1804]=10'd711;
font_index[1805]=10'd711;
font_index[1806]=10'd712;
font_index[1807]=10'd712;
font_index[1808]=10'd712;
font_index[1809]=10'd713;
font_index[1810]=10'd713;
font_index[1811]=10'd714;
font_index[1812]=10'd715;
font_index[1813]=10'd716;
font_index[1814]=10'd717;
font_index[1815]=10'd718;
font_index[1816]=10'd719;
font_index[1817]=10'd720;
font_index[1818]=10'd721;
font_index[1819]=10'd722;
font_index[1820]=10'd71;
font_index[1821]=10'd0;
font_index[1822]=10'd0;
font_index[1823]=10'd0;
font_index[1824]=10'd0;
font_index[1825]=10'd0;
font_index[1826]=10'd723;
font_index[1827]=10'd723;
font_index[1828]=10'd724;
font_index[1829]=10'd725;
font_index[1830]=10'd726;
font_index[1831]=10'd727;
font_index[1832]=10'd728;
font_index[1833]=10'd729;
font_index[1834]=10'd730;
font_index[1835]=10'd731;
font_index[1836]=10'd732;
font_index[1837]=10'd733;
font_index[1838]=10'd734;
font_index[1839]=10'd735;
font_index[1840]=10'd736;
font_index[1841]=10'd737;
font_index[1842]=10'd738;
font_index[1843]=10'd739;
font_index[1844]=10'd740;
font_index[1845]=10'd741;
font_index[1846]=10'd742;
font_index[1847]=10'd743;
font_index[1848]=10'd744;
font_index[1849]=10'd745;
font_index[1850]=10'd746;
font_index[1851]=10'd746;
font_index[1852]=10'd747;
font_index[1853]=10'd261;
font_index[1854]=10'd0;
font_index[1855]=10'd0;
font_index[1856]=10'd0;
font_index[1857]=10'd0;
font_index[1858]=10'd748;
font_index[1859]=10'd749;
font_index[1860]=10'd749;
font_index[1861]=10'd750;
font_index[1862]=10'd751;
font_index[1863]=10'd231;
font_index[1864]=10'd752;
font_index[1865]=10'd753;
font_index[1866]=10'd754;
font_index[1867]=10'd754;
font_index[1868]=10'd755;
font_index[1869]=10'd755;
font_index[1870]=10'd756;
font_index[1871]=10'd756;
font_index[1872]=10'd148;
font_index[1873]=10'd757;
font_index[1874]=10'd758;
font_index[1875]=10'd759;
font_index[1876]=10'd760;
font_index[1877]=10'd761;
font_index[1878]=10'd762;
font_index[1879]=10'd763;
font_index[1880]=10'd764;
font_index[1881]=10'd159;
font_index[1882]=10'd765;
font_index[1883]=10'd766;
font_index[1884]=10'd422;
font_index[1885]=10'd0;
font_index[1886]=10'd0;
font_index[1887]=10'd0;
font_index[1888]=10'd0;
font_index[1889]=10'd0;
font_index[1890]=10'd400;
font_index[1891]=10'd401;
font_index[1892]=10'd146;
font_index[1893]=10'd147;
font_index[1894]=10'd767;
font_index[1895]=10'd149;
font_index[1896]=10'd768;
font_index[1897]=10'd151;
font_index[1898]=10'd24;
font_index[1899]=10'd71;
font_index[1900]=10'd769;
font_index[1901]=10'd769;
font_index[1902]=10'd770;
font_index[1903]=10'd235;
font_index[1904]=10'd771;
font_index[1905]=10'd771;
font_index[1906]=10'd772;
font_index[1907]=10'd771;
font_index[1908]=10'd771;
font_index[1909]=10'd772;
font_index[1910]=10'd773;
font_index[1911]=10'd751;
font_index[1912]=10'd751;
font_index[1913]=10'd774;
font_index[1914]=10'd775;
font_index[1915]=10'd776;
font_index[1916]=10'd777;
font_index[1917]=10'd778;
font_index[1918]=10'd0;
font_index[1919]=10'd0;
font_index[1920]=10'd0;
font_index[1921]=10'd0;
font_index[1922]=10'd60;
font_index[1923]=10'd28;
font_index[1924]=10'd52;
font_index[1925]=10'd114;
font_index[1926]=10'd114;
font_index[1927]=10'd114;
font_index[1928]=10'd114;
font_index[1929]=10'd48;
font_index[1930]=10'd779;
font_index[1931]=10'd327;
font_index[1932]=10'd327;
font_index[1933]=10'd335;
font_index[1934]=10'd336;
font_index[1935]=10'd780;
font_index[1936]=10'd780;
font_index[1937]=10'd338;
font_index[1938]=10'd338;
font_index[1939]=10'd781;
font_index[1940]=10'd781;
font_index[1941]=10'd342;
font_index[1942]=10'd342;
font_index[1943]=10'd342;
font_index[1944]=10'd342;
font_index[1945]=10'd344;
font_index[1946]=10'd344;
font_index[1947]=10'd344;
font_index[1948]=10'd344;
font_index[1949]=10'd345;
font_index[1950]=10'd0;
font_index[1951]=10'd0;
font_index[1952]=10'd0;
font_index[1953]=10'd0;
font_index[1954]=10'd82;
font_index[1955]=10'd82;
font_index[1956]=10'd251;
font_index[1957]=10'd251;
font_index[1958]=10'd251;
font_index[1959]=10'd82;
font_index[1960]=10'd82;
font_index[1961]=10'd114;
font_index[1962]=10'd114;
font_index[1963]=10'd114;
font_index[1964]=10'd74;
font_index[1965]=10'd708;
font_index[1966]=10'd708;
font_index[1967]=10'd708;
font_index[1968]=10'd782;
font_index[1969]=10'd708;
font_index[1970]=10'd708;
font_index[1971]=10'd782;
font_index[1972]=10'd782;
font_index[1973]=10'd708;
font_index[1974]=10'd708;
font_index[1975]=10'd173;
font_index[1976]=10'd82;
font_index[1977]=10'd783;
font_index[1978]=10'd784;
font_index[1979]=10'd785;
font_index[1980]=10'd786;
font_index[1981]=10'd0;
font_index[1982]=10'd0;
font_index[1983]=10'd0;
font_index[1984]=10'd0;
font_index[1985]=10'd0;
font_index[1986]=10'd30;
font_index[1987]=10'd787;
font_index[1988]=10'd70;
font_index[1989]=10'd629;
font_index[1990]=10'd788;
font_index[1991]=10'd328;
font_index[1992]=10'd336;
font_index[1993]=10'd789;
font_index[1994]=10'd158;
font_index[1995]=10'd158;
font_index[1996]=10'd158;
font_index[1997]=10'd790;
font_index[1998]=10'd782;
font_index[1999]=10'd782;
font_index[2000]=10'd782;
font_index[2001]=10'd708;
font_index[2002]=10'd708;
font_index[2003]=10'd708;
font_index[2004]=10'd635;
font_index[2005]=10'd791;
font_index[2006]=10'd792;
font_index[2007]=10'd793;
font_index[2008]=10'd794;
font_index[2009]=10'd795;
font_index[2010]=10'd448;
font_index[2011]=10'd622;
font_index[2012]=10'd63;
font_index[2013]=10'd0;
font_index[2014]=10'd0;
font_index[2015]=10'd0;
font_index[2016]=10'd0;
font_index[2017]=10'd0;
font_index[2018]=10'd796;
font_index[2019]=10'd75;
font_index[2020]=10'd13;
font_index[2021]=10'd114;
font_index[2022]=10'd114;
font_index[2023]=10'd114;
font_index[2024]=10'd797;
font_index[2025]=10'd798;
font_index[2026]=10'd799;
font_index[2027]=10'd28;
font_index[2028]=10'd28;
font_index[2029]=10'd259;
font_index[2030]=10'd259;
font_index[2031]=10'd259;
font_index[2032]=10'd114;
font_index[2033]=10'd316;
font_index[2034]=10'd800;
font_index[2035]=10'd801;
font_index[2036]=10'd802;
font_index[2037]=10'd31;
font_index[2038]=10'd33;
font_index[2039]=10'd803;
font_index[2040]=10'd804;
font_index[2041]=10'd805;
font_index[2042]=10'd806;
font_index[2043]=10'd807;
font_index[2044]=10'd808;
font_index[2045]=10'd0;
font_index[2046]=10'd0;
font_index[2047]=10'd0;
font_index[2048]=10'd0;
font_index[2049]=10'd0;
font_index[2050]=10'd13;
font_index[2051]=10'd11;
font_index[2052]=10'd11;
font_index[2053]=10'd34;
font_index[2054]=10'd33;
font_index[2055]=10'd172;
font_index[2056]=10'd83;
font_index[2057]=10'd164;
font_index[2058]=10'd809;
font_index[2059]=10'd810;
font_index[2060]=10'd114;
font_index[2061]=10'd114;
font_index[2062]=10'd811;
font_index[2063]=10'd812;
font_index[2064]=10'd74;
font_index[2065]=10'd74;
font_index[2066]=10'd173;
font_index[2067]=10'd173;
font_index[2068]=10'd173;
font_index[2069]=10'd173;
font_index[2070]=10'd74;
font_index[2071]=10'd74;
font_index[2072]=10'd74;
font_index[2073]=10'd251;
font_index[2074]=10'd251;
font_index[2075]=10'd251;
font_index[2076]=10'd251;
font_index[2077]=10'd0;
font_index[2078]=10'd0;
font_index[2079]=10'd0;
font_index[2080]=10'd0;
font_index[2081]=10'd0;
font_index[2082]=10'd74;
font_index[2083]=10'd74;
font_index[2084]=10'd74;
font_index[2085]=10'd74;
font_index[2086]=10'd251;
font_index[2087]=10'd251;
font_index[2088]=10'd251;
font_index[2089]=10'd251;
font_index[2090]=10'd572;
font_index[2091]=10'd572;
font_index[2092]=10'd572;
font_index[2093]=10'd813;
font_index[2094]=10'd814;
font_index[2095]=10'd815;
font_index[2096]=10'd816;
font_index[2097]=10'd817;
font_index[2098]=10'd818;
font_index[2099]=10'd819;
font_index[2100]=10'd820;
font_index[2101]=10'd821;
font_index[2102]=10'd822;
font_index[2103]=10'd822;
font_index[2104]=10'd822;
font_index[2105]=10'd822;
font_index[2106]=10'd823;
font_index[2107]=10'd824;
font_index[2108]=10'd825;
font_index[2109]=10'd826;
font_index[2110]=10'd0;
font_index[2111]=10'd0;
font_index[2112]=10'd0;
font_index[2113]=10'd0;
font_index[2114]=10'd650;
font_index[2115]=10'd827;
font_index[2116]=10'd827;
font_index[2117]=10'd828;
font_index[2118]=10'd829;
font_index[2119]=10'd829;
font_index[2120]=10'd830;
font_index[2121]=10'd146;
font_index[2122]=10'd146;
font_index[2123]=10'd831;
font_index[2124]=10'd832;
font_index[2125]=10'd833;
font_index[2126]=10'd834;
font_index[2127]=10'd835;
font_index[2128]=10'd836;
font_index[2129]=10'd837;
font_index[2130]=10'd830;
font_index[2131]=10'd830;
font_index[2132]=10'd830;
font_index[2133]=10'd838;
font_index[2134]=10'd839;
font_index[2135]=10'd840;
font_index[2136]=10'd841;
font_index[2137]=10'd842;
font_index[2138]=10'd843;
font_index[2139]=10'd844;
font_index[2140]=10'd845;
font_index[2141]=10'd0;
font_index[2142]=10'd0;
font_index[2143]=10'd0;
font_index[2144]=10'd0;
font_index[2145]=10'd0;
font_index[2146]=10'd0;
font_index[2147]=10'd0;
font_index[2148]=10'd0;
font_index[2149]=10'd0;
font_index[2150]=10'd0;
font_index[2151]=10'd0;
font_index[2152]=10'd0;
font_index[2153]=10'd0;
font_index[2154]=10'd0;
font_index[2155]=10'd0;
font_index[2156]=10'd0;
font_index[2157]=10'd0;
font_index[2158]=10'd0;
font_index[2159]=10'd0;
font_index[2160]=10'd0;
font_index[2161]=10'd0;
font_index[2162]=10'd0;
font_index[2163]=10'd0;
font_index[2164]=10'd0;
font_index[2165]=10'd635;
font_index[2166]=10'd846;
font_index[2167]=10'd847;
font_index[2168]=10'd848;
font_index[2169]=10'd848;
font_index[2170]=10'd847;
font_index[2171]=10'd849;
font_index[2172]=10'd850;
font_index[2173]=10'd0;
font_index[2174]=10'd0;
font_index[2175]=10'd0;
font_index[2176]=10'd0;
font_index[2177]=10'd0;
font_index[2178]=10'd74;
font_index[2179]=10'd74;
font_index[2180]=10'd74;
font_index[2181]=10'd74;
font_index[2182]=10'd74;
font_index[2183]=10'd74;
font_index[2184]=10'd74;
font_index[2185]=10'd114;
font_index[2186]=10'd114;
font_index[2187]=10'd114;
font_index[2188]=10'd114;
font_index[2189]=10'd75;
font_index[2190]=10'd11;
font_index[2191]=10'd11;
font_index[2192]=10'd34;
font_index[2193]=10'd34;
font_index[2194]=10'd33;
font_index[2195]=10'd851;
font_index[2196]=10'd172;
font_index[2197]=10'd83;
font_index[2198]=10'd164;
font_index[2199]=10'd852;
font_index[2200]=10'd853;
font_index[2201]=10'd854;
font_index[2202]=10'd855;
font_index[2203]=10'd822;
font_index[2204]=10'd856;
font_index[2205]=10'd0;
font_index[2206]=10'd0;
font_index[2207]=10'd0;
font_index[2208]=10'd0;
font_index[2209]=10'd0;
font_index[2210]=10'd857;
font_index[2211]=10'd858;
font_index[2212]=10'd859;
font_index[2213]=10'd860;
font_index[2214]=10'd861;
font_index[2215]=10'd114;
font_index[2216]=10'd114;
font_index[2217]=10'd114;
font_index[2218]=10'd464;
font_index[2219]=10'd464;
font_index[2220]=10'd466;
font_index[2221]=10'd629;
font_index[2222]=10'd629;
font_index[2223]=10'd862;
font_index[2224]=10'd41;
font_index[2225]=10'd73;
font_index[2226]=10'd114;
font_index[2227]=10'd114;
font_index[2228]=10'd114;
font_index[2229]=10'd13;
font_index[2230]=10'd18;
font_index[2231]=10'd18;
font_index[2232]=10'd13;
font_index[2233]=10'd863;
font_index[2234]=10'd864;
font_index[2235]=10'd865;
font_index[2236]=10'd29;
font_index[2237]=10'd0;
font_index[2238]=10'd0;
font_index[2239]=10'd0;
font_index[2240]=10'd0;
font_index[2241]=10'd0;
font_index[2242]=10'd866;
font_index[2243]=10'd164;
font_index[2244]=10'd83;
font_index[2245]=10'd114;
font_index[2246]=10'd114;
font_index[2247]=10'd114;
font_index[2248]=10'd74;
font_index[2249]=10'd74;
font_index[2250]=10'd115;
font_index[2251]=10'd115;
font_index[2252]=10'd708;
font_index[2253]=10'd74;
font_index[2254]=10'd74;
font_index[2255]=10'd114;
font_index[2256]=10'd114;
font_index[2257]=10'd114;
font_index[2258]=10'd74;
font_index[2259]=10'd74;
font_index[2260]=10'd114;
font_index[2261]=10'd114;
font_index[2262]=10'd114;
font_index[2263]=10'd34;
font_index[2264]=10'd21;
font_index[2265]=10'd867;
font_index[2266]=10'd784;
font_index[2267]=10'd868;
font_index[2268]=10'd869;
font_index[2269]=10'd177;
font_index[2270]=10'd0;
font_index[2271]=10'd0;
font_index[2272]=10'd0;
font_index[2273]=10'd74;
font_index[2274]=10'd11;
font_index[2275]=10'd34;
font_index[2276]=10'd20;
font_index[2277]=10'd870;
font_index[2278]=10'd871;
font_index[2279]=10'd872;
font_index[2280]=10'd873;
font_index[2281]=10'd114;
font_index[2282]=10'd114;
font_index[2283]=10'd874;
font_index[2284]=10'd875;
font_index[2285]=10'd506;
font_index[2286]=10'd506;
font_index[2287]=10'd506;
font_index[2288]=10'd875;
font_index[2289]=10'd875;
font_index[2290]=10'd506;
font_index[2291]=10'd506;
font_index[2292]=10'd173;
font_index[2293]=10'd876;
font_index[2294]=10'd877;
font_index[2295]=10'd878;
font_index[2296]=10'd879;
font_index[2297]=10'd880;
font_index[2298]=10'd881;
font_index[2299]=10'd882;
font_index[2300]=10'd883;
font_index[2301]=10'd0;
font_index[2302]=10'd0;
font_index[2303]=10'd0;
font_index[2304]=10'd0;
font_index[2305]=10'd0;
font_index[2306]=10'd884;
font_index[2307]=10'd885;
font_index[2308]=10'd886;
font_index[2309]=10'd887;
font_index[2310]=10'd888;
font_index[2311]=10'd889;
font_index[2312]=10'd890;
font_index[2313]=10'd891;
font_index[2314]=10'd892;
font_index[2315]=10'd893;
font_index[2316]=10'd894;
font_index[2317]=10'd895;
font_index[2318]=10'd896;
font_index[2319]=10'd897;
font_index[2320]=10'd898;
font_index[2321]=10'd899;
font_index[2322]=10'd899;
font_index[2323]=10'd900;
font_index[2324]=10'd901;
font_index[2325]=10'd902;
font_index[2326]=10'd902;
font_index[2327]=10'd903;
font_index[2328]=10'd904;
font_index[2329]=10'd905;
font_index[2330]=10'd906;
font_index[2331]=10'd907;
font_index[2332]=10'd908;
font_index[2333]=10'd909;
font_index[2334]=10'd0;
font_index[2335]=10'd0;
font_index[2336]=10'd0;
font_index[2337]=10'd0;
font_index[2338]=10'd0;
font_index[2339]=10'd114;
font_index[2340]=10'd114;
font_index[2341]=10'd114;
font_index[2342]=10'd504;
font_index[2343]=10'd910;
font_index[2344]=10'd399;
font_index[2345]=10'd911;
font_index[2346]=10'd912;
font_index[2347]=10'd56;
font_index[2348]=10'd35;
font_index[2349]=10'd1;
font_index[2350]=10'd13;
font_index[2351]=10'd13;
font_index[2352]=10'd13;
font_index[2353]=10'd13;
font_index[2354]=10'd13;
font_index[2355]=10'd13;
font_index[2356]=10'd13;
font_index[2357]=10'd13;
font_index[2358]=10'd13;
font_index[2359]=10'd13;
font_index[2360]=10'd13;
font_index[2361]=10'd13;
font_index[2362]=10'd913;
font_index[2363]=10'd914;
font_index[2364]=10'd915;
font_index[2365]=10'd916;
font_index[2366]=10'd0;
font_index[2367]=10'd0;
font_index[2368]=10'd0;
font_index[2369]=10'd0;
font_index[2370]=10'd60;
font_index[2371]=10'd28;
font_index[2372]=10'd52;
font_index[2373]=10'd114;
font_index[2374]=10'd114;
font_index[2375]=10'd114;
font_index[2376]=10'd917;
font_index[2377]=10'd620;
font_index[2378]=10'd622;
font_index[2379]=10'd622;
font_index[2380]=10'd623;
font_index[2381]=10'd918;
font_index[2382]=10'd919;
font_index[2383]=10'd158;
font_index[2384]=10'd158;
font_index[2385]=10'd920;
font_index[2386]=10'd921;
font_index[2387]=10'd922;
font_index[2388]=10'd622;
font_index[2389]=10'd622;
font_index[2390]=10'd622;
font_index[2391]=10'd736;
font_index[2392]=10'd736;
font_index[2393]=10'd923;
font_index[2394]=10'd924;
font_index[2395]=10'd925;
font_index[2396]=10'd747;
font_index[2397]=10'd926;
font_index[2398]=10'd0;
font_index[2399]=10'd0;
font_index[2400]=10'd0;
font_index[2401]=10'd0;
font_index[2402]=10'd12;
font_index[2403]=10'd13;
font_index[2404]=10'd18;
font_index[2405]=10'd795;
font_index[2406]=10'd795;
font_index[2407]=10'd795;
font_index[2408]=10'd635;
font_index[2409]=10'd927;
font_index[2410]=10'd210;
font_index[2411]=10'd210;
font_index[2412]=10'd210;
font_index[2413]=10'd928;
font_index[2414]=10'd929;
font_index[2415]=10'd930;
font_index[2416]=10'd931;
font_index[2417]=10'd242;
font_index[2418]=10'd932;
font_index[2419]=10'd932;
font_index[2420]=10'd933;
font_index[2421]=10'd933;
font_index[2422]=10'd934;
font_index[2423]=10'd934;
font_index[2424]=10'd935;
font_index[2425]=10'd936;
font_index[2426]=10'd936;
font_index[2427]=10'd937;
font_index[2428]=10'd938;
font_index[2429]=10'd0;
font_index[2430]=10'd0;
font_index[2431]=10'd0;
font_index[2432]=10'd0;
font_index[2433]=10'd0;
font_index[2434]=10'd475;
font_index[2435]=10'd475;
font_index[2436]=10'd475;
font_index[2437]=10'd475;
font_index[2438]=10'd475;
font_index[2439]=10'd475;
font_index[2440]=10'd475;
font_index[2441]=10'd475;
font_index[2442]=10'd939;
font_index[2443]=10'd939;
font_index[2444]=10'd939;
font_index[2445]=10'd939;
font_index[2446]=10'd475;
font_index[2447]=10'd475;
font_index[2448]=10'd475;
font_index[2449]=10'd475;
font_index[2450]=10'd475;
font_index[2451]=10'd475;
font_index[2452]=10'd475;
font_index[2453]=10'd475;
font_index[2454]=10'd475;
font_index[2455]=10'd475;
font_index[2456]=10'd475;
font_index[2457]=10'd114;
font_index[2458]=10'd114;
font_index[2459]=10'd114;
font_index[2460]=10'd114;
font_index[2461]=10'd0;
font_index[2462]=10'd0;
font_index[2463]=10'd0;
font_index[2464]=10'd0;
font_index[2465]=10'd0;
font_index[2466]=10'd0;
font_index[2467]=10'd74;
font_index[2468]=10'd74;
font_index[2469]=10'd782;
font_index[2470]=10'd782;
font_index[2471]=10'd782;
font_index[2472]=10'd782;
font_index[2473]=10'd782;
font_index[2474]=10'd782;
font_index[2475]=10'd782;
font_index[2476]=10'd708;
font_index[2477]=10'd708;
font_index[2478]=10'd708;
font_index[2479]=10'd708;
font_index[2480]=10'd74;
font_index[2481]=10'd572;
font_index[2482]=10'd572;
font_index[2483]=10'd572;
font_index[2484]=10'd572;
font_index[2485]=10'd572;
font_index[2486]=10'd572;
font_index[2487]=10'd572;
font_index[2488]=10'd572;
font_index[2489]=10'd251;
font_index[2490]=10'd251;
font_index[2491]=10'd251;
font_index[2492]=10'd251;
font_index[2493]=10'd382;
font_index[2494]=10'd0;
font_index[2495]=10'd0;
font_index[2496]=10'd0;
font_index[2497]=10'd0;
font_index[2498]=10'd74;
font_index[2499]=10'd74;
font_index[2500]=10'd74;
font_index[2501]=10'd74;
font_index[2502]=10'd251;
font_index[2503]=10'd251;
font_index[2504]=10'd251;
font_index[2505]=10'd708;
font_index[2506]=10'd74;
font_index[2507]=10'd74;
font_index[2508]=10'd74;
font_index[2509]=10'd114;
font_index[2510]=10'd114;
font_index[2511]=10'd114;
font_index[2512]=10'd114;
font_index[2513]=10'd34;
font_index[2514]=10'd33;
font_index[2515]=10'd31;
font_index[2516]=10'd940;
font_index[2517]=10'd537;
font_index[2518]=10'd941;
font_index[2519]=10'd665;
font_index[2520]=10'd942;
font_index[2521]=10'd943;
font_index[2522]=10'd944;
font_index[2523]=10'd74;
font_index[2524]=10'd74;
font_index[2525]=10'd0;
font_index[2526]=10'd0;
font_index[2527]=10'd0;
font_index[2528]=10'd0;
font_index[2529]=10'd0;
font_index[2530]=10'd945;
font_index[2531]=10'd553;
font_index[2532]=10'd633;
font_index[2533]=10'd335;
font_index[2534]=10'd174;
font_index[2535]=10'd174;
font_index[2536]=10'd946;
font_index[2537]=10'd946;
font_index[2538]=10'd947;
font_index[2539]=10'd948;
font_index[2540]=10'd949;
font_index[2541]=10'd950;
font_index[2542]=10'd951;
font_index[2543]=10'd952;
font_index[2544]=10'd953;
font_index[2545]=10'd953;
font_index[2546]=10'd954;
font_index[2547]=10'd955;
font_index[2548]=10'd956;
font_index[2549]=10'd957;
font_index[2550]=10'd958;
font_index[2551]=10'd958;
font_index[2552]=10'd959;
font_index[2553]=10'd959;
font_index[2554]=10'd960;
font_index[2555]=10'd960;
font_index[2556]=10'd961;
font_index[2557]=10'd962;
font_index[2558]=10'd0;
font_index[2559]=10'd0;
font_index[2560]=10'd0;
font_index[2561]=10'd0;
font_index[2562]=10'd0;
font_index[2563]=10'd623;
font_index[2564]=10'd623;
font_index[2565]=10'd623;
font_index[2566]=10'd963;
font_index[2567]=10'd963;
font_index[2568]=10'd963;
font_index[2569]=10'd963;
font_index[2570]=10'd623;
font_index[2571]=10'd623;
font_index[2572]=10'd623;
font_index[2573]=10'd342;
font_index[2574]=10'd370;
font_index[2575]=10'd551;
font_index[2576]=10'd964;
font_index[2577]=10'd965;
font_index[2578]=10'd966;
font_index[2579]=10'd967;
font_index[2580]=10'd968;
font_index[2581]=10'd969;
font_index[2582]=10'd970;
font_index[2583]=10'd971;
font_index[2584]=10'd972;
font_index[2585]=10'd587;
font_index[2586]=10'd973;
font_index[2587]=10'd862;
font_index[2588]=10'd974;
font_index[2589]=10'd0;
font_index[2590]=10'd0;
font_index[2591]=10'd0;
font_index[2592]=10'd0;
font_index[2593]=10'd0;
font_index[2594]=10'd975;
font_index[2595]=10'd975;
font_index[2596]=10'd975;
font_index[2597]=10'd975;
font_index[2598]=10'd877;
font_index[2599]=10'd976;
font_index[2600]=10'd158;
font_index[2601]=10'd158;
font_index[2602]=10'd977;
font_index[2603]=10'd978;
font_index[2604]=10'd979;
font_index[2605]=10'd980;
font_index[2606]=10'd980;
font_index[2607]=10'd981;
font_index[2608]=10'd344;
font_index[2609]=10'd344;
font_index[2610]=10'd344;
font_index[2611]=10'd982;
font_index[2612]=10'd983;
font_index[2613]=10'd984;
font_index[2614]=10'd985;
font_index[2615]=10'd986;
font_index[2616]=10'd987;
font_index[2617]=10'd988;
font_index[2618]=10'd989;
font_index[2619]=10'd990;
font_index[2620]=10'd991;
font_index[2621]=10'd0;
font_index[2622]=10'd0;
font_index[2623]=10'd0;
font_index[2624]=10'd0;
font_index[2625]=10'd0;
font_index[2626]=10'd0;
font_index[2627]=10'd214;
font_index[2628]=10'd214;
font_index[2629]=10'd214;
font_index[2630]=10'd251;
font_index[2631]=10'd73;
font_index[2632]=10'd73;
font_index[2633]=10'd73;
font_index[2634]=10'd992;
font_index[2635]=10'd992;
font_index[2636]=10'd992;
font_index[2637]=10'd992;
font_index[2638]=10'd993;
font_index[2639]=10'd993;
font_index[2640]=10'd993;
font_index[2641]=10'd994;
font_index[2642]=10'd992;
font_index[2643]=10'd992;
font_index[2644]=10'd992;
font_index[2645]=10'd992;
font_index[2646]=10'd992;
font_index[2647]=10'd992;
font_index[2648]=10'd992;
font_index[2649]=10'd114;
font_index[2650]=10'd114;
font_index[2651]=10'd114;
font_index[2652]=10'd114;
font_index[2653]=10'd0;
font_index[2654]=10'd0;
font_index[2655]=10'd0;
font_index[2656]=10'd0;
font_index[2657]=10'd0;
font_index[2658]=10'd995;
font_index[2659]=10'd620;
font_index[2660]=10'd704;
font_index[2661]=10'd705;
font_index[2662]=10'd996;
font_index[2663]=10'd448;
font_index[2664]=10'd115;
font_index[2665]=10'd115;
font_index[2666]=10'd997;
font_index[2667]=10'd998;
font_index[2668]=10'd998;
font_index[2669]=10'd999;
font_index[2670]=10'd1000;
font_index[2671]=10'd1001;
font_index[2672]=10'd1002;
font_index[2673]=10'd1003;
font_index[2674]=10'd1004;
font_index[2675]=10'd1005;
font_index[2676]=10'd1006;
font_index[2677]=10'd1007;
font_index[2678]=10'd1008;
font_index[2679]=10'd1009;
font_index[2680]=10'd1010;
font_index[2681]=10'd1011;
font_index[2682]=10'd1012;
font_index[2683]=10'd1013;
font_index[2684]=10'd1014;
font_index[2685]=10'd1015;
font_index[2686]=10'd0;
font_index[2687]=10'd0;
font_index[2688]=10'd0;
font_index[2689]=10'd0;
font_index[2690]=10'd1016;
font_index[2691]=10'd60;
font_index[2692]=10'd87;
font_index[2693]=10'd29;
font_index[2694]=10'd1017;
font_index[2695]=10'd316;
font_index[2696]=10'd542;
font_index[2697]=10'd1018;
font_index[2698]=10'd1019;
font_index[2699]=10'd504;
font_index[2700]=10'd114;
font_index[2701]=10'd1020;
font_index[2702]=10'd1021;
font_index[2703]=10'd1022;
font_index[2704]=10'd708;
font_index[2705]=10'd708;
font_index[2706]=10'd708;
font_index[2707]=10'd1023;
font_index[2708]=10'd0;
font_index[2709]=10'd0;
font_index[2710]=10'd0;
font_index[2711]=10'd0;
font_index[2712]=10'd708;
font_index[2713]=10'd708;
font_index[2714]=10'd708;
font_index[2715]=10'd1023;
font_index[2716]=10'd0;
font_index[2717]=10'd0;
font_index[2718]=10'd0;
font_index[2719]=10'd0;
font_index[2720]=10'd0;
font_index[2721]=10'd0;
font_index[2722]=10'd0;
font_index[2723]=10'd0;
font_index[2724]=10'd0;
font_index[2725]=10'd0;
font_index[2726]=10'd0;
font_index[2727]=10'd0;
font_index[2728]=10'd0;
font_index[2729]=10'd0;
font_index[2730]=10'd0;
font_index[2731]=10'd0;
font_index[2732]=10'd0;
font_index[2733]=10'd0;
font_index[2734]=10'd0;
font_index[2735]=10'd0;
font_index[2736]=10'd0;
font_index[2737]=10'd0;
font_index[2738]=10'd0;
font_index[2739]=10'd0;
font_index[2740]=10'd0;
font_index[2741]=10'd0;
font_index[2742]=10'd0;
font_index[2743]=10'd0;
font_index[2744]=10'd0;
font_index[2745]=10'd0;
font_index[2746]=10'd0;
font_index[2747]=10'd0;
font_index[2748]=10'd0;
font_index[2749]=10'd0;
font_index[2750]=10'd0;
font_index[2751]=10'd0;
font_index[2752]=10'd0;
font_index[2753]=10'd0;
font_index[2754]=10'd0;
font_index[2755]=10'd0;
font_index[2756]=10'd0;
font_index[2757]=10'd0;
font_index[2758]=10'd0;
font_index[2759]=10'd0;
font_index[2760]=10'd0;
font_index[2761]=10'd0;
font_index[2762]=10'd0;
font_index[2763]=10'd0;
font_index[2764]=10'd0;
font_index[2765]=10'd0;
font_index[2766]=10'd0;
font_index[2767]=10'd0;
font_index[2768]=10'd0;
font_index[2769]=10'd0;
font_index[2770]=10'd0;
font_index[2771]=10'd0;
font_index[2772]=10'd0;
font_index[2773]=10'd0;
font_index[2774]=10'd0;
font_index[2775]=10'd0;
font_index[2776]=10'd0;
font_index[2777]=10'd0;
font_index[2778]=10'd0;
font_index[2779]=10'd0;
font_index[2780]=10'd0;
font_index[2781]=10'd0;
font_index[2782]=10'd0;
font_index[2783]=10'd0;
font_index[2784]=10'd0;
font_index[2785]=10'd0;
font_index[2786]=10'd0;
font_index[2787]=10'd0;
font_index[2788]=10'd0;
font_index[2789]=10'd0;
font_index[2790]=10'd0;
font_index[2791]=10'd0;
font_index[2792]=10'd0;
font_index[2793]=10'd0;
font_index[2794]=10'd0;
font_index[2795]=10'd0;
font_index[2796]=10'd0;
font_index[2797]=10'd0;
font_index[2798]=10'd0;
font_index[2799]=10'd0;
font_index[2800]=10'd0;
font_index[2801]=10'd0;
font_index[2802]=10'd0;
font_index[2803]=10'd0;
font_index[2804]=10'd0;
font_index[2805]=10'd0;
font_index[2806]=10'd0;
font_index[2807]=10'd0;
font_index[2808]=10'd0;
font_index[2809]=10'd0;
font_index[2810]=10'd0;
font_index[2811]=10'd0;
font_index[2812]=10'd0;
font_index[2813]=10'd0;
font_index[2814]=10'd0;
font_index[2815]=10'd0;
font_index[2816]=10'd0;
font_index[2817]=10'd0;
font_index[2818]=10'd0;
font_index[2819]=10'd0;
font_index[2820]=10'd0;
font_index[2821]=10'd0;
font_index[2822]=10'd0;
font_index[2823]=10'd0;
font_index[2824]=10'd0;
font_index[2825]=10'd0;
font_index[2826]=10'd0;
font_index[2827]=10'd0;
font_index[2828]=10'd0;
font_index[2829]=10'd0;
font_index[2830]=10'd0;
font_index[2831]=10'd0;
font_index[2832]=10'd0;
font_index[2833]=10'd0;
font_index[2834]=10'd0;
font_index[2835]=10'd0;
font_index[2836]=10'd0;
font_index[2837]=10'd0;
font_index[2838]=10'd0;
font_index[2839]=10'd0;
font_index[2840]=10'd0;
font_index[2841]=10'd0;
font_index[2842]=10'd0;
font_index[2843]=10'd0;
font_index[2844]=10'd0;
font_index[2845]=10'd0;
font_index[2846]=10'd0;
font_index[2847]=10'd0;
font_index[2848]=10'd0;
font_index[2849]=10'd0;
font_index[2850]=10'd0;
font_index[2851]=10'd0;
font_index[2852]=10'd0;
font_index[2853]=10'd0;
font_index[2854]=10'd0;
font_index[2855]=10'd0;
font_index[2856]=10'd0;
font_index[2857]=10'd0;
font_index[2858]=10'd0;
font_index[2859]=10'd0;
font_index[2860]=10'd0;
font_index[2861]=10'd0;
font_index[2862]=10'd0;
font_index[2863]=10'd0;
font_index[2864]=10'd0;
font_index[2865]=10'd0;
font_index[2866]=10'd0;
font_index[2867]=10'd0;
font_index[2868]=10'd0;
font_index[2869]=10'd0;
font_index[2870]=10'd0;
font_index[2871]=10'd0;
font_index[2872]=10'd0;
font_index[2873]=10'd0;
font_index[2874]=10'd0;
font_index[2875]=10'd0;
font_index[2876]=10'd0;
font_index[2877]=10'd0;
font_index[2878]=10'd0;
font_index[2879]=10'd0;
font_index[2880]=10'd0;
font_index[2881]=10'd0;
font_index[2882]=10'd0;
font_index[2883]=10'd0;
font_index[2884]=10'd0;
font_index[2885]=10'd0;
font_index[2886]=10'd0;
font_index[2887]=10'd0;
font_index[2888]=10'd0;
font_index[2889]=10'd0;
font_index[2890]=10'd0;
font_index[2891]=10'd0;
font_index[2892]=10'd0;
font_index[2893]=10'd0;
font_index[2894]=10'd0;
font_index[2895]=10'd0;
font_index[2896]=10'd0;
font_index[2897]=10'd0;
font_index[2898]=10'd0;
font_index[2899]=10'd0;
font_index[2900]=10'd0;
font_index[2901]=10'd0;
font_index[2902]=10'd0;
font_index[2903]=10'd0;
font_index[2904]=10'd0;
font_index[2905]=10'd0;
font_index[2906]=10'd0;
font_index[2907]=10'd0;
font_index[2908]=10'd0;
font_index[2909]=10'd0;
font_index[2910]=10'd0;
font_index[2911]=10'd0;
font_index[2912]=10'd0;
font_index[2913]=10'd0;
font_index[2914]=10'd0;
font_index[2915]=10'd0;
font_index[2916]=10'd0;
font_index[2917]=10'd0;
font_index[2918]=10'd0;
font_index[2919]=10'd0;
font_index[2920]=10'd0;
font_index[2921]=10'd0;
font_index[2922]=10'd0;
font_index[2923]=10'd0;
font_index[2924]=10'd0;
font_index[2925]=10'd0;
font_index[2926]=10'd0;
font_index[2927]=10'd0;
font_index[2928]=10'd0;
font_index[2929]=10'd0;
font_index[2930]=10'd0;
font_index[2931]=10'd0;
font_index[2932]=10'd0;
font_index[2933]=10'd0;
font_index[2934]=10'd0;
font_index[2935]=10'd0;
font_index[2936]=10'd0;
font_index[2937]=10'd0;
font_index[2938]=10'd0;
font_index[2939]=10'd0;
font_index[2940]=10'd0;
font_index[2941]=10'd0;
font_index[2942]=10'd0;
font_index[2943]=10'd0;
font_index[2944]=10'd0;
font_index[2945]=10'd0;
font_index[2946]=10'd0;
font_index[2947]=10'd0;
font_index[2948]=10'd0;
font_index[2949]=10'd0;
font_index[2950]=10'd0;
font_index[2951]=10'd0;
font_index[2952]=10'd0;
font_index[2953]=10'd0;
font_index[2954]=10'd0;
font_index[2955]=10'd0;
font_index[2956]=10'd0;
font_index[2957]=10'd0;
font_index[2958]=10'd0;
font_index[2959]=10'd0;
font_index[2960]=10'd0;
font_index[2961]=10'd0;
font_index[2962]=10'd0;
font_index[2963]=10'd0;
font_index[2964]=10'd0;
font_index[2965]=10'd0;
font_index[2966]=10'd0;
font_index[2967]=10'd0;
font_index[2968]=10'd0;
font_index[2969]=10'd0;
font_index[2970]=10'd0;
font_index[2971]=10'd0;
font_index[2972]=10'd0;
font_index[2973]=10'd0;
font_index[2974]=10'd0;
font_index[2975]=10'd0;
font_index[2976]=10'd0;
font_index[2977]=10'd0;
font_index[2978]=10'd0;
font_index[2979]=10'd0;
font_index[2980]=10'd0;
font_index[2981]=10'd0;
font_index[2982]=10'd0;
font_index[2983]=10'd0;
font_index[2984]=10'd0;
font_index[2985]=10'd0;
font_index[2986]=10'd0;
font_index[2987]=10'd0;
font_index[2988]=10'd0;
font_index[2989]=10'd0;
font_index[2990]=10'd0;
font_index[2991]=10'd0;
font_index[2992]=10'd0;
font_index[2993]=10'd0;
font_index[2994]=10'd0;
font_index[2995]=10'd0;
font_index[2996]=10'd0;
font_index[2997]=10'd0;
font_index[2998]=10'd0;
font_index[2999]=10'd0;
font_index[3000]=10'd0;
font_index[3001]=10'd0;
font_index[3002]=10'd0;
font_index[3003]=10'd0;
font_index[3004]=10'd0;
font_index[3005]=10'd0;
font_index[3006]=10'd0;
font_index[3007]=10'd0;
font_index[3008]=10'd0;
font_index[3009]=10'd0;
font_index[3010]=10'd0;
font_index[3011]=10'd0;
font_index[3012]=10'd0;
font_index[3013]=10'd0;
font_index[3014]=10'd0;
font_index[3015]=10'd0;
font_index[3016]=10'd0;
font_index[3017]=10'd0;
font_index[3018]=10'd0;
font_index[3019]=10'd0;
font_index[3020]=10'd0;
font_index[3021]=10'd0;
font_index[3022]=10'd0;
font_index[3023]=10'd0;
font_index[3024]=10'd0;
font_index[3025]=10'd0;
font_index[3026]=10'd0;
font_index[3027]=10'd0;
font_index[3028]=10'd0;
font_index[3029]=10'd0;
font_index[3030]=10'd0;
font_index[3031]=10'd0;
font_index[3032]=10'd0;
font_index[3033]=10'd0;
font_index[3034]=10'd0;
font_index[3035]=10'd0;
font_index[3036]=10'd0;
font_index[3037]=10'd0;
font_index[3038]=10'd0;
font_index[3039]=10'd0;
font_index[3040]=10'd0;
font_index[3041]=10'd0;
font_index[3042]=10'd0;
font_index[3043]=10'd0;
font_index[3044]=10'd0;
font_index[3045]=10'd0;
font_index[3046]=10'd0;
font_index[3047]=10'd0;
font_index[3048]=10'd0;
font_index[3049]=10'd0;
font_index[3050]=10'd0;
font_index[3051]=10'd0;
font_index[3052]=10'd0;
font_index[3053]=10'd0;
font_index[3054]=10'd0;
font_index[3055]=10'd0;
font_index[3056]=10'd0;
font_index[3057]=10'd0;
font_index[3058]=10'd0;
font_index[3059]=10'd0;
font_index[3060]=10'd0;
font_index[3061]=10'd0;
font_index[3062]=10'd0;
font_index[3063]=10'd0;
font_index[3064]=10'd0;
font_index[3065]=10'd0;
font_index[3066]=10'd0;
font_index[3067]=10'd0;
font_index[3068]=10'd0;
font_index[3069]=10'd0;
font_index[3070]=10'd0;
font_index[3071]=10'd0;
font_index[3072]=10'd0;
font_index[3073]=10'd0;
font_index[3074]=10'd0;
font_index[3075]=10'd0;
font_index[3076]=10'd0;
font_index[3077]=10'd0;
font_index[3078]=10'd0;
font_index[3079]=10'd0;
font_index[3080]=10'd0;
font_index[3081]=10'd0;
font_index[3082]=10'd0;
font_index[3083]=10'd0;
font_index[3084]=10'd0;
font_index[3085]=10'd0;
font_index[3086]=10'd0;
font_index[3087]=10'd0;
font_index[3088]=10'd0;
font_index[3089]=10'd0;
font_index[3090]=10'd0;
font_index[3091]=10'd0;
font_index[3092]=10'd0;
font_index[3093]=10'd0;
font_index[3094]=10'd0;
font_index[3095]=10'd0;
font_index[3096]=10'd0;
font_index[3097]=10'd0;
font_index[3098]=10'd0;
font_index[3099]=10'd0;
font_index[3100]=10'd0;
font_index[3101]=10'd0;
font_index[3102]=10'd0;
font_index[3103]=10'd0;
font_index[3104]=10'd0;
font_index[3105]=10'd0;
font_index[3106]=10'd0;
font_index[3107]=10'd0;
font_index[3108]=10'd0;
font_index[3109]=10'd0;
font_index[3110]=10'd0;
font_index[3111]=10'd0;
font_index[3112]=10'd0;
font_index[3113]=10'd0;
font_index[3114]=10'd0;
font_index[3115]=10'd0;
font_index[3116]=10'd0;
font_index[3117]=10'd0;
font_index[3118]=10'd0;
font_index[3119]=10'd0;
font_index[3120]=10'd0;
font_index[3121]=10'd0;
font_index[3122]=10'd0;
font_index[3123]=10'd0;
font_index[3124]=10'd0;
font_index[3125]=10'd0;
font_index[3126]=10'd0;
font_index[3127]=10'd0;
font_index[3128]=10'd0;
font_index[3129]=10'd0;
font_index[3130]=10'd0;
font_index[3131]=10'd0;
font_index[3132]=10'd0;
font_index[3133]=10'd0;
font_index[3134]=10'd0;
font_index[3135]=10'd0;
font_index[3136]=10'd0;
font_index[3137]=10'd0;
font_index[3138]=10'd0;
font_index[3139]=10'd0;
font_index[3140]=10'd0;
font_index[3141]=10'd0;
font_index[3142]=10'd0;
font_index[3143]=10'd0;
font_index[3144]=10'd0;
font_index[3145]=10'd0;
font_index[3146]=10'd0;
font_index[3147]=10'd0;
font_index[3148]=10'd0;
font_index[3149]=10'd0;
font_index[3150]=10'd0;
font_index[3151]=10'd0;
font_index[3152]=10'd0;
font_index[3153]=10'd0;
font_index[3154]=10'd0;
font_index[3155]=10'd0;
font_index[3156]=10'd0;
font_index[3157]=10'd0;
font_index[3158]=10'd0;
font_index[3159]=10'd0;
font_index[3160]=10'd0;
font_index[3161]=10'd0;
font_index[3162]=10'd0;
font_index[3163]=10'd0;
font_index[3164]=10'd0;
font_index[3165]=10'd0;
font_index[3166]=10'd0;
font_index[3167]=10'd0;
font_index[3168]=10'd0;
font_index[3169]=10'd0;
font_index[3170]=10'd0;
font_index[3171]=10'd0;
font_index[3172]=10'd0;
font_index[3173]=10'd0;
font_index[3174]=10'd0;
font_index[3175]=10'd0;
font_index[3176]=10'd0;
font_index[3177]=10'd0;
font_index[3178]=10'd0;
font_index[3179]=10'd0;
font_index[3180]=10'd0;
font_index[3181]=10'd0;
font_index[3182]=10'd0;
font_index[3183]=10'd0;
font_index[3184]=10'd0;
font_index[3185]=10'd0;
font_index[3186]=10'd0;
font_index[3187]=10'd0;
font_index[3188]=10'd0;
font_index[3189]=10'd0;
font_index[3190]=10'd0;
font_index[3191]=10'd0;
font_index[3192]=10'd0;
font_index[3193]=10'd0;
font_index[3194]=10'd0;
font_index[3195]=10'd0;
font_index[3196]=10'd0;
font_index[3197]=10'd0;
font_index[3198]=10'd0;
font_index[3199]=10'd0;
font_index[3200]=10'd0;
font_index[3201]=10'd0;
font_index[3202]=10'd0;
font_index[3203]=10'd0;
font_index[3204]=10'd0;
font_index[3205]=10'd0;
font_index[3206]=10'd0;
font_index[3207]=10'd0;
font_index[3208]=10'd0;
font_index[3209]=10'd0;
font_index[3210]=10'd0;
font_index[3211]=10'd0;
font_index[3212]=10'd0;
font_index[3213]=10'd0;
font_index[3214]=10'd0;
font_index[3215]=10'd0;
font_index[3216]=10'd0;
font_index[3217]=10'd0;
font_index[3218]=10'd0;
font_index[3219]=10'd0;
font_index[3220]=10'd0;
font_index[3221]=10'd0;
font_index[3222]=10'd0;
font_index[3223]=10'd0;
font_index[3224]=10'd0;
font_index[3225]=10'd0;
font_index[3226]=10'd0;
font_index[3227]=10'd0;
font_index[3228]=10'd0;
font_index[3229]=10'd0;
font_index[3230]=10'd0;
font_index[3231]=10'd0;
font_index[3232]=10'd0;
font_index[3233]=10'd0;
font_index[3234]=10'd0;
font_index[3235]=10'd0;
font_index[3236]=10'd0;
font_index[3237]=10'd0;
font_index[3238]=10'd0;
font_index[3239]=10'd0;
font_index[3240]=10'd0;
font_index[3241]=10'd0;
font_index[3242]=10'd0;
font_index[3243]=10'd0;
font_index[3244]=10'd0;
font_index[3245]=10'd0;
font_index[3246]=10'd0;
font_index[3247]=10'd0;
font_index[3248]=10'd0;
font_index[3249]=10'd0;
font_index[3250]=10'd0;
font_index[3251]=10'd0;
font_index[3252]=10'd0;
font_index[3253]=10'd0;
font_index[3254]=10'd0;
font_index[3255]=10'd0;
font_index[3256]=10'd0;
font_index[3257]=10'd0;
font_index[3258]=10'd0;
font_index[3259]=10'd0;
font_index[3260]=10'd0;
font_index[3261]=10'd0;
font_index[3262]=10'd0;
font_index[3263]=10'd0;
font_index[3264]=10'd0;
font_index[3265]=10'd0;
font_index[3266]=10'd0;
font_index[3267]=10'd0;
font_index[3268]=10'd0;
font_index[3269]=10'd0;
font_index[3270]=10'd0;
font_index[3271]=10'd0;
font_index[3272]=10'd0;
font_index[3273]=10'd0;
font_index[3274]=10'd0;
font_index[3275]=10'd0;
font_index[3276]=10'd0;
font_index[3277]=10'd0;
font_index[3278]=10'd0;
font_index[3279]=10'd0;
font_index[3280]=10'd0;
font_index[3281]=10'd0;
font_index[3282]=10'd0;
font_index[3283]=10'd0;
font_index[3284]=10'd0;
font_index[3285]=10'd0;
font_index[3286]=10'd0;
font_index[3287]=10'd0;
font_index[3288]=10'd0;
font_index[3289]=10'd0;
font_index[3290]=10'd0;
font_index[3291]=10'd0;
font_index[3292]=10'd0;
font_index[3293]=10'd0;
font_index[3294]=10'd0;
font_index[3295]=10'd0;
font_index[3296]=10'd0;
font_index[3297]=10'd0;
font_index[3298]=10'd0;
font_index[3299]=10'd0;
font_index[3300]=10'd0;
font_index[3301]=10'd0;
font_index[3302]=10'd0;
font_index[3303]=10'd0;
font_index[3304]=10'd0;
font_index[3305]=10'd0;
font_index[3306]=10'd0;
font_index[3307]=10'd0;
font_index[3308]=10'd0;
font_index[3309]=10'd0;
font_index[3310]=10'd0;
font_index[3311]=10'd0;
font_index[3312]=10'd0;
font_index[3313]=10'd0;
font_index[3314]=10'd0;
font_index[3315]=10'd0;
font_index[3316]=10'd0;
font_index[3317]=10'd0;
font_index[3318]=10'd0;
font_index[3319]=10'd0;
font_index[3320]=10'd0;
font_index[3321]=10'd0;
font_index[3322]=10'd0;
font_index[3323]=10'd0;
font_index[3324]=10'd0;
font_index[3325]=10'd0;
font_index[3326]=10'd0;
font_index[3327]=10'd0;
font_index[3328]=10'd0;
font_index[3329]=10'd0;
font_index[3330]=10'd0;
font_index[3331]=10'd0;
font_index[3332]=10'd0;
font_index[3333]=10'd0;
font_index[3334]=10'd0;
font_index[3335]=10'd0;
font_index[3336]=10'd0;
font_index[3337]=10'd0;
font_index[3338]=10'd0;
font_index[3339]=10'd0;
font_index[3340]=10'd0;
font_index[3341]=10'd0;
font_index[3342]=10'd0;
font_index[3343]=10'd0;
font_index[3344]=10'd0;
font_index[3345]=10'd0;
font_index[3346]=10'd0;
font_index[3347]=10'd0;
font_index[3348]=10'd0;
font_index[3349]=10'd0;
font_index[3350]=10'd0;
font_index[3351]=10'd0;
font_index[3352]=10'd0;
font_index[3353]=10'd0;
font_index[3354]=10'd0;
font_index[3355]=10'd0;
font_index[3356]=10'd0;
font_index[3357]=10'd0;
font_index[3358]=10'd0;
font_index[3359]=10'd0;
font_index[3360]=10'd0;
font_index[3361]=10'd0;
font_index[3362]=10'd0;
font_index[3363]=10'd0;
font_index[3364]=10'd0;
font_index[3365]=10'd0;
font_index[3366]=10'd0;
font_index[3367]=10'd0;
font_index[3368]=10'd0;
font_index[3369]=10'd0;
font_index[3370]=10'd0;
font_index[3371]=10'd0;
font_index[3372]=10'd0;
font_index[3373]=10'd0;
font_index[3374]=10'd0;
font_index[3375]=10'd0;
font_index[3376]=10'd0;
font_index[3377]=10'd0;
font_index[3378]=10'd0;
font_index[3379]=10'd0;
font_index[3380]=10'd0;
font_index[3381]=10'd0;
font_index[3382]=10'd0;
font_index[3383]=10'd0;
font_index[3384]=10'd0;
font_index[3385]=10'd0;
font_index[3386]=10'd0;
font_index[3387]=10'd0;
font_index[3388]=10'd0;
font_index[3389]=10'd0;
font_index[3390]=10'd0;
font_index[3391]=10'd0;
font_index[3392]=10'd0;
font_index[3393]=10'd0;
font_index[3394]=10'd0;
font_index[3395]=10'd0;
font_index[3396]=10'd0;
font_index[3397]=10'd0;
font_index[3398]=10'd0;
font_index[3399]=10'd0;
font_index[3400]=10'd0;
font_index[3401]=10'd0;
font_index[3402]=10'd0;
font_index[3403]=10'd0;
font_index[3404]=10'd0;
font_index[3405]=10'd0;
font_index[3406]=10'd0;
font_index[3407]=10'd0;
font_index[3408]=10'd0;
font_index[3409]=10'd0;
font_index[3410]=10'd0;
font_index[3411]=10'd0;
font_index[3412]=10'd0;
font_index[3413]=10'd0;
font_index[3414]=10'd0;
font_index[3415]=10'd0;
font_index[3416]=10'd0;
font_index[3417]=10'd0;
font_index[3418]=10'd0;
font_index[3419]=10'd0;
font_index[3420]=10'd0;
font_index[3421]=10'd0;
font_index[3422]=10'd0;
font_index[3423]=10'd0;
font_index[3424]=10'd0;
font_index[3425]=10'd0;
font_index[3426]=10'd0;
font_index[3427]=10'd0;
font_index[3428]=10'd0;
font_index[3429]=10'd0;
font_index[3430]=10'd0;
font_index[3431]=10'd0;
font_index[3432]=10'd0;
font_index[3433]=10'd0;
font_index[3434]=10'd0;
font_index[3435]=10'd0;
font_index[3436]=10'd0;
font_index[3437]=10'd0;
font_index[3438]=10'd0;
font_index[3439]=10'd0;
font_index[3440]=10'd0;
font_index[3441]=10'd0;
font_index[3442]=10'd0;
font_index[3443]=10'd0;
font_index[3444]=10'd0;
font_index[3445]=10'd0;
font_index[3446]=10'd0;
font_index[3447]=10'd0;
font_index[3448]=10'd0;
font_index[3449]=10'd0;
font_index[3450]=10'd0;
font_index[3451]=10'd0;
font_index[3452]=10'd0;
font_index[3453]=10'd0;
font_index[3454]=10'd0;
font_index[3455]=10'd0;
font_index[3456]=10'd0;
font_index[3457]=10'd0;
font_index[3458]=10'd0;
font_index[3459]=10'd0;
font_index[3460]=10'd0;
font_index[3461]=10'd0;
font_index[3462]=10'd0;
font_index[3463]=10'd0;
font_index[3464]=10'd0;
font_index[3465]=10'd0;
font_index[3466]=10'd0;
font_index[3467]=10'd0;
font_index[3468]=10'd0;
font_index[3469]=10'd0;
font_index[3470]=10'd0;
font_index[3471]=10'd0;
font_index[3472]=10'd0;
font_index[3473]=10'd0;
font_index[3474]=10'd0;
font_index[3475]=10'd0;
font_index[3476]=10'd0;
font_index[3477]=10'd0;
font_index[3478]=10'd0;
font_index[3479]=10'd0;
font_index[3480]=10'd0;
font_index[3481]=10'd0;
font_index[3482]=10'd0;
font_index[3483]=10'd0;
font_index[3484]=10'd0;
font_index[3485]=10'd0;
font_index[3486]=10'd0;
font_index[3487]=10'd0;
font_index[3488]=10'd0;
font_index[3489]=10'd0;
font_index[3490]=10'd0;
font_index[3491]=10'd0;
font_index[3492]=10'd0;
font_index[3493]=10'd0;
font_index[3494]=10'd0;
font_index[3495]=10'd0;
font_index[3496]=10'd0;
font_index[3497]=10'd0;
font_index[3498]=10'd0;
font_index[3499]=10'd0;
font_index[3500]=10'd0;
font_index[3501]=10'd0;
font_index[3502]=10'd0;
font_index[3503]=10'd0;
font_index[3504]=10'd0;
font_index[3505]=10'd0;
font_index[3506]=10'd0;
font_index[3507]=10'd0;
font_index[3508]=10'd0;
font_index[3509]=10'd0;
font_index[3510]=10'd0;
font_index[3511]=10'd0;
font_index[3512]=10'd0;
font_index[3513]=10'd0;
font_index[3514]=10'd0;
font_index[3515]=10'd0;
font_index[3516]=10'd0;
font_index[3517]=10'd0;
font_index[3518]=10'd0;
font_index[3519]=10'd0;
font_index[3520]=10'd0;
font_index[3521]=10'd0;
font_index[3522]=10'd0;
font_index[3523]=10'd0;
font_index[3524]=10'd0;
font_index[3525]=10'd0;
font_index[3526]=10'd0;
font_index[3527]=10'd0;
font_index[3528]=10'd0;
font_index[3529]=10'd0;
font_index[3530]=10'd0;
font_index[3531]=10'd0;
font_index[3532]=10'd0;
font_index[3533]=10'd0;
font_index[3534]=10'd0;
font_index[3535]=10'd0;
font_index[3536]=10'd0;
font_index[3537]=10'd0;
font_index[3538]=10'd0;
font_index[3539]=10'd0;
font_index[3540]=10'd0;
font_index[3541]=10'd0;
font_index[3542]=10'd0;
font_index[3543]=10'd0;
font_index[3544]=10'd0;
font_index[3545]=10'd0;
font_index[3546]=10'd0;
font_index[3547]=10'd0;
font_index[3548]=10'd0;
font_index[3549]=10'd0;
font_index[3550]=10'd0;
font_index[3551]=10'd0;
font_index[3552]=10'd0;
font_index[3553]=10'd0;
font_index[3554]=10'd0;
font_index[3555]=10'd0;
font_index[3556]=10'd0;
font_index[3557]=10'd0;
font_index[3558]=10'd0;
font_index[3559]=10'd0;
font_index[3560]=10'd0;
font_index[3561]=10'd0;
font_index[3562]=10'd0;
font_index[3563]=10'd0;
font_index[3564]=10'd0;
font_index[3565]=10'd0;
font_index[3566]=10'd0;
font_index[3567]=10'd0;
font_index[3568]=10'd0;
font_index[3569]=10'd0;
font_index[3570]=10'd0;
font_index[3571]=10'd0;
font_index[3572]=10'd0;
font_index[3573]=10'd0;
font_index[3574]=10'd0;
font_index[3575]=10'd0;
font_index[3576]=10'd0;
font_index[3577]=10'd0;
font_index[3578]=10'd0;
font_index[3579]=10'd0;
font_index[3580]=10'd0;
font_index[3581]=10'd0;
font_index[3582]=10'd0;
font_index[3583]=10'd0;
font_index[3584]=10'd0;
font_index[3585]=10'd0;
font_index[3586]=10'd0;
font_index[3587]=10'd0;
font_index[3588]=10'd0;
font_index[3589]=10'd0;
font_index[3590]=10'd0;
font_index[3591]=10'd0;
font_index[3592]=10'd0;
font_index[3593]=10'd0;
font_index[3594]=10'd0;
font_index[3595]=10'd0;
font_index[3596]=10'd0;
font_index[3597]=10'd0;
font_index[3598]=10'd0;
font_index[3599]=10'd0;
font_index[3600]=10'd0;
font_index[3601]=10'd0;
font_index[3602]=10'd0;
font_index[3603]=10'd0;
font_index[3604]=10'd0;
font_index[3605]=10'd0;
font_index[3606]=10'd0;
font_index[3607]=10'd0;
font_index[3608]=10'd0;
font_index[3609]=10'd0;
font_index[3610]=10'd0;
font_index[3611]=10'd0;
font_index[3612]=10'd0;
font_index[3613]=10'd0;
font_index[3614]=10'd0;
font_index[3615]=10'd0;
font_index[3616]=10'd0;
font_index[3617]=10'd0;
font_index[3618]=10'd0;
font_index[3619]=10'd0;
font_index[3620]=10'd0;
font_index[3621]=10'd0;
font_index[3622]=10'd0;
font_index[3623]=10'd0;
font_index[3624]=10'd0;
font_index[3625]=10'd0;
font_index[3626]=10'd0;
font_index[3627]=10'd0;
font_index[3628]=10'd0;
font_index[3629]=10'd0;
font_index[3630]=10'd0;
font_index[3631]=10'd0;
font_index[3632]=10'd0;
font_index[3633]=10'd0;
font_index[3634]=10'd0;
font_index[3635]=10'd0;
font_index[3636]=10'd0;
font_index[3637]=10'd0;
font_index[3638]=10'd0;
font_index[3639]=10'd0;
font_index[3640]=10'd0;
font_index[3641]=10'd0;
font_index[3642]=10'd0;
font_index[3643]=10'd0;
font_index[3644]=10'd0;
font_index[3645]=10'd0;
font_index[3646]=10'd0;
font_index[3647]=10'd0;
font_index[3648]=10'd0;
font_index[3649]=10'd0;
font_index[3650]=10'd0;
font_index[3651]=10'd0;
font_index[3652]=10'd0;
font_index[3653]=10'd0;
font_index[3654]=10'd0;
font_index[3655]=10'd0;
font_index[3656]=10'd0;
font_index[3657]=10'd0;
font_index[3658]=10'd0;
font_index[3659]=10'd0;
font_index[3660]=10'd0;
font_index[3661]=10'd0;
font_index[3662]=10'd0;
font_index[3663]=10'd0;
font_index[3664]=10'd0;
font_index[3665]=10'd0;
font_index[3666]=10'd0;
font_index[3667]=10'd0;
font_index[3668]=10'd0;
font_index[3669]=10'd0;
font_index[3670]=10'd0;
font_index[3671]=10'd0;
font_index[3672]=10'd0;
font_index[3673]=10'd0;
font_index[3674]=10'd0;
font_index[3675]=10'd0;
font_index[3676]=10'd0;
font_index[3677]=10'd0;
font_index[3678]=10'd0;
font_index[3679]=10'd0;
font_index[3680]=10'd0;
font_index[3681]=10'd0;
font_index[3682]=10'd0;
font_index[3683]=10'd0;
font_index[3684]=10'd0;
font_index[3685]=10'd0;
font_index[3686]=10'd0;
font_index[3687]=10'd0;
font_index[3688]=10'd0;
font_index[3689]=10'd0;
font_index[3690]=10'd0;
font_index[3691]=10'd0;
font_index[3692]=10'd0;
font_index[3693]=10'd0;
font_index[3694]=10'd0;
font_index[3695]=10'd0;
font_index[3696]=10'd0;
font_index[3697]=10'd0;
font_index[3698]=10'd0;
font_index[3699]=10'd0;
font_index[3700]=10'd0;
font_index[3701]=10'd0;
font_index[3702]=10'd0;
font_index[3703]=10'd0;
font_index[3704]=10'd0;
font_index[3705]=10'd0;
font_index[3706]=10'd0;
font_index[3707]=10'd0;
font_index[3708]=10'd0;
font_index[3709]=10'd0;
font_index[3710]=10'd0;
font_index[3711]=10'd0;
font_index[3712]=10'd0;
font_index[3713]=10'd0;
font_index[3714]=10'd0;
font_index[3715]=10'd0;
font_index[3716]=10'd0;
font_index[3717]=10'd0;
font_index[3718]=10'd0;
font_index[3719]=10'd0;
font_index[3720]=10'd0;
font_index[3721]=10'd0;
font_index[3722]=10'd0;
font_index[3723]=10'd0;
font_index[3724]=10'd0;
font_index[3725]=10'd0;
font_index[3726]=10'd0;
font_index[3727]=10'd0;
font_index[3728]=10'd0;
font_index[3729]=10'd0;
font_index[3730]=10'd0;
font_index[3731]=10'd0;
font_index[3732]=10'd0;
font_index[3733]=10'd0;
font_index[3734]=10'd0;
font_index[3735]=10'd0;
font_index[3736]=10'd0;
font_index[3737]=10'd0;
font_index[3738]=10'd0;
font_index[3739]=10'd0;
font_index[3740]=10'd0;
font_index[3741]=10'd0;
font_index[3742]=10'd0;
font_index[3743]=10'd0;
font_index[3744]=10'd0;
font_index[3745]=10'd0;
font_index[3746]=10'd0;
font_index[3747]=10'd0;
font_index[3748]=10'd0;
font_index[3749]=10'd0;
font_index[3750]=10'd0;
font_index[3751]=10'd0;
font_index[3752]=10'd0;
font_index[3753]=10'd0;
font_index[3754]=10'd0;
font_index[3755]=10'd0;
font_index[3756]=10'd0;
font_index[3757]=10'd0;
font_index[3758]=10'd0;
font_index[3759]=10'd0;
font_index[3760]=10'd0;
font_index[3761]=10'd0;
font_index[3762]=10'd0;
font_index[3763]=10'd0;
font_index[3764]=10'd0;
font_index[3765]=10'd0;
font_index[3766]=10'd0;
font_index[3767]=10'd0;
font_index[3768]=10'd0;
font_index[3769]=10'd0;
font_index[3770]=10'd0;
font_index[3771]=10'd0;
font_index[3772]=10'd0;
font_index[3773]=10'd0;
font_index[3774]=10'd0;
font_index[3775]=10'd0;
font_index[3776]=10'd0;
font_index[3777]=10'd0;
font_index[3778]=10'd0;
font_index[3779]=10'd0;
font_index[3780]=10'd0;
font_index[3781]=10'd0;
font_index[3782]=10'd0;
font_index[3783]=10'd0;
font_index[3784]=10'd0;
font_index[3785]=10'd0;
font_index[3786]=10'd0;
font_index[3787]=10'd0;
font_index[3788]=10'd0;
font_index[3789]=10'd0;
font_index[3790]=10'd0;
font_index[3791]=10'd0;
font_index[3792]=10'd0;
font_index[3793]=10'd0;
font_index[3794]=10'd0;
font_index[3795]=10'd0;
font_index[3796]=10'd0;
font_index[3797]=10'd0;
font_index[3798]=10'd0;
font_index[3799]=10'd0;
font_index[3800]=10'd0;
font_index[3801]=10'd0;
font_index[3802]=10'd0;
font_index[3803]=10'd0;
font_index[3804]=10'd0;
font_index[3805]=10'd0;
font_index[3806]=10'd0;
font_index[3807]=10'd0;
font_index[3808]=10'd0;
font_index[3809]=10'd0;
font_index[3810]=10'd0;
font_index[3811]=10'd0;
font_index[3812]=10'd0;
font_index[3813]=10'd0;
font_index[3814]=10'd0;
font_index[3815]=10'd0;
font_index[3816]=10'd0;
font_index[3817]=10'd0;
font_index[3818]=10'd0;
font_index[3819]=10'd0;
font_index[3820]=10'd0;
font_index[3821]=10'd0;
font_index[3822]=10'd0;
font_index[3823]=10'd0;
font_index[3824]=10'd0;
font_index[3825]=10'd0;
font_index[3826]=10'd0;
font_index[3827]=10'd0;
font_index[3828]=10'd0;
font_index[3829]=10'd0;
font_index[3830]=10'd0;
font_index[3831]=10'd0;
font_index[3832]=10'd0;
font_index[3833]=10'd0;
font_index[3834]=10'd0;
font_index[3835]=10'd0;
font_index[3836]=10'd0;
font_index[3837]=10'd0;
font_index[3838]=10'd0;
font_index[3839]=10'd0;
font_index[3840]=10'd0;
font_index[3841]=10'd0;
font_index[3842]=10'd0;
font_index[3843]=10'd0;
font_index[3844]=10'd0;
font_index[3845]=10'd0;
font_index[3846]=10'd0;
font_index[3847]=10'd0;
font_index[3848]=10'd0;
font_index[3849]=10'd0;
font_index[3850]=10'd0;
font_index[3851]=10'd0;
font_index[3852]=10'd0;
font_index[3853]=10'd0;
font_index[3854]=10'd0;
font_index[3855]=10'd0;
font_index[3856]=10'd0;
font_index[3857]=10'd0;
font_index[3858]=10'd0;
font_index[3859]=10'd0;
font_index[3860]=10'd0;
font_index[3861]=10'd0;
font_index[3862]=10'd0;
font_index[3863]=10'd0;
font_index[3864]=10'd0;
font_index[3865]=10'd0;
font_index[3866]=10'd0;
font_index[3867]=10'd0;
font_index[3868]=10'd0;
font_index[3869]=10'd0;
font_index[3870]=10'd0;
font_index[3871]=10'd0;
font_index[3872]=10'd0;
font_index[3873]=10'd0;
font_index[3874]=10'd0;
font_index[3875]=10'd0;
font_index[3876]=10'd0;
font_index[3877]=10'd0;
font_index[3878]=10'd0;
font_index[3879]=10'd0;
font_index[3880]=10'd0;
font_index[3881]=10'd0;
font_index[3882]=10'd0;
font_index[3883]=10'd0;
font_index[3884]=10'd0;
font_index[3885]=10'd0;
font_index[3886]=10'd0;
font_index[3887]=10'd0;
font_index[3888]=10'd0;
font_index[3889]=10'd0;
font_index[3890]=10'd0;
font_index[3891]=10'd0;
font_index[3892]=10'd0;
font_index[3893]=10'd0;
font_index[3894]=10'd0;
font_index[3895]=10'd0;
font_index[3896]=10'd0;
font_index[3897]=10'd0;
font_index[3898]=10'd0;
font_index[3899]=10'd0;
font_index[3900]=10'd0;
font_index[3901]=10'd0;
font_index[3902]=10'd0;
font_index[3903]=10'd0;
font_index[3904]=10'd0;
font_index[3905]=10'd0;
font_index[3906]=10'd0;
font_index[3907]=10'd0;
font_index[3908]=10'd0;
font_index[3909]=10'd0;
font_index[3910]=10'd0;
font_index[3911]=10'd0;
font_index[3912]=10'd0;
font_index[3913]=10'd0;
font_index[3914]=10'd0;
font_index[3915]=10'd0;
font_index[3916]=10'd0;
font_index[3917]=10'd0;
font_index[3918]=10'd0;
font_index[3919]=10'd0;
font_index[3920]=10'd0;
font_index[3921]=10'd0;
font_index[3922]=10'd0;
font_index[3923]=10'd0;
font_index[3924]=10'd0;
font_index[3925]=10'd0;
font_index[3926]=10'd0;
font_index[3927]=10'd0;
font_index[3928]=10'd0;
font_index[3929]=10'd0;
font_index[3930]=10'd0;
font_index[3931]=10'd0;
font_index[3932]=10'd0;
font_index[3933]=10'd0;
font_index[3934]=10'd0;
font_index[3935]=10'd0;
font_index[3936]=10'd0;
font_index[3937]=10'd0;
font_index[3938]=10'd0;
font_index[3939]=10'd0;
font_index[3940]=10'd0;
font_index[3941]=10'd0;
font_index[3942]=10'd0;
font_index[3943]=10'd0;
font_index[3944]=10'd0;
font_index[3945]=10'd0;
font_index[3946]=10'd0;
font_index[3947]=10'd0;
font_index[3948]=10'd0;
font_index[3949]=10'd0;
font_index[3950]=10'd0;
font_index[3951]=10'd0;
font_index[3952]=10'd0;
font_index[3953]=10'd0;
font_index[3954]=10'd0;
font_index[3955]=10'd0;
font_index[3956]=10'd0;
font_index[3957]=10'd0;
font_index[3958]=10'd0;
font_index[3959]=10'd0;
font_index[3960]=10'd0;
font_index[3961]=10'd0;
font_index[3962]=10'd0;
font_index[3963]=10'd0;
font_index[3964]=10'd0;
font_index[3965]=10'd0;
font_index[3966]=10'd0;
font_index[3967]=10'd0;
font_index[3968]=10'd0;
font_index[3969]=10'd0;
font_index[3970]=10'd0;
font_index[3971]=10'd0;
font_index[3972]=10'd0;
font_index[3973]=10'd0;
font_index[3974]=10'd0;
font_index[3975]=10'd0;
font_index[3976]=10'd0;
font_index[3977]=10'd0;
font_index[3978]=10'd0;
font_index[3979]=10'd0;
font_index[3980]=10'd0;
font_index[3981]=10'd0;
font_index[3982]=10'd0;
font_index[3983]=10'd0;
font_index[3984]=10'd0;
font_index[3985]=10'd0;
font_index[3986]=10'd0;
font_index[3987]=10'd0;
font_index[3988]=10'd0;
font_index[3989]=10'd0;
font_index[3990]=10'd0;
font_index[3991]=10'd0;
font_index[3992]=10'd0;
font_index[3993]=10'd0;
font_index[3994]=10'd0;
font_index[3995]=10'd0;
font_index[3996]=10'd0;
font_index[3997]=10'd0;
font_index[3998]=10'd0;
font_index[3999]=10'd0;
font_index[4000]=10'd0;
font_index[4001]=10'd0;
font_index[4002]=10'd0;
font_index[4003]=10'd0;
font_index[4004]=10'd0;
font_index[4005]=10'd0;
font_index[4006]=10'd0;
font_index[4007]=10'd0;
font_index[4008]=10'd0;
font_index[4009]=10'd0;
font_index[4010]=10'd0;
font_index[4011]=10'd0;
font_index[4012]=10'd0;
font_index[4013]=10'd0;
font_index[4014]=10'd0;
font_index[4015]=10'd0;
font_index[4016]=10'd0;
font_index[4017]=10'd0;
font_index[4018]=10'd0;
font_index[4019]=10'd0;
font_index[4020]=10'd0;
font_index[4021]=10'd0;
font_index[4022]=10'd0;
font_index[4023]=10'd0;
font_index[4024]=10'd0;
font_index[4025]=10'd0;
font_index[4026]=10'd0;
font_index[4027]=10'd0;
font_index[4028]=10'd0;
font_index[4029]=10'd0;
font_index[4030]=10'd0;
font_index[4031]=10'd0;
font_index[4032]=10'd0;
font_index[4033]=10'd0;
font_index[4034]=10'd0;
font_index[4035]=10'd0;
font_index[4036]=10'd0;
font_index[4037]=10'd0;
font_index[4038]=10'd0;
font_index[4039]=10'd0;
font_index[4040]=10'd0;
font_index[4041]=10'd0;
font_index[4042]=10'd0;
font_index[4043]=10'd0;
font_index[4044]=10'd0;
font_index[4045]=10'd0;
font_index[4046]=10'd0;
font_index[4047]=10'd0;
font_index[4048]=10'd0;
font_index[4049]=10'd0;
font_index[4050]=10'd0;
font_index[4051]=10'd0;
font_index[4052]=10'd0;
font_index[4053]=10'd0;
font_index[4054]=10'd0;
font_index[4055]=10'd0;
font_index[4056]=10'd0;
font_index[4057]=10'd0;
font_index[4058]=10'd0;
font_index[4059]=10'd0;
font_index[4060]=10'd0;
font_index[4061]=10'd0;
font_index[4062]=10'd0;
font_index[4063]=10'd0;
font_index[4064]=10'd0;
font_index[4065]=10'd0;
font_index[4066]=10'd0;
font_index[4067]=10'd0;
font_index[4068]=10'd0;
font_index[4069]=10'd0;
font_index[4070]=10'd0;
font_index[4071]=10'd0;
font_index[4072]=10'd0;
font_index[4073]=10'd0;
font_index[4074]=10'd0;
font_index[4075]=10'd0;
font_index[4076]=10'd0;
font_index[4077]=10'd0;
font_index[4078]=10'd0;
font_index[4079]=10'd0;
font_index[4080]=10'd0;
font_index[4081]=10'd0;
font_index[4082]=10'd0;
font_index[4083]=10'd0;
font_index[4084]=10'd0;
font_index[4085]=10'd0;
font_index[4086]=10'd0;
font_index[4087]=10'd0;
font_index[4088]=10'd0;
font_index[4089]=10'd0;
font_index[4090]=10'd0;
font_index[4091]=10'd0;
font_index[4092]=10'd0;
font_index[4093]=10'd0;
font_index[4094]=10'd0;
font_index[4095]=10'd0;
end
reg [FONT_ADDR_BITS-1:0] address_q;
// The 1025th unique row belongs only to five rows of 台. Keep the existing
// 10-bit dictionary and align this tiny exception with its two ROM stages.
reg font_extra_q1,font_extra_q2;
always @(posedge I_clk)begin
 address_q<={2'd0,font_index[{glyph_id,glyph_row}]};
 font_extra_q1<=glyph_id==7'd84 && (glyph_row[4:2]==3'd5 || glyph_row==5'd28);
 font_extra_q2<=font_extra_q1;
end
reg [23:0] base_q1, base_q2, color_q1, color_q2;
reg [4:0] col_q1, col_q2;
reg text_q1, text_q2, de_q1, de_q2;
wire [31:0] font_row_q;
wire [31:0] font_extra_row=32'h078001E0;
wire font_pixel=font_extra_q2?font_extra_row[5'd31-col_q2]:font_row_q[5'd31-col_q2];
osd_font32_dictionary u_font32(.I_clk(I_clk),.I_addr(address_q[9:0]),.O_row(font_row_q));
always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        base_q1<=0; base_q2<=0; color_q1<=0; color_q2<=0;
        col_q1<=0; col_q2<=0; text_q1<=0; text_q2<=0; de_q1<=0; de_q2<=0; O_rgb<=0;
    end else begin
        base_q1<=base_rgb; color_q1<=text_rgb; col_q1<=glyph_col; text_q1<=text_enable; de_q1<=I_de;
        base_q2<=base_q1; color_q2<=color_q1; col_q2<=col_q1; text_q2<=text_q1; de_q2<=de_q1;
        if (!de_q2) O_rgb<=0;
        else if (text_q2 && font_pixel) O_rgb<=color_q2;
        else O_rgb<=base_q2;
    end
end
endmodule

// FONT32_ROM_BEGIN
module osd_font32_rom (
    input wire I_clk, input wire [11:0] I_addr,
    output reg [31:0] O_row
);
(* rom_style = "bram" *) reg [31:0] rows [0:4095];
// Locally rasterized 32x32, MSB is left. Inline INIT avoids external path errors.
initial begin
    rows[0] = 32'h00000000;
    rows[1] = 32'h00000000;
    rows[2] = 32'h00000000;
    rows[3] = 32'h00000000;
    rows[4] = 32'h00000000;
    rows[5] = 32'h00000000;
    rows[6] = 32'h00000000;
    rows[7] = 32'h00000000;
    rows[8] = 32'h00000000;
    rows[9] = 32'h00000000;
    rows[10] = 32'h00000000;
    rows[11] = 32'h00000000;
    rows[12] = 32'h00000000;
    rows[13] = 32'h00000000;
    rows[14] = 32'h00000000;
    rows[15] = 32'h00000000;
    rows[16] = 32'h00000000;
    rows[17] = 32'h00000000;
    rows[18] = 32'h00000000;
    rows[19] = 32'h00000000;
    rows[20] = 32'h00000000;
    rows[21] = 32'h00000000;
    rows[22] = 32'h00000000;
    rows[23] = 32'h00000000;
    rows[24] = 32'h00000000;
    rows[25] = 32'h00000000;
    rows[26] = 32'h00000000;
    rows[27] = 32'h00000000;
    rows[28] = 32'h00000000;
    rows[29] = 32'h00000000;
    rows[30] = 32'h00000000;
    rows[31] = 32'h00000000;
    rows[32] = 32'h00000000;
    rows[33] = 32'h00000000;
    rows[34] = 32'h00000000;
    rows[35] = 32'h00000000;
    rows[36] = 32'h0003F000;
    rows[37] = 32'h000FF800;
    rows[38] = 32'h001FFC00;
    rows[39] = 32'h001F7E00;
    rows[40] = 32'h003C1E00;
    rows[41] = 32'h003C1F00;
    rows[42] = 32'h007C0F00;
    rows[43] = 32'h00780F00;
    rows[44] = 32'h00780F00;
    rows[45] = 32'h00780F00;
    rows[46] = 32'h00780F00;
    rows[47] = 32'h00780F00;
    rows[48] = 32'h00780F00;
    rows[49] = 32'h00780F00;
    rows[50] = 32'h00780F00;
    rows[51] = 32'h007C1E00;
    rows[52] = 32'h003C1E00;
    rows[53] = 32'h003F7E00;
    rows[54] = 32'h001FFC00;
    rows[55] = 32'h000FF800;
    rows[56] = 32'h0007E000;
    rows[57] = 32'h00000000;
    rows[58] = 32'h00000000;
    rows[59] = 32'h00000000;
    rows[60] = 32'h00000000;
    rows[61] = 32'h00000000;
    rows[62] = 32'h00000000;
    rows[63] = 32'h00000000;
    rows[64] = 32'h00000000;
    rows[65] = 32'h00000000;
    rows[66] = 32'h00000000;
    rows[67] = 32'h00000000;
    rows[68] = 32'h0000E000;
    rows[69] = 32'h0003E000;
    rows[70] = 32'h000FE000;
    rows[71] = 32'h003FE000;
    rows[72] = 32'h003FE000;
    rows[73] = 32'h003DE000;
    rows[74] = 32'h0031E000;
    rows[75] = 32'h0001E000;
    rows[76] = 32'h0001E000;
    rows[77] = 32'h0001E000;
    rows[78] = 32'h0001E000;
    rows[79] = 32'h0001E000;
    rows[80] = 32'h0001E000;
    rows[81] = 32'h0001E000;
    rows[82] = 32'h0001E000;
    rows[83] = 32'h0001E000;
    rows[84] = 32'h0001E000;
    rows[85] = 32'h0001E000;
    rows[86] = 32'h0001E000;
    rows[87] = 32'h0001E000;
    rows[88] = 32'h0001E000;
    rows[89] = 32'h00000000;
    rows[90] = 32'h00000000;
    rows[91] = 32'h00000000;
    rows[92] = 32'h00000000;
    rows[93] = 32'h00000000;
    rows[94] = 32'h00000000;
    rows[95] = 32'h00000000;
    rows[96] = 32'h00000000;
    rows[97] = 32'h00000000;
    rows[98] = 32'h00000000;
    rows[99] = 32'h00000000;
    rows[100] = 32'h000FE000;
    rows[101] = 32'h003FF000;
    rows[102] = 32'h003FF800;
    rows[103] = 32'h003FFC00;
    rows[104] = 32'h00307C00;
    rows[105] = 32'h00003C00;
    rows[106] = 32'h00003C00;
    rows[107] = 32'h00003C00;
    rows[108] = 32'h00007C00;
    rows[109] = 32'h0000F800;
    rows[110] = 32'h0001F800;
    rows[111] = 32'h0003F000;
    rows[112] = 32'h0007E000;
    rows[113] = 32'h000FC000;
    rows[114] = 32'h001F0000;
    rows[115] = 32'h003E0000;
    rows[116] = 32'h003C0000;
    rows[117] = 32'h007FFC00;
    rows[118] = 32'h007FFC00;
    rows[119] = 32'h007FFC00;
    rows[120] = 32'h007FFC00;
    rows[121] = 32'h00000000;
    rows[122] = 32'h00000000;
    rows[123] = 32'h00000000;
    rows[124] = 32'h00000000;
    rows[125] = 32'h00000000;
    rows[126] = 32'h00000000;
    rows[127] = 32'h00000000;
    rows[128] = 32'h00000000;
    rows[129] = 32'h00000000;
    rows[130] = 32'h00000000;
    rows[131] = 32'h00000000;
    rows[132] = 32'h000FE000;
    rows[133] = 32'h001FF000;
    rows[134] = 32'h001FF800;
    rows[135] = 32'h001FFC00;
    rows[136] = 32'h00007C00;
    rows[137] = 32'h00003C00;
    rows[138] = 32'h00003C00;
    rows[139] = 32'h00003C00;
    rows[140] = 32'h0000F800;
    rows[141] = 32'h000FF000;
    rows[142] = 32'h000FE000;
    rows[143] = 32'h000FF800;
    rows[144] = 32'h0001FC00;
    rows[145] = 32'h00003E00;
    rows[146] = 32'h00001E00;
    rows[147] = 32'h00001E00;
    rows[148] = 32'h00203E00;
    rows[149] = 32'h003DFE00;
    rows[150] = 32'h003FFC00;
    rows[151] = 32'h003FF800;
    rows[152] = 32'h001FE000;
    rows[153] = 32'h00000000;
    rows[154] = 32'h00000000;
    rows[155] = 32'h00000000;
    rows[156] = 32'h00000000;
    rows[157] = 32'h00000000;
    rows[158] = 32'h00000000;
    rows[159] = 32'h00000000;
    rows[160] = 32'h00000000;
    rows[161] = 32'h00000000;
    rows[162] = 32'h00000000;
    rows[163] = 32'h00000000;
    rows[164] = 32'h00003C00;
    rows[165] = 32'h00007C00;
    rows[166] = 32'h0000FC00;
    rows[167] = 32'h0000FC00;
    rows[168] = 32'h0001FC00;
    rows[169] = 32'h0003FC00;
    rows[170] = 32'h0003BC00;
    rows[171] = 32'h0007BC00;
    rows[172] = 32'h000F3C00;
    rows[173] = 32'h001E3C00;
    rows[174] = 32'h003C3C00;
    rows[175] = 32'h00783C00;
    rows[176] = 32'h00F03C00;
    rows[177] = 32'h00FFFF80;
    rows[178] = 32'h00FFFF80;
    rows[179] = 32'h00FFFF80;
    rows[180] = 32'h00003C00;
    rows[181] = 32'h00003C00;
    rows[182] = 32'h00003C00;
    rows[183] = 32'h00003C00;
    rows[184] = 32'h00003C00;
    rows[185] = 32'h00000000;
    rows[186] = 32'h00000000;
    rows[187] = 32'h00000000;
    rows[188] = 32'h00000000;
    rows[189] = 32'h00000000;
    rows[190] = 32'h00000000;
    rows[191] = 32'h00000000;
    rows[192] = 32'h00000000;
    rows[193] = 32'h00000000;
    rows[194] = 32'h00000000;
    rows[195] = 32'h00000000;
    rows[196] = 32'h000FFC00;
    rows[197] = 32'h000FFC00;
    rows[198] = 32'h000FFC00;
    rows[199] = 32'h001FFC00;
    rows[200] = 32'h001E0000;
    rows[201] = 32'h001E0000;
    rows[202] = 32'h001E0000;
    rows[203] = 32'h001E0000;
    rows[204] = 32'h001FF000;
    rows[205] = 32'h001FF800;
    rows[206] = 32'h001FFC00;
    rows[207] = 32'h001FFE00;
    rows[208] = 32'h00003E00;
    rows[209] = 32'h00001E00;
    rows[210] = 32'h00001E00;
    rows[211] = 32'h00001E00;
    rows[212] = 32'h00103E00;
    rows[213] = 32'h001EFC00;
    rows[214] = 32'h001FFC00;
    rows[215] = 32'h001FF800;
    rows[216] = 32'h0007E000;
    rows[217] = 32'h00000000;
    rows[218] = 32'h00000000;
    rows[219] = 32'h00000000;
    rows[220] = 32'h00000000;
    rows[221] = 32'h00000000;
    rows[222] = 32'h00000000;
    rows[223] = 32'h00000000;
    rows[224] = 32'h00000000;
    rows[225] = 32'h00000000;
    rows[226] = 32'h00000000;
    rows[227] = 32'h00000000;
    rows[228] = 32'h0000FE00;
    rows[229] = 32'h0003FE00;
    rows[230] = 32'h0007FE00;
    rows[231] = 32'h000FEE00;
    rows[232] = 32'h000F0000;
    rows[233] = 32'h001E0000;
    rows[234] = 32'h001E0000;
    rows[235] = 32'h003C0000;
    rows[236] = 32'h003CF800;
    rows[237] = 32'h003DFC00;
    rows[238] = 32'h003FFE00;
    rows[239] = 32'h003F3F00;
    rows[240] = 32'h003E1F00;
    rows[241] = 32'h003C0F00;
    rows[242] = 32'h003C0F00;
    rows[243] = 32'h003C0F00;
    rows[244] = 32'h001E1F00;
    rows[245] = 32'h001FFE00;
    rows[246] = 32'h000FFE00;
    rows[247] = 32'h0007FC00;
    rows[248] = 32'h0003F000;
    rows[249] = 32'h00000000;
    rows[250] = 32'h00000000;
    rows[251] = 32'h00000000;
    rows[252] = 32'h00000000;
    rows[253] = 32'h00000000;
    rows[254] = 32'h00000000;
    rows[255] = 32'h00000000;
    rows[256] = 32'h00000000;
    rows[257] = 32'h00000000;
    rows[258] = 32'h00000000;
    rows[259] = 32'h00000000;
    rows[260] = 32'h007FFF00;
    rows[261] = 32'h007FFF00;
    rows[262] = 32'h007FFF00;
    rows[263] = 32'h007FFF00;
    rows[264] = 32'h00001E00;
    rows[265] = 32'h00003C00;
    rows[266] = 32'h00007C00;
    rows[267] = 32'h00007800;
    rows[268] = 32'h0000F800;
    rows[269] = 32'h0000F000;
    rows[270] = 32'h0001F000;
    rows[271] = 32'h0001F000;
    rows[272] = 32'h0003E000;
    rows[273] = 32'h0003E000;
    rows[274] = 32'h0003C000;
    rows[275] = 32'h0007C000;
    rows[276] = 32'h0007C000;
    rows[277] = 32'h0007C000;
    rows[278] = 32'h000F8000;
    rows[279] = 32'h000F8000;
    rows[280] = 32'h000F8000;
    rows[281] = 32'h00000000;
    rows[282] = 32'h00000000;
    rows[283] = 32'h00000000;
    rows[284] = 32'h00000000;
    rows[285] = 32'h00000000;
    rows[286] = 32'h00000000;
    rows[287] = 32'h00000000;
    rows[288] = 32'h00000000;
    rows[289] = 32'h00000000;
    rows[290] = 32'h00000000;
    rows[291] = 32'h00000000;
    rows[292] = 32'h0003E000;
    rows[293] = 32'h000FF800;
    rows[294] = 32'h001FFC00;
    rows[295] = 32'h003F7E00;
    rows[296] = 32'h003C1E00;
    rows[297] = 32'h003C1E00;
    rows[298] = 32'h003C1E00;
    rows[299] = 32'h003E3C00;
    rows[300] = 32'h001FFC00;
    rows[301] = 32'h000FF800;
    rows[302] = 32'h000FF800;
    rows[303] = 32'h001FFC00;
    rows[304] = 32'h003C1E00;
    rows[305] = 32'h00780F00;
    rows[306] = 32'h00780F00;
    rows[307] = 32'h00780F00;
    rows[308] = 32'h007C1F00;
    rows[309] = 32'h007E3F00;
    rows[310] = 32'h003FFE00;
    rows[311] = 32'h001FFC00;
    rows[312] = 32'h0007F000;
    rows[313] = 32'h00000000;
    rows[314] = 32'h00000000;
    rows[315] = 32'h00000000;
    rows[316] = 32'h00000000;
    rows[317] = 32'h00000000;
    rows[318] = 32'h00000000;
    rows[319] = 32'h00000000;
    rows[320] = 32'h00000000;
    rows[321] = 32'h00000000;
    rows[322] = 32'h00000000;
    rows[323] = 32'h00000000;
    rows[324] = 32'h0007E000;
    rows[325] = 32'h001FF000;
    rows[326] = 32'h003FF800;
    rows[327] = 32'h003FFC00;
    rows[328] = 32'h007C3C00;
    rows[329] = 32'h00781E00;
    rows[330] = 32'h00781E00;
    rows[331] = 32'h00781E00;
    rows[332] = 32'h007C1E00;
    rows[333] = 32'h007C3E00;
    rows[334] = 32'h003FFE00;
    rows[335] = 32'h001FFE00;
    rows[336] = 32'h000FDE00;
    rows[337] = 32'h00001E00;
    rows[338] = 32'h00001C00;
    rows[339] = 32'h00003C00;
    rows[340] = 32'h00007C00;
    rows[341] = 32'h003DF800;
    rows[342] = 32'h003FF000;
    rows[343] = 32'h003FE000;
    rows[344] = 32'h001F8000;
    rows[345] = 32'h00000000;
    rows[346] = 32'h00000000;
    rows[347] = 32'h00000000;
    rows[348] = 32'h00000000;
    rows[349] = 32'h00000000;
    rows[350] = 32'h00000000;
    rows[351] = 32'h00000000;
    rows[352] = 32'h00000000;
    rows[353] = 32'h00000000;
    rows[354] = 32'h00000000;
    rows[355] = 32'h00000000;
    rows[356] = 32'h00000000;
    rows[357] = 32'h00000000;
    rows[358] = 32'h00000000;
    rows[359] = 32'h00000000;
    rows[360] = 32'h00000000;
    rows[361] = 32'h00000000;
    rows[362] = 32'h00038000;
    rows[363] = 32'h0007C000;
    rows[364] = 32'h0007C000;
    rows[365] = 32'h0007C000;
    rows[366] = 32'h00038000;
    rows[367] = 32'h00000000;
    rows[368] = 32'h00000000;
    rows[369] = 32'h00000000;
    rows[370] = 32'h00000000;
    rows[371] = 32'h00000000;
    rows[372] = 32'h00038000;
    rows[373] = 32'h0007C000;
    rows[374] = 32'h0007C000;
    rows[375] = 32'h0007C000;
    rows[376] = 32'h00038000;
    rows[377] = 32'h00000000;
    rows[378] = 32'h00000000;
    rows[379] = 32'h00000000;
    rows[380] = 32'h00000000;
    rows[381] = 32'h00000000;
    rows[382] = 32'h00000000;
    rows[383] = 32'h00000000;
    rows[384] = 32'h00000000;
    rows[385] = 32'h00000000;
    rows[386] = 32'h00000000;
    rows[387] = 32'h00000000;
    rows[388] = 32'h00000000;
    rows[389] = 32'h00000000;
    rows[390] = 32'h00000000;
    rows[391] = 32'h00000000;
    rows[392] = 32'h00000000;
    rows[393] = 32'h00000000;
    rows[394] = 32'h00000000;
    rows[395] = 32'h00000000;
    rows[396] = 32'h00000000;
    rows[397] = 32'h00000000;
    rows[398] = 32'h00000000;
    rows[399] = 32'h00000000;
    rows[400] = 32'h00000000;
    rows[401] = 32'h00000000;
    rows[402] = 32'h00000000;
    rows[403] = 32'h00000000;
    rows[404] = 32'h00038000;
    rows[405] = 32'h0007C000;
    rows[406] = 32'h0007C000;
    rows[407] = 32'h0007C000;
    rows[408] = 32'h00038000;
    rows[409] = 32'h00000000;
    rows[410] = 32'h00000000;
    rows[411] = 32'h00000000;
    rows[412] = 32'h00000000;
    rows[413] = 32'h00000000;
    rows[414] = 32'h00000000;
    rows[415] = 32'h00000000;
    rows[416] = 32'h00000000;
    rows[417] = 32'h00000000;
    rows[418] = 32'h00000000;
    rows[419] = 32'h00000000;
    rows[420] = 32'h00000000;
    rows[421] = 32'h00000000;
    rows[422] = 32'h00000000;
    rows[423] = 32'h00000000;
    rows[424] = 32'h00000000;
    rows[425] = 32'h00000000;
    rows[426] = 32'h00000000;
    rows[427] = 32'h00000000;
    rows[428] = 32'h00000000;
    rows[429] = 32'h00000000;
    rows[430] = 32'h00000000;
    rows[431] = 32'h000FF000;
    rows[432] = 32'h000FF000;
    rows[433] = 32'h000FF000;
    rows[434] = 32'h00000000;
    rows[435] = 32'h00000000;
    rows[436] = 32'h00000000;
    rows[437] = 32'h00000000;
    rows[438] = 32'h00000000;
    rows[439] = 32'h00000000;
    rows[440] = 32'h00000000;
    rows[441] = 32'h00000000;
    rows[442] = 32'h00000000;
    rows[443] = 32'h00000000;
    rows[444] = 32'h00000000;
    rows[445] = 32'h00000000;
    rows[446] = 32'h00000000;
    rows[447] = 32'h00000000;
    rows[448] = 32'h00000000;
    rows[449] = 32'h00000E60;
    rows[450] = 32'h07000EE0;
    rows[451] = 32'h07000EF0;
    rows[452] = 32'h07000E78;
    rows[453] = 32'h07000E30;
    rows[454] = 32'h071FFFFC;
    rows[455] = 32'h071FFFFC;
    rows[456] = 32'h1FDFFFFC;
    rows[457] = 32'h1FDC0F00;
    rows[458] = 32'h1FDC0F00;
    rows[459] = 32'h1FDC0F00;
    rows[460] = 32'h071C0F20;
    rows[461] = 32'h071FFF78;
    rows[462] = 32'h071FFF78;
    rows[463] = 32'h071FFF70;
    rows[464] = 32'h071CEFF0;
    rows[465] = 32'h071CEFF0;
    rows[466] = 32'h071CE7E0;
    rows[467] = 32'h071CE7C0;
    rows[468] = 32'h07DCE7C0;
    rows[469] = 32'h07FCE790;
    rows[470] = 32'h0FFFE79C;
    rows[471] = 32'h3FFFEF9C;
    rows[472] = 32'h1F3BDFBC;
    rows[473] = 32'h1C783FB8;
    rows[474] = 32'h10787FF8;
    rows[475] = 32'h0078F9F8;
    rows[476] = 32'h003071F8;
    rows[477] = 32'h001020F0;
    rows[478] = 32'h00000000;
    rows[479] = 32'h00000000;
    rows[480] = 32'h00000000;
    rows[481] = 32'h00000000;
    rows[482] = 32'h00018000;
    rows[483] = 32'h0007C000;
    rows[484] = 32'h0007C000;
    rows[485] = 32'h0007C000;
    rows[486] = 32'h1FFFFFF8;
    rows[487] = 32'h1FFFFFF8;
    rows[488] = 32'h1FFFFFF8;
    rows[489] = 32'h1FFFFFF8;
    rows[490] = 32'h0003C000;
    rows[491] = 32'h0003C000;
    rows[492] = 32'h07FFFFF0;
    rows[493] = 32'h07FFFFF0;
    rows[494] = 32'h07FFFFF0;
    rows[495] = 32'h07FFFFF0;
    rows[496] = 32'h0783C0F0;
    rows[497] = 32'h0783C0F0;
    rows[498] = 32'h0783C0F0;
    rows[499] = 32'h0783C0F0;
    rows[500] = 32'h0783C0F0;
    rows[501] = 32'h0783C0F0;
    rows[502] = 32'h0783C1F0;
    rows[503] = 32'h0783DFF0;
    rows[504] = 32'h0783DFE0;
    rows[505] = 32'h0783CFC0;
    rows[506] = 32'h0003C800;
    rows[507] = 32'h0003C000;
    rows[508] = 32'h0003C000;
    rows[509] = 32'h00000000;
    rows[510] = 32'h00000000;
    rows[511] = 32'h00000000;
    rows[512] = 32'h00000000;
    rows[513] = 32'h00000000;
    rows[514] = 32'h0380F000;
    rows[515] = 32'h0780F000;
    rows[516] = 32'h0780F000;
    rows[517] = 32'h3FFCF000;
    rows[518] = 32'h3FFCF000;
    rows[519] = 32'h3FFFFF80;
    rows[520] = 32'h0F07FF80;
    rows[521] = 32'h0E03FF80;
    rows[522] = 32'h0E03FF80;
    rows[523] = 32'h0EE0F380;
    rows[524] = 32'h1EE0F380;
    rows[525] = 32'h1CE0F380;
    rows[526] = 32'h1CE0F380;
    rows[527] = 32'h1FFCF380;
    rows[528] = 32'h1FFCE380;
    rows[529] = 32'h1FFCE380;
    rows[530] = 32'h01E1E390;
    rows[531] = 32'h00FDE39C;
    rows[532] = 32'h1FFDE39C;
    rows[533] = 32'h3FFDE39C;
    rows[534] = 32'h1FFDE398;
    rows[535] = 32'h1DE3C398;
    rows[536] = 32'h00E7C3F8;
    rows[537] = 32'h00E7C3F8;
    rows[538] = 32'h00E783F8;
    rows[539] = 32'h00E383F8;
    rows[540] = 32'h00E100E0;
    rows[541] = 32'h00000000;
    rows[542] = 32'h00000000;
    rows[543] = 32'h00000000;
    rows[544] = 32'h00000000;
    rows[545] = 32'h00000000;
    rows[546] = 32'h04030380;
    rows[547] = 32'h1E078780;
    rows[548] = 32'h1E038780;
    rows[549] = 32'h0F3FFFF8;
    rows[550] = 32'h0FBFFFF8;
    rows[551] = 32'h07BFFFF8;
    rows[552] = 32'h07807800;
    rows[553] = 32'h02007000;
    rows[554] = 32'h001FFFF0;
    rows[555] = 32'h001FFFF0;
    rows[556] = 32'h1F9FFFF0;
    rows[557] = 32'h1F9C00F0;
    rows[558] = 32'h1F9FFFF0;
    rows[559] = 32'h1F9FFFF0;
    rows[560] = 32'h079E00F0;
    rows[561] = 32'h079C00F0;
    rows[562] = 32'h079C00F0;
    rows[563] = 32'h079FFFF0;
    rows[564] = 32'h079FFFF0;
    rows[565] = 32'h079E00F0;
    rows[566] = 32'h079FFFF0;
    rows[567] = 32'h0F9FFFF0;
    rows[568] = 32'h1FFFFFE0;
    rows[569] = 32'h1FFFFFFC;
    rows[570] = 32'h1CFFFFF8;
    rows[571] = 32'h187FFFF8;
    rows[572] = 32'h0800FFF0;
    rows[573] = 32'h00000000;
    rows[574] = 32'h00000000;
    rows[575] = 32'h00000000;
    rows[576] = 32'h00000000;
    rows[577] = 32'h00000000;
    rows[578] = 32'h00018000;
    rows[579] = 32'h0007C000;
    rows[580] = 32'h0007C000;
    rows[581] = 32'h1FFFFFF8;
    rows[582] = 32'h1FFFFFF8;
    rows[583] = 32'h1FFFFFF8;
    rows[584] = 32'h1FFFFFF8;
    rows[585] = 32'h00701E00;
    rows[586] = 32'h00F81F00;
    rows[587] = 32'h01F80F80;
    rows[588] = 32'h03F007C0;
    rows[589] = 32'h07E003E0;
    rows[590] = 32'h0FD019F0;
    rows[591] = 32'h1FB81CF8;
    rows[592] = 32'h0E7C3E70;
    rows[593] = 32'h043C3C20;
    rows[594] = 32'h003E7C00;
    rows[595] = 32'h001FF800;
    rows[596] = 32'h000FF000;
    rows[597] = 32'h000FE000;
    rows[598] = 32'h000FF000;
    rows[599] = 32'h003FFE00;
    rows[600] = 32'h03FFFFC0;
    rows[601] = 32'h1FFE7FF8;
    rows[602] = 32'h1FF81FF8;
    rows[603] = 32'h0FC007F0;
    rows[604] = 32'h0E000070;
    rows[605] = 32'h00000000;
    rows[606] = 32'h00000000;
    rows[607] = 32'h00000000;
    rows[608] = 32'h00000000;
    rows[609] = 32'h00000000;
    rows[610] = 32'h04000000;
    rows[611] = 32'h0E3FFFF0;
    rows[612] = 32'h1F3FFFF0;
    rows[613] = 32'h0FBFFFF0;
    rows[614] = 32'h07C303E0;
    rows[615] = 32'h0787CF80;
    rows[616] = 32'h0201FE00;
    rows[617] = 32'h001FFFF0;
    rows[618] = 32'h001FFFF0;
    rows[619] = 32'h001FFFF0;
    rows[620] = 32'h1F9C3870;
    rows[621] = 32'h1F9FFFF0;
    rows[622] = 32'h1F9FFFF0;
    rows[623] = 32'h1F9C78F0;
    rows[624] = 32'h079C3870;
    rows[625] = 32'h079FFFF0;
    rows[626] = 32'h079FFFF0;
    rows[627] = 32'h079C78F0;
    rows[628] = 32'h079C38F0;
    rows[629] = 32'h079C3FF0;
    rows[630] = 32'h079C3BF0;
    rows[631] = 32'h0F9C3BE0;
    rows[632] = 32'h1FE00000;
    rows[633] = 32'h1FFFFFF8;
    rows[634] = 32'h1CFFFFF8;
    rows[635] = 32'h187FFFF8;
    rows[636] = 32'h08003FF0;
    rows[637] = 32'h00000000;
    rows[638] = 32'h00000000;
    rows[639] = 32'h00000000;
    rows[640] = 32'h00000000;
    rows[641] = 32'h00000000;
    rows[642] = 32'h04000000;
    rows[643] = 32'h0E1FFFE0;
    rows[644] = 32'h1F9FFFE0;
    rows[645] = 32'h07DFFFE0;
    rows[646] = 32'h039C01E0;
    rows[647] = 32'h011C01E0;
    rows[648] = 32'h001FFFE0;
    rows[649] = 32'h081FFFE0;
    rows[650] = 32'h1C1FFFE0;
    rows[651] = 32'h1E1C01E0;
    rows[652] = 32'h1F1FFFE0;
    rows[653] = 32'h0F9FFFE0;
    rows[654] = 32'h071FFFE0;
    rows[655] = 32'h02000000;
    rows[656] = 32'h00000000;
    rows[657] = 32'h003FFFF0;
    rows[658] = 32'h063FFFF0;
    rows[659] = 32'h073FFFF0;
    rows[660] = 32'h0F39CE70;
    rows[661] = 32'h0F39CE70;
    rows[662] = 32'h0F39CE70;
    rows[663] = 32'h0F39CE70;
    rows[664] = 32'h0E39CE70;
    rows[665] = 32'h1EFFFFFC;
    rows[666] = 32'h1EFFFFFC;
    rows[667] = 32'h1EFFFFFC;
    rows[668] = 32'h04000000;
    rows[669] = 32'h00000000;
    rows[670] = 32'h00000000;
    rows[671] = 32'h00000000;
    rows[672] = 32'h00000000;
    rows[673] = 32'h0001E000;
    rows[674] = 32'h0001E000;
    rows[675] = 32'h0001F000;
    rows[676] = 32'h0FFFFFF8;
    rows[677] = 32'h0FFFFFF8;
    rows[678] = 32'h0FFFFFF8;
    rows[679] = 32'h0F000000;
    rows[680] = 32'h0F1E0380;
    rows[681] = 32'h0FFFFFF8;
    rows[682] = 32'h0FFFFFF8;
    rows[683] = 32'h0FFFFFF8;
    rows[684] = 32'h0F1E0380;
    rows[685] = 32'h0F1E0380;
    rows[686] = 32'h0F1FFF80;
    rows[687] = 32'h0F1FFF80;
    rows[688] = 32'h0F1FFF80;
    rows[689] = 32'h0F000000;
    rows[690] = 32'h0F7FFFE0;
    rows[691] = 32'h0F7FFFE0;
    rows[692] = 32'h0F7FFFE0;
    rows[693] = 32'h0E1E03C0;
    rows[694] = 32'h1E0F0780;
    rows[695] = 32'h1E07DF00;
    rows[696] = 32'h1E03FE00;
    rows[697] = 32'h3E0FFFE0;
    rows[698] = 32'h3DFFFFF8;
    rows[699] = 32'h1CFF0FF8;
    rows[700] = 32'h08F000F0;
    rows[701] = 32'h00000000;
    rows[702] = 32'h00000000;
    rows[703] = 32'h00000000;
    rows[704] = 32'h00000000;
    rows[705] = 32'h00000000;
    rows[706] = 32'h01803800;
    rows[707] = 32'h01C07800;
    rows[708] = 32'h03C07C00;
    rows[709] = 32'h03DFFFF0;
    rows[710] = 32'h039FFFF0;
    rows[711] = 32'h079FFFF0;
    rows[712] = 32'h079C0070;
    rows[713] = 32'h071C0070;
    rows[714] = 32'h0F1FFFF0;
    rows[715] = 32'h0F1FFFF0;
    rows[716] = 32'h1F1FFFF0;
    rows[717] = 32'h1F1C0000;
    rows[718] = 32'h3F3C0000;
    rows[719] = 32'h3F3FFFF8;
    rows[720] = 32'h1F3FFFF8;
    rows[721] = 32'h1F3FFFF8;
    rows[722] = 32'h173F7338;
    rows[723] = 32'h073F7338;
    rows[724] = 32'h073F7338;
    rows[725] = 32'h077FFFF8;
    rows[726] = 32'h077FFFF8;
    rows[727] = 32'h07F7FFF8;
    rows[728] = 32'h07F77338;
    rows[729] = 32'h07E77338;
    rows[730] = 32'h072773F8;
    rows[731] = 32'h070773F0;
    rows[732] = 32'h070701F0;
    rows[733] = 32'h00000000;
    rows[734] = 32'h00000000;
    rows[735] = 32'h00000000;
    rows[736] = 32'h00000000;
    rows[737] = 32'h00000000;
    rows[738] = 32'h0003C000;
    rows[739] = 32'h0003C000;
    rows[740] = 32'h1FFFFFFC;
    rows[741] = 32'h1FFFFFFC;
    rows[742] = 32'h1FFFFFF8;
    rows[743] = 32'h00000000;
    rows[744] = 32'h03FFFFC0;
    rows[745] = 32'h03FFFFC0;
    rows[746] = 32'h03E003C0;
    rows[747] = 32'h03C003C0;
    rows[748] = 32'h03FFFFC0;
    rows[749] = 32'h03FFFFC0;
    rows[750] = 32'h00000000;
    rows[751] = 32'h0FFFFFF0;
    rows[752] = 32'h0FFFFFF0;
    rows[753] = 32'h0FFFFFF0;
    rows[754] = 32'h0F000070;
    rows[755] = 32'h0F3FFC70;
    rows[756] = 32'h0F3FFC70;
    rows[757] = 32'h0F381C70;
    rows[758] = 32'h0F381C70;
    rows[759] = 32'h0F3FFC70;
    rows[760] = 32'h0F3FFC70;
    rows[761] = 32'h0F3800F0;
    rows[762] = 32'h0F000FF0;
    rows[763] = 32'h0F000FE0;
    rows[764] = 32'h0F000FC0;
    rows[765] = 32'h00000000;
    rows[766] = 32'h00000000;
    rows[767] = 32'h00000000;
    rows[768] = 32'h00000000;
    rows[769] = 32'h00000000;
    rows[770] = 32'h00000000;
    rows[771] = 32'h03FFFFC0;
    rows[772] = 32'h03FFFFC0;
    rows[773] = 32'h03FFFFC0;
    rows[774] = 32'h03C003C0;
    rows[775] = 32'h03C003C0;
    rows[776] = 32'h03C003C0;
    rows[777] = 32'h03FFFFC0;
    rows[778] = 32'h03FFFFC0;
    rows[779] = 32'h03FFFFC0;
    rows[780] = 32'h00000000;
    rows[781] = 32'h00000000;
    rows[782] = 32'h3FFFFFFC;
    rows[783] = 32'h3FFFFFFC;
    rows[784] = 32'h3FFFFFFC;
    rows[785] = 32'h03E00000;
    rows[786] = 32'h01E00000;
    rows[787] = 32'h03FFFFC0;
    rows[788] = 32'h03FFFFC0;
    rows[789] = 32'h03FFFFC0;
    rows[790] = 32'h000003C0;
    rows[791] = 32'h000003C0;
    rows[792] = 32'h000007C0;
    rows[793] = 32'h000FFFC0;
    rows[794] = 32'h000FFF80;
    rows[795] = 32'h0007FF00;
    rows[796] = 32'h0007FC00;
    rows[797] = 32'h00000000;
    rows[798] = 32'h00000000;
    rows[799] = 32'h00000000;
    rows[800] = 32'h00000000;
    rows[801] = 32'h00000000;
    rows[802] = 32'h0180F300;
    rows[803] = 32'h03C0F780;
    rows[804] = 32'h03C0F7E0;
    rows[805] = 32'h03C0F1E0;
    rows[806] = 32'h0780F0C0;
    rows[807] = 32'h0F00F1F0;
    rows[808] = 32'h0F23FFF0;
    rows[809] = 32'h1E7FFFF8;
    rows[810] = 32'h3E7FFFE0;
    rows[811] = 32'h3FF7F000;
    rows[812] = 32'h1FF079F8;
    rows[813] = 32'h1FE7FFF8;
    rows[814] = 32'h03EFFFF8;
    rows[815] = 32'h03CFFFF0;
    rows[816] = 32'h078FF860;
    rows[817] = 32'h078078F0;
    rows[818] = 32'h0F807BF0;
    rows[819] = 32'h1FF03FE0;
    rows[820] = 32'h1FF03FC0;
    rows[821] = 32'h1FE03FB0;
    rows[822] = 32'h0C007E38;
    rows[823] = 32'h01F1FE38;
    rows[824] = 32'h3FFFFF78;
    rows[825] = 32'h3FFFEFF8;
    rows[826] = 32'h1FFF8FF0;
    rows[827] = 32'h1C0C07F0;
    rows[828] = 32'h000001E0;
    rows[829] = 32'h00000000;
    rows[830] = 32'h00000000;
    rows[831] = 32'h00000000;
    rows[832] = 32'h00000000;
    rows[833] = 32'h00000000;
    rows[834] = 32'h00007800;
    rows[835] = 32'h00007800;
    rows[836] = 32'h0FFCF800;
    rows[837] = 32'h0FFCFFF8;
    rows[838] = 32'h0FFDFFF8;
    rows[839] = 32'h0FFFFFF8;
    rows[840] = 32'h0E3FE0F0;
    rows[841] = 32'h0E3FE1F0;
    rows[842] = 32'h0E3FF3E0;
    rows[843] = 32'h0FFE7FC0;
    rows[844] = 32'h0FFC7F80;
    rows[845] = 32'h0FFC7F80;
    rows[846] = 32'h00E3FFF8;
    rows[847] = 32'h00E7FFF8;
    rows[848] = 32'h1CE7E1F8;
    rows[849] = 32'h1CFB0038;
    rows[850] = 32'h1CF80000;
    rows[851] = 32'h1CF9FFF0;
    rows[852] = 32'h1CE1FFF0;
    rows[853] = 32'h1CE1FFF0;
    rows[854] = 32'h1CE1E0F0;
    rows[855] = 32'h1DFDE0F0;
    rows[856] = 32'h1FFDE0F0;
    rows[857] = 32'h1FFDFFF0;
    rows[858] = 32'h1FF9FFF0;
    rows[859] = 32'h1C01FFF0;
    rows[860] = 32'h0001E0F0;
    rows[861] = 32'h0001E0F0;
    rows[862] = 32'h00000000;
    rows[863] = 32'h00000000;
    rows[864] = 32'h00000000;
    rows[865] = 32'h00000000;
    rows[866] = 32'h00000000;
    rows[867] = 32'h0FFFFFF8;
    rows[868] = 32'h0FFFFFF8;
    rows[869] = 32'h0FFFFFF8;
    rows[870] = 32'h0FFFFFF0;
    rows[871] = 32'h00780F00;
    rows[872] = 32'h00780F00;
    rows[873] = 32'h00780F00;
    rows[874] = 32'h00780F00;
    rows[875] = 32'h00780F00;
    rows[876] = 32'h00780F00;
    rows[877] = 32'h00780F00;
    rows[878] = 32'h3FFFFFFC;
    rows[879] = 32'h3FFFFFFC;
    rows[880] = 32'h3FFFFFFC;
    rows[881] = 32'h1FFFFFF8;
    rows[882] = 32'h00780F00;
    rows[883] = 32'h00F80F00;
    rows[884] = 32'h00F80F00;
    rows[885] = 32'h00F00F00;
    rows[886] = 32'h01F00F00;
    rows[887] = 32'h01F00F00;
    rows[888] = 32'h03E00F00;
    rows[889] = 32'h07E00F00;
    rows[890] = 32'h0FC00F00;
    rows[891] = 32'h1F800F00;
    rows[892] = 32'h0F000F00;
    rows[893] = 32'h06000F00;
    rows[894] = 32'h00000000;
    rows[895] = 32'h00000000;
    rows[896] = 32'h00000000;
    rows[897] = 32'h00000000;
    rows[898] = 32'h00C01800;
    rows[899] = 32'h00F07C00;
    rows[900] = 32'h01F03C00;
    rows[901] = 32'h03E03E00;
    rows[902] = 32'h07C01E00;
    rows[903] = 32'h1F8FFFF8;
    rows[904] = 32'h1F0FFFF8;
    rows[905] = 32'h1E4FFFF8;
    rows[906] = 32'h0CFFFFF8;
    rows[907] = 32'h08F03C00;
    rows[908] = 32'h01F03C00;
    rows[909] = 32'h03E03C00;
    rows[910] = 32'h07C03C00;
    rows[911] = 32'h0FC03C00;
    rows[912] = 32'h1FCFFFF0;
    rows[913] = 32'h1FCFFFF0;
    rows[914] = 32'h1FCFFFF0;
    rows[915] = 32'h1FC7FFF0;
    rows[916] = 32'h1BC03C00;
    rows[917] = 32'h13C03C00;
    rows[918] = 32'h03C03C00;
    rows[919] = 32'h03C03C00;
    rows[920] = 32'h03C03C00;
    rows[921] = 32'h03DFFFFC;
    rows[922] = 32'h03DFFFFC;
    rows[923] = 32'h03DFFFFC;
    rows[924] = 32'h03DFFFF8;
    rows[925] = 32'h03C00000;
    rows[926] = 32'h00000000;
    rows[927] = 32'h00000000;
    rows[928] = 32'h00000000;
    rows[929] = 32'h00000000;
    rows[930] = 32'h00000000;
    rows[931] = 32'h1FFFFFF8;
    rows[932] = 32'h1FFFFFF8;
    rows[933] = 32'h1FFFFFF8;
    rows[934] = 32'h007C1F00;
    rows[935] = 32'h08781E00;
    rows[936] = 32'h1CF01E00;
    rows[937] = 32'h1EF1FFF8;
    rows[938] = 32'h0FE1FFF8;
    rows[939] = 32'h07C1FFF8;
    rows[940] = 32'h3FFDE078;
    rows[941] = 32'h3FFDC078;
    rows[942] = 32'h3FFDCF78;
    rows[943] = 32'h07DDCF78;
    rows[944] = 32'h039DCE78;
    rows[945] = 32'h0399CE78;
    rows[946] = 32'h0399CE78;
    rows[947] = 32'h0399CE78;
    rows[948] = 32'h0399DE78;
    rows[949] = 32'h0381FE78;
    rows[950] = 32'h03803F00;
    rows[951] = 32'h03807FC0;
    rows[952] = 32'h0380FFE0;
    rows[953] = 32'h1F83F1F0;
    rows[954] = 32'h1F87E0F8;
    rows[955] = 32'h1F83C078;
    rows[956] = 32'h1F010030;
    rows[957] = 32'h00000000;
    rows[958] = 32'h00000000;
    rows[959] = 32'h00000000;
    rows[960] = 32'h00000000;
    rows[961] = 32'h00000000;
    rows[962] = 32'h03003E00;
    rows[963] = 32'h07003E00;
    rows[964] = 32'h0F803E00;
    rows[965] = 32'h07C03E00;
    rows[966] = 32'h03E03E00;
    rows[967] = 32'h03C03E00;
    rows[968] = 32'h01003E00;
    rows[969] = 32'h00003E00;
    rows[970] = 32'h001FFFFC;
    rows[971] = 32'h3F9FFFFC;
    rows[972] = 32'h3F9FFFFC;
    rows[973] = 32'h3F8FFFF8;
    rows[974] = 32'h1F803E00;
    rows[975] = 32'h07803E00;
    rows[976] = 32'h07803E00;
    rows[977] = 32'h07803E00;
    rows[978] = 32'h07803E00;
    rows[979] = 32'h07803E00;
    rows[980] = 32'h07883E00;
    rows[981] = 32'h07983E00;
    rows[982] = 32'h07F83E00;
    rows[983] = 32'h07F83E00;
    rows[984] = 32'h07F83E00;
    rows[985] = 32'h07E03E00;
    rows[986] = 32'h07C03E00;
    rows[987] = 32'h07803E00;
    rows[988] = 32'h02003E00;
    rows[989] = 32'h00003E00;
    rows[990] = 32'h00000000;
    rows[991] = 32'h00000000;
    rows[992] = 32'h00000000;
    rows[993] = 32'h00000000;
    rows[994] = 32'h000000F0;
    rows[995] = 32'h1FFFC0F0;
    rows[996] = 32'h1FFFC0F0;
    rows[997] = 32'h1FFFDEF0;
    rows[998] = 32'h03E01EF0;
    rows[999] = 32'h03C61EF0;
    rows[1000] = 32'h07CE1EF0;
    rows[1001] = 32'h078F1EF0;
    rows[1002] = 32'h0F0F9EF0;
    rows[1003] = 32'h1FFFDEF0;
    rows[1004] = 32'h1FFFDEF0;
    rows[1005] = 32'h1FFFDEF0;
    rows[1006] = 32'h0C011EF0;
    rows[1007] = 32'h00F01EF0;
    rows[1008] = 32'h00F01EF0;
    rows[1009] = 32'h00F01EF0;
    rows[1010] = 32'h1FFF9EF0;
    rows[1011] = 32'h1FFF9EF0;
    rows[1012] = 32'h1FFF9EF0;
    rows[1013] = 32'h00F81EF0;
    rows[1014] = 32'h00F01EF0;
    rows[1015] = 32'h00F000F0;
    rows[1016] = 32'h00FFC0F0;
    rows[1017] = 32'h3FFFC0F0;
    rows[1018] = 32'h1FFFDFF0;
    rows[1019] = 32'h1FFF8FF0;
    rows[1020] = 32'h1C000FE0;
    rows[1021] = 32'h00000FC0;
    rows[1022] = 32'h00000000;
    rows[1023] = 32'h00000000;
    rows[1024] = 32'h00000000;
    rows[1025] = 32'h00000000;
    rows[1026] = 32'h06007800;
    rows[1027] = 32'h0E007800;
    rows[1028] = 32'h1F007800;
    rows[1029] = 32'h0F807800;
    rows[1030] = 32'h07807800;
    rows[1031] = 32'h07C07800;
    rows[1032] = 32'h03BFFFF8;
    rows[1033] = 32'h023FFFF8;
    rows[1034] = 32'h003FFFF8;
    rows[1035] = 32'h0000F800;
    rows[1036] = 32'h1F80F000;
    rows[1037] = 32'h1F80F000;
    rows[1038] = 32'h1F80F800;
    rows[1039] = 32'h1F81FC00;
    rows[1040] = 32'h0781FF00;
    rows[1041] = 32'h0783FF80;
    rows[1042] = 32'h0787CFC0;
    rows[1043] = 32'h078FC7E0;
    rows[1044] = 32'h079F83F0;
    rows[1045] = 32'h07BF01F0;
    rows[1046] = 32'h079E00E0;
    rows[1047] = 32'h0FCC0040;
    rows[1048] = 32'h1FF80000;
    rows[1049] = 32'h1FFFFFF8;
    rows[1050] = 32'h1C7FFFF8;
    rows[1051] = 32'h183FFFF8;
    rows[1052] = 32'h0803FFF0;
    rows[1053] = 32'h00000000;
    rows[1054] = 32'h00000000;
    rows[1055] = 32'h00000000;
    rows[1056] = 32'h00000000;
    rows[1057] = 32'h00000000;
    rows[1058] = 32'h00F803C0;
    rows[1059] = 32'h1FFCC3C0;
    rows[1060] = 32'h1FFDE3C0;
    rows[1061] = 32'h1FF1F3C0;
    rows[1062] = 32'h03C0FFC0;
    rows[1063] = 32'h03C07BC0;
    rows[1064] = 32'h03C033C0;
    rows[1065] = 32'h1FFC03C0;
    rows[1066] = 32'h1FFC03C0;
    rows[1067] = 32'h1FFD83C0;
    rows[1068] = 32'h1FFBC3C0;
    rows[1069] = 32'h03C3E3C0;
    rows[1070] = 32'h03D1F3C0;
    rows[1071] = 32'h07F8F3C0;
    rows[1072] = 32'h07FC23C0;
    rows[1073] = 32'h0FDE03C0;
    rows[1074] = 32'h0FDC07FC;
    rows[1075] = 32'h1FCFFFFC;
    rows[1076] = 32'h3FC7FFFC;
    rows[1077] = 32'h1FC7FFF0;
    rows[1078] = 32'h1BC7C3C0;
    rows[1079] = 32'h1BC003C0;
    rows[1080] = 32'h03C003C0;
    rows[1081] = 32'h03C003C0;
    rows[1082] = 32'h03C003C0;
    rows[1083] = 32'h03C003C0;
    rows[1084] = 32'h03C003C0;
    rows[1085] = 32'h00000000;
    rows[1086] = 32'h00000000;
    rows[1087] = 32'h00000000;
    rows[1088] = 32'h00000000;
    rows[1089] = 32'h00000000;
    rows[1090] = 32'h03803C00;
    rows[1091] = 32'h03803C00;
    rows[1092] = 32'h03803C00;
    rows[1093] = 32'h03803C00;
    rows[1094] = 32'h038FFFF8;
    rows[1095] = 32'h1FFFFFF8;
    rows[1096] = 32'h1FFFFFF8;
    rows[1097] = 32'h1FF03E00;
    rows[1098] = 32'h1FE03C00;
    rows[1099] = 32'h03803C00;
    rows[1100] = 32'h038FFFF0;
    rows[1101] = 32'h038FFFF0;
    rows[1102] = 32'h03FFFFF0;
    rows[1103] = 32'h07F001F0;
    rows[1104] = 32'h3FF001F0;
    rows[1105] = 32'h1FE381E0;
    rows[1106] = 32'h1F87C3E0;
    rows[1107] = 32'h1383E3C0;
    rows[1108] = 32'h0381F7C0;
    rows[1109] = 32'h0381FF80;
    rows[1110] = 32'h0380FF00;
    rows[1111] = 32'h03807F00;
    rows[1112] = 32'h0381FFC0;
    rows[1113] = 32'h0787FFF0;
    rows[1114] = 32'h1F9FE7F8;
    rows[1115] = 32'h1F9FC1F8;
    rows[1116] = 32'h1F8F00F0;
    rows[1117] = 32'h0C0C0030;
    rows[1118] = 32'h00000000;
    rows[1119] = 32'h00000000;
    rows[1120] = 32'h00000000;
    rows[1121] = 32'h00000000;
    rows[1122] = 32'h00000000;
    rows[1123] = 32'h1FFFFFF8;
    rows[1124] = 32'h1FFFFFF8;
    rows[1125] = 32'h1FFFFFF8;
    rows[1126] = 32'h1E000078;
    rows[1127] = 32'h1E000078;
    rows[1128] = 32'h1E7FFE78;
    rows[1129] = 32'h1E7FFE78;
    rows[1130] = 32'h1E7FFE78;
    rows[1131] = 32'h1E000078;
    rows[1132] = 32'h1E000078;
    rows[1133] = 32'h1EFFFF78;
    rows[1134] = 32'h1EFFFF78;
    rows[1135] = 32'h1EFFFF78;
    rows[1136] = 32'h1E1CF378;
    rows[1137] = 32'h1E1CF3F8;
    rows[1138] = 32'h1E3CF3F8;
    rows[1139] = 32'h1E3CF778;
    rows[1140] = 32'h1E78FF78;
    rows[1141] = 32'h1FF07F78;
    rows[1142] = 32'h1EF07E78;
    rows[1143] = 32'h1E400078;
    rows[1144] = 32'h1FFFFFF8;
    rows[1145] = 32'h1FFFFFF8;
    rows[1146] = 32'h1FFFFFF8;
    rows[1147] = 32'h1E000078;
    rows[1148] = 32'h00000000;
    rows[1149] = 32'h00000000;
    rows[1150] = 32'h00000000;
    rows[1151] = 32'h00000000;
    rows[1152] = 32'h00000000;
    rows[1153] = 32'h00000000;
    rows[1154] = 32'h00000000;
    rows[1155] = 32'h1FFFFFF8;
    rows[1156] = 32'h1FFFFFF8;
    rows[1157] = 32'h1FFFFFF8;
    rows[1158] = 32'h1FFFFFF8;
    rows[1159] = 32'h00078000;
    rows[1160] = 32'h00078000;
    rows[1161] = 32'h00078000;
    rows[1162] = 32'h00079000;
    rows[1163] = 32'h0007BC00;
    rows[1164] = 32'h0007FE00;
    rows[1165] = 32'h0007FF00;
    rows[1166] = 32'h00079FC0;
    rows[1167] = 32'h00078FE0;
    rows[1168] = 32'h000787F0;
    rows[1169] = 32'h000781F0;
    rows[1170] = 32'h000780E0;
    rows[1171] = 32'h00078040;
    rows[1172] = 32'h00078000;
    rows[1173] = 32'h00078000;
    rows[1174] = 32'h00078000;
    rows[1175] = 32'h00078000;
    rows[1176] = 32'h00078000;
    rows[1177] = 32'h00078000;
    rows[1178] = 32'h00078000;
    rows[1179] = 32'h00078000;
    rows[1180] = 32'h00078000;
    rows[1181] = 32'h00000000;
    rows[1182] = 32'h00000000;
    rows[1183] = 32'h00000000;
    rows[1184] = 32'h00000000;
    rows[1185] = 32'h00000000;
    rows[1186] = 32'h00000000;
    rows[1187] = 32'h00000000;
    rows[1188] = 32'h00000000;
    rows[1189] = 32'h00000000;
    rows[1190] = 32'h00000000;
    rows[1191] = 32'h00000000;
    rows[1192] = 32'h00000000;
    rows[1193] = 32'h00000000;
    rows[1194] = 32'h00000000;
    rows[1195] = 32'h00000000;
    rows[1196] = 32'h00000000;
    rows[1197] = 32'h1FFFFFF8;
    rows[1198] = 32'h1FFFFFF8;
    rows[1199] = 32'h1FFFFFF8;
    rows[1200] = 32'h1FFFFFF8;
    rows[1201] = 32'h00000000;
    rows[1202] = 32'h00000000;
    rows[1203] = 32'h00000000;
    rows[1204] = 32'h00000000;
    rows[1205] = 32'h00000000;
    rows[1206] = 32'h00000000;
    rows[1207] = 32'h00000000;
    rows[1208] = 32'h00000000;
    rows[1209] = 32'h00000000;
    rows[1210] = 32'h00000000;
    rows[1211] = 32'h00000000;
    rows[1212] = 32'h00000000;
    rows[1213] = 32'h00000000;
    rows[1214] = 32'h00000000;
    rows[1215] = 32'h00000000;
    rows[1216] = 32'h00000000;
    rows[1217] = 32'h00000000;
    rows[1218] = 32'h0000E000;
    rows[1219] = 32'h1FF0FFF8;
    rows[1220] = 32'h1FF0FFF8;
    rows[1221] = 32'h1FF0FFF8;
    rows[1222] = 32'h0780E3E0;
    rows[1223] = 32'h038EE3C0;
    rows[1224] = 32'h038EE3C0;
    rows[1225] = 32'h038EE3C0;
    rows[1226] = 32'h038EE3C0;
    rows[1227] = 32'h038EE3C0;
    rows[1228] = 32'h038EE3C0;
    rows[1229] = 32'h1FFEE3C0;
    rows[1230] = 32'h1FFEEFF8;
    rows[1231] = 32'h1FFEEFF8;
    rows[1232] = 32'h078EEFF8;
    rows[1233] = 32'h038EE3C0;
    rows[1234] = 32'h038EE3C0;
    rows[1235] = 32'h0380E3C0;
    rows[1236] = 32'h0381E3C0;
    rows[1237] = 32'h0381E3C0;
    rows[1238] = 32'h0381C3C0;
    rows[1239] = 32'h03F3C3C0;
    rows[1240] = 32'h1FF3C3C0;
    rows[1241] = 32'h1FF79FFC;
    rows[1242] = 32'h1FEF9FFC;
    rows[1243] = 32'h1C0F1FFC;
    rows[1244] = 32'h000F0000;
    rows[1245] = 32'h00020000;
    rows[1246] = 32'h00000000;
    rows[1247] = 32'h00000000;
    rows[1248] = 32'h00000000;
    rows[1249] = 32'h00000000;
    rows[1250] = 32'h00000000;
    rows[1251] = 32'h00181800;
    rows[1252] = 32'h001E3C00;
    rows[1253] = 32'h003E3C00;
    rows[1254] = 32'h007C3E00;
    rows[1255] = 32'h00F81F00;
    rows[1256] = 32'h00F80F80;
    rows[1257] = 32'h01F007C0;
    rows[1258] = 32'h03E007E0;
    rows[1259] = 32'h0FC003F0;
    rows[1260] = 32'h1F8001F8;
    rows[1261] = 32'h1FFFFFF8;
    rows[1262] = 32'h1FFFFFF0;
    rows[1263] = 32'h0BFFFFB0;
    rows[1264] = 32'h03FFFF80;
    rows[1265] = 32'h003C0780;
    rows[1266] = 32'h003C0780;
    rows[1267] = 32'h003C0780;
    rows[1268] = 32'h007C0780;
    rows[1269] = 32'h00780780;
    rows[1270] = 32'h00F80780;
    rows[1271] = 32'h01F80F80;
    rows[1272] = 32'h07F3FF80;
    rows[1273] = 32'h1FE3FF00;
    rows[1274] = 32'h1FC3FF00;
    rows[1275] = 32'h0F81FC00;
    rows[1276] = 32'h06000000;
    rows[1277] = 32'h00000000;
    rows[1278] = 32'h00000000;
    rows[1279] = 32'h00000000;
    rows[1280] = 32'h00000000;
    rows[1281] = 32'h00000000;
    rows[1282] = 32'h03801E00;
    rows[1283] = 32'h07801E00;
    rows[1284] = 32'h07801E00;
    rows[1285] = 32'h07F81E00;
    rows[1286] = 32'h0FF81E00;
    rows[1287] = 32'h0FFFFFF8;
    rows[1288] = 32'h1E07FFF8;
    rows[1289] = 32'h1E07FFF8;
    rows[1290] = 32'h3FF79E78;
    rows[1291] = 32'h1FF79E78;
    rows[1292] = 32'h1FF79E78;
    rows[1293] = 32'h1BC79E78;
    rows[1294] = 32'h13879E78;
    rows[1295] = 32'h3FF79E78;
    rows[1296] = 32'h3FF79E78;
    rows[1297] = 32'h3FF7FFF8;
    rows[1298] = 32'h03C7FFF8;
    rows[1299] = 32'h0387FFF8;
    rows[1300] = 32'h03879E78;
    rows[1301] = 32'h03879E78;
    rows[1302] = 32'h03981E00;
    rows[1303] = 32'h03F81E00;
    rows[1304] = 32'h07F81E00;
    rows[1305] = 32'h07F81E00;
    rows[1306] = 32'h07E01E00;
    rows[1307] = 32'h03801E00;
    rows[1308] = 32'h03001E00;
    rows[1309] = 32'h00000000;
    rows[1310] = 32'h00000000;
    rows[1311] = 32'h00000000;
    rows[1312] = 32'h00000000;
    rows[1313] = 32'h00000000;
    rows[1314] = 32'h0003C000;
    rows[1315] = 32'h0003C000;
    rows[1316] = 32'h0103C080;
    rows[1317] = 32'h0383C1E0;
    rows[1318] = 32'h07C3C3F0;
    rows[1319] = 32'h03F3C7E0;
    rows[1320] = 32'h01F3CF80;
    rows[1321] = 32'h00E3CF00;
    rows[1322] = 32'h0043C200;
    rows[1323] = 32'h0003C000;
    rows[1324] = 32'h0FFFFFF0;
    rows[1325] = 32'h0FFFFFF0;
    rows[1326] = 32'h0FFFFFF0;
    rows[1327] = 32'h000001F0;
    rows[1328] = 32'h000000F0;
    rows[1329] = 32'h000000F0;
    rows[1330] = 32'h07FFFFF0;
    rows[1331] = 32'h07FFFFF0;
    rows[1332] = 32'h07FFFFF0;
    rows[1333] = 32'h000001F0;
    rows[1334] = 32'h000000F0;
    rows[1335] = 32'h000000F0;
    rows[1336] = 32'h1FFFFFF0;
    rows[1337] = 32'h1FFFFFF0;
    rows[1338] = 32'h1FFFFFF0;
    rows[1339] = 32'h000001F0;
    rows[1340] = 32'h000000F0;
    rows[1341] = 32'h00000000;
    rows[1342] = 32'h00000000;
    rows[1343] = 32'h00000000;
    rows[1344] = 32'h00000000;
    rows[1345] = 32'h00000000;
    rows[1346] = 32'h00600E00;
    rows[1347] = 32'h01F00F80;
    rows[1348] = 32'h00F81F00;
    rows[1349] = 32'h3FFFFFFC;
    rows[1350] = 32'h3FFFFFFC;
    rows[1351] = 32'h3FFFFFFC;
    rows[1352] = 32'h00000000;
    rows[1353] = 32'h0FFF80F0;
    rows[1354] = 32'h0FFF9CF0;
    rows[1355] = 32'h0FFF9CF0;
    rows[1356] = 32'h0F079CF0;
    rows[1357] = 32'h0F079CF0;
    rows[1358] = 32'h0FFF9CF0;
    rows[1359] = 32'h0FFF9CF0;
    rows[1360] = 32'h0FFF9CF0;
    rows[1361] = 32'h0F079CF0;
    rows[1362] = 32'h0F079CF0;
    rows[1363] = 32'h0F079CF0;
    rows[1364] = 32'h0FFF9CF0;
    rows[1365] = 32'h0FFF9CF0;
    rows[1366] = 32'h0FFF9CF0;
    rows[1367] = 32'h0F079CF0;
    rows[1368] = 32'h0F0780F0;
    rows[1369] = 32'h0F0780F0;
    rows[1370] = 32'h0F3F9FF0;
    rows[1371] = 32'h0F3F9FF0;
    rows[1372] = 32'h0F3F1FE0;
    rows[1373] = 32'h0E180F80;
    rows[1374] = 32'h00000000;
    rows[1375] = 32'h00000000;
    rows[1376] = 32'h00000000;
    rows[1377] = 32'h00000000;
    rows[1378] = 32'h00C01E00;
    rows[1379] = 32'h03C01E00;
    rows[1380] = 32'h03E01E00;
    rows[1381] = 32'h01E01E00;
    rows[1382] = 32'h01E01E00;
    rows[1383] = 32'h1FFF1FFC;
    rows[1384] = 32'h1FFF1FFC;
    rows[1385] = 32'h1FFF1FFC;
    rows[1386] = 32'h1FFE1FF8;
    rows[1387] = 32'h001C1E00;
    rows[1388] = 32'h0E3E1E00;
    rows[1389] = 32'h1E3C1E00;
    rows[1390] = 32'h0E3C1E00;
    rows[1391] = 32'h0F3DFFF8;
    rows[1392] = 32'h0F3DFFF8;
    rows[1393] = 32'h0F3DFFF8;
    rows[1394] = 32'h0F39FFF8;
    rows[1395] = 32'h0779E078;
    rows[1396] = 32'h07F9E078;
    rows[1397] = 32'h0779E078;
    rows[1398] = 32'h007FE078;
    rows[1399] = 32'h07FFE078;
    rows[1400] = 32'h3FFDFFF8;
    rows[1401] = 32'h1FFDFFF8;
    rows[1402] = 32'h1FF9FFF8;
    rows[1403] = 32'h1E01FFF8;
    rows[1404] = 32'h0001E078;
    rows[1405] = 32'h00000000;
    rows[1406] = 32'h00000000;
    rows[1407] = 32'h00000000;
    rows[1408] = 32'h00000000;
    rows[1409] = 32'h00000000;
    rows[1410] = 32'h00000000;
    rows[1411] = 32'h00000000;
    rows[1412] = 32'h00000000;
    rows[1413] = 32'h00000000;
    rows[1414] = 32'h00000000;
    rows[1415] = 32'h00000000;
    rows[1416] = 32'h00000000;
    rows[1417] = 32'h00000000;
    rows[1418] = 32'h01C00000;
    rows[1419] = 32'h03E00000;
    rows[1420] = 32'h03E00000;
    rows[1421] = 32'h03E00000;
    rows[1422] = 32'h01C00000;
    rows[1423] = 32'h00000000;
    rows[1424] = 32'h00000000;
    rows[1425] = 32'h00000000;
    rows[1426] = 32'h00000000;
    rows[1427] = 32'h00000000;
    rows[1428] = 32'h01C00000;
    rows[1429] = 32'h03E00000;
    rows[1430] = 32'h03E00000;
    rows[1431] = 32'h03E00000;
    rows[1432] = 32'h01C00000;
    rows[1433] = 32'h00000000;
    rows[1434] = 32'h00000000;
    rows[1435] = 32'h00000000;
    rows[1436] = 32'h00000000;
    rows[1437] = 32'h00000000;
    rows[1438] = 32'h00000000;
    rows[1439] = 32'h00000000;
    rows[1440] = 32'h00000000;
    rows[1441] = 32'h00000000;
    rows[1442] = 32'h0003C000;
    rows[1443] = 32'h0003C000;
    rows[1444] = 32'h0003C000;
    rows[1445] = 32'h0003C000;
    rows[1446] = 32'h0003C000;
    rows[1447] = 32'h0FFFFFF0;
    rows[1448] = 32'h0FFFFFF0;
    rows[1449] = 32'h0FFFFFF0;
    rows[1450] = 32'h0FFFFFF0;
    rows[1451] = 32'h0F03C0F0;
    rows[1452] = 32'h0F03C0F0;
    rows[1453] = 32'h0F03C0F0;
    rows[1454] = 32'h0F03C0F0;
    rows[1455] = 32'h0F03C0F0;
    rows[1456] = 32'h0FFFFFF0;
    rows[1457] = 32'h0FFFFFF0;
    rows[1458] = 32'h0FFFFFF0;
    rows[1459] = 32'h0FFFFFF0;
    rows[1460] = 32'h0F03C0F0;
    rows[1461] = 32'h0F03C0F0;
    rows[1462] = 32'h0003C000;
    rows[1463] = 32'h0003C000;
    rows[1464] = 32'h0003C000;
    rows[1465] = 32'h0003C000;
    rows[1466] = 32'h0003C000;
    rows[1467] = 32'h0003C000;
    rows[1468] = 32'h0003C000;
    rows[1469] = 32'h00000000;
    rows[1470] = 32'h00000000;
    rows[1471] = 32'h00000000;
    rows[1472] = 32'h00000000;
    rows[1473] = 32'h00000000;
    rows[1474] = 32'h00010000;
    rows[1475] = 32'h00078000;
    rows[1476] = 32'h0007C000;
    rows[1477] = 32'h0003E000;
    rows[1478] = 32'h0003E000;
    rows[1479] = 32'h0001F000;
    rows[1480] = 32'h003CF800;
    rows[1481] = 32'h003CF000;
    rows[1482] = 32'h073C4000;
    rows[1483] = 32'h07BC00C0;
    rows[1484] = 32'h07BC03E0;
    rows[1485] = 32'h07BC01E0;
    rows[1486] = 32'h0F3C01F0;
    rows[1487] = 32'h0F3C00F0;
    rows[1488] = 32'h0F3C00F0;
    rows[1489] = 32'h0F3C00F8;
    rows[1490] = 32'h1E3C0078;
    rows[1491] = 32'h1E3C0478;
    rows[1492] = 32'h1E3C07A0;
    rows[1493] = 32'h1E3C0F80;
    rows[1494] = 32'h043C0F80;
    rows[1495] = 32'h003C0F00;
    rows[1496] = 32'h003E1F00;
    rows[1497] = 32'h003FFF00;
    rows[1498] = 32'h003FFF00;
    rows[1499] = 32'h001FFE00;
    rows[1500] = 32'h0007F000;
    rows[1501] = 32'h00000000;
    rows[1502] = 32'h00000000;
    rows[1503] = 32'h00000000;
    rows[1504] = 32'h00000000;
    rows[1505] = 32'h00000000;
    rows[1506] = 32'h01C01E00;
    rows[1507] = 32'h01C01E00;
    rows[1508] = 32'h01C01FFC;
    rows[1509] = 32'h01C01FFC;
    rows[1510] = 32'h1FFC1FFC;
    rows[1511] = 32'h1FFC1E00;
    rows[1512] = 32'h1FFC1E00;
    rows[1513] = 32'h1DFDFFF0;
    rows[1514] = 32'h1DDDFFF0;
    rows[1515] = 32'h1DDDFFF0;
    rows[1516] = 32'h1DDDE0F0;
    rows[1517] = 32'h1DDDCE70;
    rows[1518] = 32'h1DDDCE70;
    rows[1519] = 32'h1DDDCE70;
    rows[1520] = 32'h1DDDCE70;
    rows[1521] = 32'h1DDDCE70;
    rows[1522] = 32'h1DDDCE70;
    rows[1523] = 32'h1DDDDE70;
    rows[1524] = 32'h1DFDDE70;
    rows[1525] = 32'h1DFDDE70;
    rows[1526] = 32'h1DF9FFF0;
    rows[1527] = 32'h01C03FC0;
    rows[1528] = 32'h01C07FE0;
    rows[1529] = 32'h01C1F9F8;
    rows[1530] = 32'h01C7F0F8;
    rows[1531] = 32'h01C3E078;
    rows[1532] = 32'h01C18010;
    rows[1533] = 32'h00000000;
    rows[1534] = 32'h00000000;
    rows[1535] = 32'h00000000;
    rows[1536] = 32'h00000000;
    rows[1537] = 32'h00000000;
    rows[1538] = 32'h0007C000;
    rows[1539] = 32'h0003C000;
    rows[1540] = 32'h1FFFFFF8;
    rows[1541] = 32'h1FFFFFF8;
    rows[1542] = 32'h1FFFFFF8;
    rows[1543] = 32'h060F9800;
    rows[1544] = 32'h070F3C40;
    rows[1545] = 32'h0F9E78E0;
    rows[1546] = 32'h07BFF1F0;
    rows[1547] = 32'h033FE3C0;
    rows[1548] = 32'h00BFC180;
    rows[1549] = 32'h03C79180;
    rows[1550] = 32'h07CF3BC0;
    rows[1551] = 32'h1FBE3FF0;
    rows[1552] = 32'h1F7FFCF8;
    rows[1553] = 32'h0C3FFE70;
    rows[1554] = 32'h083FEE20;
    rows[1555] = 32'h00000C00;
    rows[1556] = 32'h0003C000;
    rows[1557] = 32'h3FFFFFFC;
    rows[1558] = 32'h3FFFFFFC;
    rows[1559] = 32'h3FFFFFFC;
    rows[1560] = 32'h0003C000;
    rows[1561] = 32'h0003C000;
    rows[1562] = 32'h0003C000;
    rows[1563] = 32'h0003C000;
    rows[1564] = 32'h0003C000;
    rows[1565] = 32'h00000000;
    rows[1566] = 32'h00000000;
    rows[1567] = 32'h00000000;
    rows[1568] = 32'h00000000;
    rows[1569] = 32'h00000000;
    rows[1570] = 32'h00380000;
    rows[1571] = 32'h00780000;
    rows[1572] = 32'h00F80000;
    rows[1573] = 32'h00FFFFE0;
    rows[1574] = 32'h01FFFFE0;
    rows[1575] = 32'h03FFFFE0;
    rows[1576] = 32'h07F007C0;
    rows[1577] = 32'h1FF80F80;
    rows[1578] = 32'h1FFE3F00;
    rows[1579] = 32'h0E3FFE00;
    rows[1580] = 32'h0C1FF800;
    rows[1581] = 32'h00FFFF00;
    rows[1582] = 32'h1FFFFFF8;
    rows[1583] = 32'h1FFE7FF8;
    rows[1584] = 32'h1FE00FF8;
    rows[1585] = 32'h08000070;
    rows[1586] = 32'h03FFFFC0;
    rows[1587] = 32'h03FFFFC0;
    rows[1588] = 32'h03FFFFC0;
    rows[1589] = 32'h03C003C0;
    rows[1590] = 32'h03C003C0;
    rows[1591] = 32'h03C003C0;
    rows[1592] = 32'h03C003C0;
    rows[1593] = 32'h03FFFFC0;
    rows[1594] = 32'h03FFFFC0;
    rows[1595] = 32'h03FFFFC0;
    rows[1596] = 32'h03C003C0;
    rows[1597] = 32'h00000000;
    rows[1598] = 32'h00000000;
    rows[1599] = 32'h00000000;
    rows[1600] = 32'h00000000;
    rows[1601] = 32'h00000000;
    rows[1602] = 32'h00C01800;
    rows[1603] = 32'h01E07C00;
    rows[1604] = 32'h01E07C00;
    rows[1605] = 32'h01E03C00;
    rows[1606] = 32'h03DFFFF8;
    rows[1607] = 32'h03DFFFF8;
    rows[1608] = 32'h039FFFF8;
    rows[1609] = 32'h079FFFF0;
    rows[1610] = 32'h07800000;
    rows[1611] = 32'h0F8301C0;
    rows[1612] = 32'h0F8F01F0;
    rows[1613] = 32'h1F8F01E0;
    rows[1614] = 32'h1F8781E0;
    rows[1615] = 32'h3F8783E0;
    rows[1616] = 32'h1F8783C0;
    rows[1617] = 32'h1F83C3C0;
    rows[1618] = 32'h1B83C3C0;
    rows[1619] = 32'h0383C780;
    rows[1620] = 32'h0383C780;
    rows[1621] = 32'h0381E780;
    rows[1622] = 32'h0381EF00;
    rows[1623] = 32'h03818F00;
    rows[1624] = 32'h03800F00;
    rows[1625] = 32'h03BFFFFC;
    rows[1626] = 32'h03BFFFFC;
    rows[1627] = 32'h03BFFFFC;
    rows[1628] = 32'h039FFFF8;
    rows[1629] = 32'h03800000;
    rows[1630] = 32'h00000000;
    rows[1631] = 32'h00000000;
    rows[1632] = 32'h00000000;
    rows[1633] = 32'h00000000;
    rows[1634] = 32'h00001FE0;
    rows[1635] = 32'h0FFFFFF0;
    rows[1636] = 32'h0FFFFFF0;
    rows[1637] = 32'h07FFFF80;
    rows[1638] = 32'h0003C000;
    rows[1639] = 32'h0003C000;
    rows[1640] = 32'h1FFFFFF8;
    rows[1641] = 32'h1FFFFFF8;
    rows[1642] = 32'h1FFFFFF8;
    rows[1643] = 32'h00F3CF30;
    rows[1644] = 32'h1FF3CFF8;
    rows[1645] = 32'h1FF3CFF8;
    rows[1646] = 32'h1FF3CFC0;
    rows[1647] = 32'h00F3CF18;
    rows[1648] = 32'h00F7CF3C;
    rows[1649] = 32'h0FFFEFF8;
    rows[1650] = 32'h1FFFF7F8;
    rows[1651] = 32'h1FFFFBF0;
    rows[1652] = 32'h1C7BFC00;
    rows[1653] = 32'h00FBDE00;
    rows[1654] = 32'h03F3DF00;
    rows[1655] = 32'h07E3CFE0;
    rows[1656] = 32'h1FC3C3F8;
    rows[1657] = 32'h1F03C1F8;
    rows[1658] = 32'h1E03C078;
    rows[1659] = 32'h0803C010;
    rows[1660] = 32'h0003C000;
    rows[1661] = 32'h00000000;
    rows[1662] = 32'h00000000;
    rows[1663] = 32'h00000000;
    rows[1664] = 32'h00000000;
    rows[1665] = 32'h00000000;
    rows[1666] = 32'h0003C000;
    rows[1667] = 32'h0003E000;
    rows[1668] = 32'h1FFFFFF8;
    rows[1669] = 32'h1FFFFFF8;
    rows[1670] = 32'h1FFFFFF8;
    rows[1671] = 32'h1E1E0078;
    rows[1672] = 32'h1E3E0078;
    rows[1673] = 32'h1E7FFFF0;
    rows[1674] = 32'h01FFFF80;
    rows[1675] = 32'h03FFFF80;
    rows[1676] = 32'h0FF81F00;
    rows[1677] = 32'h0F3C3E00;
    rows[1678] = 32'h061FFC00;
    rows[1679] = 32'h000FF800;
    rows[1680] = 32'h007FFE00;
    rows[1681] = 32'h0FFFFFE0;
    rows[1682] = 32'h1FFC1FF8;
    rows[1683] = 32'h1FC003F8;
    rows[1684] = 32'h1BFFFFF8;
    rows[1685] = 32'h03FFFFC0;
    rows[1686] = 32'h03FFFFC0;
    rows[1687] = 32'h03C003C0;
    rows[1688] = 32'h03C003C0;
    rows[1689] = 32'h03FFFFC0;
    rows[1690] = 32'h03FFFFC0;
    rows[1691] = 32'h03FFFFC0;
    rows[1692] = 32'h03C003C0;
    rows[1693] = 32'h00000000;
    rows[1694] = 32'h00000000;
    rows[1695] = 32'h00000000;
    rows[1696] = 32'h00000000;
    rows[1697] = 32'h00000000;
    rows[1698] = 32'h00000000;
    rows[1699] = 32'h00000000;
    rows[1700] = 32'h00000000;
    rows[1701] = 32'h00000000;
    rows[1702] = 32'h00000000;
    rows[1703] = 32'h00000000;
    rows[1704] = 32'h00000000;
    rows[1705] = 32'h00000000;
    rows[1706] = 32'h00000000;
    rows[1707] = 32'h00000000;
    rows[1708] = 32'h00000000;
    rows[1709] = 32'h00000000;
    rows[1710] = 32'h00000000;
    rows[1711] = 32'h00000000;
    rows[1712] = 32'h00000000;
    rows[1713] = 32'h00000000;
    rows[1714] = 32'h00000000;
    rows[1715] = 32'h00000000;
    rows[1716] = 32'h00000000;
    rows[1717] = 32'h00000000;
    rows[1718] = 32'h01E00000;
    rows[1719] = 32'h01E00000;
    rows[1720] = 32'h01C00000;
    rows[1721] = 32'h03C00000;
    rows[1722] = 32'h03C00000;
    rows[1723] = 32'h03800000;
    rows[1724] = 32'h03800000;
    rows[1725] = 32'h00000000;
    rows[1726] = 32'h00000000;
    rows[1727] = 32'h00000000;
    rows[1728] = 32'h00000000;
    rows[1729] = 32'h00000000;
    rows[1730] = 32'h000000F0;
    rows[1731] = 32'h1FFFC0F0;
    rows[1732] = 32'h1FFFC0F0;
    rows[1733] = 32'h1FFFFCF0;
    rows[1734] = 32'h1FFFBCF0;
    rows[1735] = 32'h01E03CF0;
    rows[1736] = 32'h01C03CF0;
    rows[1737] = 32'h03FFBCF0;
    rows[1738] = 32'h03FFBCF0;
    rows[1739] = 32'h07FFBCF0;
    rows[1740] = 32'h07FFBCF0;
    rows[1741] = 32'h0F07BCF0;
    rows[1742] = 32'h0E0F3CF0;
    rows[1743] = 32'h1E0F3CF0;
    rows[1744] = 32'h3CCF3CF0;
    rows[1745] = 32'h1DFE3CF0;
    rows[1746] = 32'h19FE3CF0;
    rows[1747] = 32'h087C3CF0;
    rows[1748] = 32'h007C3CF0;
    rows[1749] = 32'h00783CF0;
    rows[1750] = 32'h00F83CF0;
    rows[1751] = 32'h01F000F0;
    rows[1752] = 32'h07E000F0;
    rows[1753] = 32'h0FC000F0;
    rows[1754] = 32'h0F803FF0;
    rows[1755] = 32'h07001FF0;
    rows[1756] = 32'h02001FE0;
    rows[1757] = 32'h00001FC0;
    rows[1758] = 32'h00000000;
    rows[1759] = 32'h00000000;
    rows[1760] = 32'h00000000;
    rows[1761] = 32'h00000000;
    rows[1762] = 32'h001E0000;
    rows[1763] = 32'h001E0000;
    rows[1764] = 32'h003E0000;
    rows[1765] = 32'h1FFFFFF0;
    rows[1766] = 32'h1FFFFFF0;
    rows[1767] = 32'h1FFFFFF0;
    rows[1768] = 32'h00F80000;
    rows[1769] = 32'h00F00000;
    rows[1770] = 32'h01F00000;
    rows[1771] = 32'h01E1E000;
    rows[1772] = 32'h03C1E000;
    rows[1773] = 32'h03C1E000;
    rows[1774] = 32'h07FFFFE0;
    rows[1775] = 32'h07FFFFE0;
    rows[1776] = 32'h07FFFFE0;
    rows[1777] = 32'h0001E000;
    rows[1778] = 32'h0001E000;
    rows[1779] = 32'h0001E000;
    rows[1780] = 32'h1FFFFFF8;
    rows[1781] = 32'h1FFFFFF8;
    rows[1782] = 32'h1FFFFFF8;
    rows[1783] = 32'h0001E000;
    rows[1784] = 32'h0001E000;
    rows[1785] = 32'h0001E000;
    rows[1786] = 32'h0001E000;
    rows[1787] = 32'h0001E000;
    rows[1788] = 32'h0001E000;
    rows[1789] = 32'h0001E000;
    rows[1790] = 32'h00000000;
    rows[1791] = 32'h00000000;
    rows[1792] = 32'h00000000;
    rows[1793] = 32'h00000000;
    rows[1794] = 32'h00000000;
    rows[1795] = 32'h0FFF7FF0;
    rows[1796] = 32'h0FFF7FF0;
    rows[1797] = 32'h0FFF7FF0;
    rows[1798] = 32'h0F0F7FF0;
    rows[1799] = 32'h0F0F7870;
    rows[1800] = 32'h0F0F7870;
    rows[1801] = 32'h0FFF7870;
    rows[1802] = 32'h0FFF7870;
    rows[1803] = 32'h0FFF7870;
    rows[1804] = 32'h0F0F7870;
    rows[1805] = 32'h0F0F7870;
    rows[1806] = 32'h0FFF7870;
    rows[1807] = 32'h0FFF7870;
    rows[1808] = 32'h0FFF7870;
    rows[1809] = 32'h0F1C7870;
    rows[1810] = 32'h0F1C7870;
    rows[1811] = 32'h0F3C7870;
    rows[1812] = 32'h0F1E78F0;
    rows[1813] = 32'h0F1E7FF0;
    rows[1814] = 32'h0F3F7FF0;
    rows[1815] = 32'h0FFF7FE0;
    rows[1816] = 32'h0FFFFB80;
    rows[1817] = 32'h1FE7F800;
    rows[1818] = 32'h0F027800;
    rows[1819] = 32'h0C007800;
    rows[1820] = 32'h00007800;
    rows[1821] = 32'h00000000;
    rows[1822] = 32'h00000000;
    rows[1823] = 32'h00000000;
    rows[1824] = 32'h00000000;
    rows[1825] = 32'h00000000;
    rows[1826] = 32'h01E07800;
    rows[1827] = 32'h01E07800;
    rows[1828] = 32'h01E0F000;
    rows[1829] = 32'h01E1FFF8;
    rows[1830] = 32'h0DE3FFF8;
    rows[1831] = 32'h1DE7FFF8;
    rows[1832] = 32'h1DEF80F8;
    rows[1833] = 32'h1FEF21F0;
    rows[1834] = 32'h0FE6FFE0;
    rows[1835] = 32'h0FE07FC0;
    rows[1836] = 32'h0FE0FF80;
    rows[1837] = 32'h05EFFE00;
    rows[1838] = 32'h01E7F1E0;
    rows[1839] = 32'h01E701E0;
    rows[1840] = 32'h01E001E0;
    rows[1841] = 32'h03EFFFFC;
    rows[1842] = 32'h07EFFFFC;
    rows[1843] = 32'h0FEFFFFC;
    rows[1844] = 32'h1FE1C1E0;
    rows[1845] = 32'h1FE3C1E0;
    rows[1846] = 32'h1FE3E1E0;
    rows[1847] = 32'h19E1F1E0;
    rows[1848] = 32'h11E0E1E0;
    rows[1849] = 32'h01E0C1E0;
    rows[1850] = 32'h01E03FE0;
    rows[1851] = 32'h01E03FE0;
    rows[1852] = 32'h01E03FC0;
    rows[1853] = 32'h01E00000;
    rows[1854] = 32'h00000000;
    rows[1855] = 32'h00000000;
    rows[1856] = 32'h00000000;
    rows[1857] = 32'h00000000;
    rows[1858] = 32'h0603C380;
    rows[1859] = 32'h0F03C380;
    rows[1860] = 32'h0F03C380;
    rows[1861] = 32'h0F83C380;
    rows[1862] = 32'h07DFFFF0;
    rows[1863] = 32'h039FFFF0;
    rows[1864] = 32'h031FFFF0;
    rows[1865] = 32'h0003C7C0;
    rows[1866] = 32'h0003C380;
    rows[1867] = 32'h0003C380;
    rows[1868] = 32'h1F83C380;
    rows[1869] = 32'h1F83C380;
    rows[1870] = 32'h1FBFFFF8;
    rows[1871] = 32'h1FBFFFF8;
    rows[1872] = 32'h07BFFFF8;
    rows[1873] = 32'h078787C0;
    rows[1874] = 32'h07878380;
    rows[1875] = 32'h078F8380;
    rows[1876] = 32'h078F0380;
    rows[1877] = 32'h079F0380;
    rows[1878] = 32'h07870380;
    rows[1879] = 32'h0FC20000;
    rows[1880] = 32'h1FF00000;
    rows[1881] = 32'h1FFFFFFC;
    rows[1882] = 32'h1EFFFFF8;
    rows[1883] = 32'h1C3FFFF8;
    rows[1884] = 32'h0803FFF0;
    rows[1885] = 32'h00000000;
    rows[1886] = 32'h00000000;
    rows[1887] = 32'h00000000;
    rows[1888] = 32'h00000000;
    rows[1889] = 32'h00000000;
    rows[1890] = 32'h06007800;
    rows[1891] = 32'h0E007800;
    rows[1892] = 32'h0F3FFFF8;
    rows[1893] = 32'h0FBFFFF8;
    rows[1894] = 32'h07807C00;
    rows[1895] = 32'h07807800;
    rows[1896] = 32'h021FFFF0;
    rows[1897] = 32'h001FFFF0;
    rows[1898] = 32'h00007C00;
    rows[1899] = 32'h00007800;
    rows[1900] = 32'h3F7FFFFC;
    rows[1901] = 32'h3F7FFFFC;
    rows[1902] = 32'h3F000000;
    rows[1903] = 32'h1F1FFFF0;
    rows[1904] = 32'h071FFFF0;
    rows[1905] = 32'h071FFFF0;
    rows[1906] = 32'h071E00F0;
    rows[1907] = 32'h071FFFF0;
    rows[1908] = 32'h071FFFF0;
    rows[1909] = 32'h071E00F0;
    rows[1910] = 32'h075E00F0;
    rows[1911] = 32'h07DFFFF0;
    rows[1912] = 32'h07DFFFF0;
    rows[1913] = 32'h07DE00F0;
    rows[1914] = 32'h0F9E01F0;
    rows[1915] = 32'h0F1E3FE0;
    rows[1916] = 32'h061E1FE0;
    rows[1917] = 32'h061E1F80;
    rows[1918] = 32'h00000000;
    rows[1919] = 32'h00000000;
    rows[1920] = 32'h00000000;
    rows[1921] = 32'h00000000;
    rows[1922] = 32'h000F0000;
    rows[1923] = 32'h001F0000;
    rows[1924] = 32'h001E0000;
    rows[1925] = 32'h1FFFFFF8;
    rows[1926] = 32'h1FFFFFF8;
    rows[1927] = 32'h1FFFFFF8;
    rows[1928] = 32'h1FFFFFF8;
    rows[1929] = 32'h00783C00;
    rows[1930] = 32'h00F83C00;
    rows[1931] = 32'h01F03C00;
    rows[1932] = 32'h01F03C00;
    rows[1933] = 32'h03E03C00;
    rows[1934] = 32'h07C03C00;
    rows[1935] = 32'h0FCFFFF0;
    rows[1936] = 32'h0FCFFFF0;
    rows[1937] = 32'h1FCFFFF0;
    rows[1938] = 32'h1FCFFFF0;
    rows[1939] = 32'h1FC03C00;
    rows[1940] = 32'h1FC03C00;
    rows[1941] = 32'h03C03C00;
    rows[1942] = 32'h03C03C00;
    rows[1943] = 32'h03C03C00;
    rows[1944] = 32'h03C03C00;
    rows[1945] = 32'h03DFFFF8;
    rows[1946] = 32'h03DFFFF8;
    rows[1947] = 32'h03DFFFF8;
    rows[1948] = 32'h03DFFFF8;
    rows[1949] = 32'h03C00000;
    rows[1950] = 32'h00000000;
    rows[1951] = 32'h00000000;
    rows[1952] = 32'h00000000;
    rows[1953] = 32'h00000000;
    rows[1954] = 32'h00781E00;
    rows[1955] = 32'h00781E00;
    rows[1956] = 32'h0FFFFFF0;
    rows[1957] = 32'h0FFFFFF0;
    rows[1958] = 32'h0FFFFFF0;
    rows[1959] = 32'h00781E00;
    rows[1960] = 32'h00781E00;
    rows[1961] = 32'h1FFFFFF8;
    rows[1962] = 32'h1FFFFFF8;
    rows[1963] = 32'h1FFFFFF8;
    rows[1964] = 32'h0003C000;
    rows[1965] = 32'h07FFFFE0;
    rows[1966] = 32'h07FFFFE0;
    rows[1967] = 32'h07FFFFE0;
    rows[1968] = 32'h0783C1E0;
    rows[1969] = 32'h07FFFFE0;
    rows[1970] = 32'h07FFFFE0;
    rows[1971] = 32'h0783C1E0;
    rows[1972] = 32'h0783C1E0;
    rows[1973] = 32'h07FFFFE0;
    rows[1974] = 32'h07FFFFE0;
    rows[1975] = 32'h03FFFFC0;
    rows[1976] = 32'h00781E00;
    rows[1977] = 32'h0FF81FF0;
    rows[1978] = 32'h1FFC3FF8;
    rows[1979] = 32'h0FE007F0;
    rows[1980] = 32'h0E000030;
    rows[1981] = 32'h00000000;
    rows[1982] = 32'h00000000;
    rows[1983] = 32'h00000000;
    rows[1984] = 32'h00000000;
    rows[1985] = 32'h00000000;
    rows[1986] = 32'h003C0000;
    rows[1987] = 32'h007C0000;
    rows[1988] = 32'h007FFF00;
    rows[1989] = 32'h00FFFF00;
    rows[1990] = 32'h01FFFF00;
    rows[1991] = 32'h03E03E00;
    rows[1992] = 32'h07C03C00;
    rows[1993] = 32'h0F807C00;
    rows[1994] = 32'h1FFFFFE0;
    rows[1995] = 32'h1FFFFFE0;
    rows[1996] = 32'h1FFFFFE0;
    rows[1997] = 32'h0F83C1E0;
    rows[1998] = 32'h0783C1E0;
    rows[1999] = 32'h0783C1E0;
    rows[2000] = 32'h0783C1E0;
    rows[2001] = 32'h07FFFFE0;
    rows[2002] = 32'h07FFFFE0;
    rows[2003] = 32'h07FFFFE0;
    rows[2004] = 32'h07800000;
    rows[2005] = 32'h07800030;
    rows[2006] = 32'h07800038;
    rows[2007] = 32'h07800078;
    rows[2008] = 32'h07C00078;
    rows[2009] = 32'h07FFFFF8;
    rows[2010] = 32'h03FFFFF0;
    rows[2011] = 32'h01FFFFE0;
    rows[2012] = 32'h003FFE00;
    rows[2013] = 32'h00000000;
    rows[2014] = 32'h00000000;
    rows[2015] = 32'h00000000;
    rows[2016] = 32'h00000000;
    rows[2017] = 32'h00000000;
    rows[2018] = 32'h0001C000;
    rows[2019] = 32'h0007C000;
    rows[2020] = 32'h0003E000;
    rows[2021] = 32'h1FFFFFF8;
    rows[2022] = 32'h1FFFFFF8;
    rows[2023] = 32'h1FFFFFF8;
    rows[2024] = 32'h1F0000F8;
    rows[2025] = 32'h1F0E0078;
    rows[2026] = 32'h1F0F8078;
    rows[2027] = 32'h001F0000;
    rows[2028] = 32'h001F0000;
    rows[2029] = 32'h3FFFFFFC;
    rows[2030] = 32'h3FFFFFFC;
    rows[2031] = 32'h3FFFFFFC;
    rows[2032] = 32'h1FFFFFF8;
    rows[2033] = 32'h00F80F00;
    rows[2034] = 32'h00F01F00;
    rows[2035] = 32'h01F83E00;
    rows[2036] = 32'h01FF7E00;
    rows[2037] = 32'h007FFC00;
    rows[2038] = 32'h001FF800;
    rows[2039] = 32'h000FFF00;
    rows[2040] = 32'h00FFFFC0;
    rows[2041] = 32'h1FFFBFF0;
    rows[2042] = 32'h1FFE07F0;
    rows[2043] = 32'h0FF001F0;
    rows[2044] = 32'h0E000060;
    rows[2045] = 32'h00000000;
    rows[2046] = 32'h00000000;
    rows[2047] = 32'h00000000;
    rows[2048] = 32'h00000000;
    rows[2049] = 32'h00000000;
    rows[2050] = 32'h0003E000;
    rows[2051] = 32'h0007E000;
    rows[2052] = 32'h0007E000;
    rows[2053] = 32'h000FF000;
    rows[2054] = 32'h001FF800;
    rows[2055] = 32'h003E7C00;
    rows[2056] = 32'h007C3E00;
    rows[2057] = 32'h00F81F00;
    rows[2058] = 32'h03F00FC0;
    rows[2059] = 32'h07E007E0;
    rows[2060] = 32'h1FFFFFF8;
    rows[2061] = 32'h1FFFFFF8;
    rows[2062] = 32'h0E7FFE70;
    rows[2063] = 32'h0C7FFE30;
    rows[2064] = 32'h0003C000;
    rows[2065] = 32'h0003C000;
    rows[2066] = 32'h03FFFFC0;
    rows[2067] = 32'h03FFFFC0;
    rows[2068] = 32'h03FFFFC0;
    rows[2069] = 32'h03FFFFC0;
    rows[2070] = 32'h0003C000;
    rows[2071] = 32'h0003C000;
    rows[2072] = 32'h0003C000;
    rows[2073] = 32'h0FFFFFF0;
    rows[2074] = 32'h0FFFFFF0;
    rows[2075] = 32'h0FFFFFF0;
    rows[2076] = 32'h0FFFFFF0;
    rows[2077] = 32'h00000000;
    rows[2078] = 32'h00000000;
    rows[2079] = 32'h00000000;
    rows[2080] = 32'h00000000;
    rows[2081] = 32'h00000000;
    rows[2082] = 32'h0003C000;
    rows[2083] = 32'h0003C000;
    rows[2084] = 32'h0003C000;
    rows[2085] = 32'h0003C000;
    rows[2086] = 32'h0FFFFFF0;
    rows[2087] = 32'h0FFFFFF0;
    rows[2088] = 32'h0FFFFFF0;
    rows[2089] = 32'h0FFFFFF0;
    rows[2090] = 32'h0F03C0F0;
    rows[2091] = 32'h0F03C0F0;
    rows[2092] = 32'h0F03C0F0;
    rows[2093] = 32'h0F07C0F0;
    rows[2094] = 32'h0F07E0F0;
    rows[2095] = 32'h0F0FF0F0;
    rows[2096] = 32'h0F1FF8F0;
    rows[2097] = 32'h0F3E7CF0;
    rows[2098] = 32'h0F7E3EF0;
    rows[2099] = 32'h0FFC1FF0;
    rows[2100] = 32'h0F780EF0;
    rows[2101] = 32'h0F3004F0;
    rows[2102] = 32'h0F0000F0;
    rows[2103] = 32'h0F0000F0;
    rows[2104] = 32'h0F0000F0;
    rows[2105] = 32'h0F0000F0;
    rows[2106] = 32'h0F001FF0;
    rows[2107] = 32'h0F001FE0;
    rows[2108] = 32'h0F001FC0;
    rows[2109] = 32'h0F000E00;
    rows[2110] = 32'h00000000;
    rows[2111] = 32'h00000000;
    rows[2112] = 32'h00000000;
    rows[2113] = 32'h00000000;
    rows[2114] = 32'h03800000;
    rows[2115] = 32'h03C1FFE0;
    rows[2116] = 32'h03C1FFE0;
    rows[2117] = 32'h03F9FFE0;
    rows[2118] = 32'h07B801E0;
    rows[2119] = 32'h07B801E0;
    rows[2120] = 32'h073FFFF8;
    rows[2121] = 32'h0F3FFFF8;
    rows[2122] = 32'h0F3FFFF8;
    rows[2123] = 32'h0F39C000;
    rows[2124] = 32'h1F39C000;
    rows[2125] = 32'h1F39FFF0;
    rows[2126] = 32'h3F3BFFF0;
    rows[2127] = 32'h1F3BFFF0;
    rows[2128] = 32'h1F3F9E00;
    rows[2129] = 32'h173B1E00;
    rows[2130] = 32'h073FFFF8;
    rows[2131] = 32'h073FFFF8;
    rows[2132] = 32'h073FFFF8;
    rows[2133] = 32'h07383F00;
    rows[2134] = 32'h07383F80;
    rows[2135] = 32'h07387BC0;
    rows[2136] = 32'h0739F3E0;
    rows[2137] = 32'h0703F1F8;
    rows[2138] = 32'h070FE0F8;
    rows[2139] = 32'h07078078;
    rows[2140] = 32'h07020010;
    rows[2141] = 32'h00000000;
    rows[2142] = 32'h00000000;
    rows[2143] = 32'h00000000;
    rows[2144] = 32'h00000000;
    rows[2145] = 32'h00000000;
    rows[2146] = 32'h00000000;
    rows[2147] = 32'h00000000;
    rows[2148] = 32'h00000000;
    rows[2149] = 32'h00000000;
    rows[2150] = 32'h00000000;
    rows[2151] = 32'h00000000;
    rows[2152] = 32'h00000000;
    rows[2153] = 32'h00000000;
    rows[2154] = 32'h00000000;
    rows[2155] = 32'h00000000;
    rows[2156] = 32'h00000000;
    rows[2157] = 32'h00000000;
    rows[2158] = 32'h00000000;
    rows[2159] = 32'h00000000;
    rows[2160] = 32'h00000000;
    rows[2161] = 32'h00000000;
    rows[2162] = 32'h00000000;
    rows[2163] = 32'h00000000;
    rows[2164] = 32'h00000000;
    rows[2165] = 32'h07800000;
    rows[2166] = 32'h0FC00000;
    rows[2167] = 32'h0FE00000;
    rows[2168] = 32'h1CE00000;
    rows[2169] = 32'h1CE00000;
    rows[2170] = 32'h0FE00000;
    rows[2171] = 32'h07C00000;
    rows[2172] = 32'h01000000;
    rows[2173] = 32'h00000000;
    rows[2174] = 32'h00000000;
    rows[2175] = 32'h00000000;
    rows[2176] = 32'h00000000;
    rows[2177] = 32'h00000000;
    rows[2178] = 32'h00000000;
    rows[2179] = 32'h00000000;
    rows[2180] = 32'h00000000;
    rows[2181] = 32'h00000000;
    rows[2182] = 32'h00000000;
    rows[2183] = 32'h00000000;
    rows[2184] = 32'h00000000;
    rows[2185] = 32'h00000000;
    rows[2186] = 32'h00000000;
    rows[2187] = 32'h00000000;
    rows[2188] = 32'h00000000;
    rows[2189] = 32'h00000000;
    rows[2190] = 32'h00000000;
    rows[2191] = 32'h00000000;
    rows[2192] = 32'h00000000;
    rows[2193] = 32'h00000000;
    rows[2194] = 32'h00000000;
    rows[2195] = 32'h00000000;
    rows[2196] = 32'h00000000;
    rows[2197] = 32'h00000000;
    rows[2198] = 32'h00000000;
    rows[2199] = 32'h00000000;
    rows[2200] = 32'h00000000;
    rows[2201] = 32'h00000000;
    rows[2202] = 32'h00000000;
    rows[2203] = 32'h00000000;
    rows[2204] = 32'h00000000;
    rows[2205] = 32'h00000000;
    rows[2206] = 32'h00000000;
    rows[2207] = 32'h00000000;
    rows[2208] = 32'h00000000;
    rows[2209] = 32'h00000000;
    rows[2210] = 32'h00000000;
    rows[2211] = 32'h00000000;
    rows[2212] = 32'h00000000;
    rows[2213] = 32'h00000000;
    rows[2214] = 32'h00000000;
    rows[2215] = 32'h00000000;
    rows[2216] = 32'h00000000;
    rows[2217] = 32'h00000000;
    rows[2218] = 32'h00000000;
    rows[2219] = 32'h00000000;
    rows[2220] = 32'h00000000;
    rows[2221] = 32'h00000000;
    rows[2222] = 32'h00000000;
    rows[2223] = 32'h00000000;
    rows[2224] = 32'h00000000;
    rows[2225] = 32'h00000000;
    rows[2226] = 32'h00000000;
    rows[2227] = 32'h00000000;
    rows[2228] = 32'h00000000;
    rows[2229] = 32'h00000000;
    rows[2230] = 32'h00000000;
    rows[2231] = 32'h00000000;
    rows[2232] = 32'h00000000;
    rows[2233] = 32'h00000000;
    rows[2234] = 32'h00000000;
    rows[2235] = 32'h00000000;
    rows[2236] = 32'h00000000;
    rows[2237] = 32'h00000000;
    rows[2238] = 32'h00000000;
    rows[2239] = 32'h00000000;
    rows[2240] = 32'h00000000;
    rows[2241] = 32'h00000000;
    rows[2242] = 32'h00000000;
    rows[2243] = 32'h00000000;
    rows[2244] = 32'h00000000;
    rows[2245] = 32'h00000000;
    rows[2246] = 32'h00000000;
    rows[2247] = 32'h00000000;
    rows[2248] = 32'h00000000;
    rows[2249] = 32'h00000000;
    rows[2250] = 32'h00000000;
    rows[2251] = 32'h00000000;
    rows[2252] = 32'h00000000;
    rows[2253] = 32'h00000000;
    rows[2254] = 32'h00000000;
    rows[2255] = 32'h00000000;
    rows[2256] = 32'h00000000;
    rows[2257] = 32'h00000000;
    rows[2258] = 32'h00000000;
    rows[2259] = 32'h00000000;
    rows[2260] = 32'h00000000;
    rows[2261] = 32'h00000000;
    rows[2262] = 32'h00000000;
    rows[2263] = 32'h00000000;
    rows[2264] = 32'h00000000;
    rows[2265] = 32'h00000000;
    rows[2266] = 32'h00000000;
    rows[2267] = 32'h00000000;
    rows[2268] = 32'h00000000;
    rows[2269] = 32'h00000000;
    rows[2270] = 32'h00000000;
    rows[2271] = 32'h00000000;
    rows[2272] = 32'h00000000;
    rows[2273] = 32'h00000000;
    rows[2274] = 32'h00000000;
    rows[2275] = 32'h00000000;
    rows[2276] = 32'h00000000;
    rows[2277] = 32'h00000000;
    rows[2278] = 32'h00000000;
    rows[2279] = 32'h00000000;
    rows[2280] = 32'h00000000;
    rows[2281] = 32'h00000000;
    rows[2282] = 32'h00000000;
    rows[2283] = 32'h00000000;
    rows[2284] = 32'h00000000;
    rows[2285] = 32'h00000000;
    rows[2286] = 32'h00000000;
    rows[2287] = 32'h00000000;
    rows[2288] = 32'h00000000;
    rows[2289] = 32'h00000000;
    rows[2290] = 32'h00000000;
    rows[2291] = 32'h00000000;
    rows[2292] = 32'h00000000;
    rows[2293] = 32'h00000000;
    rows[2294] = 32'h00000000;
    rows[2295] = 32'h00000000;
    rows[2296] = 32'h00000000;
    rows[2297] = 32'h00000000;
    rows[2298] = 32'h00000000;
    rows[2299] = 32'h00000000;
    rows[2300] = 32'h00000000;
    rows[2301] = 32'h00000000;
    rows[2302] = 32'h00000000;
    rows[2303] = 32'h00000000;
    rows[2304] = 32'h00000000;
    rows[2305] = 32'h00000000;
    rows[2306] = 32'h00000000;
    rows[2307] = 32'h00000000;
    rows[2308] = 32'h00000000;
    rows[2309] = 32'h00000000;
    rows[2310] = 32'h00000000;
    rows[2311] = 32'h00000000;
    rows[2312] = 32'h00000000;
    rows[2313] = 32'h00000000;
    rows[2314] = 32'h00000000;
    rows[2315] = 32'h00000000;
    rows[2316] = 32'h00000000;
    rows[2317] = 32'h00000000;
    rows[2318] = 32'h00000000;
    rows[2319] = 32'h00000000;
    rows[2320] = 32'h00000000;
    rows[2321] = 32'h00000000;
    rows[2322] = 32'h00000000;
    rows[2323] = 32'h00000000;
    rows[2324] = 32'h00000000;
    rows[2325] = 32'h00000000;
    rows[2326] = 32'h00000000;
    rows[2327] = 32'h00000000;
    rows[2328] = 32'h00000000;
    rows[2329] = 32'h00000000;
    rows[2330] = 32'h00000000;
    rows[2331] = 32'h00000000;
    rows[2332] = 32'h00000000;
    rows[2333] = 32'h00000000;
    rows[2334] = 32'h00000000;
    rows[2335] = 32'h00000000;
    rows[2336] = 32'h00000000;
    rows[2337] = 32'h00000000;
    rows[2338] = 32'h00000000;
    rows[2339] = 32'h00000000;
    rows[2340] = 32'h00000000;
    rows[2341] = 32'h00000000;
    rows[2342] = 32'h00000000;
    rows[2343] = 32'h00000000;
    rows[2344] = 32'h00000000;
    rows[2345] = 32'h00000000;
    rows[2346] = 32'h00000000;
    rows[2347] = 32'h00000000;
    rows[2348] = 32'h00000000;
    rows[2349] = 32'h00000000;
    rows[2350] = 32'h00000000;
    rows[2351] = 32'h00000000;
    rows[2352] = 32'h00000000;
    rows[2353] = 32'h00000000;
    rows[2354] = 32'h00000000;
    rows[2355] = 32'h00000000;
    rows[2356] = 32'h00000000;
    rows[2357] = 32'h00000000;
    rows[2358] = 32'h00000000;
    rows[2359] = 32'h00000000;
    rows[2360] = 32'h00000000;
    rows[2361] = 32'h00000000;
    rows[2362] = 32'h00000000;
    rows[2363] = 32'h00000000;
    rows[2364] = 32'h00000000;
    rows[2365] = 32'h00000000;
    rows[2366] = 32'h00000000;
    rows[2367] = 32'h00000000;
    rows[2368] = 32'h00000000;
    rows[2369] = 32'h00000000;
    rows[2370] = 32'h00000000;
    rows[2371] = 32'h00000000;
    rows[2372] = 32'h00000000;
    rows[2373] = 32'h00000000;
    rows[2374] = 32'h00000000;
    rows[2375] = 32'h00000000;
    rows[2376] = 32'h00000000;
    rows[2377] = 32'h00000000;
    rows[2378] = 32'h00000000;
    rows[2379] = 32'h00000000;
    rows[2380] = 32'h00000000;
    rows[2381] = 32'h00000000;
    rows[2382] = 32'h00000000;
    rows[2383] = 32'h00000000;
    rows[2384] = 32'h00000000;
    rows[2385] = 32'h00000000;
    rows[2386] = 32'h00000000;
    rows[2387] = 32'h00000000;
    rows[2388] = 32'h00000000;
    rows[2389] = 32'h00000000;
    rows[2390] = 32'h00000000;
    rows[2391] = 32'h00000000;
    rows[2392] = 32'h00000000;
    rows[2393] = 32'h00000000;
    rows[2394] = 32'h00000000;
    rows[2395] = 32'h00000000;
    rows[2396] = 32'h00000000;
    rows[2397] = 32'h00000000;
    rows[2398] = 32'h00000000;
    rows[2399] = 32'h00000000;
    rows[2400] = 32'h00000000;
    rows[2401] = 32'h00000000;
    rows[2402] = 32'h00000000;
    rows[2403] = 32'h00000000;
    rows[2404] = 32'h00000000;
    rows[2405] = 32'h00000000;
    rows[2406] = 32'h00000000;
    rows[2407] = 32'h00000000;
    rows[2408] = 32'h00000000;
    rows[2409] = 32'h00000000;
    rows[2410] = 32'h00000000;
    rows[2411] = 32'h00000000;
    rows[2412] = 32'h00000000;
    rows[2413] = 32'h00000000;
    rows[2414] = 32'h00000000;
    rows[2415] = 32'h00000000;
    rows[2416] = 32'h00000000;
    rows[2417] = 32'h00000000;
    rows[2418] = 32'h00000000;
    rows[2419] = 32'h00000000;
    rows[2420] = 32'h00000000;
    rows[2421] = 32'h00000000;
    rows[2422] = 32'h00000000;
    rows[2423] = 32'h00000000;
    rows[2424] = 32'h00000000;
    rows[2425] = 32'h00000000;
    rows[2426] = 32'h00000000;
    rows[2427] = 32'h00000000;
    rows[2428] = 32'h00000000;
    rows[2429] = 32'h00000000;
    rows[2430] = 32'h00000000;
    rows[2431] = 32'h00000000;
    rows[2432] = 32'h00000000;
    rows[2433] = 32'h00000000;
    rows[2434] = 32'h00000000;
    rows[2435] = 32'h00000000;
    rows[2436] = 32'h00000000;
    rows[2437] = 32'h00000000;
    rows[2438] = 32'h00000000;
    rows[2439] = 32'h00000000;
    rows[2440] = 32'h00000000;
    rows[2441] = 32'h00000000;
    rows[2442] = 32'h00000000;
    rows[2443] = 32'h00000000;
    rows[2444] = 32'h00000000;
    rows[2445] = 32'h00000000;
    rows[2446] = 32'h00000000;
    rows[2447] = 32'h00000000;
    rows[2448] = 32'h00000000;
    rows[2449] = 32'h00000000;
    rows[2450] = 32'h00000000;
    rows[2451] = 32'h00000000;
    rows[2452] = 32'h00000000;
    rows[2453] = 32'h00000000;
    rows[2454] = 32'h00000000;
    rows[2455] = 32'h00000000;
    rows[2456] = 32'h00000000;
    rows[2457] = 32'h00000000;
    rows[2458] = 32'h00000000;
    rows[2459] = 32'h00000000;
    rows[2460] = 32'h00000000;
    rows[2461] = 32'h00000000;
    rows[2462] = 32'h00000000;
    rows[2463] = 32'h00000000;
    rows[2464] = 32'h00000000;
    rows[2465] = 32'h00000000;
    rows[2466] = 32'h00000000;
    rows[2467] = 32'h00000000;
    rows[2468] = 32'h00000000;
    rows[2469] = 32'h00000000;
    rows[2470] = 32'h00000000;
    rows[2471] = 32'h00000000;
    rows[2472] = 32'h00000000;
    rows[2473] = 32'h00000000;
    rows[2474] = 32'h00000000;
    rows[2475] = 32'h00000000;
    rows[2476] = 32'h00000000;
    rows[2477] = 32'h00000000;
    rows[2478] = 32'h00000000;
    rows[2479] = 32'h00000000;
    rows[2480] = 32'h00000000;
    rows[2481] = 32'h00000000;
    rows[2482] = 32'h00000000;
    rows[2483] = 32'h00000000;
    rows[2484] = 32'h00000000;
    rows[2485] = 32'h00000000;
    rows[2486] = 32'h00000000;
    rows[2487] = 32'h00000000;
    rows[2488] = 32'h00000000;
    rows[2489] = 32'h00000000;
    rows[2490] = 32'h00000000;
    rows[2491] = 32'h00000000;
    rows[2492] = 32'h00000000;
    rows[2493] = 32'h00000000;
    rows[2494] = 32'h00000000;
    rows[2495] = 32'h00000000;
    rows[2496] = 32'h00000000;
    rows[2497] = 32'h00000000;
    rows[2498] = 32'h00000000;
    rows[2499] = 32'h00000000;
    rows[2500] = 32'h00000000;
    rows[2501] = 32'h00000000;
    rows[2502] = 32'h00000000;
    rows[2503] = 32'h00000000;
    rows[2504] = 32'h00000000;
    rows[2505] = 32'h00000000;
    rows[2506] = 32'h00000000;
    rows[2507] = 32'h00000000;
    rows[2508] = 32'h00000000;
    rows[2509] = 32'h00000000;
    rows[2510] = 32'h00000000;
    rows[2511] = 32'h00000000;
    rows[2512] = 32'h00000000;
    rows[2513] = 32'h00000000;
    rows[2514] = 32'h00000000;
    rows[2515] = 32'h00000000;
    rows[2516] = 32'h00000000;
    rows[2517] = 32'h00000000;
    rows[2518] = 32'h00000000;
    rows[2519] = 32'h00000000;
    rows[2520] = 32'h00000000;
    rows[2521] = 32'h00000000;
    rows[2522] = 32'h00000000;
    rows[2523] = 32'h00000000;
    rows[2524] = 32'h00000000;
    rows[2525] = 32'h00000000;
    rows[2526] = 32'h00000000;
    rows[2527] = 32'h00000000;
    rows[2528] = 32'h00000000;
    rows[2529] = 32'h00000000;
    rows[2530] = 32'h00000000;
    rows[2531] = 32'h00000000;
    rows[2532] = 32'h00000000;
    rows[2533] = 32'h00000000;
    rows[2534] = 32'h00000000;
    rows[2535] = 32'h00000000;
    rows[2536] = 32'h00000000;
    rows[2537] = 32'h00000000;
    rows[2538] = 32'h00000000;
    rows[2539] = 32'h00000000;
    rows[2540] = 32'h00000000;
    rows[2541] = 32'h00000000;
    rows[2542] = 32'h00000000;
    rows[2543] = 32'h00000000;
    rows[2544] = 32'h00000000;
    rows[2545] = 32'h00000000;
    rows[2546] = 32'h00000000;
    rows[2547] = 32'h00000000;
    rows[2548] = 32'h00000000;
    rows[2549] = 32'h00000000;
    rows[2550] = 32'h00000000;
    rows[2551] = 32'h00000000;
    rows[2552] = 32'h00000000;
    rows[2553] = 32'h00000000;
    rows[2554] = 32'h00000000;
    rows[2555] = 32'h00000000;
    rows[2556] = 32'h00000000;
    rows[2557] = 32'h00000000;
    rows[2558] = 32'h00000000;
    rows[2559] = 32'h00000000;
    rows[2560] = 32'h00000000;
    rows[2561] = 32'h00000000;
    rows[2562] = 32'h00000000;
    rows[2563] = 32'h00000000;
    rows[2564] = 32'h00000000;
    rows[2565] = 32'h00000000;
    rows[2566] = 32'h00000000;
    rows[2567] = 32'h00000000;
    rows[2568] = 32'h00000000;
    rows[2569] = 32'h00000000;
    rows[2570] = 32'h00000000;
    rows[2571] = 32'h00000000;
    rows[2572] = 32'h00000000;
    rows[2573] = 32'h00000000;
    rows[2574] = 32'h00000000;
    rows[2575] = 32'h00000000;
    rows[2576] = 32'h00000000;
    rows[2577] = 32'h00000000;
    rows[2578] = 32'h00000000;
    rows[2579] = 32'h00000000;
    rows[2580] = 32'h00000000;
    rows[2581] = 32'h00000000;
    rows[2582] = 32'h00000000;
    rows[2583] = 32'h00000000;
    rows[2584] = 32'h00000000;
    rows[2585] = 32'h00000000;
    rows[2586] = 32'h00000000;
    rows[2587] = 32'h00000000;
    rows[2588] = 32'h00000000;
    rows[2589] = 32'h00000000;
    rows[2590] = 32'h00000000;
    rows[2591] = 32'h00000000;
    rows[2592] = 32'h00000000;
    rows[2593] = 32'h00000000;
    rows[2594] = 32'h00000000;
    rows[2595] = 32'h00000000;
    rows[2596] = 32'h00000000;
    rows[2597] = 32'h00000000;
    rows[2598] = 32'h00000000;
    rows[2599] = 32'h00000000;
    rows[2600] = 32'h00000000;
    rows[2601] = 32'h00000000;
    rows[2602] = 32'h00000000;
    rows[2603] = 32'h00000000;
    rows[2604] = 32'h00000000;
    rows[2605] = 32'h00000000;
    rows[2606] = 32'h00000000;
    rows[2607] = 32'h00000000;
    rows[2608] = 32'h00000000;
    rows[2609] = 32'h00000000;
    rows[2610] = 32'h00000000;
    rows[2611] = 32'h00000000;
    rows[2612] = 32'h00000000;
    rows[2613] = 32'h00000000;
    rows[2614] = 32'h00000000;
    rows[2615] = 32'h00000000;
    rows[2616] = 32'h00000000;
    rows[2617] = 32'h00000000;
    rows[2618] = 32'h00000000;
    rows[2619] = 32'h00000000;
    rows[2620] = 32'h00000000;
    rows[2621] = 32'h00000000;
    rows[2622] = 32'h00000000;
    rows[2623] = 32'h00000000;
    rows[2624] = 32'h00000000;
    rows[2625] = 32'h00000000;
    rows[2626] = 32'h00000000;
    rows[2627] = 32'h00000000;
    rows[2628] = 32'h00000000;
    rows[2629] = 32'h00000000;
    rows[2630] = 32'h00000000;
    rows[2631] = 32'h00000000;
    rows[2632] = 32'h00000000;
    rows[2633] = 32'h00000000;
    rows[2634] = 32'h00000000;
    rows[2635] = 32'h00000000;
    rows[2636] = 32'h00000000;
    rows[2637] = 32'h00000000;
    rows[2638] = 32'h00000000;
    rows[2639] = 32'h00000000;
    rows[2640] = 32'h00000000;
    rows[2641] = 32'h00000000;
    rows[2642] = 32'h00000000;
    rows[2643] = 32'h00000000;
    rows[2644] = 32'h00000000;
    rows[2645] = 32'h00000000;
    rows[2646] = 32'h00000000;
    rows[2647] = 32'h00000000;
    rows[2648] = 32'h00000000;
    rows[2649] = 32'h00000000;
    rows[2650] = 32'h00000000;
    rows[2651] = 32'h00000000;
    rows[2652] = 32'h00000000;
    rows[2653] = 32'h00000000;
    rows[2654] = 32'h00000000;
    rows[2655] = 32'h00000000;
    rows[2656] = 32'h00000000;
    rows[2657] = 32'h00000000;
    rows[2658] = 32'h00000000;
    rows[2659] = 32'h00000000;
    rows[2660] = 32'h00000000;
    rows[2661] = 32'h00000000;
    rows[2662] = 32'h00000000;
    rows[2663] = 32'h00000000;
    rows[2664] = 32'h00000000;
    rows[2665] = 32'h00000000;
    rows[2666] = 32'h00000000;
    rows[2667] = 32'h00000000;
    rows[2668] = 32'h00000000;
    rows[2669] = 32'h00000000;
    rows[2670] = 32'h00000000;
    rows[2671] = 32'h00000000;
    rows[2672] = 32'h00000000;
    rows[2673] = 32'h00000000;
    rows[2674] = 32'h00000000;
    rows[2675] = 32'h00000000;
    rows[2676] = 32'h00000000;
    rows[2677] = 32'h00000000;
    rows[2678] = 32'h00000000;
    rows[2679] = 32'h00000000;
    rows[2680] = 32'h00000000;
    rows[2681] = 32'h00000000;
    rows[2682] = 32'h00000000;
    rows[2683] = 32'h00000000;
    rows[2684] = 32'h00000000;
    rows[2685] = 32'h00000000;
    rows[2686] = 32'h00000000;
    rows[2687] = 32'h00000000;
    rows[2688] = 32'h00000000;
    rows[2689] = 32'h00000000;
    rows[2690] = 32'h00000000;
    rows[2691] = 32'h00000000;
    rows[2692] = 32'h00000000;
    rows[2693] = 32'h00000000;
    rows[2694] = 32'h00000000;
    rows[2695] = 32'h00000000;
    rows[2696] = 32'h00000000;
    rows[2697] = 32'h00000000;
    rows[2698] = 32'h00000000;
    rows[2699] = 32'h00000000;
    rows[2700] = 32'h00000000;
    rows[2701] = 32'h00000000;
    rows[2702] = 32'h00000000;
    rows[2703] = 32'h00000000;
    rows[2704] = 32'h00000000;
    rows[2705] = 32'h00000000;
    rows[2706] = 32'h00000000;
    rows[2707] = 32'h00000000;
    rows[2708] = 32'h00000000;
    rows[2709] = 32'h00000000;
    rows[2710] = 32'h00000000;
    rows[2711] = 32'h00000000;
    rows[2712] = 32'h00000000;
    rows[2713] = 32'h00000000;
    rows[2714] = 32'h00000000;
    rows[2715] = 32'h00000000;
    rows[2716] = 32'h00000000;
    rows[2717] = 32'h00000000;
    rows[2718] = 32'h00000000;
    rows[2719] = 32'h00000000;
    rows[2720] = 32'h00000000;
    rows[2721] = 32'h00000000;
    rows[2722] = 32'h00000000;
    rows[2723] = 32'h00000000;
    rows[2724] = 32'h00000000;
    rows[2725] = 32'h00000000;
    rows[2726] = 32'h00000000;
    rows[2727] = 32'h00000000;
    rows[2728] = 32'h00000000;
    rows[2729] = 32'h00000000;
    rows[2730] = 32'h00000000;
    rows[2731] = 32'h00000000;
    rows[2732] = 32'h00000000;
    rows[2733] = 32'h00000000;
    rows[2734] = 32'h00000000;
    rows[2735] = 32'h00000000;
    rows[2736] = 32'h00000000;
    rows[2737] = 32'h00000000;
    rows[2738] = 32'h00000000;
    rows[2739] = 32'h00000000;
    rows[2740] = 32'h00000000;
    rows[2741] = 32'h00000000;
    rows[2742] = 32'h00000000;
    rows[2743] = 32'h00000000;
    rows[2744] = 32'h00000000;
    rows[2745] = 32'h00000000;
    rows[2746] = 32'h00000000;
    rows[2747] = 32'h00000000;
    rows[2748] = 32'h00000000;
    rows[2749] = 32'h00000000;
    rows[2750] = 32'h00000000;
    rows[2751] = 32'h00000000;
    rows[2752] = 32'h00000000;
    rows[2753] = 32'h00000000;
    rows[2754] = 32'h00000000;
    rows[2755] = 32'h00000000;
    rows[2756] = 32'h00000000;
    rows[2757] = 32'h00000000;
    rows[2758] = 32'h00000000;
    rows[2759] = 32'h00000000;
    rows[2760] = 32'h00000000;
    rows[2761] = 32'h00000000;
    rows[2762] = 32'h00000000;
    rows[2763] = 32'h00000000;
    rows[2764] = 32'h00000000;
    rows[2765] = 32'h00000000;
    rows[2766] = 32'h00000000;
    rows[2767] = 32'h00000000;
    rows[2768] = 32'h00000000;
    rows[2769] = 32'h00000000;
    rows[2770] = 32'h00000000;
    rows[2771] = 32'h00000000;
    rows[2772] = 32'h00000000;
    rows[2773] = 32'h00000000;
    rows[2774] = 32'h00000000;
    rows[2775] = 32'h00000000;
    rows[2776] = 32'h00000000;
    rows[2777] = 32'h00000000;
    rows[2778] = 32'h00000000;
    rows[2779] = 32'h00000000;
    rows[2780] = 32'h00000000;
    rows[2781] = 32'h00000000;
    rows[2782] = 32'h00000000;
    rows[2783] = 32'h00000000;
    rows[2784] = 32'h00000000;
    rows[2785] = 32'h00000000;
    rows[2786] = 32'h00000000;
    rows[2787] = 32'h00000000;
    rows[2788] = 32'h00000000;
    rows[2789] = 32'h00000000;
    rows[2790] = 32'h00000000;
    rows[2791] = 32'h00000000;
    rows[2792] = 32'h00000000;
    rows[2793] = 32'h00000000;
    rows[2794] = 32'h00000000;
    rows[2795] = 32'h00000000;
    rows[2796] = 32'h00000000;
    rows[2797] = 32'h00000000;
    rows[2798] = 32'h00000000;
    rows[2799] = 32'h00000000;
    rows[2800] = 32'h00000000;
    rows[2801] = 32'h00000000;
    rows[2802] = 32'h00000000;
    rows[2803] = 32'h00000000;
    rows[2804] = 32'h00000000;
    rows[2805] = 32'h00000000;
    rows[2806] = 32'h00000000;
    rows[2807] = 32'h00000000;
    rows[2808] = 32'h00000000;
    rows[2809] = 32'h00000000;
    rows[2810] = 32'h00000000;
    rows[2811] = 32'h00000000;
    rows[2812] = 32'h00000000;
    rows[2813] = 32'h00000000;
    rows[2814] = 32'h00000000;
    rows[2815] = 32'h00000000;
    rows[2816] = 32'h00000000;
    rows[2817] = 32'h00000000;
    rows[2818] = 32'h00000000;
    rows[2819] = 32'h00000000;
    rows[2820] = 32'h00000000;
    rows[2821] = 32'h00000000;
    rows[2822] = 32'h00000000;
    rows[2823] = 32'h00000000;
    rows[2824] = 32'h00000000;
    rows[2825] = 32'h00000000;
    rows[2826] = 32'h00000000;
    rows[2827] = 32'h00000000;
    rows[2828] = 32'h00000000;
    rows[2829] = 32'h00000000;
    rows[2830] = 32'h00000000;
    rows[2831] = 32'h00000000;
    rows[2832] = 32'h00000000;
    rows[2833] = 32'h00000000;
    rows[2834] = 32'h00000000;
    rows[2835] = 32'h00000000;
    rows[2836] = 32'h00000000;
    rows[2837] = 32'h00000000;
    rows[2838] = 32'h00000000;
    rows[2839] = 32'h00000000;
    rows[2840] = 32'h00000000;
    rows[2841] = 32'h00000000;
    rows[2842] = 32'h00000000;
    rows[2843] = 32'h00000000;
    rows[2844] = 32'h00000000;
    rows[2845] = 32'h00000000;
    rows[2846] = 32'h00000000;
    rows[2847] = 32'h00000000;
    rows[2848] = 32'h00000000;
    rows[2849] = 32'h00000000;
    rows[2850] = 32'h00000000;
    rows[2851] = 32'h00000000;
    rows[2852] = 32'h00000000;
    rows[2853] = 32'h00000000;
    rows[2854] = 32'h00000000;
    rows[2855] = 32'h00000000;
    rows[2856] = 32'h00000000;
    rows[2857] = 32'h00000000;
    rows[2858] = 32'h00000000;
    rows[2859] = 32'h00000000;
    rows[2860] = 32'h00000000;
    rows[2861] = 32'h00000000;
    rows[2862] = 32'h00000000;
    rows[2863] = 32'h00000000;
    rows[2864] = 32'h00000000;
    rows[2865] = 32'h00000000;
    rows[2866] = 32'h00000000;
    rows[2867] = 32'h00000000;
    rows[2868] = 32'h00000000;
    rows[2869] = 32'h00000000;
    rows[2870] = 32'h00000000;
    rows[2871] = 32'h00000000;
    rows[2872] = 32'h00000000;
    rows[2873] = 32'h00000000;
    rows[2874] = 32'h00000000;
    rows[2875] = 32'h00000000;
    rows[2876] = 32'h00000000;
    rows[2877] = 32'h00000000;
    rows[2878] = 32'h00000000;
    rows[2879] = 32'h00000000;
    rows[2880] = 32'h00000000;
    rows[2881] = 32'h00000000;
    rows[2882] = 32'h00000000;
    rows[2883] = 32'h00000000;
    rows[2884] = 32'h00000000;
    rows[2885] = 32'h00000000;
    rows[2886] = 32'h00000000;
    rows[2887] = 32'h00000000;
    rows[2888] = 32'h00000000;
    rows[2889] = 32'h00000000;
    rows[2890] = 32'h00000000;
    rows[2891] = 32'h00000000;
    rows[2892] = 32'h00000000;
    rows[2893] = 32'h00000000;
    rows[2894] = 32'h00000000;
    rows[2895] = 32'h00000000;
    rows[2896] = 32'h00000000;
    rows[2897] = 32'h00000000;
    rows[2898] = 32'h00000000;
    rows[2899] = 32'h00000000;
    rows[2900] = 32'h00000000;
    rows[2901] = 32'h00000000;
    rows[2902] = 32'h00000000;
    rows[2903] = 32'h00000000;
    rows[2904] = 32'h00000000;
    rows[2905] = 32'h00000000;
    rows[2906] = 32'h00000000;
    rows[2907] = 32'h00000000;
    rows[2908] = 32'h00000000;
    rows[2909] = 32'h00000000;
    rows[2910] = 32'h00000000;
    rows[2911] = 32'h00000000;
    rows[2912] = 32'h00000000;
    rows[2913] = 32'h00000000;
    rows[2914] = 32'h00000000;
    rows[2915] = 32'h00000000;
    rows[2916] = 32'h00000000;
    rows[2917] = 32'h00000000;
    rows[2918] = 32'h00000000;
    rows[2919] = 32'h00000000;
    rows[2920] = 32'h00000000;
    rows[2921] = 32'h00000000;
    rows[2922] = 32'h00000000;
    rows[2923] = 32'h00000000;
    rows[2924] = 32'h00000000;
    rows[2925] = 32'h00000000;
    rows[2926] = 32'h00000000;
    rows[2927] = 32'h00000000;
    rows[2928] = 32'h00000000;
    rows[2929] = 32'h00000000;
    rows[2930] = 32'h00000000;
    rows[2931] = 32'h00000000;
    rows[2932] = 32'h00000000;
    rows[2933] = 32'h00000000;
    rows[2934] = 32'h00000000;
    rows[2935] = 32'h00000000;
    rows[2936] = 32'h00000000;
    rows[2937] = 32'h00000000;
    rows[2938] = 32'h00000000;
    rows[2939] = 32'h00000000;
    rows[2940] = 32'h00000000;
    rows[2941] = 32'h00000000;
    rows[2942] = 32'h00000000;
    rows[2943] = 32'h00000000;
    rows[2944] = 32'h00000000;
    rows[2945] = 32'h00000000;
    rows[2946] = 32'h00000000;
    rows[2947] = 32'h00000000;
    rows[2948] = 32'h00000000;
    rows[2949] = 32'h00000000;
    rows[2950] = 32'h00000000;
    rows[2951] = 32'h00000000;
    rows[2952] = 32'h00000000;
    rows[2953] = 32'h00000000;
    rows[2954] = 32'h00000000;
    rows[2955] = 32'h00000000;
    rows[2956] = 32'h00000000;
    rows[2957] = 32'h00000000;
    rows[2958] = 32'h00000000;
    rows[2959] = 32'h00000000;
    rows[2960] = 32'h00000000;
    rows[2961] = 32'h00000000;
    rows[2962] = 32'h00000000;
    rows[2963] = 32'h00000000;
    rows[2964] = 32'h00000000;
    rows[2965] = 32'h00000000;
    rows[2966] = 32'h00000000;
    rows[2967] = 32'h00000000;
    rows[2968] = 32'h00000000;
    rows[2969] = 32'h00000000;
    rows[2970] = 32'h00000000;
    rows[2971] = 32'h00000000;
    rows[2972] = 32'h00000000;
    rows[2973] = 32'h00000000;
    rows[2974] = 32'h00000000;
    rows[2975] = 32'h00000000;
    rows[2976] = 32'h00000000;
    rows[2977] = 32'h00000000;
    rows[2978] = 32'h00000000;
    rows[2979] = 32'h00000000;
    rows[2980] = 32'h00000000;
    rows[2981] = 32'h00000000;
    rows[2982] = 32'h00000000;
    rows[2983] = 32'h00000000;
    rows[2984] = 32'h00000000;
    rows[2985] = 32'h00000000;
    rows[2986] = 32'h00000000;
    rows[2987] = 32'h00000000;
    rows[2988] = 32'h00000000;
    rows[2989] = 32'h00000000;
    rows[2990] = 32'h00000000;
    rows[2991] = 32'h00000000;
    rows[2992] = 32'h00000000;
    rows[2993] = 32'h00000000;
    rows[2994] = 32'h00000000;
    rows[2995] = 32'h00000000;
    rows[2996] = 32'h00000000;
    rows[2997] = 32'h00000000;
    rows[2998] = 32'h00000000;
    rows[2999] = 32'h00000000;
    rows[3000] = 32'h00000000;
    rows[3001] = 32'h00000000;
    rows[3002] = 32'h00000000;
    rows[3003] = 32'h00000000;
    rows[3004] = 32'h00000000;
    rows[3005] = 32'h00000000;
    rows[3006] = 32'h00000000;
    rows[3007] = 32'h00000000;
    rows[3008] = 32'h00000000;
    rows[3009] = 32'h00000000;
    rows[3010] = 32'h00000000;
    rows[3011] = 32'h00000000;
    rows[3012] = 32'h00000000;
    rows[3013] = 32'h00000000;
    rows[3014] = 32'h00000000;
    rows[3015] = 32'h00000000;
    rows[3016] = 32'h00000000;
    rows[3017] = 32'h00000000;
    rows[3018] = 32'h00000000;
    rows[3019] = 32'h00000000;
    rows[3020] = 32'h00000000;
    rows[3021] = 32'h00000000;
    rows[3022] = 32'h00000000;
    rows[3023] = 32'h00000000;
    rows[3024] = 32'h00000000;
    rows[3025] = 32'h00000000;
    rows[3026] = 32'h00000000;
    rows[3027] = 32'h00000000;
    rows[3028] = 32'h00000000;
    rows[3029] = 32'h00000000;
    rows[3030] = 32'h00000000;
    rows[3031] = 32'h00000000;
    rows[3032] = 32'h00000000;
    rows[3033] = 32'h00000000;
    rows[3034] = 32'h00000000;
    rows[3035] = 32'h00000000;
    rows[3036] = 32'h00000000;
    rows[3037] = 32'h00000000;
    rows[3038] = 32'h00000000;
    rows[3039] = 32'h00000000;
    rows[3040] = 32'h00000000;
    rows[3041] = 32'h00000000;
    rows[3042] = 32'h00000000;
    rows[3043] = 32'h00000000;
    rows[3044] = 32'h00000000;
    rows[3045] = 32'h00000000;
    rows[3046] = 32'h00000000;
    rows[3047] = 32'h00000000;
    rows[3048] = 32'h00000000;
    rows[3049] = 32'h00000000;
    rows[3050] = 32'h00000000;
    rows[3051] = 32'h00000000;
    rows[3052] = 32'h00000000;
    rows[3053] = 32'h00000000;
    rows[3054] = 32'h00000000;
    rows[3055] = 32'h00000000;
    rows[3056] = 32'h00000000;
    rows[3057] = 32'h00000000;
    rows[3058] = 32'h00000000;
    rows[3059] = 32'h00000000;
    rows[3060] = 32'h00000000;
    rows[3061] = 32'h00000000;
    rows[3062] = 32'h00000000;
    rows[3063] = 32'h00000000;
    rows[3064] = 32'h00000000;
    rows[3065] = 32'h00000000;
    rows[3066] = 32'h00000000;
    rows[3067] = 32'h00000000;
    rows[3068] = 32'h00000000;
    rows[3069] = 32'h00000000;
    rows[3070] = 32'h00000000;
    rows[3071] = 32'h00000000;
    rows[3072] = 32'h00000000;
    rows[3073] = 32'h00000000;
    rows[3074] = 32'h00000000;
    rows[3075] = 32'h00000000;
    rows[3076] = 32'h00000000;
    rows[3077] = 32'h00000000;
    rows[3078] = 32'h00000000;
    rows[3079] = 32'h00000000;
    rows[3080] = 32'h00000000;
    rows[3081] = 32'h00000000;
    rows[3082] = 32'h00000000;
    rows[3083] = 32'h00000000;
    rows[3084] = 32'h00000000;
    rows[3085] = 32'h00000000;
    rows[3086] = 32'h00000000;
    rows[3087] = 32'h00000000;
    rows[3088] = 32'h00000000;
    rows[3089] = 32'h00000000;
    rows[3090] = 32'h00000000;
    rows[3091] = 32'h00000000;
    rows[3092] = 32'h00000000;
    rows[3093] = 32'h00000000;
    rows[3094] = 32'h00000000;
    rows[3095] = 32'h00000000;
    rows[3096] = 32'h00000000;
    rows[3097] = 32'h00000000;
    rows[3098] = 32'h00000000;
    rows[3099] = 32'h00000000;
    rows[3100] = 32'h00000000;
    rows[3101] = 32'h00000000;
    rows[3102] = 32'h00000000;
    rows[3103] = 32'h00000000;
    rows[3104] = 32'h00000000;
    rows[3105] = 32'h00000000;
    rows[3106] = 32'h00000000;
    rows[3107] = 32'h00000000;
    rows[3108] = 32'h00000000;
    rows[3109] = 32'h00000000;
    rows[3110] = 32'h00000000;
    rows[3111] = 32'h00000000;
    rows[3112] = 32'h00000000;
    rows[3113] = 32'h00000000;
    rows[3114] = 32'h00000000;
    rows[3115] = 32'h00000000;
    rows[3116] = 32'h00000000;
    rows[3117] = 32'h00000000;
    rows[3118] = 32'h00000000;
    rows[3119] = 32'h00000000;
    rows[3120] = 32'h00000000;
    rows[3121] = 32'h00000000;
    rows[3122] = 32'h00000000;
    rows[3123] = 32'h00000000;
    rows[3124] = 32'h00000000;
    rows[3125] = 32'h00000000;
    rows[3126] = 32'h00000000;
    rows[3127] = 32'h00000000;
    rows[3128] = 32'h00000000;
    rows[3129] = 32'h00000000;
    rows[3130] = 32'h00000000;
    rows[3131] = 32'h00000000;
    rows[3132] = 32'h00000000;
    rows[3133] = 32'h00000000;
    rows[3134] = 32'h00000000;
    rows[3135] = 32'h00000000;
    rows[3136] = 32'h00000000;
    rows[3137] = 32'h00000000;
    rows[3138] = 32'h00000000;
    rows[3139] = 32'h00000000;
    rows[3140] = 32'h00000000;
    rows[3141] = 32'h00000000;
    rows[3142] = 32'h00000000;
    rows[3143] = 32'h00000000;
    rows[3144] = 32'h00000000;
    rows[3145] = 32'h00000000;
    rows[3146] = 32'h00000000;
    rows[3147] = 32'h00000000;
    rows[3148] = 32'h00000000;
    rows[3149] = 32'h00000000;
    rows[3150] = 32'h00000000;
    rows[3151] = 32'h00000000;
    rows[3152] = 32'h00000000;
    rows[3153] = 32'h00000000;
    rows[3154] = 32'h00000000;
    rows[3155] = 32'h00000000;
    rows[3156] = 32'h00000000;
    rows[3157] = 32'h00000000;
    rows[3158] = 32'h00000000;
    rows[3159] = 32'h00000000;
    rows[3160] = 32'h00000000;
    rows[3161] = 32'h00000000;
    rows[3162] = 32'h00000000;
    rows[3163] = 32'h00000000;
    rows[3164] = 32'h00000000;
    rows[3165] = 32'h00000000;
    rows[3166] = 32'h00000000;
    rows[3167] = 32'h00000000;
    rows[3168] = 32'h00000000;
    rows[3169] = 32'h00000000;
    rows[3170] = 32'h00000000;
    rows[3171] = 32'h00000000;
    rows[3172] = 32'h00000000;
    rows[3173] = 32'h00000000;
    rows[3174] = 32'h00000000;
    rows[3175] = 32'h00000000;
    rows[3176] = 32'h00000000;
    rows[3177] = 32'h00000000;
    rows[3178] = 32'h00000000;
    rows[3179] = 32'h00000000;
    rows[3180] = 32'h00000000;
    rows[3181] = 32'h00000000;
    rows[3182] = 32'h00000000;
    rows[3183] = 32'h00000000;
    rows[3184] = 32'h00000000;
    rows[3185] = 32'h00000000;
    rows[3186] = 32'h00000000;
    rows[3187] = 32'h00000000;
    rows[3188] = 32'h00000000;
    rows[3189] = 32'h00000000;
    rows[3190] = 32'h00000000;
    rows[3191] = 32'h00000000;
    rows[3192] = 32'h00000000;
    rows[3193] = 32'h00000000;
    rows[3194] = 32'h00000000;
    rows[3195] = 32'h00000000;
    rows[3196] = 32'h00000000;
    rows[3197] = 32'h00000000;
    rows[3198] = 32'h00000000;
    rows[3199] = 32'h00000000;
    rows[3200] = 32'h00000000;
    rows[3201] = 32'h00000000;
    rows[3202] = 32'h00000000;
    rows[3203] = 32'h00000000;
    rows[3204] = 32'h00000000;
    rows[3205] = 32'h00000000;
    rows[3206] = 32'h00000000;
    rows[3207] = 32'h00000000;
    rows[3208] = 32'h00000000;
    rows[3209] = 32'h00000000;
    rows[3210] = 32'h00000000;
    rows[3211] = 32'h00000000;
    rows[3212] = 32'h00000000;
    rows[3213] = 32'h00000000;
    rows[3214] = 32'h00000000;
    rows[3215] = 32'h00000000;
    rows[3216] = 32'h00000000;
    rows[3217] = 32'h00000000;
    rows[3218] = 32'h00000000;
    rows[3219] = 32'h00000000;
    rows[3220] = 32'h00000000;
    rows[3221] = 32'h00000000;
    rows[3222] = 32'h00000000;
    rows[3223] = 32'h00000000;
    rows[3224] = 32'h00000000;
    rows[3225] = 32'h00000000;
    rows[3226] = 32'h00000000;
    rows[3227] = 32'h00000000;
    rows[3228] = 32'h00000000;
    rows[3229] = 32'h00000000;
    rows[3230] = 32'h00000000;
    rows[3231] = 32'h00000000;
    rows[3232] = 32'h00000000;
    rows[3233] = 32'h00000000;
    rows[3234] = 32'h00000000;
    rows[3235] = 32'h00000000;
    rows[3236] = 32'h00000000;
    rows[3237] = 32'h00000000;
    rows[3238] = 32'h00000000;
    rows[3239] = 32'h00000000;
    rows[3240] = 32'h00000000;
    rows[3241] = 32'h00000000;
    rows[3242] = 32'h00000000;
    rows[3243] = 32'h00000000;
    rows[3244] = 32'h00000000;
    rows[3245] = 32'h00000000;
    rows[3246] = 32'h00000000;
    rows[3247] = 32'h00000000;
    rows[3248] = 32'h00000000;
    rows[3249] = 32'h00000000;
    rows[3250] = 32'h00000000;
    rows[3251] = 32'h00000000;
    rows[3252] = 32'h00000000;
    rows[3253] = 32'h00000000;
    rows[3254] = 32'h00000000;
    rows[3255] = 32'h00000000;
    rows[3256] = 32'h00000000;
    rows[3257] = 32'h00000000;
    rows[3258] = 32'h00000000;
    rows[3259] = 32'h00000000;
    rows[3260] = 32'h00000000;
    rows[3261] = 32'h00000000;
    rows[3262] = 32'h00000000;
    rows[3263] = 32'h00000000;
    rows[3264] = 32'h00000000;
    rows[3265] = 32'h00000000;
    rows[3266] = 32'h00000000;
    rows[3267] = 32'h00000000;
    rows[3268] = 32'h00000000;
    rows[3269] = 32'h00000000;
    rows[3270] = 32'h00000000;
    rows[3271] = 32'h00000000;
    rows[3272] = 32'h00000000;
    rows[3273] = 32'h00000000;
    rows[3274] = 32'h00000000;
    rows[3275] = 32'h00000000;
    rows[3276] = 32'h00000000;
    rows[3277] = 32'h00000000;
    rows[3278] = 32'h00000000;
    rows[3279] = 32'h00000000;
    rows[3280] = 32'h00000000;
    rows[3281] = 32'h00000000;
    rows[3282] = 32'h00000000;
    rows[3283] = 32'h00000000;
    rows[3284] = 32'h00000000;
    rows[3285] = 32'h00000000;
    rows[3286] = 32'h00000000;
    rows[3287] = 32'h00000000;
    rows[3288] = 32'h00000000;
    rows[3289] = 32'h00000000;
    rows[3290] = 32'h00000000;
    rows[3291] = 32'h00000000;
    rows[3292] = 32'h00000000;
    rows[3293] = 32'h00000000;
    rows[3294] = 32'h00000000;
    rows[3295] = 32'h00000000;
    rows[3296] = 32'h00000000;
    rows[3297] = 32'h00000000;
    rows[3298] = 32'h00000000;
    rows[3299] = 32'h00000000;
    rows[3300] = 32'h00000000;
    rows[3301] = 32'h00000000;
    rows[3302] = 32'h00000000;
    rows[3303] = 32'h00000000;
    rows[3304] = 32'h00000000;
    rows[3305] = 32'h00000000;
    rows[3306] = 32'h00000000;
    rows[3307] = 32'h00000000;
    rows[3308] = 32'h00000000;
    rows[3309] = 32'h00000000;
    rows[3310] = 32'h00000000;
    rows[3311] = 32'h00000000;
    rows[3312] = 32'h00000000;
    rows[3313] = 32'h00000000;
    rows[3314] = 32'h00000000;
    rows[3315] = 32'h00000000;
    rows[3316] = 32'h00000000;
    rows[3317] = 32'h00000000;
    rows[3318] = 32'h00000000;
    rows[3319] = 32'h00000000;
    rows[3320] = 32'h00000000;
    rows[3321] = 32'h00000000;
    rows[3322] = 32'h00000000;
    rows[3323] = 32'h00000000;
    rows[3324] = 32'h00000000;
    rows[3325] = 32'h00000000;
    rows[3326] = 32'h00000000;
    rows[3327] = 32'h00000000;
    rows[3328] = 32'h00000000;
    rows[3329] = 32'h00000000;
    rows[3330] = 32'h00000000;
    rows[3331] = 32'h00000000;
    rows[3332] = 32'h00000000;
    rows[3333] = 32'h00000000;
    rows[3334] = 32'h00000000;
    rows[3335] = 32'h00000000;
    rows[3336] = 32'h00000000;
    rows[3337] = 32'h00000000;
    rows[3338] = 32'h00000000;
    rows[3339] = 32'h00000000;
    rows[3340] = 32'h00000000;
    rows[3341] = 32'h00000000;
    rows[3342] = 32'h00000000;
    rows[3343] = 32'h00000000;
    rows[3344] = 32'h00000000;
    rows[3345] = 32'h00000000;
    rows[3346] = 32'h00000000;
    rows[3347] = 32'h00000000;
    rows[3348] = 32'h00000000;
    rows[3349] = 32'h00000000;
    rows[3350] = 32'h00000000;
    rows[3351] = 32'h00000000;
    rows[3352] = 32'h00000000;
    rows[3353] = 32'h00000000;
    rows[3354] = 32'h00000000;
    rows[3355] = 32'h00000000;
    rows[3356] = 32'h00000000;
    rows[3357] = 32'h00000000;
    rows[3358] = 32'h00000000;
    rows[3359] = 32'h00000000;
    rows[3360] = 32'h00000000;
    rows[3361] = 32'h00000000;
    rows[3362] = 32'h00000000;
    rows[3363] = 32'h00000000;
    rows[3364] = 32'h00000000;
    rows[3365] = 32'h00000000;
    rows[3366] = 32'h00000000;
    rows[3367] = 32'h00000000;
    rows[3368] = 32'h00000000;
    rows[3369] = 32'h00000000;
    rows[3370] = 32'h00000000;
    rows[3371] = 32'h00000000;
    rows[3372] = 32'h00000000;
    rows[3373] = 32'h00000000;
    rows[3374] = 32'h00000000;
    rows[3375] = 32'h00000000;
    rows[3376] = 32'h00000000;
    rows[3377] = 32'h00000000;
    rows[3378] = 32'h00000000;
    rows[3379] = 32'h00000000;
    rows[3380] = 32'h00000000;
    rows[3381] = 32'h00000000;
    rows[3382] = 32'h00000000;
    rows[3383] = 32'h00000000;
    rows[3384] = 32'h00000000;
    rows[3385] = 32'h00000000;
    rows[3386] = 32'h00000000;
    rows[3387] = 32'h00000000;
    rows[3388] = 32'h00000000;
    rows[3389] = 32'h00000000;
    rows[3390] = 32'h00000000;
    rows[3391] = 32'h00000000;
    rows[3392] = 32'h00000000;
    rows[3393] = 32'h00000000;
    rows[3394] = 32'h00000000;
    rows[3395] = 32'h00000000;
    rows[3396] = 32'h00000000;
    rows[3397] = 32'h00000000;
    rows[3398] = 32'h00000000;
    rows[3399] = 32'h00000000;
    rows[3400] = 32'h00000000;
    rows[3401] = 32'h00000000;
    rows[3402] = 32'h00000000;
    rows[3403] = 32'h00000000;
    rows[3404] = 32'h00000000;
    rows[3405] = 32'h00000000;
    rows[3406] = 32'h00000000;
    rows[3407] = 32'h00000000;
    rows[3408] = 32'h00000000;
    rows[3409] = 32'h00000000;
    rows[3410] = 32'h00000000;
    rows[3411] = 32'h00000000;
    rows[3412] = 32'h00000000;
    rows[3413] = 32'h00000000;
    rows[3414] = 32'h00000000;
    rows[3415] = 32'h00000000;
    rows[3416] = 32'h00000000;
    rows[3417] = 32'h00000000;
    rows[3418] = 32'h00000000;
    rows[3419] = 32'h00000000;
    rows[3420] = 32'h00000000;
    rows[3421] = 32'h00000000;
    rows[3422] = 32'h00000000;
    rows[3423] = 32'h00000000;
    rows[3424] = 32'h00000000;
    rows[3425] = 32'h00000000;
    rows[3426] = 32'h00000000;
    rows[3427] = 32'h00000000;
    rows[3428] = 32'h00000000;
    rows[3429] = 32'h00000000;
    rows[3430] = 32'h00000000;
    rows[3431] = 32'h00000000;
    rows[3432] = 32'h00000000;
    rows[3433] = 32'h00000000;
    rows[3434] = 32'h00000000;
    rows[3435] = 32'h00000000;
    rows[3436] = 32'h00000000;
    rows[3437] = 32'h00000000;
    rows[3438] = 32'h00000000;
    rows[3439] = 32'h00000000;
    rows[3440] = 32'h00000000;
    rows[3441] = 32'h00000000;
    rows[3442] = 32'h00000000;
    rows[3443] = 32'h00000000;
    rows[3444] = 32'h00000000;
    rows[3445] = 32'h00000000;
    rows[3446] = 32'h00000000;
    rows[3447] = 32'h00000000;
    rows[3448] = 32'h00000000;
    rows[3449] = 32'h00000000;
    rows[3450] = 32'h00000000;
    rows[3451] = 32'h00000000;
    rows[3452] = 32'h00000000;
    rows[3453] = 32'h00000000;
    rows[3454] = 32'h00000000;
    rows[3455] = 32'h00000000;
    rows[3456] = 32'h00000000;
    rows[3457] = 32'h00000000;
    rows[3458] = 32'h00000000;
    rows[3459] = 32'h00000000;
    rows[3460] = 32'h00000000;
    rows[3461] = 32'h00000000;
    rows[3462] = 32'h00000000;
    rows[3463] = 32'h00000000;
    rows[3464] = 32'h00000000;
    rows[3465] = 32'h00000000;
    rows[3466] = 32'h00000000;
    rows[3467] = 32'h00000000;
    rows[3468] = 32'h00000000;
    rows[3469] = 32'h00000000;
    rows[3470] = 32'h00000000;
    rows[3471] = 32'h00000000;
    rows[3472] = 32'h00000000;
    rows[3473] = 32'h00000000;
    rows[3474] = 32'h00000000;
    rows[3475] = 32'h00000000;
    rows[3476] = 32'h00000000;
    rows[3477] = 32'h00000000;
    rows[3478] = 32'h00000000;
    rows[3479] = 32'h00000000;
    rows[3480] = 32'h00000000;
    rows[3481] = 32'h00000000;
    rows[3482] = 32'h00000000;
    rows[3483] = 32'h00000000;
    rows[3484] = 32'h00000000;
    rows[3485] = 32'h00000000;
    rows[3486] = 32'h00000000;
    rows[3487] = 32'h00000000;
    rows[3488] = 32'h00000000;
    rows[3489] = 32'h00000000;
    rows[3490] = 32'h00000000;
    rows[3491] = 32'h00000000;
    rows[3492] = 32'h00000000;
    rows[3493] = 32'h00000000;
    rows[3494] = 32'h00000000;
    rows[3495] = 32'h00000000;
    rows[3496] = 32'h00000000;
    rows[3497] = 32'h00000000;
    rows[3498] = 32'h00000000;
    rows[3499] = 32'h00000000;
    rows[3500] = 32'h00000000;
    rows[3501] = 32'h00000000;
    rows[3502] = 32'h00000000;
    rows[3503] = 32'h00000000;
    rows[3504] = 32'h00000000;
    rows[3505] = 32'h00000000;
    rows[3506] = 32'h00000000;
    rows[3507] = 32'h00000000;
    rows[3508] = 32'h00000000;
    rows[3509] = 32'h00000000;
    rows[3510] = 32'h00000000;
    rows[3511] = 32'h00000000;
    rows[3512] = 32'h00000000;
    rows[3513] = 32'h00000000;
    rows[3514] = 32'h00000000;
    rows[3515] = 32'h00000000;
    rows[3516] = 32'h00000000;
    rows[3517] = 32'h00000000;
    rows[3518] = 32'h00000000;
    rows[3519] = 32'h00000000;
    rows[3520] = 32'h00000000;
    rows[3521] = 32'h00000000;
    rows[3522] = 32'h00000000;
    rows[3523] = 32'h00000000;
    rows[3524] = 32'h00000000;
    rows[3525] = 32'h00000000;
    rows[3526] = 32'h00000000;
    rows[3527] = 32'h00000000;
    rows[3528] = 32'h00000000;
    rows[3529] = 32'h00000000;
    rows[3530] = 32'h00000000;
    rows[3531] = 32'h00000000;
    rows[3532] = 32'h00000000;
    rows[3533] = 32'h00000000;
    rows[3534] = 32'h00000000;
    rows[3535] = 32'h00000000;
    rows[3536] = 32'h00000000;
    rows[3537] = 32'h00000000;
    rows[3538] = 32'h00000000;
    rows[3539] = 32'h00000000;
    rows[3540] = 32'h00000000;
    rows[3541] = 32'h00000000;
    rows[3542] = 32'h00000000;
    rows[3543] = 32'h00000000;
    rows[3544] = 32'h00000000;
    rows[3545] = 32'h00000000;
    rows[3546] = 32'h00000000;
    rows[3547] = 32'h00000000;
    rows[3548] = 32'h00000000;
    rows[3549] = 32'h00000000;
    rows[3550] = 32'h00000000;
    rows[3551] = 32'h00000000;
    rows[3552] = 32'h00000000;
    rows[3553] = 32'h00000000;
    rows[3554] = 32'h00000000;
    rows[3555] = 32'h00000000;
    rows[3556] = 32'h00000000;
    rows[3557] = 32'h00000000;
    rows[3558] = 32'h00000000;
    rows[3559] = 32'h00000000;
    rows[3560] = 32'h00000000;
    rows[3561] = 32'h00000000;
    rows[3562] = 32'h00000000;
    rows[3563] = 32'h00000000;
    rows[3564] = 32'h00000000;
    rows[3565] = 32'h00000000;
    rows[3566] = 32'h00000000;
    rows[3567] = 32'h00000000;
    rows[3568] = 32'h00000000;
    rows[3569] = 32'h00000000;
    rows[3570] = 32'h00000000;
    rows[3571] = 32'h00000000;
    rows[3572] = 32'h00000000;
    rows[3573] = 32'h00000000;
    rows[3574] = 32'h00000000;
    rows[3575] = 32'h00000000;
    rows[3576] = 32'h00000000;
    rows[3577] = 32'h00000000;
    rows[3578] = 32'h00000000;
    rows[3579] = 32'h00000000;
    rows[3580] = 32'h00000000;
    rows[3581] = 32'h00000000;
    rows[3582] = 32'h00000000;
    rows[3583] = 32'h00000000;
    rows[3584] = 32'h00000000;
    rows[3585] = 32'h00000000;
    rows[3586] = 32'h00000000;
    rows[3587] = 32'h00000000;
    rows[3588] = 32'h00000000;
    rows[3589] = 32'h00000000;
    rows[3590] = 32'h00000000;
    rows[3591] = 32'h00000000;
    rows[3592] = 32'h00000000;
    rows[3593] = 32'h00000000;
    rows[3594] = 32'h00000000;
    rows[3595] = 32'h00000000;
    rows[3596] = 32'h00000000;
    rows[3597] = 32'h00000000;
    rows[3598] = 32'h00000000;
    rows[3599] = 32'h00000000;
    rows[3600] = 32'h00000000;
    rows[3601] = 32'h00000000;
    rows[3602] = 32'h00000000;
    rows[3603] = 32'h00000000;
    rows[3604] = 32'h00000000;
    rows[3605] = 32'h00000000;
    rows[3606] = 32'h00000000;
    rows[3607] = 32'h00000000;
    rows[3608] = 32'h00000000;
    rows[3609] = 32'h00000000;
    rows[3610] = 32'h00000000;
    rows[3611] = 32'h00000000;
    rows[3612] = 32'h00000000;
    rows[3613] = 32'h00000000;
    rows[3614] = 32'h00000000;
    rows[3615] = 32'h00000000;
    rows[3616] = 32'h00000000;
    rows[3617] = 32'h00000000;
    rows[3618] = 32'h00000000;
    rows[3619] = 32'h00000000;
    rows[3620] = 32'h00000000;
    rows[3621] = 32'h00000000;
    rows[3622] = 32'h00000000;
    rows[3623] = 32'h00000000;
    rows[3624] = 32'h00000000;
    rows[3625] = 32'h00000000;
    rows[3626] = 32'h00000000;
    rows[3627] = 32'h00000000;
    rows[3628] = 32'h00000000;
    rows[3629] = 32'h00000000;
    rows[3630] = 32'h00000000;
    rows[3631] = 32'h00000000;
    rows[3632] = 32'h00000000;
    rows[3633] = 32'h00000000;
    rows[3634] = 32'h00000000;
    rows[3635] = 32'h00000000;
    rows[3636] = 32'h00000000;
    rows[3637] = 32'h00000000;
    rows[3638] = 32'h00000000;
    rows[3639] = 32'h00000000;
    rows[3640] = 32'h00000000;
    rows[3641] = 32'h00000000;
    rows[3642] = 32'h00000000;
    rows[3643] = 32'h00000000;
    rows[3644] = 32'h00000000;
    rows[3645] = 32'h00000000;
    rows[3646] = 32'h00000000;
    rows[3647] = 32'h00000000;
    rows[3648] = 32'h00000000;
    rows[3649] = 32'h00000000;
    rows[3650] = 32'h00000000;
    rows[3651] = 32'h00000000;
    rows[3652] = 32'h00000000;
    rows[3653] = 32'h00000000;
    rows[3654] = 32'h00000000;
    rows[3655] = 32'h00000000;
    rows[3656] = 32'h00000000;
    rows[3657] = 32'h00000000;
    rows[3658] = 32'h00000000;
    rows[3659] = 32'h00000000;
    rows[3660] = 32'h00000000;
    rows[3661] = 32'h00000000;
    rows[3662] = 32'h00000000;
    rows[3663] = 32'h00000000;
    rows[3664] = 32'h00000000;
    rows[3665] = 32'h00000000;
    rows[3666] = 32'h00000000;
    rows[3667] = 32'h00000000;
    rows[3668] = 32'h00000000;
    rows[3669] = 32'h00000000;
    rows[3670] = 32'h00000000;
    rows[3671] = 32'h00000000;
    rows[3672] = 32'h00000000;
    rows[3673] = 32'h00000000;
    rows[3674] = 32'h00000000;
    rows[3675] = 32'h00000000;
    rows[3676] = 32'h00000000;
    rows[3677] = 32'h00000000;
    rows[3678] = 32'h00000000;
    rows[3679] = 32'h00000000;
    rows[3680] = 32'h00000000;
    rows[3681] = 32'h00000000;
    rows[3682] = 32'h00000000;
    rows[3683] = 32'h00000000;
    rows[3684] = 32'h00000000;
    rows[3685] = 32'h00000000;
    rows[3686] = 32'h00000000;
    rows[3687] = 32'h00000000;
    rows[3688] = 32'h00000000;
    rows[3689] = 32'h00000000;
    rows[3690] = 32'h00000000;
    rows[3691] = 32'h00000000;
    rows[3692] = 32'h00000000;
    rows[3693] = 32'h00000000;
    rows[3694] = 32'h00000000;
    rows[3695] = 32'h00000000;
    rows[3696] = 32'h00000000;
    rows[3697] = 32'h00000000;
    rows[3698] = 32'h00000000;
    rows[3699] = 32'h00000000;
    rows[3700] = 32'h00000000;
    rows[3701] = 32'h00000000;
    rows[3702] = 32'h00000000;
    rows[3703] = 32'h00000000;
    rows[3704] = 32'h00000000;
    rows[3705] = 32'h00000000;
    rows[3706] = 32'h00000000;
    rows[3707] = 32'h00000000;
    rows[3708] = 32'h00000000;
    rows[3709] = 32'h00000000;
    rows[3710] = 32'h00000000;
    rows[3711] = 32'h00000000;
    rows[3712] = 32'h00000000;
    rows[3713] = 32'h00000000;
    rows[3714] = 32'h00000000;
    rows[3715] = 32'h00000000;
    rows[3716] = 32'h00000000;
    rows[3717] = 32'h00000000;
    rows[3718] = 32'h00000000;
    rows[3719] = 32'h00000000;
    rows[3720] = 32'h00000000;
    rows[3721] = 32'h00000000;
    rows[3722] = 32'h00000000;
    rows[3723] = 32'h00000000;
    rows[3724] = 32'h00000000;
    rows[3725] = 32'h00000000;
    rows[3726] = 32'h00000000;
    rows[3727] = 32'h00000000;
    rows[3728] = 32'h00000000;
    rows[3729] = 32'h00000000;
    rows[3730] = 32'h00000000;
    rows[3731] = 32'h00000000;
    rows[3732] = 32'h00000000;
    rows[3733] = 32'h00000000;
    rows[3734] = 32'h00000000;
    rows[3735] = 32'h00000000;
    rows[3736] = 32'h00000000;
    rows[3737] = 32'h00000000;
    rows[3738] = 32'h00000000;
    rows[3739] = 32'h00000000;
    rows[3740] = 32'h00000000;
    rows[3741] = 32'h00000000;
    rows[3742] = 32'h00000000;
    rows[3743] = 32'h00000000;
    rows[3744] = 32'h00000000;
    rows[3745] = 32'h00000000;
    rows[3746] = 32'h00000000;
    rows[3747] = 32'h00000000;
    rows[3748] = 32'h00000000;
    rows[3749] = 32'h00000000;
    rows[3750] = 32'h00000000;
    rows[3751] = 32'h00000000;
    rows[3752] = 32'h00000000;
    rows[3753] = 32'h00000000;
    rows[3754] = 32'h00000000;
    rows[3755] = 32'h00000000;
    rows[3756] = 32'h00000000;
    rows[3757] = 32'h00000000;
    rows[3758] = 32'h00000000;
    rows[3759] = 32'h00000000;
    rows[3760] = 32'h00000000;
    rows[3761] = 32'h00000000;
    rows[3762] = 32'h00000000;
    rows[3763] = 32'h00000000;
    rows[3764] = 32'h00000000;
    rows[3765] = 32'h00000000;
    rows[3766] = 32'h00000000;
    rows[3767] = 32'h00000000;
    rows[3768] = 32'h00000000;
    rows[3769] = 32'h00000000;
    rows[3770] = 32'h00000000;
    rows[3771] = 32'h00000000;
    rows[3772] = 32'h00000000;
    rows[3773] = 32'h00000000;
    rows[3774] = 32'h00000000;
    rows[3775] = 32'h00000000;
    rows[3776] = 32'h00000000;
    rows[3777] = 32'h00000000;
    rows[3778] = 32'h00000000;
    rows[3779] = 32'h00000000;
    rows[3780] = 32'h00000000;
    rows[3781] = 32'h00000000;
    rows[3782] = 32'h00000000;
    rows[3783] = 32'h00000000;
    rows[3784] = 32'h00000000;
    rows[3785] = 32'h00000000;
    rows[3786] = 32'h00000000;
    rows[3787] = 32'h00000000;
    rows[3788] = 32'h00000000;
    rows[3789] = 32'h00000000;
    rows[3790] = 32'h00000000;
    rows[3791] = 32'h00000000;
    rows[3792] = 32'h00000000;
    rows[3793] = 32'h00000000;
    rows[3794] = 32'h00000000;
    rows[3795] = 32'h00000000;
    rows[3796] = 32'h00000000;
    rows[3797] = 32'h00000000;
    rows[3798] = 32'h00000000;
    rows[3799] = 32'h00000000;
    rows[3800] = 32'h00000000;
    rows[3801] = 32'h00000000;
    rows[3802] = 32'h00000000;
    rows[3803] = 32'h00000000;
    rows[3804] = 32'h00000000;
    rows[3805] = 32'h00000000;
    rows[3806] = 32'h00000000;
    rows[3807] = 32'h00000000;
    rows[3808] = 32'h00000000;
    rows[3809] = 32'h00000000;
    rows[3810] = 32'h00000000;
    rows[3811] = 32'h00000000;
    rows[3812] = 32'h00000000;
    rows[3813] = 32'h00000000;
    rows[3814] = 32'h00000000;
    rows[3815] = 32'h00000000;
    rows[3816] = 32'h00000000;
    rows[3817] = 32'h00000000;
    rows[3818] = 32'h00000000;
    rows[3819] = 32'h00000000;
    rows[3820] = 32'h00000000;
    rows[3821] = 32'h00000000;
    rows[3822] = 32'h00000000;
    rows[3823] = 32'h00000000;
    rows[3824] = 32'h00000000;
    rows[3825] = 32'h00000000;
    rows[3826] = 32'h00000000;
    rows[3827] = 32'h00000000;
    rows[3828] = 32'h00000000;
    rows[3829] = 32'h00000000;
    rows[3830] = 32'h00000000;
    rows[3831] = 32'h00000000;
    rows[3832] = 32'h00000000;
    rows[3833] = 32'h00000000;
    rows[3834] = 32'h00000000;
    rows[3835] = 32'h00000000;
    rows[3836] = 32'h00000000;
    rows[3837] = 32'h00000000;
    rows[3838] = 32'h00000000;
    rows[3839] = 32'h00000000;
    rows[3840] = 32'h00000000;
    rows[3841] = 32'h00000000;
    rows[3842] = 32'h00000000;
    rows[3843] = 32'h00000000;
    rows[3844] = 32'h00000000;
    rows[3845] = 32'h00000000;
    rows[3846] = 32'h00000000;
    rows[3847] = 32'h00000000;
    rows[3848] = 32'h00000000;
    rows[3849] = 32'h00000000;
    rows[3850] = 32'h00000000;
    rows[3851] = 32'h00000000;
    rows[3852] = 32'h00000000;
    rows[3853] = 32'h00000000;
    rows[3854] = 32'h00000000;
    rows[3855] = 32'h00000000;
    rows[3856] = 32'h00000000;
    rows[3857] = 32'h00000000;
    rows[3858] = 32'h00000000;
    rows[3859] = 32'h00000000;
    rows[3860] = 32'h00000000;
    rows[3861] = 32'h00000000;
    rows[3862] = 32'h00000000;
    rows[3863] = 32'h00000000;
    rows[3864] = 32'h00000000;
    rows[3865] = 32'h00000000;
    rows[3866] = 32'h00000000;
    rows[3867] = 32'h00000000;
    rows[3868] = 32'h00000000;
    rows[3869] = 32'h00000000;
    rows[3870] = 32'h00000000;
    rows[3871] = 32'h00000000;
    rows[3872] = 32'h00000000;
    rows[3873] = 32'h00000000;
    rows[3874] = 32'h00000000;
    rows[3875] = 32'h00000000;
    rows[3876] = 32'h00000000;
    rows[3877] = 32'h00000000;
    rows[3878] = 32'h00000000;
    rows[3879] = 32'h00000000;
    rows[3880] = 32'h00000000;
    rows[3881] = 32'h00000000;
    rows[3882] = 32'h00000000;
    rows[3883] = 32'h00000000;
    rows[3884] = 32'h00000000;
    rows[3885] = 32'h00000000;
    rows[3886] = 32'h00000000;
    rows[3887] = 32'h00000000;
    rows[3888] = 32'h00000000;
    rows[3889] = 32'h00000000;
    rows[3890] = 32'h00000000;
    rows[3891] = 32'h00000000;
    rows[3892] = 32'h00000000;
    rows[3893] = 32'h00000000;
    rows[3894] = 32'h00000000;
    rows[3895] = 32'h00000000;
    rows[3896] = 32'h00000000;
    rows[3897] = 32'h00000000;
    rows[3898] = 32'h00000000;
    rows[3899] = 32'h00000000;
    rows[3900] = 32'h00000000;
    rows[3901] = 32'h00000000;
    rows[3902] = 32'h00000000;
    rows[3903] = 32'h00000000;
    rows[3904] = 32'h00000000;
    rows[3905] = 32'h00000000;
    rows[3906] = 32'h00000000;
    rows[3907] = 32'h00000000;
    rows[3908] = 32'h00000000;
    rows[3909] = 32'h00000000;
    rows[3910] = 32'h00000000;
    rows[3911] = 32'h00000000;
    rows[3912] = 32'h00000000;
    rows[3913] = 32'h00000000;
    rows[3914] = 32'h00000000;
    rows[3915] = 32'h00000000;
    rows[3916] = 32'h00000000;
    rows[3917] = 32'h00000000;
    rows[3918] = 32'h00000000;
    rows[3919] = 32'h00000000;
    rows[3920] = 32'h00000000;
    rows[3921] = 32'h00000000;
    rows[3922] = 32'h00000000;
    rows[3923] = 32'h00000000;
    rows[3924] = 32'h00000000;
    rows[3925] = 32'h00000000;
    rows[3926] = 32'h00000000;
    rows[3927] = 32'h00000000;
    rows[3928] = 32'h00000000;
    rows[3929] = 32'h00000000;
    rows[3930] = 32'h00000000;
    rows[3931] = 32'h00000000;
    rows[3932] = 32'h00000000;
    rows[3933] = 32'h00000000;
    rows[3934] = 32'h00000000;
    rows[3935] = 32'h00000000;
    rows[3936] = 32'h00000000;
    rows[3937] = 32'h00000000;
    rows[3938] = 32'h00000000;
    rows[3939] = 32'h00000000;
    rows[3940] = 32'h00000000;
    rows[3941] = 32'h00000000;
    rows[3942] = 32'h00000000;
    rows[3943] = 32'h00000000;
    rows[3944] = 32'h00000000;
    rows[3945] = 32'h00000000;
    rows[3946] = 32'h00000000;
    rows[3947] = 32'h00000000;
    rows[3948] = 32'h00000000;
    rows[3949] = 32'h00000000;
    rows[3950] = 32'h00000000;
    rows[3951] = 32'h00000000;
    rows[3952] = 32'h00000000;
    rows[3953] = 32'h00000000;
    rows[3954] = 32'h00000000;
    rows[3955] = 32'h00000000;
    rows[3956] = 32'h00000000;
    rows[3957] = 32'h00000000;
    rows[3958] = 32'h00000000;
    rows[3959] = 32'h00000000;
    rows[3960] = 32'h00000000;
    rows[3961] = 32'h00000000;
    rows[3962] = 32'h00000000;
    rows[3963] = 32'h00000000;
    rows[3964] = 32'h00000000;
    rows[3965] = 32'h00000000;
    rows[3966] = 32'h00000000;
    rows[3967] = 32'h00000000;
    rows[3968] = 32'h00000000;
    rows[3969] = 32'h00000000;
    rows[3970] = 32'h00000000;
    rows[3971] = 32'h00000000;
    rows[3972] = 32'h00000000;
    rows[3973] = 32'h00000000;
    rows[3974] = 32'h00000000;
    rows[3975] = 32'h00000000;
    rows[3976] = 32'h00000000;
    rows[3977] = 32'h00000000;
    rows[3978] = 32'h00000000;
    rows[3979] = 32'h00000000;
    rows[3980] = 32'h00000000;
    rows[3981] = 32'h00000000;
    rows[3982] = 32'h00000000;
    rows[3983] = 32'h00000000;
    rows[3984] = 32'h00000000;
    rows[3985] = 32'h00000000;
    rows[3986] = 32'h00000000;
    rows[3987] = 32'h00000000;
    rows[3988] = 32'h00000000;
    rows[3989] = 32'h00000000;
    rows[3990] = 32'h00000000;
    rows[3991] = 32'h00000000;
    rows[3992] = 32'h00000000;
    rows[3993] = 32'h00000000;
    rows[3994] = 32'h00000000;
    rows[3995] = 32'h00000000;
    rows[3996] = 32'h00000000;
    rows[3997] = 32'h00000000;
    rows[3998] = 32'h00000000;
    rows[3999] = 32'h00000000;
    rows[4000] = 32'h00000000;
    rows[4001] = 32'h00000000;
    rows[4002] = 32'h00000000;
    rows[4003] = 32'h00000000;
    rows[4004] = 32'h00000000;
    rows[4005] = 32'h00000000;
    rows[4006] = 32'h00000000;
    rows[4007] = 32'h00000000;
    rows[4008] = 32'h00000000;
    rows[4009] = 32'h00000000;
    rows[4010] = 32'h00000000;
    rows[4011] = 32'h00000000;
    rows[4012] = 32'h00000000;
    rows[4013] = 32'h00000000;
    rows[4014] = 32'h00000000;
    rows[4015] = 32'h00000000;
    rows[4016] = 32'h00000000;
    rows[4017] = 32'h00000000;
    rows[4018] = 32'h00000000;
    rows[4019] = 32'h00000000;
    rows[4020] = 32'h00000000;
    rows[4021] = 32'h00000000;
    rows[4022] = 32'h00000000;
    rows[4023] = 32'h00000000;
    rows[4024] = 32'h00000000;
    rows[4025] = 32'h00000000;
    rows[4026] = 32'h00000000;
    rows[4027] = 32'h00000000;
    rows[4028] = 32'h00000000;
    rows[4029] = 32'h00000000;
    rows[4030] = 32'h00000000;
    rows[4031] = 32'h00000000;
    rows[4032] = 32'h00000000;
    rows[4033] = 32'h00000000;
    rows[4034] = 32'h00000000;
    rows[4035] = 32'h00000000;
    rows[4036] = 32'h00000000;
    rows[4037] = 32'h00000000;
    rows[4038] = 32'h00000000;
    rows[4039] = 32'h00000000;
    rows[4040] = 32'h00000000;
    rows[4041] = 32'h00000000;
    rows[4042] = 32'h00000000;
    rows[4043] = 32'h00000000;
    rows[4044] = 32'h00000000;
    rows[4045] = 32'h00000000;
    rows[4046] = 32'h00000000;
    rows[4047] = 32'h00000000;
    rows[4048] = 32'h00000000;
    rows[4049] = 32'h00000000;
    rows[4050] = 32'h00000000;
    rows[4051] = 32'h00000000;
    rows[4052] = 32'h00000000;
    rows[4053] = 32'h00000000;
    rows[4054] = 32'h00000000;
    rows[4055] = 32'h00000000;
    rows[4056] = 32'h00000000;
    rows[4057] = 32'h00000000;
    rows[4058] = 32'h00000000;
    rows[4059] = 32'h00000000;
    rows[4060] = 32'h00000000;
    rows[4061] = 32'h00000000;
    rows[4062] = 32'h00000000;
    rows[4063] = 32'h00000000;
    rows[4064] = 32'h00000000;
    rows[4065] = 32'h00000000;
    rows[4066] = 32'h00000000;
    rows[4067] = 32'h00000000;
    rows[4068] = 32'h00000000;
    rows[4069] = 32'h00000000;
    rows[4070] = 32'h00000000;
    rows[4071] = 32'h00000000;
    rows[4072] = 32'h00000000;
    rows[4073] = 32'h00000000;
    rows[4074] = 32'h00000000;
    rows[4075] = 32'h00000000;
    rows[4076] = 32'h00000000;
    rows[4077] = 32'h00000000;
    rows[4078] = 32'h00000000;
    rows[4079] = 32'h00000000;
    rows[4080] = 32'h00000000;
    rows[4081] = 32'h00000000;
    rows[4082] = 32'h00000000;
    rows[4083] = 32'h00000000;
    rows[4084] = 32'h00000000;
    rows[4085] = 32'h00000000;
    rows[4086] = 32'h00000000;
    rows[4087] = 32'h00000000;
    rows[4088] = 32'h00000000;
    rows[4089] = 32'h00000000;
    rows[4090] = 32'h00000000;
    rows[4091] = 32'h00000000;
    rows[4092] = 32'h00000000;
    rows[4093] = 32'h00000000;
    rows[4094] = 32'h00000000;
    rows[4095] = 32'h00000000;
end
always @(posedge I_clk) O_row <= rows[I_addr];
endmodule
// FONT32_ROM_END


module osd_ticker_bar (
    input wire I_clk, I_rst, I_hs, I_vs, I_de, I_enable,
    input wire [23:0] I_rgb, output reg [23:0] O_rgb
);
always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) O_rgb<=0;
    else O_rgb<=I_de ? I_rgb : 24'd0;
end
endmodule

// ---------------------------------------------------------------------------
// Slice 6: 8-band bandpass (diff of 1st-order IIR LPF) + peak envelope
// Driven by the same PCM word that feeds HDMI audio IP.
// ---------------------------------------------------------------------------
module audio_bandpass_meter (
    input  wire               I_clk,
    input  wire               I_rst,
    input  wire               I_audio_valid,
    input  wire signed [23:0] I_pcm,
    output reg  [7:0]         O_bar0,
    output reg  [7:0]         O_bar1,
    output reg  [7:0]         O_bar2,
    output reg  [7:0]         O_bar3,
    output reg  [7:0]         O_bar4,
    output reg  [7:0]         O_bar5,
    output reg  [7:0]         O_bar6,
    output reg  [7:0]         O_bar7
);
localparam [7:0] BAR_MAX = 8'd96;

reg signed [15:0] lp0, lp1, lp2, lp3, lp4, lp5, lp6, lp7, lp8;
reg        [15:0] env0, env1, env2, env3, env4, env5, env6, env7;

wire signed [15:0] x = I_pcm[23:8];

function [15:0] abs16;
    input signed [15:0] v;
    begin
        abs16 = v[15] ? (~v + 16'd1) : v;
    end
endfunction

function [7:0] env_to_bar;
    input [15:0] e;
    reg   [15:0] h;
    begin
        h = e >> 5;
        if (h > {8'd0, BAR_MAX})
            env_to_bar = BAR_MAX;
        else
            env_to_bar = h[7:0];
    end
endfunction

wire [15:0] mag0 = abs16(lp0 - lp1);
wire [15:0] mag1 = abs16(lp1 - lp2);
wire [15:0] mag2 = abs16(lp2 - lp3);
wire [15:0] mag3 = abs16(lp3 - lp4);
wire [15:0] mag4 = abs16(lp4 - lp5);
wire [15:0] mag5 = abs16(lp5 - lp6);
wire [15:0] mag6 = abs16(lp6 - lp7);
wire [15:0] mag7 = abs16(lp7 - lp8);

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        lp0 <= 16'sd0; lp1 <= 16'sd0; lp2 <= 16'sd0; lp3 <= 16'sd0;
        lp4 <= 16'sd0; lp5 <= 16'sd0; lp6 <= 16'sd0; lp7 <= 16'sd0; lp8 <= 16'sd0;
        env0 <= 16'd0; env1 <= 16'd0; env2 <= 16'd0; env3 <= 16'd0;
        env4 <= 16'd0; env5 <= 16'd0; env6 <= 16'd0; env7 <= 16'd0;
        O_bar0 <= 8'd0; O_bar1 <= 8'd0; O_bar2 <= 8'd0; O_bar3 <= 8'd0;
        O_bar4 <= 8'd0; O_bar5 <= 8'd0; O_bar6 <= 8'd0; O_bar7 <= 8'd0;
    end else if (I_audio_valid) begin
        // Cascaded cutoff: smaller >> => higher cutoff. Diff ≈ bandpass.
        lp0 <= lp0 + ((x - lp0) >>> 2);
        lp1 <= lp1 + ((x - lp1) >>> 3);
        lp2 <= lp2 + ((x - lp2) >>> 4);
        lp3 <= lp3 + ((x - lp3) >>> 5);
        lp4 <= lp4 + ((x - lp4) >>> 6);
        lp5 <= lp5 + ((x - lp5) >>> 7);
        lp6 <= lp6 + ((x - lp6) >>> 8);
        lp7 <= lp7 + ((x - lp7) >>> 9);
        lp8 <= lp8 + ((x - lp8) >>> 10);

        if (mag0 > env0) env0 <= mag0; else if (env0 > 16'd8) env0 <= env0 - (env0 >> 5); else env0 <= 16'd0;
        if (mag1 > env1) env1 <= mag1; else if (env1 > 16'd8) env1 <= env1 - (env1 >> 5); else env1 <= 16'd0;
        if (mag2 > env2) env2 <= mag2; else if (env2 > 16'd8) env2 <= env2 - (env2 >> 5); else env2 <= 16'd0;
        if (mag3 > env3) env3 <= mag3; else if (env3 > 16'd8) env3 <= env3 - (env3 >> 5); else env3 <= 16'd0;
        if (mag4 > env4) env4 <= mag4; else if (env4 > 16'd8) env4 <= env4 - (env4 >> 5); else env4 <= 16'd0;
        if (mag5 > env5) env5 <= mag5; else if (env5 > 16'd8) env5 <= env5 - (env5 >> 5); else env5 <= 16'd0;
        if (mag6 > env6) env6 <= mag6; else if (env6 > 16'd8) env6 <= env6 - (env6 >> 5); else env6 <= 16'd0;
        if (mag7 > env7) env7 <= mag7; else if (env7 > 16'd8) env7 <= env7 - (env7 >> 5); else env7 <= 16'd0;

        O_bar0 <= env_to_bar(env0);
        O_bar1 <= env_to_bar(env1);
        O_bar2 <= env_to_bar(env2);
        O_bar3 <= env_to_bar(env3);
        O_bar4 <= env_to_bar(env4);
        O_bar5 <= env_to_bar(env5);
        O_bar6 <= env_to_bar(env6);
        O_bar7 <= env_to_bar(env7);
    end
end
endmodule

// Live PCM envelope over the picture. One registered RGB stage, no extra clock.
module osd_audio_wave (
    input  wire               I_clk,
    input  wire               I_rst,
    input  wire               I_hs,
    input  wire               I_vs,
    input  wire               I_de,
    input  wire               I_enable,
    input  wire               I_audio_valid,
    input  wire signed [23:0] I_pcm,
    input  wire [23:0]        I_rgb,
    output reg  [23:0]        O_rgb
);
// Picture: x=0..519, y=132..423. Keep clear of the station bar and ticker.
localparam [9:0] X0 = 10'd4;
localparam [9:0] W = 10'd512;
localparam [9:0] BASE_Y = 10'd414;
localparam [5:0] MAX_HEIGHT = 6'd40;
localparam [23:0] FILL_RGB = 24'hF2BB6C;
localparam [23:0] LINE_RGB = 24'hD97C39;
localparam [21:0] AUDIO_TIMEOUT_LAST = 22'd2499999; // 100 ms at 25 MHz

reg de_d, vs_d;
reg [9:0] pix_x, pix_y;
reg [6:0] sample_count, write_ptr;
reg [7:0] peak, history_count;
reg [21:0] audio_idle;
reg [6:0] view_base;
reg [7:0] view_count;
reg snapshot_busy;
reg [6:0] snapshot_index;
reg [7:0] snapshot_count;
reg write_pending;
reg [6:0] pending_index;
reg [5:0] pending_height;
reg [5:0] history [0:127];
reg [5:0] frame_history [0:127];

wire frame_start = vs_d && !I_vs;
wire audio_expired = (audio_idle == AUDIO_TIMEOUT_LAST);
wire [23:0] pcm_bits = I_pcm[23:0];
wire [23:0] abs_pcm = pcm_bits[23] ? (~pcm_bits + 24'd1) : pcm_bits;
wire [7:0] magnitude = abs_pcm[23:16]; // includes -8388608 without overflow
wire [7:0] next_peak = (magnitude > peak) ? magnitude : peak;
// 40/128 scaling using shifts and adds, rounded to the nearest pixel.
wire [13:0] scaled_peak = {1'b0, next_peak, 5'b0} +
                          {3'b0, next_peak, 3'b0} + 14'd64;
wire [5:0] bin_height = scaled_peak[12:7];
wire commit_bin = I_enable && I_audio_valid && (sample_count == 7'd127);
// A 128-cycle copy fits in vertical blanking. Defer a coincident bin write so
// the old frame stays coherent; the final copy can share an NBA write edge.
wire can_write = !frame_start &&
                 (!snapshot_busy || snapshot_index == 7'd127);
wire flush_pending = I_enable && write_pending && can_write;
wire direct_write = commit_bin && can_write && !write_pending;

// No array reset: the validity counts mask unused cells. The display copy is
// updated only in vertical blanking, so every scan uses one coherent history.
always @(posedge I_clk) begin
    if (!I_rst) begin
        if (flush_pending)
            history[pending_index] <= pending_height;
        else if (direct_write)
            history[write_ptr] <= bin_height;
        if (snapshot_busy && I_enable)
            frame_history[snapshot_index] <= history[snapshot_index];
    end
end

wire in_plot = I_enable && I_de &&
    (pix_x >= X0) && (pix_x < X0 + W) &&
    (pix_y >= BASE_Y - {4'b0, MAX_HEIGHT}) &&
    (pix_y <= BASE_Y + 10'd1);
wire [9:0] offset_x = pix_x - X0;
wire [6:0] column = offset_x[8:2];
wire [6:0] next_column = (column == 7'd127) ? column : column + 7'd1;
wire [6:0] index0 = view_base + column; // modulo 128
wire [6:0] index1 = view_base + next_column;
wire [7:0] first_valid = 8'd128 - view_count;
wire [5:0] height0 = ({1'b0, column} < first_valid) ? 6'd0 : frame_history[index0];
wire [5:0] height1 = ({1'b0, next_column} < first_valid) ? 6'd0 : frame_history[index1];
wire [7:0] h0 = {2'b0, height0};
wire [7:0] h1 = {2'b0, height1};
reg [7:0] weighted_height;
always @(*) begin
    case (offset_x[1:0])
        2'd0: weighted_height = h0 << 2;
        2'd1: weighted_height = (h0 << 1) + h0 + h1 + 8'd2;
        2'd2: weighted_height = (h0 << 1) + (h1 << 1) + 8'd2;
        2'd3: weighted_height = h0 + (h1 << 1) + h1 + 8'd2;
        default: weighted_height = 8'd0;
    endcase
end
wire [5:0] height = weighted_height[7:2];
wire [9:0] top_y = BASE_Y - {4'b0, height};
wire on_base = in_plot && (pix_y >= BASE_Y);
wire on_outline = in_plot && (height != 6'd0) && (pix_y == top_y);
wire on_fill = in_plot && (pix_y >= top_y) && (pix_y < BASE_Y);

// Integer alpha blends, with rounding. All sums have room for white + white.
function [7:0] mix75;
    input [7:0] background, foreground;
    reg [9:0] sum;
    begin
        sum = {1'b0, foreground, 1'b0} + {2'b0, foreground} +
              {2'b0, background} + 10'd2;
        mix75 = sum[9:2];
    end
endfunction
function [7:0] mix875;
    input [7:0] background, foreground;
    reg [10:0] sum;
    begin
        sum = {1'b0, foreground, 2'b0} + {2'b0, foreground, 1'b0} +
              {3'b0, foreground} + {3'b0, background} + 11'd4;
        mix875 = sum[10:3];
    end
endfunction
wire [23:0] fill_rgb = {
    mix75(I_rgb[23:16], FILL_RGB[23:16]),
    mix75(I_rgb[15:8], FILL_RGB[15:8]),
    mix75(I_rgb[7:0], FILL_RGB[7:0])};
wire [23:0] outline_rgb = {
    mix875(I_rgb[23:16], LINE_RGB[23:16]),
    mix875(I_rgb[15:8], LINE_RGB[15:8]),
    mix875(I_rgb[7:0], LINE_RGB[7:0])};
wire [23:0] base_rgb = outline_rgb;

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        de_d <= 1'b0;
        vs_d <= 1'b0;
        pix_x <= 10'd0;
        pix_y <= 10'd0;
        sample_count <= 7'd0;
        write_ptr <= 7'd0;
        peak <= 8'd0;
        history_count <= 8'd0;
        audio_idle <= 22'd0;
        view_base <= 7'd0;
        view_count <= 8'd0;
        snapshot_busy <= 1'b0;
        snapshot_index <= 7'd0;
        snapshot_count <= 8'd0;
        write_pending <= 1'b0;
        pending_index <= 7'd0;
        pending_height <= 6'd0;
        O_rgb <= 24'd0;
    end else begin
        de_d <= I_de;
        vs_d <= I_vs;
        if (frame_start)
            pix_y <= 10'd0;
        else if (de_d && !I_de)
            pix_y <= pix_y + 10'd1;
        if (!I_de)
            pix_x <= 10'd0;
        else
            pix_x <= pix_x + 10'd1;

        if (!I_enable) begin
            sample_count <= 7'd0;
            write_ptr <= 7'd0;
            peak <= 8'd0;
            history_count <= 8'd0;
            audio_idle <= 22'd0;
            view_count <= 8'd0;
            view_base <= 7'd0;
            snapshot_busy <= 1'b0;
            snapshot_index <= 7'd0;
            snapshot_count <= 8'd0;
            write_pending <= 1'b0;
        end else begin
            if (flush_pending)
                write_pending <= 1'b0;
            if (commit_bin && !direct_write) begin
                write_pending <= 1'b1;
                pending_index <= write_ptr;
                pending_height <= bin_height;
            end
            if (I_audio_valid) begin
                audio_idle <= 22'd0;
                if (sample_count == 7'd127) begin
                    sample_count <= 7'd0;
                    peak <= 8'd0;
                    write_ptr <= write_ptr + 7'd1;
                    if (history_count != 8'd128)
                        history_count <= history_count + 8'd1;
                end else begin
                    sample_count <= sample_count + 7'd1;
                    peak <= next_peak;
                end
            end else if (!audio_expired) begin
                audio_idle <= audio_idle + 22'd1;
            end else begin
                // Flush stale peaks if the source stops issuing PCM samples.
                sample_count <= 7'd0;
                write_ptr <= 7'd0;
                peak <= 8'd0;
                history_count <= 8'd0;
                write_pending <= 1'b0;
            end
            if (frame_start) begin
                view_base <= write_ptr;
                view_count <= 8'd0;
                snapshot_count <= audio_expired ? 8'd0 : history_count;
                snapshot_index <= 7'd0;
                snapshot_busy <= 1'b1;
            end else if (snapshot_busy) begin
                if (snapshot_index == 7'd127) begin
                    snapshot_busy <= 1'b0;
                    view_count <= snapshot_count;
                end else begin
                    snapshot_index <= snapshot_index + 7'd1;
                end
            end
        end

        if (!I_de)
            O_rgb <= 24'd0;
        else if (on_base)
            O_rgb <= base_rgb;
        else if (on_outline)
            O_rgb <= outline_rgb;
        else if (on_fill)
            O_rgb <= fill_rgb;
        else
            O_rgb <= I_rgb;
    end
end
endmodule


// Right-rail PCM waveform (below TO GATE / MIN). OSD-only; does not touch SDRAM/steal.
module osd_audio_wave_rail (
    input  wire               I_clk,
    input  wire               I_rst,
    input  wire               I_hs,
    input  wire               I_vs,
    input  wire               I_de,
    input  wire               I_enable,
    input  wire               I_audio_valid,
    input  wire signed [23:0] I_pcm,
    input  wire [23:0]        I_rgb,
    output reg  [23:0]        O_rgb
);
localparam [9:0] X0 = 10'd528;
localparam [9:0] Y0 = 10'd320;
localparam [9:0] W  = 10'd96;
localparam [9:0] H  = 10'd88;
localparam [9:0] YC = 10'd364; // mid line
localparam [6:0] N  = 7'd96;

reg        de_d, vs_d;
reg [9:0]  pix_x, pix_y;
reg [2:0]  decim;
reg [6:0]  wptr;
// One asynchronous read and one synchronous write infer distributed RAM.
// Mask unwritten locations after reset, preserving the old cleared-array output.
(* ram_style="distributed" *) reg signed [7:0] wave [0:95];
reg [6:0] filled;
integer    ki;

wire in_box = I_enable && I_de &&
    (pix_x >= X0) && (pix_x < (X0 + W)) &&
    (pix_y >= Y0) && (pix_y < (Y0 + H));
wire [9:0] col10  = pix_x - X0;
wire [6:0] col    = col10[6:0];
wire [7:0] rd_sum = {1'b0, wptr} + 8'd1 + {1'b0, col};
wire [6:0] rd_idx = (rd_sum >= 8'd96) ? (rd_sum[6:0] - 7'd96) : rd_sum[6:0];
wire signed [7:0] samp = (rd_idx<filled) ? wave[rd_idx] : 8'sd0;
// |samp|>>2, clamp to 40 → ± ±40 around mid (no $signed / >>> for TD 6.2)
wire [7:0] samp_u = samp[7] ? (~samp + 8'd1) : samp;
wire [5:0] mag0   = samp_u[7:2];
wire [5:0] mag    = (mag0 > 6'd40) ? 6'd40 : mag0;
wire [9:0] y_line = samp[7] ? (YC + {4'b0, mag}) : (YC - {4'b0, mag});
wire       on_trace = in_box &&
    ((pix_y == y_line) ||
     (pix_y == (y_line + 10'd1)) ||
     ((y_line != Y0) && (pix_y == (y_line - 10'd1))));
wire on_mid = in_box && (pix_y == YC);
wire on_edge = in_box &&
    ((pix_x == X0) || (pix_x == (X0 + W - 10'd1)) ||
     (pix_y == Y0) || (pix_y == (Y0 + H - 10'd1)));

always @(posedge I_clk) begin
    if(!I_rst && I_enable && I_audio_valid && decim==3'd7)
        wave[wptr] <= I_pcm[23:16];
end

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        de_d   <= 1'b0;
        vs_d   <= 1'b0;
        pix_x  <= 10'd0;
        pix_y  <= 10'd0;
        decim  <= 3'd0;
        wptr   <= 7'd0;
        O_rgb  <= 24'd0;
        filled <= 7'd0;
    end else begin
        de_d <= I_de;
        vs_d <= I_vs;

        if (I_vs == 1'b0 && vs_d == 1'b1)
            pix_y <= 10'd0;
        else if (de_d == 1'b1 && I_de == 1'b0)
            pix_y <= pix_y + 10'd1;

        if (!I_de)
            pix_x <= 10'd0;
        else
            pix_x <= pix_x + 10'd1;

        // Capture from the same muted/volumed PCM that feeds HDMI (no mem path)
        if (I_enable && I_audio_valid) begin
            if (decim == 3'd7) begin
                decim        <= 3'd0;
                if(filled < N) filled <= filled + 7'd1;
                wptr         <= (wptr >= (N - 7'd1)) ? 7'd0 : (wptr + 7'd1);
            end else begin
                decim <= decim + 3'd1;
            end
        end

        if (!I_de)
            O_rgb <= 24'd0;
        else if (!I_enable)
            O_rgb <= I_rgb;
        else if (on_trace)
            O_rgb <= 24'h147C71; // metro green trace
        else if (on_mid)
            O_rgb <= 24'hC6D8D0;
        else if (on_edge)
            O_rgb <= 24'h3C6877;
        else if (in_box)
            O_rgb <= 24'hEDF6F0;
        else
            O_rgb <= I_rgb;
    end
end
endmodule


// Output raster refresh rate, measured from complete VS periods at 25 MHz.
// One registered RGB stage. Independent of picture changes / source file rate.
module osd_video_fps #(
    parameter [24:0] CLOCK_HZ = 25'd25000000
)(
    input wire I_clk, I_rst, I_hs, I_vs, I_de, I_enable,
    input wire [23:0] I_rgb,
    output reg [23:0] O_rgb,
    output wire [9:0] O_fps_tenths,
    output wire O_fps_valid
);
localparam [24:0] TIMEOUT_LAST = CLOCK_HZ - 25'd1;
localparam [31:0] NUMERATOR = {7'd0, CLOCK_HZ} * 32'd10;
reg vs_d, frame_seen;
reg [24:0] period_count;
reg [9:0] fps_tenths;
reg fps_valid, div_busy;
localparam NUM_BITS=$clog2(NUMERATOR+1);
reg [$clog2(NUM_BITS+1)-1:0] cycles_left;
reg [24:0] divisor;
reg [25:0] remainder;
reg [NUM_BITS-1:0] dividend;
reg [9:0] quotient;
reg quotient_overflow;
wire frame_start = vs_d && !I_vs;
wire timed_out = (period_count == TIMEOUT_LAST);
wire [25:0] trial = {remainder[24:0], dividend[NUM_BITS-1]};
wire take_bit = (trial >= {1'b0, divisor});
wire [25:0] next_remainder = take_bit ? trial - {1'b0, divisor} : trial;
wire [9:0] next_quotient = {quotient[8:0], take_bit};

// A small serial divider runs in vertical blanking; no combinational divider
// for the variable VS period, and no additional clock domain.
always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        period_count <= 25'd0;
        frame_seen <= 1'b0;
        fps_tenths <= 10'd0;
        fps_valid <= 1'b0;
        div_busy <= 1'b0;
        cycles_left <= 6'd0;
        divisor <= 25'd1;
        remainder <= 26'd0;
        dividend <= 32'd0;
        quotient <= 10'd0;quotient_overflow<=0;
    end else begin
        if (!timed_out)
            period_count <= period_count + 25'd1;
        if (div_busy) begin
            remainder <= next_remainder;
            dividend <= {dividend[NUM_BITS-2:0], 1'b0};
            quotient <= next_quotient;
            quotient_overflow<=quotient_overflow || quotient[9];
            cycles_left <= cycles_left - 6'd1;
            if (cycles_left == 6'd1) begin
                fps_tenths <= (quotient_overflow || quotient[9] || next_quotient > 10'd999) ? 10'd999 : next_quotient[9:0];
                fps_valid <= 1'b1;
                div_busy <= 1'b0;
            end
        end
        if (timed_out) begin
            // Lose the reference after one second without a VS edge.
            frame_seen <= 1'b0;
            fps_tenths <= 10'd0;
            fps_valid <= 1'b1;
            div_busy <= 1'b0;
        end
        if (frame_start) begin
            period_count <= 25'd0;
            frame_seen <= 1'b1;
            if (frame_seen && !timed_out && !div_busy) begin
                divisor <= period_count + 25'd1;
                dividend <= NUMERATOR;
                remainder <= 26'd0;
                quotient <= 10'd0;quotient_overflow<=0;
                cycles_left <= NUM_BITS;
                div_busy <= 1'b1;
            end
        end
    end
end

assign O_fps_tenths=fps_tenths;
assign O_fps_valid=fps_valid;
always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin vs_d<=0; O_rgb<=0; end
    else begin vs_d<=I_vs; O_rgb<=I_de ? I_rgb : 24'd0; end
end
endmodule


// Slice 7: block-letter EXIT + 7-seg style latency (no nested font functions)
module osd_emerg_banner (
    input  wire        I_clk,
    input  wire        I_rst,
    input  wire        I_hs,
    input  wire        I_vs,
    input  wire        I_de,
    input  wire        I_enable,
    input  wire [7:0]  I_lat_ms,
    input  wire [23:0] I_rgb,
    output reg  [23:0] O_rgb
);
localparam [9:0] Y0 = 10'd0;
localparam [9:0] Y1 = 10'd39;

reg        de_d, vs_d;
reg [9:0]  pix_x, pix_y;
reg [7:0]  lat_hold;

wire in_banner = I_enable && I_de && (pix_y >= Y0) && (pix_y <= Y1);
wire [9:0] lx = pix_x;
wire [9:0] ly = pix_y - Y0;
wire [9:0] ey = ly - 10'd6;
wire in_text_y = (ly >= 10'd6) && (ly <= 10'd33);

wire in_e = (lx >= 10'd16)  && (lx <= 10'd39);
wire in_x = (lx >= 10'd48)  && (lx <= 10'd71);
wire in_i = (lx >= 10'd80)  && (lx <= 10'd103);
wire in_t = (lx >= 10'd112) && (lx <= 10'd135);
wire [9:0] ex = lx - 10'd16;
wire [9:0] xx = lx - 10'd48;
wire [9:0] ix = lx - 10'd80;
wire [9:0] tx = lx - 10'd112;

wire bar_h = in_text_y && ((ey <= 10'd4) || ((ey >= 10'd12) && (ey <= 10'd16)) || (ey >= 10'd24));
wire bar_v = in_text_y && (ex <= 10'd5);
wire pix_e = in_e && (bar_v || bar_h);

wire pix_xlet = in_x && in_text_y && (
    ((xx + 10'd4 >= ey) && (xx <= ey + 10'd4)) ||
    ((xx + ey >= 10'd20) && (xx + ey <= 10'd28))
);

wire pix_i = in_i && in_text_y && (
    (ey <= 10'd4) || (ey >= 10'd24) ||
    ((ix >= 10'd9) && (ix <= 10'd14))
);

wire pix_t = in_t && in_text_y && (
    (ey <= 10'd4) ||
    ((tx >= 10'd9) && (tx <= 10'd14))
);

wire [3:0] lat_tens =
    (lat_hold >= 8'd90) ? 4'd9 : (lat_hold >= 8'd80) ? 4'd8 :
    (lat_hold >= 8'd70) ? 4'd7 : (lat_hold >= 8'd60) ? 4'd6 :
    (lat_hold >= 8'd50) ? 4'd5 : (lat_hold >= 8'd40) ? 4'd4 :
    (lat_hold >= 8'd30) ? 4'd3 : (lat_hold >= 8'd20) ? 4'd2 :
    (lat_hold >= 8'd10) ? 4'd1 : 4'd0;
wire [7:0] lat_ones_w = lat_hold - {1'b0, lat_tens, 3'b0} - {5'b0, lat_tens, 1'b0};
wire [3:0] lat_ones = lat_ones_w[3:0];

function pix_seg7;
    input [9:0] dx;
    input [9:0] dy;
    input [3:0] d;
    input [9:0] w;
    reg a,b,c,bot,e,f,g,s;
    begin
        a = (dy <= 10'd3)  && (dx >= 10'd3) && (dx <= 10'd20);
        b = (dx >= 10'd18) && (dy >= 10'd3)  && (dy <= 10'd12);
        c = (dx >= 10'd18) && (dy >= 10'd14) && (dy <= 10'd24);
        bot = (dy >= 10'd22) && (dy <= 10'd26) && (dx >= 10'd3) && (dx <= 10'd20);
        e = (dx <= 10'd5)  && (dy >= 10'd14) && (dy <= 10'd24);
        f = (dx <= 10'd5)  && (dy >= 10'd3)  && (dy <= 10'd12);
        g = (dy >= 10'd11) && (dy <= 10'd15) && (dx >= 10'd3) && (dx <= 10'd20);
        case (d)
            4'd0: s = a|b|c|bot|e|f;
            4'd1: s = b|c;
            4'd2: s = a|b|g|e|bot;
            4'd3: s = a|b|c|g|bot;
            4'd4: s = f|g|b|c;
            4'd5: s = a|f|g|c|bot;
            4'd6: s = a|f|g|e|c|bot;
            4'd7: s = a|b|c;
            4'd8: s = a|b|c|e|f|g|bot;
            default: s = a|b|c|f|g|bot;
        endcase
        pix_seg7 = s;
    end
endfunction

wire in_d0 = (lx >= 10'd520) && (lx <= 10'd543) && in_text_y;
wire in_d1 = (lx >= 10'd552) && (lx <= 10'd575) && in_text_y;
wire pix_d0 = in_d0 && pix_seg7(lx - 10'd520, ey, lat_tens, 10'd24);
wire pix_d1 = in_d1 && pix_seg7(lx - 10'd552, ey, lat_ones, 10'd24);

wire on_pix = in_banner && (pix_e | pix_xlet | pix_i | pix_t | pix_d0 | pix_d1);

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        de_d <= 1'b0; vs_d <= 1'b0;
        pix_x <= 10'd0; pix_y <= 10'd0;
        lat_hold <= 8'd0;
        O_rgb <= 24'd0;
    end else begin
        de_d <= I_de;
        vs_d <= I_vs;
        if (I_vs == 1'b0 && vs_d == 1'b1) begin
            pix_y <= 10'd0;
            lat_hold <= I_lat_ms;
        end else if (de_d == 1'b1 && I_de == 1'b0) begin
            pix_y <= pix_y + 10'd1;
        end
        if (!I_de) pix_x <= 10'd0;
        else       pix_x <= pix_x + 10'd1;

        if (!I_de)
            O_rgb <= 24'd0;
        else if (on_pix)
            O_rgb <= 24'hFFFFFF;
        else if (in_banner)
            O_rgb <= 24'hAA0000;
        else
            O_rgb <= I_rgb;
    end
end
endmodule

// Slice 9: 115200 8N1. TX "I0 P1 V8 L02 E0 R+\n" ; RX + - 0-8 ?
module uart_console #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer BAUD   = 115200
)(
    input  wire       I_clk,
    input  wire       I_rst,
    input  wire       I_rx,
    output reg        O_tx,
    input  wire [1:0] I_img,
    input  wire [3:0] I_period,
    input  wire [3:0] I_vol,
    input  wire [7:0] I_lat_ms,
    input  wire       I_emerg,
    input  wire       I_frozen,
    input  wire [2:0] I_wav_n,
    input  wire [3:0] I_dbg,
    input  wire [2:0] I_bmp_n,
    input  wire [3:0] I_st,
    input  wire [1:0] I_cstat,   // Buffer C: 0 idle, 1 ready, 2 fail, 3 loading
    input  wire [23:0] I_jedec,
    input  wire [11:0] I_used,
    input wire [15:0] I_line_miss,
    input  wire [7:0] I_under,
    output reg  [3:0] O_vol,
    output reg        O_vol_stb,
    output reg        O_warn
);
localparam integer DIV      = CLK_HZ / BAUD;
localparam integer HALF_DIV = DIV / 2;
localparam integer MSG_LEN  = 51;
localparam integer PERIOD   = CLK_HZ / 4;

reg [1:0]  rx_sync;
reg        rx_bit;
reg [15:0] rx_div;
reg [3:0]  rx_bitn;
reg [7:0]  rx_sh;
reg        rx_busy;
reg [7:0]  rx_byte;
reg        rx_done;

reg [15:0] tx_div;
reg [3:0]  tx_bitn;
reg [9:0]  tx_sh;
reg        tx_busy;
reg [5:0]  tx_idx;
reg [31:0] tx_wait;
reg        tx_kick;

reg [7:0]  m1, m2, m3, m4, m5, m6, m7, m8, m9, m10, m11, m12, m13, m14, m15, m16, m17, m18, m19, m20, m21, m22, m23;
reg [7:0]  m24, m25, m26, m27, m28, m29, m30, m31, m32, m33, m34;
reg [7:0]  m35, m36, m37, m38, m39, m40, m41, m42, m43, m44;
reg [7:0] m45,m46,m47,m48,m49,m50;
reg [7:0] last_rx;
reg [3:0]  last_st;
reg [3:0]  last_dbg;

wire [3:0] lat_t =
    (I_lat_ms >= 8'd90) ? 4'd9 : (I_lat_ms >= 8'd80) ? 4'd8 :
    (I_lat_ms >= 8'd70) ? 4'd7 : (I_lat_ms >= 8'd60) ? 4'd6 :
    (I_lat_ms >= 8'd50) ? 4'd5 : (I_lat_ms >= 8'd40) ? 4'd4 :
    (I_lat_ms >= 8'd30) ? 4'd3 : (I_lat_ms >= 8'd20) ? 4'd2 :
    (I_lat_ms >= 8'd10) ? 4'd1 : 4'd0;
wire [7:0] dbg_hex =
    (I_dbg < 4'd10) ? (8'h30 + {4'b0, I_dbg}) : (8'h41 + {4'b0, I_dbg} - 8'd10);
wire [7:0] st_hex =
    (I_st < 4'd10) ? (8'h30 + {4'b0, I_st}) : (8'h41 + {4'b0, I_st} - 8'd10);
wire [7:0] c_hex = 8'h30 + {6'b0, I_cstat};
wire [7:0] jn0 = (I_jedec[23:20] < 4'd10) ? (8'h30 + {4'b0, I_jedec[23:20]}) : (8'h41 + {4'b0, I_jedec[23:20]} - 8'd10);
wire [7:0] jn1 = (I_jedec[19:16] < 4'd10) ? (8'h30 + {4'b0, I_jedec[19:16]}) : (8'h41 + {4'b0, I_jedec[19:16]} - 8'd10);
wire [7:0] jn2 = (I_jedec[15:12] < 4'd10) ? (8'h30 + {4'b0, I_jedec[15:12]}) : (8'h41 + {4'b0, I_jedec[15:12]} - 8'd10);
wire [7:0] jn3 = (I_jedec[11:8]  < 4'd10) ? (8'h30 + {4'b0, I_jedec[11:8]})  : (8'h41 + {4'b0, I_jedec[11:8]}  - 8'd10);
wire [7:0] jn4 = (I_jedec[7:4]   < 4'd10) ? (8'h30 + {4'b0, I_jedec[7:4]})   : (8'h41 + {4'b0, I_jedec[7:4]}   - 8'd10);
wire [7:0] jn5 = (I_jedec[3:0]   < 4'd10) ? (8'h30 + {4'b0, I_jedec[3:0]})   : (8'h41 + {4'b0, I_jedec[3:0]}   - 8'd10);
wire [7:0] un2 = (I_used[11:8] < 4'd10) ? (8'h30 + {4'b0, I_used[11:8]}) : (8'h41 + {4'b0, I_used[11:8]} - 8'd10);
wire [7:0] un1 = (I_used[7:4]  < 4'd10) ? (8'h30 + {4'b0, I_used[7:4]})  : (8'h41 + {4'b0, I_used[7:4]}  - 8'd10);
wire [7:0] un0 = (I_used[3:0]  < 4'd10) ? (8'h30 + {4'b0, I_used[3:0]})  : (8'h41 + {4'b0, I_used[3:0]}  - 8'd10);
wire [7:0] nn1 = (I_under[7:4] < 4'd10) ? (8'h30 + {4'b0, I_under[7:4]}) : (8'h41 + {4'b0, I_under[7:4]} - 8'd10);
wire [7:0] nn0 = (I_under[3:0] < 4'd10) ? (8'h30 + {4'b0, I_under[3:0]}) : (8'h41 + {4'b0, I_under[3:0]} - 8'd10);

function [7:0] hex_byte;
input [3:0] nibble;
begin hex_byte=nibble<10?8'h30+nibble:8'h41+nibble-10;end
endfunction
wire [7:0] tx_byte_next =
    (tx_idx == 5'd0)  ? m1  :
    (tx_idx == 5'd1)  ? m2  :
    (tx_idx == 5'd2)  ? m3  :
    (tx_idx == 5'd3)  ? m4  :
    (tx_idx == 5'd4)  ? m5  :
    (tx_idx == 5'd5)  ? m6  :
    (tx_idx == 5'd6)  ? m7  :
    (tx_idx == 5'd7)  ? m8  :
    (tx_idx == 5'd8)  ? m9  :
    (tx_idx == 5'd9)  ? m10 :
    (tx_idx == 5'd10) ? m11 :
    (tx_idx == 5'd11) ? m12 :
    (tx_idx == 5'd12) ? m13 :
    (tx_idx == 5'd13) ? m14 :
    (tx_idx == 5'd14) ? m15 :
    (tx_idx == 5'd15) ? m16 :
    (tx_idx == 5'd16) ? m17 :
    (tx_idx == 5'd17) ? m18 :
    (tx_idx == 5'd18) ? m19 :
    (tx_idx == 5'd19) ? m20 :
    (tx_idx == 6'd20) ? m21 :
    (tx_idx == 6'd21) ? m22 :
    (tx_idx == 6'd22) ? m23 :
    (tx_idx == 6'd23) ? m24 :
    (tx_idx == 6'd24) ? m25 :
    (tx_idx == 6'd25) ? m26 :
    (tx_idx == 6'd26) ? m27 :
    (tx_idx == 6'd27) ? m28 :
    (tx_idx == 6'd28) ? m29 :
    (tx_idx == 6'd29) ? m30 :
    (tx_idx == 6'd30) ? m31 :
    (tx_idx == 6'd31) ? m32 :
    (tx_idx == 6'd32) ? m33 :
    (tx_idx == 6'd33) ? m34 :
    (tx_idx == 6'd34) ? m35 :
    (tx_idx == 6'd35) ? m36 :
    (tx_idx == 6'd36) ? m37 :
    (tx_idx == 6'd37) ? m38 :
    (tx_idx == 6'd38) ? m39 :
    (tx_idx == 6'd39) ? m40 :
    (tx_idx == 6'd40) ? m41 :
    (tx_idx == 6'd41) ? m42 :
    (tx_idx == 6'd42) ? m43 :
    (tx_idx == 6'd43) ? m44 :
    (tx_idx == 6'd44) ? m45 :
    (tx_idx == 6'd45) ? m46 :
    (tx_idx == 6'd46) ? m47 :
    (tx_idx == 6'd47) ? m48 :
    (tx_idx == 6'd48) ? m49 : m50;

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        rx_sync <= 2'b11;
        rx_bit <= 1'b1;
        rx_div <= 16'd0;
        rx_bitn <= 4'd0;
        rx_sh <= 8'd0;
        rx_busy <= 1'b0;
        rx_byte <= 8'd0;
        rx_done <= 1'b0;
        O_tx <= 1'b1;
        tx_div <= 16'd0;
        tx_bitn <= 4'd0;
        tx_sh <= 10'h3FF;
        tx_busy <= 1'b0;
        tx_idx <= 6'd0;
        tx_wait <= 32'd0;
        tx_kick <= 1'b0;
        O_vol <= 4'd8;
        O_vol_stb <= 1'b0;
        O_warn <= 1'b0;
        m1 <= "0"; m2 <= " "; m3 <= "P"; m4 <= "1"; m5 <= " ";
        m6 <= "V"; m7 <= "8"; m8 <= " "; m9 <= "B"; m10 <= "0";
        m11 <= " "; m12 <= "W"; m13 <= "0"; m14 <= " "; m15 <= "D";
        m16 <= "0"; m17 <= " "; m18 <= "S"; m19 <= "0";         m20 <= " ";
        m21 <= "E"; m22 <= "0"; m23 <= " ";
        m24 <= "C"; m25 <= "0"; m26 <= " "; m27 <= "J";
        m28 <= "0"; m29 <= "0"; m30 <= "0"; m31 <= "0"; m32 <= "0"; m33 <= "0";
        m34 <= " "; m35 <= "U"; m36 <= "0"; m37 <= "0"; m38 <= "0";
        m39 <= " "; m40 <= "N"; m41 <= "0"; m42 <= "0";
        m43 <= 8'h0D; m44 <= 8'h0A;
        last_rx <= ".";
        last_st <= 4'd0;
        last_dbg <= 4'd0;
    end else begin
        rx_sync <= {rx_sync[0], I_rx};
        rx_bit  <= rx_sync[1];
        rx_done <= 1'b0;
        O_vol_stb <= 1'b0;
        tx_kick <= 1'b0;

        if (!rx_busy) begin
            if (rx_bit == 1'b0) begin
                rx_busy <= 1'b1;
                rx_div  <= HALF_DIV;
                rx_bitn <= 4'd0;
            end
        end else if (rx_div == 16'd0) begin
            rx_div <= DIV - 1;
            if (rx_bitn == 4'd0) begin
                if (rx_bit != 1'b0)
                    rx_busy <= 1'b0;
                else
                    rx_bitn <= 4'd1;
            end else if (rx_bitn <= 4'd8) begin
                rx_sh   <= {rx_bit, rx_sh[7:1]};
                rx_bitn <= rx_bitn + 4'd1;
            end else begin
                rx_busy <= 1'b0;
                if (rx_bit == 1'b1) begin
                    rx_byte <= rx_sh;
                    rx_done <= 1'b1;
                end
            end
        end else begin
            rx_div <= rx_div - 16'd1;
        end

        if (rx_done) begin
            last_rx <= rx_byte;
            if ((rx_byte == "+") || (rx_byte == "=")) begin
                if (I_vol < 4'd8) begin
                    O_vol <= I_vol + 4'd1;
                    O_vol_stb <= 1'b1;
                end
            end else if ((rx_byte == "-") && (I_vol > 4'd0)) begin
                O_vol <= I_vol - 4'd1;
                O_vol_stb <= 1'b1;
            end else if ((rx_byte >= "0") && (rx_byte <= "8")) begin
                O_vol <= rx_byte[3:0];
                O_vol_stb <= 1'b1;
            end else if (rx_byte == "?") begin
                tx_kick <= 1'b1;
            end else if ((rx_byte == "W") || (rx_byte == "w")) begin
                O_warn <= ~O_warn;
            end
        end

        if ((I_st != last_st) || (I_dbg != last_dbg)) begin
            last_st  <= I_st;
            last_dbg <= I_dbg;
            tx_kick  <= 1'b1;
        end

        if (tx_wait >= (PERIOD - 1)) begin
            tx_wait <= 32'd0;
            tx_kick <= 1'b1;
        end else begin
            tx_wait <= tx_wait + 32'd1;
        end

        if (tx_kick && !tx_busy) begin
            m1  <= 8'h30 + {6'b0, I_img};
            m2  <= " ";
            m3  <= "P";
            m4  <= 8'h30 + {4'b0, I_period};
            m5  <= " ";
            m6  <= "V";
            m7  <= 8'h30 + {4'b0, I_vol};
            m8  <= " ";
            m9  <= "B";
            m10 <= 8'h30 + {5'b0, I_bmp_n};
            m11 <= " ";
            m12 <= "W";
            m13 <= 8'h30 + {5'b0, I_wav_n};
            m14 <= " ";
            m15 <= "D";
            m16 <= dbg_hex;
            m17 <= " ";
            m18 <= "S";
            m19 <= st_hex;
            m20 <= " ";
            m21 <= "E";
            m22 <= I_emerg ? "1" : (I_frozen ? "F" : "0");
            m23 <= " ";
            m24 <= "C";
            m25 <= c_hex;
            m26 <= " ";
            m27 <= "J";
            m28 <= jn0;
            m29 <= jn1;
            m30 <= jn2;
            m31 <= jn3;
            m32 <= jn4;
            m33 <= jn5;
            m34 <= " ";
            m35 <= "U";
            m36 <= un2;
            m37 <= un1;
            m38 <= un0;
            m39 <= " ";
            m40 <= "N";
            m41 <= nn1;
            m42 <= nn0;
            m43 <= " ";m44 <= "L";
            m45<=hex_byte(I_line_miss[15:12]);m46<=hex_byte(I_line_miss[11:8]);
            m47<=hex_byte(I_line_miss[7:4]);m48<=hex_byte(I_line_miss[3:0]);
            m49<=8'h0D;m50<=8'h0A;
            tx_busy <= 1'b1;
            tx_idx  <= 6'd0;
            tx_bitn <= 4'd0;
            tx_div  <= DIV - 1;
            tx_sh   <= {1'b1, "I", 1'b0};
            O_tx    <= 1'b0;
        end else if (tx_busy) begin
            if (tx_div == 16'd0) begin
                tx_div <= DIV - 1;
                if (tx_bitn == 4'd9) begin
                    if (tx_idx == (MSG_LEN - 1)) begin
                        tx_busy <= 1'b0;
                        tx_idx  <= 6'd0;
                        O_tx    <= 1'b1;
                    end else begin
                        tx_idx  <= tx_idx + 6'd1;
                        tx_bitn <= 4'd0;
                        tx_sh   <= {1'b1, tx_byte_next, 1'b0};
                        O_tx    <= 1'b0;
                    end
                end else begin
                    tx_bitn <= tx_bitn + 4'd1;
                    O_tx    <= tx_sh[1];
                    tx_sh   <= {1'b1, tx_sh[9:1]};
                end
            end else begin
                tx_div <= tx_div - 16'd1;
            end
        end
    end
end
endmodule

// Lv.2: onboard piezo H11 + HDMI siren source. Prescaled 9-bit continuous sweep (520Hz <-> 1180Hz).
module emerg_buzzer (
    input  wire I_clk,
    input  wire I_rst,
    input  wire I_enable,
    output reg  O_spk
);
localparam [8:0] PER_MIN = 9'd166; // ~1176 Hz @ (50MHz/256)
localparam [8:0] PER_MAX = 9'd375; // ~520 Hz  @ (50MHz/256)

reg [7:0] pre_div;
reg [8:0] tone_div;
reg [8:0] tone_per;
reg [8:0] sweep_div;
reg       sweep_dir;

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst || !I_enable) begin
        pre_div   <= 8'd0;
        tone_div  <= 9'd0;
        tone_per  <= PER_MAX;
        sweep_div <= 9'd0;
        sweep_dir <= 1'b0;
        O_spk     <= 1'b0;
    end else begin
        pre_div <= pre_div + 8'd1;
        if (&pre_div) begin
            if (sweep_div >= 9'd511) begin
                sweep_div <= 9'd0;
                if (!sweep_dir) begin
                    if (tone_per <= PER_MIN) sweep_dir <= 1'b1;
                    else tone_per <= tone_per - 9'd1;
                end else begin
                    if (tone_per >= PER_MAX) sweep_dir <= 1'b0;
                    else tone_per <= tone_per + 9'd1;
                end
            end else begin
                sweep_div <= sweep_div + 9'd1;
            end

            if (tone_div >= tone_per)
                tone_div <= 9'd0;
            else
                tone_div <= tone_div + 9'd1;

            O_spk <= (tone_div < {2'b00, tone_per[8:2]});
        end
    end
end
endmodule

// EC11 rotary: CLK/DT quadrature → vol pulses; SW active-low level (debounced).
// Wiring: GND, VCC=3.3V, CLK→H13, DT→H14, SW→J14. Clockwise = volume up.
// Sample on a ~50 us tick so we do not put 16/20-bit adders on the 50 MHz clk net.
module ec11_encoder #(
    parameter integer CLK_HZ = 50_000_000
)(
    input  wire I_clk,
    input  wire I_rst,
    input  wire I_enc_clk,
    input  wire I_enc_dt,
    input  wire I_enc_sw,
    output reg  O_vol_up,
    output reg  O_vol_dn,
    output wire O_sw_n
);
localparam integer TICK_DIV = CLK_HZ / 20000; // 50 us
localparam [8:0]   SW_TICKS = 9'd400;         // 20 ms

reg [1:0]  clk_sync, dt_sync, sw_sync;
reg        clk_db, dt_db, sw_db;
reg [11:0] prescale;
reg        tick;
reg [8:0]  sw_hold;
reg [1:0]  gray_prev;
reg signed [3:0] step_acc;

assign O_sw_n = sw_db;

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        clk_sync  <= 2'b11;
        dt_sync   <= 2'b11;
        sw_sync   <= 2'b11;
        clk_db    <= 1'b1;
        dt_db     <= 1'b1;
        sw_db     <= 1'b1;
        prescale  <= 12'd0;
        tick      <= 1'b0;
        sw_hold   <= 9'd0;
        gray_prev <= 2'b11;
        step_acc  <= 4'sd0;
        O_vol_up  <= 1'b0;
        O_vol_dn  <= 1'b0;
    end else begin
        clk_sync <= {clk_sync[0], I_enc_clk};
        dt_sync  <= {dt_sync[0],  I_enc_dt};
        sw_sync  <= {sw_sync[0],  I_enc_sw};

        O_vol_up <= 1'b0;
        O_vol_dn <= 1'b0;

        if (prescale >= (TICK_DIV[11:0] - 12'd1)) begin
            prescale <= 12'd0;
            tick     <= 1'b1;
        end else begin
            prescale <= prescale + 12'd1;
            tick     <= 1'b0;
        end

        if (tick) begin
            clk_db <= clk_sync[1];
            dt_db  <= dt_sync[1];

            if (sw_sync[1] == sw_db)
                sw_hold <= 9'd0;
            else if (sw_hold >= (SW_TICKS - 9'd1)) begin
                sw_db   <= sw_sync[1];
                sw_hold <= 9'd0;
            end else
                sw_hold <= sw_hold + 9'd1;

            if ({clk_sync[1], dt_sync[1]} != gray_prev) begin
                case ({gray_prev, clk_sync[1], dt_sync[1]})
                    4'b0010, 4'b1011, 4'b1101, 4'b0100: begin
                        if (step_acc == 4'sd3) begin
                            step_acc <= 4'sd0;
                            O_vol_up <= 1'b1;
                        end else
                            step_acc <= step_acc + 4'sd1;
                    end
                    4'b0001, 4'b0111, 4'b1110, 4'b1000: begin
                        if (step_acc == -4'sd3) begin
                            step_acc <= 4'sd0;
                            O_vol_dn <= 1'b1;
                        end else
                            step_acc <= step_acc - 4'sd1;
                    end
                    default: step_acc <= 4'sd0;
                endcase
                gray_prev <= {clk_sync[1], dt_sync[1]};
            end
        end
    end
end
endmodule


// V6 physical controls. All pulse outputs belong to the 100 MHz SD domain.
module exhibit_controls #(
    parameter integer CLK_HZ=100000000,
    parameter integer DEBOUNCE=2000000,
    parameter integer LONG_TIME=100000000,
    parameter integer DOUBLE_TIME=CLK_HZ/3
)(
    input wire clk,rst,
    input wire [3:0] sw,
    input wire k2,k3,k4,enc_sw,enc_up,enc_dn,
    input wire emerg,
    input wire cfg_ack,video_cfg_ack,
    output reg video_cfg_req,output reg [92:0] video_cfg_mailbox,
    output reg cfg_req,
    output reg [92:0] cfg_mailbox,
    output reg next_p,prev_p,auto_p,emerg_p,
    input wire [4:0] uart_volume,
    output reg [3:0] volume,
    output wire volume_gesture,
    output reg media_focus,output reg video_started
);
reg [3:0] sw_s0,sw_s1,sw_candidate,mode;
reg [3:0] b_s0,b_s1,buttons;
localparam DB_BITS=$clog2(DEBOUNCE+1);
localparam HOLD_BITS=$clog2(LONG_TIME+1);
localparam CLICK_BITS=$clog2(DOUBLE_TIME+1);
reg [DB_BITS-1:0] sw_db;
reg [DB_BITS-1:0] b_db0,b_db1,b_db2,b_db3;
reg [HOLD_BITS-1:0] hold3,holds;
reg long3,longs;
reg used2,used4,useds;
reg uart_volume_seen;
reg click_pending;
reg [CLICK_BITS-1:0] click_wait;
reg [3:0] click_mode;
assign volume_gesture=useds;
reg signed [7:0] brightness[0:1];
reg [7:0] contrast[0:1],saturation[0:1],zoom[0:1];
reg signed [4:0] tone[0:1];
reg signed [9:0] pan_x[0:1],pan_y[0:1];
reg fit[0:1];
reg [6:0] opacity[0:1];
reg blend[0:1];
reg [5:0] speed[0:1];
reg [1:0] position[0:1],transition[0:1],direction[0:1];
reg [3:0] duration[0:1],period[0:1];
reg [1:0] ack_s,video_ack_s;
localparam [85:0] DEFAULT_PARAMS={8'd0,8'd100,5'd0,8'd100,8'd100,10'd0,10'd0,1'b0,7'd50,1'b0,6'd30,2'd1,2'd1,4'd4,4'd3,2'd0};
reg bank_emergency;
wire [85:0] active_params={brightness[emerg],contrast[emerg],tone[emerg],saturation[emerg],zoom[emerg],pan_x[emerg],pan_y[emerg],fit[emerg],opacity[emerg],blend[emerg],speed[emerg],position[emerg],transition[emerg],duration[emerg],period[emerg],direction[emerg]};
wire compare = !buttons[0] && !buttons[2];
wire [1:0] tuning=compare ? 2'd0 : !buttons[0] ? 2'd1 : !buttons[2] ? 2'd2 : 2'd0;
// Dedicated page registers avoid swapping and re-multiplexing 172 bits.
wire [85:0] normal_params={brightness[0],contrast[0],tone[0],saturation[0],zoom[0],pan_x[0],pan_y[0],fit[0],opacity[0],blend[0],speed[0],position[0],transition[0],duration[0],period[0],direction[0]};
wire [85:0] emergency_params={brightness[1],contrast[1],tone[1],saturation[1],zoom[1],pan_x[1],pan_y[1],fit[1],opacity[1],blend[1],speed[1],position[1],transition[1],duration[1],period[1],direction[1]};
wire [92:0] current_cfg={mode,normal_params,(emerg?1'b0:compare),(emerg?2'd0:tuning)};
wire [92:0] current_video_cfg={mode,emergency_params,(emerg?compare:1'b0),(emerg?tuning:2'd0)};
reg signed [11:0] step,val;
integer j;
reg [3:0] release_p;
task defaults;
    input [1:0] group_id;
    begin
        case(group_id)
            0: begin brightness[emerg]<=0;contrast[emerg]<=100;tone[emerg]<=0;saturation[emerg]<=100;end
            1: begin zoom[emerg]<=100;pan_x[emerg]<=0;pan_y[emerg]<=0;fit[emerg]<=0;end
            2: begin opacity[emerg]<=50;blend[emerg]<=0;speed[emerg]<=30;position[emerg]<=1;end
            3: begin transition[emerg]<=1;duration[emerg]<=4;period[emerg]<=3;direction[emerg]<=0;end
        endcase
    end
endtask
task selected_default;
    begin
        case(mode)
            0:brightness[emerg]<=0; 1:contrast[emerg]<=100; 2:tone[emerg]<=0; 3:saturation[emerg]<=100;
            4:zoom[emerg]<=100; 5:pan_x[emerg]<=0; 6:pan_y[emerg]<=0; 7:fit[emerg]<=0;
            8:opacity[emerg]<=50; 9:blend[emerg]<=0; 10:speed[emerg]<=30; 11:position[emerg]<=1;
            12:transition[emerg]<=1; 13:duration[emerg]<=4; 14:period[emerg]<=3; 15:direction[emerg]<=0;
        endcase
    end
endtask
always @(posedge clk or posedge rst) begin
    if(rst) begin
        sw_s0<=0;sw_s1<=0;sw_candidate<=0;mode<=0;sw_db<=0;
        b_s0<=15;b_s1<=15;buttons<=15;
        b_db0<=0;b_db1<=0;b_db2<=0;b_db3<=0;
        hold3<=0;holds<=0;long3<=0;longs<=0;used2<=0;used4<=0;useds<=0;
        volume<=8;uart_volume_seen<=0;
        media_focus<=0;video_started<=0;click_pending<=0;click_wait<=0;click_mode<=0;
        next_p<=0;prev_p<=0;auto_p<=0;emerg_p<=0;
        for(j=0;j<2;j=j+1)begin
        brightness[j]<=0;contrast[j]<=100;tone[j]<=0;saturation[j]<=100;
        zoom[j]<=100;pan_x[j]<=0;pan_y[j]<=0;fit[j]<=0;
        opacity[j]<=50;blend[j]<=0;speed[j]<=30;position[j]<=1;
        transition[j]<=1;duration[j]<=4;period[j]<=3;direction[j]<=0;
        end
        ack_s<=0;cfg_req<=0;cfg_mailbox<=0;video_ack_s<=0;video_cfg_req<=0;video_cfg_mailbox<=0;bank_emergency<=0;
    end else begin
        sw_s0<=sw;sw_s1<=sw_s0;
        b_s0<={enc_sw,k4,k3,k2};b_s1<=b_s0;
        ack_s<={ack_s[0],cfg_ack};video_ack_s<={video_ack_s[0],video_cfg_ack};
        next_p<=0;prev_p<=0;auto_p<=0;emerg_p<=0;
        release_p=0;
        if(uart_volume[4]!=uart_volume_seen)begin
            uart_volume_seen<=uart_volume[4];
            volume<=uart_volume[3:0]>8?4'd8:uart_volume[3:0];
        end
        if(sw_s1!=sw_candidate)begin sw_candidate<=sw_s1;sw_db<=0;end
        else if(sw_s1==mode) sw_db<=0;
        else if(sw_db>=DEBOUNCE-1) begin mode<=sw_s1;sw_db<=0;end
        else sw_db<=sw_db+1;
        if(b_s1[0]==buttons[0]) b_db0<=0;
        else if(b_db0>=DEBOUNCE-1) begin buttons[0]<=b_s1[0];b_db0<=0;release_p[0]=b_s1[0];end
        else b_db0<=b_db0+1;
        if(b_s1[1]==buttons[1]) b_db1<=0;
        else if(b_db1>=DEBOUNCE-1) begin buttons[1]<=b_s1[1];b_db1<=0;release_p[1]=b_s1[1];end
        else b_db1<=b_db1+1;
        if(b_s1[2]==buttons[2]) b_db2<=0;
        else if(b_db2>=DEBOUNCE-1) begin buttons[2]<=b_s1[2];b_db2<=0;release_p[2]=b_s1[2];end
        else b_db2<=b_db2+1;
        if(b_s1[3]==buttons[3]) b_db3<=0;
        else if(b_db3>=DEBOUNCE-1) begin buttons[3]<=b_s1[3];b_db3<=0;release_p[3]=b_s1[3];end
        else b_db3<=b_db3+1;
        if(!buttons[1]) begin
            if(hold3<LONG_TIME) hold3<=hold3+1;
            else if(!long3) begin long3<=1;emerg_p<=1;end
        end else begin
            hold3<=0;
            if(release_p[1]) begin if(!long3) prev_p<=1;end
            long3<=0;
        end
        // Release is seen one clock before the debounced level changes.
        if(release_p[1] && !long3) prev_p<=1;
        if(!buttons[3]) begin
            if(holds<LONG_TIME) holds<=holds+1;
            else if(!longs)longs<=1;
        end else begin holds<=0;longs<=0;end
        // Defer both resets until release. A held press may become a volume
        // gesture even after LONG_TIME, and must never reset picture settings.
        // Delay a single reset so a double click can change control focus.
        // Held rotation is volume only, and cancels any pending click.
        if(bank_emergency!=emerg)begin
            
            bank_emergency<=emerg;media_focus<=0;click_pending<=0;click_wait<=0;end
        else if(enc_up || enc_dn || mode!=click_mode)begin
            click_pending<=0;click_wait<=0;
        end else if(click_pending && buttons[3] && b_s1[3] && !release_p[3])begin
            if(click_wait>=DOUBLE_TIME-1)begin selected_default;click_pending<=0;click_wait<=0;end
            else click_wait<=click_wait+1'b1;
        end
        if(release_p[3])begin
            if(!useds && bank_emergency==emerg)begin
                if(longs)begin defaults(mode[3:2]);click_pending<=0;end
                else if(click_pending && mode==click_mode)begin
                    fit[emerg]<=~fit[emerg];click_pending<=0;click_wait<=0;
                end else begin click_pending<=1;click_wait<=0;click_mode<=mode;end
            end else begin click_pending<=0;click_wait<=0;end
            useds<=0;
        end
        if(compare) begin used2<=1;used4<=1;end
        if(release_p[0]) begin if(!used2 && !compare && buttons[1]) next_p<=1;used2<=0;end
        if(release_p[2]) begin if(!used4 && !compare && buttons[1]) auto_p<=1;used4<=0;end
        if((enc_up || enc_dn) && buttons[1] && !compare &&
                (!buttons[3] || !b_s1[3]) && !release_p[3])begin
            useds<=1;
            if(!buttons[0])used2<=1;
            if(!buttons[2])used4<=1;
            if(enc_dn)begin if(volume!=0)volume<=volume-1'b1;end
            else if(volume<8)volume<=volume+1'b1;
        end
        if((enc_up || enc_dn) && bank_emergency==emerg && buttons[1] && !compare &&
                buttons[3] && b_s1[3] && !useds && !release_p[3]) begin
            if(!buttons[0]) used2<=1;
            if(!buttons[2]) used4<=1;
            if(!buttons[2]) step=12'sd1;
            else if(!buttons[0]) begin
                case(mode)
                    0,10:    step=12'sd10;
                    1,3,4,8: step=12'sd25;
                    5,6:     step=12'sd20;
                    default: step=12'sd5;
                endcase
            end else begin
                case(mode)
                    0,10:    step=12'sd2;
                    1,3,4,8: step=12'sd5;
                    5,6:     step=12'sd4;
                    default: step=12'sd1;
                endcase
            end
            if(enc_dn) step=-step;
            case(mode)
                0:begin val=$signed(brightness[emerg])+step;if(val>64)val=64;if(val< -64)val=-64;brightness[emerg]<=val;end
                1:begin val=contrast[emerg]+step;if(val>150)val=150;if(val<50)val=50;contrast[emerg]<=val;end
                2:begin val=$signed(tone[emerg])+(enc_up?1:-1);if(val>8)val=8;if(val< -8)val=-8;tone[emerg]<=val;end
                3:begin val=saturation[emerg]+step;if(val>150)val=150;if(val<0)val=0;saturation[emerg]<=val;end
                4:begin val=zoom[emerg]+step;if(val>200)val=200;if(val<50)val=50;zoom[emerg]<=val;end
                5:begin val=$signed(pan_x[emerg])+step;if(val>160)val=160;if(val< -160)val=-160;pan_x[emerg]<=val;end
                6:begin val=$signed(pan_y[emerg])+step;if(val>120)val=120;if(val< -120)val=-120;pan_y[emerg]<=val;end
                7:fit[emerg]<=~fit[emerg];
                8:begin val=opacity[emerg]+step;if(val>100)val=100;if(val<0)val=0;opacity[emerg]<=val;end
                9:blend[emerg]<=~blend[emerg];
                10:begin val=speed[emerg]+step;if(val>60)val=60;if(val<0)val=0;speed[emerg]<=val;end
                11:position[emerg]<=position[emerg]+(enc_up?2'd1:2'd3);
                12:transition[emerg]<=transition[emerg]+(enc_up?2'd1:2'd3);
                13:begin val=duration[emerg]+(enc_up?1:-1);if(val>8)val=8;if(val<1)val=1;duration[emerg]<=val;end
                14:begin val=period[emerg]+(enc_up?1:-1);if(val>10)val=10;if(val<1)val=1;period[emerg]<=val;end
                15:direction[emerg]<=direction[emerg]+(enc_up?2'd1:2'd3);
            endcase
        end
        // Stable mailbox held until the video domain acknowledges capture.
        if((ack_s[1]==cfg_req) && (cfg_mailbox!=current_cfg)) begin
            cfg_mailbox<=current_cfg;cfg_req<=~cfg_req;
        end
        if((video_ack_s[1]==video_cfg_req) && (video_cfg_mailbox!=current_video_cfg))begin
            video_cfg_mailbox<=current_video_cfg;video_cfg_req<=~video_cfg_req;
        end
    end
end
endmodule


module exhibit_config_latch (
    input wire clk,rst,vs,
    input wire req,input wire [92:0] mailbox,
    output reg ack,output reg [92:0] cfg,
    output reg panel_visible
);
reg [2:0] req_s;
reg vs_d,pending;
reg [92:0] captured;
reg [7:0] visible_frames;
wire frame=vs_d && !vs;
always @(posedge clk or posedge rst) begin
    if(rst) begin
        req_s<=0;vs_d<=0;pending<=0;ack<=0;
        cfg<={4'd0,8'd0,8'd100,5'd0,8'd100,8'd100,10'd0,10'd0,
            1'b0,7'd50,1'b0,6'd30,2'd1,2'd1,4'd4,4'd3,2'd0,1'b0,2'd0};
        captured<=0;visible_frames<=0;panel_visible<=0;
    end else begin
        req_s<={req_s[1:0],req};vs_d<=vs;
        if(req_s[2]!=ack && !pending) begin captured<=mailbox;pending<=1;end
        if(frame) begin
            if(pending) begin cfg<=captured;ack<=req_s[2];pending<=0;visible_frames<=180;panel_visible<=1;end
            else if(visible_frames!=0) visible_frames<=visible_frames-1;
            else panel_visible<=0;
        end
    end
end
endmodule

// Two independent RGB565 scanline caches. No frame-wide read session is held.
// A complete next line is fetched during the preceding output line.
module exhibit_reader (
    input wire vclk,mclk,rst,
    input wire ihs,ivs,ide,
    input wire [92:0] cfg,
    input wire [1:0] display_buf,pending_buf,input wire [2:0] buffer_video,
    input wire temporal_blend,input wire transition_req,emergency,display_valid,
    input wire mem_init,mem_busy,write_busy,audio_hold,data_valid,
    input wire [31:0] data,
    output wire rd_en,
    output wire [20:0] rd_addr,
    output wire rd_busy,rd_active,
    output reg swap_ok,
    output reg rd_recover,
    output reg ohs,ovs,ode,
    output reg [23:0] rgb,
    output reg [15:0] line_miss
);
wire [3:0] mode;
wire signed [7:0] brightness;
wire [7:0] contrast,saturation,zoom;
wire signed [4:0] tone;
wire signed [9:0] pan_x,pan_y;
wire fit,blend,compare;
wire [6:0] opacity;
wire [5:0] speed;
wire [1:0] position,transition,direction,tuning;
wire [3:0] duration,period;
assign {mode,brightness,contrast,tone,saturation,zoom,pan_x,pan_y,fit,
    opacity,blend,speed,position,transition,duration,period,direction,compare,tuning}=cfg;
// display_buf/pending_buf/transition_req are one coherent vclk snapshot.
reg vs_d,vs_dd,de_d;
reg [9:0] x,y;
// Delay active-low VS start until frame descriptors/config are committed.
reg [4:0] frame_guard;
wire frame=frame_guard==24;
wire line_start=ide && !de_d;
wire line_end=!ide && de_d;
reg running,seen_req,interpolating;
reg [1:0] old_buf,new_buf,tm,td;
reg [7:0] alpha;
reg [7:0] elapsed,frames;
reg [4:0] alpha_step;
reg [6:0] alpha_rem;
reg [7:0] remainder;
reg [9:0] move_x,move_y;
wire alpha_carry=({1'b0,remainder}+{2'b0,alpha_rem})>={1'b0,frames};
wire [8:0] next_alpha={1'b0,alpha}+{4'd0,alpha_step}+alpha_carry;
wire [9:0] inc;
(* ram_style="block" *) reg [9:0] scale_rom[0:1023];
reg [9:0] scale_value;
initial $readmemh("pip_scale_coefficients.hex",scale_rom);
always @(posedge vclk) scale_value<=scale_rom[{emergency,fit,zoom}];

assign inc=compare ? (emergency?10'd256:10'd421) : scale_value;
wire [9:0] viewport_w=emergency?10'd640:10'd520;
wire [9:0] viewport_h=emergency?10'd480:10'd292;
wire [9:0] viewport_y=emergency?10'd0:10'd132;
wire [9:0] viewport_end=emergency?10'd480:10'd424;
reg signed [20:0] origin_x,origin_y;
reg signed [20:0] phase_x;
reg signed [20:0] shift_ax,shift_bx,shift_ay,shift_by;
// A 10-bit unsigned displacement times a 10-bit increment fits signed 21 bits.
// Share each positive product across both slide directions and the ROI stage;
// negating after multiplication preserves the original two's-complement result.
wire signed [20:0] move_x_scaled=$signed({1'b0,move_x})*$signed({1'b0,inc});
wire signed [20:0] move_y_scaled=$signed({1'b0,move_y})*$signed({1'b0,inc});
// Exact shared viewport products; every shifted term and sum fits signed 21 bits.
// Their halves reproduce the existing centering products without another multiplier.
wire signed [20:0] inc_wide=$signed({11'd0,inc});
wire signed [20:0] viewport_x_scaled=(inc_wide<<<9)+(emergency?(inc_wide<<<7):(inc_wide<<<3));
wire signed [20:0] viewport_y_scaled=emergency?((inc_wide<<<9)-(inc_wide<<<5)):((inc_wide<<<8)+(inc_wide<<<5)+(inc_wide<<<2));
reg signed [21:0] axq,bxq,ayq,byq;
reg signed [11:0] ax,bx,ay,by;
// Both media types share the framebuffer, row cache and transition mixer.
// Video is reconstructed to 640x480 storage. This inexpensive affine correction
// restores its 16:9 display aspect without duplicating either scaling datapath.
wire source_a_video=buffer_video[running?old_buf:display_buf];
wire source_b_video=buffer_video[new_buf];
wire signed [21:0] center_x=($signed(22'sd320)+(compare?22'sd0:$signed(pan_x)))<<<8;
wire signed [21:0] center_y=($signed(22'sd240)+(compare?22'sd0:$signed(pan_y)))<<<8;
function signed [21:0] adjust_x;
 input signed [21:0] q;input video;
 reg signed [21:0] delta;
 begin
  adjust_x=q;delta=q-center_x;
  if(video && !emergency && (!fit || compare))adjust_x=center_x+delta-(delta>>>2);
 end
endfunction
function signed [21:0] adjust_y;
 input signed [21:0] q;input video;
 reg signed [21:0] delta;
 begin
  adjust_y=q;delta=q-center_y;
  if(video && (emergency || (fit && !compare)))adjust_y=center_y+delta+(delta>>>2)+(delta>>>4)+(delta>>>6)+(delta>>>7);
 end
endfunction
reg [9:0] ra,rb;
reg [8:0] weight;
reg outside_a,outside_b,media;
reg [9:0] target_y;
reg request_toggle;
reg [9:0] req_y;
reg [8:0] req_ya,req_yb;
reg [1:0] req_a,req_b;
reg req_pair,req_half;
reg [1:0] req_bank,pixel_bank,prime_row;reg prime_active;reg [3:0] prime_settle;
reg [9:0] req_xa,req_xb,req_na,req_nb;
reg [2:0] done_s;
reg ack_seen;
reg [9:0] cached_y0,cached_y1,cached_y2;
reg ready0,ready1,ready2;
reg [2:0] ack_s;
wire request_free=ack_s[2]==request_toggle;
wire [15:0] qa,qb;
// Crossfade alone uses 320 horizontal samples per image. The final still
// returns to full resolution; wipes/pushes retain full source sampling.
wire half_display=running && tm==3 && elapsed<frames && !interpolating;
wire [9:0] cache_ra=half_display?{1'b0,ra[9:1]}:ra;
wire [9:0] cache_rb=half_display?{1'b0,rb[9:1]}:rb;
// Three 640-pixel rows fit in each existing 2048-word BRAM.
wire [10:0] read_base=pixel_bank==0?11'd0:pixel_bank==1?11'd640:11'd1280;
wire [10:0] read_a=read_base+{1'b0,cache_ra};
wire [10:0] read_b=read_base+{1'b0,cache_rb};
wire [23:0] ca={qa[15:11],qa[15:13],qa[10:5],qa[10:9],qa[4:0],qa[4:2]};
wire [23:0] cb={qb[15:11],qb[15:13],qb[10:5],qb[10:9],qb[4:0],qb[4:2]};
reg [8:0] weight_q;
reg oa_q,ob_q,media_q;
reg hs_q,vs_q,de_q;
wire cached_line_ok=pixel_bank==0?(ready0 && cached_y0==y):
    pixel_bank==1?(ready1 && cached_y1==y):(ready2 && cached_y2==y);
reg line_ok;
wire display_line_ok=line_start?cached_line_ok:line_ok;
reg [15:0] mr,mg,mb;
wire [7:0] source_r=oa_q?8'd0:ca[23:16];
wire [7:0] source_g=oa_q?8'd0:ca[15:8];
wire [7:0] source_b=oa_q?8'd0:ca[7:0];
wire [7:0] target_r=ob_q?8'd0:cb[23:16];
wire [7:0] target_g=ob_q?8'd0:cb[15:8];
wire [7:0] target_b=ob_q?8'd0:cb[7:0];
wire [8:0] inverse_weight=9'd256-weight_q;
wire signed [8:0] mix_delta_r=$signed({1'b0,target_r})-$signed({1'b0,source_r});
wire signed [18:0] mix_product_r=mix_delta_r*$signed({1'b0,weight_q});
wire signed [18:0] mix_r=$signed({3'd0,source_r,8'd0})+mix_product_r;
wire signed [8:0] mix_delta_g=$signed({1'b0,target_g})-$signed({1'b0,source_g});
wire signed [18:0] mix_product_g=mix_delta_g*$signed({1'b0,weight_q});
wire signed [18:0] mix_g=$signed({3'd0,source_g,8'd0})+mix_product_g;
wire signed [8:0] mix_delta_b=$signed({1'b0,target_b})-$signed({1'b0,source_b});
wire signed [18:0] mix_product_b=mix_delta_b*$signed({1'b0,weight_q});
wire signed [18:0] mix_b=$signed({3'd0,source_b,8'd0})+mix_product_b;
reg [2:0] req_s;
reg mem_ack;
reg [1:0] state;
reg [9:0] issue,received;
reg [20:0] base_a,base_b;
reg pair_m,half_m;
reg [1:0] bank_m;
reg [9:0] start_a,start_b,count_a,count_b;
reg [2:0] drain;
reg [11:0] read_watch;
reg row_failed,finish_valid;
wire [9:0] active_count=state==2?count_b:count_a;
wire [9:0] outstanding=issue-received;
reg [9:0] tag_m;
reg [1:0] finish_bank;
reg [9:0] finish_y;
reg finish_toggle;
// Cache addresses retain source coordinates, including padded ROI words.
wire [9:0] full_col_a=start_a+received,full_col_b=start_b+received;
wire [9:0] write_col_a=half_m?{1'b0,full_col_a[9:1]}:full_col_a;
wire [9:0] write_col_b=half_m?{1'b0,full_col_b[9:1]}:full_col_b;
wire [10:0] write_base=bank_m==0?11'd0:bank_m==1?11'd640:11'd1280;
wire [10:0] wa=write_base+{1'b0,write_col_a},wb=write_base+{1'b0,write_col_b};
wire wr_a=data_valid && state==1 && received<count_a && (!half_m || !full_col_a[0]);
wire wr_b=data_valid && state==2 && received<count_b && (!half_m || !full_col_b[0]);
// Sdr_busy reports the controller's active session, not per-beat backpressure.
// Check it before acquiring the bus, then issue the complete contiguous burst.
// Pausing each time it rises stretches a 13us row to 58us at 50MHz.
assign rd_en=(state==1 || state==2) && issue<active_count && outstanding<64;
assign rd_addr=(state==2?base_b:base_a)+{11'd0,issue};
assign rd_busy=(state!=0);
assign rd_active=rd_busy || (req_s[2]!=mem_ack);
function [20:0] buffer_base;
    input [1:0] idx;
    begin case(idx) 0:buffer_base=0;1:buffer_base=307200;2:buffer_base=614400;default:buffer_base=0;endcase end
endfunction
EG_LOGIC_BRAM #(.DATA_WIDTH_A(16),.DATA_WIDTH_B(16),.ADDR_WIDTH_A(11),.ADDR_WIDTH_B(11),
    .DATA_DEPTH_A(2048),.DATA_DEPTH_B(2048),.MODE("DP"),.REGMODE_A("NOREG"),
    .REGMODE_B("NOREG"),.WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),
    .RESETMODE("SYNC"),.IMPLEMENT("9K"),.INIT_FILE("NONE"),.FILL_ALL("NONE")) cache_a(
    .dia(data[15:0]),.dib(16'd0),.addra(wa),.addrb(read_a),.cea(1'b1),.ceb(1'b1),
    .ocea(1'b0),.oceb(1'b0),.clka(mclk),.clkb(vclk),.wea(wr_a),.web(1'b0),
    .bea(1'b0),.beb(1'b0),.rsta(1'b0),.rstb(1'b0),.doa(),.dob(qa));
EG_LOGIC_BRAM #(.DATA_WIDTH_A(16),.DATA_WIDTH_B(16),.ADDR_WIDTH_A(11),.ADDR_WIDTH_B(11),
    .DATA_DEPTH_A(2048),.DATA_DEPTH_B(2048),.MODE("DP"),.REGMODE_A("NOREG"),
    .REGMODE_B("NOREG"),.WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),
    .RESETMODE("SYNC"),.IMPLEMENT("9K"),.INIT_FILE("NONE"),.FILL_ALL("NONE")) cache_b(
    .dia(data[15:0]),.dib(16'd0),.addra(wb),.addrb(read_b),.cea(1'b1),.ceb(1'b1),
    .ocea(1'b0),.oceb(1'b0),.clka(mclk),.clkb(vclk),.wea(wr_b),.web(1'b0),
    .bea(1'b0),.beb(1'b0),.rsta(1'b0),.rstb(1'b0),.doa(),.dob(qb));
always @(posedge mclk or posedge rst) begin
    if(rst) begin
        req_s<=0;mem_ack<=0;state<=0;issue<=0;received<=0;
        base_a<=0;base_b<=0;bank_m<=0;pair_m<=0;half_m<=0;tag_m<=0;
        start_a<=0;start_b<=0;count_a<=0;count_b<=0;drain<=0;
        finish_toggle<=0;finish_bank<=0;finish_y<=0;finish_valid<=0;
        read_watch<=0;row_failed<=0;rd_recover<=0;
    end else begin
        req_s<={req_s[1:0],request_toggle};
        rd_recover<=0;
        if(state==0) begin
            if(req_s[2]!=mem_ack && mem_init && !write_busy && !audio_hold && !mem_busy) begin
                // Capture the held descriptor, then acknowledge it. A second row can
                // wait in the source mailbox while this row finishes on the bus.
                mem_ack<=req_s[2];
                base_a<=buffer_base(req_a)+({12'd0,req_ya}<<9)+({12'd0,req_ya}<<7)+req_xa;
                base_b<=buffer_base(req_b)+({12'd0,req_yb}<<9)+({12'd0,req_yb}<<7)+req_xb;
                bank_m<=req_bank;pair_m<=req_pair;half_m<=req_half;tag_m<=req_y;
                start_a<=req_xa;start_b<=req_xb;count_a<=req_na;count_b<=req_nb;
                issue<=0;received<=0;drain<=0;read_watch<=0;row_failed<=0;
                state<=req_na!=0?1:req_nb!=0?2:3;
            end
        end else if(state==3) begin
            // Finish only after all requested responses and the bus tail drained.
            if(data_valid || mem_busy)drain<=0;
            else if(drain==3) begin
                state<=0;finish_y<=tag_m;
                finish_bank<=bank_m;finish_toggle<=~finish_toggle;finish_valid<=!row_failed;
                rd_recover<=row_failed;
            end else drain<=drain+1;
        end else begin
            if(rd_en) issue<=issue+1;
            if(rd_en || data_valid)read_watch<=0;
            else if(read_watch==12'd2047)begin
                // A missing response must not reserve the bus indefinitely.
                // Drain late beats before freeing ownership; never mark this row ready.
                state<=3;drain<=0;row_failed<=1;issue<=0;received<=0;
            end else read_watch<=read_watch+1'b1;
            if(data_valid && received<active_count) begin
                if(received+1==active_count) begin
                    issue<=0;received<=0;drain<=0;
                    state<=(state==1 && count_b!=0)?2:3;
                end else received<=received+1;
            end
        end
    end
end
always @* begin
    // Centered nearest-neighbor viewport. Outside source bounds is letterbox.
    shift_ax=0;shift_bx=0;shift_ay=0;shift_by=0;
    if(running && tm==2) begin
        case(td)
            0:begin shift_ax=move_x_scaled;shift_bx=shift_ax-viewport_x_scaled;end
            1:begin shift_ax=-move_x_scaled;shift_bx=shift_ax+viewport_x_scaled;end
            2:begin shift_ay=move_y_scaled;shift_by=shift_ay-viewport_y_scaled;end
            3:begin shift_ay=-move_y_scaled;shift_by=shift_ay+viewport_y_scaled;end
        endcase
    end
    axq=adjust_x(phase_x+shift_ax,source_a_video);bxq=adjust_x(phase_x+shift_bx,source_b_video);
    ax=axq>>>8;bx=bxq>>>8;
    media=(x<viewport_w && y>=viewport_y && y<viewport_end);
    ayq=origin_y+$signed({1'b0,y})*$signed({1'b0,inc})-(emergency?22'sd0:$signed(22'd132)*$signed({1'b0,inc}))+shift_ay;
    byq=origin_y+$signed({1'b0,y})*$signed({1'b0,inc})-(emergency?22'sd0:$signed(22'd132)*$signed({1'b0,inc}))+shift_by;
    ayq=adjust_y(ayq,source_a_video);byq=adjust_y(byq,source_b_video);
    ay=ayq>>>8;by=byq>>>8;
    outside_a=(ax<0 || ax>=640 || ay<0 || ay>=480);
    outside_b=(bx<0 || bx>=640 || by<0 || by>=480);
    ra=(ax<0)?0:(ax>=640)?639:ax[9:0];
    rb=(bx<0)?0:(bx>=640)?639:bx[9:0];
    weight=0;
    if(running) begin
        if(tm==3) weight={1'b0,alpha};
        else case(td)
            0:weight=(x>=viewport_w-move_x)?256:0;
            1:weight=(x<move_x)?256:0;
            2:weight=(y>=viewport_end-move_y)?256:0;
            3:weight=(y<viewport_y+move_y)?256:0;
        endcase
        if(elapsed>=frames) weight=256;
    end
end
reg signed [21:0] next_ayq,next_byq;
reg signed [11:0] next_ay,next_by;
reg signed [11:0] normal_next_ay,normal_next_by;
always @* begin
    target_y=prime_active?{8'd0,prime_row}:y+10'd3;
    next_ayq=origin_y+$signed({1'b0,target_y})*$signed({1'b0,inc})-(emergency?22'sd0:$signed(22'd132)*$signed({1'b0,inc}))+shift_ay;
    next_byq=origin_y+$signed({1'b0,target_y})*$signed({1'b0,inc})-(emergency?22'sd0:$signed(22'd132)*$signed({1'b0,inc}))+shift_by;
    next_ayq=adjust_y(next_ayq,source_a_video);next_byq=adjust_y(next_byq,source_b_video);
    next_ay=normal_next_ay;next_by=normal_next_by;
end
reg signed [10:0] min_a,max_a,min_b,max_b;
reg signed [21:0] span_full,span_move;
reg signed [21:0] low_a_q,high_a_q,low_b_q,high_b_q;
reg signed [11:0] low_a,high_a,low_b,high_b;
reg want_a,want_b;
reg [9:0] fetch_xa,fetch_xb,fetch_na,fetch_nb;
always @* begin
    min_a=0;max_a=$signed({1'b0,viewport_w})-1;min_b=0;max_b=max_a;
    want_a=1;want_b=running;
    if(running && (tm==1 || tm==2))begin
        case(td)
            0:begin max_a=$signed({1'b0,viewport_w})-1-$signed({1'b0,move_x});min_b=$signed({1'b0,viewport_w})-$signed({1'b0,move_x});end
            1:begin min_a=$signed({1'b0,move_x});max_b=$signed({1'b0,move_x})-11'sd1;end
            2:begin want_b=target_y>=viewport_end-move_y;want_a=!want_b;end
            3:begin want_b=target_y<viewport_y+move_y;want_a=!want_b;end
        endcase
    end

    fetch_xa=low_a<0?0:low_a>639?639:low_a[9:0];
    fetch_xb=low_b<0?0:low_b>639?639:low_b[9:0];
    fetch_na=0;fetch_nb=0;
    if(want_a && min_a<=max_a && high_a>=0 && low_a<640 && next_ay>=0 && next_ay<480)
        fetch_na=(high_a>639?10'd639:high_a[9:0])-fetch_xa+1'b1;
    if(want_b && min_b<=max_b && high_b>=0 && low_b<640 && next_by>=0 && next_by<480)
        fetch_nb=(high_b>639?10'd639:high_b[9:0])-fetch_xb+1'b1;
    // The PHY transfers four contiguous words per SDRAM command. Round the
    // visible ROI out to complete groups; half-rate crossfade discards odd
    // samples on cache write rather than issuing illegal stride-two reads.
    if(fetch_na!=0)fetch_na=(({1'b0,fetch_xa}+{1'b0,fetch_na}+11'd3)&11'h7fc)-{1'b0,fetch_xa[9:2],2'b00};
    if(fetch_nb!=0)fetch_nb=(({1'b0,fetch_xb}+{1'b0,fetch_nb}+11'd3)&11'h7fc)-{1'b0,fetch_xb[9:2],2'b00};
    fetch_xa={fetch_xa[9:2],2'b00};fetch_xb={fetch_xb[9:2],2'b00};
    // Completed transitions need only the new image, at full resolution.
    if(running && elapsed>=frames)fetch_na=0;
end
// These endpoints are invariant throughout a scanline. Register them before
// the end-of-line request instead of chaining scaling, aspect correction,
// clipping and burst rounding in a single 40 ns pixel-clock path.
always @(posedge vclk or posedge rst)begin
 if(rst)begin low_a<=0;high_a<=0;low_b<=0;high_b<=0;normal_next_ay<=0;normal_next_by<=0;end
 else begin
  low_a<=adjust_x(low_a_q,source_a_video)>>>8;high_a<=adjust_x(high_a_q,source_a_video)>>>8;
  low_b<=adjust_x(low_b_q,source_b_video)>>>8;high_b<=adjust_x(high_b_q,source_b_video)>>>8;
  normal_next_ay<=next_ayq>>>8;normal_next_by<=next_byq>>>8;
 end
end
// Horizontal transform parameters change at frame boundaries. Settle these
// two stages during blanking, before the first normal picture row at y=132.
// Emergency fetches bypass the transform and use their full source row.
always @(posedge vclk or posedge rst) begin
    if(rst)begin
        span_full<=0;span_move<=0;
        low_a_q<=0;high_a_q<=0;low_b_q<=0;high_b_q<=0;
    end else begin
        // All four endpoints share the same viewport span and moving boundary.
        // This avoids four independent variable multipliers in a nearly full FPGA.
        span_full<=viewport_x_scaled-inc_wide;
        span_move<=move_x_scaled;
        low_a_q<=origin_x+shift_ax+((running && (tm==1 || tm==2) && td==1)?span_move:22'sd0);
        high_a_q<=origin_x+shift_ax+span_full-((running && (tm==1 || tm==2) && td==0)?span_move:22'sd0);
        low_b_q<=origin_x+shift_bx+((running && (tm==1 || tm==2) && td==0)?span_full+$signed({1'b0,inc})-span_move:22'sd0);
        high_b_q<=origin_x+shift_bx+((running && (tm==1 || tm==2) && td==1)?span_move-$signed({1'b0,inc}):span_full);
    end
end
always @(posedge vclk or posedge rst) begin
    if(rst) begin
        vs_d<=0;vs_dd<=0;de_d<=0;x<=0;y<=0;frame_guard<=0;
        running<=0;seen_req<=0;interpolating<=0;old_buf<=0;new_buf<=0;tm<=1;td<=0;
        alpha<=0;elapsed<=0;frames<=60;alpha_step<=4;alpha_rem<=16;remainder<=0;
        move_x<=0;move_y<=0;swap_ok<=0;
        origin_x<=0;origin_y<=0;phase_x<=0;
        request_toggle<=0;req_y<=0;req_ya<=0;req_yb<=0;req_a<=0;req_b<=0;req_pair<=0;req_bank<=0;req_half<=0;req_xa<=0;req_xb<=0;req_na<=0;req_nb<=0;line_ok<=0;
        done_s<=0;ack_s<=0;ack_seen<=0;cached_y0<=0;cached_y1<=0;cached_y2<=0;ready0<=0;ready1<=0;ready2<=0;pixel_bank<=0;prime_row<=0;prime_active<=0;prime_settle<=0;
        weight_q<=0;oa_q<=0;ob_q<=0;media_q<=0;hs_q<=0;vs_q<=0;de_q<=0;
        ohs<=0;ovs<=0;ode<=0;rgb<=0;line_miss<=0;
    end else begin
        vs_d<=ivs;vs_dd<=vs_d;de_d<=ide;
        if(vs_d && !vs_dd)frame_guard<=1;
        else if(frame_guard!=0)frame_guard<=frame?0:frame_guard+1'b1;
        if(prime_settle!=0)prime_settle<=prime_settle-1'b1;
        done_s<={done_s[1:0],finish_toggle};ack_s<={ack_s[1:0],mem_ack};
        if(done_s[2]!=ack_seen) begin
            ack_seen<=done_s[2];
            case(finish_bank)
                0:begin cached_y0<=finish_y;ready0<=finish_valid;end
                1:begin cached_y1<=finish_y;ready1<=finish_valid;end
                2:begin cached_y2<=finish_y;ready2<=finish_valid;end
                default:begin end
            endcase
        end
        if(!ide) begin x<=0;phase_x<=origin_x;end
        else begin x<=x+1;phase_x<=phase_x+$signed({1'b0,inc});end
        if(!ide) begin
            origin_x<=((21'sd320+(compare?10'sd0:pan_x))<<<8)-(viewport_x_scaled>>>1);
            origin_y<=((21'sd240+(compare?10'sd0:pan_y))<<<8)-(viewport_y_scaled>>>1);
        end
        if(frame) begin
            y<=0;pixel_bank<=0;ready0<=0;ready1<=0;ready2<=0;prime_row<=0;prime_active<=emergency;prime_settle<=8;
            if(temporal_blend===1'b1 && !transition_req && !emergency)begin
                // Blend only two COMPLETE consecutive video pages. The manager
                // owns their lifetime; this adds display samples, not source fps.
                running<=1;interpolating<=1;seen_req<=0;swap_ok<=0;
                old_buf<=display_buf;new_buf<=pending_buf;tm<=3;td<=0;
                alpha<=128;elapsed<=0;frames<=255;move_x<=0;move_y<=0;
            end else if(!transition_req) begin running<=0;interpolating<=0;seen_req<=0;swap_ok<=0;end
            else if(!seen_req) begin
                seen_req<=1;interpolating<=0;old_buf<=display_buf;new_buf<=pending_buf;tm<=transition;td<=direction;
                frames<=duration*8'd15;elapsed<=0;alpha<=0;remainder<=0;move_x<=0;move_y<=0;
                case(duration)
                    1:begin alpha_step<=17;alpha_rem<=1;end
                    2:begin alpha_step<=8;alpha_rem<=16;end
                    3:begin alpha_step<=5;alpha_rem<=31;end
                    4:begin alpha_step<=4;alpha_rem<=16;end
                    5:begin alpha_step<=3;alpha_rem<=31;end
                    6:begin alpha_step<=2;alpha_rem<=76;end
                    7:begin alpha_step<=2;alpha_rem<=46;end
                    default:begin alpha_step<=2;alpha_rem<=16;end
                endcase
                if(transition==0) begin running<=0;swap_ok<=1;end
                else running<=1;
            end else if(running && elapsed<frames) begin
                elapsed<=elapsed+1;
                if(elapsed+1>=frames) begin alpha<=255;move_x<=viewport_w;move_y<=viewport_h;swap_ok<=1;end
                else begin
                    if(alpha_carry) begin
                        remainder<=remainder+alpha_rem-frames;alpha<=alpha+alpha_step+1;
                    end else begin remainder<=remainder+alpha_rem;alpha<=alpha+alpha_step;end
                    // Use this frame's alpha, avoiding a double-size final motion step.
                    move_x<=emergency?({1'b0,next_alpha}*18'd640)>>8:({1'b0,next_alpha}*18'd520)>>8;
                    move_y<=emergency?({1'b0,next_alpha}*18'd480)>>8:({1'b0,next_alpha}*18'd292)>>8;
                end
            end
        end else if(line_end)begin
            y<=y+1;pixel_bank<=pixel_bank==2?0:pixel_bank+1'b1;
        end
        // Prefetch three rows ahead in the just-released bank. Normal rows
        // get 70.4 us of lead at 25 MHz; steady-state service must stay <32 us.
        // Row readiness is frozen for the whole displayed line.
        if(display_valid && request_free &&
            ((prime_active && prime_settle==0) ||
             (line_end && (emergency?y<477:(y>=129 && y<421))))) begin
            req_y<=target_y;
            req_bank<=prime_active?prime_row:pixel_bank;
            if(prime_active)begin
                if(prime_row==2)prime_active<=0;else prime_row<=prime_row+1'b1;
                prime_settle<=8;
            end
            req_ya<=next_ay<0?0:next_ay>=480?479:next_ay[8:0];
            req_yb<=next_by<0?0:next_by>=480?479:next_by[8:0];
            req_a<=running?old_buf:display_buf;
            req_b<=new_buf;req_pair<=running;
            req_half<=half_display;
            req_xa<=fetch_xa;req_xb<=fetch_xb;req_na<=fetch_na;req_nb<=fetch_nb;
            request_toggle<=~request_toggle;
        end
        if(line_start && display_valid && media &&
            !cached_line_ok && line_miss!=65535)
            line_miss<=line_miss+1;
        // BRAM data and these flags have one-clock latency; output adds one.
        if(line_start)line_ok<=cached_line_ok;
        weight_q<=weight;oa_q<=outside_a || !display_line_ok;ob_q<=outside_b || !display_line_ok;media_q<=media;
        hs_q<=ihs;vs_q<=ivs;de_q<=ide;
        ohs<=hs_q;ovs<=vs_q;ode<=de_q;
        if(!de_q || !media_q || !display_valid) rgb<=0;
        else if(weight_q==0) rgb<={source_r,source_g,source_b};
        else if(weight_q==256) rgb<={target_r,target_g,target_b};
        else rgb<={mix_r[15:8],mix_g[15:8],mix_b[15:8]};
    end
end
endmodule

module exhibit_color (
    input wire clk,rst,ihs,ivs,ide,enable,
    input wire [92:0] cfg,input wire [23:0] irgb,
    output wire ohs,ovs,ode,output reg [23:0] rgb
);
wire [3:0] mode;
wire signed [7:0] brightness;
wire [7:0] contrast,saturation,zoom;
wire signed [4:0] tone;
wire signed [9:0] pan_x,pan_y;
wire fit,blend,compare;
wire [6:0] opacity;
wire [5:0] speed;
wire [1:0] position,transition,direction,tuning;
wire [3:0] duration,period;
assign {mode,brightness,contrast,tone,saturation,zoom,pan_x,pan_y,fit,
    opacity,blend,speed,position,transition,duration,period,direction,compare,tuning}=cfg;
wire [8:0] sat_gain=({1'b0,saturation}*17'd128)/17'd100;
wire [8:0] con_gain=({1'b0,contrast}*17'd128)/17'd100;
reg [3:0] hp,vp,dp,bypass;
reg [23:0] original0,original1,original2;
reg [7:0] gray;
reg signed [9:0] dr,dg,db;
wire signed [19:0] sr=dr*$signed({1'b0,sat_gain});
wire signed [19:0] sg=dg*$signed({1'b0,sat_gain});
wire signed [19:0] sb=db*$signed({1'b0,sat_gain});
reg signed [9:0] cr,cg,cb;
wire signed [19:0] pr=cr*$signed({1'b0,con_gain});
wire signed [19:0] pg=cg*$signed({1'b0,con_gain});
wire signed [19:0] pb=cb*$signed({1'b0,con_gain});
reg [7:0] rr,gg,bb;
function [7:0] clamp;
    input signed [20:0] val;
    begin clamp=val<0?0:val>255?255:val[7:0];end
endfunction
wire [9:0] gray_sum={2'b0,irgb[23:16]}+{1'b0,irgb[15:8],1'b0}+{2'b0,irgb[7:0]};
assign ohs=hp[3];assign ovs=vp[3];assign ode=dp[3];
always @(posedge clk or posedge rst) begin
    if(rst) begin
        hp<=0;vp<=0;dp<=0;bypass<=15;original0<=0;original1<=0;original2<=0;
        gray<=0;dr<=0;dg<=0;db<=0;cr<=0;cg<=0;cb<=0;rr<=0;gg<=0;bb<=0;rgb<=0;
    end else begin
        hp<={hp[2:0],ihs};vp<={vp[2:0],ivs};dp<={dp[2:0],ide};
        bypass<={bypass[2:0],(!enable || compare)};
        original0<=irgb;original1<=original0;original2<=original1;
        gray<=gray_sum[9:2];
        dr<=$signed({2'b0,irgb[23:16]})-$signed({2'b0,gray_sum[9:2]});
        dg<=$signed({2'b0,irgb[15:8]})-$signed({2'b0,gray_sum[9:2]});
        db<=$signed({2'b0,irgb[7:0]})-$signed({2'b0,gray_sum[9:2]});
        cr<=$signed({2'b0,clamp($signed({1'b0,gray})+(sr>>>7))})-10'sd128;
        cg<=$signed({2'b0,clamp($signed({1'b0,gray})+(sg>>>7))})-10'sd128;
        cb<=$signed({2'b0,clamp($signed({1'b0,gray})+(sb>>>7))})-10'sd128;
        rr<=clamp(21'sd128+(pr>>>7)+$signed(brightness)+($signed(tone)<<<1));
        gg<=clamp(21'sd128+(pg>>>7)+$signed(brightness));
        bb<=clamp(21'sd128+(pb>>>7)+$signed(brightness)-($signed(tone)<<<1));
        rgb<=bypass[2]?original2:{rr,gg,bb};
    end
end
endmodule

module exhibit_layer (
    input wire clk,rst,ihs,ivs,ide,enable,input wire fullscreen,
    input wire [92:0] cfg,input wire [23:0] irgb,
    output wire ohs,ovs,ode,output reg [23:0] rgb
);
wire [3:0] mode;
wire signed [7:0] brightness;
wire [7:0] contrast,saturation,zoom;
wire signed [4:0] tone;
wire signed [9:0] pan_x,pan_y;
wire fit,blend,compare;
wire [6:0] opacity;
wire [5:0] speed;
wire [1:0] position,transition,direction,tuning;
wire [3:0] duration,period;
assign {mode,brightness,contrast,tone,saturation,zoom,pan_x,pan_y,fit,
    opacity,blend,speed,position,transition,duration,period,direction,compare,tuning}=cfg;
wire [8:0] a=({2'd0,opacity}*17'd256)/17'd100;
wire [8:0] ia=9'd256-a;
reg de_d,vs_d;
reg [9:0] x,y;
reg [9:0] gallery_scroll;
reg gallery_reverse;
reg [6:0] gallery_fraction;
wire [7:0] gallery_step={1'b0,gallery_fraction}+{2'd0,speed};
wire [9:0] left=fullscreen?(position[0]?10'd568-gallery_scroll:10'd8+gallery_scroll):(position[0]?10'd448:10'd8);
wire [9:0] upper=fullscreen?(position[1]?10'd400:10'd48):(position[1]?10'd288:10'd180);
wire inside_layer=(x>=left && x<left+64 && y>=upper && y<upper+32);
wire [9:0] lx=x-left,ly=y-upper;
wire [10:0] addr={ly[4:0],lx[5:0]};
wire [15:0] logo;
(* rom_style="bram" *) reg [15:0] pixels [0:2047];
reg [15:0] logo_q;
assign logo=logo_q;
initial begin
pixels[0]=16'h05B8;
pixels[1]=16'h05B8;
pixels[2]=16'h05B8;
pixels[3]=16'h05B8;
pixels[4]=16'h05B8;
pixels[5]=16'h05B8;
pixels[6]=16'h05B8;
pixels[7]=16'h05B8;
pixels[8]=16'h05B8;
pixels[9]=16'h05B8;
pixels[10]=16'h05B8;
pixels[11]=16'h05B8;
pixels[12]=16'h05B8;
pixels[13]=16'h05B8;
pixels[14]=16'h05B8;
pixels[15]=16'h05B8;
pixels[16]=16'h05B8;
pixels[17]=16'h05B8;
pixels[18]=16'h05B8;
pixels[19]=16'h05B8;
pixels[20]=16'h05B8;
pixels[21]=16'h05B8;
pixels[22]=16'h05B8;
pixels[23]=16'h05B8;
pixels[24]=16'h05B8;
pixels[25]=16'h05B8;
pixels[26]=16'h05B8;
pixels[27]=16'h05B8;
pixels[28]=16'h05B8;
pixels[29]=16'h05B8;
pixels[30]=16'h05B8;
pixels[31]=16'h05B8;
pixels[32]=16'h05B8;
pixels[33]=16'h05B8;
pixels[34]=16'h05B8;
pixels[35]=16'h05B8;
pixels[36]=16'h05B8;
pixels[37]=16'h05B8;
pixels[38]=16'h05B8;
pixels[39]=16'h05B8;
pixels[40]=16'h05B8;
pixels[41]=16'h05B8;
pixels[42]=16'h05B8;
pixels[43]=16'h05B8;
pixels[44]=16'h05B8;
pixels[45]=16'h05B8;
pixels[46]=16'h05B8;
pixels[47]=16'h05B8;
pixels[48]=16'h05B8;
pixels[49]=16'h05B8;
pixels[50]=16'h05B8;
pixels[51]=16'h05B8;
pixels[52]=16'h05B8;
pixels[53]=16'h05B8;
pixels[54]=16'h05B8;
pixels[55]=16'h05B8;
pixels[56]=16'h05B8;
pixels[57]=16'h05B8;
pixels[58]=16'h05B8;
pixels[59]=16'h05B8;
pixels[60]=16'h05B8;
pixels[61]=16'h05B8;
pixels[62]=16'h05B8;
pixels[63]=16'h05B8;
pixels[64]=16'h05B8;
pixels[65]=16'h05B8;
pixels[66]=16'h05B8;
pixels[67]=16'h05B8;
pixels[68]=16'h05B8;
pixels[69]=16'h05B8;
pixels[70]=16'h05B8;
pixels[71]=16'h05B8;
pixels[72]=16'h05B8;
pixels[73]=16'h05B8;
pixels[74]=16'h05B8;
pixels[75]=16'h05B8;
pixels[76]=16'h05B8;
pixels[77]=16'h05B8;
pixels[78]=16'h05B8;
pixels[79]=16'h05B8;
pixels[80]=16'h05B8;
pixels[81]=16'h05B8;
pixels[82]=16'h05B8;
pixels[83]=16'h05B8;
pixels[84]=16'h05B8;
pixels[85]=16'h05B8;
pixels[86]=16'h05B8;
pixels[87]=16'h05B8;
pixels[88]=16'h05B8;
pixels[89]=16'h05B8;
pixels[90]=16'h05B8;
pixels[91]=16'h05B8;
pixels[92]=16'h05B8;
pixels[93]=16'h05B8;
pixels[94]=16'h05B8;
pixels[95]=16'h05B8;
pixels[96]=16'h05B8;
pixels[97]=16'h05B8;
pixels[98]=16'h05B8;
pixels[99]=16'h05B8;
pixels[100]=16'h05B8;
pixels[101]=16'h05B8;
pixels[102]=16'h05B8;
pixels[103]=16'h05B8;
pixels[104]=16'h05B8;
pixels[105]=16'h05B8;
pixels[106]=16'h05B8;
pixels[107]=16'h05B8;
pixels[108]=16'h05B8;
pixels[109]=16'h05B8;
pixels[110]=16'h05B8;
pixels[111]=16'h05B8;
pixels[112]=16'h05B8;
pixels[113]=16'h05B8;
pixels[114]=16'h05B8;
pixels[115]=16'h05B8;
pixels[116]=16'h05B8;
pixels[117]=16'h05B8;
pixels[118]=16'h05B8;
pixels[119]=16'h05B8;
pixels[120]=16'h05B8;
pixels[121]=16'h05B8;
pixels[122]=16'h05B8;
pixels[123]=16'h05B8;
pixels[124]=16'h05B8;
pixels[125]=16'h05B8;
pixels[126]=16'h05B8;
pixels[127]=16'h05B8;
pixels[128]=16'h05B8;
pixels[129]=16'h05B8;
pixels[130]=16'h05B8;
pixels[131]=16'h05B8;
pixels[132]=16'h05B8;
pixels[133]=16'h05B8;
pixels[134]=16'h05B8;
pixels[135]=16'h05B8;
pixels[136]=16'h05B8;
pixels[137]=16'h05B8;
pixels[138]=16'h05B8;
pixels[139]=16'h05B8;
pixels[140]=16'h05B8;
pixels[141]=16'h05B8;
pixels[142]=16'h05B8;
pixels[143]=16'h05B8;
pixels[144]=16'h05B8;
pixels[145]=16'h05B8;
pixels[146]=16'h05B8;
pixels[147]=16'h05B8;
pixels[148]=16'h05B8;
pixels[149]=16'h05B8;
pixels[150]=16'h05B8;
pixels[151]=16'h05B8;
pixels[152]=16'h05B8;
pixels[153]=16'h05B8;
pixels[154]=16'h05B8;
pixels[155]=16'h05B8;
pixels[156]=16'h05B8;
pixels[157]=16'h05B8;
pixels[158]=16'h05B8;
pixels[159]=16'h05B8;
pixels[160]=16'h05B8;
pixels[161]=16'h05B8;
pixels[162]=16'h05B8;
pixels[163]=16'h05B8;
pixels[164]=16'h05B8;
pixels[165]=16'h05B8;
pixels[166]=16'h05B8;
pixels[167]=16'h05B8;
pixels[168]=16'h05B8;
pixels[169]=16'h05B8;
pixels[170]=16'h05B8;
pixels[171]=16'h05B8;
pixels[172]=16'h05B8;
pixels[173]=16'h05B8;
pixels[174]=16'h05B8;
pixels[175]=16'h05B8;
pixels[176]=16'h05B8;
pixels[177]=16'h05B8;
pixels[178]=16'h05B8;
pixels[179]=16'h05B8;
pixels[180]=16'h05B8;
pixels[181]=16'h05B8;
pixels[182]=16'h05B8;
pixels[183]=16'h05B8;
pixels[184]=16'h05B8;
pixels[185]=16'h05B8;
pixels[186]=16'h05B8;
pixels[187]=16'h05B8;
pixels[188]=16'h05B8;
pixels[189]=16'h05B8;
pixels[190]=16'h05B8;
pixels[191]=16'h05B8;
pixels[192]=16'h05B8;
pixels[193]=16'h05B8;
pixels[194]=16'h05B8;
pixels[195]=16'h05B8;
pixels[196]=16'h05B8;
pixels[197]=16'h05B8;
pixels[198]=16'h05B8;
pixels[199]=16'h05B8;
pixels[200]=16'h05B8;
pixels[201]=16'h05B8;
pixels[202]=16'h05B8;
pixels[203]=16'h05B8;
pixels[204]=16'h05B8;
pixels[205]=16'h05B8;
pixels[206]=16'h05B8;
pixels[207]=16'h05B8;
pixels[208]=16'h05B8;
pixels[209]=16'h05B8;
pixels[210]=16'h05B8;
pixels[211]=16'h05B8;
pixels[212]=16'h05B8;
pixels[213]=16'h05B8;
pixels[214]=16'h05B8;
pixels[215]=16'h05B8;
pixels[216]=16'h05B8;
pixels[217]=16'h05B8;
pixels[218]=16'h05B8;
pixels[219]=16'h05B8;
pixels[220]=16'h05B8;
pixels[221]=16'h05B8;
pixels[222]=16'h05B8;
pixels[223]=16'h05B8;
pixels[224]=16'h05B8;
pixels[225]=16'h05B8;
pixels[226]=16'h05B8;
pixels[227]=16'h05B8;
pixels[228]=16'h05B8;
pixels[229]=16'h05B8;
pixels[230]=16'h05B8;
pixels[231]=16'h05B8;
pixels[232]=16'h05B8;
pixels[233]=16'h05B8;
pixels[234]=16'h05B8;
pixels[235]=16'h05B8;
pixels[236]=16'h05B8;
pixels[237]=16'h05B8;
pixels[238]=16'h05B8;
pixels[239]=16'h05B8;
pixels[240]=16'h05B8;
pixels[241]=16'h05B8;
pixels[242]=16'h05B8;
pixels[243]=16'h05B8;
pixels[244]=16'h05B8;
pixels[245]=16'h05B8;
pixels[246]=16'h05B8;
pixels[247]=16'h05B8;
pixels[248]=16'h05B8;
pixels[249]=16'h05B8;
pixels[250]=16'h05B8;
pixels[251]=16'h05B8;
pixels[252]=16'h05B8;
pixels[253]=16'h05B8;
pixels[254]=16'h05B8;
pixels[255]=16'h05B8;
pixels[256]=16'h05B8;
pixels[257]=16'h05B8;
pixels[258]=16'h05B8;
pixels[259]=16'h05B8;
pixels[260]=16'h05B8;
pixels[261]=16'h05B8;
pixels[262]=16'h05B8;
pixels[263]=16'h05B8;
pixels[264]=16'h05B8;
pixels[265]=16'h05B8;
pixels[266]=16'h05B8;
pixels[267]=16'h05B8;
pixels[268]=16'h05B8;
pixels[269]=16'h05B8;
pixels[270]=16'h05B8;
pixels[271]=16'h05B8;
pixels[272]=16'h05B8;
pixels[273]=16'h05B8;
pixels[274]=16'h05B8;
pixels[275]=16'h05B8;
pixels[276]=16'h05B8;
pixels[277]=16'h05B8;
pixels[278]=16'h05B8;
pixels[279]=16'h05B8;
pixels[280]=16'h05B8;
pixels[281]=16'h05B8;
pixels[282]=16'h05B8;
pixels[283]=16'h05B8;
pixels[284]=16'h05B8;
pixels[285]=16'h05B8;
pixels[286]=16'h05B8;
pixels[287]=16'h05B8;
pixels[288]=16'h05B8;
pixels[289]=16'h05B8;
pixels[290]=16'h05B8;
pixels[291]=16'h05B8;
pixels[292]=16'h05B8;
pixels[293]=16'h05B8;
pixels[294]=16'h05B8;
pixels[295]=16'h05B8;
pixels[296]=16'h05B8;
pixels[297]=16'h05B8;
pixels[298]=16'h05B8;
pixels[299]=16'h05B8;
pixels[300]=16'h05B8;
pixels[301]=16'h05B8;
pixels[302]=16'h05B8;
pixels[303]=16'h05B8;
pixels[304]=16'h05B8;
pixels[305]=16'h05B8;
pixels[306]=16'h05B8;
pixels[307]=16'h05B8;
pixels[308]=16'h05B8;
pixels[309]=16'h05B8;
pixels[310]=16'h05B8;
pixels[311]=16'h05B8;
pixels[312]=16'h05B8;
pixels[313]=16'h05B8;
pixels[314]=16'h05B8;
pixels[315]=16'h05B8;
pixels[316]=16'h05B8;
pixels[317]=16'h05B8;
pixels[318]=16'h05B8;
pixels[319]=16'h05B8;
pixels[320]=16'h05B8;
pixels[321]=16'h05B8;
pixels[322]=16'h05B8;
pixels[323]=16'h05B8;
pixels[324]=16'h05B8;
pixels[325]=16'h05B8;
pixels[326]=16'h05B8;
pixels[327]=16'h05B8;
pixels[328]=16'h05B8;
pixels[329]=16'h05B8;
pixels[330]=16'h05B8;
pixels[331]=16'h05B8;
pixels[332]=16'h05B8;
pixels[333]=16'h05B8;
pixels[334]=16'h05B8;
pixels[335]=16'h05B8;
pixels[336]=16'h05B8;
pixels[337]=16'h05B8;
pixels[338]=16'h05B8;
pixels[339]=16'h05B8;
pixels[340]=16'h05B8;
pixels[341]=16'h05B8;
pixels[342]=16'h05B8;
pixels[343]=16'h05B8;
pixels[344]=16'h05B8;
pixels[345]=16'h05B8;
pixels[346]=16'h05B8;
pixels[347]=16'h05B8;
pixels[348]=16'h05B8;
pixels[349]=16'h05B8;
pixels[350]=16'h05B8;
pixels[351]=16'h05B8;
pixels[352]=16'h05B8;
pixels[353]=16'h05B8;
pixels[354]=16'h05B8;
pixels[355]=16'h05B8;
pixels[356]=16'h05B8;
pixels[357]=16'h05B8;
pixels[358]=16'h05B8;
pixels[359]=16'h05B8;
pixels[360]=16'h05B8;
pixels[361]=16'h05B8;
pixels[362]=16'h05B8;
pixels[363]=16'h05B8;
pixels[364]=16'h05B8;
pixels[365]=16'h05B8;
pixels[366]=16'h05B8;
pixels[367]=16'h05B8;
pixels[368]=16'h05B8;
pixels[369]=16'h05B8;
pixels[370]=16'h05B8;
pixels[371]=16'h05B8;
pixels[372]=16'h05B8;
pixels[373]=16'h05B8;
pixels[374]=16'h05B8;
pixels[375]=16'h05B8;
pixels[376]=16'h05B8;
pixels[377]=16'h05B8;
pixels[378]=16'h05B8;
pixels[379]=16'h05B8;
pixels[380]=16'h05B8;
pixels[381]=16'h05B8;
pixels[382]=16'h05B8;
pixels[383]=16'h05B8;
pixels[384]=16'h05B8;
pixels[385]=16'h05B8;
pixels[386]=16'h05B8;
pixels[387]=16'h05B8;
pixels[388]=16'h05B8;
pixels[389]=16'h05B8;
pixels[390]=16'h05B8;
pixels[391]=16'h05B8;
pixels[392]=16'h05B8;
pixels[393]=16'h05B8;
pixels[394]=16'h05B8;
pixels[395]=16'h05B8;
pixels[396]=16'h05B8;
pixels[397]=16'h05B8;
pixels[398]=16'h05B8;
pixels[399]=16'h465A;
pixels[400]=16'h7EDB;
pixels[401]=16'hAF3D;
pixels[402]=16'hE7BE;
pixels[403]=16'h463A;
pixels[404]=16'h05B8;
pixels[405]=16'h05B8;
pixels[406]=16'h05B8;
pixels[407]=16'h05B8;
pixels[408]=16'h05B8;
pixels[409]=16'h05B8;
pixels[410]=16'h05B8;
pixels[411]=16'h05B8;
pixels[412]=16'h05B8;
pixels[413]=16'h05B8;
pixels[414]=16'h05B8;
pixels[415]=16'h05B8;
pixels[416]=16'h05B8;
pixels[417]=16'h05B8;
pixels[418]=16'h05B8;
pixels[419]=16'h05B8;
pixels[420]=16'h05B8;
pixels[421]=16'h05B8;
pixels[422]=16'h05B8;
pixels[423]=16'h05B8;
pixels[424]=16'h05B8;
pixels[425]=16'h05B8;
pixels[426]=16'h05B8;
pixels[427]=16'h3E39;
pixels[428]=16'hEFDF;
pixels[429]=16'hD79E;
pixels[430]=16'hB75D;
pixels[431]=16'h3E39;
pixels[432]=16'h05B8;
pixels[433]=16'h05B8;
pixels[434]=16'h05B8;
pixels[435]=16'h05B8;
pixels[436]=16'h05B8;
pixels[437]=16'h05B8;
pixels[438]=16'h05B8;
pixels[439]=16'h05B8;
pixels[440]=16'h05B8;
pixels[441]=16'h05B8;
pixels[442]=16'h05B8;
pixels[443]=16'h05B8;
pixels[444]=16'h05B8;
pixels[445]=16'h05B8;
pixels[446]=16'h05B8;
pixels[447]=16'h05B8;
pixels[448]=16'h05B8;
pixels[449]=16'h05B8;
pixels[450]=16'h05B8;
pixels[451]=16'h05B8;
pixels[452]=16'h05B8;
pixels[453]=16'h05B8;
pixels[454]=16'h05B8;
pixels[455]=16'h05B8;
pixels[456]=16'h05B8;
pixels[457]=16'h05B8;
pixels[458]=16'h05B8;
pixels[459]=16'h05B8;
pixels[460]=16'h05B8;
pixels[461]=16'h05B8;
pixels[462]=16'h05B8;
pixels[463]=16'hB75D;
pixels[464]=16'hFFFF;
pixels[465]=16'hFFFF;
pixels[466]=16'hFFFF;
pixels[467]=16'h96FC;
pixels[468]=16'h05B8;
pixels[469]=16'h05B8;
pixels[470]=16'h05B8;
pixels[471]=16'h05B8;
pixels[472]=16'h05B8;
pixels[473]=16'h05B8;
pixels[474]=16'h05B8;
pixels[475]=16'h05B8;
pixels[476]=16'h05B8;
pixels[477]=16'h05B8;
pixels[478]=16'h05B8;
pixels[479]=16'h05B8;
pixels[480]=16'h05B8;
pixels[481]=16'h05B8;
pixels[482]=16'h05B8;
pixels[483]=16'h05B8;
pixels[484]=16'h05B8;
pixels[485]=16'h05B8;
pixels[486]=16'h05B8;
pixels[487]=16'h05B8;
pixels[488]=16'h05B8;
pixels[489]=16'h05B8;
pixels[490]=16'h05B8;
pixels[491]=16'hB75D;
pixels[492]=16'hFFFF;
pixels[493]=16'hFFFF;
pixels[494]=16'hF7DF;
pixels[495]=16'h0DD8;
pixels[496]=16'h05B8;
pixels[497]=16'h05B8;
pixels[498]=16'h05B8;
pixels[499]=16'h05B8;
pixels[500]=16'h05B8;
pixels[501]=16'h05B8;
pixels[502]=16'h05B8;
pixels[503]=16'h05B8;
pixels[504]=16'h05B8;
pixels[505]=16'h05B8;
pixels[506]=16'h05B8;
pixels[507]=16'h05B8;
pixels[508]=16'h05B8;
pixels[509]=16'h05B8;
pixels[510]=16'h05B8;
pixels[511]=16'h05B8;
pixels[512]=16'h05B8;
pixels[513]=16'h05B8;
pixels[514]=16'h05B8;
pixels[515]=16'h05B8;
pixels[516]=16'h05B8;
pixels[517]=16'h05B8;
pixels[518]=16'h05B8;
pixels[519]=16'h05B8;
pixels[520]=16'h05B8;
pixels[521]=16'h05B8;
pixels[522]=16'h05B8;
pixels[523]=16'h05B8;
pixels[524]=16'h05B8;
pixels[525]=16'h05B8;
pixels[526]=16'h05B8;
pixels[527]=16'h669B;
pixels[528]=16'hFFFF;
pixels[529]=16'hFFFF;
pixels[530]=16'hFFFF;
pixels[531]=16'hE7BF;
pixels[532]=16'h05B8;
pixels[533]=16'h05B8;
pixels[534]=16'h05B8;
pixels[535]=16'h05B8;
pixels[536]=16'h05B8;
pixels[537]=16'h05B8;
pixels[538]=16'h05B8;
pixels[539]=16'h05B8;
pixels[540]=16'h05B8;
pixels[541]=16'h05B8;
pixels[542]=16'h05B8;
pixels[543]=16'h4E5A;
pixels[544]=16'hFFFF;
pixels[545]=16'hFFFF;
pixels[546]=16'hFFFF;
pixels[547]=16'hFFFF;
pixels[548]=16'hFFFF;
pixels[549]=16'hFFFF;
pixels[550]=16'hFFFF;
pixels[551]=16'hFFFF;
pixels[552]=16'h3619;
pixels[553]=16'h05B8;
pixels[554]=16'h3E39;
pixels[555]=16'hFFFF;
pixels[556]=16'hFFFF;
pixels[557]=16'hFFFF;
pixels[558]=16'h96FC;
pixels[559]=16'h05B8;
pixels[560]=16'h05B8;
pixels[561]=16'h05B8;
pixels[562]=16'h05B8;
pixels[563]=16'h05B8;
pixels[564]=16'h05B8;
pixels[565]=16'h05B8;
pixels[566]=16'h05B8;
pixels[567]=16'h05B8;
pixels[568]=16'h05B8;
pixels[569]=16'h05B8;
pixels[570]=16'h05B8;
pixels[571]=16'h05B8;
pixels[572]=16'h05B8;
pixels[573]=16'h05B8;
pixels[574]=16'h05B8;
pixels[575]=16'h05B8;
pixels[576]=16'h05B8;
pixels[577]=16'h05B8;
pixels[578]=16'h05B8;
pixels[579]=16'h05B8;
pixels[580]=16'h05B8;
pixels[581]=16'h05B8;
pixels[582]=16'h9F1C;
pixels[583]=16'hFFFF;
pixels[584]=16'hFFFF;
pixels[585]=16'hFFFF;
pixels[586]=16'hFFFF;
pixels[587]=16'hFFFF;
pixels[588]=16'hFFFF;
pixels[589]=16'hFFFF;
pixels[590]=16'hFFFF;
pixels[591]=16'hFFFF;
pixels[592]=16'hFFFF;
pixels[593]=16'hFFFF;
pixels[594]=16'hFFFF;
pixels[595]=16'hFFFF;
pixels[596]=16'hFFFF;
pixels[597]=16'hFFFF;
pixels[598]=16'hFFFF;
pixels[599]=16'hFFFF;
pixels[600]=16'hFFFF;
pixels[601]=16'hFFFF;
pixels[602]=16'hFFFF;
pixels[603]=16'hFFFF;
pixels[604]=16'hB75D;
pixels[605]=16'h05B8;
pixels[606]=16'h05B8;
pixels[607]=16'h4E5A;
pixels[608]=16'hFFFF;
pixels[609]=16'hFFFF;
pixels[610]=16'hFFFF;
pixels[611]=16'hFFFF;
pixels[612]=16'hFFFF;
pixels[613]=16'hFFFF;
pixels[614]=16'hFFFF;
pixels[615]=16'hFFFF;
pixels[616]=16'h3619;
pixels[617]=16'h05B8;
pixels[618]=16'hCF9E;
pixels[619]=16'hFFFF;
pixels[620]=16'hFFFF;
pixels[621]=16'hFFFF;
pixels[622]=16'hFFFF;
pixels[623]=16'hFFFF;
pixels[624]=16'hFFFF;
pixels[625]=16'hFFFF;
pixels[626]=16'hFFFF;
pixels[627]=16'hFFFF;
pixels[628]=16'hFFFF;
pixels[629]=16'hA73D;
pixels[630]=16'h05B8;
pixels[631]=16'h05B8;
pixels[632]=16'h05B8;
pixels[633]=16'h05B8;
pixels[634]=16'h05B8;
pixels[635]=16'h05B8;
pixels[636]=16'h05B8;
pixels[637]=16'h05B8;
pixels[638]=16'h05B8;
pixels[639]=16'h05B8;
pixels[640]=16'h05B8;
pixels[641]=16'h05B8;
pixels[642]=16'h05B8;
pixels[643]=16'h05B8;
pixels[644]=16'h05B8;
pixels[645]=16'h05B8;
pixels[646]=16'h9F1C;
pixels[647]=16'hFFFF;
pixels[648]=16'hFFFF;
pixels[649]=16'hFFFF;
pixels[650]=16'hFFFF;
pixels[651]=16'hFFFF;
pixels[652]=16'hFFFF;
pixels[653]=16'hFFFF;
pixels[654]=16'hFFFF;
pixels[655]=16'hFFFF;
pixels[656]=16'hFFFF;
pixels[657]=16'hFFFF;
pixels[658]=16'hFFFF;
pixels[659]=16'hFFFF;
pixels[660]=16'hFFFF;
pixels[661]=16'hFFFF;
pixels[662]=16'hFFFF;
pixels[663]=16'hFFFF;
pixels[664]=16'hFFFF;
pixels[665]=16'hFFFF;
pixels[666]=16'hFFFF;
pixels[667]=16'hFFFF;
pixels[668]=16'hB75D;
pixels[669]=16'h05B8;
pixels[670]=16'h05B8;
pixels[671]=16'h4E5A;
pixels[672]=16'hFFFF;
pixels[673]=16'hFFFF;
pixels[674]=16'hFFFF;
pixels[675]=16'hFFFF;
pixels[676]=16'hFFFF;
pixels[677]=16'hFFFF;
pixels[678]=16'hFFFF;
pixels[679]=16'hFFFF;
pixels[680]=16'h3619;
pixels[681]=16'h7EDB;
pixels[682]=16'hFFFF;
pixels[683]=16'hFFFF;
pixels[684]=16'hFFFF;
pixels[685]=16'hFFFF;
pixels[686]=16'hFFFF;
pixels[687]=16'hFFFF;
pixels[688]=16'hFFFF;
pixels[689]=16'hFFFF;
pixels[690]=16'hFFFF;
pixels[691]=16'hFFFF;
pixels[692]=16'hFFFF;
pixels[693]=16'hA73D;
pixels[694]=16'h05B8;
pixels[695]=16'h05B8;
pixels[696]=16'h05B8;
pixels[697]=16'h05B8;
pixels[698]=16'h05B8;
pixels[699]=16'h05B8;
pixels[700]=16'h05B8;
pixels[701]=16'h05B8;
pixels[702]=16'h05B8;
pixels[703]=16'h05B8;
pixels[704]=16'h05B8;
pixels[705]=16'h05B8;
pixels[706]=16'h05B8;
pixels[707]=16'h05B8;
pixels[708]=16'h05B8;
pixels[709]=16'h05B8;
pixels[710]=16'h9F1C;
pixels[711]=16'hFFFF;
pixels[712]=16'hFFFF;
pixels[713]=16'hFFFF;
pixels[714]=16'hFFFF;
pixels[715]=16'hFFFF;
pixels[716]=16'hFFFF;
pixels[717]=16'hFFFF;
pixels[718]=16'hFFFF;
pixels[719]=16'hFFFF;
pixels[720]=16'hFFFF;
pixels[721]=16'hFFFF;
pixels[722]=16'hFFFF;
pixels[723]=16'hFFFF;
pixels[724]=16'hFFFF;
pixels[725]=16'hFFFF;
pixels[726]=16'hFFFF;
pixels[727]=16'hFFFF;
pixels[728]=16'hFFFF;
pixels[729]=16'hFFFF;
pixels[730]=16'hFFFF;
pixels[731]=16'hFFFF;
pixels[732]=16'hB75D;
pixels[733]=16'h05B8;
pixels[734]=16'h05B8;
pixels[735]=16'h4E5A;
pixels[736]=16'hFFFF;
pixels[737]=16'hFFFF;
pixels[738]=16'hC77E;
pixels[739]=16'h25F9;
pixels[740]=16'h25F9;
pixels[741]=16'hDFBE;
pixels[742]=16'hFFFF;
pixels[743]=16'hFFFF;
pixels[744]=16'h76BB;
pixels[745]=16'hFFFF;
pixels[746]=16'hFFFF;
pixels[747]=16'hFFFF;
pixels[748]=16'hF7FF;
pixels[749]=16'hEFDF;
pixels[750]=16'hEFDF;
pixels[751]=16'hEFDF;
pixels[752]=16'hEFDF;
pixels[753]=16'hEFDF;
pixels[754]=16'hFFFF;
pixels[755]=16'hFFFF;
pixels[756]=16'hFFFF;
pixels[757]=16'hA71D;
pixels[758]=16'h05B8;
pixels[759]=16'h05B8;
pixels[760]=16'h05B8;
pixels[761]=16'h05B8;
pixels[762]=16'h05B8;
pixels[763]=16'h05B8;
pixels[764]=16'h05B8;
pixels[765]=16'h05B8;
pixels[766]=16'h05B8;
pixels[767]=16'h05B8;
pixels[768]=16'h05B8;
pixels[769]=16'h05B8;
pixels[770]=16'h05B8;
pixels[771]=16'h05B8;
pixels[772]=16'h05B8;
pixels[773]=16'h05B8;
pixels[774]=16'h9F1C;
pixels[775]=16'hFFFF;
pixels[776]=16'hFFFF;
pixels[777]=16'hF7DF;
pixels[778]=16'h0DB8;
pixels[779]=16'h0DB8;
pixels[780]=16'h0DB8;
pixels[781]=16'h0DB8;
pixels[782]=16'h0DB8;
pixels[783]=16'h0DB8;
pixels[784]=16'h0DB8;
pixels[785]=16'h0DB8;
pixels[786]=16'h0DB8;
pixels[787]=16'h0DB8;
pixels[788]=16'h0DB8;
pixels[789]=16'h0DB8;
pixels[790]=16'h0DB8;
pixels[791]=16'h0DB8;
pixels[792]=16'h0DB8;
pixels[793]=16'hD79E;
pixels[794]=16'hFFFF;
pixels[795]=16'hFFFF;
pixels[796]=16'hB75D;
pixels[797]=16'h05B8;
pixels[798]=16'h05B8;
pixels[799]=16'h4E5A;
pixels[800]=16'hFFFF;
pixels[801]=16'hFFFF;
pixels[802]=16'hBF5D;
pixels[803]=16'h05B8;
pixels[804]=16'h05B8;
pixels[805]=16'hD79E;
pixels[806]=16'hFFFF;
pixels[807]=16'hFFFF;
pixels[808]=16'hFFFF;
pixels[809]=16'hFFFF;
pixels[810]=16'hFFFF;
pixels[811]=16'hFFFF;
pixels[812]=16'hEFDF;
pixels[813]=16'h25F9;
pixels[814]=16'h05B8;
pixels[815]=16'h05B8;
pixels[816]=16'h05B8;
pixels[817]=16'h9F1C;
pixels[818]=16'hFFFF;
pixels[819]=16'hFFFF;
pixels[820]=16'hFFFF;
pixels[821]=16'h3E39;
pixels[822]=16'h05B8;
pixels[823]=16'h05B8;
pixels[824]=16'h05B8;
pixels[825]=16'h05B8;
pixels[826]=16'h05B8;
pixels[827]=16'h05B8;
pixels[828]=16'h05B8;
pixels[829]=16'h05B8;
pixels[830]=16'h05B8;
pixels[831]=16'h05B8;
pixels[832]=16'h05B8;
pixels[833]=16'h05B8;
pixels[834]=16'h05B8;
pixels[835]=16'h05B8;
pixels[836]=16'h05B8;
pixels[837]=16'h05B8;
pixels[838]=16'h9F1C;
pixels[839]=16'hFFFF;
pixels[840]=16'hFFFF;
pixels[841]=16'hF7DF;
pixels[842]=16'h05B8;
pixels[843]=16'h05B8;
pixels[844]=16'h05B8;
pixels[845]=16'h05B8;
pixels[846]=16'hCF7E;
pixels[847]=16'hB75D;
pixels[848]=16'h76BB;
pixels[849]=16'h3619;
pixels[850]=16'h05B8;
pixels[851]=16'h05B8;
pixels[852]=16'h05B8;
pixels[853]=16'h05B8;
pixels[854]=16'h05B8;
pixels[855]=16'h05B8;
pixels[856]=16'h05B8;
pixels[857]=16'hD79E;
pixels[858]=16'hFFFF;
pixels[859]=16'hFFFF;
pixels[860]=16'hB75D;
pixels[861]=16'h05B8;
pixels[862]=16'h05B8;
pixels[863]=16'h4E5A;
pixels[864]=16'hFFFF;
pixels[865]=16'hFFFF;
pixels[866]=16'hBF5D;
pixels[867]=16'h05B8;
pixels[868]=16'h05B8;
pixels[869]=16'hD79E;
pixels[870]=16'hFFFF;
pixels[871]=16'hFFFF;
pixels[872]=16'hEFDF;
pixels[873]=16'hFFFF;
pixels[874]=16'hFFFF;
pixels[875]=16'hFFFF;
pixels[876]=16'hFFFF;
pixels[877]=16'hDFBE;
pixels[878]=16'h3619;
pixels[879]=16'h05B8;
pixels[880]=16'h8EFC;
pixels[881]=16'hFFFF;
pixels[882]=16'hFFFF;
pixels[883]=16'hFFFF;
pixels[884]=16'h971C;
pixels[885]=16'h05B8;
pixels[886]=16'h05B8;
pixels[887]=16'h05B8;
pixels[888]=16'h05B8;
pixels[889]=16'h05B8;
pixels[890]=16'h05B8;
pixels[891]=16'h05B8;
pixels[892]=16'h05B8;
pixels[893]=16'h05B8;
pixels[894]=16'h05B8;
pixels[895]=16'h05B8;
pixels[896]=16'h05B8;
pixels[897]=16'h05B8;
pixels[898]=16'h05B8;
pixels[899]=16'h05B8;
pixels[900]=16'h05B8;
pixels[901]=16'h05B8;
pixels[902]=16'h05B8;
pixels[903]=16'h05B8;
pixels[904]=16'h05B8;
pixels[905]=16'h05B8;
pixels[906]=16'h05B8;
pixels[907]=16'h05B8;
pixels[908]=16'h05B8;
pixels[909]=16'h5E9A;
pixels[910]=16'hFFFF;
pixels[911]=16'hFFFF;
pixels[912]=16'hFFFF;
pixels[913]=16'hEFDF;
pixels[914]=16'h1DD8;
pixels[915]=16'h05B8;
pixels[916]=16'h05B8;
pixels[917]=16'h05B8;
pixels[918]=16'h05B8;
pixels[919]=16'h05B8;
pixels[920]=16'h05B8;
pixels[921]=16'h05B8;
pixels[922]=16'h05B8;
pixels[923]=16'h05B8;
pixels[924]=16'h05B8;
pixels[925]=16'h05B8;
pixels[926]=16'h05B8;
pixels[927]=16'h4E5A;
pixels[928]=16'hFFFF;
pixels[929]=16'hFFFF;
pixels[930]=16'hFFFF;
pixels[931]=16'hFFFF;
pixels[932]=16'hFFFF;
pixels[933]=16'hFFFF;
pixels[934]=16'hFFFF;
pixels[935]=16'hFFFF;
pixels[936]=16'h567A;
pixels[937]=16'hBF5D;
pixels[938]=16'h2E19;
pixels[939]=16'hE7BF;
pixels[940]=16'hFFFF;
pixels[941]=16'hFFFF;
pixels[942]=16'hF7FF;
pixels[943]=16'hD79E;
pixels[944]=16'hFFFF;
pixels[945]=16'hFFFF;
pixels[946]=16'hFFFF;
pixels[947]=16'hC77E;
pixels[948]=16'h0DB8;
pixels[949]=16'h05B8;
pixels[950]=16'h05B8;
pixels[951]=16'h05B8;
pixels[952]=16'h05B8;
pixels[953]=16'h05B8;
pixels[954]=16'h05B8;
pixels[955]=16'h05B8;
pixels[956]=16'h05B8;
pixels[957]=16'h05B8;
pixels[958]=16'h05B8;
pixels[959]=16'h05B8;
pixels[960]=16'h05B8;
pixels[961]=16'h05B8;
pixels[962]=16'h05B8;
pixels[963]=16'h05B8;
pixels[964]=16'h05B8;
pixels[965]=16'h05B8;
pixels[966]=16'h05B8;
pixels[967]=16'h05B8;
pixels[968]=16'h05B8;
pixels[969]=16'h05B8;
pixels[970]=16'h05B8;
pixels[971]=16'h05B8;
pixels[972]=16'h05B8;
pixels[973]=16'hDFBE;
pixels[974]=16'hFFFF;
pixels[975]=16'hFFFF;
pixels[976]=16'hFFFF;
pixels[977]=16'h4E5A;
pixels[978]=16'h05B8;
pixels[979]=16'h05B8;
pixels[980]=16'h05B8;
pixels[981]=16'h05B8;
pixels[982]=16'h05B8;
pixels[983]=16'h05B8;
pixels[984]=16'h05B8;
pixels[985]=16'h05B8;
pixels[986]=16'h05B8;
pixels[987]=16'h05B8;
pixels[988]=16'h05B8;
pixels[989]=16'h05B8;
pixels[990]=16'h05B8;
pixels[991]=16'h4E5A;
pixels[992]=16'hFFFF;
pixels[993]=16'hFFFF;
pixels[994]=16'hFFFF;
pixels[995]=16'hFFFF;
pixels[996]=16'hFFFF;
pixels[997]=16'hFFFF;
pixels[998]=16'hFFFF;
pixels[999]=16'hFFFF;
pixels[1000]=16'h3619;
pixels[1001]=16'h05B8;
pixels[1002]=16'h05B8;
pixels[1003]=16'h25F9;
pixels[1004]=16'hE7BF;
pixels[1005]=16'hFFFF;
pixels[1006]=16'hFFFF;
pixels[1007]=16'hFFFF;
pixels[1008]=16'hFFFF;
pixels[1009]=16'hFFFF;
pixels[1010]=16'hC77E;
pixels[1011]=16'h15D8;
pixels[1012]=16'h05B8;
pixels[1013]=16'h05B8;
pixels[1014]=16'h05B8;
pixels[1015]=16'h05B8;
pixels[1016]=16'h05B8;
pixels[1017]=16'h05B8;
pixels[1018]=16'h05B8;
pixels[1019]=16'h05B8;
pixels[1020]=16'h05B8;
pixels[1021]=16'h05B8;
pixels[1022]=16'h05B8;
pixels[1023]=16'h05B8;
pixels[1024]=16'h05B8;
pixels[1025]=16'h05B8;
pixels[1026]=16'h05B8;
pixels[1027]=16'h05B8;
pixels[1028]=16'h05B8;
pixels[1029]=16'h96FC;
pixels[1030]=16'hFFFF;
pixels[1031]=16'hFFFF;
pixels[1032]=16'hFFFF;
pixels[1033]=16'hFFFF;
pixels[1034]=16'hFFFF;
pixels[1035]=16'hFFFF;
pixels[1036]=16'hFFFF;
pixels[1037]=16'hFFFF;
pixels[1038]=16'hFFFF;
pixels[1039]=16'hFFFF;
pixels[1040]=16'hFFFF;
pixels[1041]=16'hFFFF;
pixels[1042]=16'hFFFF;
pixels[1043]=16'hFFFF;
pixels[1044]=16'hFFFF;
pixels[1045]=16'hFFFF;
pixels[1046]=16'hFFFF;
pixels[1047]=16'hFFFF;
pixels[1048]=16'hFFFF;
pixels[1049]=16'hFFFF;
pixels[1050]=16'hFFFF;
pixels[1051]=16'hFFFF;
pixels[1052]=16'hFFFF;
pixels[1053]=16'h971C;
pixels[1054]=16'h05B8;
pixels[1055]=16'h3639;
pixels[1056]=16'hBF5D;
pixels[1057]=16'hBF5D;
pixels[1058]=16'hBF5D;
pixels[1059]=16'hFFFF;
pixels[1060]=16'hFFFF;
pixels[1061]=16'hFFFF;
pixels[1062]=16'hC77D;
pixels[1063]=16'hBF5D;
pixels[1064]=16'h2E19;
pixels[1065]=16'h3639;
pixels[1066]=16'h76BB;
pixels[1067]=16'hC77D;
pixels[1068]=16'hFFFF;
pixels[1069]=16'hFFFF;
pixels[1070]=16'hFFFF;
pixels[1071]=16'hFFFF;
pixels[1072]=16'hFFFF;
pixels[1073]=16'hFFFF;
pixels[1074]=16'hD79E;
pixels[1075]=16'h971C;
pixels[1076]=16'h6E9B;
pixels[1077]=16'h4E5A;
pixels[1078]=16'h1DF9;
pixels[1079]=16'h05B8;
pixels[1080]=16'h05B8;
pixels[1081]=16'h05B8;
pixels[1082]=16'h05B8;
pixels[1083]=16'h05B8;
pixels[1084]=16'h05B8;
pixels[1085]=16'h05B8;
pixels[1086]=16'h05B8;
pixels[1087]=16'h05B8;
pixels[1088]=16'h05B8;
pixels[1089]=16'h05B8;
pixels[1090]=16'h05B8;
pixels[1091]=16'h05B8;
pixels[1092]=16'h05B8;
pixels[1093]=16'h96FC;
pixels[1094]=16'hFFFF;
pixels[1095]=16'hFFFF;
pixels[1096]=16'hFFFF;
pixels[1097]=16'hFFFF;
pixels[1098]=16'hFFFF;
pixels[1099]=16'hFFFF;
pixels[1100]=16'hFFFF;
pixels[1101]=16'hFFFF;
pixels[1102]=16'hFFFF;
pixels[1103]=16'hFFFF;
pixels[1104]=16'hFFFF;
pixels[1105]=16'hFFFF;
pixels[1106]=16'hFFFF;
pixels[1107]=16'hFFFF;
pixels[1108]=16'hFFFF;
pixels[1109]=16'hFFFF;
pixels[1110]=16'hFFFF;
pixels[1111]=16'hFFFF;
pixels[1112]=16'hFFFF;
pixels[1113]=16'hFFFF;
pixels[1114]=16'hFFFF;
pixels[1115]=16'hFFFF;
pixels[1116]=16'hFFFF;
pixels[1117]=16'h971C;
pixels[1118]=16'h05B8;
pixels[1119]=16'h05B8;
pixels[1120]=16'h05B8;
pixels[1121]=16'h05B8;
pixels[1122]=16'h05B8;
pixels[1123]=16'hFFFF;
pixels[1124]=16'hFFFF;
pixels[1125]=16'hFFFF;
pixels[1126]=16'h15D8;
pixels[1127]=16'h567A;
pixels[1128]=16'hFFFF;
pixels[1129]=16'hFFFF;
pixels[1130]=16'hFFFF;
pixels[1131]=16'hFFFF;
pixels[1132]=16'hFFFF;
pixels[1133]=16'hFFFF;
pixels[1134]=16'hFFFF;
pixels[1135]=16'hFFFF;
pixels[1136]=16'hFFFF;
pixels[1137]=16'hFFFF;
pixels[1138]=16'hFFFF;
pixels[1139]=16'hFFFF;
pixels[1140]=16'hFFFF;
pixels[1141]=16'hFFFF;
pixels[1142]=16'h76BB;
pixels[1143]=16'h05B8;
pixels[1144]=16'h05B8;
pixels[1145]=16'h05B8;
pixels[1146]=16'h05B8;
pixels[1147]=16'h05B8;
pixels[1148]=16'h05B8;
pixels[1149]=16'h05B8;
pixels[1150]=16'h05B8;
pixels[1151]=16'h05B8;
pixels[1152]=16'h05B8;
pixels[1153]=16'h05B8;
pixels[1154]=16'h05B8;
pixels[1155]=16'h05B8;
pixels[1156]=16'h05B8;
pixels[1157]=16'h96FC;
pixels[1158]=16'hFFFF;
pixels[1159]=16'hFFFF;
pixels[1160]=16'hFFFF;
pixels[1161]=16'hFFFF;
pixels[1162]=16'hFFFF;
pixels[1163]=16'hFFFF;
pixels[1164]=16'hFFFF;
pixels[1165]=16'hFFFF;
pixels[1166]=16'hFFFF;
pixels[1167]=16'hFFFF;
pixels[1168]=16'hFFFF;
pixels[1169]=16'hFFFF;
pixels[1170]=16'hFFFF;
pixels[1171]=16'hFFFF;
pixels[1172]=16'hFFFF;
pixels[1173]=16'hFFFF;
pixels[1174]=16'hFFFF;
pixels[1175]=16'hFFFF;
pixels[1176]=16'hFFFF;
pixels[1177]=16'hFFFF;
pixels[1178]=16'hFFFF;
pixels[1179]=16'hFFFF;
pixels[1180]=16'hFFFF;
pixels[1181]=16'h971C;
pixels[1182]=16'h05B8;
pixels[1183]=16'hCF7E;
pixels[1184]=16'hFFFF;
pixels[1185]=16'hAF3D;
pixels[1186]=16'h05B8;
pixels[1187]=16'hFFFF;
pixels[1188]=16'hFFFF;
pixels[1189]=16'hFFFF;
pixels[1190]=16'h15D8;
pixels[1191]=16'h0DB8;
pixels[1192]=16'hEFDF;
pixels[1193]=16'hFFFF;
pixels[1194]=16'hFFFF;
pixels[1195]=16'hFFFF;
pixels[1196]=16'hFFFF;
pixels[1197]=16'hBF5D;
pixels[1198]=16'h5E7A;
pixels[1199]=16'h3E39;
pixels[1200]=16'hA71D;
pixels[1201]=16'hF7DF;
pixels[1202]=16'hFFFF;
pixels[1203]=16'hFFFF;
pixels[1204]=16'hFFFF;
pixels[1205]=16'hFFFF;
pixels[1206]=16'h25F9;
pixels[1207]=16'h05B8;
pixels[1208]=16'h05B8;
pixels[1209]=16'h05B8;
pixels[1210]=16'h05B8;
pixels[1211]=16'h05B8;
pixels[1212]=16'h05B8;
pixels[1213]=16'h05B8;
pixels[1214]=16'h05B8;
pixels[1215]=16'h05B8;
pixels[1216]=16'h05B8;
pixels[1217]=16'h05B8;
pixels[1218]=16'h05B8;
pixels[1219]=16'h05B8;
pixels[1220]=16'h05B8;
pixels[1221]=16'h0DB8;
pixels[1222]=16'h15D8;
pixels[1223]=16'h15D8;
pixels[1224]=16'h15D8;
pixels[1225]=16'h15D8;
pixels[1226]=16'h25F9;
pixels[1227]=16'hF7DF;
pixels[1228]=16'hFFFF;
pixels[1229]=16'hFFFF;
pixels[1230]=16'hDFBE;
pixels[1231]=16'h1DD8;
pixels[1232]=16'h15D8;
pixels[1233]=16'h15D8;
pixels[1234]=16'h15D8;
pixels[1235]=16'h15D8;
pixels[1236]=16'h1DF8;
pixels[1237]=16'hF7DF;
pixels[1238]=16'hFFFF;
pixels[1239]=16'hFFFF;
pixels[1240]=16'hF7DF;
pixels[1241]=16'h1DF8;
pixels[1242]=16'h15D8;
pixels[1243]=16'h15D8;
pixels[1244]=16'h15D8;
pixels[1245]=16'h0DB8;
pixels[1246]=16'h05B8;
pixels[1247]=16'hCF7E;
pixels[1248]=16'hFFFF;
pixels[1249]=16'hAF3D;
pixels[1250]=16'h05B8;
pixels[1251]=16'hFFFF;
pixels[1252]=16'hFFFF;
pixels[1253]=16'hFFFF;
pixels[1254]=16'hFFFF;
pixels[1255]=16'hFFFF;
pixels[1256]=16'h86DC;
pixels[1257]=16'hD79E;
pixels[1258]=16'h9F1C;
pixels[1259]=16'h5E9A;
pixels[1260]=16'h1DF8;
pixels[1261]=16'h05B8;
pixels[1262]=16'h05B8;
pixels[1263]=16'h05B8;
pixels[1264]=16'h05B8;
pixels[1265]=16'h0DB8;
pixels[1266]=16'h4E5A;
pixels[1267]=16'h8EFC;
pixels[1268]=16'hC77E;
pixels[1269]=16'hD79E;
pixels[1270]=16'h05B8;
pixels[1271]=16'h05B8;
pixels[1272]=16'h05B8;
pixels[1273]=16'h05B8;
pixels[1274]=16'h05B8;
pixels[1275]=16'h05B8;
pixels[1276]=16'h05B8;
pixels[1277]=16'h05B8;
pixels[1278]=16'h05B8;
pixels[1279]=16'h05B8;
pixels[1280]=16'h05B8;
pixels[1281]=16'h05B8;
pixels[1282]=16'h05B8;
pixels[1283]=16'h05B8;
pixels[1284]=16'h05B8;
pixels[1285]=16'h05B8;
pixels[1286]=16'h05B8;
pixels[1287]=16'h05B8;
pixels[1288]=16'h05B8;
pixels[1289]=16'h05B8;
pixels[1290]=16'h9F1C;
pixels[1291]=16'hFFFF;
pixels[1292]=16'hFFFF;
pixels[1293]=16'hFFFF;
pixels[1294]=16'h4E5A;
pixels[1295]=16'h05B8;
pixels[1296]=16'h05B8;
pixels[1297]=16'h05B8;
pixels[1298]=16'h05B8;
pixels[1299]=16'h05B8;
pixels[1300]=16'h6E9B;
pixels[1301]=16'hFFFF;
pixels[1302]=16'hFFFF;
pixels[1303]=16'hFFFF;
pixels[1304]=16'h9F1C;
pixels[1305]=16'h05B8;
pixels[1306]=16'h05B8;
pixels[1307]=16'h05B8;
pixels[1308]=16'h05B8;
pixels[1309]=16'h05B8;
pixels[1310]=16'h05B8;
pixels[1311]=16'hCF7E;
pixels[1312]=16'hFFFF;
pixels[1313]=16'hAF3D;
pixels[1314]=16'h05B8;
pixels[1315]=16'hFFFF;
pixels[1316]=16'hFFFF;
pixels[1317]=16'hFFFF;
pixels[1318]=16'hFFFF;
pixels[1319]=16'hFFFF;
pixels[1320]=16'h05B8;
pixels[1321]=16'h05B8;
pixels[1322]=16'h05B8;
pixels[1323]=16'h05B8;
pixels[1324]=16'h05B8;
pixels[1325]=16'h05B8;
pixels[1326]=16'h05B8;
pixels[1327]=16'h05B8;
pixels[1328]=16'h05B8;
pixels[1329]=16'h05B8;
pixels[1330]=16'h05B8;
pixels[1331]=16'h05B8;
pixels[1332]=16'h05B8;
pixels[1333]=16'h05B8;
pixels[1334]=16'h05B8;
pixels[1335]=16'h05B8;
pixels[1336]=16'h05B8;
pixels[1337]=16'h05B8;
pixels[1338]=16'h05B8;
pixels[1339]=16'h05B8;
pixels[1340]=16'h05B8;
pixels[1341]=16'h05B8;
pixels[1342]=16'h05B8;
pixels[1343]=16'h05B8;
pixels[1344]=16'h05B8;
pixels[1345]=16'h05B8;
pixels[1346]=16'h05B8;
pixels[1347]=16'h05B8;
pixels[1348]=16'h05B8;
pixels[1349]=16'h05B8;
pixels[1350]=16'h05B8;
pixels[1351]=16'h05B8;
pixels[1352]=16'h05B8;
pixels[1353]=16'h3619;
pixels[1354]=16'hFFFF;
pixels[1355]=16'hFFFF;
pixels[1356]=16'hFFFF;
pixels[1357]=16'hE7BF;
pixels[1358]=16'h25F9;
pixels[1359]=16'h05B8;
pixels[1360]=16'h05B8;
pixels[1361]=16'h05B8;
pixels[1362]=16'h05B8;
pixels[1363]=16'h15D8;
pixels[1364]=16'hEFDF;
pixels[1365]=16'hFFFF;
pixels[1366]=16'hFFFF;
pixels[1367]=16'hFFFF;
pixels[1368]=16'h2E19;
pixels[1369]=16'h05B8;
pixels[1370]=16'h05B8;
pixels[1371]=16'h05B8;
pixels[1372]=16'h05B8;
pixels[1373]=16'h05B8;
pixels[1374]=16'h05B8;
pixels[1375]=16'hCF7E;
pixels[1376]=16'hFFFF;
pixels[1377]=16'hAF3D;
pixels[1378]=16'h05B8;
pixels[1379]=16'hFFFF;
pixels[1380]=16'hFFFF;
pixels[1381]=16'hFFFF;
pixels[1382]=16'hD79E;
pixels[1383]=16'hCF9E;
pixels[1384]=16'h05B8;
pixels[1385]=16'h669B;
pixels[1386]=16'hFFFF;
pixels[1387]=16'hFFFF;
pixels[1388]=16'hFFFF;
pixels[1389]=16'hFFFF;
pixels[1390]=16'hFFFF;
pixels[1391]=16'hFFFF;
pixels[1392]=16'hFFFF;
pixels[1393]=16'hFFFF;
pixels[1394]=16'hFFFF;
pixels[1395]=16'hFFFF;
pixels[1396]=16'hFFFF;
pixels[1397]=16'h0DD8;
pixels[1398]=16'h05B8;
pixels[1399]=16'h05B8;
pixels[1400]=16'h05B8;
pixels[1401]=16'h05B8;
pixels[1402]=16'h05B8;
pixels[1403]=16'h05B8;
pixels[1404]=16'h05B8;
pixels[1405]=16'h05B8;
pixels[1406]=16'h05B8;
pixels[1407]=16'h05B8;
pixels[1408]=16'h05B8;
pixels[1409]=16'h05B8;
pixels[1410]=16'h05B8;
pixels[1411]=16'h05B8;
pixels[1412]=16'h05B8;
pixels[1413]=16'h05B8;
pixels[1414]=16'h05B8;
pixels[1415]=16'h05B8;
pixels[1416]=16'h05B8;
pixels[1417]=16'hC77E;
pixels[1418]=16'hFFFF;
pixels[1419]=16'hFFFF;
pixels[1420]=16'hFFFF;
pixels[1421]=16'hFFFF;
pixels[1422]=16'hFFFF;
pixels[1423]=16'hBF5D;
pixels[1424]=16'h567A;
pixels[1425]=16'h05B8;
pixels[1426]=16'h05B8;
pixels[1427]=16'hBF5D;
pixels[1428]=16'hFFFF;
pixels[1429]=16'hFFFF;
pixels[1430]=16'hFFFF;
pixels[1431]=16'hA73D;
pixels[1432]=16'h05B8;
pixels[1433]=16'h05B8;
pixels[1434]=16'h05B8;
pixels[1435]=16'h05B8;
pixels[1436]=16'h05B8;
pixels[1437]=16'h05B8;
pixels[1438]=16'h05B8;
pixels[1439]=16'hCF7E;
pixels[1440]=16'hFFFF;
pixels[1441]=16'hAF3D;
pixels[1442]=16'h05B8;
pixels[1443]=16'hFFFF;
pixels[1444]=16'hFFFF;
pixels[1445]=16'hFFFF;
pixels[1446]=16'h15D8;
pixels[1447]=16'h05B8;
pixels[1448]=16'h05B8;
pixels[1449]=16'h669B;
pixels[1450]=16'hFFFF;
pixels[1451]=16'hFFFF;
pixels[1452]=16'hFFFF;
pixels[1453]=16'hFFFF;
pixels[1454]=16'hFFFF;
pixels[1455]=16'hFFFF;
pixels[1456]=16'hFFFF;
pixels[1457]=16'hFFFF;
pixels[1458]=16'hFFFF;
pixels[1459]=16'hFFFF;
pixels[1460]=16'hFFFF;
pixels[1461]=16'h0DD8;
pixels[1462]=16'h05B8;
pixels[1463]=16'h05B8;
pixels[1464]=16'h05B8;
pixels[1465]=16'h05B8;
pixels[1466]=16'h05B8;
pixels[1467]=16'h05B8;
pixels[1468]=16'h05B8;
pixels[1469]=16'h05B8;
pixels[1470]=16'h05B8;
pixels[1471]=16'h05B8;
pixels[1472]=16'h05B8;
pixels[1473]=16'h05B8;
pixels[1474]=16'h05B8;
pixels[1475]=16'h05B8;
pixels[1476]=16'h05B8;
pixels[1477]=16'h05B8;
pixels[1478]=16'h05B8;
pixels[1479]=16'h05B8;
pixels[1480]=16'h05B8;
pixels[1481]=16'h3619;
pixels[1482]=16'h971C;
pixels[1483]=16'hF7DF;
pixels[1484]=16'hFFFF;
pixels[1485]=16'hFFFF;
pixels[1486]=16'hFFFF;
pixels[1487]=16'hFFFF;
pixels[1488]=16'hFFFF;
pixels[1489]=16'hE7BF;
pixels[1490]=16'hCF7E;
pixels[1491]=16'hFFFF;
pixels[1492]=16'hFFFF;
pixels[1493]=16'hFFFF;
pixels[1494]=16'hEFDF;
pixels[1495]=16'h1DD8;
pixels[1496]=16'h05B8;
pixels[1497]=16'h05B8;
pixels[1498]=16'h05B8;
pixels[1499]=16'h05B8;
pixels[1500]=16'h05B8;
pixels[1501]=16'h05B8;
pixels[1502]=16'h05B8;
pixels[1503]=16'hCF7E;
pixels[1504]=16'hFFFF;
pixels[1505]=16'hAF3D;
pixels[1506]=16'h05B8;
pixels[1507]=16'hFFFF;
pixels[1508]=16'hFFFF;
pixels[1509]=16'hFFFF;
pixels[1510]=16'h15D8;
pixels[1511]=16'h05B8;
pixels[1512]=16'h05B8;
pixels[1513]=16'h669B;
pixels[1514]=16'hFFFF;
pixels[1515]=16'hFFFF;
pixels[1516]=16'hFFFF;
pixels[1517]=16'hFFFF;
pixels[1518]=16'hFFFF;
pixels[1519]=16'hFFFF;
pixels[1520]=16'hFFFF;
pixels[1521]=16'hFFFF;
pixels[1522]=16'hFFFF;
pixels[1523]=16'hFFFF;
pixels[1524]=16'hFFFF;
pixels[1525]=16'h0DD8;
pixels[1526]=16'h05B8;
pixels[1527]=16'h05B8;
pixels[1528]=16'h05B8;
pixels[1529]=16'h05B8;
pixels[1530]=16'h05B8;
pixels[1531]=16'h05B8;
pixels[1532]=16'h05B8;
pixels[1533]=16'h05B8;
pixels[1534]=16'h05B8;
pixels[1535]=16'h05B8;
pixels[1536]=16'h05B8;
pixels[1537]=16'h05B8;
pixels[1538]=16'h05B8;
pixels[1539]=16'h05B8;
pixels[1540]=16'h05B8;
pixels[1541]=16'h05B8;
pixels[1542]=16'h05B8;
pixels[1543]=16'h05B8;
pixels[1544]=16'h05B8;
pixels[1545]=16'h05B8;
pixels[1546]=16'h05B8;
pixels[1547]=16'h0DB8;
pixels[1548]=16'h669B;
pixels[1549]=16'hCF9E;
pixels[1550]=16'hFFFF;
pixels[1551]=16'hFFFF;
pixels[1552]=16'hFFFF;
pixels[1553]=16'hFFFF;
pixels[1554]=16'hFFFF;
pixels[1555]=16'hFFFF;
pixels[1556]=16'hFFFF;
pixels[1557]=16'hFFFF;
pixels[1558]=16'h4E5A;
pixels[1559]=16'h05B8;
pixels[1560]=16'h05B8;
pixels[1561]=16'h05B8;
pixels[1562]=16'h05B8;
pixels[1563]=16'h05B8;
pixels[1564]=16'h05B8;
pixels[1565]=16'h05B8;
pixels[1566]=16'h05B8;
pixels[1567]=16'hCF7E;
pixels[1568]=16'hFFFF;
pixels[1569]=16'hAF3D;
pixels[1570]=16'h05B8;
pixels[1571]=16'hFFFF;
pixels[1572]=16'hFFFF;
pixels[1573]=16'hFFFF;
pixels[1574]=16'h15D8;
pixels[1575]=16'h05B8;
pixels[1576]=16'h05B8;
pixels[1577]=16'h669B;
pixels[1578]=16'hFFFF;
pixels[1579]=16'hFFFF;
pixels[1580]=16'hB75D;
pixels[1581]=16'h0DB8;
pixels[1582]=16'h0DB8;
pixels[1583]=16'h0DB8;
pixels[1584]=16'h0DB8;
pixels[1585]=16'h0DD8;
pixels[1586]=16'hFFFF;
pixels[1587]=16'hFFFF;
pixels[1588]=16'hFFFF;
pixels[1589]=16'h0DD8;
pixels[1590]=16'h05B8;
pixels[1591]=16'h05B8;
pixels[1592]=16'h05B8;
pixels[1593]=16'h05B8;
pixels[1594]=16'h05B8;
pixels[1595]=16'h05B8;
pixels[1596]=16'h05B8;
pixels[1597]=16'h05B8;
pixels[1598]=16'h05B8;
pixels[1599]=16'h05B8;
pixels[1600]=16'h05B8;
pixels[1601]=16'h05B8;
pixels[1602]=16'h05B8;
pixels[1603]=16'h05B8;
pixels[1604]=16'h05B8;
pixels[1605]=16'h05B8;
pixels[1606]=16'h05B8;
pixels[1607]=16'h05B8;
pixels[1608]=16'h05B8;
pixels[1609]=16'h05B8;
pixels[1610]=16'h05B8;
pixels[1611]=16'h05B8;
pixels[1612]=16'h05B8;
pixels[1613]=16'h2E19;
pixels[1614]=16'hC77E;
pixels[1615]=16'hFFFF;
pixels[1616]=16'hFFFF;
pixels[1617]=16'hFFFF;
pixels[1618]=16'hFFFF;
pixels[1619]=16'hFFFF;
pixels[1620]=16'hFFFF;
pixels[1621]=16'hFFFF;
pixels[1622]=16'hCF7E;
pixels[1623]=16'h5E7A;
pixels[1624]=16'h05B8;
pixels[1625]=16'h05B8;
pixels[1626]=16'h05B8;
pixels[1627]=16'h05B8;
pixels[1628]=16'h05B8;
pixels[1629]=16'h05B8;
pixels[1630]=16'h05B8;
pixels[1631]=16'hCF7E;
pixels[1632]=16'hFFFF;
pixels[1633]=16'hAF3D;
pixels[1634]=16'h05B8;
pixels[1635]=16'hFFFF;
pixels[1636]=16'hFFFF;
pixels[1637]=16'hFFFF;
pixels[1638]=16'hC77E;
pixels[1639]=16'hDFBE;
pixels[1640]=16'hBF5D;
pixels[1641]=16'h669B;
pixels[1642]=16'hFFFF;
pixels[1643]=16'hFFFF;
pixels[1644]=16'hB75D;
pixels[1645]=16'h05B8;
pixels[1646]=16'h05B8;
pixels[1647]=16'h05B8;
pixels[1648]=16'h05B8;
pixels[1649]=16'h0DB8;
pixels[1650]=16'hFFFF;
pixels[1651]=16'hFFFF;
pixels[1652]=16'hFFFF;
pixels[1653]=16'h0DD8;
pixels[1654]=16'h05B8;
pixels[1655]=16'h05B8;
pixels[1656]=16'h05B8;
pixels[1657]=16'h05B8;
pixels[1658]=16'h05B8;
pixels[1659]=16'h05B8;
pixels[1660]=16'h05B8;
pixels[1661]=16'h05B8;
pixels[1662]=16'h05B8;
pixels[1663]=16'h05B8;
pixels[1664]=16'h05B8;
pixels[1665]=16'h05B8;
pixels[1666]=16'h05B8;
pixels[1667]=16'h05B8;
pixels[1668]=16'h05B8;
pixels[1669]=16'h05B8;
pixels[1670]=16'h05B8;
pixels[1671]=16'h05B8;
pixels[1672]=16'h05B8;
pixels[1673]=16'h05B8;
pixels[1674]=16'h25F9;
pixels[1675]=16'h6EBB;
pixels[1676]=16'hC77E;
pixels[1677]=16'hFFFF;
pixels[1678]=16'hFFFF;
pixels[1679]=16'hFFFF;
pixels[1680]=16'hFFFF;
pixels[1681]=16'hFFFF;
pixels[1682]=16'hFFFF;
pixels[1683]=16'hFFFF;
pixels[1684]=16'hFFFF;
pixels[1685]=16'hFFFF;
pixels[1686]=16'hFFFF;
pixels[1687]=16'hFFFF;
pixels[1688]=16'hE7DF;
pixels[1689]=16'h7EDB;
pixels[1690]=16'h15D8;
pixels[1691]=16'h05B8;
pixels[1692]=16'h05B8;
pixels[1693]=16'h05B8;
pixels[1694]=16'h463A;
pixels[1695]=16'hE7BF;
pixels[1696]=16'hFFFF;
pixels[1697]=16'hEFDF;
pixels[1698]=16'hDFBE;
pixels[1699]=16'hFFFF;
pixels[1700]=16'hFFFF;
pixels[1701]=16'hFFFF;
pixels[1702]=16'hFFFF;
pixels[1703]=16'hFFFF;
pixels[1704]=16'hB75D;
pixels[1705]=16'h669B;
pixels[1706]=16'hFFFF;
pixels[1707]=16'hFFFF;
pixels[1708]=16'hB75D;
pixels[1709]=16'h05B8;
pixels[1710]=16'h05B8;
pixels[1711]=16'h05B8;
pixels[1712]=16'h05B8;
pixels[1713]=16'h0DB8;
pixels[1714]=16'hFFFF;
pixels[1715]=16'hFFFF;
pixels[1716]=16'hFFFF;
pixels[1717]=16'h0DD8;
pixels[1718]=16'h05B8;
pixels[1719]=16'h05B8;
pixels[1720]=16'h05B8;
pixels[1721]=16'h05B8;
pixels[1722]=16'h05B8;
pixels[1723]=16'h05B8;
pixels[1724]=16'h05B8;
pixels[1725]=16'h05B8;
pixels[1726]=16'h05B8;
pixels[1727]=16'h05B8;
pixels[1728]=16'h05B8;
pixels[1729]=16'h05B8;
pixels[1730]=16'h05B8;
pixels[1731]=16'h05B8;
pixels[1732]=16'h05B8;
pixels[1733]=16'h0DB8;
pixels[1734]=16'h3639;
pixels[1735]=16'h669B;
pixels[1736]=16'h9F1C;
pixels[1737]=16'hDFBE;
pixels[1738]=16'hFFFF;
pixels[1739]=16'hFFFF;
pixels[1740]=16'hFFFF;
pixels[1741]=16'hFFFF;
pixels[1742]=16'hFFFF;
pixels[1743]=16'hFFFF;
pixels[1744]=16'hFFFF;
pixels[1745]=16'hC77E;
pixels[1746]=16'h465A;
pixels[1747]=16'h5E9A;
pixels[1748]=16'hD79E;
pixels[1749]=16'hFFFF;
pixels[1750]=16'hFFFF;
pixels[1751]=16'hFFFF;
pixels[1752]=16'hFFFF;
pixels[1753]=16'hFFFF;
pixels[1754]=16'hFFFF;
pixels[1755]=16'hA71D;
pixels[1756]=16'h3619;
pixels[1757]=16'h05B8;
pixels[1758]=16'h7EDB;
pixels[1759]=16'hFFFF;
pixels[1760]=16'hFFFF;
pixels[1761]=16'hFFFF;
pixels[1762]=16'hFFFF;
pixels[1763]=16'hFFFF;
pixels[1764]=16'hFFFF;
pixels[1765]=16'hFFFF;
pixels[1766]=16'hFFFF;
pixels[1767]=16'hFFFF;
pixels[1768]=16'hB75D;
pixels[1769]=16'h669B;
pixels[1770]=16'hFFFF;
pixels[1771]=16'hFFFF;
pixels[1772]=16'hFFFF;
pixels[1773]=16'hFFFF;
pixels[1774]=16'hFFFF;
pixels[1775]=16'hFFFF;
pixels[1776]=16'hFFFF;
pixels[1777]=16'hFFFF;
pixels[1778]=16'hFFFF;
pixels[1779]=16'hFFFF;
pixels[1780]=16'hFFFF;
pixels[1781]=16'h0DD8;
pixels[1782]=16'h05B8;
pixels[1783]=16'h05B8;
pixels[1784]=16'h05B8;
pixels[1785]=16'h05B8;
pixels[1786]=16'h05B8;
pixels[1787]=16'h05B8;
pixels[1788]=16'h05B8;
pixels[1789]=16'h05B8;
pixels[1790]=16'h05B8;
pixels[1791]=16'h05B8;
pixels[1792]=16'h05B8;
pixels[1793]=16'h05B8;
pixels[1794]=16'h05B8;
pixels[1795]=16'h05B8;
pixels[1796]=16'h05B8;
pixels[1797]=16'h465A;
pixels[1798]=16'hFFFF;
pixels[1799]=16'hFFFF;
pixels[1800]=16'hFFFF;
pixels[1801]=16'hFFFF;
pixels[1802]=16'hFFFF;
pixels[1803]=16'hFFFF;
pixels[1804]=16'hFFFF;
pixels[1805]=16'hFFFF;
pixels[1806]=16'hFFFF;
pixels[1807]=16'hAF3D;
pixels[1808]=16'h465A;
pixels[1809]=16'h05B8;
pixels[1810]=16'h05B8;
pixels[1811]=16'h05B8;
pixels[1812]=16'h05B8;
pixels[1813]=16'h567A;
pixels[1814]=16'hCF7E;
pixels[1815]=16'hFFFF;
pixels[1816]=16'hFFFF;
pixels[1817]=16'hFFFF;
pixels[1818]=16'hFFFF;
pixels[1819]=16'hFFFF;
pixels[1820]=16'h9F1C;
pixels[1821]=16'h05B8;
pixels[1822]=16'h669B;
pixels[1823]=16'hFFFF;
pixels[1824]=16'hFFFF;
pixels[1825]=16'hFFFF;
pixels[1826]=16'hFFFF;
pixels[1827]=16'hFFFF;
pixels[1828]=16'hFFFF;
pixels[1829]=16'hD79E;
pixels[1830]=16'hA73D;
pixels[1831]=16'h76BB;
pixels[1832]=16'h3639;
pixels[1833]=16'h669B;
pixels[1834]=16'hFFFF;
pixels[1835]=16'hFFFF;
pixels[1836]=16'hFFFF;
pixels[1837]=16'hFFFF;
pixels[1838]=16'hFFFF;
pixels[1839]=16'hFFFF;
pixels[1840]=16'hFFFF;
pixels[1841]=16'hFFFF;
pixels[1842]=16'hFFFF;
pixels[1843]=16'hFFFF;
pixels[1844]=16'hFFFF;
pixels[1845]=16'h0DD8;
pixels[1846]=16'h05B8;
pixels[1847]=16'h05B8;
pixels[1848]=16'h05B8;
pixels[1849]=16'h05B8;
pixels[1850]=16'h05B8;
pixels[1851]=16'h05B8;
pixels[1852]=16'h05B8;
pixels[1853]=16'h05B8;
pixels[1854]=16'h05B8;
pixels[1855]=16'h05B8;
pixels[1856]=16'h05B8;
pixels[1857]=16'h05B8;
pixels[1858]=16'h05B8;
pixels[1859]=16'h05B8;
pixels[1860]=16'h05B8;
pixels[1861]=16'h05B8;
pixels[1862]=16'hAF3D;
pixels[1863]=16'hFFFF;
pixels[1864]=16'hFFFF;
pixels[1865]=16'hFFFF;
pixels[1866]=16'hFFFF;
pixels[1867]=16'hF7FF;
pixels[1868]=16'hB75D;
pixels[1869]=16'h6EBB;
pixels[1870]=16'h15D8;
pixels[1871]=16'h05B8;
pixels[1872]=16'h05B8;
pixels[1873]=16'h05B8;
pixels[1874]=16'h05B8;
pixels[1875]=16'h05B8;
pixels[1876]=16'h05B8;
pixels[1877]=16'h05B8;
pixels[1878]=16'h05B8;
pixels[1879]=16'h465A;
pixels[1880]=16'hBF5D;
pixels[1881]=16'hFFFF;
pixels[1882]=16'hFFFF;
pixels[1883]=16'hF7DF;
pixels[1884]=16'h1DD8;
pixels[1885]=16'h05B8;
pixels[1886]=16'h465A;
pixels[1887]=16'hE7BE;
pixels[1888]=16'hB75D;
pixels[1889]=16'h86DC;
pixels[1890]=16'h5E7A;
pixels[1891]=16'h2E19;
pixels[1892]=16'h05B8;
pixels[1893]=16'h05B8;
pixels[1894]=16'h05B8;
pixels[1895]=16'h05B8;
pixels[1896]=16'h05B8;
pixels[1897]=16'h669B;
pixels[1898]=16'hFFFF;
pixels[1899]=16'hFFFF;
pixels[1900]=16'hFFFF;
pixels[1901]=16'hFFFF;
pixels[1902]=16'hFFFF;
pixels[1903]=16'hFFFF;
pixels[1904]=16'hFFFF;
pixels[1905]=16'hFFFF;
pixels[1906]=16'hFFFF;
pixels[1907]=16'hFFFF;
pixels[1908]=16'hFFFF;
pixels[1909]=16'h0DD8;
pixels[1910]=16'h05B8;
pixels[1911]=16'h05B8;
pixels[1912]=16'h05B8;
pixels[1913]=16'h05B8;
pixels[1914]=16'h05B8;
pixels[1915]=16'h05B8;
pixels[1916]=16'h05B8;
pixels[1917]=16'h05B8;
pixels[1918]=16'h05B8;
pixels[1919]=16'h05B8;
pixels[1920]=16'h05B8;
pixels[1921]=16'h05B8;
pixels[1922]=16'h05B8;
pixels[1923]=16'h05B8;
pixels[1924]=16'h05B8;
pixels[1925]=16'h05B8;
pixels[1926]=16'h25F9;
pixels[1927]=16'hE7DF;
pixels[1928]=16'hB75D;
pixels[1929]=16'h7EDB;
pixels[1930]=16'h465A;
pixels[1931]=16'h0DB8;
pixels[1932]=16'h05B8;
pixels[1933]=16'h05B8;
pixels[1934]=16'h05B8;
pixels[1935]=16'h05B8;
pixels[1936]=16'h05B8;
pixels[1937]=16'h05B8;
pixels[1938]=16'h05B8;
pixels[1939]=16'h05B8;
pixels[1940]=16'h05B8;
pixels[1941]=16'h05B8;
pixels[1942]=16'h05B8;
pixels[1943]=16'h05B8;
pixels[1944]=16'h05B8;
pixels[1945]=16'h3639;
pixels[1946]=16'hAF3D;
pixels[1947]=16'h76BB;
pixels[1948]=16'h05B8;
pixels[1949]=16'h05B8;
pixels[1950]=16'h05B8;
pixels[1951]=16'h05B8;
pixels[1952]=16'h05B8;
pixels[1953]=16'h05B8;
pixels[1954]=16'h05B8;
pixels[1955]=16'h05B8;
pixels[1956]=16'h05B8;
pixels[1957]=16'h05B8;
pixels[1958]=16'h05B8;
pixels[1959]=16'h05B8;
pixels[1960]=16'h05B8;
pixels[1961]=16'h669B;
pixels[1962]=16'hFFFF;
pixels[1963]=16'hFFFF;
pixels[1964]=16'hB75D;
pixels[1965]=16'h0DB8;
pixels[1966]=16'h0DB8;
pixels[1967]=16'h0DB8;
pixels[1968]=16'h0DB8;
pixels[1969]=16'h0DD8;
pixels[1970]=16'hF7FF;
pixels[1971]=16'hF7FF;
pixels[1972]=16'hF7FF;
pixels[1973]=16'h0DB8;
pixels[1974]=16'h05B8;
pixels[1975]=16'h05B8;
pixels[1976]=16'h05B8;
pixels[1977]=16'h05B8;
pixels[1978]=16'h05B8;
pixels[1979]=16'h05B8;
pixels[1980]=16'h05B8;
pixels[1981]=16'h05B8;
pixels[1982]=16'h05B8;
pixels[1983]=16'h05B8;
pixels[1984]=16'h05B8;
pixels[1985]=16'h05B8;
pixels[1986]=16'h05B8;
pixels[1987]=16'h05B8;
pixels[1988]=16'h05B8;
pixels[1989]=16'h05B8;
pixels[1990]=16'h05B8;
pixels[1991]=16'h05B8;
pixels[1992]=16'h05B8;
pixels[1993]=16'h05B8;
pixels[1994]=16'h05B8;
pixels[1995]=16'h05B8;
pixels[1996]=16'h05B8;
pixels[1997]=16'h05B8;
pixels[1998]=16'h05B8;
pixels[1999]=16'h05B8;
pixels[2000]=16'h05B8;
pixels[2001]=16'h05B8;
pixels[2002]=16'h05B8;
pixels[2003]=16'h05B8;
pixels[2004]=16'h05B8;
pixels[2005]=16'h05B8;
pixels[2006]=16'h05B8;
pixels[2007]=16'h05B8;
pixels[2008]=16'h05B8;
pixels[2009]=16'h05B8;
pixels[2010]=16'h05B8;
pixels[2011]=16'h05B8;
pixels[2012]=16'h05B8;
pixels[2013]=16'h05B8;
pixels[2014]=16'h05B8;
pixels[2015]=16'h05B8;
pixels[2016]=16'h05B8;
pixels[2017]=16'h05B8;
pixels[2018]=16'h05B8;
pixels[2019]=16'h05B8;
pixels[2020]=16'h05B8;
pixels[2021]=16'h05B8;
pixels[2022]=16'h05B8;
pixels[2023]=16'h05B8;
pixels[2024]=16'h05B8;
pixels[2025]=16'h05B8;
pixels[2026]=16'h05B8;
pixels[2027]=16'h05B8;
pixels[2028]=16'h05B8;
pixels[2029]=16'h05B8;
pixels[2030]=16'h05B8;
pixels[2031]=16'h05B8;
pixels[2032]=16'h05B8;
pixels[2033]=16'h05B8;
pixels[2034]=16'h05B8;
pixels[2035]=16'h05B8;
pixels[2036]=16'h05B8;
pixels[2037]=16'h05B8;
pixels[2038]=16'h05B8;
pixels[2039]=16'h05B8;
pixels[2040]=16'h05B8;
pixels[2041]=16'h05B8;
pixels[2042]=16'h05B8;
pixels[2043]=16'h05B8;
pixels[2044]=16'h05B8;
pixels[2045]=16'h05B8;
pixels[2046]=16'h05B8;
pixels[2047]=16'h05B8;
end
always @(posedge clk) logo_q<=pixels[addr];
reg [23:0] bg;
reg active;
reg hq,vq,dq;
reg [1:0] hp,vp,dp;
wire [7:0] lr={logo[15:11],logo[15:13]};
wire [7:0] lg={logo[10:5],logo[10:9]};
wire [7:0] lb={logo[4:0],logo[4:2]};
wire [16:0] mul_r=bg[23:16]*{1'b0,lr};
wire [16:0] mul_g=bg[15:8]*{1'b0,lg};
wire [16:0] mul_b=bg[7:0]*{1'b0,lb};
wire [7:0] fr=blend?(lr==255?bg[23:16]:mul_r[15:8]):lr;
wire [7:0] fg=blend?(lg==255?bg[15:8]:mul_g[15:8]):lg;
wire [7:0] fb=blend?(lb==255?bg[7:0]:mul_b[15:8]):lb;
wire signed [8:0] mix_delta_r=$signed({1'b0,fr})-$signed({1'b0,bg[23:16]});
wire signed [18:0] mix_product_r=mix_delta_r*$signed({1'b0,a});
wire signed [18:0] mix_r=$signed({3'd0,bg[23:16],8'd0})+mix_product_r;
wire signed [8:0] mix_delta_g=$signed({1'b0,fg})-$signed({1'b0,bg[15:8]});
wire signed [18:0] mix_product_g=mix_delta_g*$signed({1'b0,a});
wire signed [18:0] mix_g=$signed({3'd0,bg[15:8],8'd0})+mix_product_g;
wire signed [8:0] mix_delta_b=$signed({1'b0,fb})-$signed({1'b0,bg[7:0]});
wire signed [18:0] mix_product_b=mix_delta_b*$signed({1'b0,a});
wire signed [18:0] mix_b=$signed({3'd0,bg[7:0],8'd0})+mix_product_b;
assign ohs=hp[1];assign ovs=vp[1];assign ode=dp[1];
always @(posedge clk or posedge rst) begin
    if(rst) begin
        de_d<=0;vs_d<=0;x<=0;y<=0;bg<=0;active<=0;hp<=0;vp<=0;dp<=0;rgb<=0;gallery_scroll<=0;gallery_reverse<=0;gallery_fraction<=0;
    end else begin
        de_d<=ide;vs_d<=ivs;
        if(vs_d && !ivs && fullscreen && enable && !compare && opacity!=0)begin
            if(gallery_step>=60)begin
                gallery_fraction<=gallery_step-60;
                if(gallery_reverse)begin
                    if(gallery_scroll==0)begin gallery_reverse<=0;gallery_scroll<=1;end
                    else gallery_scroll<=gallery_scroll-1'b1;
                end else begin
                    if(gallery_scroll==560)begin gallery_reverse<=1;gallery_scroll<=559;end
                    else gallery_scroll<=gallery_scroll+1'b1;
                end
            end else gallery_fraction<=gallery_step;
        end
        if(vs_d && !ivs)y<=0;else if(de_d && !ide)y<=y+1;
        if(!ide)x<=0;else x<=x+1;
        hp<={hp[0],ihs};vp<={vp[0],ivs};dp<={dp[0],ide};
        bg<=irgb;active<=enable && !compare && inside_layer && ide;
        if(!active || opacity==0)rgb<=bg;
        else if(a==256)rgb<={fr,fg,fb};
        else rgb<={mix_r[15:8],mix_g[15:8],mix_b[15:8]};
    end
end
endmodule

module exhibit_panel (
    input wire clk,rst,vs,de,enable,
    input wire [23:0] irgb,input wire [92:0] cfg,
    input wire volume_show,input wire [3:0] volume,input wire media_video,input wire [9:0] video_panel_x,video_panel_y,
    output reg [23:0] rgb
);
wire [3:0] mode;
wire signed [7:0] brightness;
wire [7:0] contrast,saturation,zoom;
wire signed [4:0] tone;
wire signed [9:0] pan_x,pan_y;
wire fit,blend,compare;
wire [6:0] opacity;
wire [5:0] speed;
wire [1:0] position,transition,direction,tuning;
wire [3:0] duration,period;
assign {mode,brightness,contrast,tone,saturation,zoom,pan_x,pan_y,fit,
    opacity,blend,speed,position,transition,duration,period,direction,compare,tuning}=cfg;
reg de_d,vs_d;
reg [9:0] x,y;
reg [10:0] addr;
reg [15:0] row;
(* rom_style="bram" *) reg [15:0] font [0:2047];
initial begin
font[0]=16'h0000;
font[1]=16'h0000;
font[2]=16'h0000;
font[3]=16'h0000;
font[4]=16'h0000;
font[5]=16'h0000;
font[6]=16'h0000;
font[7]=16'h0000;
font[8]=16'h0000;
font[9]=16'h0000;
font[10]=16'h0000;
font[11]=16'h0000;
font[12]=16'h0000;
font[13]=16'h0000;
font[14]=16'h0000;
font[15]=16'h0000;
font[16]=16'h0000;
font[17]=16'h0000;
font[18]=16'h0000;
font[19]=16'h03E0;
font[20]=16'h0770;
font[21]=16'h0630;
font[22]=16'h0630;
font[23]=16'h0630;
font[24]=16'h0630;
font[25]=16'h0630;
font[26]=16'h0770;
font[27]=16'h03E0;
font[28]=16'h01C0;
font[29]=16'h0000;
font[30]=16'h0000;
font[31]=16'h0000;
font[32]=16'h0000;
font[33]=16'h0000;
font[34]=16'h0000;
font[35]=16'h03C0;
font[36]=16'h02C0;
font[37]=16'h00C0;
font[38]=16'h00C0;
font[39]=16'h00C0;
font[40]=16'h00C0;
font[41]=16'h00C0;
font[42]=16'h00C0;
font[43]=16'h00C0;
font[44]=16'h00C0;
font[45]=16'h0000;
font[46]=16'h0000;
font[47]=16'h0000;
font[48]=16'h0000;
font[49]=16'h0000;
font[50]=16'h0000;
font[51]=16'h07F0;
font[52]=16'h0430;
font[53]=16'h0030;
font[54]=16'h0070;
font[55]=16'h00E0;
font[56]=16'h01C0;
font[57]=16'h0380;
font[58]=16'h0700;
font[59]=16'h07F0;
font[60]=16'h07F0;
font[61]=16'h0000;
font[62]=16'h0000;
font[63]=16'h0000;
font[64]=16'h0000;
font[65]=16'h0000;
font[66]=16'h0000;
font[67]=16'h07E0;
font[68]=16'h0060;
font[69]=16'h0060;
font[70]=16'h03C0;
font[71]=16'h03E0;
font[72]=16'h0070;
font[73]=16'h0030;
font[74]=16'h0030;
font[75]=16'h07E0;
font[76]=16'h07C0;
font[77]=16'h0000;
font[78]=16'h0000;
font[79]=16'h0000;
font[80]=16'h0000;
font[81]=16'h0000;
font[82]=16'h0000;
font[83]=16'h00E0;
font[84]=16'h01E0;
font[85]=16'h01E0;
font[86]=16'h0360;
font[87]=16'h0660;
font[88]=16'h0C60;
font[89]=16'h0FF8;
font[90]=16'h0FF8;
font[91]=16'h0060;
font[92]=16'h0060;
font[93]=16'h0000;
font[94]=16'h0000;
font[95]=16'h0000;
font[96]=16'h0000;
font[97]=16'h0000;
font[98]=16'h0000;
font[99]=16'h07F0;
font[100]=16'h0600;
font[101]=16'h0600;
font[102]=16'h07C0;
font[103]=16'h07F0;
font[104]=16'h0030;
font[105]=16'h0030;
font[106]=16'h0030;
font[107]=16'h07E0;
font[108]=16'h07C0;
font[109]=16'h0000;
font[110]=16'h0000;
font[111]=16'h0000;
font[112]=16'h0000;
font[113]=16'h0000;
font[114]=16'h0000;
font[115]=16'h03E0;
font[116]=16'h0300;
font[117]=16'h0600;
font[118]=16'h06E0;
font[119]=16'h07F0;
font[120]=16'h0630;
font[121]=16'h0630;
font[122]=16'h0630;
font[123]=16'h03E0;
font[124]=16'h01C0;
font[125]=16'h0000;
font[126]=16'h0000;
font[127]=16'h0000;
font[128]=16'h0000;
font[129]=16'h0000;
font[130]=16'h0000;
font[131]=16'h0FF0;
font[132]=16'h0060;
font[133]=16'h0060;
font[134]=16'h00C0;
font[135]=16'h00C0;
font[136]=16'h0180;
font[137]=16'h0180;
font[138]=16'h0300;
font[139]=16'h0300;
font[140]=16'h0300;
font[141]=16'h0000;
font[142]=16'h0000;
font[143]=16'h0000;
font[144]=16'h0000;
font[145]=16'h0000;
font[146]=16'h0000;
font[147]=16'h07F0;
font[148]=16'h0630;
font[149]=16'h0630;
font[150]=16'h03E0;
font[151]=16'h03E0;
font[152]=16'h0770;
font[153]=16'h0630;
font[154]=16'h0630;
font[155]=16'h07F0;
font[156]=16'h03E0;
font[157]=16'h0000;
font[158]=16'h0000;
font[159]=16'h0000;
font[160]=16'h0000;
font[161]=16'h0000;
font[162]=16'h0000;
font[163]=16'h03E0;
font[164]=16'h0630;
font[165]=16'h0630;
font[166]=16'h0730;
font[167]=16'h03F0;
font[168]=16'h01B0;
font[169]=16'h0030;
font[170]=16'h0060;
font[171]=16'h03E0;
font[172]=16'h03C0;
font[173]=16'h0000;
font[174]=16'h0000;
font[175]=16'h0000;
font[176]=16'h0000;
font[177]=16'h0000;
font[178]=16'h0000;
font[179]=16'h0000;
font[180]=16'h01C0;
font[181]=16'h01C0;
font[182]=16'h0000;
font[183]=16'h0000;
font[184]=16'h0000;
font[185]=16'h0000;
font[186]=16'h01C0;
font[187]=16'h01C0;
font[188]=16'h0000;
font[189]=16'h0000;
font[190]=16'h0000;
font[191]=16'h0000;
font[192]=16'h0000;
font[193]=16'h0000;
font[194]=16'h0000;
font[195]=16'h0000;
font[196]=16'h0080;
font[197]=16'h0080;
font[198]=16'h0080;
font[199]=16'h07F0;
font[200]=16'h07F0;
font[201]=16'h0080;
font[202]=16'h0080;
font[203]=16'h0000;
font[204]=16'h0000;
font[205]=16'h0000;
font[206]=16'h0000;
font[207]=16'h0000;
font[208]=16'h0000;
font[209]=16'h0000;
font[210]=16'h0000;
font[211]=16'h0000;
font[212]=16'h0000;
font[213]=16'h0000;
font[214]=16'h03C0;
font[215]=16'h0000;
font[216]=16'h0000;
font[217]=16'h0000;
font[218]=16'h0000;
font[219]=16'h0000;
font[220]=16'h0000;
font[221]=16'h0000;
font[222]=16'h0000;
font[223]=16'h0000;
font[224]=16'h0000;
font[225]=16'h0000;
font[226]=16'h0000;
font[227]=16'h1B70;
font[228]=16'h1160;
font[229]=16'h11E0;
font[230]=16'h0EC0;
font[231]=16'h01F8;
font[232]=16'h01EC;
font[233]=16'h03C4;
font[234]=16'h0344;
font[235]=16'h076C;
font[236]=16'h0638;
font[237]=16'h0000;
font[238]=16'h0000;
font[239]=16'h0000;
font[240]=16'h0000;
font[241]=16'h0000;
font[242]=16'h0000;
font[243]=16'h0000;
font[244]=16'h0000;
font[245]=16'h0000;
font[246]=16'h0000;
font[247]=16'h01C0;
font[248]=16'h01C0;
font[249]=16'h0000;
font[250]=16'h0000;
font[251]=16'h0000;
font[252]=16'h0000;
font[253]=16'h0000;
font[254]=16'h0000;
font[255]=16'h0000;
font[256]=16'h0000;
font[257]=16'h0000;
font[258]=16'h1860;
font[259]=16'h1B64;
font[260]=16'h1B6C;
font[261]=16'h7FE8;
font[262]=16'h7DE0;
font[263]=16'h3FFC;
font[264]=16'h3FFC;
font[265]=16'h7FFC;
font[266]=16'h19FC;
font[267]=16'h180C;
font[268]=16'h1BFC;
font[269]=16'h1BFC;
font[270]=16'h0000;
font[271]=16'h0000;
font[272]=16'h0000;
font[273]=16'h0000;
font[274]=16'h0C60;
font[275]=16'h7E60;
font[276]=16'h39F8;
font[277]=16'h19EC;
font[278]=16'h7FE6;
font[279]=16'h7F64;
font[280]=16'h3F6C;
font[281]=16'h3E7C;
font[282]=16'h7E38;
font[283]=16'h1870;
font[284]=16'h1BE0;
font[285]=16'h1B00;
font[286]=16'h0000;
font[287]=16'h0000;
font[288]=16'h0000;
font[289]=16'h0000;
font[290]=16'h7FFE;
font[291]=16'h0000;
font[292]=16'h1FF8;
font[293]=16'h1818;
font[294]=16'h1FF8;
font[295]=16'h0000;
font[296]=16'h3FFC;
font[297]=16'h300C;
font[298]=16'h37EE;
font[299]=16'h3E6E;
font[300]=16'h3C7C;
font[301]=16'h3838;
font[302]=16'h0000;
font[303]=16'h0000;
font[304]=16'h0000;
font[305]=16'h0000;
font[306]=16'h3FFE;
font[307]=16'h3FFC;
font[308]=16'h3FFE;
font[309]=16'h37FC;
font[310]=16'h33F8;
font[311]=16'h3000;
font[312]=16'h3FF8;
font[313]=16'h37F8;
font[314]=16'h33B0;
font[315]=16'h73FC;
font[316]=16'h7F1C;
font[317]=16'h2000;
font[318]=16'h0000;
font[319]=16'h0000;
font[320]=16'h0000;
font[321]=16'h0000;
font[322]=16'h7E0C;
font[323]=16'h7FFE;
font[324]=16'h26FE;
font[325]=16'h360C;
font[326]=16'h3E8C;
font[327]=16'h1DCC;
font[328]=16'h0CEC;
font[329]=16'h1E4C;
font[330]=16'h1F0C;
font[331]=16'h321C;
font[332]=16'h607C;
font[333]=16'h2078;
font[334]=16'h0000;
font[335]=16'h0000;
font[336]=16'h0000;
font[337]=16'h0000;
font[338]=16'h0000;
font[339]=16'h30C0;
font[340]=16'h30CC;
font[341]=16'h3EFC;
font[342]=16'h3EF0;
font[343]=16'h30C0;
font[344]=16'h30C4;
font[345]=16'h30C6;
font[346]=16'h32CE;
font[347]=16'h3EFC;
font[348]=16'h3E7C;
font[349]=16'h0000;
font[350]=16'h0000;
font[351]=16'h0000;
font[352]=16'h0000;
font[353]=16'h0000;
font[354]=16'h3FFC;
font[355]=16'h3FFC;
font[356]=16'h2DC0;
font[357]=16'h2CC0;
font[358]=16'h3FFE;
font[359]=16'h3DC0;
font[360]=16'h2DFC;
font[361]=16'h2FDC;
font[362]=16'h3F78;
font[363]=16'h3FFC;
font[364]=16'h05CC;
font[365]=16'h0000;
font[366]=16'h0000;
font[367]=16'h0000;
font[368]=16'h0000;
font[369]=16'h0000;
font[370]=16'h00E0;
font[371]=16'h31F0;
font[372]=16'h33BC;
font[373]=16'h0FDE;
font[374]=16'h04C4;
font[375]=16'h0040;
font[376]=16'h0000;
font[377]=16'h27FC;
font[378]=16'h37FC;
font[379]=16'h33F0;
font[380]=16'h73F0;
font[381]=16'h2070;
font[382]=16'h0000;
font[383]=16'h0000;
font[384]=16'h0000;
font[385]=16'h0000;
font[386]=16'h31FC;
font[387]=16'h3FFC;
font[388]=16'h7F0C;
font[389]=16'h6FEC;
font[390]=16'h3BEC;
font[391]=16'h1B6C;
font[392]=16'h1BEC;
font[393]=16'h1BFC;
font[394]=16'h1F3C;
font[395]=16'h1F06;
font[396]=16'h3BFC;
font[397]=16'h11F8;
font[398]=16'h0000;
font[399]=16'h0000;
font[400]=16'h0000;
font[401]=16'h0000;
font[402]=16'h3EFC;
font[403]=16'h3CFC;
font[404]=16'h0CCC;
font[405]=16'h7FCC;
font[406]=16'h7FCC;
font[407]=16'h1CCC;
font[408]=16'h3ECC;
font[409]=16'h7FCC;
font[410]=16'h6FFC;
font[411]=16'h0CFC;
font[412]=16'h0CCC;
font[413]=16'h0C00;
font[414]=16'h0000;
font[415]=16'h0000;
font[416]=16'h0000;
font[417]=16'h0000;
font[418]=16'h1860;
font[419]=16'h1860;
font[420]=16'h33FE;
font[421]=16'h3FFE;
font[422]=16'h6BFE;
font[423]=16'h7B7C;
font[424]=16'h3B7C;
font[425]=16'h7F7C;
font[426]=16'h277C;
font[427]=16'h1F6C;
font[428]=16'h7B7C;
font[429]=16'h637C;
font[430]=16'h0000;
font[431]=16'h0000;
font[432]=16'h0000;
font[433]=16'h0000;
font[434]=16'h0860;
font[435]=16'h0C60;
font[436]=16'h7F7E;
font[437]=16'h7FFE;
font[438]=16'h30CC;
font[439]=16'h3FCC;
font[440]=16'h3E78;
font[441]=16'h3678;
font[442]=16'h3638;
font[443]=16'h3678;
font[444]=16'h7FFE;
font[445]=16'h2C84;
font[446]=16'h0000;
font[447]=16'h0000;
font[448]=16'h0000;
font[449]=16'h0000;
font[450]=16'h0180;
font[451]=16'h0180;
font[452]=16'h018C;
font[453]=16'h3FDC;
font[454]=16'h3DF8;
font[455]=16'h0DE0;
font[456]=16'h0DF0;
font[457]=16'h19B8;
font[458]=16'h399C;
font[459]=16'h718E;
font[460]=16'h2F84;
font[461]=16'h0700;
font[462]=16'h0000;
font[463]=16'h0000;
font[464]=16'h0000;
font[465]=16'h0000;
font[466]=16'h3FFC;
font[467]=16'h3FFC;
font[468]=16'h0180;
font[469]=16'h1188;
font[470]=16'h199C;
font[471]=16'h1DB8;
font[472]=16'h0DB0;
font[473]=16'h0180;
font[474]=16'h7FFE;
font[475]=16'h7FFE;
font[476]=16'h0180;
font[477]=16'h0180;
font[478]=16'h0000;
font[479]=16'h0000;
font[480]=16'h0000;
font[481]=16'h0000;
font[482]=16'h3FFC;
font[483]=16'h1980;
font[484]=16'h3FFC;
font[485]=16'h3FFC;
font[486]=16'h0998;
font[487]=16'h7FFE;
font[488]=16'h7FFE;
font[489]=16'h0998;
font[490]=16'h3FFC;
font[491]=16'h3FFC;
font[492]=16'h3FFC;
font[493]=16'h3FFC;
font[494]=16'h0000;
font[495]=16'h0000;
font[496]=16'h0000;
font[497]=16'h0000;
font[498]=16'h7FFE;
font[499]=16'h3FFC;
font[500]=16'h1FF8;
font[501]=16'h1FF8;
font[502]=16'h1FF8;
font[503]=16'h1818;
font[504]=16'h1FF8;
font[505]=16'h1818;
font[506]=16'h1FF8;
font[507]=16'h1818;
font[508]=16'h7FFE;
font[509]=16'h3FFC;
font[510]=16'h0000;
font[511]=16'h0000;
font[512]=16'h0000;
font[513]=16'h303C;
font[514]=16'h3FFC;
font[515]=16'h13E0;
font[516]=16'h07FE;
font[517]=16'h07FC;
font[518]=16'h7860;
font[519]=16'h3BFC;
font[520]=16'h1BFC;
font[521]=16'h1B0C;
font[522]=16'h1BFC;
font[523]=16'h3BF8;
font[524]=16'h3C00;
font[525]=16'h27FE;
font[526]=16'h03FC;
font[527]=16'h0000;
font[528]=16'h0000;
font[529]=16'h0000;
font[530]=16'h3F7C;
font[531]=16'h1E0C;
font[532]=16'h3F0C;
font[533]=16'h3F7C;
font[534]=16'h3F7C;
font[535]=16'h3760;
font[536]=16'h3160;
font[537]=16'h3F64;
font[538]=16'h3366;
font[539]=16'h3F66;
font[540]=16'h337C;
font[541]=16'h213C;
font[542]=16'h0000;
font[543]=16'h0000;
font[544]=16'h0000;
font[545]=16'h0000;
font[546]=16'h33F8;
font[547]=16'h3860;
font[548]=16'h17FC;
font[549]=16'h01FC;
font[550]=16'h766C;
font[551]=16'h7FF0;
font[552]=16'h11BC;
font[553]=16'h130C;
font[554]=16'h163C;
font[555]=16'h3800;
font[556]=16'h6FFE;
font[557]=16'h07FC;
font[558]=16'h0000;
font[559]=16'h0000;
font[560]=16'h0000;
font[561]=16'h0000;
font[562]=16'h3EFC;
font[563]=16'h36CC;
font[564]=16'h36CC;
font[565]=16'h3EFC;
font[566]=16'h3EFC;
font[567]=16'h36CC;
font[568]=16'h36FC;
font[569]=16'h37FC;
font[570]=16'h3F8C;
font[571]=16'h3F8C;
font[572]=16'h033C;
font[573]=16'h0138;
font[574]=16'h0000;
font[575]=16'h0000;
font[576]=16'h0000;
font[577]=16'h0000;
font[578]=16'h1FF8;
font[579]=16'h0830;
font[580]=16'h0E60;
font[581]=16'h3FFC;
font[582]=16'h36CC;
font[583]=16'h1E78;
font[584]=16'h32C4;
font[585]=16'h7FFE;
font[586]=16'h181E;
font[587]=16'h1FF8;
font[588]=16'h1818;
font[589]=16'h7FFE;
font[590]=16'h0000;
font[591]=16'h0000;
font[592]=16'h0000;
font[593]=16'h0000;
font[594]=16'h187C;
font[595]=16'h7F7C;
font[596]=16'h7F6C;
font[597]=16'h1B6C;
font[598]=16'h1B6C;
font[599]=16'h1B6C;
font[600]=16'h136C;
font[601]=16'h336C;
font[602]=16'h337C;
font[603]=16'h337C;
font[604]=16'h6F6C;
font[605]=16'h2E6C;
font[606]=16'h0000;
font[607]=16'h0000;
font[608]=16'h0000;
font[609]=16'h0000;
font[610]=16'h2060;
font[611]=16'h37FE;
font[612]=16'h1BFC;
font[613]=16'h37DC;
font[614]=16'h36D4;
font[615]=16'h1198;
font[616]=16'h03FC;
font[617]=16'h13EC;
font[618]=16'h33FC;
font[619]=16'h3FBC;
font[620]=16'h37DC;
font[621]=16'h2184;
font[622]=16'h0000;
font[623]=16'h0000;
font[624]=16'h0000;
font[625]=16'h0000;
font[626]=16'h37FC;
font[627]=16'h3FFC;
font[628]=16'h03FC;
font[629]=16'h73EC;
font[630]=16'h726C;
font[631]=16'h33FC;
font[632]=16'h31E0;
font[633]=16'h337C;
font[634]=16'h3264;
font[635]=16'h3800;
font[636]=16'h6FFE;
font[637]=16'h03FC;
font[638]=16'h0000;
font[639]=16'h0000;
font[640]=16'h0000;
font[641]=16'h0000;
font[642]=16'h1860;
font[643]=16'h1FFC;
font[644]=16'h1BFC;
font[645]=16'h3B00;
font[646]=16'h3B0C;
font[647]=16'h799C;
font[648]=16'h7998;
font[649]=16'h1998;
font[650]=16'h19B0;
font[651]=16'h1830;
font[652]=16'h1FFE;
font[653]=16'h07FE;
font[654]=16'h0000;
font[655]=16'h0000;
font[656]=16'h0000;
font[657]=16'h0000;
font[658]=16'h366C;
font[659]=16'h366C;
font[660]=16'h3FFC;
font[661]=16'h0180;
font[662]=16'h7FFE;
font[663]=16'h3C7C;
font[664]=16'h1FF8;
font[665]=16'h1818;
font[666]=16'h1FF8;
font[667]=16'h1818;
font[668]=16'h7FFE;
font[669]=16'h0000;
font[670]=16'h0000;
font[671]=16'h0000;
font[672]=16'h0000;
font[673]=16'h0000;
font[674]=16'h1860;
font[675]=16'h1060;
font[676]=16'h7FFC;
font[677]=16'h7EFC;
font[678]=16'h3DFE;
font[679]=16'h6DFC;
font[680]=16'h7EC0;
font[681]=16'h7EFE;
font[682]=16'h7EFC;
font[683]=16'h7EF8;
font[684]=16'h0C38;
font[685]=16'h0C08;
font[686]=16'h0000;
font[687]=16'h0000;
font[688]=16'h0000;
font[689]=16'h0000;
font[690]=16'h1BF8;
font[691]=16'h1870;
font[692]=16'h7DC0;
font[693]=16'h7FFC;
font[694]=16'h1BFC;
font[695]=16'h19B4;
font[696]=16'h1BB4;
font[697]=16'h1F6C;
font[698]=16'h1F6C;
font[699]=16'h7ECC;
font[700]=16'h79FC;
font[701]=16'h01B8;
font[702]=16'h0000;
font[703]=16'h0000;
font[704]=16'h0000;
font[705]=16'h0000;
font[706]=16'h3E0C;
font[707]=16'h3FFE;
font[708]=16'h37FE;
font[709]=16'h360C;
font[710]=16'h3E8C;
font[711]=16'h3ECC;
font[712]=16'h366C;
font[713]=16'h366C;
font[714]=16'h360C;
font[715]=16'h3E1C;
font[716]=16'h3E7C;
font[717]=16'h3078;
font[718]=16'h0000;
font[719]=16'h0000;
font[720]=16'h0000;
font[721]=16'h0000;
font[722]=16'h1818;
font[723]=16'h1878;
font[724]=16'h19E0;
font[725]=16'h1980;
font[726]=16'h1800;
font[727]=16'h7FFE;
font[728]=16'h7FFC;
font[729]=16'h18C0;
font[730]=16'h18E0;
font[731]=16'h19F8;
font[732]=16'h1F9E;
font[733]=16'h1C0C;
font[734]=16'h0000;
font[735]=16'h0000;
font[736]=16'h0000;
font[737]=16'h0000;
font[738]=16'h0DFC;
font[739]=16'h3C0C;
font[740]=16'h300C;
font[741]=16'h37EC;
font[742]=16'h37EC;
font[743]=16'h37EC;
font[744]=16'h37EC;
font[745]=16'h37EC;
font[746]=16'h37EC;
font[747]=16'h300C;
font[748]=16'h307C;
font[749]=16'h3038;
font[750]=16'h0000;
font[751]=16'h0000;
font[752]=16'h0000;
font[753]=16'h0000;
font[754]=16'h3DFC;
font[755]=16'h2DFC;
font[756]=16'h398C;
font[757]=16'h39FC;
font[758]=16'h3C00;
font[759]=16'h2FFC;
font[760]=16'h2FFC;
font[761]=16'h3F74;
font[762]=16'h3FFC;
font[763]=16'h236C;
font[764]=16'h237C;
font[765]=16'h2308;
font[766]=16'h0000;
font[767]=16'h0000;
font[768]=16'h0000;
font[769]=16'h0180;
font[770]=16'h0180;
font[771]=16'h7FFE;
font[772]=16'h3FFC;
font[773]=16'h0600;
font[774]=16'h0FF8;
font[775]=16'h0FF8;
font[776]=16'h0C18;
font[777]=16'h0C18;
font[778]=16'h1C18;
font[779]=16'h3838;
font[780]=16'h71F8;
font[781]=16'h21F0;
font[782]=16'h0000;
font[783]=16'h0000;
font[784]=16'h0000;
font[785]=16'h0000;
font[786]=16'h0300;
font[787]=16'h3FFC;
font[788]=16'h3FFC;
font[789]=16'h300C;
font[790]=16'h37EC;
font[791]=16'h37EC;
font[792]=16'h366C;
font[793]=16'h37EC;
font[794]=16'h37EC;
font[795]=16'h360C;
font[796]=16'h307C;
font[797]=16'h3078;
font[798]=16'h0000;
font[799]=16'h0000;
font[800]=16'h0000;
font[801]=16'h0000;
font[802]=16'h7FFE;
font[803]=16'h3FFC;
font[804]=16'h0FF0;
font[805]=16'h3DBC;
font[806]=16'h3DBC;
font[807]=16'h3FFC;
font[808]=16'h3DBC;
font[809]=16'h3DBC;
font[810]=16'h3FFC;
font[811]=16'h300C;
font[812]=16'h3FFC;
font[813]=16'h3FFC;
font[814]=16'h0000;
font[815]=16'h0000;
font[816]=16'h0000;
font[817]=16'h0000;
font[818]=16'h7FFE;
font[819]=16'h3FFC;
font[820]=16'h3FFC;
font[821]=16'h3FFC;
font[822]=16'h366C;
font[823]=16'h37EC;
font[824]=16'h366C;
font[825]=16'h37EC;
font[826]=16'h366C;
font[827]=16'h366C;
font[828]=16'h3FFC;
font[829]=16'h3FFC;
font[830]=16'h0000;
font[831]=16'h0000;
font[832]=16'h0000;
font[833]=16'h0000;
font[834]=16'h37FC;
font[835]=16'h17FC;
font[836]=16'h06EC;
font[837]=16'h77FC;
font[838]=16'h760C;
font[839]=16'h37FC;
font[840]=16'h37BC;
font[841]=16'h3FFC;
font[842]=16'h3F8C;
font[843]=16'h2C0C;
font[844]=16'h0C3C;
font[845]=16'h0418;
font[846]=16'h0000;
font[847]=16'h0000;
font[848]=16'h0000;
font[849]=16'h0000;
font[850]=16'h1860;
font[851]=16'h1870;
font[852]=16'h7FFE;
font[853]=16'h7FFC;
font[854]=16'h3998;
font[855]=16'h3F0C;
font[856]=16'h7F9C;
font[857]=16'h78F8;
font[858]=16'h1870;
font[859]=16'h18F8;
font[860]=16'h1BFE;
font[861]=16'h1B0C;
font[862]=16'h0000;
font[863]=16'h0000;
font[864]=16'h0000;
font[865]=16'h0000;
font[866]=16'h1980;
font[867]=16'h19FC;
font[868]=16'h7FFC;
font[869]=16'h3FCC;
font[870]=16'h3BCC;
font[871]=16'h39AC;
font[872]=16'h393C;
font[873]=16'h7FFC;
font[874]=16'h7FFC;
font[875]=16'h180C;
font[876]=16'h187C;
font[877]=16'h1878;
font[878]=16'h0000;
font[879]=16'h0000;
font[880]=16'h0000;
font[881]=16'h0000;
font[882]=16'h3FFC;
font[883]=16'h3FFC;
font[884]=16'h37FC;
font[885]=16'h3E6C;
font[886]=16'h3BCC;
font[887]=16'h37EC;
font[888]=16'h3E3C;
font[889]=16'h33CC;
font[890]=16'h31EC;
font[891]=16'h3FEC;
font[892]=16'h3FFC;
font[893]=16'h3FFC;
font[894]=16'h0000;
font[895]=16'h0000;
font[896]=16'h0000;
font[897]=16'h0000;
font[898]=16'h3FFC;
font[899]=16'h300C;
font[900]=16'h3FFC;
font[901]=16'h3FFC;
font[902]=16'h37FC;
font[903]=16'h37F8;
font[904]=16'h3FFE;
font[905]=16'h3FFC;
font[906]=16'h3330;
font[907]=16'h3718;
font[908]=16'h6FFC;
font[909]=16'h27EC;
font[910]=16'h0000;
font[911]=16'h0000;
font[912]=16'h0000;
font[913]=16'h0000;
font[914]=16'h3E60;
font[915]=16'h01FC;
font[916]=16'h7FFC;
font[917]=16'h7E6C;
font[918]=16'h186C;
font[919]=16'h306C;
font[920]=16'h36CC;
font[921]=16'h37CC;
font[922]=16'h7FCC;
font[923]=16'h3FBC;
font[924]=16'h01B8;
font[925]=16'h0000;
font[926]=16'h0000;
font[927]=16'h0000;
font[928]=16'h0000;
font[929]=16'h0000;
font[930]=16'h137C;
font[931]=16'h7968;
font[932]=16'h7FFE;
font[933]=16'h11F8;
font[934]=16'h3BFC;
font[935]=16'h7F6E;
font[936]=16'h7BFC;
font[937]=16'h136C;
font[938]=16'h13FC;
font[939]=16'h136C;
font[940]=16'h73FC;
font[941]=16'h3300;
font[942]=16'h0000;
font[943]=16'h0000;
font[944]=16'h0000;
font[945]=16'h0000;
font[946]=16'h0180;
font[947]=16'h3FFC;
font[948]=16'h3FFC;
font[949]=16'h300C;
font[950]=16'h1FF8;
font[951]=16'h1FF8;
font[952]=16'h3FFE;
font[953]=16'h3FFC;
font[954]=16'h06E4;
font[955]=16'h0CEE;
font[956]=16'h3C7C;
font[957]=16'h3078;
font[958]=16'h0000;
font[959]=16'h0000;
font[960]=16'h0000;
font[961]=16'h0000;
font[962]=16'h7FFE;
font[963]=16'h3FFC;
font[964]=16'h3F7C;
font[965]=16'h1E38;
font[966]=16'h3FFC;
font[967]=16'h2D44;
font[968]=16'h3FFC;
font[969]=16'h0180;
font[970]=16'h01F8;
font[971]=16'h1980;
font[972]=16'h7FFE;
font[973]=16'h0000;
font[974]=16'h0000;
font[975]=16'h0000;
font[976]=16'h0000;
font[977]=16'h0000;
font[978]=16'h1FF8;
font[979]=16'h1FF8;
font[980]=16'h1FF8;
font[981]=16'h1FF8;
font[982]=16'h1FF8;
font[983]=16'h366C;
font[984]=16'h366C;
font[985]=16'h1E78;
font[986]=16'h1E78;
font[987]=16'h0670;
font[988]=16'h7FFE;
font[989]=16'h3FFE;
font[990]=16'h0000;
font[991]=16'h0000;
font[992]=16'h0000;
font[993]=16'h0000;
font[994]=16'h1FF8;
font[995]=16'h0000;
font[996]=16'h0000;
font[997]=16'h7FFE;
font[998]=16'h7FFE;
font[999]=16'h0180;
font[1000]=16'h0990;
font[1001]=16'h19B8;
font[1002]=16'h319C;
font[1003]=16'h218E;
font[1004]=16'h0784;
font[1005]=16'h0780;
font[1006]=16'h0000;
font[1007]=16'h0000;
font[1008]=16'h0000;
font[1009]=16'h3068;
font[1010]=16'h306C;
font[1011]=16'h3FFE;
font[1012]=16'h7BFC;
font[1013]=16'h7FFC;
font[1014]=16'h3BFC;
font[1015]=16'h13FC;
font[1016]=16'h7F6C;
font[1017]=16'h3BFC;
font[1018]=16'h1F6C;
font[1019]=16'h1F6C;
font[1020]=16'h1B7C;
font[1021]=16'h1308;
font[1022]=16'h0000;
font[1023]=16'h0000;
font[1024]=16'h0000;
font[1025]=16'h0000;
font[1026]=16'h3FFE;
font[1027]=16'h13B8;
font[1028]=16'h2000;
font[1029]=16'h3FFE;
font[1030]=16'h11F0;
font[1031]=16'h07FC;
font[1032]=16'h17BC;
font[1033]=16'h37FC;
font[1034]=16'h37FC;
font[1035]=16'h376C;
font[1036]=16'h263C;
font[1037]=16'h0000;
font[1038]=16'h0000;
font[1039]=16'h0000;
font[1040]=16'h0000;
font[1041]=16'h0000;
font[1042]=16'h3FEC;
font[1043]=16'h0C6C;
font[1044]=16'h7FFE;
font[1045]=16'h3FFC;
font[1046]=16'h7FEC;
font[1047]=16'h3F7C;
font[1048]=16'h3FB8;
font[1049]=16'h7FB4;
font[1050]=16'h3FF6;
font[1051]=16'h1DFE;
font[1052]=16'h109C;
font[1053]=16'h0000;
font[1054]=16'h0000;
font[1055]=16'h0000;
font[1056]=16'h0000;
font[1057]=16'h0000;
font[1058]=16'h1BFC;
font[1059]=16'h18CC;
font[1060]=16'h7ECC;
font[1061]=16'h7ECC;
font[1062]=16'h18CC;
font[1063]=16'h18CC;
font[1064]=16'h18CC;
font[1065]=16'h1FCC;
font[1066]=16'h1D8C;
font[1067]=16'h3B8C;
font[1068]=16'h137C;
font[1069]=16'h0278;
font[1070]=16'h0000;
font[1071]=16'h0000;
font[1072]=16'h0000;
font[1073]=16'h0000;
font[1074]=16'h0C30;
font[1075]=16'h3FFC;
font[1076]=16'h3FFC;
font[1077]=16'h1E78;
font[1078]=16'h7FFE;
font[1079]=16'h0000;
font[1080]=16'h1FF8;
font[1081]=16'h1818;
font[1082]=16'h1FF8;
font[1083]=16'h1818;
font[1084]=16'h1FF8;
font[1085]=16'h1818;
font[1086]=16'h0000;
font[1087]=16'h0000;
font[1088]=16'h0000;
font[1089]=16'h0000;
font[1090]=16'h3BBC;
font[1091]=16'h11F0;
font[1092]=16'h07FC;
font[1093]=16'h766C;
font[1094]=16'h77FC;
font[1095]=16'h366C;
font[1096]=16'h37FC;
font[1097]=16'h366C;
font[1098]=16'h367C;
font[1099]=16'h3800;
font[1100]=16'h7FFE;
font[1101]=16'h07FC;
font[1102]=16'h0000;
font[1103]=16'h0000;
font[1104]=16'h0000;
font[1105]=16'h0000;
font[1106]=16'h3FFC;
font[1107]=16'h00C0;
font[1108]=16'h00C0;
font[1109]=16'h18C0;
font[1110]=16'h18FC;
font[1111]=16'h18FC;
font[1112]=16'h18C0;
font[1113]=16'h18C0;
font[1114]=16'h18C0;
font[1115]=16'h18C0;
font[1116]=16'h7FFE;
font[1117]=16'h7FFE;
font[1118]=16'h0000;
font[1119]=16'h0000;
font[1120]=16'h0000;
font[1121]=16'h0000;
font[1122]=16'h1860;
font[1123]=16'h1860;
font[1124]=16'h1860;
font[1125]=16'h1FFE;
font[1126]=16'h1FFE;
font[1127]=16'h1FF0;
font[1128]=16'h1FF0;
font[1129]=16'h1830;
font[1130]=16'h1830;
font[1131]=16'h3030;
font[1132]=16'h7030;
font[1133]=16'h2030;
font[1134]=16'h0000;
font[1135]=16'h0000;
font[1136]=16'h0000;
font[1137]=16'h0000;
font[1138]=16'h0180;
font[1139]=16'h01C0;
font[1140]=16'h3FFE;
font[1141]=16'h3FFC;
font[1142]=16'h37FC;
font[1143]=16'h37F0;
font[1144]=16'h37FE;
font[1145]=16'h37FC;
font[1146]=16'h36F0;
font[1147]=16'h37B6;
font[1148]=16'h77FE;
font[1149]=16'h24DC;
font[1150]=16'h0000;
font[1151]=16'h0000;
font[1152]=16'h0000;
font[1153]=16'h0000;
font[1154]=16'h0300;
font[1155]=16'h3FFE;
font[1156]=16'h3FFE;
font[1157]=16'h0600;
font[1158]=16'h0E00;
font[1159]=16'h1FFC;
font[1160]=16'h1FFC;
font[1161]=16'h38C0;
font[1162]=16'h30C0;
font[1163]=16'h20C0;
font[1164]=16'h1FFE;
font[1165]=16'h0FFE;
font[1166]=16'h0000;
font[1167]=16'h0000;
font[1168]=16'h0000;
font[1169]=16'h0000;
font[1170]=16'h0380;
font[1171]=16'h0380;
font[1172]=16'h0380;
font[1173]=16'h03FC;
font[1174]=16'h03FC;
font[1175]=16'h0380;
font[1176]=16'h0380;
font[1177]=16'h0380;
font[1178]=16'h0380;
font[1179]=16'h0380;
font[1180]=16'h7FFE;
font[1181]=16'h7FFE;
font[1182]=16'h0000;
font[1183]=16'h0000;
font[1184]=16'h0000;
font[1185]=16'h0000;
font[1186]=16'h0300;
font[1187]=16'h7FFE;
font[1188]=16'h7FFE;
font[1189]=16'h0E00;
font[1190]=16'h1C00;
font[1191]=16'h3FFC;
font[1192]=16'h7FFC;
font[1193]=16'h2C0C;
font[1194]=16'h0C0C;
font[1195]=16'h0FFC;
font[1196]=16'h0FFC;
font[1197]=16'h0C0C;
font[1198]=16'h0000;
font[1199]=16'h0000;
font[1200]=16'h0000;
font[1201]=16'h0000;
font[1202]=16'h0000;
font[1203]=16'h7FFE;
font[1204]=16'h0380;
font[1205]=16'h03C0;
font[1206]=16'h03E0;
font[1207]=16'h03B0;
font[1208]=16'h0398;
font[1209]=16'h039C;
font[1210]=16'h0388;
font[1211]=16'h0380;
font[1212]=16'h0380;
font[1213]=16'h0000;
font[1214]=16'h0000;
font[1215]=16'h0000;
font[1216]=16'h0000;
font[1217]=16'h0000;
font[1218]=16'h1860;
font[1219]=16'h1BFC;
font[1220]=16'h7FFC;
font[1221]=16'h3D98;
font[1222]=16'h1FFE;
font[1223]=16'h1FFC;
font[1224]=16'h7FFE;
font[1225]=16'h7BFC;
font[1226]=16'h1B98;
font[1227]=16'h19F0;
font[1228]=16'h7BFC;
font[1229]=16'h338C;
font[1230]=16'h0000;
font[1231]=16'h0000;
font[1232]=16'h0000;
font[1233]=16'h0000;
font[1234]=16'h11F8;
font[1235]=16'h7FF8;
font[1236]=16'h7FFC;
font[1237]=16'h13FC;
font[1238]=16'h136C;
font[1239]=16'h7B6C;
font[1240]=16'h7FFE;
font[1241]=16'h3FFC;
font[1242]=16'h10F0;
font[1243]=16'h1198;
font[1244]=16'h771E;
font[1245]=16'h3604;
font[1246]=16'h0000;
font[1247]=16'h0000;
font[1248]=16'h0000;
font[1249]=16'h0000;
font[1250]=16'h31C0;
font[1251]=16'h1FFC;
font[1252]=16'h13FC;
font[1253]=16'h0300;
font[1254]=16'h7FFC;
font[1255]=16'h3BFC;
font[1256]=16'h1860;
font[1257]=16'h1FFE;
font[1258]=16'h1FFC;
font[1259]=16'h3C00;
font[1260]=16'h7FFE;
font[1261]=16'h27FC;
font[1262]=16'h0000;
font[1263]=16'h0000;
font[1264]=16'h0000;
font[1265]=16'h0000;
font[1266]=16'h1860;
font[1267]=16'h33FC;
font[1268]=16'h35FC;
font[1269]=16'h6FFE;
font[1270]=16'h7BFE;
font[1271]=16'h1B84;
font[1272]=16'h31F0;
font[1273]=16'h7BFE;
font[1274]=16'h3FFC;
font[1275]=16'h1DF8;
font[1276]=16'h7F8C;
font[1277]=16'h6304;
font[1278]=16'h0000;
font[1279]=16'h0000;
font[1280]=16'h0000;
font[1281]=16'h0000;
font[1282]=16'h130C;
font[1283]=16'h7FFC;
font[1284]=16'h7B0C;
font[1285]=16'h13FC;
font[1286]=16'h7B80;
font[1287]=16'h7BFC;
font[1288]=16'h33EC;
font[1289]=16'h13F4;
font[1290]=16'h139C;
font[1291]=16'h33FC;
font[1292]=16'h703C;
font[1293]=16'h3018;
font[1294]=16'h0000;
font[1295]=16'h0000;
font[1296]=16'h0000;
font[1297]=16'h0660;
font[1298]=16'h3FFE;
font[1299]=16'h3FFE;
font[1300]=16'h1FF8;
font[1301]=16'h1818;
font[1302]=16'h1FF8;
font[1303]=16'h1818;
font[1304]=16'h1FF8;
font[1305]=16'h0700;
font[1306]=16'h3FFC;
font[1307]=16'h3DBC;
font[1308]=16'h3FFC;
font[1309]=16'h19F8;
font[1310]=16'h0000;
font[1311]=16'h0000;
font[1312]=16'h0000;
font[1313]=16'h0000;
font[1314]=16'h3B18;
font[1315]=16'h13E8;
font[1316]=16'h2368;
font[1317]=16'h3FFC;
font[1318]=16'h160C;
font[1319]=16'h07FC;
font[1320]=16'h171C;
font[1321]=16'h37FC;
font[1322]=16'h371C;
font[1323]=16'h361C;
font[1324]=16'h2638;
font[1325]=16'h0000;
font[1326]=16'h0000;
font[1327]=16'h0000;
font[1328]=16'h0000;
font[1329]=16'h0000;
font[1330]=16'h19B0;
font[1331]=16'h7FFE;
font[1332]=16'h7FFC;
font[1333]=16'h1BFC;
font[1334]=16'h1B30;
font[1335]=16'h7B30;
font[1336]=16'h7BFC;
font[1337]=16'h1B30;
font[1338]=16'h1B30;
font[1339]=16'h1BFE;
font[1340]=16'h7BFC;
font[1341]=16'h3300;
font[1342]=16'h0000;
font[1343]=16'h0000;
font[1344]=16'h0000;
font[1345]=16'h0000;
font[1346]=16'h0CC0;
font[1347]=16'h7DFC;
font[1348]=16'h3B9C;
font[1349]=16'h1BF8;
font[1350]=16'h7FF0;
font[1351]=16'h3FFE;
font[1352]=16'h3C4E;
font[1353]=16'h3D8C;
font[1354]=16'h7B18;
font[1355]=16'h1870;
font[1356]=16'h1BE0;
font[1357]=16'h1B80;
font[1358]=16'h0000;
font[1359]=16'h0000;
font[1360]=16'h0000;
font[1361]=16'h0000;
font[1362]=16'h0180;
font[1363]=16'h0180;
font[1364]=16'h7FFE;
font[1365]=16'h7FFC;
font[1366]=16'h0430;
font[1367]=16'h1E18;
font[1368]=16'h3E7C;
font[1369]=16'h37E4;
font[1370]=16'h03C0;
font[1371]=16'h07E0;
font[1372]=16'h7FFC;
font[1373]=16'h383C;
font[1374]=16'h0000;
font[1375]=16'h0000;
font[1376]=16'h0000;
font[1377]=16'h0000;
font[1378]=16'h3FFC;
font[1379]=16'h0018;
font[1380]=16'h1B98;
font[1381]=16'h1DB0;
font[1382]=16'h0C70;
font[1383]=16'h06E0;
font[1384]=16'h03C0;
font[1385]=16'h03C0;
font[1386]=16'h1FF0;
font[1387]=16'h7C7C;
font[1388]=16'h300C;
font[1389]=16'h0000;
font[1390]=16'h0000;
font[1391]=16'h0000;
font[1392]=16'h0000;
font[1393]=16'h0000;
font[1394]=16'h260C;
font[1395]=16'h767C;
font[1396]=16'h3FF0;
font[1397]=16'h0FE0;
font[1398]=16'h247E;
font[1399]=16'h7F7E;
font[1400]=16'h2FEC;
font[1401]=16'h0FEC;
font[1402]=16'h336C;
font[1403]=16'h3FEC;
font[1404]=16'h3FEC;
font[1405]=16'h234C;
font[1406]=16'h0000;
font[1407]=16'h0000;
font[1408]=16'h0000;
font[1409]=16'h0000;
font[1410]=16'h7FFE;
font[1411]=16'h3FFC;
font[1412]=16'h1E78;
font[1413]=16'h3E7C;
font[1414]=16'h2664;
font[1415]=16'h0000;
font[1416]=16'h1FF8;
font[1417]=16'h1FF8;
font[1418]=16'h0E30;
font[1419]=16'h07E0;
font[1420]=16'h3FFE;
font[1421]=16'h3C3C;
font[1422]=16'h0000;
font[1423]=16'h0000;
font[1424]=16'h0000;
font[1425]=16'h0000;
font[1426]=16'h7BFC;
font[1427]=16'h3BFC;
font[1428]=16'h3F8C;
font[1429]=16'h1DFC;
font[1430]=16'h7FFC;
font[1431]=16'h7F8C;
font[1432]=16'h1DFC;
font[1433]=16'h3FFC;
font[1434]=16'h7F8C;
font[1435]=16'h798C;
font[1436]=16'h1BFE;
font[1437]=16'h1BFE;
font[1438]=16'h0000;
font[1439]=16'h0000;
font[1440]=16'h0000;
font[1441]=16'h0000;
font[1442]=16'h1BFC;
font[1443]=16'h33FC;
font[1444]=16'h376C;
font[1445]=16'h7F6C;
font[1446]=16'h7BFC;
font[1447]=16'h13FC;
font[1448]=16'h3F6C;
font[1449]=16'h7B6C;
font[1450]=16'h3F6C;
font[1451]=16'h7BFC;
font[1452]=16'h63FC;
font[1453]=16'h030C;
font[1454]=16'h0000;
font[1455]=16'h0000;
font[1456]=16'h0000;
font[1457]=16'h0000;
font[1458]=16'h19FE;
font[1459]=16'h19FC;
font[1460]=16'h7FFE;
font[1461]=16'h7EFE;
font[1462]=16'h3FB4;
font[1463]=16'h37B0;
font[1464]=16'h35BC;
font[1465]=16'h35BC;
font[1466]=16'h25F0;
font[1467]=16'h67F0;
font[1468]=16'h7FFE;
font[1469]=16'h193C;
font[1470]=16'h0000;
font[1471]=16'h0000;
font[1472]=16'h0000;
font[1473]=16'h0000;
font[1474]=16'h0670;
font[1475]=16'h7FFE;
font[1476]=16'h7FFE;
font[1477]=16'h0000;
font[1478]=16'h3FFC;
font[1479]=16'h3FFC;
font[1480]=16'h061C;
font[1481]=16'h061C;
font[1482]=16'h06F8;
font[1483]=16'h06F8;
font[1484]=16'h0600;
font[1485]=16'h0600;
font[1486]=16'h0000;
font[1487]=16'h0000;
font[1488]=16'h0000;
font[1489]=16'h0000;
font[1490]=16'h3FFC;
font[1491]=16'h37FC;
font[1492]=16'h370C;
font[1493]=16'h37FC;
font[1494]=16'h360C;
font[1495]=16'h37FC;
font[1496]=16'h30E0;
font[1497]=16'h3068;
font[1498]=16'h327C;
font[1499]=16'h3E6E;
font[1500]=16'h77E4;
font[1501]=16'h21C0;
font[1502]=16'h0000;
font[1503]=16'h0000;
font[1504]=16'h0000;
font[1505]=16'h0000;
font[1506]=16'h1860;
font[1507]=16'h1060;
font[1508]=16'h7FFE;
font[1509]=16'h7DFC;
font[1510]=16'h3CD8;
font[1511]=16'h2D8C;
font[1512]=16'h7FDC;
font[1513]=16'h3EF8;
font[1514]=16'h7E70;
font[1515]=16'h7EF8;
font[1516]=16'h0FFE;
font[1517]=16'h0D8C;
font[1518]=16'h0000;
font[1519]=16'h0000;
font[1520]=16'h0000;
font[1521]=16'h0000;
font[1522]=16'h0180;
font[1523]=16'h3FFC;
font[1524]=16'h3FFC;
font[1525]=16'h0E70;
font[1526]=16'h7FFE;
font[1527]=16'h3FFC;
font[1528]=16'h1FF8;
font[1529]=16'h1FF8;
font[1530]=16'h1FF8;
font[1531]=16'h1818;
font[1532]=16'h1FF8;
font[1533]=16'h1FF8;
font[1534]=16'h0000;
font[1535]=16'h0000;
font[1536]=16'h0000;
font[1537]=16'h0000;
font[1538]=16'h1818;
font[1539]=16'h1FF8;
font[1540]=16'h1818;
font[1541]=16'h7FFE;
font[1542]=16'h318C;
font[1543]=16'h3FF8;
font[1544]=16'h3188;
font[1545]=16'h3188;
font[1546]=16'h3FFC;
font[1547]=16'h0180;
font[1548]=16'h7FFE;
font[1549]=16'h0000;
font[1550]=16'h0000;
font[1551]=16'h0000;
font[1552]=16'h1818;
font[1553]=16'h1818;
font[1554]=16'h1E78;
font[1555]=16'h1E78;
font[1556]=16'h1998;
font[1557]=16'h1998;
font[1558]=16'h1998;
font[1559]=16'h1998;
font[1560]=16'h1818;
font[1561]=16'h1818;
font[1562]=16'h1818;
font[1563]=16'h1818;
font[1564]=16'h1818;
font[1565]=16'h1818;
font[1566]=16'h0000;
font[1567]=16'h0000;
font[1568]=16'h07E0;
font[1569]=16'h07E0;
font[1570]=16'h1818;
font[1571]=16'h1818;
font[1572]=16'h1818;
font[1573]=16'h1818;
font[1574]=16'h1FF8;
font[1575]=16'h1FF8;
font[1576]=16'h1818;
font[1577]=16'h1818;
font[1578]=16'h1818;
font[1579]=16'h1818;
font[1580]=16'h1818;
font[1581]=16'h1818;
font[1582]=16'h0000;
font[1583]=16'h0000;
font[1584]=16'h1818;
font[1585]=16'h1818;
font[1586]=16'h0660;
font[1587]=16'h0660;
font[1588]=16'h0180;
font[1589]=16'h0180;
font[1590]=16'h0180;
font[1591]=16'h0180;
font[1592]=16'h0180;
font[1593]=16'h0180;
font[1594]=16'h0660;
font[1595]=16'h0660;
font[1596]=16'h1818;
font[1597]=16'h1818;
font[1598]=16'h0000;
font[1599]=16'h0000;
font[1600]=16'h0000;
font[1601]=16'h0000;
font[1602]=16'h0000;
font[1603]=16'h0000;
font[1604]=16'h0000;
font[1605]=16'h0000;
font[1606]=16'h0000;
font[1607]=16'h0000;
font[1608]=16'h0000;
font[1609]=16'h0000;
font[1610]=16'h0000;
font[1611]=16'h0000;
font[1612]=16'h0000;
font[1613]=16'h0000;
font[1614]=16'h0000;
font[1615]=16'h0000;
font[1616]=16'h0000;
font[1617]=16'h0000;
font[1618]=16'h0000;
font[1619]=16'h0000;
font[1620]=16'h0000;
font[1621]=16'h0000;
font[1622]=16'h0000;
font[1623]=16'h0000;
font[1624]=16'h0000;
font[1625]=16'h0000;
font[1626]=16'h0000;
font[1627]=16'h0000;
font[1628]=16'h0000;
font[1629]=16'h0000;
font[1630]=16'h0000;
font[1631]=16'h0000;
font[1632]=16'h0000;
font[1633]=16'h0000;
font[1634]=16'h0000;
font[1635]=16'h0000;
font[1636]=16'h0000;
font[1637]=16'h0000;
font[1638]=16'h0000;
font[1639]=16'h0000;
font[1640]=16'h0000;
font[1641]=16'h0000;
font[1642]=16'h0000;
font[1643]=16'h0000;
font[1644]=16'h0000;
font[1645]=16'h0000;
font[1646]=16'h0000;
font[1647]=16'h0000;
font[1648]=16'h0000;
font[1649]=16'h0000;
font[1650]=16'h0000;
font[1651]=16'h0000;
font[1652]=16'h0000;
font[1653]=16'h0000;
font[1654]=16'h0000;
font[1655]=16'h0000;
font[1656]=16'h0000;
font[1657]=16'h0000;
font[1658]=16'h0000;
font[1659]=16'h0000;
font[1660]=16'h0000;
font[1661]=16'h0000;
font[1662]=16'h0000;
font[1663]=16'h0000;
font[1664]=16'h0000;
font[1665]=16'h0000;
font[1666]=16'h0000;
font[1667]=16'h0000;
font[1668]=16'h0000;
font[1669]=16'h0000;
font[1670]=16'h0000;
font[1671]=16'h0000;
font[1672]=16'h0000;
font[1673]=16'h0000;
font[1674]=16'h0000;
font[1675]=16'h0000;
font[1676]=16'h0000;
font[1677]=16'h0000;
font[1678]=16'h0000;
font[1679]=16'h0000;
font[1680]=16'h0000;
font[1681]=16'h0000;
font[1682]=16'h0000;
font[1683]=16'h0000;
font[1684]=16'h0000;
font[1685]=16'h0000;
font[1686]=16'h0000;
font[1687]=16'h0000;
font[1688]=16'h0000;
font[1689]=16'h0000;
font[1690]=16'h0000;
font[1691]=16'h0000;
font[1692]=16'h0000;
font[1693]=16'h0000;
font[1694]=16'h0000;
font[1695]=16'h0000;
font[1696]=16'h0000;
font[1697]=16'h0000;
font[1698]=16'h0000;
font[1699]=16'h0000;
font[1700]=16'h0000;
font[1701]=16'h0000;
font[1702]=16'h0000;
font[1703]=16'h0000;
font[1704]=16'h0000;
font[1705]=16'h0000;
font[1706]=16'h0000;
font[1707]=16'h0000;
font[1708]=16'h0000;
font[1709]=16'h0000;
font[1710]=16'h0000;
font[1711]=16'h0000;
font[1712]=16'h0000;
font[1713]=16'h0000;
font[1714]=16'h0000;
font[1715]=16'h0000;
font[1716]=16'h0000;
font[1717]=16'h0000;
font[1718]=16'h0000;
font[1719]=16'h0000;
font[1720]=16'h0000;
font[1721]=16'h0000;
font[1722]=16'h0000;
font[1723]=16'h0000;
font[1724]=16'h0000;
font[1725]=16'h0000;
font[1726]=16'h0000;
font[1727]=16'h0000;
font[1728]=16'h0000;
font[1729]=16'h0000;
font[1730]=16'h0000;
font[1731]=16'h0000;
font[1732]=16'h0000;
font[1733]=16'h0000;
font[1734]=16'h0000;
font[1735]=16'h0000;
font[1736]=16'h0000;
font[1737]=16'h0000;
font[1738]=16'h0000;
font[1739]=16'h0000;
font[1740]=16'h0000;
font[1741]=16'h0000;
font[1742]=16'h0000;
font[1743]=16'h0000;
font[1744]=16'h0000;
font[1745]=16'h0000;
font[1746]=16'h0000;
font[1747]=16'h0000;
font[1748]=16'h0000;
font[1749]=16'h0000;
font[1750]=16'h0000;
font[1751]=16'h0000;
font[1752]=16'h0000;
font[1753]=16'h0000;
font[1754]=16'h0000;
font[1755]=16'h0000;
font[1756]=16'h0000;
font[1757]=16'h0000;
font[1758]=16'h0000;
font[1759]=16'h0000;
font[1760]=16'h0000;
font[1761]=16'h0000;
font[1762]=16'h0000;
font[1763]=16'h0000;
font[1764]=16'h0000;
font[1765]=16'h0000;
font[1766]=16'h0000;
font[1767]=16'h0000;
font[1768]=16'h0000;
font[1769]=16'h0000;
font[1770]=16'h0000;
font[1771]=16'h0000;
font[1772]=16'h0000;
font[1773]=16'h0000;
font[1774]=16'h0000;
font[1775]=16'h0000;
font[1776]=16'h0000;
font[1777]=16'h0000;
font[1778]=16'h0000;
font[1779]=16'h0000;
font[1780]=16'h0000;
font[1781]=16'h0000;
font[1782]=16'h0000;
font[1783]=16'h0000;
font[1784]=16'h0000;
font[1785]=16'h0000;
font[1786]=16'h0000;
font[1787]=16'h0000;
font[1788]=16'h0000;
font[1789]=16'h0000;
font[1790]=16'h0000;
font[1791]=16'h0000;
font[1792]=16'h0000;
font[1793]=16'h0000;
font[1794]=16'h0000;
font[1795]=16'h0000;
font[1796]=16'h0000;
font[1797]=16'h0000;
font[1798]=16'h0000;
font[1799]=16'h0000;
font[1800]=16'h0000;
font[1801]=16'h0000;
font[1802]=16'h0000;
font[1803]=16'h0000;
font[1804]=16'h0000;
font[1805]=16'h0000;
font[1806]=16'h0000;
font[1807]=16'h0000;
font[1808]=16'h0000;
font[1809]=16'h0000;
font[1810]=16'h0000;
font[1811]=16'h0000;
font[1812]=16'h0000;
font[1813]=16'h0000;
font[1814]=16'h0000;
font[1815]=16'h0000;
font[1816]=16'h0000;
font[1817]=16'h0000;
font[1818]=16'h0000;
font[1819]=16'h0000;
font[1820]=16'h0000;
font[1821]=16'h0000;
font[1822]=16'h0000;
font[1823]=16'h0000;
font[1824]=16'h0000;
font[1825]=16'h0000;
font[1826]=16'h0000;
font[1827]=16'h0000;
font[1828]=16'h0000;
font[1829]=16'h0000;
font[1830]=16'h0000;
font[1831]=16'h0000;
font[1832]=16'h0000;
font[1833]=16'h0000;
font[1834]=16'h0000;
font[1835]=16'h0000;
font[1836]=16'h0000;
font[1837]=16'h0000;
font[1838]=16'h0000;
font[1839]=16'h0000;
font[1840]=16'h0000;
font[1841]=16'h0000;
font[1842]=16'h0000;
font[1843]=16'h0000;
font[1844]=16'h0000;
font[1845]=16'h0000;
font[1846]=16'h0000;
font[1847]=16'h0000;
font[1848]=16'h0000;
font[1849]=16'h0000;
font[1850]=16'h0000;
font[1851]=16'h0000;
font[1852]=16'h0000;
font[1853]=16'h0000;
font[1854]=16'h0000;
font[1855]=16'h0000;
font[1856]=16'h0000;
font[1857]=16'h0000;
font[1858]=16'h0000;
font[1859]=16'h0000;
font[1860]=16'h0000;
font[1861]=16'h0000;
font[1862]=16'h0000;
font[1863]=16'h0000;
font[1864]=16'h0000;
font[1865]=16'h0000;
font[1866]=16'h0000;
font[1867]=16'h0000;
font[1868]=16'h0000;
font[1869]=16'h0000;
font[1870]=16'h0000;
font[1871]=16'h0000;
font[1872]=16'h0000;
font[1873]=16'h0000;
font[1874]=16'h0000;
font[1875]=16'h0000;
font[1876]=16'h0000;
font[1877]=16'h0000;
font[1878]=16'h0000;
font[1879]=16'h0000;
font[1880]=16'h0000;
font[1881]=16'h0000;
font[1882]=16'h0000;
font[1883]=16'h0000;
font[1884]=16'h0000;
font[1885]=16'h0000;
font[1886]=16'h0000;
font[1887]=16'h0000;
font[1888]=16'h0000;
font[1889]=16'h0000;
font[1890]=16'h0000;
font[1891]=16'h0000;
font[1892]=16'h0000;
font[1893]=16'h0000;
font[1894]=16'h0000;
font[1895]=16'h0000;
font[1896]=16'h0000;
font[1897]=16'h0000;
font[1898]=16'h0000;
font[1899]=16'h0000;
font[1900]=16'h0000;
font[1901]=16'h0000;
font[1902]=16'h0000;
font[1903]=16'h0000;
font[1904]=16'h0000;
font[1905]=16'h0000;
font[1906]=16'h0000;
font[1907]=16'h0000;
font[1908]=16'h0000;
font[1909]=16'h0000;
font[1910]=16'h0000;
font[1911]=16'h0000;
font[1912]=16'h0000;
font[1913]=16'h0000;
font[1914]=16'h0000;
font[1915]=16'h0000;
font[1916]=16'h0000;
font[1917]=16'h0000;
font[1918]=16'h0000;
font[1919]=16'h0000;
font[1920]=16'h0000;
font[1921]=16'h0000;
font[1922]=16'h0000;
font[1923]=16'h0000;
font[1924]=16'h0000;
font[1925]=16'h0000;
font[1926]=16'h0000;
font[1927]=16'h0000;
font[1928]=16'h0000;
font[1929]=16'h0000;
font[1930]=16'h0000;
font[1931]=16'h0000;
font[1932]=16'h0000;
font[1933]=16'h0000;
font[1934]=16'h0000;
font[1935]=16'h0000;
font[1936]=16'h0000;
font[1937]=16'h0000;
font[1938]=16'h0000;
font[1939]=16'h0000;
font[1940]=16'h0000;
font[1941]=16'h0000;
font[1942]=16'h0000;
font[1943]=16'h0000;
font[1944]=16'h0000;
font[1945]=16'h0000;
font[1946]=16'h0000;
font[1947]=16'h0000;
font[1948]=16'h0000;
font[1949]=16'h0000;
font[1950]=16'h0000;
font[1951]=16'h0000;
font[1952]=16'h0000;
font[1953]=16'h0000;
font[1954]=16'h0000;
font[1955]=16'h0000;
font[1956]=16'h0000;
font[1957]=16'h0000;
font[1958]=16'h0000;
font[1959]=16'h0000;
font[1960]=16'h0000;
font[1961]=16'h0000;
font[1962]=16'h0000;
font[1963]=16'h0000;
font[1964]=16'h0000;
font[1965]=16'h0000;
font[1966]=16'h0000;
font[1967]=16'h0000;
font[1968]=16'h0000;
font[1969]=16'h0000;
font[1970]=16'h0000;
font[1971]=16'h0000;
font[1972]=16'h0000;
font[1973]=16'h0000;
font[1974]=16'h0000;
font[1975]=16'h0000;
font[1976]=16'h0000;
font[1977]=16'h0000;
font[1978]=16'h0000;
font[1979]=16'h0000;
font[1980]=16'h0000;
font[1981]=16'h0000;
font[1982]=16'h0000;
font[1983]=16'h0000;
font[1984]=16'h0000;
font[1985]=16'h0000;
font[1986]=16'h0000;
font[1987]=16'h0000;
font[1988]=16'h0000;
font[1989]=16'h0000;
font[1990]=16'h0000;
font[1991]=16'h0000;
font[1992]=16'h0000;
font[1993]=16'h0000;
font[1994]=16'h0000;
font[1995]=16'h0000;
font[1996]=16'h0000;
font[1997]=16'h0000;
font[1998]=16'h0000;
font[1999]=16'h0000;
font[2000]=16'h0000;
font[2001]=16'h0000;
font[2002]=16'h0000;
font[2003]=16'h0000;
font[2004]=16'h0000;
font[2005]=16'h0000;
font[2006]=16'h0000;
font[2007]=16'h0000;
font[2008]=16'h0000;
font[2009]=16'h0000;
font[2010]=16'h0000;
font[2011]=16'h0000;
font[2012]=16'h0000;
font[2013]=16'h0000;
font[2014]=16'h0000;
font[2015]=16'h0000;
font[2016]=16'h0000;
font[2017]=16'h0000;
font[2018]=16'h0000;
font[2019]=16'h0000;
font[2020]=16'h0000;
font[2021]=16'h0000;
font[2022]=16'h0000;
font[2023]=16'h0000;
font[2024]=16'h0000;
font[2025]=16'h0000;
font[2026]=16'h0000;
font[2027]=16'h0000;
font[2028]=16'h0000;
font[2029]=16'h0000;
font[2030]=16'h0000;
font[2031]=16'h0000;
font[2032]=16'h0000;
font[2033]=16'h0000;
font[2034]=16'h0000;
font[2035]=16'h0000;
font[2036]=16'h0000;
font[2037]=16'h0000;
font[2038]=16'h0000;
font[2039]=16'h0000;
font[2040]=16'h0000;
font[2041]=16'h0000;
font[2042]=16'h0000;
font[2043]=16'h0000;
font[2044]=16'h0000;
font[2045]=16'h0000;
font[2046]=16'h0000;
font[2047]=16'h0000;
end
always @(posedge clk)row<=font[addr];
reg [23:0] bg1,bg2;
reg [3:0] col1,col2;
reg text1,text2;
reg de1,de2;
reg [6:0] gid;
reg [3:0] text_cell,ry;
reg signed [10:0] number;
reg [10:0] magnitude;
reg [5:0] phrase;
reg [3:0] pc;
reg [23:0] background;
function [6:0] text_glyph;
input [5:0] phrase;input [3:0] column;
begin
text_glyph=0;
case(phrase)
6'd0:begin case(column)
4'd0:text_glyph=7'd18;
4'd1:text_glyph=7'd19;
default:text_glyph=0;endcase end
6'd1:begin case(column)
4'd0:text_glyph=7'd20;
4'd1:text_glyph=7'd21;
4'd2:text_glyph=7'd19;
default:text_glyph=0;endcase end
6'd2:begin case(column)
4'd0:text_glyph=7'd22;
4'd1:text_glyph=7'd23;
default:text_glyph=0;endcase end
6'd3:begin case(column)
4'd0:text_glyph=7'd24;
4'd1:text_glyph=7'd25;
4'd2:text_glyph=7'd19;
default:text_glyph=0;endcase end
6'd4:begin case(column)
4'd0:text_glyph=7'd26;
4'd1:text_glyph=7'd27;
default:text_glyph=0;endcase end
6'd5:begin case(column)
4'd0:text_glyph=7'd28;
4'd1:text_glyph=7'd29;
default:text_glyph=0;endcase end
6'd6:begin case(column)
4'd0:text_glyph=7'd30;
4'd1:text_glyph=7'd31;
default:text_glyph=0;endcase end
6'd7:begin case(column)
4'd0:text_glyph=7'd32;
4'd1:text_glyph=7'd33;
default:text_glyph=0;endcase end
6'd8:begin case(column)
4'd0:text_glyph=7'd34;
4'd1:text_glyph=7'd35;
4'd2:text_glyph=7'd19;
default:text_glyph=0;endcase end
6'd9:begin case(column)
4'd0:text_glyph=7'd36;
4'd1:text_glyph=7'd37;
default:text_glyph=0;endcase end
6'd10:begin case(column)
4'd0:text_glyph=7'd38;
4'd1:text_glyph=7'd39;
default:text_glyph=0;endcase end
6'd11:begin case(column)
4'd0:text_glyph=7'd40;
4'd1:text_glyph=7'd41;
default:text_glyph=0;endcase end
6'd12:begin case(column)
4'd0:text_glyph=7'd42;
4'd1:text_glyph=7'd43;
default:text_glyph=0;endcase end
6'd13:begin case(column)
4'd0:text_glyph=7'd44;
4'd1:text_glyph=7'd45;
default:text_glyph=0;endcase end
6'd14:begin case(column)
4'd0:text_glyph=7'd46;
4'd1:text_glyph=7'd47;
default:text_glyph=0;endcase end
6'd15:begin case(column)
4'd0:text_glyph=7'd48;
4'd1:text_glyph=7'd49;
default:text_glyph=0;endcase end
6'd16:begin case(column)
4'd0:text_glyph=7'd50;
4'd1:text_glyph=7'd51;
4'd2:text_glyph=7'd52;
4'd3:text_glyph=7'd53;
default:text_glyph=0;endcase end
6'd17:begin case(column)
4'd0:text_glyph=7'd26;
4'd1:text_glyph=7'd27;
4'd2:text_glyph=7'd54;
4'd3:text_glyph=7'd55;
default:text_glyph=0;endcase end
6'd18:begin case(column)
4'd0:text_glyph=7'd55;
4'd1:text_glyph=7'd56;
4'd2:text_glyph=7'd38;
4'd3:text_glyph=7'd57;
default:text_glyph=0;endcase end
6'd19:begin case(column)
4'd0:text_glyph=7'd58;
4'd1:text_glyph=7'd27;
4'd2:text_glyph=7'd42;
4'd3:text_glyph=7'd43;
default:text_glyph=0;endcase end
6'd20:begin case(column)
4'd0:text_glyph=7'd59;
4'd1:text_glyph=7'd60;
4'd2:text_glyph=7'd61;
4'd3:text_glyph=7'd62;
default:text_glyph=0;endcase end
6'd21:begin case(column)
4'd0:text_glyph=7'd63;
4'd1:text_glyph=7'd64;
4'd2:text_glyph=7'd65;
4'd3:text_glyph=7'd66;
default:text_glyph=0;endcase end
6'd22:begin case(column)
4'd0:text_glyph=7'd67;
4'd1:text_glyph=7'd68;
4'd2:text_glyph=7'd36;
4'd3:text_glyph=7'd37;
default:text_glyph=0;endcase end
6'd23:begin case(column)
4'd0:text_glyph=7'd69;
4'd1:text_glyph=7'd70;
4'd2:text_glyph=7'd36;
4'd3:text_glyph=7'd71;
default:text_glyph=0;endcase end
6'd24:begin case(column)
4'd0:text_glyph=7'd72;
4'd1:text_glyph=7'd73;
default:text_glyph=0;endcase end
6'd25:begin case(column)
4'd0:text_glyph=7'd74;
4'd1:text_glyph=7'd73;
default:text_glyph=0;endcase end
6'd26:begin case(column)
4'd0:text_glyph=7'd72;
4'd1:text_glyph=7'd75;
default:text_glyph=0;endcase end
6'd27:begin case(column)
4'd0:text_glyph=7'd74;
4'd1:text_glyph=7'd75;
default:text_glyph=0;endcase end
6'd28:begin case(column)
4'd0:text_glyph=7'd31;
4'd1:text_glyph=7'd76;
4'd2:text_glyph=7'd66;
4'd3:text_glyph=7'd77;
default:text_glyph=0;endcase end
6'd29:begin case(column)
4'd0:text_glyph=7'd78;
4'd1:text_glyph=7'd79;
4'd2:text_glyph=7'd80;
4'd3:text_glyph=7'd81;
default:text_glyph=0;endcase end
6'd30:begin case(column)
4'd0:text_glyph=7'd29;
4'd1:text_glyph=7'd82;
4'd2:text_glyph=7'd83;
4'd3:text_glyph=7'd84;
default:text_glyph=0;endcase end
6'd31:begin case(column)
4'd0:text_glyph=7'd85;
4'd1:text_glyph=7'd86;
4'd2:text_glyph=7'd87;
4'd3:text_glyph=7'd88;
default:text_glyph=0;endcase end
6'd32:begin case(column)
4'd0:text_glyph=7'd49;
4'd1:text_glyph=7'd72;
default:text_glyph=0;endcase end
6'd33:begin case(column)
4'd0:text_glyph=7'd49;
4'd1:text_glyph=7'd74;
default:text_glyph=0;endcase end
6'd34:begin case(column)
4'd0:text_glyph=7'd49;
4'd1:text_glyph=7'd73;
default:text_glyph=0;endcase end
6'd35:begin case(column)
4'd0:text_glyph=7'd49;
4'd1:text_glyph=7'd75;
default:text_glyph=0;endcase end
6'd36:begin case(column)
4'd0:text_glyph=7'd89;
4'd1:text_glyph=7'd52;
default:text_glyph=0;endcase end
6'd37:begin case(column)
4'd0:text_glyph=7'd90;
4'd1:text_glyph=7'd52;
default:text_glyph=0;endcase end
6'd38:begin case(column)
4'd0:text_glyph=7'd91;
4'd1:text_glyph=7'd42;
4'd2:text_glyph=7'd52;
4'd3:text_glyph=7'd92;
default:text_glyph=0;endcase end
6'd39:begin case(column)
4'd0:text_glyph=7'd93;
4'd1:text_glyph=7'd55;
4'd2:text_glyph=7'd21;
4'd3:text_glyph=7'd94;
default:text_glyph=0;endcase end
6'd40:begin case(column)
4'd0:text_glyph=7'd95;
4'd1:text_glyph=7'd96;
default:text_glyph=0;endcase end
6'd41:begin case(column)
4'd0:text_glyph=7'd95;
4'd1:text_glyph=7'd96;
4'd2:text_glyph=7'd52;
4'd3:text_glyph=7'd92;
default:text_glyph=0;endcase end
default:text_glyph=0;endcase
end
endfunction
// Readable 112x104 rail card: native 16x16 glyphs, separate label/value.
wire [9:0] panel_dx=x-video_panel_x,panel_dy=y-video_panel_y;
wire card_text=(panel_dx>=8 && panel_dx<104) &&
 ((panel_dy>=8 && panel_dy<24)||(panel_dy>=32 && panel_dy<48)||
  (panel_dy>=56 && panel_dy<72)||(panel_dy>=80 && panel_dy<96));
wire card_value=panel_dy>=56 && panel_dy<72;
wire [9:0] render_x=media_video?10'd296+(panel_dx-8)+(card_value?10'd64:10'd0):x;
wire [9:0] render_y=media_video?
 (panel_dy<24?10'd184+panel_dy-8:panel_dy<48?10'd206+panel_dy-32:
  panel_dy<72?10'd206+panel_dy-56:10'd232+panel_dy-80):y;
wire panel_region=!media_video || (x>=video_panel_x && panel_dx<112 && y>=video_panel_y && panel_dy<104);
reg value1,value2;
always @* begin
    number=0;
    case(mode)
        0:number=brightness;1:number={3'd0,contrast};2:number=tone;3:number={3'd0,saturation};
        4:number={3'd0,zoom};5:number=pan_x;6:number=pan_y;8:number={4'd0,opacity};
        10:number={5'd0,speed};13:number={7'd0,duration};14:number={7'd0,period};
    endcase
    magnitude=number<0?-number:number;
    gid=0;text_cell=(render_x-296)>>4;ry=(render_y-184)&15;phrase=0;pc=text_cell;
    background=irgb;
    if(enable && panel_region)begin
        if(media_video)begin
            background=panel_dy<28?24'h17344E:24'h102539;
            if(panel_dx==0 || panel_dx==111 || panel_dy==103)background=24'h73A8B6;
            if(panel_dy<3)background=24'hF2AA70;
            if(panel_dy==76 && panel_dx>=8 && panel_dx<104)background=24'h36576A;
        end else if(render_x>=288 && render_x<512 && render_y>=176 && render_y<256)begin
            background=24'h25354F;
            if(render_x<290 || render_x>=510 || render_y<178 || render_y>=254)background=24'hA5CFE4;
        end
    end
    if(render_y>=184 && render_y<200) begin
        phrase=16+mode[3:2];gid=text_glyph(phrase,text_cell);
    end else if(render_y>=206 && render_y<222) begin
        ry=(render_y-206)&15;
        if(text_cell<3)gid=text_glyph({2'd0,mode},text_cell);
        else if(text_cell==3)gid=7'd11;
        else if(text_cell==4)gid=number<0?7'd13:7'd12;
        else if(text_cell==5)gid=7'd1+(magnitude/100)%10;
        else if(text_cell==6)gid=7'd1+(magnitude/10)%10;
        else if(text_cell==7)gid=7'd1+magnitude%10;
        else if(text_cell==8)begin
            if(mode==1 || mode==3 || mode==4 || mode==8)gid=7'd14;
            else if(mode==2)gid=7'd16;
            else if(mode==14)gid=7'd17;
        end
        // Duration is stored in quarter-seconds, render decimal seconds.
        if(mode==13 && text_cell>=4 && text_cell<=8)begin
            case(text_cell)
                4:gid=7'd1+duration/4;
                5:gid=7'd15;
                6:gid=7'd1+((duration%4)*25)/10;
                7:gid=7'd1+((duration%4)*25)%10;
                8:gid=7'd17;
            endcase
        end
        if(mode==7 || mode==9 || mode==11 || mode==12 || mode==15)begin
            if(text_cell>=4)begin
                pc=text_cell-4;
                case(mode)
                    7:phrase=20+fit;
                    9:phrase=22+blend;
                    11:phrase=24+position;
                    12:phrase=28+transition;
                    15:phrase=32+direction;
                    default:phrase=0;
                endcase
                gid=text_glyph(phrase,pc);
            end
        end
    end else if(render_y>=232 && render_y<248)begin
        ry=(render_y-232)&15;
        phrase=compare?39:tuning==1?36:tuning==2?37:38;
        gid=text_glyph(phrase,text_cell);
    end
    if(media_video && mode==4 && zoom==255 && render_y>=206 && render_y<222 && text_cell>=4)begin
        case(text_cell)4:gid=7'd97;5:gid=7'd98;6:gid=7'd99;default:gid=0;endcase
    end
    if(volume_show)begin
        gid=0;
        if(render_y>=184 && render_y<200)gid=text_glyph(6'd41,text_cell);
        else if(render_y>=206 && render_y<222)begin
            ry=(render_y-206)&15;
            if(text_cell<2)gid=text_glyph(6'd40,text_cell);
            else if(text_cell==2)gid=7'd11;
            else if(text_cell==4)gid=7'd1+{3'd0,volume};
            else if(text_cell==6)gid=7'd16;
        end else if(render_y>=232 && render_y<248)begin
            ry=(render_y-232)&15;gid=text_glyph(6'd38,text_cell);
        end
    end
    if(media_video && panel_dy>=32 && panel_dy<48 && text_cell>=4)gid=0;
end
always @(posedge clk or posedge rst) begin
    if(rst)begin
        de_d<=0;vs_d<=0;x<=0;y<=0;addr<=0;bg1<=0;bg2<=0;col1<=0;col2<=0;
        text1<=0;text2<=0;de1<=0;de2<=0;rgb<=0;value1<=0;value2<=0;
    end else begin
        de_d<=de;vs_d<=vs;
        if(vs_d && !vs)y<=0;else if(de_d && !de)y<=y+1;
        if(!de)x<=0;else x<=x+1;
        addr<={gid,ry};bg1<=background;bg2<=bg1;
        col1<=render_x-296;col2<=col1;
        text1<=enable && panel_region && (media_video?card_text:(render_x>=296 && render_x<504 && render_y>=184 && render_y<248));
        value1<=media_video && card_value;value2<=value1;
        text2<=text1;de1<=de;de2<=de1;
        rgb<=!de2?24'd0:text2 && row[15-col2]?(value2?24'hFFD69B:24'hF3FAFF):bg2;
    end
end
endmodule

// Coherent multi-bit CDC: payload is held from request until acknowledge.
// Destination adds a guard cycle after the three-stage request synchronizer.
// Updates while busy coalesce to the newest source value after acknowledgement.
module exhibit_cdc_mailbox #(parameter WIDTH=1)(
 input wire src_clk,dst_clk,rst,
 input wire [WIDTH-1:0] src_data,
 output reg [WIDTH-1:0] dst_data);
reg [WIDTH-1:0] held;
reg req,ack,capture;
reg [2:0] req_sync,ack_sync;
always @(posedge src_clk or posedge rst)begin
 if(rst)begin held<=0;req<=0;ack_sync<=0;end
 else begin
  ack_sync<={ack_sync[1:0],ack};
  if(ack_sync[2]==req && src_data!=held)begin held<=src_data;req<=~req;end
 end
end
always @(posedge dst_clk or posedge rst)begin
 if(rst)begin dst_data<=0;ack<=0;req_sync<=0;capture<=0;end
 else begin
  req_sync<={req_sync[1:0],req};
  if(capture)begin dst_data<=held;ack<=req_sync[2];capture<=0;end
  else if(req_sync[2]!=ack)capture<=1;
 end
end
endmodule

// Frame-boundary volume overlay. Ordinary picture changes dismiss the popup;
// holding the active volume gesture keeps it visible until release.
module exhibit_volume_latch(
    input wire clk,rst,vs,
    input wire [4:0] status,
    input wire picture_ack,
    output reg [3:0] volume,
    output reg visible
);
reg vs_d;
reg [4:0] seen;
reg ack_seen;
reg [7:0] remaining;
always @(posedge clk or posedge rst)begin
    if(rst)begin vs_d<=0;seen<=5'd8;ack_seen<=0;remaining<=0;volume<=8;visible<=0;end
    else begin
        vs_d<=vs;
        if(vs_d && !vs)begin
            seen<=status;ack_seen<=picture_ack;volume<=status[3:0];
            if(status[4] || status[3:0]!=seen[3:0])begin remaining<=180;visible<=1;end
            else if(picture_ack!=ack_seen)begin remaining<=0;visible<=0;end
            else if(remaining>1)begin remaining<=remaining-1'b1;visible<=1;end
            else begin remaining<=0;visible<=0;end
        end
    end
end
endmodule


module osd_font32_dictionary(input wire I_clk,input wire [9:0] I_addr,output reg [31:0] O_row);
(* rom_style="bram" *) reg [31:0] rows[0:1023];
initial begin
rows[0]=32'h00000000;
rows[1]=32'h0003F000;
rows[2]=32'h000FF800;
rows[3]=32'h001FFC00;
rows[4]=32'h001F7E00;
rows[5]=32'h003C1E00;
rows[6]=32'h003C1F00;
rows[7]=32'h007C0F00;
rows[8]=32'h00780F00;
rows[9]=32'h007C1E00;
rows[10]=32'h003F7E00;
rows[11]=32'h0007E000;
rows[12]=32'h0000E000;
rows[13]=32'h0003E000;
rows[14]=32'h000FE000;
rows[15]=32'h003FE000;
rows[16]=32'h003DE000;
rows[17]=32'h0031E000;
rows[18]=32'h0001E000;
rows[19]=32'h003FF000;
rows[20]=32'h003FF800;
rows[21]=32'h003FFC00;
rows[22]=32'h00307C00;
rows[23]=32'h00003C00;
rows[24]=32'h00007C00;
rows[25]=32'h0000F800;
rows[26]=32'h0001F800;
rows[27]=32'h000FC000;
rows[28]=32'h001F0000;
rows[29]=32'h003E0000;
rows[30]=32'h003C0000;
rows[31]=32'h007FFC00;
rows[32]=32'h001FF000;
rows[33]=32'h001FF800;
rows[34]=32'h000FF000;
rows[35]=32'h0001FC00;
rows[36]=32'h00003E00;
rows[37]=32'h00001E00;
rows[38]=32'h00203E00;
rows[39]=32'h003DFE00;
rows[40]=32'h001FE000;
rows[41]=32'h0000FC00;
rows[42]=32'h0003FC00;
rows[43]=32'h0003BC00;
rows[44]=32'h0007BC00;
rows[45]=32'h000F3C00;
rows[46]=32'h001E3C00;
rows[47]=32'h003C3C00;
rows[48]=32'h00783C00;
rows[49]=32'h00F03C00;
rows[50]=32'h00FFFF80;
rows[51]=32'h000FFC00;
rows[52]=32'h001E0000;
rows[53]=32'h001FFE00;
rows[54]=32'h00103E00;
rows[55]=32'h001EFC00;
rows[56]=32'h0000FE00;
rows[57]=32'h0003FE00;
rows[58]=32'h0007FE00;
rows[59]=32'h000FEE00;
rows[60]=32'h000F0000;
rows[61]=32'h003CF800;
rows[62]=32'h003DFC00;
rows[63]=32'h003FFE00;
rows[64]=32'h003F3F00;
rows[65]=32'h003E1F00;
rows[66]=32'h003C0F00;
rows[67]=32'h001E1F00;
rows[68]=32'h000FFE00;
rows[69]=32'h0007FC00;
rows[70]=32'h007FFF00;
rows[71]=32'h00007800;
rows[72]=32'h0000F000;
rows[73]=32'h0001F000;
rows[74]=32'h0003C000;
rows[75]=32'h0007C000;
rows[76]=32'h000F8000;
rows[77]=32'h003E3C00;
rows[78]=32'h007C1F00;
rows[79]=32'h007E3F00;
rows[80]=32'h0007F000;
rows[81]=32'h007C3C00;
rows[82]=32'h00781E00;
rows[83]=32'h007C3E00;
rows[84]=32'h000FDE00;
rows[85]=32'h00001C00;
rows[86]=32'h003DF800;
rows[87]=32'h001F8000;
rows[88]=32'h00038000;
rows[89]=32'h00000E60;
rows[90]=32'h07000EE0;
rows[91]=32'h07000EF0;
rows[92]=32'h07000E78;
rows[93]=32'h07000E30;
rows[94]=32'h071FFFFC;
rows[95]=32'h1FDFFFFC;
rows[96]=32'h1FDC0F00;
rows[97]=32'h071C0F20;
rows[98]=32'h071FFF78;
rows[99]=32'h071FFF70;
rows[100]=32'h071CEFF0;
rows[101]=32'h071CE7E0;
rows[102]=32'h071CE7C0;
rows[103]=32'h07DCE7C0;
rows[104]=32'h07FCE790;
rows[105]=32'h0FFFE79C;
rows[106]=32'h3FFFEF9C;
rows[107]=32'h1F3BDFBC;
rows[108]=32'h1C783FB8;
rows[109]=32'h10787FF8;
rows[110]=32'h0078F9F8;
rows[111]=32'h003071F8;
rows[112]=32'h001020F0;
rows[113]=32'h00018000;
rows[114]=32'h1FFFFFF8;
rows[115]=32'h07FFFFF0;
rows[116]=32'h0783C0F0;
rows[117]=32'h0783C1F0;
rows[118]=32'h0783DFF0;
rows[119]=32'h0783DFE0;
rows[120]=32'h0783CFC0;
rows[121]=32'h0003C800;
rows[122]=32'h0380F000;
rows[123]=32'h0780F000;
rows[124]=32'h3FFCF000;
rows[125]=32'h3FFFFF80;
rows[126]=32'h0F07FF80;
rows[127]=32'h0E03FF80;
rows[128]=32'h0EE0F380;
rows[129]=32'h1EE0F380;
rows[130]=32'h1CE0F380;
rows[131]=32'h1FFCF380;
rows[132]=32'h1FFCE380;
rows[133]=32'h01E1E390;
rows[134]=32'h00FDE39C;
rows[135]=32'h1FFDE39C;
rows[136]=32'h3FFDE39C;
rows[137]=32'h1FFDE398;
rows[138]=32'h1DE3C398;
rows[139]=32'h00E7C3F8;
rows[140]=32'h00E783F8;
rows[141]=32'h00E383F8;
rows[142]=32'h00E100E0;
rows[143]=32'h04030380;
rows[144]=32'h1E078780;
rows[145]=32'h1E038780;
rows[146]=32'h0F3FFFF8;
rows[147]=32'h0FBFFFF8;
rows[148]=32'h07BFFFF8;
rows[149]=32'h07807800;
rows[150]=32'h02007000;
rows[151]=32'h001FFFF0;
rows[152]=32'h1F9FFFF0;
rows[153]=32'h1F9C00F0;
rows[154]=32'h079E00F0;
rows[155]=32'h079C00F0;
rows[156]=32'h079FFFF0;
rows[157]=32'h0F9FFFF0;
rows[158]=32'h1FFFFFE0;
rows[159]=32'h1FFFFFFC;
rows[160]=32'h1CFFFFF8;
rows[161]=32'h187FFFF8;
rows[162]=32'h0800FFF0;
rows[163]=32'h00701E00;
rows[164]=32'h00F81F00;
rows[165]=32'h01F80F80;
rows[166]=32'h03F007C0;
rows[167]=32'h07E003E0;
rows[168]=32'h0FD019F0;
rows[169]=32'h1FB81CF8;
rows[170]=32'h0E7C3E70;
rows[171]=32'h043C3C20;
rows[172]=32'h003E7C00;
rows[173]=32'h03FFFFC0;
rows[174]=32'h1FFE7FF8;
rows[175]=32'h1FF81FF8;
rows[176]=32'h0FC007F0;
rows[177]=32'h0E000070;
rows[178]=32'h04000000;
rows[179]=32'h0E3FFFF0;
rows[180]=32'h1F3FFFF0;
rows[181]=32'h0FBFFFF0;
rows[182]=32'h07C303E0;
rows[183]=32'h0787CF80;
rows[184]=32'h0201FE00;
rows[185]=32'h1F9C3870;
rows[186]=32'h1F9C78F0;
rows[187]=32'h079C3870;
rows[188]=32'h079C78F0;
rows[189]=32'h079C38F0;
rows[190]=32'h079C3FF0;
rows[191]=32'h079C3BF0;
rows[192]=32'h0F9C3BE0;
rows[193]=32'h1FE00000;
rows[194]=32'h08003FF0;
rows[195]=32'h0E1FFFE0;
rows[196]=32'h1F9FFFE0;
rows[197]=32'h07DFFFE0;
rows[198]=32'h039C01E0;
rows[199]=32'h011C01E0;
rows[200]=32'h001FFFE0;
rows[201]=32'h081FFFE0;
rows[202]=32'h1C1FFFE0;
rows[203]=32'h1E1C01E0;
rows[204]=32'h1F1FFFE0;
rows[205]=32'h0F9FFFE0;
rows[206]=32'h071FFFE0;
rows[207]=32'h02000000;
rows[208]=32'h003FFFF0;
rows[209]=32'h063FFFF0;
rows[210]=32'h073FFFF0;
rows[211]=32'h0F39CE70;
rows[212]=32'h0E39CE70;
rows[213]=32'h1EFFFFFC;
rows[214]=32'h0FFFFFF8;
rows[215]=32'h0F000000;
rows[216]=32'h0F1E0380;
rows[217]=32'h0F1FFF80;
rows[218]=32'h0F7FFFE0;
rows[219]=32'h0E1E03C0;
rows[220]=32'h1E0F0780;
rows[221]=32'h1E07DF00;
rows[222]=32'h1E03FE00;
rows[223]=32'h3E0FFFE0;
rows[224]=32'h3DFFFFF8;
rows[225]=32'h1CFF0FF8;
rows[226]=32'h08F000F0;
rows[227]=32'h01803800;
rows[228]=32'h01C07800;
rows[229]=32'h03C07C00;
rows[230]=32'h03DFFFF0;
rows[231]=32'h039FFFF0;
rows[232]=32'h079C0070;
rows[233]=32'h071C0070;
rows[234]=32'h0F1FFFF0;
rows[235]=32'h1F1FFFF0;
rows[236]=32'h1F1C0000;
rows[237]=32'h3F3C0000;
rows[238]=32'h3F3FFFF8;
rows[239]=32'h1F3FFFF8;
rows[240]=32'h173F7338;
rows[241]=32'h073F7338;
rows[242]=32'h077FFFF8;
rows[243]=32'h07F7FFF8;
rows[244]=32'h07F77338;
rows[245]=32'h07E77338;
rows[246]=32'h072773F8;
rows[247]=32'h070773F0;
rows[248]=32'h070701F0;
rows[249]=32'h03E003C0;
rows[250]=32'h03C003C0;
rows[251]=32'h0FFFFFF0;
rows[252]=32'h0F000070;
rows[253]=32'h0F3FFC70;
rows[254]=32'h0F381C70;
rows[255]=32'h0F3800F0;
rows[256]=32'h0F000FF0;
rows[257]=32'h0F000FE0;
rows[258]=32'h0F000FC0;
rows[259]=32'h3FFFFFFC;
rows[260]=32'h03E00000;
rows[261]=32'h01E00000;
rows[262]=32'h000003C0;
rows[263]=32'h000007C0;
rows[264]=32'h000FFFC0;
rows[265]=32'h000FFF80;
rows[266]=32'h0007FF00;
rows[267]=32'h0180F300;
rows[268]=32'h03C0F780;
rows[269]=32'h03C0F7E0;
rows[270]=32'h03C0F1E0;
rows[271]=32'h0780F0C0;
rows[272]=32'h0F00F1F0;
rows[273]=32'h0F23FFF0;
rows[274]=32'h1E7FFFF8;
rows[275]=32'h3E7FFFE0;
rows[276]=32'h3FF7F000;
rows[277]=32'h1FF079F8;
rows[278]=32'h1FE7FFF8;
rows[279]=32'h03EFFFF8;
rows[280]=32'h03CFFFF0;
rows[281]=32'h078FF860;
rows[282]=32'h078078F0;
rows[283]=32'h0F807BF0;
rows[284]=32'h1FF03FE0;
rows[285]=32'h1FF03FC0;
rows[286]=32'h1FE03FB0;
rows[287]=32'h0C007E38;
rows[288]=32'h01F1FE38;
rows[289]=32'h3FFFFF78;
rows[290]=32'h3FFFEFF8;
rows[291]=32'h1FFF8FF0;
rows[292]=32'h1C0C07F0;
rows[293]=32'h000001E0;
rows[294]=32'h0FFCF800;
rows[295]=32'h0FFCFFF8;
rows[296]=32'h0FFDFFF8;
rows[297]=32'h0E3FE0F0;
rows[298]=32'h0E3FE1F0;
rows[299]=32'h0E3FF3E0;
rows[300]=32'h0FFE7FC0;
rows[301]=32'h0FFC7F80;
rows[302]=32'h00E3FFF8;
rows[303]=32'h00E7FFF8;
rows[304]=32'h1CE7E1F8;
rows[305]=32'h1CFB0038;
rows[306]=32'h1CF80000;
rows[307]=32'h1CF9FFF0;
rows[308]=32'h1CE1FFF0;
rows[309]=32'h1CE1E0F0;
rows[310]=32'h1DFDE0F0;
rows[311]=32'h1FFDE0F0;
rows[312]=32'h1FFDFFF0;
rows[313]=32'h1FF9FFF0;
rows[314]=32'h1C01FFF0;
rows[315]=32'h0001E0F0;
rows[316]=32'h00F80F00;
rows[317]=32'h00F00F00;
rows[318]=32'h01F00F00;
rows[319]=32'h03E00F00;
rows[320]=32'h07E00F00;
rows[321]=32'h0FC00F00;
rows[322]=32'h1F800F00;
rows[323]=32'h0F000F00;
rows[324]=32'h06000F00;
rows[325]=32'h00C01800;
rows[326]=32'h00F07C00;
rows[327]=32'h01F03C00;
rows[328]=32'h03E03E00;
rows[329]=32'h07C01E00;
rows[330]=32'h1F8FFFF8;
rows[331]=32'h1F0FFFF8;
rows[332]=32'h1E4FFFF8;
rows[333]=32'h0CFFFFF8;
rows[334]=32'h08F03C00;
rows[335]=32'h03E03C00;
rows[336]=32'h07C03C00;
rows[337]=32'h0FC03C00;
rows[338]=32'h1FCFFFF0;
rows[339]=32'h1FC7FFF0;
rows[340]=32'h1BC03C00;
rows[341]=32'h13C03C00;
rows[342]=32'h03C03C00;
rows[343]=32'h03DFFFFC;
rows[344]=32'h03DFFFF8;
rows[345]=32'h03C00000;
rows[346]=32'h08781E00;
rows[347]=32'h1CF01E00;
rows[348]=32'h1EF1FFF8;
rows[349]=32'h0FE1FFF8;
rows[350]=32'h07C1FFF8;
rows[351]=32'h3FFDE078;
rows[352]=32'h3FFDC078;
rows[353]=32'h3FFDCF78;
rows[354]=32'h07DDCF78;
rows[355]=32'h039DCE78;
rows[356]=32'h0399CE78;
rows[357]=32'h0399DE78;
rows[358]=32'h0381FE78;
rows[359]=32'h03803F00;
rows[360]=32'h03807FC0;
rows[361]=32'h0380FFE0;
rows[362]=32'h1F83F1F0;
rows[363]=32'h1F87E0F8;
rows[364]=32'h1F83C078;
rows[365]=32'h1F010030;
rows[366]=32'h03003E00;
rows[367]=32'h07003E00;
rows[368]=32'h0F803E00;
rows[369]=32'h07C03E00;
rows[370]=32'h03C03E00;
rows[371]=32'h01003E00;
rows[372]=32'h001FFFFC;
rows[373]=32'h3F9FFFFC;
rows[374]=32'h3F8FFFF8;
rows[375]=32'h1F803E00;
rows[376]=32'h07803E00;
rows[377]=32'h07883E00;
rows[378]=32'h07983E00;
rows[379]=32'h07F83E00;
rows[380]=32'h07E03E00;
rows[381]=32'h02003E00;
rows[382]=32'h000000F0;
rows[383]=32'h1FFFC0F0;
rows[384]=32'h1FFFDEF0;
rows[385]=32'h03E01EF0;
rows[386]=32'h03C61EF0;
rows[387]=32'h07CE1EF0;
rows[388]=32'h078F1EF0;
rows[389]=32'h0F0F9EF0;
rows[390]=32'h0C011EF0;
rows[391]=32'h00F01EF0;
rows[392]=32'h1FFF9EF0;
rows[393]=32'h00F81EF0;
rows[394]=32'h00F000F0;
rows[395]=32'h00FFC0F0;
rows[396]=32'h3FFFC0F0;
rows[397]=32'h1FFFDFF0;
rows[398]=32'h1C000FE0;
rows[399]=32'h00000FC0;
rows[400]=32'h06007800;
rows[401]=32'h0E007800;
rows[402]=32'h1F007800;
rows[403]=32'h0F807800;
rows[404]=32'h07C07800;
rows[405]=32'h03BFFFF8;
rows[406]=32'h023FFFF8;
rows[407]=32'h003FFFF8;
rows[408]=32'h1F80F000;
rows[409]=32'h1F80F800;
rows[410]=32'h1F81FC00;
rows[411]=32'h0781FF00;
rows[412]=32'h0783FF80;
rows[413]=32'h0787CFC0;
rows[414]=32'h078FC7E0;
rows[415]=32'h079F83F0;
rows[416]=32'h07BF01F0;
rows[417]=32'h079E00E0;
rows[418]=32'h0FCC0040;
rows[419]=32'h1FF80000;
rows[420]=32'h1C7FFFF8;
rows[421]=32'h183FFFF8;
rows[422]=32'h0803FFF0;
rows[423]=32'h00F803C0;
rows[424]=32'h1FFCC3C0;
rows[425]=32'h1FFDE3C0;
rows[426]=32'h1FF1F3C0;
rows[427]=32'h03C0FFC0;
rows[428]=32'h03C07BC0;
rows[429]=32'h03C033C0;
rows[430]=32'h1FFC03C0;
rows[431]=32'h1FFD83C0;
rows[432]=32'h1FFBC3C0;
rows[433]=32'h03C3E3C0;
rows[434]=32'h03D1F3C0;
rows[435]=32'h07F8F3C0;
rows[436]=32'h07FC23C0;
rows[437]=32'h0FDE03C0;
rows[438]=32'h0FDC07FC;
rows[439]=32'h1FCFFFFC;
rows[440]=32'h3FC7FFFC;
rows[441]=32'h1BC7C3C0;
rows[442]=32'h1BC003C0;
rows[443]=32'h03803C00;
rows[444]=32'h038FFFF8;
rows[445]=32'h1FF03E00;
rows[446]=32'h1FE03C00;
rows[447]=32'h038FFFF0;
rows[448]=32'h03FFFFF0;
rows[449]=32'h07F001F0;
rows[450]=32'h3FF001F0;
rows[451]=32'h1FE381E0;
rows[452]=32'h1F87C3E0;
rows[453]=32'h1383E3C0;
rows[454]=32'h0381F7C0;
rows[455]=32'h0381FF80;
rows[456]=32'h0380FF00;
rows[457]=32'h03807F00;
rows[458]=32'h0381FFC0;
rows[459]=32'h0787FFF0;
rows[460]=32'h1F9FE7F8;
rows[461]=32'h1F9FC1F8;
rows[462]=32'h1F8F00F0;
rows[463]=32'h0C0C0030;
rows[464]=32'h1E000078;
rows[465]=32'h1E7FFE78;
rows[466]=32'h1EFFFF78;
rows[467]=32'h1E1CF378;
rows[468]=32'h1E1CF3F8;
rows[469]=32'h1E3CF3F8;
rows[470]=32'h1E3CF778;
rows[471]=32'h1E78FF78;
rows[472]=32'h1FF07F78;
rows[473]=32'h1EF07E78;
rows[474]=32'h1E400078;
rows[475]=32'h00078000;
rows[476]=32'h00079000;
rows[477]=32'h00079FC0;
rows[478]=32'h00078FE0;
rows[479]=32'h000787F0;
rows[480]=32'h000781F0;
rows[481]=32'h000780E0;
rows[482]=32'h00078040;
rows[483]=32'h1FF0FFF8;
rows[484]=32'h0780E3E0;
rows[485]=32'h038EE3C0;
rows[486]=32'h1FFEE3C0;
rows[487]=32'h1FFEEFF8;
rows[488]=32'h078EEFF8;
rows[489]=32'h0380E3C0;
rows[490]=32'h0381E3C0;
rows[491]=32'h0381C3C0;
rows[492]=32'h03F3C3C0;
rows[493]=32'h1FF3C3C0;
rows[494]=32'h1FF79FFC;
rows[495]=32'h1FEF9FFC;
rows[496]=32'h1C0F1FFC;
rows[497]=32'h00020000;
rows[498]=32'h00181800;
rows[499]=32'h00F80F80;
rows[500]=32'h01F007C0;
rows[501]=32'h03E007E0;
rows[502]=32'h0FC003F0;
rows[503]=32'h1F8001F8;
rows[504]=32'h1FFFFFF0;
rows[505]=32'h0BFFFFB0;
rows[506]=32'h03FFFF80;
rows[507]=32'h003C0780;
rows[508]=32'h007C0780;
rows[509]=32'h00780780;
rows[510]=32'h00F80780;
rows[511]=32'h07F3FF80;
rows[512]=32'h1FE3FF00;
rows[513]=32'h1FC3FF00;
rows[514]=32'h0F81FC00;
rows[515]=32'h06000000;
rows[516]=32'h03801E00;
rows[517]=32'h07801E00;
rows[518]=32'h07F81E00;
rows[519]=32'h0FF81E00;
rows[520]=32'h1E07FFF8;
rows[521]=32'h3FF79E78;
rows[522]=32'h1FF79E78;
rows[523]=32'h1BC79E78;
rows[524]=32'h13879E78;
rows[525]=32'h3FF7FFF8;
rows[526]=32'h03C7FFF8;
rows[527]=32'h0387FFF8;
rows[528]=32'h03879E78;
rows[529]=32'h03981E00;
rows[530]=32'h03F81E00;
rows[531]=32'h07E01E00;
rows[532]=32'h03001E00;
rows[533]=32'h0103C080;
rows[534]=32'h0383C1E0;
rows[535]=32'h07C3C3F0;
rows[536]=32'h03F3C7E0;
rows[537]=32'h01F3CF80;
rows[538]=32'h00E3CF00;
rows[539]=32'h0043C200;
rows[540]=32'h000001F0;
rows[541]=32'h00600E00;
rows[542]=32'h01F00F80;
rows[543]=32'h0FFF80F0;
rows[544]=32'h0FFF9CF0;
rows[545]=32'h0F079CF0;
rows[546]=32'h0F0780F0;
rows[547]=32'h0F3F9FF0;
rows[548]=32'h0F3F1FE0;
rows[549]=32'h0E180F80;
rows[550]=32'h00C01E00;
rows[551]=32'h03C01E00;
rows[552]=32'h03E01E00;
rows[553]=32'h01E01E00;
rows[554]=32'h1FFF1FFC;
rows[555]=32'h1FFE1FF8;
rows[556]=32'h001C1E00;
rows[557]=32'h0E3E1E00;
rows[558]=32'h1E3C1E00;
rows[559]=32'h0E3C1E00;
rows[560]=32'h0F3DFFF8;
rows[561]=32'h0F39FFF8;
rows[562]=32'h0779E078;
rows[563]=32'h07F9E078;
rows[564]=32'h007FE078;
rows[565]=32'h07FFE078;
rows[566]=32'h3FFDFFF8;
rows[567]=32'h1FFDFFF8;
rows[568]=32'h1FF9FFF8;
rows[569]=32'h1E01FFF8;
rows[570]=32'h0001E078;
rows[571]=32'h01C00000;
rows[572]=32'h0F03C0F0;
rows[573]=32'h00010000;
rows[574]=32'h003CF000;
rows[575]=32'h073C4000;
rows[576]=32'h07BC00C0;
rows[577]=32'h07BC03E0;
rows[578]=32'h07BC01E0;
rows[579]=32'h0F3C01F0;
rows[580]=32'h0F3C00F0;
rows[581]=32'h0F3C00F8;
rows[582]=32'h1E3C0078;
rows[583]=32'h1E3C0478;
rows[584]=32'h1E3C07A0;
rows[585]=32'h1E3C0F80;
rows[586]=32'h043C0F80;
rows[587]=32'h003FFF00;
rows[588]=32'h01C01E00;
rows[589]=32'h01C01FFC;
rows[590]=32'h1FFC1FFC;
rows[591]=32'h1FFC1E00;
rows[592]=32'h1DFDFFF0;
rows[593]=32'h1DDDFFF0;
rows[594]=32'h1DDDE0F0;
rows[595]=32'h1DDDCE70;
rows[596]=32'h1DDDDE70;
rows[597]=32'h1DFDDE70;
rows[598]=32'h1DF9FFF0;
rows[599]=32'h01C03FC0;
rows[600]=32'h01C07FE0;
rows[601]=32'h01C1F9F8;
rows[602]=32'h01C7F0F8;
rows[603]=32'h01C3E078;
rows[604]=32'h01C18010;
rows[605]=32'h060F9800;
rows[606]=32'h070F3C40;
rows[607]=32'h0F9E78E0;
rows[608]=32'h07BFF1F0;
rows[609]=32'h033FE3C0;
rows[610]=32'h00BFC180;
rows[611]=32'h03C79180;
rows[612]=32'h07CF3BC0;
rows[613]=32'h1FBE3FF0;
rows[614]=32'h1F7FFCF8;
rows[615]=32'h0C3FFE70;
rows[616]=32'h083FEE20;
rows[617]=32'h00000C00;
rows[618]=32'h00380000;
rows[619]=32'h00780000;
rows[620]=32'h00F80000;
rows[621]=32'h00FFFFE0;
rows[622]=32'h01FFFFE0;
rows[623]=32'h03FFFFE0;
rows[624]=32'h07F007C0;
rows[625]=32'h1FF80F80;
rows[626]=32'h1FFE3F00;
rows[627]=32'h0E3FFE00;
rows[628]=32'h0C1FF800;
rows[629]=32'h00FFFF00;
rows[630]=32'h1FE00FF8;
rows[631]=32'h08000070;
rows[632]=32'h01E07C00;
rows[633]=32'h01E03C00;
rows[634]=32'h039FFFF8;
rows[635]=32'h07800000;
rows[636]=32'h0F8301C0;
rows[637]=32'h0F8F01F0;
rows[638]=32'h1F8F01E0;
rows[639]=32'h1F8781E0;
rows[640]=32'h3F8783E0;
rows[641]=32'h1F8783C0;
rows[642]=32'h1F83C3C0;
rows[643]=32'h1B83C3C0;
rows[644]=32'h0383C780;
rows[645]=32'h0381E780;
rows[646]=32'h0381EF00;
rows[647]=32'h03818F00;
rows[648]=32'h03800F00;
rows[649]=32'h03BFFFFC;
rows[650]=32'h03800000;
rows[651]=32'h00001FE0;
rows[652]=32'h07FFFF80;
rows[653]=32'h00F3CF30;
rows[654]=32'h1FF3CFF8;
rows[655]=32'h1FF3CFC0;
rows[656]=32'h00F3CF18;
rows[657]=32'h00F7CF3C;
rows[658]=32'h0FFFEFF8;
rows[659]=32'h1FFFF7F8;
rows[660]=32'h1FFFFBF0;
rows[661]=32'h1C7BFC00;
rows[662]=32'h00FBDE00;
rows[663]=32'h03F3DF00;
rows[664]=32'h07E3CFE0;
rows[665]=32'h1FC3C3F8;
rows[666]=32'h1F03C1F8;
rows[667]=32'h1E03C078;
rows[668]=32'h0803C010;
rows[669]=32'h1E1E0078;
rows[670]=32'h1E3E0078;
rows[671]=32'h1E7FFFF0;
rows[672]=32'h01FFFF80;
rows[673]=32'h0FF81F00;
rows[674]=32'h0F3C3E00;
rows[675]=32'h061FFC00;
rows[676]=32'h007FFE00;
rows[677]=32'h0FFFFFE0;
rows[678]=32'h1FFC1FF8;
rows[679]=32'h1FC003F8;
rows[680]=32'h1BFFFFF8;
rows[681]=32'h1FFFFCF0;
rows[682]=32'h1FFFBCF0;
rows[683]=32'h01E03CF0;
rows[684]=32'h01C03CF0;
rows[685]=32'h03FFBCF0;
rows[686]=32'h07FFBCF0;
rows[687]=32'h0F07BCF0;
rows[688]=32'h0E0F3CF0;
rows[689]=32'h1E0F3CF0;
rows[690]=32'h3CCF3CF0;
rows[691]=32'h1DFE3CF0;
rows[692]=32'h19FE3CF0;
rows[693]=32'h087C3CF0;
rows[694]=32'h007C3CF0;
rows[695]=32'h00783CF0;
rows[696]=32'h00F83CF0;
rows[697]=32'h01F000F0;
rows[698]=32'h07E000F0;
rows[699]=32'h0FC000F0;
rows[700]=32'h0F803FF0;
rows[701]=32'h07001FF0;
rows[702]=32'h02001FE0;
rows[703]=32'h00001FC0;
rows[704]=32'h00F00000;
rows[705]=32'h01F00000;
rows[706]=32'h01E1E000;
rows[707]=32'h03C1E000;
rows[708]=32'h07FFFFE0;
rows[709]=32'h0FFF7FF0;
rows[710]=32'h0F0F7FF0;
rows[711]=32'h0F0F7870;
rows[712]=32'h0FFF7870;
rows[713]=32'h0F1C7870;
rows[714]=32'h0F3C7870;
rows[715]=32'h0F1E78F0;
rows[716]=32'h0F1E7FF0;
rows[717]=32'h0F3F7FF0;
rows[718]=32'h0FFF7FE0;
rows[719]=32'h0FFFFB80;
rows[720]=32'h1FE7F800;
rows[721]=32'h0F027800;
rows[722]=32'h0C007800;
rows[723]=32'h01E07800;
rows[724]=32'h01E0F000;
rows[725]=32'h01E1FFF8;
rows[726]=32'h0DE3FFF8;
rows[727]=32'h1DE7FFF8;
rows[728]=32'h1DEF80F8;
rows[729]=32'h1FEF21F0;
rows[730]=32'h0FE6FFE0;
rows[731]=32'h0FE07FC0;
rows[732]=32'h0FE0FF80;
rows[733]=32'h05EFFE00;
rows[734]=32'h01E7F1E0;
rows[735]=32'h01E701E0;
rows[736]=32'h01E001E0;
rows[737]=32'h03EFFFFC;
rows[738]=32'h07EFFFFC;
rows[739]=32'h0FEFFFFC;
rows[740]=32'h1FE1C1E0;
rows[741]=32'h1FE3C1E0;
rows[742]=32'h1FE3E1E0;
rows[743]=32'h19E1F1E0;
rows[744]=32'h11E0E1E0;
rows[745]=32'h01E0C1E0;
rows[746]=32'h01E03FE0;
rows[747]=32'h01E03FC0;
rows[748]=32'h0603C380;
rows[749]=32'h0F03C380;
rows[750]=32'h0F83C380;
rows[751]=32'h07DFFFF0;
rows[752]=32'h031FFFF0;
rows[753]=32'h0003C7C0;
rows[754]=32'h0003C380;
rows[755]=32'h1F83C380;
rows[756]=32'h1FBFFFF8;
rows[757]=32'h078787C0;
rows[758]=32'h07878380;
rows[759]=32'h078F8380;
rows[760]=32'h078F0380;
rows[761]=32'h079F0380;
rows[762]=32'h07870380;
rows[763]=32'h0FC20000;
rows[764]=32'h1FF00000;
rows[765]=32'h1EFFFFF8;
rows[766]=32'h1C3FFFF8;
rows[767]=32'h07807C00;
rows[768]=32'h021FFFF0;
rows[769]=32'h3F7FFFFC;
rows[770]=32'h3F000000;
rows[771]=32'h071FFFF0;
rows[772]=32'h071E00F0;
rows[773]=32'h075E00F0;
rows[774]=32'h07DE00F0;
rows[775]=32'h0F9E01F0;
rows[776]=32'h0F1E3FE0;
rows[777]=32'h061E1FE0;
rows[778]=32'h061E1F80;
rows[779]=32'h00F83C00;
rows[780]=32'h0FCFFFF0;
rows[781]=32'h1FC03C00;
rows[782]=32'h0783C1E0;
rows[783]=32'h0FF81FF0;
rows[784]=32'h1FFC3FF8;
rows[785]=32'h0FE007F0;
rows[786]=32'h0E000030;
rows[787]=32'h007C0000;
rows[788]=32'h01FFFF00;
rows[789]=32'h0F807C00;
rows[790]=32'h0F83C1E0;
rows[791]=32'h07800030;
rows[792]=32'h07800038;
rows[793]=32'h07800078;
rows[794]=32'h07C00078;
rows[795]=32'h07FFFFF8;
rows[796]=32'h0001C000;
rows[797]=32'h1F0000F8;
rows[798]=32'h1F0E0078;
rows[799]=32'h1F0F8078;
rows[800]=32'h00F01F00;
rows[801]=32'h01F83E00;
rows[802]=32'h01FF7E00;
rows[803]=32'h000FFF00;
rows[804]=32'h00FFFFC0;
rows[805]=32'h1FFFBFF0;
rows[806]=32'h1FFE07F0;
rows[807]=32'h0FF001F0;
rows[808]=32'h0E000060;
rows[809]=32'h03F00FC0;
rows[810]=32'h07E007E0;
rows[811]=32'h0E7FFE70;
rows[812]=32'h0C7FFE30;
rows[813]=32'h0F07C0F0;
rows[814]=32'h0F07E0F0;
rows[815]=32'h0F0FF0F0;
rows[816]=32'h0F1FF8F0;
rows[817]=32'h0F3E7CF0;
rows[818]=32'h0F7E3EF0;
rows[819]=32'h0FFC1FF0;
rows[820]=32'h0F780EF0;
rows[821]=32'h0F3004F0;
rows[822]=32'h0F0000F0;
rows[823]=32'h0F001FF0;
rows[824]=32'h0F001FE0;
rows[825]=32'h0F001FC0;
rows[826]=32'h0F000E00;
rows[827]=32'h03C1FFE0;
rows[828]=32'h03F9FFE0;
rows[829]=32'h07B801E0;
rows[830]=32'h073FFFF8;
rows[831]=32'h0F39C000;
rows[832]=32'h1F39C000;
rows[833]=32'h1F39FFF0;
rows[834]=32'h3F3BFFF0;
rows[835]=32'h1F3BFFF0;
rows[836]=32'h1F3F9E00;
rows[837]=32'h173B1E00;
rows[838]=32'h07383F00;
rows[839]=32'h07383F80;
rows[840]=32'h07387BC0;
rows[841]=32'h0739F3E0;
rows[842]=32'h0703F1F8;
rows[843]=32'h070FE0F8;
rows[844]=32'h07078078;
rows[845]=32'h07020010;
rows[846]=32'h0FC00000;
rows[847]=32'h0FE00000;
rows[848]=32'h1CE00000;
rows[849]=32'h07C00000;
rows[850]=32'h01000000;
rows[851]=32'h001E7800;
rows[852]=32'h03F81FC0;
rows[853]=32'h07F00FE0;
rows[854]=32'h1FE007F8;
rows[855]=32'h1F8003F8;
rows[856]=32'h04000060;
rows[857]=32'h01818100;
rows[858]=32'h03C783C0;
rows[859]=32'h03C7C3E0;
rows[860]=32'h01E3C7C0;
rows[861]=32'h01E1CF80;
rows[862]=32'h00007E00;
rows[863]=32'h007FE000;
rows[864]=32'h007FC000;
rows[865]=32'h003F8000;
rows[866]=32'h00700E00;
rows[867]=32'h00FE7FC0;
rows[868]=32'h1FF80FF8;
rows[869]=32'h1FE003F0;
rows[870]=32'h007E7E00;
rows[871]=32'h01FDBF00;
rows[872]=32'h03F38FC0;
rows[873]=32'h0FE3C7F8;
rows[874]=32'h0FFFFFB0;
rows[875]=32'h03C00380;
rows[876]=32'h03C703E0;
rows[877]=32'h03CFFFE0;
rows[878]=32'h03C7FF80;
rows[879]=32'h03CFFF00;
rows[880]=32'h03FF3FE0;
rows[881]=32'h03FF0FF0;
rows[882]=32'h03F801E0;
rows[883]=32'h01800060;
rows[884]=32'h01038000;
rows[885]=32'h01C38000;
rows[886]=32'h03C387F8;
rows[887]=32'h07DFF7F8;
rows[888]=32'h0F9FF7F8;
rows[889]=32'h0F1FF000;
rows[890]=32'h1F078000;
rows[891]=32'h3E038000;
rows[892]=32'h1DBFF000;
rows[893]=32'h09FFF000;
rows[894]=32'h03FFF7FC;
rows[895]=32'h07C007FC;
rows[896]=32'h0F8387FC;
rows[897]=32'h0F0381F0;
rows[898]=32'h1F0380F0;
rows[899]=32'h3F3FF8F0;
rows[900]=32'h1F3FF8F0;
rows[901]=32'h170780F0;
rows[902]=32'h070380F0;
rows[903]=32'h0703BCF0;
rows[904]=32'h077FFCE0;
rows[905]=32'h077FFDE0;
rows[906]=32'h077FFFE0;
rows[907]=32'h073E0FE0;
rows[908]=32'h07000FC0;
rows[909]=32'h00000400;
rows[910]=32'h000007F0;
rows[911]=32'h00001F80;
rows[912]=32'h00003F00;
rows[913]=32'h00FFE000;
rows[914]=32'h00FFC000;
rows[915]=32'h00FF8000;
rows[916]=32'h00FF0000;
rows[917]=32'h00FC0000;
rows[918]=32'h07E001E0;
rows[919]=32'h0FE001E0;
rows[920]=32'h1DFFFFE0;
rows[921]=32'h19E001E0;
rows[922]=32'h11E001E0;
rows[923]=32'h01E003E0;
rows[924]=32'h01E07FE0;
rows[925]=32'h01E07FC0;
rows[926]=32'h01E03F00;
rows[927]=32'h07000000;
rows[928]=32'h070703F0;
rows[929]=32'h070F8FC0;
rows[930]=32'h0707FF00;
rows[931]=32'h0701FE00;
rows[932]=32'h0F7FFFF8;
rows[933]=32'h0F007878;
rows[934]=32'h0F0078F0;
rows[935]=32'h1E007860;
rows[936]=32'h1E0FF800;
rows[937]=32'h0C0FF000;
rows[938]=32'h04040000;
rows[939]=32'h0007FFF0;
rows[940]=32'h00FBDF00;
rows[941]=32'h07E3C7E0;
rows[942]=32'h1F83C1F8;
rows[943]=32'h0E03C0F0;
rows[944]=32'h0403C020;
rows[945]=32'h01E01800;
rows[946]=32'h1FFEFFF8;
rows[947]=32'h1E1FF078;
rows[948]=32'h1E1FE078;
rows[949]=32'h1E1E6078;
rows[950]=32'h1E1E3078;
rows[951]=32'h1E1E7078;
rows[952]=32'h1FFEF878;
rows[953]=32'h1FFE7C78;
rows[954]=32'h1FFE3E78;
rows[955]=32'h1E1E1E78;
rows[956]=32'h1E1E1C78;
rows[957]=32'h1E1E0870;
rows[958]=32'h1E1E0070;
rows[959]=32'h1FFE00F0;
rows[960]=32'h1FFE3FF0;
rows[961]=32'h1E1E1FE0;
rows[962]=32'h1E1E1FC0;
rows[963]=32'h03C001E0;
rows[964]=32'h078E1F00;
rows[965]=32'h079FFF80;
rows[966]=32'h079FFFC0;
rows[967]=32'h0F8FFFF0;
rows[968]=32'h0F003BFC;
rows[969]=32'h1F6000F8;
rows[970]=32'h1EFE0078;
rows[971]=32'h0EFFE030;
rows[972]=32'h04FFFC00;
rows[973]=32'h0003FF00;
rows[974]=32'h00000600;
rows[975]=32'h03C07800;
rows[976]=32'h03EFFFE0;
rows[977]=32'h1FF878E0;
rows[978]=32'h1BF878E0;
rows[979]=32'h1BD078E0;
rows[980]=32'h3BC078E0;
rows[981]=32'h3BDFFFF8;
rows[982]=32'h03C0FC00;
rows[983]=32'h03C1FE00;
rows[984]=32'h03C1EF00;
rows[985]=32'h03C3CF00;
rows[986]=32'h03C7C7C0;
rows[987]=32'h03DF87E0;
rows[988]=32'h03FF03F8;
rows[989]=32'h03FE01F8;
rows[990]=32'h03DC00F0;
rows[991]=32'h03C80020;
rows[992]=32'h01E1F000;
rows[993]=32'h01E1FFF0;
rows[994]=32'h01E1FFE0;
rows[995]=32'h00700000;
rows[996]=32'h01FFFFF0;
rows[997]=32'h0F9E3CF0;
rows[998]=32'h1F1E3CF0;
rows[999]=32'h0E3C78F0;
rows[1000]=32'h043C78F0;
rows[1001]=32'h007C78F0;
rows[1002]=32'h00F8F8F0;
rows[1003]=32'h01F0F0F0;
rows[1004]=32'h03F0F0F0;
rows[1005]=32'h07E1F0F0;
rows[1006]=32'h1FC3E0F0;
rows[1007]=32'h1F83E1E0;
rows[1008]=32'h0F07C1E0;
rows[1009]=32'h040F81E0;
rows[1010]=32'h001F83E0;
rows[1011]=32'h003F7FE0;
rows[1012]=32'h00FE7FE0;
rows[1013]=32'h00FC7FC0;
rows[1014]=32'h00787F00;
rows[1015]=32'h00300000;
rows[1016]=32'h00060000;
rows[1017]=32'h007C0600;
rows[1018]=32'h03E007C0;
rows[1019]=32'h0FC007E0;
rows[1020]=32'h0FFFFFFC;
rows[1021]=32'h04000038;
rows[1022]=32'h00000030;
rows[1023]=32'h07C003E0;
end
always @(posedge I_clk) O_row<=rows[I_addr];
endmodule
