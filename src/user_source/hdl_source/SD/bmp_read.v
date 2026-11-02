module bmp_read(

    input pip_req,input [31:0] pip_sector,
    output pip_ack,pip_valid,pip_end,pip_bus_busy,output [7:0] pip_byte,
    output reg [1:0] pip_catalog_valid,
    output reg [31:0] pip_sector0,pip_sector1,pip_size0,pip_size1,
    input                       clk,
    input                       rst,
    output                      ready,

    // Scan: BM 640x480/24 + RIFF/WAVE (same sequential sector walk, not FAT)
    input                       scan_start,
    input  [31:0]               scan_start_sector,
    input  [31:0]               scan_max_sector,
    input  [3:0]                scan_target_count,
    output reg                  scan_done,
    output reg                  scan_found_valid,
    output reg [31:0]           scan_found_sector,
    output reg [3:0]            scan_found_total,
    output reg                  wav_found_valid,
    output reg [31:0]           wav_found_sector,
    output reg [2:0]            wav_found_total,

    input load_video,load_video_clip,
    input [15:0] load_video_frame,input [31:0] load_video_size,
    input normalize_pause,normalize_done,
    output reg load_video_last,output wire [3:0] load_video_period,
    input                       load_start,
    input                       load_abort,     // pulse: drop BMP load, return IDLE (keep scan_done)
    input  [31:0]               load_sector,

    input                       wav_scan_start,
    output reg [31:0]           wav_resume_sector,
    output reg                  wav_scan_done,
    output reg                  wav_catalog_complete, // FAT root scan is authoritative

    input                       wav_load_start,
    input                       wav_spi_hold,   // pause between sectors while U33 page-programs
    input  [31:0]               wav_load_sector,
    input  [19:0]               wav_pcm_base,
    output reg                  wav_pcm_we,
    output reg [19:0]           wav_pcm_addr,
    output reg [15:0]           wav_pcm_data,
    output reg                  wav_load_done,
    output reg                  wav_load_ok,
    output reg                  wav_is_rail,
    output reg                  load_fail,

    input                       sd_init_done,
    output reg [3:0]            state_code,
    input  [15:0]               bmp_width,
    input  [15:0]               bmp_height,

    output reg                  write_req,
    input                       write_req_ack,

    output reg                  sd_sec_read,
    output reg [31:0]           sd_sec_read_addr,
    input  [7:0]                sd_sec_read_data,
    input                       sd_sec_read_data_valid,
    input                       sd_sec_read_end,

    output wire [15:0] source_width,source_height,
    output reg                  bmp_data_wr_en,
    output reg [23:0]           bmp_data
);

localparam ST_PIP=4'd8;
reg [3:0] pip_resume;
reg [31:0] pip_resume_sector;
reg pip_cancel;
reg ent_pip,ent_pip_slot,ent_rail,rail_catalog_preferred;
reg [31:0] ent_size;
localparam ST_IDLE      = 4'd0;
localparam ST_SCAN      = 4'd1;
localparam ST_LOAD_HDR  = 4'd2;
localparam ST_LOAD_WAIT = 4'd3;
localparam ST_LOAD_DATA = 4'd4;
localparam ST_WAV_LOAD  = 4'd5;
localparam ST_WAV_SCAN  = 4'd6;
localparam ST_FAT       = 4'd7;
localparam PH_MBR       = 3'd0;
localparam PH_BOOT      = 3'd1;
localparam PH_DIR       = 3'd2;
localparam PH_EMIT      = 3'd3;
localparam PH_PREP_BASE = 3'd4;
localparam PH_PREP_FAT  = 3'd5;
localparam PH_PREP_DIR  = 3'd6;
localparam integer WAV_KEEP_SAMPLES = 960000; // 48 kHz mono * 20 s
localparam integer WAV_BYTE_CAP     = 2100000;
localparam integer WAV_HUNT_AFTER_BMP = 4096;

reg [3:0]  state;
reg [9:0]  rd_cnt;
reg        wav_sec_pend;

reg [7:0]  header_0, header_1, header_2, header_3;
reg [7:0]  wave_0, wave_1, wave_2, wave_3;
reg [31:0] file_len;
reg [31:0] riff_payload;
reg [31:0] pixel_offset;
reg [31:0] width;
reg [31:0] height;
reg [15:0] bit_count;
reg [31:0] compression;

reg [31:0] scan_sector;
reg [31:0] load_sector_latched;
reg [31:0] bmp_len_cnt;
reg [1:0]  bmp_byte_idx;
reg [12:0] hunt_left;
reg        bmp_quota_met;

reg [2:0]  fat_ph;
reg [31:0] fat_part;
reg [31:0] fat_data;
reg [31:0] fat_root_offset;
reg [31:0] fat_dir_sec;
reg [6:0]  fat_dir_left;
reg [2:0]  fat_shift;
reg        fat_dir_end;
reg [3:0]  fat_bmp_n;
reg [2:0]  fat_wav_n;
reg [3:0]  fat_emit;
reg        fat_emit_wav;
reg [31:0] qb0, qb1, qb2, qb3, qb4, qb5, qb6, qb7;
reg [3:0] kb0,kb1,kb2,kb3,kb4,kb5,kb6,kb7;
reg [7:0] named_mask;
reg ent_named;
reg [2:0] ent_number;
reg [31:0] qw0, qw1, qw2, qw3;

reg [7:0]  cap0;
reg [15:0] cap_bps;
reg [7:0]  cap_spc;
reg [7:0]  cap_nfats;
reg [31:0] cap_rsvd;
reg [31:0] cap_fatsz;
reg [31:0] cap_root;
reg [31:0] cap_lba;
reg [7:0]  cap_f0, cap_f1, cap_f2, cap_f3;
reg        cap55;
reg        capaa;
reg [15:0] ent_hi;
reg [15:0] ent_lo;
reg [7:0]  ent_e0, ent_e1, ent_e2;
reg        ent_skip;
reg        ent_push;
reg        ent_wav;
reg [31:0] ent_sec;

reg [31:0] fat_clus_off_q, fat_scaled_off_q, fat_sec_now;
wire [31:0] fat_scaled_off = fat_clus_off_q << fat_shift;
always @(posedge clk or posedge rst) begin
    if (rst) begin
        fat_clus_off_q <= 32'd0;
        fat_scaled_off_q <= 32'd0;
        fat_sec_now <= 32'd0;
    end else begin
        fat_clus_off_q <= {ent_hi, ent_lo} - 32'd2;
        fat_scaled_off_q <= fat_scaled_off;
        fat_sec_now <= fat_data + fat_scaled_off_q;
    end
end
wire        cap_is_fat32 = (cap_f0 == "F") && (cap_f1 == "A") &&
                           (cap_f2 == "T") && (cap_f3 == "3");
wire        cap_boot_ok  = cap55 && capaa && cap_is_fat32 && (cap_bps == 16'd512) &&
                           ((cap_spc == 8'd1) || (cap_spc == 8'd2) || (cap_spc == 8'd4) ||
                            (cap_spc == 8'd8) || (cap_spc == 8'd16) || (cap_spc == 8'd32) ||
                            (cap_spc == 8'd64) || (cap_spc == 8'd128));
wire [31:0] cap_fat_bytes = (cap_nfats == 8'd2) ? {cap_fatsz[30:0], 1'b0} : cap_fatsz;
wire [31:0] cap_root_use  = (cap_root < 32'd2) ? 32'd2 : cap_root;


function [7:0] prefix_mask;
    input [3:0] n;
    begin case(n)
        1:prefix_mask=8'h01;2:prefix_mask=8'h03;3:prefix_mask=8'h07;4:prefix_mask=8'h0f;
        5:prefix_mask=8'h1f;6:prefix_mask=8'h3f;7:prefix_mask=8'h7f;default:prefix_mask=8'hff;
    endcase end
endfunction
function [31:0] named_sector;
 input [3:0] n;
 reg [6:0] hit;reg [7:0] pick;
 begin
  hit=7'd0;
  if(kb0==n)hit[0]=1'b1;
  if(kb1==n)hit[1]=1'b1;
  if(kb2==n)hit[2]=1'b1;
  if(kb3==n)hit[3]=1'b1;
  if(kb4==n)hit[4]=1'b1;
  if(kb5==n)hit[5]=1'b1;
  if(kb6==n)hit[6]=1'b1;
  pick[0]=hit[0];
  pick[1]=hit[1] && !(|hit[0:0]);
  pick[2]=hit[2] && !(|hit[1:0]);
  pick[3]=hit[3] && !(|hit[2:0]);
  pick[4]=hit[4] && !(|hit[3:0]);
  pick[5]=hit[5] && !(|hit[4:0]);
  pick[6]=hit[6] && !(|hit[5:0]);
  pick[7]=!(|hit);
  named_sector=
   ({32{pick[0]}} & qb0) |
   ({32{pick[1]}} & qb1) |
   ({32{pick[2]}} & qb2) |
   ({32{pick[3]}} & qb3) |
   ({32{pick[4]}} & qb4) |
   ({32{pick[5]}} & qb5) |
   ({32{pick[6]}} & qb6) |
   ({32{pick[7]}} & qb7);
 end
endfunction

// Select a three-bit bank first, then use one common 32-bit data mux.
// Preserve first-match priority and the legacy/default bank exactly.
reg [2:0] selected_emit_bank;
always @*begin
 case(fat_emit)
  0:selected_emit_bank=0;1:selected_emit_bank=1;2:selected_emit_bank=2;
  3:selected_emit_bank=3;4:selected_emit_bank=4;5:selected_emit_bank=5;
  6:selected_emit_bank=6;default:selected_emit_bank=7;
 endcase
 if(named_mask==prefix_mask(fat_bmp_n))begin
  if(kb0==fat_emit+4'd1)selected_emit_bank=0;
  else if(kb1==fat_emit+4'd1)selected_emit_bank=1;
  else if(kb2==fat_emit+4'd1)selected_emit_bank=2;
  else if(kb3==fat_emit+4'd1)selected_emit_bank=3;
  else if(kb4==fat_emit+4'd1)selected_emit_bank=4;
  else if(kb5==fat_emit+4'd1)selected_emit_bank=5;
  else if(kb6==fat_emit+4'd1)selected_emit_bank=6;
  else selected_emit_bank=7;
 end
end
function [31:0] sector_by_bank;
 input [2:0] bank;
 begin case(bank)
  0:sector_by_bank=qb0;1:sector_by_bank=qb1;2:sector_by_bank=qb2;
  3:sector_by_bank=qb3;4:sector_by_bank=qb4;5:sector_by_bank=qb5;
  6:sector_by_bank=qb6;default:sector_by_bank=qb7;
 endcase end
endfunction

function [2:0] spc_shift;
    input [7:0] spc;
    begin
        case (spc)
            8'd2:   spc_shift = 3'd1;
            8'd4:   spc_shift = 3'd2;
            8'd8:   spc_shift = 3'd3;
            8'd16:  spc_shift = 3'd4;
            8'd32:  spc_shift = 3'd5;
            8'd64:  spc_shift = 3'd6;
            8'd128: spc_shift = 3'd7;
            default: spc_shift = 3'd0;
        endcase
    end
endfunction

function [31:0] clus2sec;
    input [31:0] base;
    input [31:0] clus;
    input [2:0]  sh;
    reg   [31:0] off;
    begin
        off = clus - 32'd2;
        case (sh)
            3'd1: clus2sec = base + {off[30:0], 1'b0};
            3'd2: clus2sec = base + {off[29:0], 2'b00};
            3'd3: clus2sec = base + {off[28:0], 3'b000};
            3'd4: clus2sec = base + {off[27:0], 4'b0000};
            3'd5: clus2sec = base + {off[26:0], 5'b00000};
            3'd6: clus2sec = base + {off[25:0], 6'b000000};
            3'd7: clus2sec = base + {off[24:0], 7'b0000000};
            default: clus2sec = base + off;
        endcase
    end
endfunction

function [6:0] spc_count;
    input [2:0] sh;
    begin
        case (sh)
            3'd0: spc_count = 7'd1;
            3'd1: spc_count = 7'd2;
            3'd2: spc_count = 7'd4;
            3'd3: spc_count = 7'd8;
            3'd4: spc_count = 7'd16;
            3'd5: spc_count = 7'd32;
            default: spc_count = 7'd64;
        endcase
    end
endfunction

reg [31:0] wav_byte_cnt;
reg [2:0] wav_skip_bytes;
reg [19:0] wav_samp_cnt;
reg        wav_pair;
reg [7:0]  wav_lo;
reg        wav_seen_data;
reg [31:0] wav_data_mark; // last 4 bytes window for "data"

wire header_match;
wire is_riff;
wire is_wave;
wire bmp_data_valid;
wire [31:0] bmp_skip;
wire [31:0] wav_skip;
wire [31:0] next_if_bmp;
wire [31:0] next_if_wav;
wire [31:0] next_if_miss;

reg [31:0] pixel_bytes_q,pixel_end_q;
always @(posedge clk or posedge rst)begin
    if(rst)begin pixel_bytes_q<=0;pixel_end_q<=0;end
    else begin
        pixel_bytes_q<=width==320?32'd230400:width==800?32'd1440000:32'd921600;
        pixel_end_q<=pixel_offset+pixel_bytes_q;
    end
end
wire [31:0] height_abs = height[31] ? (~height + 32'd1) : height;
localparam ST_VIDEO_DRAIN=4'd9;
reg video_latched,video_clip_latched,video_wait;
reg [15:0] video_requested,video_frames;
reg [31:0] video_size;
reg [7:0] video_width_lo,video_width_hi,video_height,video_period,video_sectors,video_bits,video_id,video_low;
// Decode directly on header byte 4's capture edge. These static format flags
// have the same validity cycle as the former raw version register, while the
// decoder reset, geometry and byte-count paths no longer start with an 8-bit
// version comparison. Unknown binary version values leave all flags clear.
reg video_256,video_q512,video_q640,video_compressed;
reg [7:0] video_height_hi;
reg video_368,video_384,video_352,video_320;
reg [7:0] video_sectors_hi;
assign load_video_period=4'd6;
// Capture the invariant descriptor on that same edge, removing the format
// mux from the byte-limit/FSM and sector-product paths without adding latency.
reg [31:0] video_frame_bytes,video_stride;
wire [15:0] video_index=video_requested<video_frames?video_requested:16'd0;
wire [31:0] vf={16'd0,video_frames};
// A validated clip contains at most 4096 frames. Register just its 12-bit
// selected frame index while the header is being read. This separates the
// bounds check from sector arithmetic with less state than a 32-bit offset.
reg [11:0] video_offset_index_q;
always @(posedge clk or posedge rst)begin
 if(rst)video_offset_index_q<=0;
 else video_offset_index_q<=video_index[11:0];
end
wire [31:0] vi={20'd0,video_offset_index_q};
// Reuse one 16x9 multiplier for the two sector products. The header
// fields settle hundreds of clocks before validation; each result refreshes
// every other clock. All accepted clips fit in 21 sector bits.
reg video_product_phase;
wire [15:0] video_product_factor=video_product_phase?{4'd0,video_offset_index_q}:video_frames;
wire [24:0] video_sector_product=video_product_factor*video_stride[8:0];
reg [20:0] video_file_sectors_q,video_offset_sectors_q;
always @(posedge clk or posedge rst)begin
 if(rst)begin video_product_phase<=0;video_file_sectors_q<=0;video_offset_sectors_q<=0;end
 else begin
  video_product_phase<=~video_product_phase;
  if(video_product_phase)video_offset_sectors_q<=video_sector_product[20:0];
  else video_file_sectors_q<=video_sector_product[20:0];
 end
end
wire [31:0] video_file_bytes={2'd0,video_file_sectors_q,9'd0}+32'd512;
wire [31:0] video_frame_sector={11'd0,video_offset_sectors_q};
wire video_header_ok=header_0=="P" && header_1=="V" && header_2=="F" && header_3=="4" &&
 ((video_256 && video_width_lo==0 && video_height==144 && video_sectors==144 && video_sectors_hi==0) ||
  (video_320 && video_width_lo==64 && video_height==180 && video_sectors==225 && video_sectors_hi==0) ||
  (video_352 && video_width_lo==96 && video_height==198 && video_sectors==17 && video_sectors_hi==1) ||
  (video_384 && video_width_lo==128 && video_height==216 && video_sectors==68 && video_sectors_hi==1) ||
  (video_368 && video_width_lo==112 && video_height==207 && video_sectors==42 && video_sectors_hi==1) ||
  (video_q512 && video_width_lo==0 && video_height==32 && video_height_hi==1 && video_sectors==128 && video_sectors_hi==1) ||
  (video_q640 && video_width_lo==128 && video_height==104 && video_height_hi==1 && video_sectors==128 && video_sectors_hi==1)) &&
 video_width_hi==(video_compressed?2:1) && video_period==6 && video_bits==16 && video_id=={7'd0,video_clip_latched} &&
 video_frames!=0 && video_frames<=4096 && video_size==video_file_bytes;
always @(posedge clk or posedge rst)begin
 if(rst)begin
  video_256<=0;video_q512<=0;video_q640<=0;video_compressed<=0;
  video_368<=0;video_384<=0;video_352<=0;video_320<=0;
  video_frame_bytes<=32'd73728;video_stride<=32'd144;
  video_width_lo<=0;video_width_hi<=0;video_height<=0;video_period<=0;
  video_height_hi<=0;video_sectors<=0;video_sectors_hi<=0;video_bits<=0;video_id<=0;video_frames<=0;
 end else if(state==ST_LOAD_HDR && video_latched && sd_sec_read_data_valid)case(rd_cnt)
  4:begin
   video_256<=sd_sec_read_data==8'd6;video_320<=sd_sec_read_data==8'd7;
   video_352<=sd_sec_read_data==8'd8;video_384<=sd_sec_read_data==8'd9;
   video_368<=sd_sec_read_data==8'd10;video_q512<=sd_sec_read_data==8'd11;
   video_q640<=sd_sec_read_data==8'd12;
   video_compressed<=sd_sec_read_data==8'd11 || sd_sec_read_data==8'd12;
   case(sd_sec_read_data)
    7:begin video_frame_bytes<=32'd115200;video_stride<=32'd225;end
    8:begin video_frame_bytes<=32'd139392;video_stride<=32'd273;end
    9:begin video_frame_bytes<=32'd165888;video_stride<=32'd324;end
    10:begin video_frame_bytes<=32'd152352;video_stride<=32'd298;end
    11,12:begin video_frame_bytes<=32'd196608;video_stride<=32'd384;end
    default:begin video_frame_bytes<=32'd73728;video_stride<=32'd144;end
   endcase
  end
  5:video_width_lo<=sd_sec_read_data;
  6:video_height<=sd_sec_read_data;7:video_period<=sd_sec_read_data;
  8:video_frames[7:0]<=sd_sec_read_data;9:video_frames[15:8]<=sd_sec_read_data;
  10:video_sectors<=sd_sec_read_data;11:video_bits<=sd_sec_read_data;
  12:video_id<=sd_sec_read_data;13:video_width_hi<=sd_sec_read_data;14:video_height_hi<=sd_sec_read_data;15:video_sectors_hi<=sd_sec_read_data;
  default:;
 endcase
end


wire q_done,q_error,q_sector_ready,q_pixel_valid;
wire [15:0] q_pixel;
exhibit_q565_decoder u_q565(.clk(clk),
 // write_req is a registered load-boundary signal shared with the normalizer.
 // Never use combinational state decoding as an asynchronous decoder reset:
 // DATA -> DRAIN state-bit skew must not erase the completed frame status.
 .rst(rst || load_abort || write_req || !video_latched || !video_compressed),
 .byte_valid(sd_sec_read_data_valid && state==ST_LOAD_DATA && video_compressed),.byte_data(sd_sec_read_data),
 .no_more(state==ST_VIDEO_DRAIN),.pixel_limit(video_q640?18'd230400:18'd147456),.pixel_ready(!normalize_pause),
 .pixel_valid(q_pixel_valid),.pixel(q_pixel),.sector_ready(q_sector_ready),.done(q_done),.error(q_error));

assign ready = (state == ST_IDLE) && !pip_req;
assign pip_bus_busy=(state==ST_PIP);
assign pip_valid=(state==ST_PIP) && sd_sec_read_data_valid;
assign pip_end=(state==ST_PIP) && sd_sec_read_end;
assign pip_byte=sd_sec_read_data;
assign pip_ack=sd_init_done && !load_abort &&
 (((state==ST_IDLE) && !scan_start && !load_start && !wav_scan_start && !wav_load_start && pip_req) ||
 ((state==ST_LOAD_DATA) && sd_sec_read_end && bmp_len_cnt<file_len && pip_req));
assign source_width=video_latched?(video_q640?16'd640:video_q512?16'd512:video_368?16'd368:video_384?16'd384:video_352?16'd352:video_320?16'd320:16'd256):width[15:0];
assign source_height=video_latched?(video_q640?16'd360:video_q512?16'd288:video_368?16'd207:video_384?16'd216:video_352?16'd198:video_320?16'd180:16'd144):height[15:0];
assign header_match = (header_0 == "B") &&
                      (header_1 == "M") &&
                      (((width==320)&&(height==240)) || ((width==640)&&(height==480)) || ((width==800)&&(height==600))) &&
                      pixel_offset>=54 && pixel_end_q>=pixel_offset && pixel_end_q<=file_len &&
                      (bit_count    == 16'd24) &&
                      (compression  == 32'd0);
assign is_riff = (header_0 == "R") && (header_1 == "I") &&
                 (header_2 == "F") && (header_3 == "F");
assign is_wave = (wave_0 == "W") && (wave_1 == "A") &&
                 (wave_2 == "V") && (wave_3 == "E");
assign bmp_data_valid = (sd_sec_read_data_valid == 1'b1) &&
                        (bmp_len_cnt >= pixel_offset) &&
                        (bmp_len_cnt < pixel_end_q);
reg [31:0] bmp_round_q, wav_round_q;
reg [31:0] bmp_skip_q, wav_skip_q;
always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_round_q <= 32'd0; wav_round_q <= 32'd0;
        bmp_skip_q <= 32'd1; wav_skip_q <= 32'd1;
    end else begin
        bmp_round_q <= file_len + 32'd511;
        wav_round_q <= riff_payload + 32'd519;
        bmp_skip_q <= ((file_len == 32'd0) || (file_len > 32'd1500000)) ?
                       32'd1 : (bmp_round_q >> 9);
        wav_skip_q <= wav_round_q >> 9;
    end
end
assign bmp_skip = bmp_skip_q;
assign wav_skip = wav_skip_q;
assign next_if_bmp  = scan_sector + bmp_skip;
assign next_if_wav  = scan_sector + (((wav_skip == 32'd0) || (wav_skip > 32'd4096)) ? 32'd1 : wav_skip);
assign next_if_miss = scan_sector + 32'd1;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        rd_cnt <= 10'd0;
    end else if ((state == ST_SCAN) || (state == ST_LOAD_HDR) || (state == ST_WAV_LOAD) ||
                 (state == ST_WAV_SCAN) || (state == ST_FAT)) begin
        if (sd_sec_read_data_valid)
            rd_cnt <= rd_cnt + 10'd1;
        else if (sd_sec_read_end)
            rd_cnt <= 10'd0;
    end else begin
        rd_cnt <= 10'd0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        header_0     <= 8'd0;
        header_1     <= 8'd0;
        header_2     <= 8'd0;
        header_3     <= 8'd0;
        wave_0       <= 8'd0;
        wave_1       <= 8'd0;
        wave_2       <= 8'd0;
        wave_3       <= 8'd0;
        file_len     <= 32'd0;
        riff_payload <= 32'd0;
        pixel_offset <= 32'd54;
        width        <= 32'd0;
        height       <= 32'd0;
        bit_count    <= 16'd0;
        compression  <= 32'd0;
    end else if (((state == ST_SCAN) || (state == ST_LOAD_HDR) || (state == ST_WAV_SCAN)) && sd_sec_read_data_valid) begin
        case (rd_cnt)
            10'd0 : header_0 <= sd_sec_read_data;
            10'd1 : header_1 <= sd_sec_read_data;
            10'd2 : begin
                header_2        <= sd_sec_read_data;
                file_len[7:0]   <= sd_sec_read_data;
            end
            10'd3 : begin
                header_3        <= sd_sec_read_data;
                file_len[15:8]  <= sd_sec_read_data;
            end
            10'd4 : begin
                file_len[23:16]     <= sd_sec_read_data;
                riff_payload[7:0]   <= sd_sec_read_data;
            end
            10'd5 : begin
                file_len[31:24]     <= sd_sec_read_data;
                riff_payload[15:8]  <= sd_sec_read_data;
            end
            10'd6 : riff_payload[23:16] <= sd_sec_read_data;
            10'd7 : riff_payload[31:24] <= sd_sec_read_data;
            10'd8 : wave_0 <= sd_sec_read_data;
            10'd9 : wave_1 <= sd_sec_read_data;
            10'd10: begin
                wave_2              <= sd_sec_read_data;
                pixel_offset[7:0]   <= sd_sec_read_data;
            end
            10'd11: begin
                wave_3              <= sd_sec_read_data;
                pixel_offset[15:8]  <= sd_sec_read_data;
            end
            10'd12: pixel_offset[23:16] <= sd_sec_read_data;
            10'd13: pixel_offset[31:24] <= sd_sec_read_data;
            10'd18: width[7:0] <= sd_sec_read_data;
            10'd19: width[15:8] <= sd_sec_read_data;
            10'd20: width[23:16] <= sd_sec_read_data;
            10'd21: width[31:24] <= sd_sec_read_data;
            10'd22: height[7:0] <= sd_sec_read_data;
            10'd23: height[15:8] <= sd_sec_read_data;
            10'd24: height[23:16] <= sd_sec_read_data;
            10'd25: height[31:24] <= sd_sec_read_data;
            10'd28: bit_count[7:0] <= sd_sec_read_data;
            10'd29: bit_count[15:8] <= sd_sec_read_data;
            10'd30: compression[7:0] <= sd_sec_read_data;
            10'd31: compression[15:8] <= sd_sec_read_data;
            10'd32: compression[23:16] <= sd_sec_read_data;
            10'd33: compression[31:24] <= sd_sec_read_data;
            default: ;
        endcase
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_len_cnt <= 32'd0;
    end else if (state == ST_LOAD_DATA) begin
        if (sd_sec_read_data_valid)
            bmp_len_cnt <= bmp_len_cnt + 32'd1;
    end else if (!(state==ST_PIP && pip_resume==ST_LOAD_DATA)) begin
        bmp_len_cnt <= 32'd0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_byte_idx <= 2'd0;
    end else if (state == ST_LOAD_DATA) begin
        if (bmp_data_valid && !video_latched)
            bmp_byte_idx <= (bmp_byte_idx == 2'd2) ? 2'd0 : (bmp_byte_idx + 2'd1);
    end else if (!(state==ST_PIP && pip_resume==ST_LOAD_DATA)) begin
        bmp_byte_idx <= 2'd0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_data_wr_en <= 1'b0;
        bmp_data       <= 24'd0;
    end else if (state == ST_LOAD_DATA && video_latched) begin
        bmp_data_wr_en<=0;
        if(video_compressed)begin
            bmp_data_wr_en<=q_pixel_valid;
            bmp_data<={q_pixel[15:11],q_pixel[15:13],q_pixel[10:5],q_pixel[10:9],q_pixel[4:0],q_pixel[4:2]};
        end else if(sd_sec_read_data_valid && bmp_len_cnt<video_frame_bytes)begin
            if(!bmp_len_cnt[0])video_low<=sd_sec_read_data;
            else begin
                bmp_data_wr_en<=1;
                bmp_data<={sd_sec_read_data[7:3],sd_sec_read_data[7:5],sd_sec_read_data[2:0],video_low[7:5],sd_sec_read_data[2:1],video_low[4:0],video_low[4:2]};
            end
        end
    end else if (state == ST_LOAD_DATA) begin
        if (bmp_data_valid) begin
            case (bmp_byte_idx)
                2'd0: begin
                    bmp_data_wr_en <= 1'b0;
                    bmp_data[7:0]  <= sd_sec_read_data;
                end
                2'd1: begin
                    bmp_data_wr_en <= 1'b0;
                    bmp_data[15:8] <= sd_sec_read_data;
                end
                2'd2: begin
                    bmp_data_wr_en  <= 1'b1;
                    bmp_data[23:16] <= sd_sec_read_data;
                end
                default: begin
                    bmp_data_wr_en <= 1'b0;
                end
            endcase
        end else begin
            bmp_data_wr_en <= 1'b0;
        end
    end else begin
        bmp_data_wr_en <= 1'b0;
    end
end

// RAIL/RAU1 identifies the generated five-announcement master WAV.
reg wav_rail_header;
always @(posedge clk or posedge rst)begin
 if(rst)begin wav_is_rail<=0;wav_rail_header<=0;end
 else if(wav_load_start)begin wav_is_rail<=0;wav_rail_header<=0;end
 else if(state==ST_WAV_LOAD && sd_sec_read_data_valid)begin
  if(wav_byte_cnt==39)wav_rail_header<=({sd_sec_read_data,wav_data_mark[31:8]}==32'h4c494152);
  if(wav_byte_cnt==47 && wav_rail_header)wav_is_rail<=({sd_sec_read_data,wav_data_mark[31:8]}==32'h31554152);
 end
end
// WAV: find "data", then keep every 16-bit mono sample (no stereo stride)
always @(posedge clk or posedge rst) begin
    if (rst) begin
        wav_pcm_we    <= 1'b0;
        wav_pcm_addr  <= 20'd0;
        wav_pcm_data  <= 16'd0;
        wav_byte_cnt  <= 32'd0;
        wav_skip_bytes<=3'd0;
        wav_samp_cnt  <= 20'd0;
        wav_pair      <= 1'b0;
        wav_lo        <= 8'd0;
        wav_seen_data <= 1'b0;
        wav_data_mark <= 32'd0;
    end else if (state != ST_WAV_LOAD) begin
        wav_pcm_we    <= 1'b0;
        wav_byte_cnt  <= 32'd0;
        wav_samp_cnt  <= 20'd0;
        wav_pair      <= 1'b0;
        wav_seen_data <= 1'b0;
        wav_data_mark <= 32'd0;
        wav_skip_bytes<=3'd0;
    end else begin
        wav_pcm_we <= 1'b0;
        if (sd_sec_read_data_valid) begin
            wav_byte_cnt  <= wav_byte_cnt + 32'd1;
            wav_data_mark <= {sd_sec_read_data, wav_data_mark[31:8]};
            if (!wav_seen_data && (wav_byte_cnt >= 32'd3) &&
                ({sd_sec_read_data, wav_data_mark[31:8]} == 32'h61746164)) begin
                wav_seen_data <= 1'b1;
                wav_skip_bytes<=3'd4;
                wav_pair      <= 1'b0;
            end else if(wav_seen_data && wav_skip_bytes!=0)begin
                wav_skip_bytes<=wav_skip_bytes-1'b1;
            end else if (wav_seen_data && (wav_samp_cnt < WAV_KEEP_SAMPLES[19:0])) begin
                if (!wav_pair)
                    wav_lo <= sd_sec_read_data;
                else begin
                    wav_pcm_we   <= 1'b1;
                    wav_pcm_addr <= wav_samp_cnt;
                    wav_pcm_data <= {sd_sec_read_data, wav_lo};
                    wav_samp_cnt <= wav_samp_cnt + 20'd1;
                end
                wav_pair <= ~wav_pair;
            end
        end else if ((wav_byte_cnt >= WAV_BYTE_CAP[31:0]) &&
                     (wav_samp_cnt < WAV_KEEP_SAMPLES[19:0])) begin
            wav_pcm_we   <= 1'b1;
            wav_pcm_addr <= wav_samp_cnt;
            wav_pcm_data <= 16'd0;
            wav_samp_cnt <= wav_samp_cnt + 20'd1;
        end
    end
end

// FAT32: MBR/boot + root directory. ent_push is one cycle, consumed by the main FSM.
always @(posedge clk or posedge rst) begin
    if (rst) begin
        cap0      <= 8'd0;
        cap_bps   <= 16'd0;
        cap_spc   <= 8'd0;
        cap_nfats <= 8'd0;
        cap_rsvd  <= 32'd0;
        cap_fatsz <= 32'd0;
        cap_root  <= 32'd0;
        cap_lba   <= 32'd0;
        cap_f0    <= 8'd0;
        cap_f1    <= 8'd0;
        cap_f2    <= 8'd0;
        cap_f3    <= 8'd0;
        cap55     <= 1'b0;
        capaa     <= 1'b0;
        ent_named <= 1'b0;ent_rail<=0;ent_pip<=0;ent_pip_slot<=0;ent_size<=0;
        pip_catalog_valid<=0;pip_sector0<=0;pip_sector1<=0;pip_size0<=0;pip_size1<=0;
        ent_number <= 3'd0;
        ent_hi    <= 16'd0;
        ent_lo    <= 16'd0;
        ent_e0    <= 8'd0;
        ent_e1    <= 8'd0;
        ent_e2    <= 8'd0;
        ent_skip  <= 1'b1;
        ent_push  <= 1'b0;
        ent_wav   <= 1'b0;
        ent_sec   <= 32'd0;
        fat_dir_end <= 1'b0;
    end else if ((state == ST_FAT) && sd_sec_read_data_valid &&
                 ((fat_ph == PH_MBR) || (fat_ph == PH_BOOT))) begin
        ent_push <= 1'b0;
        case (rd_cnt)
            10'd0:   cap0 <= sd_sec_read_data;
            10'd11:  cap_bps[7:0]  <= sd_sec_read_data;
            10'd12:  cap_bps[15:8] <= sd_sec_read_data;
            10'd13:  cap_spc <= sd_sec_read_data;
            10'd14:  cap_rsvd <= {24'd0, sd_sec_read_data};
            10'd15:  cap_rsvd[15:8] <= sd_sec_read_data;
            10'd16:  cap_nfats <= sd_sec_read_data;
            10'd36:  cap_fatsz <= {24'd0, sd_sec_read_data};
            10'd37:  cap_fatsz[15:8]  <= sd_sec_read_data;
            10'd38:  cap_fatsz[23:16] <= sd_sec_read_data;
            10'd39:  cap_fatsz[31:24] <= sd_sec_read_data;
            10'd44:  cap_root <= {24'd0, sd_sec_read_data};
            10'd45:  cap_root[15:8]  <= sd_sec_read_data;
            10'd46:  cap_root[23:16] <= sd_sec_read_data;
            10'd47:  cap_root[31:24] <= sd_sec_read_data;
            10'd82:  cap_f0 <= sd_sec_read_data;
            10'd83:  cap_f1 <= sd_sec_read_data;
            10'd84:  cap_f2 <= sd_sec_read_data;
            10'd85:  cap_f3 <= sd_sec_read_data;
            10'd510: cap55 <= (sd_sec_read_data == 8'h55);
            10'd511: capaa <= (sd_sec_read_data == 8'hAA);
            default: ;
        endcase
        if ((fat_ph == PH_MBR) && (rd_cnt == 10'd454))
            cap_lba <= {24'd0, sd_sec_read_data};
        else if ((fat_ph == PH_MBR) && (rd_cnt == 10'd455))
            cap_lba[15:8] <= sd_sec_read_data;
        else if ((fat_ph == PH_MBR) && (rd_cnt == 10'd456))
            cap_lba[23:16] <= sd_sec_read_data;
        else if ((fat_ph == PH_MBR) && (rd_cnt == 10'd457))
            cap_lba[31:24] <= sd_sec_read_data;
    end else if ((state == ST_FAT) && (fat_ph == PH_DIR) && sd_sec_read_data_valid) begin
        ent_push <= 1'b0;
        case (rd_cnt[4:0])
            5'd0: begin
                ent_rail <= sd_sec_read_data=="R";ent_named <= (sd_sec_read_data == "0");ent_pip <= (sd_sec_read_data=="P");
                ent_skip <= (sd_sec_read_data == 8'h00) || (sd_sec_read_data == 8'hE5);
                if (sd_sec_read_data == 8'h00)
                    fat_dir_end <= 1'b1;
            end
            5'd1:begin ent_rail<=ent_rail && sd_sec_read_data=="A";ent_named<=ent_named && sd_sec_read_data=="0";ent_pip<=ent_pip && sd_sec_read_data=="I";end
            5'd2:begin ent_rail<=ent_rail && sd_sec_read_data=="I";ent_named<=ent_named && sd_sec_read_data=="0";ent_pip<=ent_pip && sd_sec_read_data=="P";end
            5'd3: begin
                ent_rail<=ent_rail && sd_sec_read_data=="L";
                ent_named <= ent_named && sd_sec_read_data >= "1" && sd_sec_read_data <= "8";
                ent_number <= sd_sec_read_data[2:0] - 3'd1;
                ent_pip<=ent_pip && (sd_sec_read_data=="0" || sd_sec_read_data=="1");ent_pip_slot<=sd_sec_read_data=="1";
            end
            5'd4,5'd5,5'd6,5'd7:begin
                ent_rail<=ent_rail && sd_sec_read_data==" ";ent_named<=ent_named && sd_sec_read_data==" ";ent_pip<=ent_pip && sd_sec_read_data==" ";end
            5'd8:  ent_e0 <= sd_sec_read_data;
            5'd9:  ent_e1 <= sd_sec_read_data;
            5'd10: ent_e2 <= sd_sec_read_data;
            5'd11: if ((sd_sec_read_data == 8'h0F) || sd_sec_read_data[4] || sd_sec_read_data[3])
                       ent_skip <= 1'b1;
            5'd20: ent_hi[7:0]  <= sd_sec_read_data;
            5'd21: ent_hi[15:8] <= sd_sec_read_data;
            5'd26: ent_lo[7:0]  <= sd_sec_read_data;
            5'd27: ent_lo[15:8] <= sd_sec_read_data;
            5'd28:ent_size[7:0]<=sd_sec_read_data;
            5'd29:ent_size[15:8]<=sd_sec_read_data;
            5'd30:ent_size[23:16]<=sd_sec_read_data;
            5'd31: begin
                if (!ent_skip && ({ent_hi, ent_lo} >= 32'd2)) begin
                    if(ent_pip && ent_e0=="P" && ent_e1=="V" && ent_e2=="4" && !pip_catalog_valid[ent_pip_slot])begin
                        pip_catalog_valid[ent_pip_slot]<=1;
                        if(ent_pip_slot)begin pip_sector1<=fat_sec_now;pip_size1<={sd_sec_read_data,ent_size[23:0]};end
                        else begin pip_sector0<=fat_sec_now;pip_size0<={sd_sec_read_data,ent_size[23:0]};end
                    end else if ((ent_e0 == "B") && (ent_e1 == "M") && (ent_e2 == "P")) begin
                        ent_push <= 1'b1;
                        ent_wav  <= 1'b0;
                        ent_sec  <= fat_sec_now;
                    end else if ((ent_e0 == "W") && (ent_e1 == "A") && (ent_e2 == "V")) begin
                        ent_push <= 1'b1;
                        ent_wav  <= 1'b1;
                        ent_sec  <= fat_sec_now;
                    end
                end
            end
            default: ;
        endcase
    end else begin
        ent_push <= 1'b0;
        if (state != ST_FAT)
            fat_dir_end <= 1'b0;
        if(!sd_init_done || scan_start)pip_catalog_valid<=0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        video_latched<=0;video_clip_latched<=0;video_requested<=0;video_size<=0;video_wait<=0;load_video_last<=0;
        pip_resume<=ST_IDLE;pip_resume_sector<=0;pip_cancel<=0;
        state             <= ST_IDLE;
        state_code        <= 4'd0;
        sd_sec_read       <= 1'b0;
        sd_sec_read_addr  <= 32'd0;
        write_req         <= 1'b0;
        scan_done         <= 1'b0;
        scan_found_valid  <= 1'b0;
        scan_found_sector <= 32'd0;
        scan_found_total  <= 4'd0;
        wav_found_valid   <= 1'b0;
        wav_found_sector  <= 32'd0;
        wav_found_total   <= 3'd0;
        scan_sector       <= 32'd0;
        load_sector_latched <= 32'd0;
        hunt_left         <= 13'd0;
        bmp_quota_met     <= 1'b0;
        wav_resume_sector <= 32'd0;
        wav_scan_done     <= 1'b0;
        wav_catalog_complete <= 1'b0;rail_catalog_preferred<=0;
        wav_load_done     <= 1'b0;
        wav_load_ok       <= 1'b0;
        load_fail         <= 1'b0;
        wav_sec_pend      <= 1'b0;
    end else if (!sd_init_done) begin
        state             <= ST_IDLE;
        state_code        <= 4'd0;
        sd_sec_read       <= 1'b0;
        sd_sec_read_addr  <= 32'd0;
        write_req         <= 1'b0;
        scan_done         <= 1'b0;
        scan_found_valid  <= 1'b0;
        scan_found_sector <= 32'd0;
        scan_found_total  <= 4'd0;
        wav_found_valid   <= 1'b0;
        wav_found_sector  <= 32'd0;
        wav_found_total   <= 3'd0;
        scan_sector       <= 32'd0;
        load_sector_latched <= 32'd0;
        hunt_left         <= 13'd0;
        bmp_quota_met     <= 1'b0;
        wav_resume_sector <= 32'd0;
        wav_scan_done     <= 1'b0;
        wav_catalog_complete <= 1'b0;rail_catalog_preferred<=0;
        wav_load_done     <= 1'b0;
        wav_load_ok       <= 1'b0;
        load_fail         <= 1'b0;
    end else if (load_abort &&
                 ((state == ST_LOAD_HDR) || (state == ST_LOAD_WAIT) || (state == ST_LOAD_DATA) || (state==ST_VIDEO_DRAIN) ||
                  (state == ST_WAV_LOAD) || (state == ST_WAV_SCAN))) begin
        // Soft cancel: do not clear scan_done / FAT finds (unlike full rst).
        state            <= ST_IDLE;
        state_code       <= 4'd1;
        sd_sec_read      <= 1'b0;
        write_req        <= 1'b0;
        load_fail        <= 1'b1;
        wav_load_done    <= 1'b0;
        wav_load_ok      <= 1'b0;
        wav_scan_done    <= 1'b0;
        scan_found_valid <= 1'b0;
        wav_found_valid  <= 1'b0;
    end else begin
        scan_found_valid <= 1'b0;
        wav_found_valid  <= 1'b0;
        wav_load_done    <= 1'b0;
        wav_load_ok      <= 1'b0;
        wav_scan_done    <= 1'b0;
        load_fail        <= 1'b0;

        case (state)
            ST_IDLE: begin
                state_code  <= 4'd1;
                sd_sec_read <= 1'b0;
                write_req   <= 1'b0;

                if (scan_start) begin
                    scan_done        <= 1'b0;
                    wav_catalog_complete <= 1'b0;rail_catalog_preferred<=0;
                    scan_found_total <= 4'd0;
                    wav_found_total  <= 3'd0;
                    bmp_quota_met    <= 1'b0;
                    hunt_left        <= 13'd0;
                    fat_ph           <= PH_MBR;
                    fat_bmp_n        <= 4'd0;
                    named_mask       <= 8'd0;
                    fat_wav_n        <= 3'd0;rail_catalog_preferred<=0;
                    fat_emit         <= 4'd0;
                    fat_emit_wav     <= 1'b0;
                    sd_sec_read_addr <= 32'd0;
                    state            <= ST_FAT;
                end else if (load_start) begin
                    video_latched<=load_video;video_clip_latched<=load_video_clip;
                    video_requested<=load_video_frame;video_size<=load_video_size;load_video_last<=0;
                    load_sector_latched <= load_sector;
                    sd_sec_read_addr    <= load_sector;
                    state               <= ST_LOAD_HDR;
                end else if (wav_scan_start) begin
                    hunt_left        <= WAV_HUNT_AFTER_BMP[12:0];
                    scan_sector      <= wav_resume_sector;
                    sd_sec_read_addr <= wav_resume_sector;
                    state            <= ST_WAV_SCAN;
                end else if (wav_load_start) begin
                    sd_sec_read_addr <= wav_load_sector;
                    state            <= ST_WAV_LOAD;
                end else if(pip_req && !load_abort)begin
                    pip_resume<=ST_IDLE;pip_cancel<=0;sd_sec_read_addr<=pip_sector;state<=ST_PIP;
                end
            end

            ST_SCAN: begin
                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;

                    if (header_match) begin
                        scan_found_valid  <= 1'b1;
                        scan_found_sector <= scan_sector;
                        scan_found_total  <= scan_found_total + 4'd1;
                        wav_resume_sector <= next_if_bmp;
                        sd_sec_read_addr  <= next_if_bmp;
                        scan_sector       <= next_if_bmp;
                        if (scan_found_total + 4'd1 >= scan_target_count) begin
                            scan_done <= 1'b1;
                            state     <= ST_IDLE;
                        end
                    end else if (is_riff && is_wave && (wav_found_total < 3'd4)) begin
                        wav_found_valid  <= 1'b1;
                        wav_found_sector <= scan_sector;
                        wav_found_total  <= wav_found_total + 3'd1;
                        sd_sec_read_addr <= next_if_wav;
                        scan_sector      <= next_if_wav;
                        if (bmp_quota_met && (hunt_left != 13'd0))
                            hunt_left <= hunt_left - 13'd1;
                    end else if (is_riff) begin
                        sd_sec_read_addr <= next_if_wav;
                        scan_sector      <= next_if_wav;
                        if (bmp_quota_met && (hunt_left != 13'd0))
                            hunt_left <= hunt_left - 13'd1;
                    end else if ((header_0 == "B") && (header_1 == "M")) begin
                        sd_sec_read_addr <= next_if_bmp;
                        scan_sector      <= next_if_bmp;
                        if (bmp_quota_met && (hunt_left != 13'd0))
                            hunt_left <= hunt_left - 13'd1;
                    end else begin
                        sd_sec_read_addr <= next_if_miss;
                        scan_sector      <= next_if_miss;
                        if (bmp_quota_met && (hunt_left != 13'd0))
                            hunt_left <= hunt_left - 13'd1;
                    end

                    if (scan_sector >= scan_max_sector) begin
                        scan_done <= 1'b1;
                        state     <= ST_IDLE;
                    end
                end
            end

            ST_LOAD_HDR: begin
                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;
                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if (video_latched?(video_header_ok && rd_cnt==512):header_match) begin
                        load_video_last<=video_latched && video_index+1'b1>=video_frames;
                        write_req        <= 1'b1;
                        sd_sec_read_addr <= load_sector_latched;
                        state            <= ST_LOAD_WAIT;
                    end else begin
                        load_fail <= 1'b1;
                        state     <= ST_IDLE;
                    end
                end
            end

            ST_LOAD_WAIT: begin
                state_code <= 4'd3;
                if (write_req_ack) begin
                    write_req <= 1'b0;
                    if(video_latched)begin sd_sec_read_addr<=load_sector_latched+32'd1+video_frame_sector;video_wait<=1;end
                    state     <= ST_LOAD_DATA;
                end
            end

            ST_LOAD_DATA: begin
                state_code  <= 4'd4;
                if(video_latched && video_wait)begin
                    sd_sec_read<=0;
                    if(video_compressed && (q_done || q_error))state<=ST_VIDEO_DRAIN;
                    else if(video_compressed?q_sector_ready:!normalize_pause)begin sd_sec_read<=1;video_wait<=0;end
                end else sd_sec_read <= 1'b1;
                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;video_wait<=1;
                    if (video_latched && (bmp_len_cnt>=video_frame_bytes || (video_compressed && (q_done || q_error))))state<=ST_VIDEO_DRAIN;
                    else if (!video_latched && bmp_len_cnt >= file_len) begin
                        state <= ST_IDLE;
                    end else begin
                        if(pip_req)begin
                            pip_resume<=ST_LOAD_DATA;pip_resume_sector<=sd_sec_read_addr+32'd1;
                            pip_cancel<=0;sd_sec_read_addr<=pip_sector;state<=ST_PIP;
                        end else sd_sec_read_addr <= sd_sec_read_addr + 32'd1;
                    end
                end
            end

            ST_VIDEO_DRAIN:begin
                sd_sec_read<=0;
                if(video_compressed && q_error)begin load_fail<=1;state<=ST_IDLE;end
                else if(normalize_done)state<=ST_IDLE;
            end
            ST_PIP:begin
                sd_sec_read<=1;
                if(load_abort && pip_resume==ST_LOAD_DATA)pip_cancel<=1;
                if(sd_sec_read_end)begin
                    sd_sec_read<=0;sd_sec_read_addr<=pip_resume_sector;
                    if(pip_resume==ST_LOAD_DATA && (pip_cancel || load_abort))begin state<=ST_IDLE;load_fail<=1;end
                    else state<=pip_resume;
                end
            end

            ST_WAV_LOAD: begin
                state_code <= 4'd2;
                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if (wav_samp_cnt >= WAV_KEEP_SAMPLES[19:0]) begin
                        wav_load_done <= 1'b1;
                        wav_load_ok   <= 1'b1;
                        wav_sec_pend  <= 1'b0;
                        state         <= ST_IDLE;
                    end else if (wav_byte_cnt < WAV_BYTE_CAP[31:0]) begin
                        if (wav_spi_hold)
                            wav_sec_pend <= 1'b1;
                        else
                            sd_sec_read_addr <= sd_sec_read_addr + 32'd1;
                    end
                end else if (wav_sec_pend && !wav_spi_hold) begin
                    wav_sec_pend     <= 1'b0;
                    sd_sec_read_addr <= sd_sec_read_addr + 32'd1;
                    sd_sec_read      <= 1'b1;
                end else if ((wav_samp_cnt >= WAV_KEEP_SAMPLES[19:0]) && !sd_sec_read) begin
                    wav_load_done <= 1'b1;
                    wav_load_ok   <= 1'b1;
                    state         <= ST_IDLE;
                end else if (wav_byte_cnt >= WAV_BYTE_CAP[31:0]) begin
                    sd_sec_read <= 1'b0;
                end else if (!wav_spi_hold) begin
                    sd_sec_read <= 1'b1;
                end
            end

            ST_WAV_SCAN: begin
                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;
                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if (is_riff && is_wave && (wav_found_total < 3'd4)) begin
                        wav_found_valid  <= 1'b1;
                        wav_found_sector <= scan_sector;
                        wav_found_total  <= wav_found_total + 3'd1;
                        sd_sec_read_addr <= next_if_wav;
                        scan_sector      <= next_if_wav;
                    end else if (is_riff) begin
                        sd_sec_read_addr <= next_if_wav;
                        scan_sector      <= next_if_wav;
                    end else begin
                        sd_sec_read_addr <= next_if_miss;
                        scan_sector      <= next_if_miss;
                    end
                    if (hunt_left != 13'd0)
                        hunt_left <= hunt_left - 13'd1;
                    if ((wav_found_total + ((is_riff && is_wave && wav_found_total < 3'd4) ? 3'd1 : 3'd0) >= 3'd4) ||
                        (hunt_left == 13'd1) ||
                        (scan_sector >= scan_max_sector)) begin
                        wav_scan_done <= 1'b1;
                        state         <= ST_IDLE;
                    end
                end
            end

            ST_FAT: begin
                state_code <= 4'd7;
                write_req  <= 1'b0;
                if ((fat_ph == PH_DIR) && ent_push) begin
                    if (ent_wav) begin
                        if(ent_rail)begin qw0<=ent_sec;fat_wav_n<=1;rail_catalog_preferred<=1;end
                        else if (!rail_catalog_preferred && fat_wav_n < 3'd4) begin
                            case (fat_wav_n)
                                3'd0: qw0 <= ent_sec;
                                3'd1: qw1 <= ent_sec;
                                3'd2: qw2 <= ent_sec;
                                default: qw3 <= ent_sec;
                            endcase
                            fat_wav_n <= fat_wav_n + 3'd1;
                        end
                    end else if (fat_bmp_n < 4'd8) begin
                        // Numbered files keep normal 0001..0004 and emergency
                        // 0005..0008 separate even when directory entries move.
                        case (fat_bmp_n)
                            4'd0: begin qb0<=ent_sec;kb0<=ent_named?{1'b0,ent_number}+4'd1:4'd0;end
                            4'd1: begin qb1<=ent_sec;kb1<=ent_named?{1'b0,ent_number}+4'd1:4'd0;end
                            4'd2: begin qb2<=ent_sec;kb2<=ent_named?{1'b0,ent_number}+4'd1:4'd0;end
                            4'd3: begin qb3<=ent_sec;kb3<=ent_named?{1'b0,ent_number}+4'd1:4'd0;end
                            4'd4: begin qb4<=ent_sec;kb4<=ent_named?{1'b0,ent_number}+4'd1:4'd0;end
                            4'd5: begin qb5<=ent_sec;kb5<=ent_named?{1'b0,ent_number}+4'd1:4'd0;end
                            4'd6: begin qb6<=ent_sec;kb6<=ent_named?{1'b0,ent_number}+4'd1:4'd0;end
                            default: begin qb7<=ent_sec;kb7<=ent_named?{1'b0,ent_number}+4'd1:4'd0;end
                        endcase
                        if(ent_named) named_mask[ent_number]<=1'b1;
                        fat_bmp_n <= fat_bmp_n + 4'd1;
                    end
                end
                if (fat_ph == PH_EMIT) begin
                    sd_sec_read <= 1'b0;
                    if (fat_bmp_n == 3'd0) begin
                        scan_sector      <= scan_start_sector;
                        sd_sec_read_addr <= scan_start_sector;
                        state            <= ST_SCAN;
                    end else if (!fat_emit_wav) begin
                        if (fat_emit < fat_bmp_n) begin
                            scan_found_valid  <= 1'b1;
                            scan_found_total  <= fat_emit + 4'd1;
                            scan_found_sector <= sector_by_bank(selected_emit_bank);
                            fat_emit <= fat_emit + 4'd1;
                        end else begin
                            fat_emit_wav <= 1'b1;
                            fat_emit     <= 4'd0;
                        end
                    end else if (fat_emit < fat_wav_n) begin
                        wav_found_valid <= 1'b1;
                        wav_found_total <= fat_emit[2:0] + 3'd1;
                        case (fat_emit)
                            3'd0: wav_found_sector <= qw0;
                            3'd1: wav_found_sector <= qw1;
                            3'd2: wav_found_sector <= qw2;
                            default: wav_found_sector <= qw3;
                        endcase
                        fat_emit <= fat_emit + 4'd1;
                    end else begin
                        scan_done         <= 1'b1;
                        wav_catalog_complete <= 1'b1;
                        wav_resume_sector <= qb0 + 32'd2048;
                        state             <= ST_IDLE;
                    end
                end else if (fat_ph == PH_PREP_BASE) begin
                    sd_sec_read <= 1'b0;
                    fat_data <= fat_part + cap_rsvd;
                    fat_root_offset <= cap_root_use - 32'd2;
                    fat_ph <= PH_PREP_FAT;
                end else if (fat_ph == PH_PREP_FAT) begin
                    sd_sec_read <= 1'b0;
                    fat_data <= fat_data + cap_fat_bytes;
                    fat_ph <= PH_PREP_DIR;
                end else if (fat_ph == PH_PREP_DIR) begin
                    sd_sec_read <= 1'b0;
                    fat_dir_sec <= fat_data + (fat_root_offset << fat_shift);
                    sd_sec_read_addr <= fat_data + (fat_root_offset << fat_shift);
                    fat_dir_left <= spc_count(fat_shift);
                    fat_ph <= PH_DIR;
                end else begin
                    sd_sec_read <= 1'b1;
                    if (sd_sec_read_end) begin
                        sd_sec_read <= 1'b0;
                        if (fat_ph == PH_MBR) begin
                            if (cap_boot_ok && ((cap0 == 8'hEB) || (cap0 == 8'hE9))) begin
                                fat_part         <= 32'd0;
                                fat_shift        <= spc_shift(cap_spc);
                                fat_ph           <= PH_PREP_BASE;
                            end else if (cap55 && capaa && (cap_lba != 32'd0)) begin
                                fat_part         <= cap_lba;
                                sd_sec_read_addr <= cap_lba;
                                fat_ph           <= PH_BOOT;
                            end else begin
                                scan_sector      <= scan_start_sector;
                                sd_sec_read_addr <= scan_start_sector;
                                state            <= ST_SCAN;
                            end
                        end else if (fat_ph == PH_BOOT) begin
                            if (cap_boot_ok && ((cap0 == 8'hEB) || (cap0 == 8'hE9))) begin
                                fat_shift        <= spc_shift(cap_spc);
                                fat_ph           <= PH_PREP_BASE;
                            end else begin
                                scan_sector      <= scan_start_sector;
                                sd_sec_read_addr <= scan_start_sector;
                                state            <= ST_SCAN;
                            end
                        end else if (!fat_dir_end && (fat_dir_left > 7'd1)) begin
                            fat_dir_left     <= fat_dir_left - 7'd1;
                            fat_dir_sec      <= fat_dir_sec + 32'd1;
                            sd_sec_read_addr <= fat_dir_sec + 32'd1;
                        end else begin
                            fat_ph   <= PH_EMIT;
                            fat_emit <= 4'd0;
                            fat_emit_wav <= 1'b0;
                        end
                    end
                end
            end

            default: begin
                state       <= ST_IDLE;
                state_code  <= 4'd1;
                sd_sec_read <= 1'b0;
                write_req   <= 1'b0;
            end
        endcase
    end
end

endmodule
