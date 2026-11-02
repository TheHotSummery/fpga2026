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