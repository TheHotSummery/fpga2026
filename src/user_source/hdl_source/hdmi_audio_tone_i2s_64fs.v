// Slice 15: I2S at 48 kHz, one PCM sample per LRCK from SDRAM FIFO (no on-chip loop BRAM).
module hdmi_audio_tone_i2s_64fs #(
    parameter [31:0] PHASE_INC = 32'd39370534,
    parameter signed [23:0] AMP = 24'sd2000000,
    parameter integer NOTE_HOLD_FRAMES = 24000,
    parameter integer LOOP_N = 16384
)(
    input  wire        I_mclk,
    input  wire        I_rst,
    input  wire [1:0]  I_img_idx,
    input  wire        I_wav_clk,
    input  wire        I_wav_we,
    input  wire [14:0] I_wav_addr,
    input  wire [15:0] I_wav_data,
    input  wire        I_wav_ready,
    input  wire [15:0] I_pcm,
    input  wire        I_pcm_empty,
    output reg         O_pcm_pop,
    output reg         O_i2s_BCLK,
    output reg         O_i2s_LRCK,
    output reg         O_i2s_DOUT
);

function [31:0] img_inc_lut;
    input [1:0] idx;
    begin
        case (idx)
            2'd0: img_inc_lut = 32'd23409862;
            2'd1: img_inc_lut = 32'd29494578;
            2'd2: img_inc_lut = 32'd35075155;
            default: img_inc_lut = 32'd46819716;
        endcase
    end
endfunction

reg        S_wav_ready_d0, S_wav_ready_d1;
reg        S_had_wav;
reg [19:0] S_pcm_idx;

reg  [5:0]  S_bit_cnt;
reg  [63:0] S_shift_reg;
reg  [31:0] S_phase_acc;
reg signed [23:0] S_pcm_hold;
reg  [1:0]  S_img_idx_d0;
reg  [1:0]  S_img_idx_d1;
reg  [1:0]  S_img_idx_hold;

wire [31:0] W_note_inc = img_inc_lut(S_img_idx_hold);
wire signed [23:0] W_pcm24  = {I_pcm, 8'h00};

always @(posedge I_mclk or posedge I_rst) begin
    if (I_rst) begin
        O_i2s_BCLK      <= 1'b0;
        O_i2s_LRCK      <= 1'b0;
        O_i2s_DOUT      <= 1'b0;
        O_pcm_pop       <= 1'b0;
        S_bit_cnt       <= 6'd0;
        S_shift_reg     <= 64'd0;
        S_phase_acc     <= 32'd0;
        S_pcm_hold      <= 24'sd0;
        S_img_idx_d0    <= 2'd0;
        S_img_idx_d1    <= 2'd0;
        S_img_idx_hold  <= 2'd0;
        S_wav_ready_d0  <= 1'b0;
        S_wav_ready_d1  <= 1'b0;
        S_had_wav       <= 1'b0;
        S_pcm_idx       <= 20'd0;
    end
    else begin
        S_img_idx_d0   <= I_img_idx;
        S_img_idx_d1   <= S_img_idx_d0;
        S_wav_ready_d0 <= I_wav_ready;
        S_wav_ready_d1 <= S_wav_ready_d0;
        if (S_wav_ready_d1)
            S_had_wav <= 1'b1;
        else
            S_pcm_idx <= 20'd0;
        O_pcm_pop <= 1'b0;

        O_i2s_BCLK <= ~O_i2s_BCLK;

        if (O_i2s_BCLK == 1'b1) begin
            O_i2s_DOUT  <= S_shift_reg[63];
            S_shift_reg <= {S_shift_reg[62:0], 1'b0};

            if (S_bit_cnt == 6'd63) begin
                S_bit_cnt <= 6'd0;

                if (O_i2s_LRCK == 1'b0) begin
                    O_i2s_LRCK  <= 1'b1;
                    S_shift_reg <= {S_pcm_hold[23:0], 40'd0};
                end
                else begin
                    O_i2s_LRCK <= 1'b0;

                    if (S_img_idx_d1 != S_img_idx_hold) begin
                        S_img_idx_hold <= S_img_idx_d1;
                        S_phase_acc    <= 32'd0;
                    end

                    if (S_wav_ready_d1) begin
                        if (!I_pcm_empty) begin
                            O_pcm_pop   <= 1'b1;
                            S_pcm_hold  <= W_pcm24;
                            S_shift_reg <= {W_pcm24[23:0], 40'd0};
                            if (S_pcm_idx == 20'd959999)
                                S_pcm_idx <= 20'd0;
                            else
                                S_pcm_idx <= S_pcm_idx + 20'd1;
                        end else begin
                            if (!S_had_wav) begin
                                S_pcm_hold  <= 24'sd0;
                                S_shift_reg <= 64'd0;
                            end else begin
                                S_pcm_hold<=24'd0;
                                S_shift_reg<=64'd0;
                            end
                        end
                    end
                    else if (S_had_wav) begin
                        S_pcm_hold  <= 24'sd0;
                        S_shift_reg <= 64'd0;
                    end
                    else begin
                        // Keep I2S clocks running, but send digital silence until PCM is ready.
                        S_pcm_hold <= 24'sd0;
                        S_shift_reg <= 64'd0;
                    end
                end
            end
            else begin
                S_bit_cnt <= S_bit_cnt + 6'd1;
            end
        end
    end
end

endmodule
