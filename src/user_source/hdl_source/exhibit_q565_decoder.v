// Q565: lossless RGB565 bytecode. INDEX 00iiiiii; DIFF 01rrggbb;
// RUN 10nnnnnn (1..64); LUMA C0..FD + rrbbbb; FE + low/high RGB565.
// FF is deliberately invalid. Every frame starts with black and an empty cache.
// A bounded sector FIFO decouples an unstoppable SPI sector from per-pixel stalls.
module exhibit_q565_decoder(
 input wire clk,rst,input wire byte_valid,input wire [7:0] byte_data,
 input wire no_more,input wire [17:0] pixel_limit,input wire pixel_ready,
 output wire pixel_valid,output wire [15:0] pixel,
 output wire sector_ready,output reg done,error
);
localparam OP=0,RAW_LO=1,RAW_HI=2,LUMA=3,INDEX_WAIT=4,INDEX_GET=5,EMIT=6;
reg [2:0] state;
reg warming;reg [5:0] clear_index;
reg [8:0] wp,rp;reg [9:0] used;
reg pending;wire [7:0] fq;
wire need_byte=state==OP || state==RAW_LO || state==RAW_HI || state==LUMA;
wire take=!rst && !done && !error && !warming && need_byte && !pending && used!=0;
wire push=byte_valid && !done && !error && !rst;
wire store=push && (used<512 || take);
assign sector_ready=!warming && used==0 && !pending && !done && !error;
EG_LOGIC_BRAM #(.DATA_WIDTH_A(8),.DATA_WIDTH_B(8),.ADDR_WIDTH_A(9),.ADDR_WIDTH_B(9),
 .DATA_DEPTH_A(512),.DATA_DEPTH_B(512),.MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
 .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.RESETMODE("SYNC"),.IMPLEMENT("9K"),.INIT_FILE("NONE"),.FILL_ALL("NONE")) bytes_fifo(
 .dia(byte_data),.dib(8'd0),.addra(wp),.addrb(rp),.cea(1'b1),.ceb(1'b1),.ocea(1'b0),.oceb(1'b0),
 .clka(clk),.clkb(clk),.wea(store),.web(1'b0),.bea(1'b0),.beb(1'b0),.rsta(1'b0),.rstb(1'b0),.doa(),.dob(fq));
reg [15:0] previous,color;
reg [7:0] low_byte;
reg [5:0] luma_green,index_addr;
reg [6:0] remaining;
reg [17:0] produced;
// Clear the cache in 64 cycles before the first sector is requested.
// This removes a 64-bit register bank and its variable read/write muxes.
wire [15:0] cache_q;
wire [5:0] color_hash=color[5:0]^color[11:6]^{2'b0,color[15:12]};
assign pixel_valid=state==EMIT && !warming && pixel_ready && !done && !error && !rst;
assign pixel=color;
EG_LOGIC_BRAM #(.DATA_WIDTH_A(16),.DATA_WIDTH_B(16),.ADDR_WIDTH_A(6),.ADDR_WIDTH_B(6),
 .DATA_DEPTH_A(64),.DATA_DEPTH_B(64),.MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
 .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.RESETMODE("SYNC"),.IMPLEMENT("9K"),.INIT_FILE("NONE"),.FILL_ALL("NONE")) color_cache(
 .dia(warming?16'd0:color),.dib(16'd0),.addra(warming?clear_index:color_hash),.addrb(index_addr),.cea(1'b1),.ceb(1'b1),.ocea(1'b0),.oceb(1'b0),
 .clka(clk),.clkb(clk),.wea(warming || pixel_valid),.web(1'b0),.bea(1'b0),.beb(1'b0),.rsta(1'b0),.rstb(1'b0),.doa(),.dob(cache_q));
wire [4:0] diff_r=previous[15:11]+{3'b0,fq[5:4]}-5'd2;
wire [5:0] diff_g=previous[10:5]+{4'b0,fq[3:2]}-6'd2;
wire [4:0] diff_b=previous[4:0]+{3'b0,fq[1:0]}-5'd2;
// Modular channel arithmetic is exact for every legal encoder delta.
wire [4:0] luma_r=previous[15:11]+luma_green[4:0]+{1'b0,fq[7:4]}-5'd8;
wire [5:0] luma_g=previous[10:5]+luma_green;
wire [4:0] luma_b=previous[4:0]+luma_green[4:0]+{1'b0,fq[3:0]}-5'd8;
always @(posedge clk or posedge rst)begin
 if(rst)begin
  state<=OP;wp<=0;rp<=0;used<=0;pending<=0;previous<=0;color<=0;low_byte<=0;
  luma_green<=0;index_addr<=0;remaining<=0;produced<=0;warming<=1;clear_index<=0;done<=0;error<=0;
 end else begin
  if(warming)begin if(clear_index==63)warming<=0;else clear_index<=clear_index+1'b1;end
  pending<=take;
  if(store)wp<=wp+1'b1;
  if(take)rp<=rp+1'b1;
  case({store,take})2'b10:used<=used+1'b1;2'b01:used<=used-1'b1;default:;endcase
  if(push && !store)error<=1;
  if(no_more && !warming && need_byte && used==0 && !pending && !done)error<=1;
  if(!done && !error && !warming)begin
   if(pending)case(state)
    OP:begin
     remaining<=1;
     case(fq[7:6])
      0:begin index_addr<=fq[5:0];state<=INDEX_WAIT;end
      1:begin color<={diff_r,diff_g,diff_b};state<=EMIT;end
      2:begin color<=previous;remaining<={1'b0,fq[5:0]}+7'd1;state<=EMIT;end
      3:if(fq==8'hFE)state<=RAW_LO;
        else if(fq==8'hFF)error<=1;
        else begin luma_green<=fq[5:0]-6'd32;state<=LUMA;end
     endcase
    end
    RAW_LO:begin low_byte<=fq;state<=RAW_HI;end
    RAW_HI:begin color<={fq,low_byte};state<=EMIT;end
    LUMA:begin color<={luma_r,luma_g,luma_b};state<=EMIT;end
    default:error<=1;
   endcase
   if(state==INDEX_WAIT)state<=INDEX_GET;
   if(state==INDEX_GET)begin color<=cache_q;state<=EMIT;end
   if(pixel_valid)begin
    previous<=color;produced<=produced+1'b1;
    if(produced+1'b1==pixel_limit)begin
     if(remaining!=1)error<=1;else done<=1;
    end else if(produced>=pixel_limit)error<=1;
    else if(remaining>1)remaining<=remaining-1'b1;
    else state<=OP;
   end
  end
 end
end
endmodule
