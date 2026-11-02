// PVF4 RGB565 reconstruction before the shared photo/video framebuffer.
// Four bounded source rows; sector-level backpressure protects every live row.
// Three interpolation multipliers are time-shared between both axes.
module exhibit_media_bilinear_reference(
 input wire clk,rst,valid,out_ready,
 input wire [23:0] pixel,input wire [15:0] width,height,
 output reg out_valid,output reg [23:0] out_pixel,
 output wire pause_source,output reg done,overrun
);
reg [9:0] in_x;
reg [8:0] in_y,complete_rows;
reg [9:0] out_x,out_y;
reg [25:0] phase_x;
reg [24:0] phase_y;
reg [2:0] phase;
wire compressed_format=(width==512 && height==288)||(width==640 && height==360);
wire format_ok=compressed_format || (width==256 && height==144)||(width==320 && height==180)||(width==352 && height==198)||(width==384 && height==216)||(width==368 && height==207);
wire [9:0] sx=out_x==639?width[9:0]-1'b1:phase_x[25:16];
wire [8:0] sy=out_y==479?height[8:0]-1'b1:phase_y[24:16];
wire [9:0] sx1=sx+1'b1>=width? sx:sx+1'b1;
wire [8:0] sy1=sy+1'b1>=height?sy:sy+1'b1;
wire [7:0] fx=out_x==639?8'd0:phase_x[15:8];
wire [7:0] fy=out_y==479?8'd0:phase_y[15:8];
// Legacy formats reserve a whole sector; Q565 uses a byte FIFO and per-pixel
// backpressure. Four spare pixels cover the registered decoder/owner stages.
assign pause_source=done || !format_ok || in_y>sy+9'd3 ||
 (in_y==sy+9'd3 && in_x>width-(compressed_format?16'd4:16'd256));
wire rows_ready=complete_rows>sy1;
wire input_collision=valid && (in_y>=sy+9'd4 || in_y>=height);
wire [11:0] wa={in_y[1:0],in_x};
reg [11:0] ra;
wire [15:0] q;
always @*begin
 case(phase)
  0:ra={sy[1:0],sx};1:ra={sy[1:0],sx1};
  2:ra={sy1[1:0],sx};default:ra={sy1[1:0],sx1};
 endcase
end
EG_LOGIC_BRAM #(.DATA_WIDTH_A(16),.DATA_WIDTH_B(16),.ADDR_WIDTH_A(12),.ADDR_WIDTH_B(12),
 .DATA_DEPTH_A(4096),.DATA_DEPTH_B(4096),.MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
 .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.RESETMODE("SYNC"),.IMPLEMENT("9K"),.INIT_FILE("NONE"),.FILL_ALL("NONE")) rows(
 .dia({pixel[23:19],pixel[15:10],pixel[7:3]}),.dib(16'd0),.addra(wa),.addrb(ra),
 .cea(1'b1),.ceb(1'b1),.ocea(1'b0),.oceb(1'b0),.clka(clk),.clkb(clk),.wea(valid && !input_collision),.web(1'b0),
 .bea(1'b0),.beb(1'b0),.rsta(1'b0),.rstb(1'b0),.doa(),.dob(q));
wire [23:0] expanded={q[15:11],q[15:13],q[10:5],q[10:9],q[4:0],q[4:2]};
reg [23:0] p00,p01,p10,p11,h0,h1;
wire [23:0] a=phase==4?p00:phase==5?p10:h0;
wire [23:0] b=phase==4?p01:phase==5?p11:h1;
wire [7:0] fraction=phase==6?fy:fx;
wire signed [8:0] dr=$signed({1'b0,b[23:16]})-$signed({1'b0,a[23:16]});
wire signed [8:0] dg=$signed({1'b0,b[15:8]})-$signed({1'b0,a[15:8]});
wire signed [8:0] db=$signed({1'b0,b[7:0]})-$signed({1'b0,a[7:0]});
wire signed [17:0] pr=dr*$signed({1'b0,fraction});
wire signed [17:0] pg=dg*$signed({1'b0,fraction});
wire signed [17:0] pb=db*$signed({1'b0,fraction});
wire signed [18:0] vr=$signed({1'b0,a[23:16]})+(($signed(pr)+19'sd128)>>>8);
wire signed [18:0] vg=$signed({1'b0,a[15:8]})+(($signed(pg)+19'sd128)>>>8);
wire signed [18:0] vb=$signed({1'b0,a[7:0]})+(($signed(pb)+19'sd128)>>>8);
wire [23:0] result={vr[7:0],vg[7:0],vb[7:0]};
wire [25:0] x_step=width==640?26'd65536:width==512?26'd52408:width==368?25'd37640:width==384?25'd39281:width==352?25'd35999:width==320?25'd32716:25'd26152;
wire [24:0] y_step=height==360?25'd49118:height==288?25'd39267:height==207?25'd28185:height==216?25'd29416:height==198?25'd26953:height==180?25'd24490:25'd19565;
always @(posedge clk or posedge rst)begin
 if(rst)begin
  in_x<=0;in_y<=0;complete_rows<=0;out_x<=0;out_y<=0;phase_x<=0;phase_y<=0;phase<=0;
  out_valid<=0;out_pixel<=0;done<=0;overrun<=0;p00<=0;p01<=0;p10<=0;p11<=0;h0<=0;h1<=0;
 end else begin
  out_valid<=0;
  if(input_collision)overrun<=1;
  if(valid && format_ok && !input_collision)begin
   if(in_x+1'b1>=width)begin in_x<=0;in_y<=in_y+1'b1;complete_rows<=in_y+1'b1;end
   else in_x<=in_x+1'b1;
  end
  if(!done && format_ok)case(phase)
   0:if(rows_ready)phase<=1;
   1:begin p00<=expanded;phase<=2;end
   2:begin p01<=expanded;phase<=3;end
   3:begin p10<=expanded;phase<=4;end
   4:begin p11<=expanded;h0<=result;phase<=5;end
   5:begin h1<=result;phase<=6;end
   6:if(out_ready)begin
    out_pixel<=result;out_valid<=1;phase<=0;
    if(out_x==639)begin
     out_x<=0;phase_x<=0;
     if(out_y==479)done<=1;
     else begin out_y<=out_y+1'b1;phase_y<=phase_y+y_step;end
    end else begin out_x<=out_x+1'b1;phase_x<=phase_x+x_step;end
   end
   default:phase<=0;
  endcase
 end
end
endmodule
