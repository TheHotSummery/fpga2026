module EG_LOGIC_BRAM #(
 parameter DATA_WIDTH_A=16,DATA_WIDTH_B=16,ADDR_WIDTH_A=8,ADDR_WIDTH_B=8,
 DATA_DEPTH_A=256,DATA_DEPTH_B=256,MODE="DP",REGMODE_A="NOREG",REGMODE_B="NOREG",
 WRITEMODE_A="NORMAL",WRITEMODE_B="NORMAL",RESETMODE="SYNC",IMPLEMENT="9K",INIT_FILE="NONE",FILL_ALL="NONE"
)(input [DATA_WIDTH_A-1:0] dia,input [DATA_WIDTH_B-1:0] dib,
 input [ADDR_WIDTH_A-1:0] addra,input [ADDR_WIDTH_B-1:0] addrb,
 input cea,ceb,ocea,oceb,clka,clkb,wea,web,bea,beb,rsta,rstb,
 output reg [DATA_WIDTH_A-1:0] doa,output reg [DATA_WIDTH_B-1:0] dob);
 reg [DATA_WIDTH_A-1:0] ram[0:DATA_DEPTH_A-1];
 always @(posedge clka)if(cea)begin if(wea)ram[addra]<=dia;doa<=ram[addra];end
 always @(posedge clkb)if(ceb)begin if(web)ram[addrb]<=dib;dob<=ram[addrb];end
endmodule
