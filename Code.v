////////////////////////////////Instruction Memory Design

module D_ff(input clk, input reset, input regWrite, input d, output reg q);
	always @ (negedge clk)
	begin
	if(reset==1)
		q=0;
	else
		if(regWrite == 1)
		begin
			q=d;
		end
	end
endmodule

module register16bit(input clk, input reset, input regWrite, input [15:0] in, output [15:0] out);
	
	D_ff dff0(clk, reset, regWrite, in[0], out[0]);
	D_ff dff1(clk, reset, regWrite, in[1], out[1]);
	D_ff dff2(clk, reset, regWrite, in[2], out[2]);
	D_ff dff3(clk, reset, regWrite, in[3], out[3]);
	D_ff dff4(clk, reset, regWrite, in[4], out[4]);
	D_ff dff5(clk, reset, regWrite, in[5], out[5]);
	D_ff dff6(clk, reset, regWrite, in[6], out[6]);
	D_ff dff7(clk, reset, regWrite, in[7], out[7]);
	D_ff dff8(clk, reset, regWrite, in[8], out[8]);
	D_ff dff9(clk, reset, regWrite, in[9], out[9]);
	D_ff dff10(clk, reset, regWrite, in[10], out[10]);
	D_ff dff11(clk, reset, regWrite, in[11], out[11]);
	D_ff dff12(clk, reset, regWrite, in[12], out[12]);
	D_ff dff13(clk, reset, regWrite, in[13], out[13]);
	D_ff dff14(clk, reset, regWrite, in[14], out[14]);
	D_ff dff15(clk, reset, regWrite, in[15], out[15]);
	
endmodule

module mux2to1_128bits(input [127:0] in0, input [127:0] in1, input sel, output reg [127:0] out);
	
	always@(in0, in1, sel)
		begin
			case(sel)
				1'd0: out=in0;
				1'd1: out=in1;
			endcase
		end
endmodule

module mux2to1_48bits(input [47:0] in0, input [47:0] in1, input sel, output reg [47:0] out);
	
	always@(in0, in1, sel)
		begin
			case(sel)
				1'd0: out=in0;
				1'd1: out=in1;
			endcase
		end
endmodule

module demux1to8_1bit(input in, input [2:0] sel, output reg out0, output reg out1, output reg out2, 
								output reg out3, output reg out4, output reg out5, output reg out6, output reg out7);
	
	always@(in, sel)
		begin	
			out0=1'b0;
			out1=1'b0;
			out2=1'b0;
			out3=1'b0;
			out4=1'b0;
			out5=1'b0;
			out6=1'b0;
			out7=1'b0;
			case(sel)
				3'd0: out0=in;
				3'd1: out1=in;
				3'd2: out2=in;
				3'd3: out3=in;
				3'd4: out4=in;
				3'd5: out5=in;
				3'd6: out6=in;
				3'd7: out7=in;
			endcase
		end
endmodule

// for valid-invalid one having index as select
module mux16to1_1bits(input in0,input in1,input in2,input in3,input in4,input in5,input in6, input in7,input in8,input in9,input in10,
						input in11, input in12, input in13, input in14, input in15,input [3:0] sel, output reg muxOut);
    always@(in0, in1, in2, in3, in4, in5, in6, in7, in8, in9, in10, in11, in12, in13, in14, in15, sel)
		begin
			case(sel)
				4'd0: muxOut=in0;
				4'd1: muxOut=in1;
				4'd2: muxOut=in2;
				4'd3: muxOut=in3;
				4'd4: muxOut=in4;
				4'd5: muxOut=in5;
				4'd6: muxOut=in6;
				4'd7: muxOut=in7;
				4'd8: muxOut=in8;
				4'd9: muxOut=in9;
				4'd10: muxOut=in10;
				4'd11: muxOut=in11;
				4'd12: muxOut=in12;
				4'd13: muxOut=in13;
				4'd14: muxOut=in14;
				4'd15: muxOut=in15;
			endcase
		end
endmodule

// for wayHit/Miss
module mux4to1_1bits(input in0,input in1,input in2,input in3,input [1:0] sel, output reg muxOut);
	
	always@(in0,in1,in2,in3,sel)
		begin	
			case(sel)
				2'd0: muxOut=in0;
				2'd1: muxOut=in1;
				2'd2: muxOut=in2;
				2'd3: muxOut=in3;
			endcase
		end
endmodule
//encoder for hitwaycache
module encoder4to2_1bit(input [3:0] in, output reg[1:0] out);

	always@(in)
		begin
			case(in)
				4'b0001: out=2'd0;
				4'b0010: out=2'd1;
				4'b0100: out=2'd2;
				4'b1000: out=2'd3;
			endcase
		end
endmodule

//for mruarray and fifo
module mux16to1_2bits(input [1:0] in0,input [1:0] in1,input [1:0] in2,input [1:0] in3,input [1:0] in4,input [1:0] in5,input [1:0] in6, input [1:0] in7,
						input [1:0] in8,input [1:0] in9,input [1:0] in10,input [1:0] in11, input [1:0] in12, input [1:0] in13, input [1:0] in14,
							input [1:0] in15,input [3:0] sel, output reg [1:0] muxOut);
    always@(in0, in1, in2, in3, in4, in5, in6, in7, in8, in9, in10, in11, in12, in13, in14, in15, sel)
		begin
			case(sel)
				4'd0: muxOut=in0;
				4'd1: muxOut=in1;
				4'd2: muxOut=in2;
				4'd3: muxOut=in3;
				4'd4: muxOut=in4;
				4'd5: muxOut=in5;
				4'd6: muxOut=in6;
				4'd7: muxOut=in7;
				4'd8: muxOut=in8;
				4'd9: muxOut=in9;
				4'd10: muxOut=in10;
				4'd11: muxOut=in11;
				4'd12: muxOut=in12;
				4'd13: muxOut=in13;
				4'd14: muxOut=in14;
				4'd15: muxOut=in15;
			endcase
		end
endmodule

module mux16to1_25bits(input [24:0]in0,input [24:0] in1,input [24:0] in2,input [24:0] in3,
								input [24:0] in4,input [24:0] in5,input [24:0] in6, input [24:0] in7,
									input [24:0] in8,input [24:0] in9,input [24:0] in10,input [24:0] in11, 
										input [24:0] in12, input [24:0] in13, input [24:0] in14, input [24:0] in15,
											input [3:0] sel, output reg [24:0] muxOut);
    always@(in0, in1, in2, in3, in4, in5, in6, in7, in8, in9, in10, in11, in12, in13, in14, in15, sel)
		begin
			case(sel)
				4'd0: muxOut=in0;
				4'd1: muxOut=in1;
				4'd2: muxOut=in2;
				4'd3: muxOut=in3;
				4'd4: muxOut=in4;
				4'd5: muxOut=in5;
				4'd6: muxOut=in6;
				4'd7: muxOut=in7;
				4'd8: muxOut=in8;
				4'd9: muxOut=in9;
				4'd10: muxOut=in10;
				4'd11: muxOut=in11;
				4'd12: muxOut=in12;
				4'd13: muxOut=in13;
				4'd14: muxOut=in14;
				4'd15: muxOut=in15;
			endcase
		end
endmodule

//for data array
module mux16to1_128bits(input [127:0]in0,input [127:0] in1,input [127:0] in2,input [127:0] in3,input [127:0] in4,input [127:0] in5,input [127:0] in6, 
							input [127:0] in7,input [127:0] in8,input [127:0] in9,input [127:0] in10,input [127:0] in11, input [127:0] in12, input [127:0] in13,
								input [127:0] in14, input [127:0] in15,input [3:0] sel, output reg [127:0] muxOut);
	always@(in0, in1, in2, in3, in4, in5, in6, in7, in8, in9, in10, in11, in12, in13, in14, in15, sel)
		begin
			case(sel)
				4'd0: muxOut=in0;
				4'd1: muxOut=in1;
				4'd2: muxOut=in2;
				4'd3: muxOut=in3;
				4'd4: muxOut=in4;
				4'd5: muxOut=in5;
				4'd6: muxOut=in6;
				4'd7: muxOut=in7;
				4'd8: muxOut=in8;
				4'd9: muxOut=in9;
				4'd10: muxOut=in10;
				4'd11: muxOut=in11;
				4'd12: muxOut=in12;
				4'd13: muxOut=in13;
				4'd14: muxOut=in14;
				4'd15: muxOut=in15;
			endcase
		end
endmodule

//this mux output splitted and given to cpu as 2bytes at a time
module mux4to1_128bits(input [127:0]in0,input [127:0] in1,input [127:0] in2,input [127:0] in3,
								input [1:0] sel, output reg [127:0] muxOut);
	always@(in0, in1, in2, in3, sel)
		begin
			case(sel)
				2'd0: muxOut=in0;
				2'd1: muxOut=in1;
				2'd2: muxOut=in2;
				2'd3: muxOut=in3;				
			endcase
		end
endmodule

module mux2to1_2bits(input [1:0] in0, input [1:0] in1, input sel, output reg [1:0] out);
	
	always@(in0, in1, sel)
		begin
			case(sel)
				1'd0: out=in0;
				1'd1: out=in1;
			endcase
		end
endmodule

module mux8to1_48bits(input [47:0]in0,input [47:0] in1,input [47:0] in2,input [47:0] in3,
								input [47:0] in4,input [47:0] in5,input [47:0] in6, input [47:0] in7,
									input [2:0] sel, output reg [47:0] muxOut);
    always@(in0, in1, in2, in3, in4, in5, in6, in7, sel)
		begin
			case(sel)
				3'd0: muxOut=in0;
				3'd1: muxOut=in1;
				3'd2: muxOut=in2;
				3'd3: muxOut=in3;
				3'd4: muxOut=in4;
				3'd5: muxOut=in5;
				3'd6: muxOut=in6;
				3'd7: muxOut=in7;
			endcase
		end
endmodule

module demux1to16_1bit(input in, input [3:0] sel, output reg out0, output reg out1, output reg out2, 
								output reg out3, output reg out4, output reg out5, output reg out6, output reg out7, 
									output reg out8, output reg out9, output reg out10, output reg out11, output reg out12, 
										output reg out13, output reg out14, output reg out15);
	
	always@(in, sel)
		begin	
			out0=1'b0;
			out1=1'b0;
			out2=1'b0;
			out3=1'b0;
			out4=1'b0;
			out5=1'b0;
			out6=1'b0;
			out7=1'b0;
			out8=1'b0;
			out9=1'b0;
			out10=1'b0;
			out11=1'b0;
			out12=1'b0;
			out13=1'b0;
			out14=1'b0;
			out15=1'b0;
			case(sel)
				4'd0: out0=in;
				4'd1: out1=in;
				4'd2: out2=in;
				4'd3: out3=in;
				4'd4: out4=in;
				4'd5: out5=in;
				4'd6: out6=in;
				4'd7: out7=in;
				4'd8: out8=in;
				4'd9: out9=in;
				4'd10: out10=in;
				4'd11: out11=in;
				4'd12: out12=in;
				4'd13: out13=in;
				4'd14: out14=in;
				4'd15: out15=in;
			endcase
		end
endmodule

//writeway place
module priorityEncoder4to2_1bit(input in0, input in1, input in2, input in3, output reg [1:0] out);
	always@(in0, in1, in2, in3)
		begin
		casex({in3, in2, in1, in0})
			4'bxxx0:	out = 2'd0;
			4'bxx0x:	out = 2'd1;
			4'bx0xx:	out = 2'd2;
			4'b0xxx:	out = 2'd3;
		endcase 		
		end
endmodule

//
module decoder2to4_1bit(input [1:0] in , output reg [3:0] out);
	always@(in)
		begin
		case(in)
			2'd0:	out = 4'b1110;
			2'd1:	out = 4'b1101;
			2'd2:	out = 4'b1011;
			2'd3:	out = 4'b0111;
		endcase	
		end
endmodule

module tagComparator(input [24:0] tag0, input [24:0] tag1, output reg out);
	
	always@(tag0, tag1)
	begin 
		if(tag0==tag1)
			out = 1'b1;
		else
			out = 1'b0;
	end	
	
endmodule

module tagComparator29bit(input [28:0] tag0, input [28:0] tag1, output reg out);
	
	always@(tag0, tag1)
	begin 
		if(tag0==tag1)
			out = 1'b1;
		else
			out = 1'b0;
	end	
	
endmodule

module cacheDataBlock(input clk, input reset, input blockWrite, input dWordWr, input [2:0] dWordOffset,
								input [31:0] dWord, input [127:0] inputDataBlock, output [127:0] out);
	
	wire [7:0] demuxOut;
	wire [127:0] in, in0;
	
	demux1to8_1bit demux1to8(1'b1, dWordOffset, demuxOut[0], demuxOut[1], demuxOut[2], demuxOut[3], demuxOut[4],
							demuxOut[5], demuxOut[6], demuxOut[7]);
	mux2to1_128bits mux0({4{dWord}}, {dWord[31:16], {3{dWord}}, dWord[15:0]}, dWordOffset[0], in0);
	mux2to1_128bits mux1(in0, inputDataBlock, blockWrite, in);
	
	//16 Byte Cache Data Block
	register16bit reg0(clk, reset,(dWordWr & demuxOut[0]) | blockWrite,  in[15:0], out[15:0]);
	register16bit reg1(clk, reset, (dWordWr & (demuxOut[1] | demuxOut[0])) | blockWrite,  in[31:16], out[31:16]);
	register16bit reg2(clk, reset, (dWordWr & (demuxOut[2] | demuxOut[1])) | blockWrite,  in[47:32], out[47:32]);
	register16bit reg3(clk, reset, (dWordWr & (demuxOut[3] | demuxOut[2])) | blockWrite,  in[63:48], out[63:48]);
	register16bit reg4(clk, reset, (dWordWr & (demuxOut[4] | demuxOut[3])) | blockWrite,  in[79:64], out[79:64]);
	register16bit reg5(clk, reset, (dWordWr & (demuxOut[5] | demuxOut[4])) | blockWrite,  in[95:80], out[95:80]);
	register16bit reg6(clk, reset, (dWordWr & (demuxOut[6] | demuxOut[5])) | blockWrite,  in[111:96], out[111:96]);
	register16bit reg7(clk, reset, (dWordWr & (demuxOut[7] | demuxOut[6])) | blockWrite,  in[127:112], out[127:112]);
	
endmodule

module cacheDataBlockArray(input clk, input reset, input [15:0] blockWrite, input [15:0] dWordWr, 
										input [2:0] dWordOffset, input [31:0] dWord, input [2047:0] in, output [2047:0] out);
	
	cacheDataBlock db0(clk, reset, blockWrite[0], dWordWr[0], dWordOffset, dWord, in[127:0], out[127:0]);
	cacheDataBlock db1(clk, reset, blockWrite[1], dWordWr[1], dWordOffset, dWord, in[255:128], out[255:128]);
	cacheDataBlock db2(clk, reset, blockWrite[2], dWordWr[2], dWordOffset, dWord, in[383:256], out[383:256]);
	cacheDataBlock db3(clk, reset, blockWrite[3], dWordWr[3], dWordOffset, dWord, in[511:384], out[511:384]);
	cacheDataBlock db4(clk, reset, blockWrite[4], dWordWr[4], dWordOffset, dWord, in[639:512], out[639:512]);
	cacheDataBlock db5(clk, reset, blockWrite[5], dWordWr[5], dWordOffset, dWord, in[767:640], out[767:640]);
	cacheDataBlock db6(clk, reset, blockWrite[6], dWordWr[6], dWordOffset, dWord, in[895:768], out[895:768]);
	cacheDataBlock db7(clk, reset, blockWrite[7], dWordWr[7], dWordOffset, dWord, in[1023:896], out[1023:896]);
	cacheDataBlock db8(clk, reset, blockWrite[8], dWordWr[8], dWordOffset, dWord, in[1151:1024], out[1151:1024]);
	cacheDataBlock db9(clk, reset, blockWrite[9], dWordWr[9], dWordOffset, dWord, in[1279:1152], out[1279:1152]);
	cacheDataBlock db10(clk, reset, blockWrite[10], dWordWr[10], dWordOffset, dWord, in[1407:1280], out[1407:1280]);
	cacheDataBlock db11(clk, reset, blockWrite[11], dWordWr[11], dWordOffset, dWord, in[1535:1408], out[1535:1408]);
	cacheDataBlock db12(clk, reset, blockWrite[12], dWordWr[12], dWordOffset, dWord, in[1663:1536], out[1663:1536]);
	cacheDataBlock db13(clk, reset, blockWrite[13], dWordWr[13], dWordOffset, dWord, in[1791:1664], out[1791:1664]);
	cacheDataBlock db14(clk, reset, blockWrite[14], dWordWr[14], dWordOffset, dWord, in[1919:1792], out[1919:1792]);
	cacheDataBlock db15(clk, reset, blockWrite[15], dWordWr[15], dWordOffset, dWord, in[2047:1920], out[2047:1920]);
	
endmodule

module viArray(input clk, input reset, input [15:0] we, input [15:0] d, output [15:0] valid);
	
	D_ff dff0(clk, reset, we[0], d[0], valid[0]);
	D_ff dff1(clk, reset, we[1], d[1], valid[1]);
	D_ff dff2(clk, reset, we[2], d[2], valid[2]);
	D_ff dff3(clk, reset, we[3], d[3], valid[3]);
	D_ff dff4(clk, reset, we[4], d[4], valid[4]);
	D_ff dff5(clk, reset, we[5], d[5], valid[5]);
	D_ff dff6(clk, reset, we[6], d[6], valid[6]);
	D_ff dff7(clk, reset, we[7], d[7], valid[7]);
	D_ff dff8(clk, reset, we[8], d[8], valid[8]);
	D_ff dff9(clk, reset, we[9], d[9], valid[9]);
	D_ff dff10(clk, reset, we[10], d[10], valid[10]);
	D_ff dff11(clk, reset, we[11], d[11], valid[11]);
	D_ff dff12(clk, reset, we[12], d[12], valid[12]);
	D_ff dff13(clk, reset, we[13], d[13], valid[13]);
	D_ff dff14(clk, reset, we[14], d[14], valid[14]);
	D_ff dff15(clk, reset, we[15], d[15], valid[15]);
		
endmodule

module tag25bitRegister(input clk, input reset, input we, input [24:0] tagIn, output [24:0] tagOut);
	
	D_ff dff0(clk, reset, we, tagIn[0], tagOut[0]);
	D_ff dff1(clk, reset, we, tagIn[1], tagOut[1]);
	D_ff dff2(clk, reset, we, tagIn[2], tagOut[2]);
	D_ff dff3(clk, reset, we, tagIn[3], tagOut[3]);
	D_ff dff4(clk, reset, we, tagIn[4], tagOut[4]);
	D_ff dff5(clk, reset, we, tagIn[5], tagOut[5]);
	D_ff dff6(clk, reset, we, tagIn[6], tagOut[6]);
	D_ff dff7(clk, reset, we, tagIn[7], tagOut[7]);
	D_ff dff8(clk, reset, we, tagIn[8], tagOut[8]);
	D_ff dff9(clk, reset, we, tagIn[9], tagOut[9]);
	D_ff dff10(clk, reset, we, tagIn[10], tagOut[10]);
	D_ff dff11(clk, reset, we, tagIn[11], tagOut[11]);
	D_ff dff12(clk, reset, we, tagIn[12], tagOut[12]);
	D_ff dff13(clk, reset, we, tagIn[13], tagOut[13]);
	D_ff dff14(clk, reset, we, tagIn[14], tagOut[14]);
	D_ff dff15(clk, reset, we, tagIn[15], tagOut[15]);
	D_ff dff16(clk, reset, we, tagIn[16], tagOut[16]);
	D_ff dff17(clk, reset, we, tagIn[17], tagOut[17]);
	D_ff dff18(clk, reset, we, tagIn[18], tagOut[18]);
	D_ff dff19(clk, reset, we, tagIn[19], tagOut[19]);
	D_ff dff20(clk, reset, we, tagIn[20], tagOut[20]);
	D_ff dff21(clk, reset, we, tagIn[21], tagOut[21]);
	D_ff dff22(clk, reset, we, tagIn[22], tagOut[22]);
	D_ff dff23(clk, reset, we, tagIn[23], tagOut[23]);
	D_ff dff24(clk, reset, we, tagIn[24], tagOut[24]);
	
		
endmodule

module tag29bitRegister(input clk, input reset, input we, input [28:0] tagIn, output [28:0] tagOut);

	
	D_ff dff0(clk, reset, we, tagIn[0], tagOut[0]);
	D_ff dff1(clk, reset, we, tagIn[1], tagOut[1]);
	D_ff dff2(clk, reset, we, tagIn[2], tagOut[2]);
	D_ff dff3(clk, reset, we, tagIn[3], tagOut[3]);
	D_ff dff4(clk, reset, we, tagIn[4], tagOut[4]);
	D_ff dff5(clk, reset, we, tagIn[5], tagOut[5]);
	D_ff dff6(clk, reset, we, tagIn[6], tagOut[6]);
	D_ff dff7(clk, reset, we, tagIn[7], tagOut[7]);
	D_ff dff8(clk, reset, we, tagIn[8], tagOut[8]);
	D_ff dff9(clk, reset, we, tagIn[9], tagOut[9]);
	D_ff dff10(clk, reset, we, tagIn[10], tagOut[10]);
	D_ff dff11(clk, reset, we, tagIn[11], tagOut[11]);
	D_ff dff12(clk, reset, we, tagIn[12], tagOut[12]);
	D_ff dff13(clk, reset, we, tagIn[13], tagOut[13]);
	D_ff dff14(clk, reset, we, tagIn[14], tagOut[14]);
	D_ff dff15(clk, reset, we, tagIn[15], tagOut[15]);
	D_ff dff16(clk, reset, we, tagIn[16], tagOut[16]);
	D_ff dff17(clk, reset, we, tagIn[17], tagOut[17]);
	D_ff dff18(clk, reset, we, tagIn[18], tagOut[18]);
	D_ff dff19(clk, reset, we, tagIn[19], tagOut[19]);
	D_ff dff20(clk, reset, we, tagIn[20], tagOut[20]);
	D_ff dff21(clk, reset, we, tagIn[21], tagOut[21]);
	D_ff dff22(clk, reset, we, tagIn[22], tagOut[22]);
	D_ff dff23(clk, reset, we, tagIn[23], tagOut[23]);
	D_ff dff24(clk, reset, we, tagIn[24], tagOut[24]);
	D_ff dff25(clk, reset, we, tagIn[25], tagOut[25]);
	D_ff dff26(clk, reset, we, tagIn[26], tagOut[26]);
	D_ff dff27(clk, reset, we, tagIn[27], tagOut[27]);
	D_ff dff28(clk, reset, we, tagIn[28], tagOut[28]);
	
		
endmodule

module tagArray(input clk, input reset, input [15:0] we, input [399:0] tagIn, output [399:0] tagOut);
			
	tag25bitRegister tag0(clk, reset, we[0], tagIn[24:0], tagOut[24:0]);
	tag25bitRegister tag1(clk, reset, we[1], tagIn[49:25], tagOut[49:25]);
	tag25bitRegister tag2(clk, reset, we[2], tagIn[74:50], tagOut[74:50]);
	tag25bitRegister tag3(clk, reset, we[3], tagIn[99:75], tagOut[99:75]);
	tag25bitRegister tag4(clk, reset, we[4], tagIn[124:100], tagOut[124:100]);
	tag25bitRegister tag5(clk, reset, we[5], tagIn[149:125], tagOut[149:125]);
	tag25bitRegister tag6(clk, reset, we[6], tagIn[174:150], tagOut[174:150]);
	tag25bitRegister tag7(clk, reset, we[7], tagIn[199:175], tagOut[199:175]);
	tag25bitRegister tag8(clk, reset, we[8], tagIn[224:200], tagOut[224:200]);
	tag25bitRegister tag9(clk, reset, we[9], tagIn[249:225], tagOut[249:225]);
	tag25bitRegister tag10(clk, reset, we[10], tagIn[274:250], tagOut[274:250]);
	tag25bitRegister tag11(clk, reset, we[11], tagIn[299:275], tagOut[299:275]);
	tag25bitRegister tag12(clk, reset, we[12], tagIn[324:300], tagOut[324:300]);
	tag25bitRegister tag13(clk, reset, we[13], tagIn[349:325], tagOut[349:325]);
	tag25bitRegister tag14(clk, reset, we[14], tagIn[374:350], tagOut[374:350]);
	tag25bitRegister tag15(clk, reset, we[15], tagIn[399:375], tagOut[399:375]);


endmodule

module mru2bitRegister(input clk, input reset, input we, input [1:0] in, input [1:0] out);
	
	D_ff dff0(clk, reset, we, in[0], out[0]);
	D_ff dff1(clk, reset, we, in[1], out[1]);
	
endmodule

module mruArray(input clk, input reset, input [15:0] we, input [1:0] in0, input [1:0] in1,
						 input [1:0] in2, input [1:0] in3, input [1:0] in4, input [1:0] in5, input [1:0] in6,
						  input [1:0] in7, input [1:0] in8, input [1:0] in9, input [1:0] in10, input [1:0] in11,
						   input [1:0] in12, input [1:0] in13, input [1:0] in14, input [1:0] in15, output [1:0] out0,
							 output [1:0] out1, output [1:0] out2, output [1:0] out3, output [1:0] out4, output [1:0] out5,
							  output [1:0] out6, output [1:0] out7, output [1:0] out8, output [1:0] out9, output [1:0] out10,
							   output [1:0] out11, output [1:0] out12, output [1:0] out13, output [1:0] out14, output [1:0] out15);

	mru2bitRegister mru0(clk, reset, we[0], in0, out0);
	mru2bitRegister mru1(clk, reset, we[1], in1, out1);
	mru2bitRegister mru2(clk, reset, we[2], in2, out2);
	mru2bitRegister mru3(clk, reset, we[3], in3, out3);
	mru2bitRegister mru4(clk, reset, we[4], in4, out4);
	mru2bitRegister mru5(clk, reset, we[5], in5, out5);
	mru2bitRegister mru6(clk, reset, we[6], in6, out6);
	mru2bitRegister mru7(clk, reset, we[7], in7, out7);
	mru2bitRegister mru8(clk, reset, we[8], in8, out8);
	mru2bitRegister mru9(clk, reset, we[9], in9, out9);
	mru2bitRegister mru10(clk, reset, we[10], in10, out10);
	mru2bitRegister mru11(clk, reset, we[11], in11, out11);
	mru2bitRegister mru12(clk, reset, we[12], in12, out12);
	mru2bitRegister mru13(clk, reset, we[13], in13, out13);
	mru2bitRegister mru14(clk, reset, we[14], in14, out14);
	mru2bitRegister mru15(clk, reset, we[15], in15, out15);
								
endmodule

module fifo2bitCounter(input clk, input reset, input set, input incrementEnable, output reg [1:0] out);
		
	reg [1:0] state;
	
	always @(negedge clk)
	begin
		if(set)
			state = 2'b00;
		else if(reset)
			state = 2'b11;
		else if(incrementEnable)
		begin
			case(state)
				2'd0: state = 2'd1;
				2'd1: state = 2'd2;
				2'd2: state = 2'd3;
				2'd3: state = 2'd3;
			endcase
		end
		out = state;
	end
	
endmodule

module fifoArray(input clk, input reset, input [15:0] set, input [15:0] incrementEnable, output [31:0] out);

	fifo2bitCounter fifoBits0(clk, reset, set[0], incrementEnable[0], out[1:0]);
	fifo2bitCounter fifoBits1(clk, reset, set[1], incrementEnable[1], out[3:2]);
	fifo2bitCounter fifoBits2(clk, reset, set[2], incrementEnable[2], out[5:4]);
	fifo2bitCounter fifoBits3(clk, reset, set[3], incrementEnable[3], out[7:6]);
	fifo2bitCounter fifoBits4(clk, reset, set[4], incrementEnable[4], out[9:8]);
	fifo2bitCounter fifoBits5(clk, reset, set[5], incrementEnable[5], out[11:10]);
	fifo2bitCounter fifoBits6(clk, reset, set[6], incrementEnable[6], out[13:12]);
	fifo2bitCounter fifoBits7(clk, reset, set[7], incrementEnable[7], out[15:14]);
	fifo2bitCounter fifoBits8(clk, reset, set[8], incrementEnable[8], out[17:16]);
	fifo2bitCounter fifoBits9(clk, reset, set[9], incrementEnable[9], out[19:18]);
	fifo2bitCounter fifoBits10(clk, reset, set[10], incrementEnable[10], out[21:20]);
	fifo2bitCounter fifoBits11(clk, reset, set[11], incrementEnable[11], out[23:22]);
	fifo2bitCounter fifoBits12(clk, reset, set[12], incrementEnable[12], out[25:24]);
	fifo2bitCounter fifoBits13(clk, reset, set[13], incrementEnable[13], out[27:26]);
	fifo2bitCounter fifoBits14(clk, reset, set[14], incrementEnable[14], out[29:28]);
	fifo2bitCounter fifoBits15(clk, reset, set[15], incrementEnable[15], out[31:30]);
	
endmodule

module writeBuffer(input clk, input reset, input we, input [28:0] inputTag, input [127:0] dataBlock);
	wire pEncoder0,pEncoder1,pEncoder2,pEncoder3,vi0,vi1,vi2,vi3,tagPresent;
	wire [1:0] fifoOut0, fifoOut1, fifoOut2, fifoOut3, muxIn0, muxIn1, outP;
	wire [3:0] set, incrementEnable;
	wire [28:0] tagOut0, tagOut1, tagOut2, tagOut3;
	wire [127:0] dbOut0, dbOut1, dbOut2, dbOut3;
	
	assign tagPresent=(pEncoder0 & vi0) | (pEncoder1 & vi1) | (pEncoder2 & vi2) | (pEncoder3 & vi3);
	
	fifo2bitCounter fifo0(clk, reset, we & ~incrementEnable[0] & ~tagPresent, we & incrementEnable[0] & ~tagPresent, fifoOut0);
	fifo2bitCounter fifo1(clk, reset, we & ~incrementEnable[1] & ~tagPresent, we & incrementEnable[1] & ~tagPresent, fifoOut1);
	fifo2bitCounter fifo2(clk, reset, we & ~incrementEnable[2] & ~tagPresent, we & incrementEnable[2] & ~tagPresent, fifoOut2);
	fifo2bitCounter fifo3(clk, reset, we & ~incrementEnable[3] & ~tagPresent, we & incrementEnable[3] & ~tagPresent, fifoOut3);
	
	tag29bitRegister tag0(clk, reset, we & ~incrementEnable[0], inputTag, tagOut0);
	tag29bitRegister tag1(clk, reset, we & ~incrementEnable[1], inputTag, tagOut1);
	tag29bitRegister tag2(clk, reset, we & ~incrementEnable[2], inputTag, tagOut2);
	tag29bitRegister tag3(clk, reset, we & ~incrementEnable[3], inputTag, tagOut3);
	
	D_ff vibit0(clk, reset, we & ~incrementEnable[0], 1'b1, vi0);
	D_ff vibit1(clk, reset, we & ~incrementEnable[1], 1'b1, vi1);
	D_ff vibit2(clk, reset, we & ~incrementEnable[2], 1'b1, vi2);
	D_ff vibit3(clk, reset, we & ~incrementEnable[3], 1'b1, vi3);
	
	cacheDataBlock db0(clk, reset,  we & ~incrementEnable[0], 1'd0, 3'd0, 32'd0, dataBlock, dbOut0);
	cacheDataBlock db1(clk, reset,  we & ~incrementEnable[1], 1'd0, 3'd0, 32'd0, dataBlock, dbOut1);
	cacheDataBlock db2(clk, reset,  we & ~incrementEnable[2], 1'd0, 3'd0, 32'd0, dataBlock, dbOut2);
	cacheDataBlock db3(clk, reset,  we & ~incrementEnable[3], 1'd0, 3'd0, 32'd0, dataBlock, dbOut3);
	
	priorityEncoder4to2_1bit fifoEncoder(~(fifoOut0[1] & fifoOut0[0]), ~(fifoOut1[1] & fifoOut1[0]),
														~(fifoOut2[1] & fifoOut2[0]), ~(fifoOut3[1] & fifoOut3[0]), muxIn0);
														
	tagComparator29bit tc0(inputTag, tagOut0, pEncoder0);
	tagComparator29bit tc1(inputTag, tagOut1, pEncoder1);
	tagComparator29bit tc2(inputTag, tagOut2, pEncoder2);
	tagComparator29bit tc3(inputTag, tagOut3, pEncoder3);
	
	priorityEncoder4to2_1bit tagEncdr(~(pEncoder0 & vi0), ~(pEncoder1 & vi1), ~(pEncoder2 & vi2), ~(pEncoder3 & vi3), muxIn1);
	
	mux2to1_2bits writeSelMux(muxIn0, muxIn1, tagPresent, outP);
	
	decoder2to4_1bit decoder(outP,incrementEnable);
	
endmodule

module counter(input clk, input reset, output reg [2:0] count);
	
	reg [2:0] state,nxtState;
	
	always@(negedge clk)
	begin
		if(reset)
			nxtState = 3'd0;
		else
			state = nxtState;
		count = state;
	end
	
	always@(state)
	begin
		case(state)
		3'd0: nxtState = 3'd1;
		3'd1: nxtState = 3'd2;
		3'd2: nxtState = 3'd3;
		3'd3: nxtState = 3'd4;
		3'd4: nxtState = 3'd0;
		endcase
	end
	
endmodule

module cacheControlCkt(input clk,input reset, input memRd, input memWr, input wayHit, 
								input [2:0] count, input cacheHit, output reg mruUpd, output reg writeBuffer, 
								output reg fifoIncEn, output reg replace, output reg outputEn,	output reg dWordWr);
	
	reg [2:0] state, nxtState;
	
	always@(negedge clk)
	begin
		if(reset)
		begin
			mruUpd = 1'b0;
			writeBuffer = 1'b0;
			fifoIncEn = 1'b0;
			replace = 1'b0;
			dWordWr = 1'b0;
			outputEn = 1'b0;
			
			nxtState = 3'd0;
		end
		else
			state = nxtState;
	end
	
	always@(state, memRd, memWr, wayHit, count, cacheHit)
	begin
		case(state)
		3'd0:
		begin
			//Set Outputs
			mruUpd = 1'b0;
			writeBuffer = 1'b0;
			fifoIncEn = 1'b0;
			replace = 1'b0;
			dWordWr = 1'b0;
			outputEn = 1'b0;
			
			if(wayHit && memRd)
				nxtState = 3'd2;
			else if(wayHit && memWr)
				nxtState = 3'd4;
			else if(~wayHit && (memRd || memWr))
				nxtState = 3'd1;
			else
				nxtState = 3'd5;
		end
		3'd1:
		begin
			//Set Outputs
			mruUpd = 1'b0;
			writeBuffer = 1'b0;
			fifoIncEn = 1'b0;
			replace = 1'b0;
			dWordWr = 1'b0;
			outputEn = 1'b0;
			
			if(cacheHit && memRd)
				nxtState = 3'd2;
			else if(cacheHit && memWr)
				nxtState = 3'd4;
			else if(~cacheHit && (memRd || memWr))
				nxtState = 3'd3;
			else
				nxtState = 3'd5;
		end
		3'd2:
		begin
			//Set Outputs
			mruUpd = 1'b1;
			writeBuffer = 1'b0;
			fifoIncEn = 1'b0;
			replace = 1'b0;
			dWordWr = 1'b0;
			
			if(count == 3'd4 && memRd)
				outputEn = 1'b1;
			
			if(memWr)
				writeBuffer = 1'b1;
			if(count == 3'd4)
				nxtState = 3'd0;
			else
				nxtState = 3'd5;
			
		end
		3'd3:
		begin
			//Set Outputs
			mruUpd = 1'b0;
			writeBuffer = 1'b0;
			dWordWr = 1'b0;
			fifoIncEn = 1'b1;
			replace = 1'b1;
			outputEn = 1'b0;
			
			if(memRd)
				nxtState = 3'd2;
			else if(memWr)
				nxtState = 3'd4;
			else 
				nxtState = 3'd5;
		end
		3'd4:
		begin
			//Set Outputs & disable writes
			mruUpd = 1'b0;
			writeBuffer = 1'b0;
			fifoIncEn = 1'b0;
			replace = 1'b0;
			dWordWr = 1'b1;
			outputEn = 1'b0;
			
			nxtState = 3'd2;
		end
		3'd5:
		begin
			mruUpd = 1'b0;
			writeBuffer = 1'b0;
			fifoIncEn = 1'b0;
			replace = 1'b0;
			dWordWr = 1'b0;
			
			if(count == 3'd4 && memRd)
				outputEn = 1'b1;
			
			if(count == 3'd4)
				nxtState = 3'd0;
			else
				nxtState = 3'd5;
		end
		endcase
	end
endmodule

module cache(input clk, input reset, input memRd, input memWr, input [31:0] physicalAdd, 
					input [31:0] dataIn, input [127:0] memDataBus, output [47:0] dataOut, output miss);
	
	wire viMuxOut0, viMuxOut1, viMuxOut2, viMuxOut3, compOut0, compOut1, compOut2, compOut3, wayHit, cacheHit, mruUpd, writeToBuffer, fifoIncEn, replace, dWordWrite, outputEn;
	wire [1:0] mruOut0, mruOut1, mruOut2, mruOut3, mruOut4, mruOut5, mruOut6, mruOut7, mruOut8, mruOut9, mruOut10, mruOut11, mruOut12, mruOut13, mruOut14, mruOut15, muxFifoOut0, muxFifoOut1, muxFifoOut2, muxFifoOut3, predictedWay, cacheHitWay, waySelected, replacementWay;
	wire [2:0] count;
	wire [3:0] demuxIn, dWordDemuxIn;
	wire [15:0] valid0, valid1, valid2, valid3, fifoSet0, fifoSet1, fifoSet2, fifoSet3, incEnable0, incEnable1, incEnable2, incEnable3, mruWriteEn, dWordWr0, dWordWr1, dWordWr2, dWordWr3, repeatedDWordWr, b0DWordWr, b1DWordWr, b2DWordWr, b3DWordWr;
	wire [24:0] tagMuxOut0, tagMuxOut1, tagMuxOut2, tagMuxOut3;
	wire [31:0] fifoOut0, fifoOut1, fifoOut2, fifoOut3;
	wire [47:0] outputMuxIn1;
	wire [127:0] muxdbOut0, muxdbOut1, muxdbOut2, muxdbOut3, selectedBlock;
	wire [399:0] tagOut0, tagOut1, tagOut2, tagOut3;
	wire [2047:0] dataOut0, dataOut1, dataOut2, dataOut3;
	mruArray mru_arr(clk, reset, {mruWriteEn & {16{mruUpd}}}, waySelected, waySelected, waySelected, waySelected,
							 waySelected, waySelected, waySelected, waySelected, waySelected, waySelected, waySelected,
								 waySelected, waySelected, waySelected, waySelected, waySelected,mruOut0, mruOut1, 
									mruOut2, mruOut3, mruOut4, mruOut5, mruOut6, mruOut7, mruOut8, mruOut9, mruOut10, mruOut11, 
										mruOut12, mruOut13, mruOut14, mruOut15);
	
	//We can hardcode d(4th input of viArray) in viArray
	viArray vi0(clk, reset, {fifoSet0 & {16{replace}}}, {16{1'b1}}, valid0);
	viArray vi1(clk, reset, {fifoSet1 & {16{replace}}}, {16{1'b1}}, valid1);
	viArray vi2(clk, reset, {fifoSet2 & {16{replace}}}, {16{1'b1}}, valid2);
	viArray vi3(clk, reset, {fifoSet3 & {16{replace}}}, {16{1'b1}}, valid3);
	
	//inputTag = physicalAdd[31:7]
	tagArray tagArr0(clk, reset, {fifoSet0 & {16{replace}}}, {16{physicalAdd[31:7]}}, tagOut0);
	tagArray tagArr1(clk, reset, {fifoSet1 & {16{replace}}}, {16{physicalAdd[31:7]}}, tagOut1);
	tagArray tagArr2(clk, reset, {fifoSet2 & {16{replace}}}, {16{physicalAdd[31:7]}}, tagOut2);
	tagArray tagArr3(clk, reset, {fifoSet3 & {16{replace}}}, {16{physicalAdd[31:7]}}, tagOut3);
	
	fifoArray fifoArr0(clk, reset, {fifoSet0 & {16{replace}}}, {incEnable0 & {16{fifoIncEn}}}, fifoOut0);
	fifoArray fifoArr1(clk, reset, {fifoSet1 & {16{replace}}}, {incEnable1 & {16{fifoIncEn}}}, fifoOut1);
	fifoArray fifoArr2(clk, reset, {fifoSet2 & {16{replace}}}, {incEnable2 & {16{fifoIncEn}}}, fifoOut2);
	fifoArray fifoArr3(clk, reset, {fifoSet3 & {16{replace}}}, {incEnable3 & {16{fifoIncEn}}}, fifoOut3);
	
	//change dword
	cacheDataBlockArray block0(clk, reset, {fifoSet0 & {16{replace}}}, {dWordWr0 & {16{dWordWrite}}}, 
										physicalAdd[2:0], dataIn, {16{memDataBus}}, dataOut0);
	cacheDataBlockArray block1(clk, reset, {fifoSet1 & {16{replace}}}, {dWordWr1 & {16{dWordWrite}}}, 
										physicalAdd[2:0], dataIn, {16{memDataBus}}, dataOut1);
	cacheDataBlockArray block2(clk, reset, {fifoSet2 & {16{replace}}}, {dWordWr2 & {16{dWordWrite}}}, 
										physicalAdd[2:0],  dataIn, {16{memDataBus}}, dataOut2);
	cacheDataBlockArray block3(clk, reset, {fifoSet3 & {16{replace}}}, {dWordWr3 & {16{dWordWrite}}}, 
										physicalAdd[2:0],  dataIn, {16{memDataBus}}, dataOut3);
										
	writeBuffer buffer(clk, reset, writeToBuffer, physicalAdd[31:3], selectedBlock);
	
	//Mux for Way Hit or Cache Hit
	//For Valid arrays
	//index = physicalAdd[6:3];
	mux16to1_1bits muxvi0(valid0[0], valid0[1], valid0[2], valid0[3], valid0[4], 
									 valid0[5], valid0[6], valid0[7], valid0[8], valid0[9], 
										valid0[10], valid0[11], valid0[12], valid0[13], valid0[14], 
											valid0[15], physicalAdd[6:3], viMuxOut0);
	mux16to1_1bits muxvi1(valid1[0], valid1[1], valid1[2], valid1[3], valid1[4], 
									 valid1[5], valid1[6], valid1[7], valid1[8], valid1[9], 
										valid1[10], valid1[11], valid1[12], valid1[13], valid1[14], 
											valid1[15], physicalAdd[6:3], viMuxOut1);
	mux16to1_1bits muxvi2(valid2[0], valid2[1], valid2[2], valid2[3], valid2[4], 
									 valid2[5], valid2[6], valid2[7], valid2[8], valid2[9], 
										valid2[10], valid2[11], valid2[12], valid2[13], valid2[14], 
											valid2[15], physicalAdd[6:3], viMuxOut2);
	mux16to1_1bits muxvi3(valid3[0], valid3[1], valid3[2], valid3[3], valid3[4], 
									 valid3[5], valid3[6], valid3[7], valid3[8], valid3[9], 
										valid3[10], valid3[11], valid3[12], valid3[13], valid3[14], 
											valid3[15], physicalAdd[6:3], viMuxOut3);
											
	//For Tag arrays
	mux16to1_25bits muxTag0(tagOut0[24:0] ,tagOut0[49:25] ,tagOut0[74:50] ,tagOut0[99:75] ,tagOut0[124:100] ,
									tagOut0[149:125] ,tagOut0[174:150] ,tagOut0[199:175] ,tagOut0[224:200] ,tagOut0[249:225] ,
										tagOut0[274:250] ,tagOut0[299:275] ,tagOut0[324:300] ,tagOut0[349:325] ,tagOut0[374:350] ,
											tagOut0[399:375] , physicalAdd[6:3], tagMuxOut0);
	mux16to1_25bits muxTag1(tagOut1[24:0] ,tagOut1[49:25] ,tagOut1[74:50] ,tagOut1[99:75] ,tagOut1[124:100] ,
									tagOut1[149:125] ,tagOut1[174:150] ,tagOut1[199:175] ,tagOut1[224:200] ,tagOut1[249:225] ,
										tagOut1[274:250] ,tagOut1[299:275] ,tagOut1[324:300] ,tagOut1[349:325] ,tagOut1[374:350] ,
											tagOut1[399:375] , physicalAdd[6:3], tagMuxOut1);
	mux16to1_25bits muxTag2(tagOut2[24:0] ,tagOut2[49:25] ,tagOut2[74:50] ,tagOut2[99:75] ,tagOut2[124:100] ,
									tagOut2[149:125] ,tagOut2[174:150] ,tagOut2[199:175] ,tagOut2[224:200] ,tagOut2[249:225] ,
										tagOut2[274:250] ,tagOut2[299:275] ,tagOut2[324:300] ,tagOut2[349:325] ,tagOut2[374:350] ,
											tagOut2[399:375] , physicalAdd[6:3], tagMuxOut2);
	mux16to1_25bits muxTag3(tagOut3[24:0] ,tagOut3[49:25] ,tagOut3[74:50] ,tagOut3[99:75] ,tagOut3[124:100] ,
									tagOut3[149:125] ,tagOut3[174:150] ,tagOut3[199:175] ,tagOut3[224:200] ,tagOut3[249:225] ,
										tagOut3[274:250] ,tagOut3[299:275] ,tagOut3[324:300] ,tagOut3[349:325] ,tagOut3[374:350] ,
											tagOut3[399:375] , physicalAdd[6:3], tagMuxOut3);
	//Tag comparators
	tagComparator comp0(physicalAdd[31:7], tagMuxOut0, compOut0);
	tagComparator comp1(physicalAdd[31:7], tagMuxOut1, compOut1);
	tagComparator comp2(physicalAdd[31:7], tagMuxOut2, compOut2);
	tagComparator comp3(physicalAdd[31:7], tagMuxOut3, compOut3);
	
	//Mru mux
	mux16to1_2bits mruMux(mruOut0, mruOut1, mruOut2, mruOut3, mruOut4, mruOut5, mruOut6, 
									mruOut7, mruOut8, mruOut9, mruOut10, mruOut11, mruOut12, mruOut13, 
										mruOut14, mruOut15, physicalAdd[6:3], predictedWay);
	
	//Way hit mux
	mux4to1_1bits wayHitMux(viMuxOut0 & compOut0, viMuxOut1 & compOut1, viMuxOut2 & compOut2, viMuxOut3 & compOut3, predictedWay, wayHit);
	
	//Cache hit way mux
	encoder4to2_1bit cacheHitWayEncoder({viMuxOut3 & compOut3, viMuxOut2 & compOut2, viMuxOut1 & compOut1,viMuxOut0 & compOut0}, cacheHitWay);
	
	//Cache hit assign
	assign cacheHit = (viMuxOut3 & compOut3) | (viMuxOut2 & compOut2) | (viMuxOut1 & compOut1) | (viMuxOut0 & compOut0);
	
	//Way selected Mux
	mux2to1_2bits waySelectMux(predictedWay, cacheHitWay, ~wayHit, waySelected);
	
	//Counter for counting the cycles
	counter cycleCount(clk, reset, count);
	
	//Control Circuit
	cacheControlCkt ctrlCkt(clk, reset, memRd, memWr, wayHit, count, cacheHit,	mruUpd, writeToBuffer, fifoIncEn, 
										replace, outputEn, dWordWrite);
	
	//For Fifo arrays
	mux16to1_2bits muxFIFOBits0(fifoOut0[1:0], fifoOut0[3:2], fifoOut0[5:4], fifoOut0[7:6], fifoOut0[9:8], 
											fifoOut0[11:10], fifoOut0[13:12], fifoOut0[15:14], fifoOut0[17:16], fifoOut0[19:18], 
												fifoOut0[21:20], fifoOut0[23:22], fifoOut0[25:24], fifoOut0[27:26], fifoOut0[29:28], 
													fifoOut0[31:30], physicalAdd[6:3], muxFifoOut0);
	mux16to1_2bits muxFIFOBits1(fifoOut1[1:0], fifoOut1[3:2], fifoOut1[5:4], fifoOut1[7:6], fifoOut1[9:8], 
											fifoOut1[11:10], fifoOut1[13:12], fifoOut1[15:14], fifoOut1[17:16], fifoOut1[19:18], 
												fifoOut1[21:20], fifoOut1[23:22], fifoOut1[25:24], fifoOut1[27:26], fifoOut1[29:28], 
													fifoOut1[31:30], physicalAdd[6:3], muxFifoOut1);
	mux16to1_2bits muxFIFOBits2(fifoOut2[1:0], fifoOut2[3:2], fifoOut2[5:4], fifoOut2[7:6], fifoOut2[9:8], 
											fifoOut2[11:10], fifoOut2[13:12], fifoOut2[15:14], fifoOut2[17:16], fifoOut2[19:18], 
												fifoOut2[21:20], fifoOut2[23:22], fifoOut2[25:24], fifoOut2[27:26], fifoOut2[29:28], 
													fifoOut2[31:30], physicalAdd[6:3], muxFifoOut2);
	mux16to1_2bits muxFIFOBits3(fifoOut3[1:0], fifoOut3[3:2], fifoOut3[5:4], fifoOut3[7:6], fifoOut3[9:8], 
											fifoOut3[11:10], fifoOut3[13:12], fifoOut3[15:14], fifoOut3[17:16], fifoOut3[19:18], 
												fifoOut3[21:20], fifoOut3[23:22], fifoOut3[25:24], fifoOut3[27:26], fifoOut3[29:28], 
													fifoOut3[31:30], physicalAdd[6:3], muxFifoOut3);
	
	//For Data Block Arrays
	mux16to1_128bits muxdb0(dataOut0[127:0], dataOut0[255:128], dataOut0[383:256], dataOut0[511:384],
										dataOut0[639:512], dataOut0[767:640], dataOut0[895:768], dataOut0[1023:896], 
											dataOut0[1151:1024], dataOut0[1279:1152], dataOut0[1407:1280], dataOut0[1535:1408],
												dataOut0[1663:1536], dataOut0[1791:1664] , dataOut0[1919:1792], dataOut0[2047:1920], 
													physicalAdd[6:3], muxdbOut0);
	mux16to1_128bits muxdb1(dataOut1[127:0], dataOut1[255:128], dataOut1[383:256], dataOut1[511:384],
										dataOut1[639:512], dataOut1[767:640], dataOut1[895:768], dataOut1[1023:896], 
											dataOut1[1151:1024], dataOut1[1279:1152], dataOut1[1407:1280], dataOut1[1535:1408],
												dataOut1[1663:1536], dataOut1[1791:1664] , dataOut1[1919:1792], dataOut1[2047:1920], 
													physicalAdd[6:3], muxdbOut1);
	mux16to1_128bits muxdb2(dataOut2[127:0], dataOut2[255:128], dataOut2[383:256], dataOut2[511:384],
										dataOut2[639:512], dataOut2[767:640], dataOut2[895:768], dataOut2[1023:896], 
											dataOut2[1151:1024], dataOut2[1279:1152], dataOut2[1407:1280], dataOut2[1535:1408],
												dataOut2[1663:1536], dataOut2[1791:1664] , dataOut2[1919:1792], dataOut2[2047:1920], 
													physicalAdd[6:3], muxdbOut2);
	mux16to1_128bits muxdb3(dataOut3[127:0], dataOut3[255:128], dataOut3[383:256], dataOut3[511:384],
										dataOut3[639:512], dataOut3[767:640], dataOut3[895:768], dataOut3[1023:896], 
											dataOut3[1151:1024], dataOut3[1279:1152], dataOut3[1407:1280], dataOut3[1535:1408],
												dataOut3[1663:1536], dataOut3[1791:1664] , dataOut3[1919:1792], dataOut3[2047:1920], 
													physicalAdd[6:3], muxdbOut3);
	
	//Block Select mux
	mux4to1_128bits blockSelectMux(muxdbOut0, muxdbOut1, muxdbOut2, muxdbOut3, waySelected, selectedBlock);
	
	//Offset = physicalAdd[2:0];
	//Mem read output mux
	mux8to1_48bits dataOutMux(selectedBlock[47:0] , selectedBlock[63:16], selectedBlock[79:32], selectedBlock[95:48],
							selectedBlock[111:64], selectedBlock[127:80],{16'd0, selectedBlock[127:96]},
							{32'd0, selectedBlock[127:112]}, physicalAdd[2:0], outputMuxIn1);				
	
	//Way to replace using priority encoder
	priorityEncoder4to2_1bit replacementWayEncoder(~(muxFifoOut0[0] & muxFifoOut0[1]), 
								~(muxFifoOut1[0] & muxFifoOut1[1]), ~(muxFifoOut2[0] & muxFifoOut2[1]), 
									~(muxFifoOut3[0] & muxFifoOut3[1]), replacementWay);
	
	//Input of set of FIFO array demux
	decoder2to4_1bit demuxInput(replacementWay, demuxIn);
	
	//Demux for Set of fifo array
	demux1to16_1bit setDemux0(~demuxIn[0], physicalAdd[6:3], fifoSet0[0], fifoSet0[1], fifoSet0[2], fifoSet0[3], 
										fifoSet0[4], fifoSet0[5], fifoSet0[6], fifoSet0[7], fifoSet0[8], fifoSet0[9], 
											fifoSet0[10], fifoSet0[11], fifoSet0[12], fifoSet0[13], fifoSet0[14], fifoSet0[15]);
	demux1to16_1bit setDemux1(~demuxIn[1], physicalAdd[6:3], fifoSet1[0], fifoSet1[1], fifoSet1[2], fifoSet1[3], 
										fifoSet1[4], fifoSet1[5], fifoSet1[6], fifoSet1[7], fifoSet1[8], fifoSet1[9], 
											fifoSet1[10], fifoSet1[11], fifoSet1[12], fifoSet1[13], fifoSet1[14], fifoSet1[15]);
	
	demux1to16_1bit setDemux2(~demuxIn[2], physicalAdd[6:3], fifoSet2[0], fifoSet2[1], fifoSet2[2], fifoSet2[3], 
										fifoSet2[4], fifoSet2[5], fifoSet2[6], fifoSet2[7], fifoSet2[8], fifoSet2[9], 
											fifoSet2[10], fifoSet2[11], fifoSet2[12], fifoSet2[13], fifoSet2[14], fifoSet2[15]);
	demux1to16_1bit setDemux3(~demuxIn[3], physicalAdd[6:3], fifoSet3[0], fifoSet3[1], fifoSet3[2], fifoSet3[3], 
										fifoSet3[4], fifoSet3[5], fifoSet3[6], fifoSet3[7], fifoSet3[8], fifoSet3[9], 
											fifoSet3[10], fifoSet3[11], fifoSet3[12], fifoSet3[13], fifoSet3[14], fifoSet3[15]);
	
	//Demux for increment enable
	demux1to16_1bit fifoIncrementdemux0(demuxIn[0], physicalAdd[6:3], incEnable0[0], incEnable0[1],
													incEnable0[2], incEnable0[3], incEnable0[4], incEnable0[5], 
														incEnable0[6], incEnable0[7], incEnable0[8], incEnable0[9], 
															incEnable0[10], incEnable0[11], incEnable0[12], incEnable0[13],
																incEnable0[14], incEnable0[15]);
	
	demux1to16_1bit fifoIncrementdemux1(demuxIn[1], physicalAdd[6:3], incEnable1[0], incEnable1[1],
													incEnable1[2], incEnable1[3], incEnable1[4], incEnable1[5], 
														incEnable1[6], incEnable1[7], incEnable1[8], incEnable1[9], 
															incEnable1[10], incEnable1[11], incEnable1[12], incEnable1[13],
																incEnable1[14], incEnable1[15]);
	
	demux1to16_1bit fifoIncrementdemux2(demuxIn[2], physicalAdd[6:3], incEnable2[0], incEnable2[1],
													incEnable2[2], incEnable2[3], incEnable2[4], incEnable2[5], 
														incEnable2[6], incEnable2[7], incEnable2[8], incEnable2[9], 
															incEnable2[10], incEnable2[11], incEnable2[12], incEnable2[13],
																incEnable2[14], incEnable2[15]);
	
	demux1to16_1bit fifoIncrementdemux3(demuxIn[3], physicalAdd[6:3], incEnable3[0], incEnable3[1],
													incEnable3[2], incEnable3[3], incEnable3[4], incEnable3[5], 
														incEnable3[6], incEnable3[7], incEnable3[8], incEnable3[9], 
															incEnable3[10], incEnable3[11], incEnable3[12], incEnable3[13],
																incEnable3[14], incEnable3[15]);
	//Mru Write Enable
	demux1to16_1bit mruWriteEnable(1'b1, physicalAdd[6:3], mruWriteEn[0], mruWriteEn[1], mruWriteEn[2], mruWriteEn[3], 
							mruWriteEn[4], mruWriteEn[5], mruWriteEn[6], mruWriteEn[7], mruWriteEn[8], mruWriteEn[9], 
								mruWriteEn[10], mruWriteEn[11], mruWriteEn[12], mruWriteEn[13], mruWriteEn[14], mruWriteEn[15]);
	
	//OutputMux
	mux2to1_48bits outputMux(48'd0, outputMuxIn1, outputEn, dataOut);
	
	//Mux and Demux for dWordWr of all ways
	decoder2to4_1bit decoderForDwordWr(waySelected, dWordDemuxIn);
	
	demux1to16_1bit dwordWrdemux0(~dWordDemuxIn[0], physicalAdd[6:3], dWordWr0[0], dWordWr0[1],
													dWordWr0[2], dWordWr0[3], dWordWr0[4], dWordWr0[5], 
														dWordWr0[6], dWordWr0[7], dWordWr0[8], dWordWr0[9], 
															dWordWr0[10], dWordWr0[11], dWordWr0[12], dWordWr0[13],
																dWordWr0[14], dWordWr0[15]);
	demux1to16_1bit dwordWrdemux1(~dWordDemuxIn[1], physicalAdd[6:3], dWordWr1[0], dWordWr1[1],
													dWordWr1[2], dWordWr1[3], dWordWr1[4], dWordWr1[5], 
														dWordWr1[6], dWordWr1[7], dWordWr1[8], dWordWr1[9], 
															dWordWr1[10], dWordWr1[11], dWordWr1[12], dWordWr1[13],
																dWordWr1[14], dWordWr1[15]);
	demux1to16_1bit dwordWrdemux2(~dWordDemuxIn[2], physicalAdd[6:3], dWordWr2[0], dWordWr2[1],
													dWordWr2[2], dWordWr2[3], dWordWr2[4], dWordWr2[5], 
														dWordWr2[6], dWordWr2[7], dWordWr2[8], dWordWr2[9], 
															dWordWr2[10], dWordWr2[11], dWordWr2[12], dWordWr2[13],
																dWordWr2[14], dWordWr2[15]);
	demux1to16_1bit dwordWrdemux3(~dWordDemuxIn[3], physicalAdd[6:3], dWordWr3[0], dWordWr3[1],
													dWordWr3[2], dWordWr3[3], dWordWr3[4], dWordWr3[5], 
														dWordWr3[6], dWordWr3[7], dWordWr3[8], dWordWr3[9], 
															dWordWr3[10], dWordWr3[11], dWordWr3[12], dWordWr3[13],
																dWordWr3[14], dWordWr3[15]);
																
	//D-Flip Flop to latch cache miss
	D_ff miss_latch(clk, (reset | (count[2] & ~count[1] & ~count[0])), ~cacheHit & (~count[2] & ~count[1] & count[0]), 1'b1, miss);
endmodule


module CacheModuleTestBench;
	reg clk;
	reg reset;
	wire miss;
	wire [47:0] Result;
	reg memRd;
	reg [127:0] memDataBus;
	reg memWr;
	reg [31:0] vAdd;
	reg [31:0] dataIn;
	cache uut (clk,reset,memRd,memWr,vAdd,dataIn,memDataBus,Result,miss);

	always
	#5 clk=~clk;
	
	initial
	begin
		clk=0; reset=1;
		#10	reset=0; memRd=0; memWr=1; vAdd=32'd0; dataIn=32'haaaaaaaa; memDataBus=128'h11111111111111111111111111111111;//Write Miss
		#50	memRd=1; memWr=0; vAdd=32'd0; memDataBus=128'h22222222222222222222222222222222;//Way Read Hit
		#50	memRd=0; memWr=0;//Idle for 1 cycle
		#50	memRd=0; memWr=1; vAdd=32'h80; dataIn=32'hbbbbbbbb; memDataBus=128'h22222222222222222222222222222222;//Write in same set, Cache Write Miss
		#50	memRd=0; memWr=1; vAdd=32'h80; dataIn=32'hcccccccc; memDataBus=128'h33333333333333333333333333333333;//Way Hit write
		#50  	memRd=1; memWr=0; vAdd=32'h40; dataIn=32'h88888888; memDataBus=128'h44444444444444444444444444444444;//Cache Read Miss
		#50	memRd=1; memWr=0; vAdd=32'd0; dataIn=32'haaaaaaaa; memDataBus=128'h55555555555555555555555555555555;//Way Read Miss, Cache Read hit
		#50	memRd=1; memWr=0; vAdd=32'h80; dataIn=32'h00000000; memDataBus=128'h55555555555555555555555555555555;//Way read Hit
		#50	memRd=0; memWr=1; vAdd=32'h80; dataIn=32'h00000000; memDataBus=128'h44444444444444444444444444444444;//Way write Hit
		#50	memRd=1; memWr=0; vAdd=32'h80;	//Way Read Hit
		#60 $finish; 
	end
endmodule




//Pipelining
module D_ff_IM(input clk,input reset,input d,output reg q);
	always@(reset or posedge clk)
	if(reset)
		q=d;
endmodule


module register_IM(input clk, input reset, input [15:0] d_in, output [15:0] q_out);
	D_ff_IM dIM0 (clk, reset, d_in[0], q_out[0]);
	D_ff_IM dIM1 (clk, reset, d_in[1], q_out[1]);
	D_ff_IM dIM2 (clk, reset, d_in[2], q_out[2]);
	D_ff_IM dIM3 (clk, reset, d_in[3], q_out[3]);
	D_ff_IM dIM4 (clk, reset, d_in[4], q_out[4]);
	D_ff_IM dIM5 (clk, reset, d_in[5], q_out[5]);
	D_ff_IM dIM6 (clk, reset, d_in[6], q_out[6]);
	D_ff_IM dIM7 (clk, reset, d_in[7], q_out[7]);
	D_ff_IM dIM8 (clk, reset, d_in[8], q_out[8]);
	D_ff_IM dIM9 (clk, reset, d_in[9], q_out[9]);
	D_ff_IM dIM10 (clk, reset, d_in[10], q_out[10]);
	D_ff_IM dIM11 (clk, reset, d_in[11], q_out[11]);
	D_ff_IM dIM12 (clk, reset, d_in[12], q_out[12]);
	D_ff_IM dIM13 (clk, reset, d_in[13], q_out[13]);
	D_ff_IM dIM14 (clk, reset, d_in[14], q_out[14]);
	D_ff_IM dIM15 (clk, reset, d_in[15], q_out[15]);
endmodule


//Why 32??

module mux32to1_IM(input [15:0] outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15,
	outR16,outR17,outR18,outR19,outR20,outR21,outR22,outR23,outR24,outR25,outR26,outR27,outR28,outR29,outR30,outR31,
	input [4:0] Sel, output reg [47:0] outBus );

	always@(outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15,
		outR16,outR17,outR18,outR19,outR20,outR21,outR22,outR23,outR24,outR25,outR26,outR27,outR28,outR29,outR30,outR31,
		Sel)

	case(Sel)
		5'd0: outBus = {outR2,outR1,outR0};
		5'd1: outBus = {outR3,outR2,outR1};
		5'd2: outBus = {outR4,outR3,outR2};
		5'd3: outBus = {outR5,outR4,outR3};
		5'd4: outBus = {outR6,outR5,outR4};
		5'd5: outBus = {outR7,outR6,outR5};
		5'd6: outBus = {outR8,outR7,outR6};
		5'd7: outBus = {outR9,outR8,outR7};
		5'd8: outBus = {outR10,outR9,outR8};
		5'd9: outBus = {outR11,outR10,outR9};
		5'd10: outBus = {outR12,outR11,outR10};
		5'd11: outBus = {outR13,outR12,outR11};
		5'd12: outBus = {outR14,outR13,outR12};
		5'd13: outBus = {outR15,outR14,outR13};
		5'd14: outBus = {outR16,outR15,outR14};
		5'd15: outBus = {outR17,outR16,outR15};
		5'd16: outBus = {outR18,outR17,outR16};
		5'd17: outBus = {outR19,outR18,outR17};
		5'd18: outBus = {outR20,outR19,outR18};
		5'd19: outBus = {outR21,outR20,outR19};
		5'd20: outBus = {outR22,outR21,outR20};
		5'd21: outBus = {outR23,outR22,outR21};
		5'd22: outBus = {outR24,outR23,outR22};
		5'd23: outBus = {outR25,outR24,outR23};
		5'd24: outBus = {outR26,outR25,outR24};
		5'd25: outBus = {outR27,outR26,outR25};
		5'd26: outBus = {outR28,outR27,outR26};
		5'd27: outBus = {outR29,outR28,outR27};
		5'd28: outBus = {outR30,outR29,outR28};
		5'd29: outBus = {outR31,outR30,outR29};
		5'd30: outBus = {16'b0,outR31,outR30};
		5'd31: outBus = {16'b0,16'b0,outR31};
		endcase
endmodule


module mux64to1_IM(input [15:0] outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15,
	outR16,outR17,outR18,outR19,outR20,outR21,outR22,outR23,outR24,outR25,outR26,outR27,outR28,outR29,outR30,outR31,outR32,outR33,outR34,
	outR35,outR36,outR37,outR38,outR39,outR40,outR41,outR42,outR43,outR44,outR45,outR46,outR47,outR48,outR49,outR50,outR51,outR52,outR53,
	outR54,outR55,outR56,outR57,outR58,outR59,outR60,outR61,outR62,outR63,input [5:0] Sel, output reg [47:0] outBus );

	always@(outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15,
		outR16,outR17,outR18,outR19,outR20,outR21,outR22,outR23,outR24,outR25,outR26,outR27,outR28,outR29,outR30,outR31,
		outR32,outR33,outR34,outR35,outR36,outR37,outR38,outR39,outR40,outR41,outR42,outR43,outR44,outR45,outR46,outR47,
		outR48,outR49,outR50,outR51,outR52,outR53,outR54,outR55,outR56,outR57,outR58,outR59,outR60,outR61,outR62,outR63,Sel)

	case(Sel)
		6'd0: outBus = {outR2,outR1,outR0};
		6'd1: outBus = {outR3,outR2,outR1};
		6'd2: outBus = {outR4,outR3,outR2};
		6'd3: outBus = {outR5,outR4,outR3};
		6'd4: outBus = {outR6,outR5,outR4};
		6'd5: outBus = {outR7,outR6,outR5};
		6'd6: outBus = {outR8,outR7,outR6};
		6'd7: outBus = {outR9,outR8,outR7};
		6'd8: outBus = {outR10,outR9,outR8};
		6'd9: outBus = {outR11,outR10,outR9};
		6'd10: outBus = {outR12,outR11,outR10};
		6'd11: outBus = {outR13,outR12,outR11};
		6'd12: outBus = {outR14,outR13,outR12};
		6'd13: outBus = {outR15,outR14,outR13};
		6'd14: outBus = {outR16,outR15,outR14};
		6'd15: outBus = {outR17,outR16,outR15};
		6'd16: outBus = {outR18,outR17,outR16};
		6'd17: outBus = {outR19,outR18,outR17};
		6'd18: outBus = {outR20,outR19,outR18};
		6'd19: outBus = {outR21,outR20,outR19};
		6'd20: outBus = {outR22,outR21,outR20};
		6'd21: outBus = {outR23,outR22,outR21};
		6'd22: outBus = {outR24,outR23,outR22};
		6'd23: outBus = {outR25,outR24,outR23};
		6'd24: outBus = {outR26,outR25,outR24};
		6'd25: outBus = {outR27,outR26,outR25};
		6'd26: outBus = {outR28,outR27,outR26};
		6'd27: outBus = {outR29,outR28,outR27};
		6'd28: outBus = {outR30,outR29,outR28};
		6'd29: outBus = {outR31,outR30,outR29};
		6'd30: outBus = {outR32,outR31,outR30};
		6'd31: outBus = {outR33,outR32,outR31};
		6'd32: outBus = {outR34,outR33,outR32};
		6'd33: outBus = {outR35,outR34,outR33};
		6'd34: outBus = {outR36,outR35,outR34};
		6'd35: outBus = {outR37,outR36,outR35};
		6'd36: outBus = {outR38,outR37,outR36};
		6'd37: outBus = {outR39,outR38,outR37};
		6'd38: outBus = {outR40,outR39,outR38};
		6'd39: outBus = {outR41,outR40,outR39};
		6'd40: outBus = {outR42,outR41,outR40};
		6'd41: outBus = {outR43,outR42,outR41};
		6'd42: outBus = {outR44,outR43,outR42};
		6'd43: outBus = {outR45,outR44,outR43};
		6'd44: outBus = {outR46,outR45,outR44};
		6'd45: outBus = {outR47,outR46,outR45};
		6'd46: outBus = {outR48,outR47,outR46};
		6'd47: outBus = {outR49,outR48,outR47};
		6'd48: outBus = {outR50,outR49,outR48};
		6'd49: outBus = {outR51,outR50,outR49};
		6'd50: outBus = {outR52,outR51,outR50};
		6'd51: outBus = {outR53,outR52,outR51};
		6'd52: outBus = {outR54,outR53,outR52};
		6'd53: outBus = {outR55,outR54,outR53};
		6'd54: outBus = {outR56,outR55,outR54};
		6'd55: outBus = {outR57,outR56,outR55};
		6'd56: outBus = {outR58,outR57,outR56};
		6'd57: outBus = {outR59,outR58,outR57};
		6'd58: outBus = {outR60,outR59,outR58};
		6'd59: outBus = {outR61,outR60,outR59};
		6'd60: outBus = {outR62,outR61,outR60};
		6'd61: outBus = {outR63,outR62,outR61};
		6'd62: outBus = {16'd0,outR63,outR62};
		6'd63: outBus = {16'd0,16'd0,outR63};

		endcase
endmodule

//Little or Big Indian

//module IM(input clk, input reset, input [4:0] pc_5bits, output [47:0] IR);
//	wire [15:0] Qout0, Qout1, Qout2, Qout3, Qout4, Qout5, Qout6, Qout7,
//					Qout8, Qout9, Qout10, Qout11, Qout12, Qout13, Qout14, Qout15,
//					Qout16, Qout17, Qout18, Qout19, Qout20, Qout21, Qout22, Qout23,
//					Qout24, Qout25, Qout26, Qout27, Qout28, Qout29, Qout30, Qout31;
//
//
//	register_IM rIM0 (clk, reset, 16'h8013, Qout0); //addi $0, $1, 1h
//	register_IM rIM1 (clk, reset, 16'h0010, Qout1); 
//	register_IM rIM2 (clk, reset, 16'h0000, Qout2);
//	register_IM rIM3 (clk, reset, 16'h0000, Qout3);//nop 
//	register_IM rIM4 (clk, reset, 16'h0113, Qout4);// addi $2,$0,2h
//	register_IM rIM5 (clk, reset, 16'h0020, Qout5); 
//	register_IM rIM6 (clk, reset, 16'he801, Qout6); 
//	register_IM rIM7 (clk, reset, 16'h0000, Qout7); //c.bneq $0, 10h
//	register_IM rIM8 (clk, reset, 16'h0193, Qout8);//addi $3, $2, 2h
//	register_IM rIM9 (clk, reset,  16'h0021, Qout9); 
//	register_IM rIM10 (clk, reset, 16'h0000, Qout10); 	
//	register_IM rIM11 (clk, reset, 16'h0000, Qout11);//nop
//	register_IM rIM12 (clk, reset, 16'h5213, Qout12);//srli $4, $0, 4h 
//	register_IM rIM13 (clk, reset, 16'h0040, Qout13);
//	register_IM rIM14 (clk, reset, 16'h0000, Qout14); 	
//	register_IM rIM15 (clk, reset, 16'h0000, Qout15);//nop
//	register_IM rIM16 (clk, reset, 16'h42b3, Qout16);//xor $5, $0, $2
//	register_IM rIM17 (clk, reset, 16'h0020, Qout17);
//	register_IM rIM18 (clk, reset, 16'h0000, Qout18);
//	register_IM rIM19 (clk, reset, 16'h0000, Qout19);//nop
//	register_IM rIM20 (clk, reset, 16'h2313, Qout20);//slti $6, $0, 4h
//	register_IM rIM21 (clk, reset, 16'h0040, Qout21);
//	register_IM rIM22 (clk, reset, 16'h0000, Qout22);
//	register_IM rIM23 (clk, reset, 16'h0000, Qout23);//nop
//	register_IM rIM24 (clk, reset, 16'h1223, Qout24);// sw $7, 4($6)
//	register_IM rIM25 (clk, reset, 16'h0073, Qout25);		
//	register_IM rIM26 (clk, reset, 16'h0000, Qout26);
//	register_IM rIM27 (clk, reset, 16'h0000, Qout27);//nop 	
//	register_IM rIM28 (clk, reset, 16'h0000, Qout28);//nop 	
//	register_IM rIM29 (clk, reset, 16'h0000, Qout29); 
//	register_IM rIM30 (clk, reset, 16'h0000, Qout30);
//	register_IM rIM31 (clk, reset, 16'h0000, Qout31);//nop 		
//	mux32to1_IM mIM (Qout0,Qout1,Qout2,Qout3,Qout4,Qout5,Qout6,Qout7,Qout8,Qout9,Qout10,Qout11,Qout12,Qout13,Qout14,Qout15,
//		Qout16,Qout17,Qout18,Qout19,Qout20,Qout21,Qout22,Qout23,Qout24,Qout25,Qout26,Qout27,Qout28,Qout29,Qout30,Qout31,
//		pc_5bits[4:0],IR);
//endmodule

module IM(input clk, input reset, input [5:0] pc_5bits, output [47:0] IR);

	wire [15:0] Qout0, Qout1, Qout2, Qout3, Qout4, Qout5, Qout6, Qout7,
					Qout8, Qout9, Qout10, Qout11, Qout12, Qout13, Qout14, Qout15,
					Qout16, Qout17, Qout18, Qout19, Qout20, Qout21, Qout22, Qout23,
					Qout24, Qout25, Qout26, Qout27, Qout28, Qout29, Qout30, Qout31,Qout32,
					Qout33,Qout34,Qout35,Qout36,Qout37,Qout38,Qout39,Qout40,Qout41,Qout42,
					Qout43,Qout44,Qout45,Qout46,Qout47,Qout48,Qout49,Qout50,Qout51,Qout52,Qout53,
					Qout54,Qout55,Qout56,Qout57,Qout58,Qout59,Qout60,Qout61,Qout62,Qout63;

 
	register_IM rIM0 (clk, reset, 16'h0013, Qout0); //addi $0, $0 3h
	register_IM rIM1 (clk, reset, 16'h0030, Qout1); 
	register_IM rIM2 (clk, reset, 16'h60ed, Qout2);
	register_IM rIM3 (clk, reset, 16'h0000, Qout3);//c.lui $1, 27b 
	register_IM rIM4 (clk, reset, 16'h0000, Qout4);// nop
	register_IM rIM5 (clk, reset, 16'h0000, Qout5); 
	register_IM rIM6 (clk, reset, 16'he411, Qout6); 
	register_IM rIM7 (clk, reset, 16'h0000, Qout7); //c.bneq $0, 12b
	register_IM rIM8 (clk, reset, 16'h0000, Qout8);//nop
	register_IM rIM9 (clk, reset,  16'h0000, Qout9); 
	register_IM rIM10 (clk, reset, 16'h0000, Qout10); 	
	register_IM rIM11 (clk, reset, 16'h0000, Qout11);//nop
	register_IM rIM12 (clk, reset, 16'h0000, Qout12);//nop 
	register_IM rIM13 (clk, reset, 16'h0000, Qout13);
	register_IM rIM14 (clk, reset, 16'h0000, Qout14); 	
	register_IM rIM15 (clk, reset, 16'h0000, Qout15);//nop
	register_IM rIM16 (clk, reset, 16'h1423, Qout16);//sw $1, 8($0)
	register_IM rIM17 (clk, reset, 16'h0010, Qout17);
	register_IM rIM18 (clk, reset, 16'h0000, Qout18);
	register_IM rIM19 (clk, reset, 16'h0000, Qout19);//nop
	register_IM rIM20 (clk, reset, 16'hd093, Qout20);//srli $1, $1, 2h
	register_IM rIM21 (clk, reset, 16'h0020, Qout21);
	register_IM rIM22 (clk, reset, 16'h440c, Qout22);
	register_IM rIM23 (clk, reset, 16'h0000, Qout23);//c.lw $3, $0, 8b
	register_IM rIM24 (clk, reset, 16'ha213, Qout24);// slti $4, $3, 29b
	register_IM rIM25 (clk, reset, 16'h01d1, Qout25);		
	register_IM rIM26 (clk, reset, 16'h6085, Qout26);
	register_IM rIM27 (clk, reset, 16'h0000, Qout27);//c.lui $1, 1h	
	register_IM rIM28 (clk, reset, 16'h82e7, Qout28);//jalr $5, $5, 0h 
	register_IM rIM29 (clk, reset, 16'h0002, Qout29); 
	register_IM rIM30 (clk, reset, 16'h0000, Qout30);
	register_IM rIM31 (clk, reset, 16'h0000, Qout31);//nop
	register_IM rIM32(clk, reset, 16'h0000, Qout32);//nop
	register_IM rIM33(clk, reset, 16'h0000, Qout33);
	register_IM rIM34(clk, reset, 16'h0000, Qout34);
	register_IM rIM35(clk, reset, 16'h0000, Qout35);//nop
	register_IM rIM36(clk, reset, 16'hc1b3, Qout36);//xor $3, $1, $0
	register_IM rIM37(clk, reset, 16'h0000, Qout37);
	register_IM rIM38(clk, reset, 16'h0000, Qout38);
	register_IM rIM39(clk, reset, 16'h0000, Qout39);//nop
	register_IM rIM40(clk, reset, 16'h0000, Qout40);
	register_IM rIM41(clk, reset, 16'h0000, Qout41);
	register_IM rIM42(clk, reset, 16'h0000, Qout42);
	register_IM rIM43(clk, reset, 16'h0000, Qout43);
	register_IM rIM44(clk, reset, 16'h0000, Qout44);
	register_IM rIM45(clk, reset, 16'h0000, Qout45);
	register_IM rIM46(clk, reset, 16'h0000, Qout46);
	register_IM rIM47(clk, reset, 16'h0000, Qout47);
	register_IM rIM48(clk, reset, 16'h0000, Qout48);
	register_IM rIM49(clk, reset, 16'h0000, Qout49);
	register_IM rIM50(clk, reset, 16'h0000, Qout50);
	register_IM rIM51(clk, reset, 16'h0000, Qout51);
	register_IM rIM52(clk, reset, 16'h0000, Qout52);
	register_IM rIM53(clk, reset, 16'h0000, Qout53);
	register_IM rIM54(clk, reset, 16'h0000, Qout54);
	register_IM rIM55(clk, reset, 16'h0000, Qout55);
	register_IM rIM56(clk, reset, 16'h0000, Qout56);
	register_IM rIM57(clk, reset, 16'h0000, Qout57);
	register_IM rIM58(clk, reset, 16'h0000, Qout58);
	register_IM rIM59(clk, reset, 16'h0000, Qout59);
	register_IM rIM60(clk, reset, 16'h0000, Qout60);
	register_IM rIM61(clk, reset, 16'h0000, Qout61);
	register_IM rIM62(clk, reset, 16'h0000, Qout62);
	register_IM rIM63(clk, reset, 16'h0000, Qout63);
	mux64to1_IM mIM (Qout0,Qout1,Qout2,Qout3,Qout4,Qout5,Qout6,Qout7,Qout8,Qout9,Qout10,Qout11,Qout12,Qout13,Qout14,Qout15,
		Qout16,Qout17,Qout18,Qout19,Qout20,Qout21,Qout22,Qout23,Qout24,Qout25,Qout26,Qout27,Qout28,Qout29,Qout30,Qout31,Qout32,
		Qout33,Qout34,Qout35,Qout36,Qout37,Qout38,Qout39,Qout40,Qout41,Qout42,Qout43,Qout44,Qout45,Qout46,Qout47,Qout48,Qout49,
		Qout50,Qout51,Qout52,Qout53,Qout54,Qout55,Qout56,Qout57,Qout58,Qout59,Qout60,Qout61,Qout62,Qout63,
		pc_5bits[5:0],IR);
endmodule

////////////////////////////////IM design ends

////////////////////////////////Register Memory Design Starts

module  Dff_regFile(input clk, input reset, input d1, input d2, input we1, input we2, output reg q);
	
	always@(negedge clk)
	begin
		if(reset)
			q = 1'b0;
		else
		begin
			if(we1)
				q = d1;
			else if(we2)
				q = d2;
		end
	end
	
endmodule

module register32bit_regFile(input clk, input reset, input [31:0] d1, input [31:0] d2, input we1, input we2, input [31:0] out);
	
	 Dff_regFile dff0(clk, reset, d1[0], d2[0], we1, we2, out[0]);
	 Dff_regFile dff1(clk, reset, d1[1], d2[1], we1, we2, out[1]);
	 Dff_regFile dff2(clk, reset, d1[2], d2[2], we1, we2, out[2]);
	 Dff_regFile dff3(clk, reset, d1[3], d2[3], we1, we2, out[3]);
	 Dff_regFile dff4(clk, reset, d1[4], d2[4], we1, we2, out[4]);
	 Dff_regFile dff5(clk, reset, d1[5], d2[5], we1, we2, out[5]);
	 Dff_regFile dff6(clk, reset, d1[6], d2[6], we1, we2, out[6]);
	 Dff_regFile dff7(clk, reset, d1[7], d2[7], we1, we2, out[7]);
	 Dff_regFile dff8(clk, reset, d1[8], d2[8], we1, we2, out[8]);
	 Dff_regFile dff9(clk, reset, d1[9], d2[9], we1, we2, out[9]);
	 Dff_regFile dff10(clk, reset, d1[10], d2[10], we1, we2, out[10]);
	 Dff_regFile dff11(clk, reset, d1[11], d2[11], we1, we2, out[11]);
	 Dff_regFile dff12(clk, reset, d1[12], d2[12], we1, we2, out[12]);
	 Dff_regFile dff13(clk, reset, d1[13], d2[13], we1, we2, out[13]);
	 Dff_regFile dff14(clk, reset, d1[14], d2[14], we1, we2, out[14]);
	 Dff_regFile dff15(clk, reset, d1[15], d2[15], we1, we2, out[15]);
	 Dff_regFile dff16(clk, reset, d1[16], d2[16], we1, we2, out[16]);
	 Dff_regFile dff17(clk, reset, d1[17], d2[17], we1, we2, out[17]);
	 Dff_regFile dff18(clk, reset, d1[18], d2[18], we1, we2, out[18]);
	 Dff_regFile dff19(clk, reset, d1[19], d2[19], we1, we2, out[19]);
	 Dff_regFile dff20(clk, reset, d1[20], d2[20], we1, we2, out[20]);
	 Dff_regFile dff21(clk, reset, d1[21], d2[21], we1, we2, out[21]);
	 Dff_regFile dff22(clk, reset, d1[22], d2[22], we1, we2, out[22]);
	 Dff_regFile dff23(clk, reset, d1[23], d2[23], we1, we2, out[23]);
	 Dff_regFile dff24(clk, reset, d1[24], d2[24], we1, we2, out[24]);
	 Dff_regFile dff25(clk, reset, d1[25], d2[25], we1, we2, out[25]);
	 Dff_regFile dff26(clk, reset, d1[26], d2[26], we1, we2, out[26]);
	 Dff_regFile dff27(clk, reset, d1[27], d2[27], we1, we2, out[27]);
	 Dff_regFile dff28(clk, reset, d1[28], d2[28], we1, we2, out[28]);
	 Dff_regFile dff29(clk, reset, d1[29], d2[29], we1, we2, out[29]);
	 Dff_regFile dff30(clk, reset, d1[30], d2[30], we1, we2, out[30]);
	 Dff_regFile dff31(clk, reset, d1[31], d2[31], we1, we2, out[31]);	
	
endmodule

module registerFile(input clk, input reset, input regWrite1, input regWrite2, input [31:0] writeData1, input [31:0] writeData2, 
							input [4:0] rs1, input [4:0] rt1, input [4:0] rs2, input [4:0] rd1, input [4:0] rd2,
								output [31:0] regRs1, output [31:0] regRt1, output [31:0] regRs2);
	
	wire [31:0] we1, we2, regOut0, regOut1, regOut2, regOut3, regOut4, regOut5, regOut6, regOut7, regOut8, regOut9, regOut10, regOut11, regOut12, regOut13, regOut14, regOut15, regOut16, regOut17, regOut18, regOut19, regOut20, regOut21, regOut22, regOut23, regOut24, regOut25, regOut26, regOut27, regOut28, regOut29, regOut30, regOut31;
	
	decoder5to32_32bits d1(rd1, we1);
	decoder5to32_32bits d2(rd2, we2);
	
	register32bit_regFile reg0(clk, reset, writeData1, writeData2, regWrite1 & we1[0], regWrite2 & we2[0], regOut0);
	register32bit_regFile reg1(clk, reset, writeData1, writeData2, regWrite1 & we1[1], regWrite2 & we2[1], regOut1);
	register32bit_regFile reg2(clk, reset, writeData1, writeData2, regWrite1 & we1[2], regWrite2 & we2[2], regOut2);
	register32bit_regFile reg3(clk, reset, writeData1, writeData2, regWrite1 & we1[3], regWrite2 & we2[3], regOut3);
	register32bit_regFile reg4(clk, reset, writeData1, writeData2, regWrite1 & we1[4], regWrite2 & we2[4], regOut4);
	register32bit_regFile reg5(clk, reset, writeData1, writeData2, regWrite1 & we1[5], regWrite2 & we2[5], regOut5);
	register32bit_regFile reg6(clk, reset, writeData1, writeData2, regWrite1 & we1[6], regWrite2 & we2[6], regOut6);
	register32bit_regFile reg7(clk, reset, writeData1, writeData2, regWrite1 & we1[7], regWrite2 & we2[7], regOut7);
	register32bit_regFile reg8(clk, reset, writeData1, writeData2, regWrite1 & we1[8], regWrite2 & we2[8], regOut8);
	register32bit_regFile reg9(clk, reset, writeData1, writeData2, regWrite1 & we1[9], regWrite2 & we2[9], regOut9);
	register32bit_regFile reg10(clk, reset, writeData1, writeData2, regWrite1 & we1[10], regWrite2 & we2[10], regOut10);
	register32bit_regFile reg11(clk, reset, writeData1, writeData2, regWrite1 & we1[11], regWrite2 & we2[11], regOut11);
	register32bit_regFile reg12(clk, reset, writeData1, writeData2, regWrite1 & we1[12], regWrite2 & we2[12], regOut12);
	register32bit_regFile reg13(clk, reset, writeData1, writeData2, regWrite1 & we1[13], regWrite2 & we2[13], regOut13);
	register32bit_regFile reg14(clk, reset, writeData1, writeData2, regWrite1 & we1[14], regWrite2 & we2[14], regOut14);
	register32bit_regFile reg15(clk, reset, writeData1, writeData2, regWrite1 & we1[15], regWrite2 & we2[15], regOut15);
	register32bit_regFile reg16(clk, reset, writeData1, writeData2, regWrite1 & we1[16], regWrite2 & we2[16], regOut16);
	register32bit_regFile reg17(clk, reset, writeData1, writeData2, regWrite1 & we1[17], regWrite2 & we2[17], regOut17);
	register32bit_regFile reg18(clk, reset, writeData1, writeData2, regWrite1 & we1[18], regWrite2 & we2[18], regOut18);
	register32bit_regFile reg19(clk, reset, writeData1, writeData2, regWrite1 & we1[19], regWrite2 & we2[19], regOut19);
	register32bit_regFile reg20(clk, reset, writeData1, writeData2, regWrite1 & we1[20], regWrite2 & we2[20], regOut20);
	register32bit_regFile reg21(clk, reset, writeData1, writeData2, regWrite1 & we1[21], regWrite2 & we2[21], regOut21);
	register32bit_regFile reg22(clk, reset, writeData1, writeData2, regWrite1 & we1[22], regWrite2 & we2[22], regOut22);
	register32bit_regFile reg23(clk, reset, writeData1, writeData2, regWrite1 & we1[23], regWrite2 & we2[23], regOut23);
	register32bit_regFile reg24(clk, reset, writeData1, writeData2, regWrite1 & we1[24], regWrite2 & we2[24], regOut24);
	register32bit_regFile reg25(clk, reset, writeData1, writeData2, regWrite1 & we1[25], regWrite2 & we2[25], regOut25);
	register32bit_regFile reg26(clk, reset, writeData1, writeData2, regWrite1 & we1[26], regWrite2 & we2[26], regOut26);
	register32bit_regFile reg27(clk, reset, writeData1, writeData2, regWrite1 & we1[27], regWrite2 & we2[27], regOut27);
	register32bit_regFile reg28(clk, reset, writeData1, writeData2, regWrite1 & we1[28], regWrite2 & we2[28], regOut28);
	register32bit_regFile reg29(clk, reset, writeData1, writeData2, regWrite1 & we1[29], regWrite2 & we2[29], regOut29);
	register32bit_regFile reg30(clk, reset, writeData1, writeData2, regWrite1 & we1[30], regWrite2 & we2[30], regOut30);
	register32bit_regFile reg31(clk, reset, writeData1, writeData2, regWrite1 & we1[31], regWrite2 & we2[31], regOut31);
	
	mux32to1_32bits regRs1Mux(regOut0, regOut1, regOut2, regOut3, regOut4, regOut5, regOut6, 
										regOut7, regOut8, regOut9, regOut10, regOut11, regOut12, regOut13, 
											regOut14, regOut15, regOut16, regOut17, regOut18, regOut19, regOut20, 
												regOut21, regOut22, regOut23, regOut24, regOut25, regOut26, regOut27, 
													regOut28, regOut29, regOut30, regOut31, rs1, regRs1);
	
	mux32to1_32bits regRt1Mux(regOut0, regOut1, regOut2, regOut3, regOut4, regOut5, regOut6, 
										regOut7, regOut8, regOut9, regOut10, regOut11, regOut12, regOut13, 
											regOut14, regOut15, regOut16, regOut17, regOut18, regOut19, regOut20, 
												regOut21, regOut22, regOut23, regOut24, regOut25, regOut26, regOut27, 
													regOut28, regOut29, regOut30, regOut31, rt1, regRt1);
													
	mux32to1_32bits regRs2Mux(regOut0, regOut1, regOut2, regOut3, regOut4, regOut5, regOut6, 
										regOut7, regOut8, regOut9, regOut10, regOut11, regOut12, regOut13, 
											regOut14, regOut15, regOut16, regOut17, regOut18, regOut19, regOut20, 
												regOut21, regOut22, regOut23, regOut24, regOut25, regOut26, regOut27, 
													regOut28, regOut29, regOut30, regOut31, rs2, regRs2);
	
endmodule

////////////////////////////////END Register File Design

module D_ff_reg(input clk, input reset, input regWrite, input d, output reg q);
	always @ (negedge clk)
	begin
	if(reset==1)
		q=0;
	else
		if(regWrite == 1)
		begin
			q=d;
		end
	end
endmodule

module Register_2bit(input clk, input reset, input regWrite, input [1:0] writeData, output [1:0] outR);
	
	D_ff_reg d0(clk, reset, regWrite,  writeData[0], outR[0]);
	D_ff_reg d1(clk, reset, regWrite,  writeData[1], outR[1]);
	
endmodule

module Register_3bit(input clk, input reset, input regWrite, input [2:0] writeData, output [2:0] outR);
	
	D_ff_reg d0(clk, reset, regWrite,  writeData[0], outR[0]);
	D_ff_reg d1(clk, reset, regWrite,  writeData[1], outR[1]);
	D_ff_reg d2(clk, reset, regWrite,  writeData[2], outR[2]);
	
endmodule


module Register_5bit(input clk, input reset, input regWrite, input [4:0] writeData, output [4:0] outR);
	
	D_ff_reg d0(clk, reset, regWrite,  writeData[0], outR[0]);
	D_ff_reg d1(clk, reset, regWrite,  writeData[1], outR[1]);
	D_ff_reg d2(clk, reset, regWrite,  writeData[2], outR[2]);
	D_ff_reg d3(clk, reset, regWrite,  writeData[3], outR[3]);
	D_ff_reg d4(clk, reset, regWrite,  writeData[4], outR[4]);
	
endmodule


module Register_16bit(input clk, input reset, input regWrite, input [15:0] writeData, output [15:0] outR);

	D_ff_reg d0(clk, reset, regWrite,  writeData[0], outR[0]);
	D_ff_reg d1(clk, reset, regWrite,  writeData[1], outR[1]);
	D_ff_reg d2(clk, reset, regWrite,  writeData[2], outR[2]);
	D_ff_reg d3(clk, reset, regWrite,  writeData[3], outR[3]);
	D_ff_reg d4(clk, reset, regWrite,  writeData[4], outR[4]);
	D_ff_reg d5(clk, reset, regWrite,  writeData[5], outR[5]);
	D_ff_reg d6(clk, reset, regWrite,  writeData[6], outR[6]);
	D_ff_reg d7(clk, reset, regWrite,  writeData[7], outR[7]);
	D_ff_reg d8(clk, reset, regWrite,  writeData[8], outR[8]);
	D_ff_reg d9(clk, reset, regWrite,  writeData[9], outR[9]);
	D_ff_reg d10(clk, reset, regWrite, writeData[10], outR[10]);
	D_ff_reg d11(clk, reset, regWrite, writeData[11], outR[11]);
	D_ff_reg d12(clk, reset, regWrite, writeData[12], outR[12]);
	D_ff_reg d13(clk, reset, regWrite, writeData[13], outR[13]);
	D_ff_reg d14(clk, reset, regWrite, writeData[14], outR[14]);
	D_ff_reg d15(clk, reset, regWrite, writeData[15], outR[15]);
	
endmodule


module Register_32bit(input clk, input reset, input regWrite, input [31:0] writeData, output  [31:0] outR);

	D_ff_reg d0(clk, reset, regWrite,  writeData[0], outR[0]);
	D_ff_reg d1(clk, reset, regWrite,  writeData[1], outR[1]);
	D_ff_reg d2(clk, reset, regWrite,  writeData[2], outR[2]);
	D_ff_reg d3(clk, reset, regWrite,  writeData[3], outR[3]);
	D_ff_reg d4(clk, reset, regWrite,  writeData[4], outR[4]);
	D_ff_reg d5(clk, reset, regWrite,  writeData[5], outR[5]);
	D_ff_reg d6(clk, reset, regWrite,  writeData[6], outR[6]);
	D_ff_reg d7(clk, reset, regWrite,  writeData[7], outR[7]);
	D_ff_reg d8(clk, reset, regWrite,  writeData[8], outR[8]);
	D_ff_reg d9(clk, reset, regWrite,  writeData[9], outR[9]);
	D_ff_reg d10(clk, reset, regWrite, writeData[10], outR[10]);
	D_ff_reg d11(clk, reset, regWrite, writeData[11], outR[11]);
	D_ff_reg d12(clk, reset, regWrite, writeData[12], outR[12]);
	D_ff_reg d13(clk, reset, regWrite, writeData[13], outR[13]);
	D_ff_reg d14(clk, reset, regWrite, writeData[14], outR[14]);
	D_ff_reg d15(clk, reset, regWrite, writeData[15], outR[15]);
	D_ff_reg d16(clk, reset, regWrite, writeData[16], outR[16]);
	D_ff_reg d17(clk, reset, regWrite, writeData[17], outR[17]);
	D_ff_reg d18(clk, reset, regWrite, writeData[18], outR[18]);
	D_ff_reg d19(clk, reset, regWrite, writeData[19], outR[19]);
	D_ff_reg d20(clk, reset, regWrite, writeData[20], outR[20]);
	D_ff_reg d21(clk, reset, regWrite, writeData[21], outR[21]);
	D_ff_reg d22(clk, reset, regWrite, writeData[22], outR[22]);
	D_ff_reg d23(clk, reset, regWrite, writeData[23], outR[23]);
	D_ff_reg d24(clk, reset, regWrite, writeData[24], outR[24]);
	D_ff_reg d25(clk, reset, regWrite, writeData[25], outR[25]);
	D_ff_reg d26(clk, reset, regWrite, writeData[26], outR[26]);
	D_ff_reg d27(clk, reset, regWrite, writeData[27], outR[27]);
	D_ff_reg d28(clk, reset, regWrite, writeData[28], outR[28]);
	D_ff_reg d29(clk, reset, regWrite, writeData[29], outR[29]);
	D_ff_reg d30(clk, reset, regWrite, writeData[30], outR[30]);
	D_ff_reg d31(clk, reset, regWrite, writeData[31], outR[31]);
	
endmodule

////////////////////////////////Sign/Zero Extend
//srli
module zeroExt5to32(input [4:0] in, output reg [31:0] zeroExtin);
	always@(in)
		zeroExtin = {27'b0, in};
endmodule

//xor
//addi_slti_sw
module signExt12to32(input [11:0] in, output reg [31:0] signExtin);
	always@(in)
		signExtin={{20{in[11]}},in};
endmodule

//jalr
module signExt13to32(input [12:0] in, output reg [31:0] signExtin);
	always@(in)
		signExtin={{19{in[12]}},in};
endmodule

//c_lui
module signExt18to32(input [17:0] in, output reg [31:0] signExtin);
	always@(in)
		signExtin={{14{in[17]}},in};
endmodule

//c_lw
//c_bnez
module signExt9to32(input [8:0] in, output reg [31:0] signExtin);
	always@(in)
		signExtin={{23{in[8]}},in};
endmodule

////////////////////////////////End Sign/Zero Extend


////////////////////////////////MUX / DECODERS


module mux2to1_32bits(input [31:0] in0, input [31:0] in1, input sel, output reg [31:0] muxOut);
    always@(in0, in1, sel)
		begin
			case(sel)
				1'b0: muxOut=in0;
				1'b1: muxOut=in1;
			endcase
		end
endmodule

module mux2to1_14bits(input [13:0] in0, input [13:0] in1, input sel, output reg [13:0] muxOut);
	always@(in0, in1, sel)
		begin
			case(sel)
				1'b0: muxOut=in0;
				1'b1: muxOut=in1;
			endcase
		end
endmodule

module mux2to1_5bits(input [4:0] in0, input [4:0] in1, input sel, output reg [4:0] muxOut);
    always@(in0, in1, sel)
		begin
			case(sel)
				1'b0: muxOut=in0;
				1'b1: muxOut=in1;
			endcase
		end
endmodule

module mux2to1_32x16bits(input [511:0] in0, input [511:0] in1, input sel, output reg [511:0] muxOut);
    always@(in0, in1, sel)
		begin
			case(sel)
				1'b0: muxOut=in0;
				1'b1: muxOut=in1;
			endcase
		end
endmodule

module mux4to1_32bits(input [31:0] in0, input [31:0] in1, input [31:0] in2, input [31:0] in3, input [1:0] sel, output reg [31:0] muxOut);
	always@(in0, in1, in2, in3, sel)
		begin
			case(sel)
				2'b00: muxOut = in0;
				2'b01: muxOut = in1;
				2'b10: muxOut = in2;
				2'b11: muxOut = in3;
			endcase
		end
endmodule

module mux8to1_32bits(input [31:0] in0, in1, in2, in3, in4, in5, in6, in7, input [2:0] sel, output reg [31:0] muxOut);
	always@(in0, in1, in2, in3, in4, in5, in6, in7, sel)
		begin
			case(sel)
				3'b000: muxOut = in0;
				3'b001: muxOut = in1;
				3'b010: muxOut = in2;
				3'b011: muxOut = in3;
				3'b100: muxOut = in4;
				3'b101: muxOut = in5;
				3'b110: muxOut = in6;
				3'b111: muxOut = in7;
			endcase
		end
endmodule

module mux32to1_32bits(input [31:0] in0, in1, in2, in3, in4, in5, in6, in7, in8, in9, in10, in11, in12, in13, in14, in15, in16, in17, in18, in19, in20, in21, in22, in23, in24, in25, in26, in27, in28, in29, in30, in31, input [4:0] sel, output reg [31:0] muxOut);
	always@(in0, in1, in2, in3, in4, in5, in6, in7, in8, in9, in10, in11, in12, in13, in14, in15, in16, in17, in18, in19, in20, in21, in22, in23, in24, in25, in26, in27, in28, in29, in30, in31, sel)
		begin
			case(sel)
				5'b00000: muxOut = in0;
				5'b00001: muxOut = in1;
				5'b00010: muxOut = in2;
				5'b00011: muxOut = in3;
				5'b00100: muxOut = in4;
				5'b00101: muxOut = in5;
				5'b00110: muxOut = in6;
				5'b00111: muxOut = in7;
				5'b01000: muxOut = in8;
				5'b01001: muxOut = in9;
				5'b01010: muxOut = in10;
				5'b01011: muxOut = in11;
				5'b01100: muxOut = in12;
				5'b01101: muxOut = in13;
				5'b01110: muxOut = in14;
				5'b01111: muxOut = in15;
				5'b10000: muxOut = in16;
				5'b10001: muxOut = in17;
				5'b10010: muxOut = in18;
				5'b10011: muxOut = in19;
				5'b10100: muxOut = in20;
				5'b10101: muxOut = in21;
				5'b10110: muxOut = in22;
				5'b10111: muxOut = in23;
				5'b11000: muxOut = in24;
				5'b11001: muxOut = in25;
				5'b11010: muxOut = in26;
				5'b11011: muxOut = in27;
				5'b11100: muxOut = in28;
				5'b11101: muxOut = in29;
				5'b11110: muxOut = in30;
				5'b11111: muxOut = in31;
			endcase
		end
endmodule

module mux32to1_DM(input [15:0] outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15,
	outR16,outR17,outR18,outR19,outR20,outR21,outR22,outR23,outR24,outR25,outR26,outR27,outR28,outR29,outR30,outR31,
	input [4:0] Sel, output reg [31:0] outBus );
	
	always@(outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15,
		outR16,outR17,outR18,outR19,outR20,outR21,outR22,outR23,outR24,outR25,outR26,outR27,outR28,outR29,outR30,outR31,
		Sel)
		case (Sel)
			5'd0: outBus = {outR1,outR0};
			5'd1: outBus = {outR2,outR1};
			5'd2: outBus = {outR3,outR2};
			5'd3: outBus = {outR4,outR3};
			5'd4: outBus = {outR5,outR4};
			5'd5: outBus = {outR6,outR5};
			5'd6: outBus = {outR7,outR6};
			5'd7: outBus = {outR8,outR7};
			5'd8: outBus = {outR9,outR8};
			5'd9: outBus = {outR10,outR9};
			5'd10: outBus = {outR11,outR10};
			5'd11: outBus = {outR12,outR11};
			5'd12: outBus = {outR13,outR12};
			5'd13: outBus = {outR14,outR13};
			5'd14: outBus = {outR15,outR14};
			5'd15: outBus = {outR16,outR15};
			5'd16: outBus = {outR17,outR16};
			5'd17: outBus = {outR18,outR17};
			5'd18: outBus = {outR19,outR18};
			5'd19: outBus = {outR20,outR19};
			5'd20: outBus = {outR21,outR20};
			5'd21: outBus = {outR22,outR21};
			5'd22: outBus = {outR23,outR22};
			5'd23: outBus = {outR24,outR23};
			5'd24: outBus = {outR25,outR24};
			5'd25: outBus = {outR26,outR25};
			5'd26: outBus = {outR27,outR26};
			5'd27: outBus = {outR28,outR27};
			5'd28: outBus = {outR29,outR28};
			5'd29: outBus = {outR30,outR29};
			5'd30: outBus = {outR31,outR30};
			5'd31: outBus = {32'b0,outR31};
		endcase
endmodule

module decoder5to32_32bits(input [4:0] destReg, output reg [31:0] decOut);
	always @(destReg)
		case(destReg)
			5'd0: decOut = 32'b0000_0000_0000_0000_0000_0000_0000_0001;
			5'd1: decOut = 32'b0000_0000_0000_0000_0000_0000_0000_0010;
			5'd2: decOut = 32'b0000_0000_0000_0000_0000_0000_0000_0100;
			5'd3: decOut = 32'b0000_0000_0000_0000_0000_0000_0000_1000;
			5'd4: decOut = 32'b0000_0000_0000_0000_0000_0000_0001_0000;
			5'd5: decOut = 32'b0000_0000_0000_0000_0000_0000_0010_0000;
			5'd6: decOut = 32'b0000_0000_0000_0000_0000_0000_0100_0000;
			5'd7: decOut = 32'b0000_0000_0000_0000_0000_0000_1000_0000;
			5'd8: decOut = 32'b0000_0000_0000_0000_0000_0001_0000_0000;
			5'd9: decOut = 32'b0000_0000_0000_0000_0000_0010_0000_0000;
			5'd10: decOut = 32'b0000_0000_0000_0000_0000_0100_0000_0000;
			5'd11: decOut = 32'b0000_0000_0000_0000_0000_1000_0000_0000;
			5'd12: decOut = 32'b0000_0000_0000_0000_0001_0000_0000_0000;
			5'd13: decOut = 32'b0000_0000_0000_0000_0010_0000_0000_0000;
			5'd14: decOut = 32'b0000_0000_0000_0000_0100_0000_0000_0000;
			5'd15: decOut = 32'b0000_0000_0000_0000_1000_0000_0000_0000;
			5'd16: decOut = 32'b0000_0000_0000_0001_0000_0000_0000_0000;
			5'd17: decOut = 32'b0000_0000_0000_0010_0000_0000_0000_0000;
			5'd18: decOut = 32'b0000_0000_0000_0100_0000_0000_0000_0000;
			5'd19: decOut = 32'b0000_0000_0000_1000_0000_0000_0000_0000;
			5'd20: decOut = 32'b0000_0000_0001_0000_0000_0000_0000_0000;
			5'd21: decOut = 32'b0000_0000_0010_0000_0000_0000_0000_0000;
			5'd22: decOut = 32'b0000_0000_0100_0000_0000_0000_0000_0000;
			5'd23: decOut = 32'b0000_0000_1000_0000_0000_0000_0000_0000;
			5'd24: decOut = 32'b0000_0001_0000_0000_0000_0000_0000_0000;
			5'd25: decOut = 32'b0000_0010_0000_0000_0000_0000_0000_0000;
			5'd26: decOut = 32'b0000_0100_0000_0000_0000_0000_0000_0000;
			5'd27: decOut = 32'b0000_1000_0000_0000_0000_0000_0000_0000;
			5'd28: decOut = 32'b0001_0000_0000_0000_0000_0000_0000_0000;
			5'd29: decOut = 32'b0010_0000_0000_0000_0000_0000_0000_0000;
			5'd30: decOut = 32'b0100_0000_0000_0000_0000_0000_0000_0000;
			5'd31: decOut = 32'b1000_0000_0000_0000_0000_0000_0000_0000;
		endcase
endmodule


////////////////////////////////END MUX / DECODERS


// instruction
// PC Value
module IF_ID(input clk,input reset,input regWrite,input [47:0] instruction, input [31:0] PC, output [31:0] p0_instruction, output [15:0] c_p0_instruction, output [31:0] p0_PC);

	Register_32bit r1(clk,reset, regWrite, instruction[31:0], p0_instruction);
	Register_16bit r2(clk, reset, regWrite, instruction[47:32], c_p0_instruction);
	Register_32bit r3(clk,reset,regWrite,PC,p0_PC);

endmodule

//Wb
//Mem
//Ex
//PC+3
//rd
//c_rd(a)
//c_rd(b)
//reg(rs1)
//reg(rs2)
//reg(c_rs1)
//shamt
//addi/jalr(IMM)
//sw(IMM)
//c_lui(IMM)
//c_bnez(IMM)
//c_lw(IMM)
//regWrite1
//regWrite2

//output reg regWrite1, output reg regWrite2, output reg [2:0] aluSrcB, 
//output reg regDest, output reg [2:0] aluOp, output reg memWrite, output reg memToReg1, 
//output reg memToReg2, output reg dest, output reg branch

//aluOp, branch, branch_taken, aluSrcB, regDest //EX

//memWrite, dest //MEM

//memToReg1, memToReg2, , regWrite1, regWrite2 //WB

module ID_EX(input clk, input reset,input regWrite,
				input [2:0] aluOp,
				input [1:0] aluSrcB,
				input regDest,
				input memWrite,
				input dest,
				input memToReg1,
				input memToReg2,
				input regWrite1,
				input regWrite2,
 				input [4:0] rd,
				input [4:0] c_rda,
				input [4:0] c_rdb,
				input [31:0] reg_rs1,
				input [31:0] reg_rs2,
				input [31:0] reg_c_rs1,
				input [31:0] shamt,
				input [31:0] addi_jalr_imm,
				input [31:0] sw_imm,
				input [31:0] c_lui_imm,
				input [31:0] c_lw_imm,
				input [31:0] PC,
				input [4:0] rs1,
				input [4:0] rs2,
				input [4:0] c_rs1,
				input memRd,
				output [2:0] out_aluOp,
				output [1:0] out_aluSrcB,
				output out_regDest,
				output out_memWrite,
				output out_dest,
				output out_memToReg1,
				output out_memToReg2,
				output out_regWrite1,
				output out_regWrite2,
				output [4:0] out_rd,
				output [4:0] out_c_rda,
				output [4:0] out_c_rdb,
				output [31:0] out_reg_rs1,
				output [31:0] out_reg_rs2,
				output [31:0] out_reg_c_rs1,
				output [31:0] out_shamt,
				output [31:0] out_addi_jalr_imm,
				output [31:0] out_sw_imm,
				output [31:0] out_c_lui_imm,
				output [31:0] out_c_lw_imm,
				output [31:0] out_PC,
				output [4:0] out_rs1,
				output [4:0] out_rs2,
				output [4:0] out_c_rs1,
				output out_memRd);


	//Register_32bit(input clk, input reset, input regWrite, input [31:0] writeData, output  [31:0] outR);
	//Wb
	//Mem
	//Execute
	Register_3bit reg_aluOp(clk, reset, regWrite, aluOp, out_aluOp);
	Register_2bit reg_aluSrcB(clk, reset, regWrite, aluSrcB, out_aluSrcB);
	D_ff_reg reg_regDest(clk,reset,regWrite,regDest,out_regDest);
	D_ff_reg reg_memWrite(clk,reset,regWrite,memWrite,out_memWrite);
	D_ff_reg reg_dest(clk,reset,regWrite,dest,out_dest);
	D_ff_reg reg_memToReg1(clk,reset,regWrite,memToReg1,out_memToReg1);
	D_ff_reg reg_memToReg2(clk,reset,regWrite,memToReg2,out_memToReg2);
	D_ff_reg reg_regWrite1(clk,reset,regWrite,regWrite1,out_regWrite1);
	D_ff_reg reg_regWrite2(clk,reset,regWrite,regWrite2,out_regWrite2);
	Register_5bit reg_rd(clk,reset,regWrite,rd,out_rd);
	Register_5bit reg_c_rda(clk,reset,regWrite,c_rda,out_c_rda);
	Register_5bit reg_c_rdb(clk,reset,regWrite,c_rdb,out_c_rdb);
	Register_32bit reg_reg_rs1(clk,reset,regWrite,reg_rs1,out_reg_rs1);
	Register_32bit reg_reg_rs2(clk,reset,regWrite,reg_rs2,out_reg_rs2);
	Register_32bit reg_reg_c_rs1(clk,reset,regWrite,reg_c_rs1,out_reg_c_rs1);
	Register_32bit reg_shamt(clk,reset,regWrite,shamt,out_shamt);
	Register_32bit reg_addi_jalr_imm(clk,reset,regWrite,addi_jalr_imm,out_addi_jalr_imm);
	Register_32bit reg_sw_imm(clk,reset,regWrite,sw_imm,out_sw_imm);
	Register_32bit reg_c_lui_imm(clk,reset,regWrite,c_lui_imm,out_c_lui_imm);
	Register_32bit reg_c_lw_imm(clk,reset,regWrite,c_lw_imm,out_c_lw_imm);
	Register_32bit reg_PC(clk,reset,regWrite,PC,out_PC);
	Register_5bit reg_rs1_reg(clk,reset,regWrite,rs1,out_rs1);
	Register_5bit reg_rs2_reg(clk,reset,regWrite,rs2,out_rs2);
	Register_5bit reg_c_rda_reg(clk,reset,regWrite,c_rs1,out_c_rs1);
	D_ff_reg reg_memRd(clk,reset,regWrite,memRd,out_memRd);

endmodule


//Wb
//Execute
//PC+3
//rd
//c_rd
//Alu_A_out
//Alu_B_out
//reg[rs2]
//regWrite1
//regWrite2

module EX_MEM(input clk,input reset,input regWrite,
				//input wb,
				//input mem,
				input memRd,
				input memWrite,
				input dest,
				input memToReg1,
				input memToReg2,
				input regWrite1,
				input regWrite2,
				input [4:0] rd,
				input [4:0] c_rd,
				input [31:0] reg_rs2,
				input [31:0] alu_a_out,
				input [31:0] alu_b_out,
				input [31:0] c_lui_imm,
				input [31:0] PC,
				output out_memRd,
				output out_memWrite,
				output out_dest,
				output out_memToReg1,
				output out_memToReg2,
				output out_regWrite1,
				output out_regWrite2,
				output [4:0] out_rd,
				output [4:0] out_c_rd,
				output [31:0] out_reg_rs2,
				output [31:0] out_alu_a_out,
				output [31:0] out_alu_b_out,
				output [31:0] out_c_lui_imm,
				output [31:0] out_PC);

	//module D_ff_reg(input clk, input reset, input regWrite, input d, output reg q);
	D_ff_reg reg_memRd(clk,reset,regWrite,memRd,out_memRd);
	D_ff_reg reg_memWrite(clk,reset,regWrite,memWrite,out_memWrite);
	D_ff_reg reg_dest(clk,reset,regWrite,dest,out_dest);
	D_ff_reg reg_memToReg1(clk,reset,regWrite,memToReg1,out_memToReg1);
	D_ff_reg reg_memToReg2(clk,reset,regWrite,memToReg2,out_memToReg2);
	D_ff_reg reg_regWrite1(clk,reset,regWrite,regWrite1,out_regWrite1);
	D_ff_reg reg_regWrite2(clk,reset,regWrite,regWrite2,out_regWrite2);
	Register_5bit reg_rd(clk,reset,regWrite,rd,out_rd);
	Register_5bit reg_c_rd(clk,reset,regWrite,c_rd,out_c_rd);
	Register_32bit reg_reg_rs2(clk,reset,regWrite,reg_rs2,out_reg_rs2);
	Register_32bit reg_alu_a_out(clk,reset,regWrite,alu_a_out,out_alu_a_out);
	Register_32bit reg_alu_b_out(clk,reset,regWrite,alu_b_out,out_alu_b_out);
	Register_32bit reg_c_lui_imm(clk, reset, regWrite, c_lui_imm, out_c_lui_imm);
	Register_32bit reg_PC(clk, reset, regWrite, PC, out_PC);

endmodule

//wb
//rd
//c_rd
//alu_a_out
//alu_b_out
//dm_out
//c_dm_out
//regWrite1
//regWrite2
module MEM_WB(input clk,input reset,input regWrite,
				//input wb,
				input [4:0] rd,
				input [4:0] c_rd,
				input [31:0] alu_a_out,
				input [31:0] c_lui_imm,
				input [31:0] c_dm_out,
				input memToReg2,
				input regWrite1,
				input regWrite2,
				output [4:0] out_rd,
				output [4:0] out_c_rd,
				output [31:0] out_alu_a_out,
				output [31:0] out_c_lui_imm,
				output [31:0] out_c_dm_out,
				output out_memToReg2,
				output out_regWrite1,
				output out_regWrite2);

	D_ff_reg reg_memToReg2(clk,reset,regWrite,memToReg2,out_memToReg2);
	D_ff_reg reg_regWrite1(clk,reset,regWrite,regWrite1,out_regWrite1);
	D_ff_reg reg_regWrite2(clk,reset,regWrite,regWrite2,out_regWrite2);
	Register_5bit reg_rd(clk,reset,regWrite,rd,out_rd);
	Register_5bit reg_c_rd(clk,reset,regWrite,c_rd,out_c_rd);
	Register_32bit reg_alu_a_out(clk,reset,regWrite,alu_a_out,out_alu_a_out);
	Register_32bit reg_c_lui_imm(clk,reset,regWrite,c_lui_imm,out_c_lui_imm);
	Register_32bit reg_c_dm_out(clk,reset,regWrite,c_dm_out,out_c_dm_out);

endmodule

module WB(input clk,input reset, input regWrite,
			input [4:0] rd,
			input [4:0] c_rd,
			input [31:0] reg_rd,
			input [31:0] c_reg_rd,
			input regWrite1,
			input regWrite2,
			output [4:0] out_rd,
			output [4:0] out_c_rd,
			output [31:0] out_reg_rd,
			output [31:0] out_c_reg_rd,
			output out_regWrite1,
			output out_regWrite2);

	D_ff_reg reg_regWrite1(clk,reset,regWrite,regWrite1,out_regWrite1);
	D_ff_reg reg_regWrite2(clk,reset,regWrite,regWrite2,out_regWrite2);
	Register_5bit reg_reg_rd(clk,reset,regWrite,rd,out_rd);
	Register_5bit reg_reg_c_rd(clk,reset,regWrite,c_rd,out_c_rd);
	Register_32bit reg_reg_rd1(clk,reset,regWrite,reg_rd,out_reg_rd);
	Register_32bit reg_c_reg_rd(clk,reset,regWrite,c_reg_rd,out_c_reg_rd);

endmodule



module DM(input clk,input reset,input [4:0] address,input [31:0] data,input MemWrite,output [31:0] dm_output);

	wire [31:0] dm_decout;
	wire[511:0] mux_out;
	wire [15:0] q_out_0,q_out_1,q_out_2,q_out_3,q_out_4,q_out_5,q_out_6,q_out_7,q_out_8,q_out_9, q_out_10,q_out_11,q_out_12,q_out_13,q_out_14,q_out_15,q_out_16,q_out_17,q_out_18,q_out_19,
 	q_out_20,q_out_21,q_out_22,q_out_23,q_out_24,q_out_25,q_out_26,q_out_27,q_out_28,q_out_29,
  	q_out_30,q_out_31;

	decoder5to32_32bits d_dm(address,dm_decout);
	mux2to1_32x16bits dm_mux0({16{data}},{16'b0,{15{data}},data[15:0]},address[0],mux_out);
	

	Register_16bit r0(clk, reset, dm_decout[0] & MemWrite, mux_out[15:0], q_out_0);	
	Register_16bit r1(clk, reset, (dm_decout[1] | dm_decout[0]) & MemWrite, mux_out[31:16], q_out_1);
	Register_16bit r2(clk, reset, (dm_decout[2] | dm_decout[1]) & MemWrite, mux_out[47:32], q_out_2);
	Register_16bit r3(clk, reset, (dm_decout[3] | dm_decout[2]) & MemWrite, mux_out[63:48], q_out_3);
	Register_16bit r4(clk, reset, (dm_decout[4] | dm_decout[3]) & MemWrite, mux_out[79:64], q_out_4);
	Register_16bit r5(clk, reset, (dm_decout[5] | dm_decout[4]) & MemWrite, mux_out[95:80], q_out_5);
	Register_16bit r6(clk, reset, (dm_decout[6] | dm_decout[5]) & MemWrite, mux_out[111:96], q_out_6);
	Register_16bit r7(clk, reset, (dm_decout[7] | dm_decout[6]) & MemWrite, mux_out[127:112], q_out_7);
	Register_16bit r8(clk, reset, (dm_decout[8] | dm_decout[7]) & MemWrite, mux_out[143:128], q_out_8);
	Register_16bit r9(clk, reset, (dm_decout[9] | dm_decout[8]) & MemWrite, mux_out[159:144], q_out_9);
	Register_16bit r10(clk, reset, (dm_decout[10] | dm_decout[9]) & MemWrite, mux_out[175:160], q_out_10);
	Register_16bit r11(clk, reset, (dm_decout[11] | dm_decout[10]) & MemWrite, mux_out[191:176], q_out_11);
	Register_16bit r12(clk, reset, (dm_decout[12] | dm_decout[11]) & MemWrite, mux_out[207:192], q_out_12);
	Register_16bit r13(clk, reset, (dm_decout[13] | dm_decout[12]) & MemWrite, mux_out[223:208], q_out_13);
	Register_16bit r14(clk, reset, (dm_decout[14] | dm_decout[13]) & MemWrite, mux_out[239:224], q_out_14);
	Register_16bit r15(clk, reset, (dm_decout[15] | dm_decout[14]) & MemWrite, mux_out[255:240], q_out_15);
	Register_16bit r16(clk, reset, (dm_decout[16] | dm_decout[15]) & MemWrite, mux_out[271:256], q_out_16);
	Register_16bit r17(clk, reset, (dm_decout[17] | dm_decout[16]) & MemWrite, mux_out[287:272], q_out_17);
	Register_16bit r18(clk, reset, (dm_decout[18] | dm_decout[17]) & MemWrite, mux_out[303:288], q_out_18);
	Register_16bit r19(clk, reset, (dm_decout[19] | dm_decout[18]) & MemWrite, mux_out[319:304], q_out_19);
	Register_16bit r20(clk, reset, (dm_decout[20] | dm_decout[19]) & MemWrite, mux_out[335:320], q_out_20);
	Register_16bit r21(clk, reset, (dm_decout[21] | dm_decout[20]) & MemWrite, mux_out[351:336], q_out_21);
	Register_16bit r22(clk, reset, (dm_decout[22] | dm_decout[21]) & MemWrite, mux_out[367:352], q_out_22);
	Register_16bit r23(clk, reset, (dm_decout[23] | dm_decout[22]) & MemWrite, mux_out[383:368], q_out_23);
	Register_16bit r24(clk, reset, (dm_decout[24] | dm_decout[23]) & MemWrite, mux_out[399:384], q_out_24);
	Register_16bit r25(clk, reset, (dm_decout[25] | dm_decout[24]) & MemWrite, mux_out[415:400], q_out_25);
	Register_16bit r26(clk, reset, (dm_decout[26] | dm_decout[25]) & MemWrite, mux_out[431:416], q_out_26);
	Register_16bit r27(clk, reset, (dm_decout[27] | dm_decout[26]) & MemWrite, mux_out[447:432], q_out_27);
	Register_16bit r28(clk, reset, (dm_decout[28] | dm_decout[27]) & MemWrite, mux_out[463:448], q_out_28);
	Register_16bit r29(clk, reset, (dm_decout[29] | dm_decout[28]) & MemWrite, mux_out[479:464], q_out_29);
	Register_16bit r30(clk, reset, (dm_decout[30] | dm_decout[29]) & MemWrite, mux_out[495:480], q_out_30);
	Register_16bit r31(clk, reset, (dm_decout[31] | dm_decout[30]) & MemWrite, mux_out[511:496], q_out_31);


	mux32to1_DM dm_mux1 (q_out_0,q_out_1,q_out_2,q_out_3,q_out_4,q_out_5,q_out_6,q_out_7,q_out_8,q_out_9, q_out_10,q_out_11,q_out_12,q_out_13,q_out_14,q_out_15,q_out_16,q_out_17,q_out_18,q_out_19,
 	q_out_20,q_out_21,q_out_22,q_out_23,q_out_24,q_out_25,q_out_26,q_out_27,q_out_28,q_out_29,
  	q_out_30,q_out_31,address, dm_output );	

endmodule 



module forwarding_unit_A(input clk ,input reset,
							input [4:0] ID_EX_rs1,
							input [4:0] ID_EX_rs2,
							input [4:0] ID_EX_c_rs1,
							input [4:0] EX_MEM_rd,
							input [4:0] EX_MEM_c_rd,
							input [4:0] MEM_WB_rd,
							input [4:0] MEM_WB_c_rd,
							input [4:0] WB_rd,
							input [4:0] WB_c_rd,
							input EX_MEM_regWrite1,
							input EX_MEM_regWrite2,
							input MEM_WB_regWrite1,
							input MEM_WB_regWrite2,
							input WB_regWrite1,
							input WB_regWrite2, 
							output reg [2:0] forward_A,
							output reg [2:0] forward_B,
							output reg [2:0] forward_c_A);
						


	always @ (clk ,reset, EX_MEM_regWrite1, EX_MEM_regWrite2, MEM_WB_regWrite1, MEM_WB_regWrite2, WB_regWrite1, WB_regWrite2, ID_EX_rs1, ID_EX_rs2, ID_EX_c_rs1, EX_MEM_rd, EX_MEM_c_rd, MEM_WB_rd, MEM_WB_c_rd, WB_rd, WB_c_rd )


		begin
			forward_A = 3'b000;
			forward_B = 3'b000;
			forward_c_A=3'b000;


			if(EX_MEM_regWrite1==1'b1 && EX_MEM_rd == ID_EX_rs1)
				forward_A=3'b001;
			else if(EX_MEM_regWrite2==1'b1 && EX_MEM_c_rd == ID_EX_rs1)
				forward_A=3'b010;
			else if(MEM_WB_regWrite1==1'b1 && MEM_WB_rd == ID_EX_rs1)
				forward_A=3'b011;
			else if(MEM_WB_regWrite2==1'b1 && MEM_WB_c_rd == ID_EX_rs1)
				forward_A=3'b100;
			else if(WB_regWrite1==1'b1 && WB_rd == ID_EX_rs1)
				forward_A=3'b101;
			else if(WB_regWrite2==1'b1 && WB_c_rd == ID_EX_rs1)
				forward_A=3'b110;

			if(EX_MEM_regWrite1==1'b1 && EX_MEM_rd == ID_EX_rs2)
				forward_B=3'b001;
			else if(EX_MEM_regWrite2==1'b1 && EX_MEM_c_rd == ID_EX_rs2)
				forward_B=3'b010;
			else if(MEM_WB_regWrite1==1'b1 && MEM_WB_rd == ID_EX_rs2)
				forward_B=3'b011;
			else if(MEM_WB_regWrite2==1'b1 && MEM_WB_c_rd == ID_EX_rs2)
				forward_B=3'b100;
			else if(WB_regWrite1==1'b1 && WB_rd == ID_EX_rs2)
				forward_B=3'b101;
			else if(WB_regWrite2==1'b1 && WB_c_rd == ID_EX_rs2)
				forward_B=3'b110;

			if(EX_MEM_regWrite1==1'b1 && EX_MEM_rd==ID_EX_c_rs1)
				forward_c_A=3'b001;
			else if(EX_MEM_regWrite2==1'b1 && EX_MEM_c_rd==ID_EX_c_rs1)
				forward_c_A=3'b010;	
			else if(MEM_WB_regWrite1==1'b1 && MEM_WB_rd == ID_EX_c_rs1)
				forward_c_A=3'b011;
			else if(MEM_WB_regWrite2==1'b1 && MEM_WB_c_rd == ID_EX_c_rs1)
				forward_c_A=3'b100;
			else if(WB_regWrite1==1'b1 && WB_rd == ID_EX_c_rs1)
				forward_c_A=3'b101;
			else if(WB_regWrite2==1'b1 && WB_c_rd == ID_EX_c_rs1)
				forward_c_A=3'b110;

		end

endmodule
			


module forwarding_unit_B(input clk ,input reset,
							input [4:0] IF_ID_rs1,
							input [4:0] IF_ID_c_rs1,
							input [4:0] EX_MEM_rd,
							input [4:0] EX_MEM_c_rd,
							input [4:0] MEM_WB_rd,
							input [4:0] MEM_WB_c_rd,
							input [4:0] WB_rd,
							input [4:0] WB_c_rd,
							input EX_MEM_regWrite1,
							input EX_MEM_regWrite2,
							input MEM_WB_regWrite1,
							input MEM_WB_regWrite2,
							input WB_regWrite1,
							input WB_regWrite2,
							output reg [2:0] j_forward,
							output reg [2:0] b_forward);


	always @ ( clk, reset, IF_ID_rs1, IF_ID_c_rs1, EX_MEM_rd, EX_MEM_c_rd, MEM_WB_rd, MEM_WB_c_rd, WB_rd, WB_c_rd, EX_MEM_regWrite1, EX_MEM_regWrite2, MEM_WB_regWrite1, MEM_WB_regWrite2, WB_regWrite1, WB_regWrite2)

		begin

			j_forward = 3'b000;
			b_forward = 3'b000;

			if( EX_MEM_regWrite1 == 1'b1 && EX_MEM_rd == IF_ID_rs1)
				j_forward = 3'b001;
			else if(EX_MEM_regWrite2 == 1'b1 && EX_MEM_c_rd == IF_ID_rs1)
				j_forward = 3'b010;
			else if ( MEM_WB_regWrite1 == 1'b1 && MEM_WB_rd == IF_ID_rs1)
				j_forward = 3'b011;
			else if ( MEM_WB_regWrite2 == 1'b1 && MEM_WB_c_rd == IF_ID_rs1)
				j_forward = 3'b100;
			else if ( WB_regWrite1 == 1'b1 && WB_rd == IF_ID_rs1)
				j_forward = 3'b101;
			else if ( WB_regWrite2 == 1'b1 && WB_c_rd == IF_ID_rs1)
				j_forward = 3'b110;

			if( EX_MEM_regWrite1 == 1'b1 && EX_MEM_rd == IF_ID_c_rs1)
				b_forward = 3'b001;
			else if(EX_MEM_regWrite2 == 1'b1 && EX_MEM_c_rd == IF_ID_c_rs1)
				b_forward = 3'b010;
			else if ( MEM_WB_regWrite1 == 1'b1 && MEM_WB_rd == IF_ID_c_rs1)
				b_forward = 3'b011;
			else if ( MEM_WB_regWrite2 == 1'b1 && MEM_WB_c_rd == IF_ID_c_rs1)
				b_forward = 3'b100;
			else if ( WB_regWrite1 == 1'b1 && WB_rd == IF_ID_c_rs1)
				b_forward = 3'b101;
			else if ( WB_regWrite2 == 1'b1 && WB_c_rd == IF_ID_c_rs1)
				b_forward = 3'b110;

		end
endmodule


module hazardControl(input clk, input reset, input [6:0] opcode32 , input [2:0] funct3_16, input [4:0] if_id_rs1, input [4:0] if_id_rt1, 
								input [4:0] if_id_rs2, input [4:0] id_ex_rd1, input [4:0] id_ex_rd2, input id_ex_memRd, input id_branch, input id_jump, 
								input id_ex_regWr1, input id_ex_regWr2, input id_branchTaken, input [4:0] ex_mem_regDest2, 
								input nop32, input nop16, output reg if_id_flush, output reg pcWr, output reg if_id_wr, output reg controlSignalMuxSel);

	always@(negedge clk, reset, opcode32, funct3_16, if_id_rs1, if_id_rt1, if_id_rs2, id_ex_rd1, id_ex_rd2, id_ex_memRd, id_branch, id_jump, id_ex_regWr1, id_ex_regWr2, id_branchTaken, ex_mem_regDest2, nop32, nop16)
	begin
		if(reset)
		begin
			if_id_flush = 1'b0;
			pcWr = 1'b1;
			if_id_wr = 1'b1;
			controlSignalMuxSel = 1'b0;
		end
		else
		begin
			//Load Use (Load then using that value)
			if(id_ex_memRd && (((id_ex_rd2 == if_id_rs1) || ((id_ex_rd2 == if_id_rt1) && ((opcode32 != 7'b0110011) || (opcode32 != 7'b0100011))) && !nop32) || ((id_ex_rd2 == if_id_rs2) && (funct3_16 != 3'b011) && !nop16)))
			begin
				//Checking with rs1
				if_id_flush = 1'b0;
				pcWr = 1'b0;
				if_id_wr = 1'b0;
				controlSignalMuxSel = 1'b1; //Make control signals 0 for noOp
			end
			//Modification before branch or jump
			else if((id_branch && !nop16 && id_ex_regWr1 && (id_ex_rd1 == if_id_rs2)) || (id_jump && !nop32 && id_ex_regWr1 && (id_ex_rd1 == if_id_rs1)))
			begin
			if_id_flush = 1'b0;
			pcWr = 1'b0;
			if_id_wr = 1'b0;
			controlSignalMuxSel = 1'b1; //Make control signals 0 for noOp		
			end
			else if((id_branch && !nop16 && id_ex_regWr2 && (id_ex_rd2 == if_id_rs2)) || (id_jump && !nop32 && id_ex_regWr2 && (id_ex_rd2 == if_id_rs1)))
			begin
			if_id_flush = 1'b0;
			pcWr = 1'b0;
			if_id_wr = 1'b0;
			controlSignalMuxSel = 1'b1; //Make control signals 0 for noOp		
			end
			//load then branch or jump between 1st and 2nd instr (Check if can be taken care by above else if)
			else if(((id_branch && !nop16 && (id_ex_rd2 == if_id_rs2)) || (id_jump && !nop32 && (id_ex_rd2 == if_id_rs1))) && id_ex_memRd)
			begin 
			//Implement
			if_id_flush = 1'b0;
			pcWr = 1'b0;
			if_id_wr = 1'b0;
			controlSignalMuxSel = 1'b1;
			end
			//load then branch or jump between 2nd and 3rd instr
			else if(((id_branch && !nop16 && (ex_mem_regDest2 == if_id_rs2)) || (id_jump && !nop32 && (ex_mem_regDest2 == if_id_rs1))) && id_ex_memRd)
			begin 
			//Implement
			if_id_flush = 1'b0;
			pcWr = 1'b0;
			if_id_wr = 1'b0;
			controlSignalMuxSel = 1'b1;
			end
			//branch then flush (id_ex_branchTaken is the output of comparator with regRs1 and 0 as input(check if we have to flush for jalr)
			else if(id_branchTaken && id_branch && !nop16)
			begin
			//Implement
			if_id_flush = 1'b1;
			pcWr = 1'b1;
			if_id_wr = 1'b0;
			controlSignalMuxSel = 1'b0;
			end
			//default
			else if(id_jump && !nop32)
			begin
			//jump then flush 
			if_id_flush = 1'b1;
			pcWr = 1'b1;
			if_id_wr = 1'b0;
			controlSignalMuxSel = 1'b0;
			end
			else
			begin
				if_id_flush = 1'b0;
				pcWr = 1'b1;
				if_id_wr = 1'b1;
				controlSignalMuxSel = 1'b0;
			end
		end
	end	
endmodule


module controlCkt(input [6:0] opcode, input [2:0] funct3, input [1:0] c_opcode, input [2:0] c_funct3, input [4:0] c_rd, output reg regWrite1, output reg regWrite2, output reg [1:0] aluSrcB, output reg regDest, output reg [2:0] aluOp, output reg memRd, output reg memWrite, output reg memToReg1, output reg memToReg2, output reg dest, output reg jump, output reg branch, output reg nop32, output reg nop16);
	always@(opcode, funct3, c_opcode, c_funct3, c_rd)
		begin
			regWrite1 = 1'b1;
			regWrite2 = 1'b0;//don't care
			regDest = 1'b0;//don't care
			memToReg1 = 1'b0;
			memToReg2 = 1'b0;//don't care
			memWrite = 1'b0;
			jump = 1'b0;
			branch = 1'b0;
			dest = 1'b0;
			memRd =1'b0;
			nop32 = 1'b0;
			nop16 = 1'b0;
			if(opcode == 7'd0)
				nop32 = 1'b1;
			if(c_funct3 == 3'd0)
				nop16 = 1'b1;
			if(opcode[6:4]==3'b001)
				begin
					case(funct3)
						3'b000:
							begin
								aluSrcB = 2'b10;
								aluOp = 3'b000;
							end
						3'b010:
							begin
								aluSrcB = 2'b10;
								aluOp = 3'b100;
							end
						3'b101:
						begin
							aluSrcB = 2'b00;
							aluOp = 3'b011;
						end
					endcase
				end
			else if(opcode[6:4]==3'b011 && funct3==3'b100)
				begin
					aluSrcB = 2'b01;
					aluOp = 3'b010;
				end
			else if(opcode[6:4]==3'b110 && funct3==3'b000)
				begin
					//aluSrcB = 3'b011;
					aluOp = 3'b000;
					jump = 1'b1;
					memToReg1 = 1'b1;
				end
			else if(opcode[6:4]==3'b010 && funct3==3'b001)
				begin
					regWrite1 = 1'b0;
					memWrite = 1'b1;
					aluSrcB = 2'b11;
					aluOp = 3'b000;
					dest = 1'b1;
				end
			if(c_opcode[0]==1'b1)
				begin
					case(c_funct3)
						3'b011:
							begin
								regWrite2 = 1'b1;
								if(c_rd[4:0] == 5'b00000)
									regWrite2 = 1'b0;
									
								memToReg2 = 1'b1;
							end
						3'b111:
							begin
								regWrite2 = 1'b0;
								branch = 1'b1;
							end
					endcase
				end
			else if(c_opcode[0]==1'b0 && c_funct3==3'b010)
				begin
					memRd =1'b1;
					regWrite2 = 1'b1;
					regDest = 1'b1;
				end
		end

endmodule

module ALU_32bits(input signed [31:0] aluIn1, input signed [31:0] aluIn2, input [2:0] aluOp, output reg [31:0] aluOut);
	always@(aluIn1, aluIn2, aluOp)
		begin
			case(aluOp)
				3'd0: aluOut = aluIn1 + aluIn2;
				3'd1: aluOut = aluIn1 - aluIn2;
				3'd2: aluOut = aluIn1 ^ aluIn2;
				3'd3: aluOut = aluIn1 >> aluIn2;
				3'd4: aluOut = aluIn1 < aluIn2 ? 32'd1 : 32'd0;
			endcase
		end
endmodule

module PC_32bits(input clk, input reset, input [31:0] pc_in, output [31:0] pc_out);
	Register_32bit pc(clk, reset, 1'b1, pc_in, pc_out);
endmodule

module adder_32bits(input [31:0] in1, input [31:0] in2, output reg [31:0] adder_out);
	always@(in1, in2)
		adder_out = in1 + in2;
endmodule

module subtractor_32bits(input [31:0] in1, input [31:0] in2, output reg [31:0] subtractor_out);
	always@(in1, in2)
		subtractor_out = in1 - in2;
endmodule

module comparator_32bits(input [31:0] in1, input [31:0] in2, output reg comparator_out);
	always@(in1, in2)
		comparator_out = in1 == in2 ? 1'b0 : 1'b1;
endmodule

module data_path(input clk, input clk2, input reset, input reset2, input [127:0] memToCacheDataBus, output [31:0] result1, output [31:0] result2);
	
	wire regWrite1, regWrite2, regDest, memRd, memWrite, memToReg1, memToReg2, dest, jump, branch, id_ex_regWrite1_in, id_ex_regWrite2_in, id_ex_regDest_in, id_ex_memWrite_in, id_ex_memToReg1_in, id_ex_memToReg2_in, id_ex_dest_in, id_ex_jump_in, id_ex_branch_in,
		  if_id_flush, if_id_wr, pcWr, controlSignalMuxSel, branch_taken, id_ex_out_regDest,
		  id_ex_out_memWrite, id_ex_out_dest,	id_ex_out_memToReg1,	id_ex_out_memToReg2,	id_ex_out_regWrite1,	id_ex_out_regWrite2, ex_mem_out_memRd, ex_mem_out_memWrite,
		  ex_mem_out_dest,ex_mem_out_memToReg1,ex_mem_out_memToReg2,ex_mem_out_regWrite1,ex_mem_out_regWrite2, mem_wb_out_memToReg2,
		  mem_wb_out_regWrite1, mem_wb_out_regWrite2, wb_out_regWrite1, wb_out_regWrite2, id_ex_out_memRd, nop32, nop16, cacheMiss; 
	
	wire [1:0] aluSrcB, id_ex_aluSrcB_in, id_ex_out_aluSrcB;			
	
	wire [2:0] aluOp, id_ex_aluOp_in, j_forward, b_forward, id_ex_out_aluOp, aluSrcA_fwd_mux_sel, aluSrcB_fwd_mux_sel, src16_mux_sel;
	
	wire [4:0] rd_c, id_ex_out_rd, id_ex_out_c_rda, id_ex_out_c_rdb, ex_mem_out_rd, ex_mem_out_c_rd, mem_wb_out_rd, mem_wb_out_c_rd,
				  wb_out_rd, wb_out_c_rd, id_ex_out_rs1, id_ex_out_rs2, id_ex_out_c_rs1;
	
	wire [15:0] if_id_irc;
	
	wire [31:0] pc_in, pc_out, pc_adder_out, if_id_ira, if_id_pc, reg_rs1, reg_rs2, reg_c_rs1, shamt_in, addi_jalr_in, sw_in, c_lui_in, old_pc,
					adder_branch_in, branch_address, jump_mux_fwd_out, jump_imm_ext_out, jump_address_out, jump_mux_out, branch_taken_mux_out,
					id_ex_out_reg_rs1, id_ex_out_reg_rs2, id_ex_out_reg_c_rs1, id_ex_out_shamt, id_ex_out_addi_jalr_imm, id_ex_out_sw_imm,
					id_ex_out_c_lui_imm,	id_ex_out_c_lw_imm, aluIn0 , aluIn1 , aluIn1Mux1, adder_16_in, ex_mem_in_aluOuta, ex_mem_in_aluOutb,
					ex_mem_out_reg_rs2, ex_mem_out_alu_a_out, ex_mem_out_alu_b_out, ex_mem_out_c_lui_imm,  dm_mux_out, mem_wb_dm_in, mem_wb_out_wrt_to_rd1,
					mem_wb_out_c_lui_imm, mem_wb_out_c_dm_out, id_ex_out_pc, mem_reg_1_out, mem_reg_2_out, wb_out_reg_rd, wb_out_c_reg_rd, ex_mem_out_pc;
					
	wire [47:0] IM_out, cacheOut;				
						
	//IF
	mux2to1_32bits jump_mux(pc_adder_out, jump_address_out, jump, jump_mux_out);
	mux2to1_32bits branch_mux(jump_mux_out, branch_address, (id_ex_branch_in & id_ex_branch_taken_in), pc_in);
	Register_32bit PC(clk, reset, pcWr, pc_in, pc_out);
	adder_32bits pc_adder(pc_out, 32'd4, pc_adder_out);
	IM instr_mem(clk, reset, pc_out[5:0], IM_out);
	IF_ID if_id(clk, (reset | if_id_flush), if_id_wr, IM_out, pc_adder_out, if_id_ira, if_id_irc, if_id_pc);
	
	
	//ID
	registerFile regFile(clk, reset, mem_wb_out_regWrite1, mem_wb_out_regWrite2, mem_wb_out_wrt_to_rd1, mem_reg_2_out, 
							if_id_ira[19:15], if_id_ira[24:20], {2'b00, if_id_irc[9:7]}, mem_wb_out_rd, mem_wb_out_c_rd,
								reg_rs1, reg_rs2, reg_c_rs1);
	
	controlCkt ctrlCkt(if_id_ira[6:0], if_id_ira[14:12], if_id_irc[1:0], if_id_irc[15:13], if_id_irc[11:7], regWrite1, regWrite2, aluSrcB, regDest, aluOp, memRd, memWrite, memToReg1, memToReg2, dest, jump, branch, nop32, nop16);

	zeroExt5to32 shamt_ext(if_id_ira[24:20], shamt_in);
	signExt12to32 addi_jalr_ext(if_id_ira[31:20], addi_jalr_in);
	signExt12to32 sw_ext({if_id_ira[31:25], if_id_ira[11:7]}, sw_in);
	signExt18to32 c_lui_ext({if_id_irc[12], if_id_irc[6:2], 12'd0}, c_lui_in);
	subtractor_32bits sub_branch(if_id_pc, 32'd4, old_pc);
	signExt9to32 c_bnez_ext({if_id_irc[12], if_id_irc[6:5], if_id_irc[2], if_id_irc[11:10], if_id_irc[4:3], 1'b0}, adder_branch_in);
	adder_32bits adder_branch(old_pc, adder_branch_in, branch_address);
	mux8to1_32bits jump_mux_fwd(reg_rs1, mem_reg_1_out, ex_mem_out_c_lui_imm, mem_wb_out_wrt_to_rd1, mem_reg_2_out, wb_out_reg_rd, wb_out_c_reg_rd, 32'd0, j_forward, jump_mux_fwd_out);
	signExt13to32 jump_imm_ext({if_id_ira[31:20], 1'b0}, jump_imm_ext_out);
	adder_32bits jump_address(jump_mux_fwd_out, jump_imm_ext_out, jump_address_out);
	mux8to1_32bits branch_taken_mux(reg_c_rs1, mem_reg_1_out, ex_mem_out_c_lui_imm, mem_wb_out_wrt_to_rd1, mem_reg_2_out, wb_out_reg_rd, wb_out_c_reg_rd, 32'd0, b_forward, branch_taken_mux_out);
	comparator_32bits compare(branch_taken_mux_out, 32'd0, branch_taken); 
	mux2to1_14bits ctrl_signal_mux({aluOp, aluSrcB, branch, branch_taken, regDest, memWrite, dest, memToReg1, memToReg2, regWrite1, regWrite2}, 14'd0, controlSignalMuxSel, {id_ex_aluOp_in, id_ex_aluSrcB_in, id_ex_branch_in, id_ex_branch_taken_in, id_ex_regDest_in, id_ex_memWrite_in, id_ex_dest_in, id_ex_memToReg1_in, id_ex_memToReg2_in, id_ex_regWrite1_in, id_ex_regWrite2_in});
	
	hazardControl hzd(clk, reset, if_id_ira[6:0] , if_id_irc[15:13], if_id_ira[19:15], if_id_ira[24:20], 
								{2'b00, if_id_irc[9:7]}, id_ex_out_rd, rd_c, id_ex_out_memRd, branch, jump, 
									id_ex_out_regWrite1, id_ex_out_regWrite2, branch_taken, ex_mem_out_c_rd, nop32, nop16, 
									if_id_flush, pcWr, if_id_wr, controlSignalMuxSel);
	forwarding_unit_B fwd_ID(clk, reset,
							if_id_ira[19:15],
							{2'b00,if_id_irc[9:7]},
							ex_mem_out_rd,
							ex_mem_out_c_rd,
							mem_wb_out_rd,
							mem_wb_out_c_rd,
							wb_out_rd,
							wb_out_c_rd,
							ex_mem_out_regWrite1,
							ex_mem_out_regWrite2,
							mem_wb_out_regWrite1,
							mem_wb_out_regWrite2,
							wb_out_regWrite1,
							wb_out_regWrite2,
							j_forward,
							b_forward);

	ID_EX id_ex(clk, reset, 1'b1,  
				id_ex_aluOp_in, 
				id_ex_aluSrcB_in,  
				id_ex_regDest_in, 
				id_ex_memWrite_in, 
				id_ex_dest_in, 
				id_ex_memToReg1_in, 
				id_ex_memToReg2_in, 
				id_ex_regWrite1_in, 
				id_ex_regWrite2_in, 
 				if_id_ira[11:7],
				if_id_irc[11:7],
				{2'b00, if_id_irc[4:2]},
				reg_rs1,
				reg_rs2,
				reg_c_rs1,
				shamt_in,
				addi_jalr_in,
				sw_in,
				c_lui_in,
				{25'd0, if_id_irc[5], if_id_irc[12:10], if_id_irc[6], 2'd0},
				if_id_pc,
				if_id_ira[19:15],
				if_id_ira[24:20],
				{2'b00,if_id_irc[9:7]},
				memRd,
				id_ex_out_aluOp,
				id_ex_out_aluSrcB,
				id_ex_out_regDest,
				id_ex_out_memWrite,
				id_ex_out_dest,
				id_ex_out_memToReg1,
				id_ex_out_memToReg2,
				id_ex_out_regWrite1,
				id_ex_out_regWrite2,
				id_ex_out_rd,
				id_ex_out_c_rda,
				id_ex_out_c_rdb,
				id_ex_out_reg_rs1,
				id_ex_out_reg_rs2,
				id_ex_out_reg_c_rs1,
				id_ex_out_shamt,
				id_ex_out_addi_jalr_imm,
				id_ex_out_sw_imm,
				id_ex_out_c_lui_imm,
				id_ex_out_c_lw_imm,
				id_ex_out_pc,
				id_ex_out_rs1,
				id_ex_out_rs2,
				id_ex_out_c_rs1,
				id_ex_out_memRd);
				
		//EX
		mux2to1_5bits rd_c_mux(id_ex_out_c_rda, id_ex_out_c_rdb, id_ex_out_regDest, rd_c);
		mux8to1_32bits aluSrcA_fwd_mux(id_ex_out_reg_rs1, mem_reg_1_out, ex_mem_out_c_lui_imm, mem_wb_out_wrt_to_rd1, mem_reg_2_out, wb_out_reg_rd, wb_out_c_reg_rd, 32'd0, aluSrcA_fwd_mux_sel, aluIn0); //sel from fwd unit a and rest inputs from pipeline registers
		mux8to1_32bits aluSrcB_fwd_mux(id_ex_out_reg_rs2,mem_reg_1_out, ex_mem_out_c_lui_imm, mem_wb_out_wrt_to_rd1, mem_reg_2_out, wb_out_reg_rd, wb_out_c_reg_rd, 32'd0, aluSrcB_fwd_mux_sel, aluIn1Mux1);
		mux4to1_32bits aluIn1Mux(id_ex_out_shamt, aluIn1Mux1, id_ex_out_addi_jalr_imm, id_ex_out_sw_imm, id_ex_out_aluSrcB, aluIn1);
		ALU_32bits alu_for32Bits(aluIn0, aluIn1, id_ex_out_aluOp, ex_mem_in_aluOuta);
		mux8to1_32bits src16_mux(id_ex_out_reg_c_rs1,mem_reg_1_out, ex_mem_out_c_lui_imm, mem_wb_out_wrt_to_rd1, mem_reg_2_out, wb_out_reg_rd, wb_out_c_reg_rd, 32'd0, src16_mux_sel, adder_16_in);
		adder_32bits adder_for_c(adder_16_in, id_ex_out_c_lw_imm, ex_mem_in_aluOutb);
		
		forwarding_unit_A fwdA(clk ,reset,
							id_ex_out_rs1,
							id_ex_out_rs2,
							id_ex_out_c_rs1,
							ex_mem_out_rd,
							ex_mem_out_c_rd,
							mem_wb_out_rd,
							mem_wb_out_c_rd,
							wb_out_rd,
							wb_out_c_rd,
							ex_mem_out_regWrite1,
							ex_mem_out_regWrite2,
							mem_wb_out_regWrite1,
							mem_wb_out_regWrite2,
							wb_out_regWrite1,
							wb_out_regWrite2,
							aluSrcA_fwd_mux_sel,
							aluSrcB_fwd_mux_sel,
							src16_mux_sel);
							
		EX_MEM ex_mem(clk, reset, 1'b1,
				id_ex_out_memRd,
				id_ex_out_memWrite,
				id_ex_out_dest,
				id_ex_out_memToReg1,
				id_ex_out_memToReg2,
				id_ex_out_regWrite1,
				id_ex_out_regWrite2,
				id_ex_out_rd,
				rd_c,
				aluIn1Mux1,
				ex_mem_in_aluOuta,
				ex_mem_in_aluOutb,
				id_ex_out_c_lui_imm,
				id_ex_out_pc,
				ex_mem_out_memRd,
				ex_mem_out_memWrite,
				ex_mem_out_dest,
				ex_mem_out_memToReg1,
				ex_mem_out_memToReg2,
				ex_mem_out_regWrite1,
				ex_mem_out_regWrite2,
				ex_mem_out_rd,
				ex_mem_out_c_rd,
				ex_mem_out_reg_rs2,
				ex_mem_out_alu_a_out,
				ex_mem_out_alu_b_out,
				ex_mem_out_c_lui_imm,
				ex_mem_out_pc);
		
		//MEM
		mux2to1_32bits dm_mux(ex_mem_out_alu_b_out, ex_mem_out_alu_a_out, ex_mem_out_dest, dm_mux_out);
		cache dmCache(clk2, reset2, ex_mem_out_memRd, ex_mem_out_memWrite, dm_mux_out, 
					ex_mem_out_reg_rs2, memToCacheDataBus, cacheOut, cacheMiss);
		//DM dm(clk, reset, dm_mux_out[4:0], ex_mem_out_reg_rs2, ex_mem_out_memWrite, mem_wb_dm_in);
		mux2to1_32bits mem_reg_1(ex_mem_out_alu_a_out, ex_mem_out_pc, ex_mem_out_memToReg1, mem_reg_1_out);
		
		//MEM/WB
		
		
		MEM_WB mem_wb(clk, reset, 1'b1,
				//input wb,
				ex_mem_out_rd,
				ex_mem_out_c_rd,
				mem_reg_1_out,
				ex_mem_out_c_lui_imm,
				cacheOut[31:0],
				ex_mem_out_memToReg2,
				ex_mem_out_regWrite1,
				ex_mem_out_regWrite2,
				mem_wb_out_rd,
				mem_wb_out_c_rd,
				mem_wb_out_wrt_to_rd1,
				mem_wb_out_c_lui_imm,
				mem_wb_out_c_dm_out,
				mem_wb_out_memToReg2,
				mem_wb_out_regWrite1,
				mem_wb_out_regWrite2);
		
		//WB
		mux2to1_32bits mem_reg_2(mem_wb_out_c_dm_out, mem_wb_out_c_lui_imm, mem_wb_out_memToReg2, mem_reg_2_out);
		
		WB wb(clk, reset, 1'b1,
			mem_wb_out_rd,
			mem_wb_out_c_rd,
			mem_wb_out_wrt_to_rd1,
			mem_reg_2_out,
			mem_wb_out_regWrite1,
			mem_wb_out_regWrite2,
			wb_out_rd,
			wb_out_c_rd,
			wb_out_reg_rd,
			wb_out_c_reg_rd,
			wb_out_regWrite1,
			wb_out_regWrite2);
				
		assign result1 = wb_out_reg_rd;
		assign result2 = wb_out_c_reg_rd;
		
		
endmodule

module testbench;

	reg clk;
	reg clk2;
	reg reset;
	reg reset2;
	reg [127:0] memToCacheDataBus;
	wire [31:0] result1;
	wire [31:0] result2;
	data_path d1(clk, clk2, reset, reset2, memToCacheDataBus, result1, result2);
	
	always
		#5 clk=~clk;
	always
		#1 clk2=~clk2;

		initial
		begin
			clk=0;clk2=0;reset=1;reset2=1;memToCacheDataBus=128'h11111111111111111111111111111111;
			#15 reset=0;
			#4 reset2=0;
			#200 $finish;
		end

endmodule
