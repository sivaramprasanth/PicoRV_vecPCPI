//2-port model
// module vector_regs (
// 	input clk, wen,
// 	input [4:0] waddr,
// 	input [15:0] vec_wstrb, //16 bit data so that each word has 1 bit
// 	input [4:0] raddr1,
// 	input [4:0] raddr2,
// 	input port3_en,    //Will be 1 only when we need to read three variables
// 	input [15:0] vec_rstrb1, //Currently using only one strb for all the reads
// 	// input [15:0] vec_rstrb2;
// 	// input [15:0] vec_rstrb3;
// 	input [31:0] wdata,
// 	output reg [31:0] rdata1,
// 	output reg [31:0] rdata2
// );
// 	reg [511:0] vregs [0:31]; //32 512 bit vector registers

// 	always @(posedge clk) begin
// 		if (wen) begin
// 			// vregs[waddr[4:0]] <= wdata;
// 			$display("wdata: %x, waddr:%d, wstrb:%b, time:%d",wdata, waddr, vec_wstrb, $time);
// 			if (vec_wstrb[0])	vregs[waddr[4:0]][31: 0] <= wdata;
// 			if (vec_wstrb[1]) 	vregs[waddr[4:0]][63: 32] <= wdata;
// 			if (vec_wstrb[2]) 	vregs[waddr[4:0]][95: 64] <= wdata;
// 			if (vec_wstrb[3]) 	vregs[waddr[4:0]][127: 96] <= wdata;
// 			if (vec_wstrb[4]) 	vregs[waddr[4:0]][159: 128] <= wdata;
// 			if (vec_wstrb[5]) 	vregs[waddr[4:0]][191: 160] <= wdata;
// 			if (vec_wstrb[6]) 	vregs[waddr[4:0]][223: 192] <= wdata;
// 			if (vec_wstrb[7]) 	vregs[waddr[4:0]][255: 224] <= wdata;
// 			if (vec_wstrb[8]) 	vregs[waddr[4:0]][287: 256] <= wdata;
// 			if (vec_wstrb[9]) 	vregs[waddr[4:0]][319: 288] <= wdata;
// 			if (vec_wstrb[10]) 	vregs[waddr[4:0]][351: 320] <= wdata;
// 			if (vec_wstrb[11]) 	vregs[waddr[4:0]][383: 352] <= wdata;
// 			if (vec_wstrb[12]) 	vregs[waddr[4:0]][415: 384] <= wdata;
// 			if (vec_wstrb[13]) 	vregs[waddr[4:0]][447: 416] <= wdata;
// 			if (vec_wstrb[14]) 	vregs[waddr[4:0]][479: 448] <= wdata;
// 			if (vec_wstrb[15]) 	vregs[waddr[4:0]][511: 480] <= wdata;
// 		end
// 		if(!port3_en) begin
// 			case(1'b1)
// 				vec_rstrb1[0]: begin
// 								rdata1 <= vregs[raddr1[4:0]][31:0];
// 								rdata2 <= vregs[raddr2[4:0]][31:0];
// 								// $display("rdata1: %x, rdata2:%x, raddr1:%d, raddr2:%d, rstrb:%b, time:%d",rdata1, rdata2, raddr1, raddr2, vec_rstrb1, $time);
// 							end
// 				vec_rstrb1[1]: begin
// 								rdata1 <= vregs[raddr1[4:0]][63:32];
// 								rdata2 <= vregs[raddr2[4:0]][63:32];
// 								// $display("rdata1: %x, rdata2:%x, raddr1:%d, raddr2:%d, rstrb:%b, time:%d",rdata1, rdata2, raddr1, raddr2, vec_rstrb1, $time);
// 							end
// 				vec_rstrb1[2]: begin
// 								rdata1 <= vregs[raddr1[4:0]][95:64];
// 								rdata2 <= vregs[raddr2[4:0]][95:64];
// 								// $display("rdata1: %x, rdata2:%x, raddr1:%d, raddr2:%d, rstrb:%b, time:%d",rdata1, rdata2, raddr1, raddr2, vec_rstrb1, $time);
// 							end
// 				vec_rstrb1[3]: begin
// 								rdata1 <= vregs[raddr1[4:0]][127: 96];
// 								rdata2 <= vregs[raddr2[4:0]][127: 96];
// 								// $display("rdata1: %x, rdata2:%x, raddr1:%d, raddr2:%d, rstrb:%b, time:%d",rdata1, rdata2, raddr1, raddr2, vec_rstrb1, $time);
// 							end
// 				vec_rstrb1[4]: begin
// 								rdata1 <= vregs[raddr1[4:0]][159: 128];
// 								rdata2 <= vregs[raddr2[4:0]][159: 128];
// 								// $display("rdata1: %x, rdata2:%x, raddr1:%d, raddr2:%d, rstrb:%b, time:%d",rdata1, rdata2, raddr1, raddr2, vec_rstrb1, $time);
// 							end
// 				vec_rstrb1[5]: begin
// 								rdata1 <= vregs[raddr1[4:0]][191: 160];
// 								rdata2 <= vregs[raddr2[4:0]][191: 160];
// 								// $display("rdata1: %x, rdata2:%x, raddr1:%d, raddr2:%d, rstrb:%b, time:%d",rdata1, rdata2, raddr1, raddr2, vec_rstrb1, $time);
// 							end
// 				vec_rstrb1[6]: begin
// 								rdata1 <= vregs[raddr1[4:0]][223: 192];
// 								rdata2 <= vregs[raddr2[4:0]][223: 192];
// 								// $display("rdata1: %x, rdata2:%x, raddr1:%d, raddr2:%d, rstrb:%b, time:%d",rdata1, rdata2, raddr1, raddr2, vec_rstrb1, $time);
// 							end
// 				vec_rstrb1[7]: begin
// 								rdata1 <= vregs[raddr1[4:0]][255: 224];
// 								rdata2 <= vregs[raddr2[4:0]][255: 224];
// 								// $display("rdata1: %x, rdata2:%x, raddr1:%d, raddr2:%d, rstrb:%b, time:%d",rdata1, rdata2, raddr1, raddr2, vec_rstrb1, $time);
// 							end
// 				vec_rstrb1[8]: begin
// 								rdata1 <= vregs[raddr1[4:0]][287:256];
// 								rdata2 <= vregs[raddr2[4:0]][287:256];
// 								// $display("rdata1: %x, rdata2:%x, raddr1:%d, raddr2:%d, rstrb:%b, time:%d",rdata1, rdata2, raddr1, raddr2, vec_rstrb1, $time);
// 							end
// 				vec_rstrb1[9]: begin
// 								rdata1 <= vregs[raddr1[4:0]][319: 288];
// 								rdata2 <= vregs[raddr2[4:0]][319: 288];
// 								// $display("rdata1: %x, rdata2:%x, raddr1:%d, raddr2:%d, rstrb:%b, time:%d",rdata1, rdata2, raddr1, raddr2, vec_rstrb1, $time);
// 							end
// 				vec_rstrb1[10]: begin
// 								rdata1 <= vregs[raddr1[4:0]][351: 320];
// 								rdata2 <= vregs[raddr2[4:0]][351: 320];
// 								// $display("rdata1: %x, rdata2:%x, raddr1:%d, raddr2:%d, rstrb:%b, time:%d",rdata1, rdata2, raddr1, raddr2, vec_rstrb1, $time);
// 							end
// 				vec_rstrb1[11]: begin
// 								rdata1 <= vregs[raddr1[4:0]][383: 352];
// 								rdata2 <= vregs[raddr2[4:0]][383: 352];
// 								// $display("rdata1: %x, rdata2:%x, raddr1:%d, raddr2:%d, rstrb:%b, time:%d",rdata1, rdata2, raddr1, raddr2, vec_rstrb1, $time);
// 							end
// 				vec_rstrb1[12]: begin
// 								rdata1 <= vregs[raddr1[4:0]][415: 384];
// 								rdata2 <= vregs[raddr2[4:0]][415: 384];
// 								// $display("rdata1: %x, rdata2:%x, raddr1:%d, raddr2:%d, rstrb:%b, time:%d",rdata1, rdata2, raddr1, raddr2, vec_rstrb1, $time);
// 							end
// 				vec_rstrb1[13]: begin
// 								rdata1 <= vregs[raddr1[4:0]][447: 416];
// 								rdata2 <= vregs[raddr2[4:0]][447: 416];
// 								// $display("rdata1: %x, rdata2:%x, raddr1:%d, raddr2:%d, rstrb:%b, time:%d",rdata1, rdata2, raddr1, raddr2, vec_rstrb1, $time);
// 							end
// 				vec_rstrb1[14]: begin
// 								rdata1 <= vregs[raddr1[4:0]][479:448];
// 								rdata2 <= vregs[raddr2[4:0]][479:448];
// 								// $display("rdata1: %x, rdata2:%x, raddr1:%d, raddr2:%d, rstrb:%b, time:%d",rdata1, rdata2, raddr1, raddr2, vec_rstrb1, $time);
// 							end
// 				vec_rstrb1[15]: begin
// 								rdata1 <= vregs[raddr1[4:0]][511:480];
// 								rdata2 <= vregs[raddr2[4:0]][511:480];
// 								// $display("rdata1: %x, rdata2:%x, raddr1:%d, raddr2:%d, rstrb:%b, time:%d",rdata1, rdata2, raddr1, raddr2, vec_rstrb1, $time);
// 							end
// 			endcase
// 		end
// 		if(port3_en) begin
// 			case(1'b1)
// 			vec_rstrb1[0]: begin
// 							rdata1 <= vregs[raddr1[4:0]][31:0];
// 						   end
// 			vec_rstrb1[1]: begin
// 							rdata1 <= vregs[raddr1[4:0]][63:32];
// 						   end
// 			vec_rstrb1[2]: begin
// 							rdata1 <= vregs[raddr1[4:0]][95:64];
// 						   end
// 			vec_rstrb1[3]: begin
// 							rdata1 <= vregs[raddr1[4:0]][127: 96];
// 						   end
// 			vec_rstrb1[4]: begin
// 							rdata1 <= vregs[raddr1[4:0]][159: 128];
// 						   end
// 			vec_rstrb1[5]: begin
// 							rdata1 <= vregs[raddr1[4:0]][191: 160];
// 						   end
// 			vec_rstrb1[6]: begin
// 							rdata1 <= vregs[raddr1[4:0]][223: 192];
// 						   end
// 			vec_rstrb1[7]: begin
// 							rdata1 <= vregs[raddr1[4:0]][255: 224];
// 						   end
// 			vec_rstrb1[8]: begin
// 							rdata1 <= vregs[raddr1[4:0]][287:256];
// 						   end
// 			vec_rstrb1[9]: begin
// 							rdata1 <= vregs[raddr1[4:0]][319: 288];
// 						   end
// 			vec_rstrb1[10]: begin
// 							rdata1 <= vregs[raddr1[4:0]][351: 320];
// 						   end
// 			vec_rstrb1[11]: begin
// 							rdata1 <= vregs[raddr1[4:0]][383: 352];
// 						   end
// 			vec_rstrb1[12]: begin
// 							rdata1 <= vregs[raddr1[4:0]][415: 384];
// 						   end
// 			vec_rstrb1[13]: begin
// 							rdata1 <= vregs[raddr1[4:0]][447: 416];
// 						   end
// 			vec_rstrb1[14]: begin
// 							rdata1 <= vregs[raddr1[4:0]][479:448];
// 						   end
// 			vec_rstrb1[15]: begin
// 							rdata1 <= vregs[raddr1[4:0]][511:480];
// 						   end
// 			endcase
// 		end
// 	end
// 	// assign rdata1 = vregs[raddr1[4:0]];
// 	// assign rdata2 = vregs[raddr2[4:0]];
// 	// assign rdata3 = vregs[raddr3[4:0]];

// endmodule


//Three port model
// module vector_regs (
// 	input clk, wen,
// 	input [4:0] waddr,
// 	input [15:0] vec_wstrb, //16 bit data so that each word has 1 bit
// 	input [4:0] raddr1,
// 	input [4:0] raddr2,
// 	input [4:0] raddr3,
// 	input port3_en,    //Will be 1 only when we need to read three variables
// 	input [15:0] vec_rstrb1, //Currently using only one strb for all the reads
// 	// input [15:0] vec_rstrb2;
// 	// input [15:0] vec_rstrb3;
// 	input [31:0] wdata,
// 	output reg [31:0] rdata1,
// 	output reg [31:0] rdata2,
// 	output reg [31:0] rdata3
// );
// 	reg [511:0] vregs [0:31]; //32 512 bit vector registers

// 	always @(posedge clk) begin
// 		if (wen) begin
// 			// vregs[waddr[4:0]] <= wdata;
// 			$display("wdata: %x, waddr:%d, wstrb:%b, time:%d",wdata, waddr, vec_wstrb, $time);
// 			if (vec_wstrb[0])	vregs[waddr[4:0]][31: 0] <= wdata;
// 			if (vec_wstrb[1]) 	vregs[waddr[4:0]][63: 32] <= wdata;
// 			if (vec_wstrb[2]) 	vregs[waddr[4:0]][95: 64] <= wdata;
// 			if (vec_wstrb[3]) 	vregs[waddr[4:0]][127: 96] <= wdata;
// 			if (vec_wstrb[4]) 	vregs[waddr[4:0]][159: 128] <= wdata;
// 			if (vec_wstrb[5]) 	vregs[waddr[4:0]][191: 160] <= wdata;
// 			if (vec_wstrb[6]) 	vregs[waddr[4:0]][223: 192] <= wdata;
// 			if (vec_wstrb[7]) 	vregs[waddr[4:0]][255: 224] <= wdata;
// 			if (vec_wstrb[8]) 	vregs[waddr[4:0]][287: 256] <= wdata;
// 			if (vec_wstrb[9]) 	vregs[waddr[4:0]][319: 288] <= wdata;
// 			if (vec_wstrb[10]) 	vregs[waddr[4:0]][351: 320] <= wdata;
// 			if (vec_wstrb[11]) 	vregs[waddr[4:0]][383: 352] <= wdata;
// 			if (vec_wstrb[12]) 	vregs[waddr[4:0]][415: 384] <= wdata;
// 			if (vec_wstrb[13]) 	vregs[waddr[4:0]][447: 416] <= wdata;
// 			if (vec_wstrb[14]) 	vregs[waddr[4:0]][479: 448] <= wdata;
// 			if (vec_wstrb[15]) 	vregs[waddr[4:0]][511: 480] <= wdata;
// 		end
// 		case(1'b1)
// 			vec_rstrb1[0]: begin
// 							rdata1 <= vregs[raddr1[4:0]][31:0];
// 							rdata2 <= vregs[raddr2[4:0]][31:0];
// 							// $display("rdata1: %x, rdata2:%x, raddr1:%d, raddr2:%d, rstrb:%b, time:%d",rdata1, rdata2, raddr1, raddr2, vec_rstrb1, $time);
// 						   end
// 			vec_rstrb1[1]: begin
// 							rdata1 <= vregs[raddr1[4:0]][63:32];
// 							rdata2 <= vregs[raddr2[4:0]][63:32];
// 							// $display("rdata1: %x, rdata2:%x, raddr1:%d, raddr2:%d, rstrb:%b, time:%d",rdata1, rdata2, raddr1, raddr2, vec_rstrb1, $time);
// 						   end
// 			vec_rstrb1[2]: begin
// 							rdata1 <= vregs[raddr1[4:0]][95:64];
// 							rdata2 <= vregs[raddr2[4:0]][95:64];
// 							// $display("rdata1: %x, rdata2:%x, raddr1:%d, raddr2:%d, rstrb:%b, time:%d",rdata1, rdata2, raddr1, raddr2, vec_rstrb1, $time);
// 						   end
// 			vec_rstrb1[3]: begin
// 							rdata1 <= vregs[raddr1[4:0]][127: 96];
// 							rdata2 <= vregs[raddr2[4:0]][127: 96];
// 							// $display("rdata1: %x, rdata2:%x, raddr1:%d, raddr2:%d, rstrb:%b, time:%d",rdata1, rdata2, raddr1, raddr2, vec_rstrb1, $time);
// 						   end
// 			vec_rstrb1[4]: begin
// 							rdata1 <= vregs[raddr1[4:0]][159: 128];
// 							rdata2 <= vregs[raddr2[4:0]][159: 128];
// 							// $display("rdata1: %x, rdata2:%x, raddr1:%d, raddr2:%d, rstrb:%b, time:%d",rdata1, rdata2, raddr1, raddr2, vec_rstrb1, $time);
// 						   end
// 			vec_rstrb1[5]: begin
// 							rdata1 <= vregs[raddr1[4:0]][191: 160];
// 							rdata2 <= vregs[raddr2[4:0]][191: 160];
// 							// $display("rdata1: %x, rdata2:%x, raddr1:%d, raddr2:%d, rstrb:%b, time:%d",rdata1, rdata2, raddr1, raddr2, vec_rstrb1, $time);
// 						   end
// 			vec_rstrb1[6]: begin
// 							rdata1 <= vregs[raddr1[4:0]][223: 192];
// 							rdata2 <= vregs[raddr2[4:0]][223: 192];
// 							// $display("rdata1: %x, rdata2:%x, raddr1:%d, raddr2:%d, rstrb:%b, time:%d",rdata1, rdata2, raddr1, raddr2, vec_rstrb1, $time);
// 						   end
// 			vec_rstrb1[7]: begin
// 							rdata1 <= vregs[raddr1[4:0]][255: 224];
// 							rdata2 <= vregs[raddr2[4:0]][255: 224];
// 							// $display("rdata1: %x, rdata2:%x, raddr1:%d, raddr2:%d, rstrb:%b, time:%d",rdata1, rdata2, raddr1, raddr2, vec_rstrb1, $time);
// 						   end
// 			vec_rstrb1[8]: begin
// 							rdata1 <= vregs[raddr1[4:0]][287:256];
// 							rdata2 <= vregs[raddr2[4:0]][287:256];
// 							// $display("rdata1: %x, rdata2:%x, raddr1:%d, raddr2:%d, rstrb:%b, time:%d",rdata1, rdata2, raddr1, raddr2, vec_rstrb1, $time);
// 						   end
// 			vec_rstrb1[9]: begin
// 							rdata1 <= vregs[raddr1[4:0]][319: 288];
// 							rdata2 <= vregs[raddr2[4:0]][319: 288];
// 							// $display("rdata1: %x, rdata2:%x, raddr1:%d, raddr2:%d, rstrb:%b, time:%d",rdata1, rdata2, raddr1, raddr2, vec_rstrb1, $time);
// 						   end
// 			vec_rstrb1[10]: begin
// 							rdata1 <= vregs[raddr1[4:0]][351: 320];
// 							rdata2 <= vregs[raddr2[4:0]][351: 320];
// 							// $display("rdata1: %x, rdata2:%x, raddr1:%d, raddr2:%d, rstrb:%b, time:%d",rdata1, rdata2, raddr1, raddr2, vec_rstrb1, $time);
// 						   end
// 			vec_rstrb1[11]: begin
// 							rdata1 <= vregs[raddr1[4:0]][383: 352];
// 							rdata2 <= vregs[raddr2[4:0]][383: 352];
// 							// $display("rdata1: %x, rdata2:%x, raddr1:%d, raddr2:%d, rstrb:%b, time:%d",rdata1, rdata2, raddr1, raddr2, vec_rstrb1, $time);
// 						   end
// 			vec_rstrb1[12]: begin
// 							rdata1 <= vregs[raddr1[4:0]][415: 384];
// 							rdata2 <= vregs[raddr2[4:0]][415: 384];
// 							// $display("rdata1: %x, rdata2:%x, raddr1:%d, raddr2:%d, rstrb:%b, time:%d",rdata1, rdata2, raddr1, raddr2, vec_rstrb1, $time);
// 						   end
// 			vec_rstrb1[13]: begin
// 							rdata1 <= vregs[raddr1[4:0]][447: 416];
// 							rdata2 <= vregs[raddr2[4:0]][447: 416];
// 							// $display("rdata1: %x, rdata2:%x, raddr1:%d, raddr2:%d, rstrb:%b, time:%d",rdata1, rdata2, raddr1, raddr2, vec_rstrb1, $time);
// 						   end
// 			vec_rstrb1[14]: begin
// 							rdata1 <= vregs[raddr1[4:0]][479:448];
// 							rdata2 <= vregs[raddr2[4:0]][479:448];
// 							// $display("rdata1: %x, rdata2:%x, raddr1:%d, raddr2:%d, rstrb:%b, time:%d",rdata1, rdata2, raddr1, raddr2, vec_rstrb1, $time);
// 						   end
// 			vec_rstrb1[15]: begin
// 							rdata1 <= vregs[raddr1[4:0]][511:480];
// 							rdata2 <= vregs[raddr2[4:0]][511:480];
// 							// $display("rdata1: %x, rdata2:%x, raddr1:%d, raddr2:%d, rstrb:%b, time:%d",rdata1, rdata2, raddr1, raddr2, vec_rstrb1, $time);
// 						   end
// 		endcase
// 		if(port3_en) begin
// 			case(1'b1)
// 			vec_rstrb1[0]: begin
// 							rdata3 <= vregs[raddr3[4:0]][31:0];
// 						   end
// 			vec_rstrb1[1]: begin
// 							rdata3 <= vregs[raddr3[4:0]][63:32];
// 						   end
// 			vec_rstrb1[2]: begin
// 							rdata3 <= vregs[raddr3[4:0]][95:64];
// 						   end
// 			vec_rstrb1[3]: begin
// 							rdata3 <= vregs[raddr3[4:0]][127: 96];
// 						   end
// 			vec_rstrb1[4]: begin
// 							rdata3 <= vregs[raddr3[4:0]][159: 128];
// 						   end
// 			vec_rstrb1[5]: begin
// 							rdata3 <= vregs[raddr3[4:0]][191: 160];
// 						   end
// 			vec_rstrb1[6]: begin
// 							rdata3 <= vregs[raddr3[4:0]][223: 192];
// 						   end
// 			vec_rstrb1[7]: begin
// 							rdata3 <= vregs[raddr3[4:0]][255: 224];
// 						   end
// 			vec_rstrb1[8]: begin
// 							rdata3 <= vregs[raddr3[4:0]][287:256];
// 						   end
// 			vec_rstrb1[9]: begin
// 							rdata3 <= vregs[raddr3[4:0]][319: 288];
// 						   end
// 			vec_rstrb1[10]: begin
// 							rdata3 <= vregs[raddr3[4:0]][351: 320];
// 						   end
// 			vec_rstrb1[11]: begin
// 							rdata3 <= vregs[raddr3[4:0]][383: 352];
// 						   end
// 			vec_rstrb1[12]: begin
// 							rdata3 <= vregs[raddr3[4:0]][415: 384];
// 						   end
// 			vec_rstrb1[13]: begin
// 							rdata3 <= vregs[raddr3[4:0]][447: 416];
// 						   end
// 			vec_rstrb1[14]: begin
// 							rdata3 <= vregs[raddr3[4:0]][479:448];
// 						   end
// 			vec_rstrb1[15]: begin
// 							rdata3 <= vregs[raddr3[4:0]][511:480];
// 						   end
// 			endcase
// 		end
// 	end
// 	// assign rdata1 = vregs[raddr1[4:0]];
// 	// assign rdata2 = vregs[raddr2[4:0]];
// 	// assign rdata3 = vregs[raddr3[4:0]];

// endmodule






//ALU
else if(instruction==instr_vmulvarp)begin 
                    // $display("Inside final condition, is_opA_neg:%b, time:%d", is_opA_neg, $time);
                    if(is_opA_neg[0])   peout[7:0] = -accumulator[7:0];
                    else                peout[7:0] = accumulator[7:0];
                    if(is_opA_neg[1])   peout[15:8] = -accumulator[15:8];
                    else                peout[15:8] = accumulator[15:8];
                    if(is_opA_neg[2])   peout[23:16] = -accumulator[23:16];
                    else                peout[23:16] = accumulator[23:16];
                    if(is_opA_neg[3])   peout[31:24] = -accumulator[31:24];
                    else                peout[31:24] = accumulator[31:24];
                end


else if(instruction==instr_vdotvarp)begin 
                    // $display("Inside final condition, is_opA_neg:%b, time:%d", is_opA_neg, $time);
                    if(is_opA_neg[0])   peout[7:0] = -accumulator[7:0] + opC[7:0];
                    else                peout[7:0] = accumulator[7:0] + opC[7:0];
                    if(is_opA_neg[1])   peout[15:8] = -accumulator[15:8] + opC[15:8];
                    else                peout[15:8] = accumulator[15:8] + opC[15:8];
                    if(is_opA_neg[2])   peout[23:16] = -accumulator[23:16] + opC[23:16];
                    else                peout[23:16] = accumulator[23:16] + opC[23:16];
                    if(is_opA_neg[3])   peout[31:24] = -accumulator[31:24] + opC[31:24];
                    else                peout[31:24] = accumulator[31:24] + opC[31:24];
                end


reg [3:0] is_opA_neg;

//To check whether opA has negative elements
always @(posedge clk) begin
    if(!reset) begin
        is_opA_neg = 0;
        temp = 0;
    end
    if(is_vap_instr && !temp) begin
        if(vap == 1) begin
            if(opA[0]) is_opA_neg[0] = 1;
            if(opA[8]) is_opA_neg[1] = 1;
            if(opA[16]) is_opA_neg[2] = 1;
            if(opA[24]) is_opA_neg[3] = 1;
            temp = 1;
        end
        else if(vap == 2) begin
            if(opA[1]) is_opA_neg[0] = 1;
            if(opA[9]) is_opA_neg[1] = 1;
            if(opA[17]) is_opA_neg[2] = 1;
            if(opA[25]) is_opA_neg[3] = 1;
            temp = 1;
        end
        else if(vap == 4) begin
            if(opA[3]) is_opA_neg[0] = 1;
            if(opA[11]) is_opA_neg[1] = 1;
            if(opA[19]) is_opA_neg[2] = 1;
            if(opA[27]) is_opA_neg[3] = 1;
            temp = 1;
        end
    end
end