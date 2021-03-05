// This is free and unencumbered software released into the public domain.
//
// Anyone is free to copy, modify, publish, use, compile, sell, or
// distribute this software, either in source code form or as a compiled
// binary, for any purpose, commercial or non-commercial, and by any
// means.

`timescale 1 ns / 1 ps

module testbench;
	integer i;
	reg clk = 1;
	reg resetn = 0;
	wire trap;

	always #5 clk = ~clk;

	initial begin
		if ($test$plusargs("vcd")) begin
			$dumpfile("testbench.vcd");
			$dumpvars(0, testbench);
		end
		repeat (1) @(posedge clk);
		resetn <= 1;
		repeat (300) @(posedge clk);
		$finish;
	end
	integer ix;
	wire mem_valid;
	wire mem_instr;
	reg  mem_ready;
	wire [31:0] mem_addr;
	wire [31:0] mem_wdata;
	wire [3:0] mem_wstrb;
	reg  [31:0] mem_rdata;
	
	wire mem_delayed_ready;
	wire  [31:0] mem_delayed_rdata;
	
// 	always @(posedge clk) begin
// 		if (mem_valid && mem_ready) begin
// //			if (mem_instr)
// ////				$display("ifetch 0x%08x: 0x%08x", mem_addr, mem_rdata);
// //			else 
// 			if (mem_wstrb)
// 				$display("write  0x%08x: 0x%08x (wstrb=%b)", mem_addr, mem_wdata, mem_wstrb);
// 			else
// 				$display("read   0x%08x: 0x%08x", mem_addr, mem_rdata);
// 		end
// 	end

	picorv32 #(
	) uut (
		.clk         (clk        ),
		.resetn      (resetn     ),
		.trap        (trap       ),
		.mem_valid   (mem_valid  ),
		.mem_instr   (mem_instr  ),
		.mem_ready   (mem_ready  ),
		.mem_addr    (mem_addr   ),
		.mem_wdata   (mem_wdata  ),
		.mem_wstrb   (mem_wstrb  ),
		.mem_rdata   (mem_rdata  )
	);

	reg [31:0] memory [0:255];

	initial begin
		for(i = 0;i <256; i=i+1)
		memory[i] = 32'h 00000093;//NOP
		//memory[0] = 32'h 3bc00093; //       li      x1,239*4 = (956)
		//memory[1] = 32'h 0000a023; //  		sw      x0,0(x1)
		/*memory[0] = 32'h 3bc00093; //       li      x1,200*4 = (800)
		memory[1] = 32'h 00600113; //       li      x2,6
		memory[2] = 32'h 000171d7; //Vsetvli x3,x2, LMUL=1 E8 0 000000 000 00 00010 111 00011 1010111
		memory[3] = 32'b 00000010000000001111000000000111; //  000 000 1 00000 00001 111 00000 0000111 vl
		memory[4] = 32'b 00000010000000001111000000100111; //  000 000 1 00000 00001 111 00000 0100111 vs
		memory[5] = 32'h 00000093;
		*/
		memory[0] = 32'h 3bc00093; //       li      x1,239*4   (956)
		memory[1] = 32'h 32000193; //       li 		x3,200*4   (800)
		memory[2] = 32'h 04000113; //       li      x2,16   ---> to set vl as 16
		memory[3] = 32'b 00000000000000010111001001010111; //Vsetvli x4,x2, LMUL=1 E8 --->  0 00000000000 00010 111 00100 1010111 ---> 00017257
		memory[4] = 32'b 00000010000000001111000000000111; //  000 000 1 00000 00001 111 00000 0000111 vl v0, 0(x1) --> 0x0200f007
		memory[5] = 32'b 00000010000000011111000010000111; //  000 000 1 00000 00011 111 00001 0000111 vl v1, 0(x3) --> 0x0201f087
		memory[6] = 32'b 00000010000100000000000101010111; //  000000 1 00001 00000 000 00010 1010111 vadd v2 = v0 + v1 --> 0x02100157
		// memory[7] = 32'b 11100110000100000000000101010111; //  111001 1 00001 00000 000 00010 1010111 vdot V2 = v2 + v0*v1 --> 

		for(ix=0;ix<4;ix=ix+1)begin
			memory[200+4*ix]   = {{8'b00000001},{8'b00000010},{8'b00000011},{8'b00000100}};
			memory[200+4*ix+1] = {{8'b00000101},{8'b00000110},{8'b00000111},{8'b00001000}};
			memory[200+4*ix+2] = {{8'b00001001},{8'b00001010},{8'b00001011},{8'b00001100}};
			memory[200+4*ix+3] = {{8'b00001101},{8'b00001110},{8'b00001111},{8'b00010000}};
			memory[239+4*ix]   = {{8'b00010001},{8'b00010010},{8'b00010011},{8'b00010100}};
			memory[239+4*ix+1] = {{8'b00010101},{8'b00010110},{8'b00010111},{8'b00011000}};
			memory[239+4*ix+2] = {{8'b00011001},{8'b00011010},{8'b00011011},{8'b00011100}};
			memory[239+4*ix+3] = {{8'b00011101},{8'b00011110},{8'b00011111},{8'b00100000}};
		end

		// for(ix=0;ix<16;ix=ix+1)begin
		// 	memory[200+ix] = {{8'b00000001},{8'b00000011},{8'b00000010},{8'b00000100}};
		// 	memory[239+ix] = {{8'b00000001},{8'b00000011},{8'b00000010},{8'b00000100}};
		// end
		
		/*
		memory[1] = 32'b 00000000100000001111000101010111;//Vsetvli x2,x1, LMUL=1 E32 0 000000 010 00 00001 111 00010 1010111
		memory[2] = 32'b 10000000000100001111000101010111;//Vsetvl x2,x1,x1  1 000000 00001 00001 111 00010 1010111
		*/
	end

	always @(posedge clk) begin
		mem_ready <= 0;
		if (mem_valid && !mem_ready) begin
			if (mem_addr < 1024) begin
				mem_ready <= 1;
				mem_rdata <= memory[mem_addr >> 2];
				if (mem_wstrb[0]) memory[mem_addr >> 2][ 7: 0] <= mem_wdata[ 7: 0];
				if (mem_wstrb[1]) memory[mem_addr >> 2][15: 8] <= mem_wdata[15: 8];
				if (mem_wstrb[2]) memory[mem_addr >> 2][23:16] <= mem_wdata[23:16];
				if (mem_wstrb[3]) memory[mem_addr >> 2][31:24] <= mem_wdata[31:24];
			end
			/* add memory-mapped IO here */
		end
	end
endmodule