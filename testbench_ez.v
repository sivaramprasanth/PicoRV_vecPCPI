// This is free and unencumbered software released into the public domain.
//
// Anyone is free to copy, modify, publish, use, compile, sell, or
// distribute this software, either in source code form or as a compiled
// binary, for any purpose, commercial or non-commercial, and by any
// means.

`timescale 1 ns / 1 ps

module testbench;
	reg clk = 1;
	reg resetn = 0;
	wire trap;

	always #5 clk = ~clk;

	initial begin
		if ($test$plusargs("vcd")) begin
			$dumpfile("testbench.vcd");
			$dumpvars(0, testbench);
		end
		//I think, here he is waiting for 100 clock cycles before executing the instructions i.e making the resetn as 1
		repeat (100) @(posedge clk);
		resetn <= 1; //After 100 cycles he makes the resetn as 1
		repeat (1000) @(posedge clk);
		$finish;
	end

	wire mem_valid;
	wire mem_instr;
	reg mem_ready;
	wire [31:0] mem_addr;
	wire [31:0] mem_wdata;
	wire [3:0] mem_wstrb;
	reg  [31:0] mem_rdata;

	always @(posedge clk) begin
		if (mem_valid && mem_ready) begin
			if (mem_instr)
				$display("ifetch 0x%08x: 0x%08x", mem_addr, mem_rdata);
			else if (mem_wstrb)
				$display("write  0x%08x: 0x%08x (wstrb=%b)", mem_addr, mem_wdata, mem_wstrb);
			else
				$display("read   0x%08x: 0x%08x", mem_addr, mem_rdata);
		end
	end

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

	reg [31:0] memory [0:255]; //Memory of 256 indices.

	initial begin
	    //x0 always contains 0 in riscV
//		memory[0] = 32'h 3fc00093; //       li      x1,1020 ---> It's actually ADDI x1,x0,1020
//		memory[1] = 32'h 0000a023; //       sw      x0,0(x1)
		// memory[2] = 32'h 0000a103; // loop: lw      x2,0(x1)
		// memory[3] = 32'h 00110113; //       addi    x2,x2,1
		// memory[4] = 32'h 0020a023; //       sw      x2,0(x1)
		// memory[5] = 32'h ff5ff06f; //       j       <loop>

//	    x0 always contains 0 in riscV
		memory[0] = 32'h 3fc00093; //       li      x1,1020 ---> It's actually ADDI x1,x0,1020
		memory[1] = 32'h 00200113; //       li      x2,2 ---> It's actually ADDI x2,x0,2
		memory[2] = 32'h 02208233; //    MUL x4,x1,x2
//		memory[3] = 32'h 0220c233; //    DIV x4,x1,x2    x1/x2 in x4
	end

	always @(posedge clk) begin
		mem_ready <= 0;
		if (mem_valid && !mem_ready) begin
			if (mem_addr < 1024) //Checking if the address is valid
			begin
				mem_ready <= 1;
				mem_rdata <= memory[mem_addr >> 2]; //To convert it to word address, we are right shifting by 2
				if (mem_wstrb[0]) memory[mem_addr >> 2][ 7: 0] <= mem_wdata[ 7: 0];
				if (mem_wstrb[1]) memory[mem_addr >> 2][15: 8] <= mem_wdata[15: 8];
				if (mem_wstrb[2]) memory[mem_addr >> 2][23:16] <= mem_wdata[23:16];
				if (mem_wstrb[3]) memory[mem_addr >> 2][31:24] <= mem_wdata[31:24];
			end
			/* add memory-mapped IO here */
		end
	end
endmodule
