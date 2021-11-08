// This is free and unencumbered software released into the public domain.
//
// Anyone is free to copy, modify, publish, use, compile, sell, or
// distribute this software, either in source code form or as a compiled
// binary, for any purpose, commercial or non-commercial, and by any
// means.
//SEW of 8

`timescale 1 ns / 1 ps

module testbench;

    parameter enable_vec = 1;
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

    //For vector coprocessor
    wire  vec_mem_valid;
    reg  vec_mem_ready;
	wire [31:0] vec_mem_addr;
	wire [31:0] vec_mem_wdata;
	wire [3:0]  vec_mem_wstrb;
	reg  [31:0] vec_mem_rdata;
	
    
    // For vector instructions
	wire	 pcpi_vec_valid; //Valid for vector co-processor
	wire	[31:0] pcpi_vec_insn;  //insn to be sent to vector co-processor
	wire    [31:0] pcpi_vec_rs1; //Value stored in cpu rs1 transferred to pcpi core
	wire 	[31:0] pcpi_vec_rs2; //Only used by vselvl instrn
	wire  	[31:0] pcpi_vec_rd; //The output of pcpi_co-processor
	wire 		   pcpi_vec_wait;
	wire 		   pcpi_vec_ready; //Flag to notify if the instruction is executed or not
	wire		   pcpi_vec_wr;	 //Flag to notify the main processor to write to cpu reg
	

	picorv32 #(
        .ENABLE_VEC(enable_vec)
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
		.mem_rdata   (mem_rdata  ),

        //For vector coprocessor
        .pcpi_vec_valid(pcpi_vec_valid),
        .pcpi_vec_insn(pcpi_vec_insn),
        .pcpi_vec_rs1(pcpi_vec_rs1),
        .pcpi_vec_rs2(pcpi_vec_rs2),
        .pcpi_vec_rd(pcpi_vec_rd),
        .pcpi_vec_wait(pcpi_vec_wait),
        .pcpi_vec_ready(pcpi_vec_ready),
        .pcpi_vec_wr(pcpi_vec_wr)
	);

    // For vector instructions
	generate if (enable_vec) begin
		picorv32_pcpi_vec #(
			
		) pcpi_vec(
			.clk(clk),
			.resetn(resetn),
			.pcpi_valid(pcpi_vec_valid),
			.pcpi_insn(pcpi_vec_insn),
			.pcpi_cpurs1(pcpi_vec_rs1),
			.pcpi_cpurs2(pcpi_vec_rs2),
			.pcpi_wr(pcpi_vec_wr),
			.pcpi_rd(pcpi_vec_rd),
			.pcpi_wait(pcpi_vec_wait),
			.pcpi_ready(pcpi_vec_ready), //Becomes 1 if the output of co-processor is ready
            //Memory interface
            .mem_valid(vec_mem_valid),
            .mem_ready(vec_mem_ready),
            .mem_addr(vec_mem_addr),
            .mem_wdata(vec_mem_wdata),
            .mem_wstrb(vec_mem_wstrb),
            .mem_rdata(vec_mem_rdata)
		);
	end else begin
		assign pcpi_vec_wr = 0;
		assign pcpi_vec_rd = 32'bx;
		assign pcpi_vec_wait = 0;
		assign pcpi_vec_ready = 0;
	end endgenerate



	reg [31:0] memory [0:255];

	initial begin
		for(i = 0;i <256; i=i+1)
			memory[i] = 32'h 00000093; //NOP
		
		//Vl is the number of elements to modify every time
        memory[0] = 32'h 00400113; //---> to set vap as 4 (Addi x2,x0,4) 
        memory[1] = 32'h 00100093; //---> to set elem_off as 1 (Addi x1,x0,1)  000000000001 00000 000 00001 0010011
        // funct  rs2(off)  vap  fun        opcode
        //1000000 00001    00010 111 00000 1011011
        memory[2] = 32'b 10000000000100010111000001011011; //Setting the value of vap as 2
		// Ox 00017257
<<<<<<< HEAD
        memory[3] = 32'h 02000113; //Addi x2, x0, 8  (Set Vl as 32)
		memory[4] = 32'b 00000000000000010111001001010111; //Vsetvli x4,x2, LMUL=1 E4 --->  0 00000000000 00010 111 00100 1010111 ---> 00817257 (sew - 4)
		memory[5] = 32'h 19000093; //addi x1,x0,400
        memory[6] = 32'h 00300393; //addi x7,x0,1  --> byte offset (stride)
		//31 30 29 26 25  24 	 20 19  15 14 12 11    7 6     0
		// 00 | 0000 | vm | 00000 |  rs1 | width |  vd  |1011011| vleu_varp
		// 00 | 0001 | vm |  rs2  |  rs1 | width |  vd  |1011011| vles_varp
        memory[7] = 32'b 00000110011100001111000011011011; //  00 0001 1 00111 00001 111 00001 1011011 vles_varp.v v1, (x1), x7
		// memory[8] = 32'h 25800093; //addi x1,x0,600
		//31 30 29 26 25  24 	  20 19  15 14 12 11    7 6     0
		// 01 | 0000 | vm | 00000  |  rs1 | width |  vd  |1011011| vseu_varp
		// 01 | 0001 | vm |  rs2   |  rs1 | width |  vd  |1011011| vses_varp
		// memory[9] = 32'b 01000110011100001111000011011011; //  01 0001 1 00111 00001 111 00001 1011011 vses_varp.v v1, (x1), x7
=======
        memory[3] = 32'h 00800113; //Addi x2, x0, 8  (Set Vl as 8)
		memory[4] = 32'b 00000000000000010111001001010111; //Vsetvli x4,x2, LMUL=1 E8 --->  0 00000000000 00010 111 00100 1010111 ---> 00817257 (sew - 8)
		memory[5] = 32'h 19000093; //addi x1,x0,400
        memory[6] = 32'h 00100393; //addi x7,x0,1  --> byte offset (stride)
		//31 30 29 26 25 24 	20 19  15 14 12 11    7 6     0
		// 00 | 0000 | vm |  rs2 |  rs1 | width |  vd  |1011011| vles_varp
		// 00 | 0001 | vm |  rs2 |  rs1 | width |  vd  |1011011| vles_varp
        memory[7] = 32'b 00000110011100001111000011011011; //  000 010 1 00111 00001 111 00001 0000111 vles_varp.v v1, (x1), x7
>>>>>>> 278fc3216d763ae96c7b1099f0f85e5982c1206c
		
		//Data in memory to be loaded
		memory[100] = 32'b 00000100000000110000001000000001;
		memory[101] = 32'b 00001000000001110000011000000101;
        memory[102] = 32'b 00001100000010110000101000001001;
		memory[103] = 32'b 00000000000011110000111000001101;
        memory[104] = 32'h 14131211;
		memory[105] = 32'h 18171615;
        memory[106] = 32'h 1c1b1a19;
		memory[107] = 32'h 101f1e1d;
        memory[108] = 32'h 00000009; 
		memory[109] = 32'h 00000000;
	
		memory[110] = 32'h 0000000a;
        memory[111] = 32'h 00000014;
		memory[112] = 32'h 0000001e;
		memory[113] = 32'h 00000028;
        memory[114] = 32'h 00000032;
		memory[115] = 32'h 0000003c;
		memory[116] = 32'h 00000046;
        memory[117] = 32'h 00000050;
		memory[118] = 32'h 0000005a;
<<<<<<< HEAD
		memory[119] = 32'h 1000000a;
        memory[120] = 32'h 11000014;
		
		memory[121] = 32'h 1200000a;
        memory[122] = 32'h 13000014;
		memory[123] = 32'h 1400001e;
		memory[124] = 32'h 15000028;
        memory[125] = 32'h 16000032;
		memory[126] = 32'h 1700003c;
		memory[127] = 32'h 18000046;
        memory[128] = 32'h 19000050;
		memory[129] = 32'h 2000005a;
		memory[130] = 32'h 21000050;
		memory[131] = 32'h 2200005a;
=======

>>>>>>> 278fc3216d763ae96c7b1099f0f85e5982c1206c
		//Vtype reg is 00000000000, vtype[1:0] -> vlmul[1:0] (sets LMUL value)
		//							vtype[4:2] -> vsew[2:0] (sets SEW value)
		//							vtype[6:5] -> vdiv[1:0] (used by EDIV extension)
		//							vl gets it's value from 00010 reg i.e it gets 16
	end

	always @(posedge clk) begin
		mem_ready <= 0;
		if (mem_valid && !mem_ready) begin
			if (mem_addr < 1024) begin
				mem_ready <= 1;
				mem_rdata <= memory[mem_addr >> 2];
				// $display("Time:%d ,Data read from memory: %x, addr: %d", $time,mem_rdata, mem_addr);
				if (mem_wstrb[0]) memory[mem_addr >> 2][ 7: 0] <= mem_wdata[ 7: 0];
				if (mem_wstrb[1]) memory[mem_addr >> 2][15: 8] <= mem_wdata[15: 8];
				if (mem_wstrb[2]) memory[mem_addr >> 2][23:16] <= mem_wdata[23:16];
				if (mem_wstrb[3]) memory[mem_addr >> 2][31:24] <= mem_wdata[31:24];
			end
			/* add memory-mapped IO here */
		end
	end


	always @(posedge clk) begin
		vec_mem_ready <= 0;
		if (vec_mem_valid && !vec_mem_ready) begin
			if (vec_mem_addr < 1024) begin
				vec_mem_rdata <= memory[vec_mem_addr >> 2];
				vec_mem_ready <= 1;
				// $display("mem_addr: %d, mem_data: %x,mem_ready:%d, time:%d",vec_mem_addr, vec_mem_rdata,vec_mem_ready, $time);
				// $display("Time:%d ,Data read from memory: %x, addr: %d", $time,vec_mem_rdata, vec_mem_addr);
				if(|(vec_mem_wstrb) == 1)
					$display("Data written to memory addr: %d is %x, mem_wstrb: %b, time:%d", vec_mem_addr, vec_mem_wdata, vec_mem_wstrb, $time);
				if (vec_mem_wstrb[0]) memory[vec_mem_addr >> 2][ 7: 0] <= vec_mem_wdata[ 7: 0];
				if (vec_mem_wstrb[1]) memory[vec_mem_addr >> 2][15: 8] <= vec_mem_wdata[15: 8];
				if (vec_mem_wstrb[2]) memory[vec_mem_addr >> 2][23:16] <= vec_mem_wdata[23:16];
				if (vec_mem_wstrb[3]) memory[vec_mem_addr >> 2][31:24] <= vec_mem_wdata[31:24];
			end
			/* add memory-mapped IO here */
		end
	end
endmodule


// li x1, 0x3BC
// li x3, 0x320
// li x2, 0x020
// vsetvli x4, x2, e8, m1
// vle.v v0, x1
// vle.v v1, x3
// vmult.vv v2, v0, v1
// vse.v v2, x1
// nop
// nop