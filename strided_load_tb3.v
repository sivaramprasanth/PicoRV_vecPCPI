// This is free and unencumbered software released into the public domain.
//
// Anyone is free to copy, modify, publish, use, compile, sell, or
// distribute this software, either in source code form or as a compiled
// binary, for any purpose, commercial or non-commercial, and by any
// means.
//SEW of 16 bits

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
		repeat (500) @(posedge clk);
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
		
		// memory[0] = 32'h 01000113; //---> to set vl(no of elements) as 16 (Addi x2,x0,4) 
		// memory[1] = 32'b 00000000010000010111001001010111; //Vsetvli x4,x2, LMUL=1 E16 --->  0 00000000100 00010 111 00100 1010111 ---> 00817257 (sew - 32)
		// memory[2] = 32'h 19000093; //addi x1,x0,400
        // memory[3] = 32'h 00200393; //addi x7,x0,4  --> Loading the stride
		// //31 29 28 26 25 24    20 19  15 14 12 11    7 6     0
		// // nf | mop | vm |  rs2 |  rs1 | width |  vd  |0000111| VL* strided
        // memory[4] = 32'b 00001010011100001111000010000111; //  000 010 1 00111 00001 111 00001 0000111 vlse.v v1, (x1), x7
		// // memory[5] = 32'h 1a400093; //addi x1,x0,420
		// memory[5] = 32'h 19000093; //addi x1,x0,400
		// memory[6] = 32'b 00001010011100001111000100000111; //  000 010 1 00111 00001 111 00010 0000111 vlse.v v2, (x1), x7
		// // memory[7] = 32'h 1E400093; //addi x1,x0,484   ---> Loading 00000093 repetetively into v8
		// memory[7] = 32'h 1b800093; //addi x1,x0,440
		// memory[8] = 32'b 00001010011100001111010000000111; //  000 010 1 00111 00001 111 01000 0000111 vlse.v v8, (x1), x7
		// // memory[9] = 32'b 00000010001000001000010001010111; //  000000 1 00010 00001 000 01000 1010111  vadd.vv v8,v2,v1 --> 0x0a100157
		// // memory[9] = 32'b 10010110001000001000010001010111; //  000000 1 00010 00001 000 01000 1010111  vmul.vv v8,v2,v1 --> 0x09620857
		// memory[9] = 32'b 11100110001000001000010001010111; //  111001 1 00010 00001 000 01000 1010111  vdot.vv v8, v2, v1
        // // Strided store instruction with the same stride in x7 i.e 8
		// memory[10] = 32'h 32000093; //addi x1,x0,800
		// memory[11] = 32'b 00001010011100001111010000100111; //  000 010 1 00111 00001 111 01000 0100111 vsse.v v8, (x1), x7


		memory[0] = 32'h 00300113; //---> to set vl as 3 (Addi x2,x0,4)
		memory[1] = 32'b 00000000010000010111001001010111; //Vsetvli x4,x2, LMUL=1 E16 --->  0 00000000100 00010 111 00100 1010111 ---> 00817257 (sew - 32)
		memory[2] = 32'h 19000093; //addi x1,x0,400
		memory[3] = 32'h 00100393; //addi x7,x0,1  --> Loading the stride
        memory[3] = 32'h 00400393; //li x7,1(x0) --> addi (loading stride)
        memory[4] = 32'b 00001010011100001111000010000111; //  000 010 1 00111 00001 111 00001 0000111 vlse.v v1, (x1), x7
        memory[5] = 32'h 00000413; //li x8,0(x0) --> stride for second matrix
		memory[6] = 32'h 19100193; //li x3,401(x0) --> addi
        memory[7] = 32'h 19200293; //li x5,402(x0) --> addi
        memory[8] = 32'h 19300513; //li x10,403(x0) --> addi
        memory[9] = 32'h 1a000313; //li x6,416(x0) --> addi
		memory[10] = 32'h 1b400213; //li x4,436(x0) --> addi (for loading 0 into Vd)
        memory[11] = 32'b 00001010011100011111000100000111; //  000 010 1 00111 00011 111 00010 0000111 vlse.v v2, (x3), x7
        memory[12] = 32'b 00001010011100101111000110000111; //  000 010 1 00111 00101 111 00011 0000111 vlse.v v3, (x5), x7
        memory[13] = 32'b 00001010011101010111001010000111; //  000 010 1 00111 01010 111 00101 0000111 vlse.v v5, (x10), x7 (rs8 --> stride)
        memory[14] = 32'b 00001010100000110111001000000111; //  000 010 1 01000 00110 111 00100 0000111 vlse.v v4, (x6), x8 (rs8 --> stride)
		//Loading 0 into Vd initially
		memory[15] = 32'b 00001010100000100111010000000111; //  000 010 1 01000 00100 111 01000 0000111 vlse.v v8, x4, x8 (rs8 --> stride)
		memory[16] = 32'b 11100110010000001000010001010111; //  111001 1 00100 00001 000 01000 1010111 (e6408457) vdot.vv v8, v4, v1
		memory[17] = 32'h 1a400313; //li x6,420(x0) --> addi
        memory[18] = 32'b 00001010100000110111001000000111; //  00001010100000110111001000000111 vlse.v v4, (x6), x8 (rs8 --> stride)
		memory[19] = 32'b 11100110010000010000010001010111; //  111001 1 00100 00010 000 01000 1010111 vdot.vv v8, v4, v2 
		memory[20] = 32'h 1a800313; //li x6,424(x0) --> addi
        memory[21] = 32'b 00001010100000110111001000000111; //  00001010100000110111001000000111 vlse.v v4, (x6), x8
		memory[22] = 32'b 11100110010000011000010001010111; //  111001 1 00100 00011 000 01000 1010111 vdot.vv v8, v4, v3 
        memory[23] = 32'h 1ac00313; //li x6,428(x0) --> addi
        memory[24] = 32'b 00001010100000110111001000000111; //  00001010100000110111001000000111 vlse.v v4, (x6), x8
		memory[25] = 32'b 11100110010000101000010001010111; //  111001 1 00100 00101 000 01000 1010111 vdot.vv v8, v4, v5
		
		//second column (Will be stores in V9), loading 0 into Vd (v9) initially
		memory[26] = 32'b 00001010100000100111010010000111; //  000 010 1 01000 00100 111 01001 0000111 vlse.v v9, x4, x8 (rs8 --> stride)
		memory[27] = 32'h 1a100313; //li x6,417(x0) --> addi
        memory[28] = 32'b 00001010100000110111001000000111; //  00001010100000110111001000000111 vlse.v v4, (x6), x8
		memory[29] = 32'b 11100110010000001000010011010111; //  111001 1 00100 00001 000 01001 1010111 vdot.vv v9, v4, v1
		memory[30] = 32'h 1a500313; //li x6,421(x0) --> addi
        memory[31] = 32'b 00001010100000110111001000000111; //  00001010100000110111001000000111 vlse.v v4, (x6), x8
		memory[32] = 32'b 11100110010000010000010011010111; //  111001 1 00100 00010 000 01001 1010111 vdot.vv v9, v4, v2 
		memory[33] = 32'h 1a900313; //li x6,425(x0) --> addi
        memory[34] = 32'b 00001010100000110111001000000111; //  00001010100000110111001000000111 vlse.v v4, (x6), x8
		memory[35] = 32'b 11100110010000011000010011010111; //  111001 1 00100 00011 000 01001 1010111 vdot.vv v9, v4, v3 
        memory[36] = 32'h 1ad00313; //li x6,429(x0) --> addi
        memory[37] = 32'b 00001010100000110111001000000111; //  00001010100000110111001000000111 vlse.v v4, (x6), x8
		memory[38] = 32'b 11100110010000101000010011010111; //  111001 1 00100 00101 000 01001 1010111 vdot.vv v9, v4, v5



		/*
        Matrix is stored in memory from addr 100 - addr 108
        ex: [1 2 3; 
			 4 5 6; 
			 7 8 9]
        mem[100] = 1;
        mem[101] = 2; etc
		*/
        //First matrix
        /*
        [ 1 2 3 
          4 5 6
          7 8 9 ]
        */
		memory[100] =32'h00020001; 
		memory[101] =32'h00040003;
		memory[102] =32'h00060005;
		memory[103] =32'h00080007;
		memory[104] =32'h00000009;
        
        //Contains the second matrix
        /*
        [3 7
         4 8
         5 9
         6 10]
        */
        memory[104] = 32'h 00000703;
		memory[105] = 32'h 00000804;
        memory[106] = 32'h 00000905;
		memory[107] = 32'h 00000a06;



		// memory[100] = 32'h 04030201;
		// memory[101] = 32'h 08070605;
        // memory[102] = 32'h 0c0b0a09;
		// memory[103] = 32'h 000f0e0d;
        // memory[104] = 32'h 14131211;
		// memory[105] = 32'h 18171615;
        // memory[106] = 32'h 1c1b1a19;
		// memory[107] = 32'h 101f1e1d;
        // memory[108] = 32'h 24232221; 
		// memory[109] = 32'h 28272625;

		//Data is 1 1 1 1 1 0 1 0 1 0 0 1 1 1 0 0
		//Data is 2 1 2 1 1 3 1 0 2 1 2 1 1 3 1 0
		// memory[100] = 32'b 00000000000000010000000000000001;    //1 1
		// memory[101] = 32'b 00000000000000010000000000000001;    //1 1
        // memory[102] = 32'b 00000000000000010000000000000000;    //1 0
		// memory[103] = 32'b 00000000000000010000000000000000;    //1 0
        // memory[104] = 32'b 00000000000000010000000000000000;	//1 0
		// memory[105] = 32'b 00000000000000000000000000000001;	//0 1
        // memory[106] = 32'b 00000000000000010000000000000001;    //1 1
		// memory[107] = 32'b 00000000000000000000000000000000;    //0 0
		
		// memory[100] = 32'b 00000000000000100000000000000001;    //2 1
		// memory[101] = 32'b 00000000000000100000000000000001;    //2 1
        // memory[102] = 32'b 00000000000000010000000000000011;    //1 3
		// memory[103] = 32'b 00000000000000010000000000000000;    //1 0
        // memory[104] = 32'b 00000000000000100000000000000001;	//2 1
		// memory[105] = 32'b 00000000000000100000000000000001;	//2 1
        // memory[106] = 32'b 00000000000000010000000000000011;    //1 3
		// memory[107] = 32'b 00000000000000010000000000000000;    //1 0

		// memory[108] = 32'b 00000001000000010000000000000000;	//1 1 0 0
        // memory[109] = 32'b 00000001000000000000000000000001;	//1 0 0 1 
		// memory[110] = 32'b 00000000000000010000000000000001;    //1 1
		// memory[111] = 32'b 00000000000000010000000000000001;    //1 1
        // memory[112] = 32'b 00000000000000010000000000000000;    //1 0
		// memory[113] = 32'b 00000000000000010000000000000000;    //1 0
		// memory[114] = 32'b 00000000000000010000000000000000;	//1 0
		// memory[115] = 32'b 00000000000000000000000000000001;	//0 1
        // memory[116] = 32'b 00000000000000010000000000000001;    //1 1
		// memory[117] = 32'b 00000000000000000000000000000000;    //0 0

		// memory[110] = 32'h 0000000a;
        // memory[111] = 32'h 00000014;
		// memory[112] = 32'h 0000001e;
		// memory[113] = 32'h 00000028;
        // memory[114] = 32'h 00000032;
		// memory[115] = 32'h 0000003c;
		// memory[116] = 32'h 00000046;
        // memory[117] = 32'h 00000050;
		// memory[118] = 32'h 0000005a;
		// memory[119] = 32'h 1000000a;
        // memory[120] = 32'h 11000014;
		
		// memory[121] = 32'h 1200000a;
        // memory[122] = 32'h 13000014;
		// memory[123] = 32'h 1400001e;
		// memory[124] = 32'h 15000028;
        // memory[125] = 32'h 16000032;
		// memory[126] = 32'h 1700003c;
		// memory[127] = 32'h 18000046;
        // memory[128] = 32'h 19000050;
		// memory[129] = 32'h 2000005a;
		// memory[130] = 32'h 21000050;
		// memory[131] = 32'h 2200005a;

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
				// if(vec_mem_wstrb == 4'b0)
					// $display("mem_addr: %d, mem_data: %x,mem_ready:%d, time:%d",vec_mem_addr, vec_mem_rdata,vec_mem_ready, $time);
				if(vec_mem_wstrb != 4'b0)
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