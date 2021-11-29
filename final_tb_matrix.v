// This is free and unencumbered software released into the public domain.
//
// Anyone is free to copy, modify, publish, use, compile, sell, or
// distribute this software, either in source code form or as a compiled
// binary, for any purpose, commercial or non-commercial, and by any
// means.
//SEW of 32 bits

`timescale 1 ns / 1 ps

module testbench;

    parameter enable_vec = 1;
	integer i;
    reg [31:0] cycle_count;
	reg clk = 1;
	reg resetn = 0;
	wire trap;
    always @(posedge clk)begin
		if(resetn)
		    cycle_count <= cycle_count + 1;
	    else 
	        cycle_count <= 0;
	end
	always #5 clk = ~clk;

	initial begin
		if ($test$plusargs("vcd")) begin
			$dumpfile("testbench.vcd");
			$dumpvars(0, testbench);
		end
		repeat (1) @(posedge clk);
		resetn <= 1;
		repeat (1500) @(posedge clk);
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



    reg [31:0] memory [0:2047];
    integer eoi_word;
	
	integer data_space = 400;//1600 (i.e, 400th word)

	initial begin
		for(i = 0;i <256; i=i+1)
			memory[i] = 32'h 00000093; //NOP
		
	//Matrix Multiplication
	memory[0] = 32'h 64000093 ;                                                        // lui x1,0x640		// address for matrix A
	memory[1] = 32'h 65400113 ;                                                        // lui x2,0x654		// address for matrix B
	memory[2] = 32'h 00400193 ;                                                        // li x3,0x003		// stride = 3x3
	memory[3] = 32'h 00e00213 ;                                                        // li x4,0x004		// precision
	memory[4] = 32'h 00000293 ;                                                        // li x5,0x000		// element offset
	memory[5] = 32'b 00000000010000011111000111010111;                                //   vsetvli x3,x3,e16      //vsetvli 0|000000|vsew[4:2]|00|rs1|111|rd|1010111 (zimm[4:2] vsew)
	memory[6] = 32'b 10000000010100100111000001011011;                                //   vsetprecision x4,x5 (->vap,eloffset) 1000000|00101|00100|111|00000|1011011
	// Loading data unit stride load for Nbit 
	memory[7] = 32'b 00000000000000010111000011011011;                                //   vleu_varp v1,(x2),x0 00|00000|00000|00010|111|00001|1011011     
	memory[8] = 32'h 00328293 ;                                                        // addi x5,x5,0x003		// element offset
	memory[9] = 32'b 10000000010100100111000001011011;                                //   vsetprecision x4,x5 (->vap,eloffset) 1000000|00101|00100|111|00000|1011011
	
	memory[10] = 32'b 00000000000000010111000101011011;                                //   vleu_varp v2,(x2),x0 00|00000|00000|00010|111|00010|1011011     
	memory[11] = 32'h 00328293 ;                                                        // addi x5,x5,0x003		// element offset
	memory[12] = 32'b 10000000010100100111000001011011;                                //   vsetprecision x4,x5 (->vap,eloffset) 1000000|00101|00100|111|00000|1011011
	
	memory[13] = 32'b 00000000000000010111000111011011;                                //   vleu_varp v3,(x2),x0 00|00000|00000|00010|111|00011|1011011     
	memory[14] = 32'h 00328293 ;                                                        // addi x5,x5,0x003		// element offset
	memory[15] = 32'b 10000000010100100111000001011011;                                //   vsetprecision x4,x5 (->vap,eloffset) 1000000|00101|00100|111|00000|1011011
	//load and multiply 

    memory[16] = 32'b 00011010000000001101001000000111;                                //   vlshv v4,(x1),x0     000|110|1|00000|00001|101|00100|0000111
	memory[17] = 32'h 00208093 ;                                                       //   addi x1,x1,0x002		
	memory[18] = 32'b 00011010000000001101001010000111;                                //   vlshv v5,(x1),x0     000|110|1|00000|00001|101|00100|0000111
	memory[19] = 32'h 00208093 ;                                                       //   addi x1,x1,0x002		
	memory[20] = 32'b 00011010000000001101001100000111;                                //   vlshv v6,(x1),x0     000|110|1|00000|00001|101|00100|0000111
	memory[21] = 32'h 00208093 ;                                                       //   addi x1,x1,0x002		
	memory[22] = 32'b 00001000101001010000010101010111;                                //   vsub_vv v10,v10,v10 000010|0|01010|01010|000|01010|1010111
	memory[23] = 32'b 11000100000100100000001111011011;                                //   vmul_varp v7,v4,v1 1100010|00001|00100|000|00111|1011011
    memory[24] = 32'b 00000000011101010000010101010111;                                //   vadd_vv v10,v10,v7 000000|0|00111|01010|000|01010|1010111
	memory[25] = 32'b 11000100001000101000001111011011;                                //   vmul_varp v7,v5,v2 1100010|00010|00101|000|00111|1011011
    memory[26] = 32'b 00000000011101010000010101010111;                                //   vadd_vv v10,v10,v7 000000|0|00111|01010|000|01010|1010111
	memory[27] = 32'b 11000100001100110000001111011011;                                //   vmul_varp v7,v6,v3 1100010|00011|00110|000|00111|1011011
    memory[28] = 32'b 00000000011101010000010101010111;                                //   vadd_vv v10,v10,v7 000000|0|00111|01010|000|01010|1010111
	
	
    memory[29] = 32'b 00011010000000001101001000000111;                                //   vlshv v4,(x1),x0     000|110|1|00000|00001|101|00100|0000111
	memory[30] = 32'h 00208093 ;                                                       //   addi x1,x1,0x002		
	memory[31] = 32'b 00011010000000001101001010000111;                                //   vlshv v5,(x1),x0     000|110|1|00000|00001|101|00100|0000111
	memory[32] = 32'h 00208093 ;                                                       //   addi x1,x1,0x002		
	memory[33] = 32'b 00011010000000001101001100000111;                                //   vlshv v6,(x1),x0     000|110|1|00000|00001|101|00100|0000111
	memory[34] = 32'h 00208093 ;                                                       //   addi x1,x1,0x002		
	memory[35] = 32'b 00001000101101011000010111010111;                                //   vsub_vv v11,v11,v11 000010|0|01011|01011|000|01011|1010111
	memory[36] = 32'b 11000100000100100000001111011011;                                //   vmul_varp v7,v4,v1 1100010|00001|00100|000|00111|1011011
    memory[37] = 32'b 00000000011101011000010111010111;                                //   vadd_vv v11,v11,v7 000000|0|00111|01011|000|01011|1010111
	memory[38] = 32'b 11000100001000101000001111011011;                                //   vmul_varp v7,v5,v2 1100010|00010|00101|000|00111|1011011
    memory[39] = 32'b 00000000011101011000010111010111;                                //   vadd_vv v11,v11,v7 000000|0|00111|01011|000|01011|1010111
	memory[40] = 32'b 11000100001100110000001111011011;                                //   vmul_varp v7,v6,v3 1100010|00011|00110|000|00111|1011011
    memory[41] = 32'b 00000000011101011000010111010111;                                //   vadd_vv v11,v11,v7 000000|0|00111|01011|000|01011|1010111
    
    
    memory[42] = 32'b 00011010000000001101001000000111;                                //   vlshv v4,(x1),x0     000|110|1|00000|00001|101|00100|0000111
	memory[43] = 32'h 00208093 ;                                                       //   addi x1,x1,0x002		
	memory[44] = 32'b 00011010000000001101001010000111;                                //   vlshv v5,(x1),x0     000|110|1|00000|00001|101|00100|0000111
	memory[45] = 32'h 00208093 ;                                                       //   addi x1,x1,0x002		
	memory[46] = 32'b 00011010000000001101001100000111;                                //   vlshv v6,(x1),x0     000|110|1|00000|00001|101|00100|0000111
	memory[47] = 32'h 00208093 ;                                                       //   addi x1,x1,0x002		
	memory[48] = 32'b 00001000110001100000011001010111;                                //   vsub_vv v12,v12,v12 000010|0|01100|01100|000|01100|1010111
	memory[49] = 32'b 11000100000100100000001111011011;                                //   vmul_varp v7,v4,v1 1100010|00001|00100|000|00111|1011011
    memory[50] = 32'b 00000000011101100000011001010111;                                //   vadd_vv v12,v12,v7 000000|0|00111|01100|000|01100|1010111
	memory[51] = 32'b 11000100001000110000001111011011;                                //   vmul_varp v7,v5,v2 1100010|00010|00101|000|00111|1011011
    memory[52] = 32'b 00000000011101100000011001010111;                                //   vadd_vv v12,v12,v7 000000|0|00111|01100|000|01100|1010111
	memory[53] = 32'b 11000100001100110000001111011011;                                //   vmul_varp v7,v6,v3 1100010|00011|00110|000|00111|1011011
    memory[54] = 32'b 00000000011101100000011001010111;                                //   vadd_vv v12,v12,v7 000000|0|00111|01100|000|01100|1010111
    eoi_word = 55;

		//Vtype reg is 00000000000, vtype[1:0] -> vlmul[1:0] (sets LMUL value)
		//							vtype[4:2] -> vsew[2:0] (sets SEW value)
		//							vtype[6:5] -> vdiv[1:0] (used by EDIV extension)
		//							vl gets it's value from 00010 reg i.e it gets 16
    //declaring memory data space
	//400th word - 1600 address 0x640 
	memory[data_space+0] =32'h00010002; 
	memory[data_space+1] =32'h00030004;
	memory[data_space+2] =32'h00050006;
	memory[data_space+3] =32'h00070008;
	memory[data_space+4] =32'h00090000;
	//405th word - 1620 address 0x654
	{memory[data_space+5],memory[data_space+6],memory[data_space+7],memory[data_space+8],memory[data_space+9]} = 160'b0000100010000110010000101001100011101000010010000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
	
	//160'b0000 1000 1000 011 0010 0001 0100 110 0011 1010 0001 001 0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
	
	end
	//3a12
	initial begin
		memory[0+eoi_word] = 32'h 00400093; 									//       li      x1,004
		memory[1+eoi_word] = 32'b 01000000000100000000000010110011 ; 		//       sub   x1,x0,x1   MC: 0100000|00001|00000|000|00001|0110011
		memory[2+eoi_word] = 32'h 0000a103; 									// 		 lw      x2,0(x1)
		memory[3+eoi_word] = 32'h 00000093; 									//       li      x1,000
		memory[4+eoi_word] = 32'h 00000093; 									//       li      x1,000
		memory[5+eoi_word] = 32'h 00000093; 									//       li      x1,000
	end

	always @(posedge clk) begin
		mem_ready <= 0;
		if (mem_valid && !mem_ready) begin
			if (mem_addr < 8192) begin
				mem_ready <= 1;
				mem_rdata <= memory[mem_addr >> 2];
				// $display("Time:%d ,Data read from memory: %x, addr: %d", $time,mem_rdata, mem_addr);
				if (mem_wstrb[0]) memory[mem_addr >> 2][ 7: 0] <= mem_wdata[ 7: 0];
				if (mem_wstrb[1]) memory[mem_addr >> 2][15: 8] <= mem_wdata[15: 8];
				if (mem_wstrb[2]) memory[mem_addr >> 2][23:16] <= mem_wdata[23:16];
				if (mem_wstrb[3]) memory[mem_addr >> 2][31:24] <= mem_wdata[31:24];
			end
            else if(mem_addr ==32'hFFFFFFFC ) begin
                $display("cycle complete:%d",cycle_count -17 );
                //17 cycles are subtracted to take into account the latency of reading a particular address(FFFFFFC) to mark end of program
                $finish;
			end
			/* add memory-mapped IO here */
		end
	end

	always @(posedge clk) begin
		vec_mem_ready <= 0;
		if (vec_mem_valid && !vec_mem_ready) begin
			if (vec_mem_addr < 8192) begin
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


