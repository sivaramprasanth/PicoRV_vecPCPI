
/***************************************************************
 * picorv32_pcpi_vec: A PCPI core that implements the vector instructions
 ***************************************************************/

 //In this module, we are usng element offset instead of byte offset
module picorv32_pcpi_vec #(
	//Bus width between ALU unit and coprocessor
	parameter [7:0] BUS_WIDTH = 8'B00100000, //Default is 32
	parameter [31:0] vlen = 32'h00000200 //No of bits in vector 
)(
	input clk, resetn,
	input pcpi_valid,
	input       [31:0]  pcpi_insn,
	input       [31:0]  pcpi_cpurs1, //Value in cpu_regs, used by vsetvl, vsetvli, vload, vstore
	input 		[31:0]  pcpi_cpurs2, //Value in cpu_regs, only used by vsetvl instrn
    output reg          pcpi_wr,
	output reg  [31:0]  pcpi_rd,
	output reg          pcpi_wait,
	output reg 	        pcpi_ready,

	//Memory interface
	input mem_ready, //Given by memory 
	input [31:0] mem_rdata, //data from memory for vload
	output reg mem_valid, //Assigned by coprocessor
	output reg [31:0] mem_addr, //Given to memory by coprocessor
	output reg [31:0] mem_wdata, //For store
	output reg [3:0]  mem_wstrb  //For store
);

	reg [31:0] reg_op1; //stores the value of pcpi_cpurs1
	reg [31:0] reg_op2; //stores the value of pcpi_cpurs2
	reg [31:0] temp_reg; //Used by vstore instruction

	//Memory Interface
	reg [1:0] mem_state;
	reg [1:0] mem_wordsize; //To tell whether to read/write whole word or a part of the word
	reg [31:0] mem_rdata_word; // //Stores the data depending on mem_wordsize from mem_rdata 
	reg [31:0] mem_rdata_word_next; //For strided loads
	reg [31:0] mem_wdata_word_next; //For strided stores

	reg mem_do_rdata; //Flag to read data
	reg mem_do_wdata; //Flag to write data

	wire mem_busy = |{mem_do_rdata, mem_do_wdata};

	reg [1:0] mem_str_state; //FSM for strided load.
	reg mem_str_ready;  //Used as the ready signal for strided load instruction
	reg [1:0] vstore_bit;  //Used to initialize the store instruction

//memory interface 
	always @(posedge clk) begin
		// (* full_case *)
		if(pcpi_valid) begin
			// $display("mem_rdata:%x, time:%d",mem_rdata,$time);
			case (mem_wordsize)
				0: begin
					mem_str_ready <= 0;
					if(mem_ready == 1) begin
						// // $display("Inside mem_interface, mem_wdata:%x, time:%d", vreg_rdata1_latched,$time);
						// if(instr_vstore || instr_vstore_str) begin
						// 	// mem_addr <= reg_op1; //This should work only is the instr is store
						// 	mem_wdata <= vreg_rdata1_latched; //changed here
						// 	mem_wstrb <= 4'b1111 & {4{mem_do_wdata}}; //mem_la_wstrb & {4{mem_la_write}}
						// end
						if(instr_vload || instr_vload_str || instr_vleuvarp || instr_vlesvarp) begin
							mem_rdata_word <= mem_rdata; //reads 32 bits
						end
						mem_str_ready <= 1; //str_ready will be 1 irrespective of the instruction
					end
				end
				1: begin
					mem_str_ready <= 0;
					if(instr_vload || instr_vload_str) begin
						if(SEW == 10'b0000001000) begin
							// $display("Inside mem_wsize1,ind1:%d, mem_rdata_word:%x, time:%d",ind1, mem_rdata_word, $time);
							mem_rdata_word[7:0]   <= ld_data[ind1 +: 8];
							mem_rdata_word[15:8]  <= ld_data[(ind1+reg_op2*8) +: 8];
							mem_rdata_word[23:16]  <= ld_data[(ind1+reg_op2*16) +: 8];
							mem_rdata_word[31:24] <= ld_data[(ind1+reg_op2*24) +: 8];
							mem_str_ready <= 1; 
							ind1 <= ind1 + 4*8*reg_op2;
						end
						else if(SEW == 10'b0000010000) begin
							// $display("Inside mem_wsize1,ind1:%d, mem_rdata_word:%x, time:%d",ind1, mem_rdata_word, $time);
							mem_rdata_word[15:0]   <= ld_data[ind1 +: 16];
							mem_rdata_word[31:16]  <= ld_data[(ind1+reg_op2*8) +: 16];
							mem_str_ready <= 1; 
							ind1 <= ind1 + 2*8*reg_op2;
						end
						//SEW is 32
						else if(SEW == 10'b0000100000) begin
							mem_rdata_word <= ld_data[ind1 +: 32];
							mem_str_ready <= 1; 
							ind1 <= ind1 + 8*reg_op2;
						end
					end
					else if(instr_vleuvarp || instr_vlesvarp) begin
						if(vap == 10'b0000000001) begin
							//The first addr will be ind1, next will be ind1+regop2*1*8, next reg_op2*2*8 etc
							mem_rdata_word[0]   <= ld_data[ind1 +: 1];
							mem_rdata_word[1]   <= ld_data[(ind1+reg_op2*8) +: 1];
							mem_rdata_word[2]   <= ld_data[(ind1+reg_op2*16) +: 1];
							mem_rdata_word[3]   <= ld_data[ind1+reg_op2*24 +: 1];
							mem_rdata_word[4]   <= ld_data[(ind1+reg_op2*32) +: 1];
							mem_rdata_word[5] <= ld_data[(ind1+reg_op2*40) +: 1];
							mem_rdata_word[6] <= ld_data[(ind1+reg_op2*48) +: 1];
							mem_rdata_word[7] <= ld_data[(ind1+reg_op2*56) +: 1];
							mem_rdata_word[8] <= ld_data[ind1+reg_op2*8*8 +: 1];
							mem_rdata_word[9] <= ld_data[(ind1+reg_op2*9*8) +: 1];
							mem_rdata_word[10] <= ld_data[(ind1+reg_op2*10*8) +: 1];
							mem_rdata_word[11] <= ld_data[ind1+reg_op2*11*8 +: 1];
							mem_rdata_word[12] <= ld_data[(ind1+reg_op2*12*8) +: 1];
							mem_rdata_word[13] <= ld_data[(ind1+reg_op2*13*8) +: 1];
							mem_rdata_word[14] <= ld_data[(ind1+reg_op2*14*8) +: 1];
							mem_rdata_word[15] <= ld_data[(ind1+reg_op2*15*8) +: 1];
							mem_rdata_word[16]   <= ld_data[ind1 +: 1];
							mem_rdata_word[17]   <= ld_data[(ind1+reg_op2*8) +: 1];
							mem_rdata_word[18]   <= ld_data[(ind1+reg_op2*16) +: 1];
							mem_rdata_word[19]   <= ld_data[ind1+reg_op2*24 +: 1];
							mem_rdata_word[20]   <= ld_data[(ind1+reg_op2*32) +: 1];
							mem_rdata_word[21] <= ld_data[(ind1+reg_op2*40) +: 1];
							mem_rdata_word[22] <= ld_data[(ind1+reg_op2*48) +: 1];
							mem_rdata_word[23] <= ld_data[(ind1+reg_op2*56) +: 1];
							mem_rdata_word[24] <= ld_data[ind1+reg_op2*8*8 +: 1];
							mem_rdata_word[25] <= ld_data[(ind1+reg_op2*9*8) +: 1];
							mem_rdata_word[26] <= ld_data[(ind1+reg_op2*10*8) +: 1];
							mem_rdata_word[27] <= ld_data[ind1+reg_op2*11*8 +: 1];
							mem_rdata_word[28] <= ld_data[(ind1+reg_op2*12*8) +: 1];
							mem_rdata_word[39] <= ld_data[(ind1+reg_op2*13*8) +: 1];
							mem_rdata_word[30] <= ld_data[(ind1+reg_op2*14*8) +: 1];
							mem_rdata_word[31] <= ld_data[(ind1+reg_op2*15*8) +: 1];
							mem_str_ready <= 1; 
							ind1 <= ind1 + 16*8*reg_op2;
						end
						else if(vap == 10'b0000000010) begin
							//The first addr will be ind1, next will be ind1+regop2*1*8, next reg_op2*2*8 etc
							mem_rdata_word[1:0]   <= ld_data[ind1 +: 2];
							mem_rdata_word[3:2]   <= ld_data[(ind1+reg_op2*8) +: 2];
							mem_rdata_word[5:4]   <= ld_data[(ind1+reg_op2*16) +: 2];
							mem_rdata_word[7:6]   <= ld_data[ind1+reg_op2*24 +: 2];
							mem_rdata_word[9:8]   <= ld_data[(ind1+reg_op2*32) +: 2];
							mem_rdata_word[11:10] <= ld_data[(ind1+reg_op2*40) +: 2];
							mem_rdata_word[13:12] <= ld_data[(ind1+reg_op2*48) +: 2];
							mem_rdata_word[15:14] <= ld_data[(ind1+reg_op2*56) +: 2];
							mem_rdata_word[17:16] <= ld_data[ind1+reg_op2*8*8 +: 2];
							mem_rdata_word[19:18] <= ld_data[(ind1+reg_op2*9*8) +: 2];
							mem_rdata_word[21:20] <= ld_data[(ind1+reg_op2*10*8) +: 2];
							mem_rdata_word[23:22] <= ld_data[ind1+reg_op2*11*8 +: 2];
							mem_rdata_word[25:24] <= ld_data[(ind1+reg_op2*12*8) +: 2];
							mem_rdata_word[27:26] <= ld_data[(ind1+reg_op2*13*8) +: 2];
							mem_rdata_word[29:28] <= ld_data[(ind1+reg_op2*14*8) +: 2];
							mem_rdata_word[31:30] <= ld_data[(ind1+reg_op2*15*8) +: 2];
							mem_str_ready <= 1; 
							ind1 <= ind1 + 16*8*reg_op2;
						end
						else if(vap == 10'b0000000011) begin
							//The first addr will be ind1, next will be ind1+regop2*1*8, next reg_op2*2*8 etc
							mem_rdata_word[2:0]   <= ld_data[ind1 +: 3];
							mem_rdata_word[5:3]   <= ld_data[(ind1+reg_op2*8) +: 3];
							mem_rdata_word[8:6]   <= ld_data[(ind1+reg_op2*16) +: 3];
							mem_rdata_word[11:9]   <= ld_data[ind1+reg_op2*24 +: 3];
							mem_rdata_word[14:12]   <= ld_data[(ind1+reg_op2*32) +: 3];
							mem_rdata_word[17:15] <= ld_data[(ind1+reg_op2*40) +: 3];
							mem_rdata_word[20:18] <= ld_data[(ind1+reg_op2*48) +: 3];
							mem_rdata_word[23:21] <= ld_data[(ind1+reg_op2*56) +: 3];
							mem_rdata_word[26:24] <= ld_data[ind1+reg_op2*8*8 +: 3];
							mem_rdata_word[29:27] <= ld_data[(ind1+reg_op2*9*8) +: 3];
							mem_rdata_word[31:30] <= ld_data[(ind1+reg_op2*10*8) +: 2];
							mem_str_ready <= 1; 
							ind1 <= ind1 + 10*8*reg_op2 + 2;
						end
						else if(vap == 10'b0000000100) begin
							// $display("Inside mem_wsize2,ind1:%d, mem_rdata_word:%x, time:%d",ind1, mem_rdata_word, $time);
							//The first addr will be ind1, next will be ind1+regop2*1*8, next reg_op2*2*8 etc
							mem_rdata_word[3:0]   <= ld_data[ind1 +: 4];
							mem_rdata_word[7:4]   <= ld_data[(ind1+reg_op2*8) +: 4];
							mem_rdata_word[11:8]  <= ld_data[(ind1+reg_op2*16) +: 4];
							mem_rdata_word[15:12] <= ld_data[ind1+reg_op2*24 +: 4];
							mem_rdata_word[19:16] <= ld_data[(ind1+reg_op2*32) +: 4];
							mem_rdata_word[23:20] <= ld_data[(ind1+reg_op2*40) +: 4];
							mem_rdata_word[27:24] <= ld_data[(ind1+reg_op2*48) +: 4];
							mem_rdata_word[31:28] <= ld_data[(ind1+reg_op2*56) +: 4];
							mem_str_ready <= 1; 
							ind1 <= ind1 + 8*8*reg_op2;
						end
						if(vap == 10'b0000001000) begin
							// $display("Inside mem_wsize1,ind1:%d, mem_rdata_word:%x, time:%d",ind1, mem_rdata_word, $time);
							mem_rdata_word[7:0]   <= ld_data[ind1 +: 8];
							mem_rdata_word[15:8]  <= ld_data[(ind1+reg_op2*8) +: 8];
							mem_rdata_word[23:16]  <= ld_data[(ind1+reg_op2*16) +: 8];
							mem_rdata_word[31:24] <= ld_data[(ind1+reg_op2*24) +: 8];
							mem_str_ready <= 1; 
							ind1 <= ind1 + 4*8*reg_op2;
						end
						if(vap == 10'b0000001010) begin
							// $display("Inside mem_wsize1,ind1:%d, mem_rdata_word:%x, time:%d",ind1, mem_rdata_word, $time);
							mem_rdata_word[9:0]   <= ld_data[ind1 +: 10];
							mem_rdata_word[19:10]  <= ld_data[(ind1+reg_op2*8) +: 10];
							mem_rdata_word[29:20]  <= ld_data[(ind1+reg_op2*16) +: 10];
							mem_rdata_word[31:30] <= ld_data[(ind1+reg_op2*24) +: 2];
							mem_str_ready <= 1; 
							ind1 <= ind1 + 4*8*reg_op2;
						end
                        if(vap == 10'b0000001100) begin
							// $display("Inside mem_wsize1,ind1:%d, mem_rdata_word:%x, time:%d",ind1, mem_rdata_word, $time);
							mem_rdata_word[11:0]   <= ld_data[ind1 +: 12];
							mem_rdata_word[23:12]  <= ld_data[(ind1+reg_op2*8) +: 12];
							mem_rdata_word[31:24]  <= ld_data[(ind1+reg_op2*16) +: 8];
							mem_str_ready <= 1; 
							ind1 <= ind1 + 2*8*reg_op2 + 8;
						end
					end
				end
				2: begin

				end
				3: begin
                   
                end
			endcase
		end
	end

	//Instruction decoder
	reg instr_vsetvli, instr_vsetvl, instr_vsetprecision; //Vec instrn to set the csr reg values
	reg instr_vload,instr_vload_str,instr_vstore, instr_vstore_str;   //Vec load and store instr
	reg instr_vdot,instr_vadd; //For dot product and addition
	wire is_vec_instr; //To check whether the forwarded instruction to coprocessor is vector instruction or not

	//Instructions for variable bit precision
	reg instr_vleuvarp, instr_vlesvarp, instr_vseuvarp, instr_vsesvarp;
	
	assign is_vec_instr = |{instr_vsetvli,instr_vsetvl,instr_vsetprecision,instr_vload,instr_vload_str, instr_vleuvarp, instr_vlesvarp, instr_vstore, instr_vstore_str, instr_vseuvarp, instr_vsesvarp, instr_vdot,instr_vadd};

	reg [4:0] decoded_vs1, decoded_vs2, decoded_vd; //For vect instrns
	reg [10:0] decoded_vimm; //For vect instrns

	//Instruction decoder
	always@(posedge clk) begin
		if (!resetn || !pcpi_valid) begin
			instr_vsetvl <= 0;
			instr_vsetvli <= 0;
			instr_vsetprecision <= 0;
			instr_vload <= 0;
			instr_vload_str <= 0;
			instr_vstore <= 0;
			instr_vstore_str <= 0; //For strided store
			instr_vleuvarp <= 0; 
			instr_vlesvarp <= 0;
			instr_vseuvarp <= 0;
			instr_vsesvarp <= 0;
			instr_vdot  <= 0;
			instr_vadd  <= 0;
			mem_str_state <= 0; //default value of mem_str_state for strided loads
			mem_str_ready <= 0; //default value for mem_str_ready
			vstore_bit <= 2'b00; //Resetting the vstore bit
			mem_wstrb <= 4'b0;
		end
		else begin
			// $display("Inside decode stage, instrn: %x, time: %d", pcpi_insn, $time);
			decoded_vs1 <= pcpi_insn[19:15];
			decoded_vs2 <= pcpi_insn[24:20];
			decoded_vd  <= pcpi_insn[11:7];
			decoded_vimm <= pcpi_insn[30:20];
			//Load and store currently supports only unit stride
			instr_vload   <= (pcpi_insn[24:20]==5'b00000) && (pcpi_insn[28:26]==3'b000) && (pcpi_insn[6:0] == 7'b0000111); // NF not supported
			instr_vload_str <= (pcpi_insn[28:26]==3'b010) && (pcpi_insn[14:12]==3'b111) && (pcpi_insn[6:0] == 7'b0000111); //Strided load
			instr_vleuvarp <= (pcpi_insn[29:26]==4'b0000 && pcpi_insn[14:12]==3'b111 && pcpi_insn[6:0] == 7'b1011011 && pcpi_insn[31:30] == 2'b00); //Unit strided load for vbp
			instr_vlesvarp <= (pcpi_insn[29:26]==4'b0001 && pcpi_insn[14:12]==3'b111 && pcpi_insn[6:0] == 7'b1011011 && pcpi_insn[31:30] == 2'b00); //Strided load for vbp
			instr_vstore  <= (pcpi_insn[24:20]==5'b00000) && (pcpi_insn[28:26]==3'b000) && (pcpi_insn[6:0] == 7'b0100111); // only unit stride supported,NF not supported 
			instr_vstore_str <= (pcpi_insn[28:26]==3'b010) && (pcpi_insn[14:12]==3'b111) && (pcpi_insn[6:0] == 7'b0100111); //Strided store, works for 32 bit SEW
			instr_vsetvl  <= (pcpi_insn[14:12]==3'b111) && (pcpi_insn[31]==1) && (pcpi_insn[6:0] == 7'b1010111); 
			instr_vsetvli <= (pcpi_insn[14:12]==3'b111) && (pcpi_insn[31]==0) && (pcpi_insn[6:0] == 7'b1010111);
			instr_vsetprecision <= (pcpi_insn[14:12]==3'b111 && pcpi_insn[6:0] == 7'b1011011 && pcpi_insn[31:25] == 7'b1000000);
			instr_vdot    <= (pcpi_insn[31:26]==6'b111001) && (pcpi_insn[14:12] == 3'b000) && (pcpi_insn[6:0]==7'b1010111);
			instr_vadd    <= (pcpi_insn[31:26]==6'b000000) && (pcpi_insn[14:12] == 3'b000) && (pcpi_insn[6:0]==7'b1010111);
			v_enc_width   <= (instr_vload || instr_vstore || instr_vload_str || instr_vstore_str || instr_vleuvarp || instr_vlesvarp)? pcpi_insn[14:12]:0;
		end
	end
	
	//For vector instructions
	wire [31:0] vcsr_vlenb = 32'h00000080; //vector reg length in bytes
	reg [31:0] vcsr_vlen;
	reg [31:0] vcsr_vtype; //Can be updated by vsetvli or vsetvl instr
	reg [31:0] vcsr_vl;    //No of elements to be updated by a vector instrn
	reg [31:0] vcsr_vap; //Register used by custom inst for variable precision
	wire [3:0] vap;  //Contains the SEW for variable precision (1 to 15)
	reg [31:0] vcsr_elem_offset; //Used for strided load and store (Against RISC V specs)
	assign vap = vcsr_vap[3:0]; //Storing the SEW in vap
	wire [2:0]vsew  = vcsr_vtype[4:2]; //Encoding for the no of bits in an element (part of vtype)
	wire [1:0]vlmul = vcsr_vtype[1:0]; //Encoding for the no of vec regs in a group (part of vtype)
	wire [9:0] SEW;
	assign SEW  = (4*(2<<vsew));
    wire [6:0] sew_bytes; //No of bytes in a SEW - 1 (Used to calculate final addr for ld and st)
	wire [1:0] sew_bytes_vap; //No of bytes in a SEW (Used for vap load and store instructions)
    assign sew_bytes = ((1<<vsew)-1);  //Used for vector load and store instruction
	assign sew_bytes_vap = (vap-1) >> 2; //if vap <= 8, it should be 0 else 1
	wire [31:0]LMUL = (1 << (vlmul)); //No of vector regs in a group (2^vlmul)
	wire [31:0]VLMAX = (512/SEW)*LMUL; //Represents the max no of elements that can be operated on with a vec instrn

	//Variables used by vload and vstore
    reg [5:0] mem_read_no; //No of memory reads for load instrn
    reg [9:0] init_addr; //Used for vector ld and st
    reg [9:0] final_addr; //Used for vector ld and st
	reg [5:0] vecldstrcnt;
	reg [5:0] var_vlen; //Used to load the vector reg partially
	reg [31:0] vreg_op1;  //Data from v1 for all vector instr
	reg [31:0] vreg_op2;  //Data from v2 for all vector instr
	reg [31:0] vreg_op3;  //Data from vd for dot product instruction
	reg [31:0] vreg_rdata1_latched; //data readfrom vector regs, used for vstore
	reg [2:0] v_enc_width;
	reg [10:0] v_membits; //Contains the number of bits to be loaded from memory
	reg [15:0] vecregs_wstrb_temp; //Used to store write strobe
    reg [511:0] ld_data;
    reg [9:0] cnt;
	reg temp_var = 0; //Used for a reset condition for vector load
    reg [9:0] ind1; //Used for vec load
    reg [5:0] no_words; //No of words to read
	reg [4:0] bits_remaining; //Bits remaining after words are loaded
    reg [5:0] temp_count; //Used for indexing (strb in vector regs)
	//Variables used by Vadd and Vdot
	reg [7:0] elem_n;
	reg [31:0] vecrs1;
	reg [31:0] vecrs2;
	reg [31:0] vecrs3; //For dit product
	reg [31:0] vecrd;
	reg [511:0] valu_out; //Stores the output of ALU

	reg set_mem_do_rdata;
	reg set_mem_do_wdata;

	// For vector instructions
	//vector register bank for vector instructions
	reg vecregs_write;  //Write enable
	reg [4:0] vecregs_waddr; 
	reg [15:0] vecregs_wstrb;
	reg [4:0] vecregs_raddr1;
	reg [4:0] vecregs_raddr2;
	reg [4:0] vecregs_raddr3; //For vdot instruction
	reg port3_en;  //Used for the third register
	reg [15:0] vecregs_rstrb1;
	reg [31:0] vecregs_wdata;
	wire [31:0] vecregs_rdata1;
	wire [31:0] vecregs_rdata2;
	wire [31:0] vecregs_rdata3; //For vdot instruction
		
	vector_regs vecreg_inst (
		.clk(clk),
		.wen(resetn && vecregs_write),
		.waddr(vecregs_waddr),
		.vec_wstrb(vecregs_wstrb),
		.raddr1(vecregs_raddr1),
		.raddr2(vecregs_raddr2),
		.raddr3(vecregs_raddr3),
		.port3_en(port3_en),
		.vec_rstrb1(vecregs_rstrb1),
		.wdata(vecregs_wdata),
		.rdata1(vecregs_rdata1),
		.rdata2(vecregs_rdata2),
		.rdata3(vecregs_rdata3)
		);	

	//Main state machine
	localparam cpu_state_fetch   = 8'b10000000;
	localparam cpu_state_ld_rs1  = 8'b01000000;
	localparam cpu_state_exec    = 8'b00100000;
	// localparam cpu_state_ld_rs2 = 8'b00010000;
	localparam cpu_state_stmem   = 8'b00010000;
	// localparam cpu_state_shift  = 8'b00000100;
	localparam cpu_state_ldmem   = 8'b00001000;
    localparam cpu_state_ldmem2  = 8'b00000100;
	// localparam cpu_state_stmem  = 8'b00001000;

	reg [7:0] cpu_state;
	reg latched_vstore;  //Added for vector instruction
	reg latched_stalu;	//This wil be 1 if the result to be written to register is the output of ALU

	always @(posedge clk) begin
		// $display("inside vstore, latched_vs:%b, time:%d", latched_vstore, $time);
		case(1'b1)
			//latched_vstore will be 1 for 1 clk cycle
			latched_vstore: begin
				if(!instr_vstore && !instr_vstore_str) begin //If the instr is not store
					vecregs_wstrb <= vecregs_wstrb_temp;
					vecregs_wdata <= latched_stalu ? valu_out:vreg_op1;
					vecregs_write <= 1;  //wen for load instruction
				end
			end
		endcase
	end


	always@(posedge clk) begin
		set_mem_do_rdata = 0;
		set_mem_do_wdata = 0;
		if(!resetn || !pcpi_valid || !is_vec_instr) begin
			// $display("Inside reset condition, time: %d, reset:%b, pcpi_valid:%b, is_vec_instr: %b", $time, resetn, pcpi_valid, is_vec_instr);
			pcpi_rd <= 0;
			vecrd <= 0;
			pcpi_wait <= 0;
	        pcpi_ready <= 0;
            pcpi_wr <= 0;
			cpu_state <= cpu_state_fetch; //Default state
			latched_stalu <= 0;
			latched_vstore <= 0;
			vecregs_write <= 0; //If pcpi_valid is 0, make wen as 0
			mem_valid <= 0; //If pcpi_valid is 0, mem_valid = 0
			temp_reg <= 0; //Making the store data as 0 in default
			mem_do_wdata <= 0;
			vecregs_rstrb1 <= 16'b0;
			vecregs_wstrb_temp <= 16'b0;
		end
		else begin
			pcpi_wait <= 1;
			vecregs_write = 0; //Will be 1 for only 1 clk cycle
			pcpi_ready <= 0; //Will be 1 for only 1 clk cycle
			latched_vstore <= 0; 
			case(cpu_state)
				cpu_state_fetch: begin
					$display("Inside fetch state, mem_do_wdata:%b, time: %d", mem_do_wdata, $time);
					mem_valid <= 0; //Not initializing the memory in fetch stage
					$display("Inside fetch, pcpi_rs1:%d", pcpi_cpurs1);
					reg_op1 <= pcpi_cpurs1;
					reg_op2 <= pcpi_cpurs2;
					// latched_vstore <= 0;  //latched_vstore will be 1 for 1 clk cycle
					latched_stalu <= 0;
					mem_wordsize <= 0; //Has to write/read 32 bit data
					cpu_state <= cpu_state_ld_rs1;
				end
				cpu_state_ld_rs1: begin
					case(1'b1)
						(instr_vsetvli || instr_vsetvl):  begin	
							$display("Inside vsetvli condition, time:%d", $time);
							vcsr_vl <= (decoded_vs1!=5'b00000) ? pcpi_cpurs1:vcsr_vl;
							vcsr_vtype <= (instr_vsetvl) ? pcpi_cpurs2:decoded_vimm;
							// $display("pcpi_cpurs1: %d",pcpi_cpurs1);
							// $display("Inside coproc, vcsr_vl = %d, vcsr_vtype = %b, decoded_vs1 = %b", vcsr_vl,vcsr_vtype, decoded_vs1);
							cpu_state <= cpu_state_ld_rs1;
							pcpi_ready <= 1;
							pcpi_wait <= 0;
						end
						(instr_vsetprecision): begin
							$display("Inside vsetprecsion condition, vcsr_vap:%d, elem_off:%d, time:%d", reg_op1, reg_op2, $time);
							vcsr_vap <= reg_op1;
							vcsr_elem_offset <= reg_op2;
							cpu_state <= cpu_state_ld_rs1;
							pcpi_ready <= 1;
							pcpi_wait <= 0;
						end
						(instr_vload): begin
							$display("Inside v_load condition, time:%d", $time);
							// mem_valid <= 1;
							cpu_state <= cpu_state_ldmem;
                            init_addr <= reg_op1 >> 2; //Initial word addrr
                            final_addr <= ((reg_op1 + (vcsr_vl-1)*reg_op2 + sew_bytes) >> 2);  //Calculates the word addr of final byte 
							vecregs_waddr <= decoded_vd;
							vecregs_raddr1 <= decoded_vs1;
							mem_str_ready <= 0; //Initial value of mem_str_ready
							mem_str_state <= 2'b00; //Initial state for strided load
							//Updating mem_bits depending on vcsr_vl and v_enc_width(i.e instr[14:12])
							v_membits <= vcsr_vl * ((v_enc_width==3'b000)? 8:(v_enc_width==3'b101)?16:(v_enc_width==3'b110)?32:SEW); 
                            no_words <= ((vcsr_vl*SEW)>>5); //No of words to read
                            cnt <= 0;
							temp_var <= 0;
                            temp_count <= 0;
                            ind1 <= (reg_op1[1:0] << 3); //byte addr to bit addr
							//Number of bits to read in each cycle
							// if(SEW == 10'b0000100000)
							mem_wordsize = 0;
						end
						(instr_vload_str): begin
							$display("Inside v_load_stride condition, time:%d", $time);
							// mem_valid <= 1;
							cpu_state <= cpu_state_ldmem;
                            init_addr <= reg_op1 >> 2; //Initial word addrr
                            final_addr <= ((reg_op1 + (vcsr_vl-1)*reg_op2 + sew_bytes) >> 2);  //Calculates the word addr of final byte 
							vecregs_waddr <= decoded_vd;
							vecregs_raddr1 <= decoded_vs1;
							mem_str_ready <= 0; //Initial value of mem_str_ready
							mem_str_state <= 2'b00; //Initial state for strided load
							//Updating mem_bits depending on vcsr_vl and v_enc_width(i.e instr[14:12])
							v_membits <= vcsr_vl * ((v_enc_width==3'b000)? 8:(v_enc_width==3'b101)?16:(v_enc_width==3'b110)?32:SEW); 
                            no_words <= ((vcsr_vl*SEW)>>5); //No of words to read
                            cnt <= 0;
							temp_var <= 0;
                            temp_count <= 0;
                            ind1 <= (reg_op1[1:0] << 3); //byte addr to bit addr
							//Number many bits to read in each cycle
							// if(SEW == 10'b0000100000)
							mem_wordsize = 0;
						end
						(instr_vleuvarp): begin
							$display("Inside vap_unit_load condition, time:%d", $time);
							// mem_valid <= 1;
							cpu_state <= cpu_state_ldmem;
                            init_addr <= reg_op1 >> 2; //Initial word addrr
                            final_addr <= ((reg_op1 + (vcsr_vl-1)*reg_op2 + sew_bytes_vap) >> 2);  //Calculates the word addr of final byte 
							vecregs_waddr <= decoded_vd;
							vecregs_raddr1 <= decoded_vs1;
							mem_str_ready <= 0; //Initial value of mem_str_ready
							mem_str_state <= 2'b00; //Initial state for strided load
							//Updating mem_bits depending on vcsr_vl and v_enc_width(i.e instr[14:12])
							v_membits <= vcsr_vl * ((v_enc_width==3'b000)? 8:(v_enc_width==3'b101)?16:(v_enc_width==3'b110)?32:vap); 
                            no_words <= ((vcsr_vl*vap)>>5); //No of words to read
                            cnt <= 0;
							temp_var <= 0;
                            temp_count <= 0;
                            ind1 <= (reg_op1[1:0] << 3); //byte addr to bit addr
							//Number of bits to read in each cycle
							// if(SEW == 10'b0000100000)
							mem_wordsize = 0;
						end
						(instr_vlesvarp): begin
							$display("Inside vap_load_stride condition, time:%d", $time);
							cpu_state <= cpu_state_ldmem;
                            init_addr <= reg_op1 >> 2; //Initial word addrr
                            final_addr <= ((reg_op1 + (vcsr_vl-1)*reg_op2 + sew_bytes_vap) >> 2);  //Calculates the word addr of final byte 
							vecregs_waddr <= decoded_vd;
							vecregs_raddr1 <= decoded_vs1;
							mem_str_ready <= 0; //Initial value of mem_str_ready
							mem_str_state <= 2'b00; //Initial state for strided load
							//Updating mem_bits depending on vcsr_vl and v_enc_width(i.e instr[14:12])
							v_membits <= vcsr_vl * ((v_enc_width==3'b000)? 8:(v_enc_width==3'b101)?16:(v_enc_width==3'b110)?32:vap); 
                            no_words <= ((vcsr_vl*vap)>>5); //No of words to read
                            cnt <= 0;
							temp_var <= 0;
                            temp_count <= 0;
                            ind1 <= (reg_op1[1:0] << 3); //byte addr to bit addr
							//Number many bits to read in each cycle
							// if(SEW == 10'b0000100000)
							mem_wordsize = 0;
						end
						(instr_vstore): begin
							$display("Inside v_store condition, time:%d", $time);
							// mem_valid <= 1;
							vstore_bit <= 2'b00;
							cpu_state <= cpu_state_stmem;
							var_vlen <= 17 - ((vcsr_vl * ((v_enc_width==3'b000)? 8:(v_enc_width==3'b101)?16:(v_enc_width==3'b110)?32:SEW)) >> 5);//right shift to divide with 32
							$display("var_length:%d, time:%d",var_vlen, $time);
							vecregs_waddr <= decoded_vd;
							vecregs_raddr1 <= decoded_vd; //specifies v register holding store data
							vecregs_raddr2 <= decoded_vs2;
							vecregs_rstrb1 <= 16'b1;  //To read the first word
							mem_str_ready <= 0; //Initial value of mem_str_ready
							mem_str_state <= 2'b00; //Initial state for strided load
							//Updating mem_bits depending on vcsr_vl and v_enc_width(i.e instr[14:12])
							v_membits <= vcsr_vl * ((v_enc_width==3'b000)? 8:(v_enc_width==3'b101)?16:(v_enc_width==3'b110)?32:SEW); 
							if(SEW == 10'b0000100000) begin
								vecldstrcnt <= 17;
								mem_wordsize = 0;
							end
							if((SEW == 10'b0000001000)) begin
								vecldstrcnt <= 18;
								reg_op2 = 1;
								mem_wordsize = 3;
							end
						end
						(instr_vstore_str): begin
							$display("Inside v_store_stride condition, time:%d", $time);
							// mem_valid <= 1;
							vstore_bit <= 2'b00;
							cpu_state <= cpu_state_stmem;
							var_vlen <= 17 - ((vcsr_vl * ((v_enc_width==3'b000)? 8:(v_enc_width==3'b101)?16:(v_enc_width==3'b110)?32:SEW)) >> 5);//right shift to divide with 32
							$display("var_length:%d, time:%d",var_vlen, $time);
							vecregs_waddr <= decoded_vd;
							vecregs_raddr1 <= decoded_vd; //specifies v register holding store data
							vecregs_raddr2 <= decoded_vs2;
							vecregs_rstrb1 <= 16'b1;  //To read the first word
							mem_str_ready <= 0; //Initial value of mem_str_ready
							mem_str_state <= 2'b00; //Initial state for strided load
							//Updating mem_bits depending on vcsr_vl and v_enc_width(i.e instr[14:12])
							v_membits <= vcsr_vl * ((v_enc_width==3'b000)? 8:(v_enc_width==3'b101)?16:(v_enc_width==3'b110)?32:SEW); 
							if(SEW == 10'b0000100000) begin
								vecldstrcnt <= 17;
								mem_wordsize <= 0;
							end
							if((SEW == 10'b0000001000) && reg_op2 >= 4) begin
								vecldstrcnt <= 18;
								mem_wordsize <= 2;
							end
							if((SEW == 10'b0000001000) && reg_op2 < 4) begin
								vecldstrcnt <= 18;
								mem_wordsize <= 3;
							end
						end
						(instr_vadd): begin
							vecregs_waddr = decoded_vd;
							vecregs_raddr1 = decoded_vs1;
							vecregs_raddr2 = decoded_vs2;
							// vreg_op1 <= vecregs_rdata1;
							// vreg_op2 <= vecregs_rdata2;
							elem_n = 0;
							v_membits <= vcsr_vl * ((v_enc_width==3'b000)? 8:(v_enc_width==3'b101)?16:(v_enc_width==3'b110)?32:SEW); 
							cpu_state <= cpu_state_exec;
						end
						(instr_vdot): begin
							vecregs_waddr = decoded_vd;
							vecregs_raddr1 = decoded_vs1;
							vecregs_raddr2 = decoded_vs2;
							vecregs_raddr3 = decoded_vd; //For dot product
							// vreg_op1 <= vecregs_rdata1;
							// vreg_op2 <= vecregs_rdata2;
							// vreg_op3 <= vecregs_rdata3; //Used for dot product
							elem_n = 0;
							v_membits <= vcsr_vl * ((v_enc_width==3'b000)? 8:(v_enc_width==3'b101)?16:(v_enc_width==3'b110)?32:SEW); 
							cpu_state <= cpu_state_exec;
						end
					endcase
				end 
				
				cpu_state_ldmem: begin
                    //Calculate for the first time and make valid as 1 
                    if(temp_var == 0) begin
						$display("Inside cnt=0 condition, no_words:%d, time: %d", no_words, $time);
						bits_remaining <= v_membits[4:0]; //remainder after dividing mem_bits with 32
						if(v_membits[4:0] > 0)
							no_words <= no_words+1;
                       	mem_read_no <= final_addr - init_addr + 1; //No of mem_reads required
					   	mem_valid <= 1; //Making the valid bit 1 after changing the address for the first time
						temp_var <= 1; //So that it enters this block only once
                    end
                    // $display("final_addr: %d, init_addr: %d, No of mem_reads: %d, time:%d",final_addr, init_addr, mem_read_no, $time);
					if(mem_read_no >= 1) begin
                        if(mem_ready == 1) begin //This is how memory works
                            reg_op1 = reg_op1 + 4;
                        end
						if((mem_str_ready == 1)) begin 
                            ld_data[cnt +: 32] <= mem_rdata_word;  //Selects 32 bits starting from cnt
							$display("Inside mem_addr:%x, mem_rdata: %x, mem_read_no: %d, time:%d",reg_op1, mem_rdata_word, mem_read_no, $time);
                            mem_read_no <= mem_read_no - 1;
                            cnt <= cnt + 32;
							//If we are loading the last word, then go to ldmem2 stage to load the data into vector reg
							if(mem_read_no == 1) begin
                                $display("Inside mem_str_ready,no_words:%d, ld_data :%x, time:%d",no_words, ld_data, $time);
								// if(SEW == 10'b0000100000)
								//Irrespective of SEW, the mem_wordsize will be 1 (used in mem FSM)
								mem_wordsize <= 1;
                                mem_valid <= 0;
                                mem_str_ready <= 0; //Will be made 1 again in mem_wordsize FSM
								cpu_state <= cpu_state_ldmem2;
							end
						end
					end
					mem_addr = reg_op1; //Updating the mem_addr
				end
                cpu_state_ldmem2: begin
                    if(no_words > 0) begin
                       if(mem_str_ready == 1) begin
                        //    $display("Inside mem_str_ready, ld_data:%x, mem_rdata:%x, time: %d", ld_data[95:0], mem_rdata_word, $time);
                            vreg_op1 <= mem_rdata_word;
                            vecregs_wstrb_temp <= 1 << temp_count; //left shift temp count digits
                            temp_count <= temp_count+1;
                            latched_vstore <= 1;
                            no_words <= no_words-1;
                        end 
                    end
                    if(no_words == 0) begin
						// $display("V_membits:%d, bits remaining after words: %d, time:%d", v_membits, bits_remaining, $time);
						mem_str_ready <= 0; //Will be made 1 again in mem_wordsize FSM
						cpu_state <= cpu_state_fetch;
						pcpi_wait <= 0;
						pcpi_ready <= 1;
                    end
                end
			endcase
		end
		if (set_mem_do_rdata)
			mem_do_rdata <= 1;
		if (set_mem_do_wdata)
			mem_do_wdata <= 1;
	end
endmodule
