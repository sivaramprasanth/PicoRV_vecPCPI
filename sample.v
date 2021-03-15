/***************************************************************
 * picorv32_pcpi_vec: A PCPI core that implements the vector instructions
 ***************************************************************/
module picorv32_pcpi_vec (
	input clk, resetn,
	input pcpi_valid,
	input       [31:0]  pcpi_insn,
	input       [31:0]  pcpi_cpurs1, //Value in cpu_regs, used by vsetvl, vsetvli, vload, vstore
	input 		[31:0]  pcpi_cpurs2, //Value in cpu_regs, only used by vsetvl instrn
    output reg          pcpi_wr
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
	localparam vlen = 32'h00000200;   //Vlen is 512 bits
	reg reg_op1 = resetn ? pcpi_cpurs1 : 0;
	reg reg_op2 = resetn ? pcpi_cpurs2 : 0;

	//Memory Interface
	reg [1:0] mem_state;
	reg [1:0] mem_wordsize; //To tell whether to read/write whole word or a part of the word
	reg [31:0] mem_rdata_word; // //Stores the data depending on mem_wordsize from mem_rdata 

	reg mem_do_rdata; //Flag to read data
	reg mem_do_wdata; //Flag to write data

	wire mem_busy = |{mem_do_rdata, mem_do_wdata};

	always @* begin
		(* full_case *)
		case (mem_wordsize)
			0: begin
				// mem_la_wdata = reg_op2;
				// mem_la_wstrb = 4'b1111;
				mem_rdata_word = mem_rdata; //reads 32 bits
			end
			1: begin
				// mem_la_wdata = {2{reg_op2[15:0]}};
				// mem_la_wstrb = reg_op1[1] ? 4'b1100 : 4'b0011;
				case (reg_op1[1])
					1'b0: mem_rdata_word = {16'b0, mem_rdata[15: 0]};
					1'b1: mem_rdata_word = {16'b0, mem_rdata[31:16]};
				endcase
			end
			2: begin
				// mem_la_wdata = {4{reg_op2[7:0]}};
				// mem_la_wstrb = 4'b0001 << reg_op1[1:0];
				case (reg_op1[1:0])
					2'b00: mem_rdata_word = {24'b0, mem_rdata[ 7: 0]};
					2'b01: mem_rdata_word = {24'b0, mem_rdata[15: 8]};
					2'b10: mem_rdata_word = {24'b0, mem_rdata[23:16]};
					2'b11: mem_rdata_word = {24'b0, mem_rdata[31:24]};
				endcase
			end
		endcase
	end

// //State machine for memory read and write
// 	always @(posedge clk) begin
// 		// if (mem_ready)
// 		// 		mem_valid <= 0;
// 		if (!resetn) begin
// 			mem_state <= 0;
// 			mem_valid <= 0;
// 		end
// 		else begin
// 			case (mem_state) //
// 				0: begin
// 					if (mem_do_prefetch || mem_do_rinst || mem_do_rdata) begin
// 						mem_valid <= !mem_la_use_prefetched_high_word;
// 						mem_instr <= mem_do_prefetch || mem_do_rinst;
// 						mem_wstrb <= 0;
// 						mem_state <= 1;
// 					end
// 					if (mem_do_wdata) begin
// 						mem_valid <= 1;
// 						mem_instr <= 0;
// 						mem_state <= 2;
// 					end
// 				end
// 				1: begin
// 					if (mem_xfer) begin
// 						if (COMPRESSED_ISA && mem_la_read) begin
// 							mem_valid <= 1;
// 							mem_la_secondword <= 1;
// 							if (!mem_la_use_prefetched_high_word)
// 								mem_16bit_buffer <= mem_rdata[31:16];
// 						end
// 						else begin
// 							mem_valid <= 0;
// 							mem_la_secondword <= 0;
// 							mem_state <= mem_do_rinst || mem_do_rdata ? 0 : 3;
// 						end
// 					end
// 				end
// 				2: begin
// 					if (mem_xfer) begin
// 						mem_valid <= 0;
// 						mem_state <= 0;
// 					end
// 				end
// 				3: begin
// 					if (mem_do_rinst) begin
// 						mem_state <= 0;
// 					end
// 				end
// 			endcase
// 		end
// 	end


	//Instruction decoder
	reg instr_vsetvli,instr_vsetvl; //Vec instrn to set the csr reg values
	reg instr_vload,instr_vstore;   //Vec load and store instr
	reg instr_vdot,instr_vadd; //For dot product and addition
	wire is_vec_instr;
	
	assign is_vec_instr = |{instr_vsetvli,instr_vsetvl,instr_vload,instr_vstore,instr_vdot,instr_vadd};

	reg [31:0] decoded_vs1, decoded_vs2, decoded_vd, decoded_vimm; //For vect instrns

	//Instruction decoder
	always@(posedge clk) begin
		if (!resetn || !pcpi_valid) begin
			instr_vsetvl <= 0;
			instr_vsetvli <= 0;
			instr_vload <= 0;
			instr_vstore <= 0;
			instr_vdot  <= 0;
			instr_vadd  <= 0;
		end
		else begin
			decoded_vs1 <= pcpi_insn[19:15];
			decoded_vs2 <= pcpi_insn[24:20];
			decoded_vd  <= pcpi_insn[11:7];
			decoded_vimm <= pcpi_insn[30:20];
			//Load and store currently supports only unit stride
			instr_vload   <= (pcpi_insn[24:20]==5'b00000) && (pcpi_insn[28:26]==3'b000) && pcpi_insn[6:0] == 7'b0000111; // only unit stride supported,NF not supported
			instr_vstore  <= (pcpi_insn[24:20]==5'b00000) && (pcpi_insn[28:26]==3'b000) && pcpi_insn[6:0] == 7'b0100111; // only unit stride supported,NF not supported 
			instr_vsetvl  <= (pcpi_insn[14:12]==3'b111) && (pcpi_insn[31]==1) && (pcpi_insn[6:0] == 7'b1010111); 
			instr_vsetvli <= (pcpi_insn[14:12]==3'b111) && (pcpi_insn[31]==0) && (pcpi_insn[6:0] == 7'b1010111);
			instr_vdot    <= (pcpi_insn[31:26]==6'b111001) && (pcpi_insn[14:12] == 3'b000) && (pcpi_insn[6:0]==7'b1010111);
			instr_vadd    <= (pcpi_insn[31:26]==6'b000000) && (pcpi_insn[14:12] == 3'b000) && (pcpi_insn[6:0]==7'b1010111);
			v_enc_width <= (is_vload || instr_vstore)? pcpi_insn[14:12]:0;
		end
	end
	
	//For vector instructions
	wire [31:0]vcsr_vlenb = 32'h00000080; //vector reg length in bytes
	reg [31:0]vcsr_vlen;
	reg [31:0]vcsr_vtype; //Can be updated by vsetvli or vsetvl instr
	reg [31:0]vcsr_vl;    //No of elements to be updated by a vector instrn

	assign SEW  = (4*(2<<vsew));
	wire [31:0]LMUL = (1 << (vlmul)); //No of vector regs in a group (2^vlmul)
	wire [31:0]VLMAX = (512/SEW)*LMUL; //Represents the max no of elements that can be operated on with a vec instrn
	wire [2:0]vsew  = vcsr_vtype[4:2]; //Encoding for the no of bits in an element (part of vtype)
	wire [1:0]vlmul = vcsr_vtype[1:0]; //Encoding for the no of vec regs in a group (part of vtype)

	//Variables used by vload and vstore
	reg [5:0] vecldstrcnt;
	// reg [5:0] vecreadcnt;
	reg [511:0] vreg_op1;  //Data from v1 for all vector instr
	reg [511:0] vreg_op2;  //Data from v2 for all vector instr
	reg [2:0] v_enc_width;
	reg [10:0] v_membits; //Contains the number of bits to be loaded from memory
	
	reg set_mem_do_rdata;
	reg set_mem_do_wdata;

	// For vector instructions
	//vector register bank for vector instructions
	reg vecregs_write;
	reg [4:0] vecregs_waddr;
	reg [4:0] vecregs_raddr1;
	reg [4:0] vecregs_raddr2;
	reg [511:0] vecregs_wdata;
	wire [511:0] vecregs_rdata1;
	wire [511:0] vecregs_rdata2;

	//Giving the addresses for vector access
	always@* begin
		vecregs_waddr <= decoded_rd;
		vecregs_raddr1 <= decoded_rs1;
		vecregs_raddr2 <= decoded_rs2;
	end
		
	vector_regs vecreg_inst (
		.clk(clk),
		.wen(resetn && vecregs_write),
		.waddr(vecregs_waddr),
		.raddr1(vecregs_raddr1),
		.raddr2(vecregs_raddr2),
		.wdata(vecregs_wdata),
		.rdata1(vecregs_rdata1),
		.rdata2(vecregs_rdata2)
		);	

	//Main state machine
	localparam cpu_state_fetch   = 8'b10000000;
	localparam cpu_state_ld_rs1  = 8'b01000000;
	localparam cpu_state_exec    = 8'b00100000;
	// localparam cpu_state_ld_rs2 = 8'b00010000;
	localparam cpu_state_stmem   = 8'b00010000;
	// localparam cpu_state_shift  = 8'b00000100;
	localparam cpu_state_ldmem   = 8'b00001000;
	// localparam cpu_state_stmem  = 8'b00001000;

	reg [7:0] cpu_state;
	reg latched_vstore;  //Added for vector instruction
	reg latched_stalu;	//This wil be 1 if the result to be written to register is the out of ALU


	always@(posedge clk) begin
		set_mem_do_rdata = 0;
		set_mem_do_wdata = 0;
		if(!resetn || !pcpi_valid) begin
			pcpi_rd <= 0;
			pcpi_wait <= 0;
	        pcpi_ready <= 0;
            pcpi_wr <= 0;
			cpu_state <= cpu_state_fetch; //Default state
			latched_stalu <= 0;
			latched_vstore <= 0;
		end
		else begin
			pcpi_wait <= 1;
			case(cpu_state)
				cpu_state_fetch: begin
					latched_vstore <= 0;
					latched_stalu <= 0;
					mem_wordsize <= 0; //Has to write/read 32 bit data
					case(1'b1)
						latched_vstore: begin
							vecregs_wdata = latched_stalu ? valu_out:vreg_op1;
							vecregs_write = 1;
							pcpi_ready <= 1;
						end
					endcase
					cpu_state <= cpu_state_ld_rs1;
				end
				cpu_state_ld_rs1: begin
					case(1'b1)
						(instr_vsetvli || instr_vsetvl):  begin	
							vcsr_vl <= (decoded_vs1!=5'b00000) ? pcpi_cpurs1:vcsr_vl;
							vcsr_vtype <= (instr_vsetvl) ? pcpi_cpurs2:decoded_vimm;
							cpu_state <= cpu_state_ld_rs1;
							pcpi_ready <= 1;
							pcpi_wait <= 0;
						end
						(instr_vload): begin
							cpu_state <= cpu_state_ldmem;
							vecldstrcnt <= 16;//512/32
							vecreadcnt <=0;
							vecregs_waddr = decoded_rd;
							vecregs_raddr1 = decoded_rs1;
							vecregs_raddr2 = decoded_rs2;
							v_membits <= vcsr_vl * ((v_enc_width==3'b000)? 8:(v_enc_width==3'b101)?16:(v_enc_width==3'b110)?32:SEW); 
						end
						(instr_vstore): begin
							cpu_state <= cpu_state_stmem;
							vecldstrcnt <= 16;//512/32
							vecreadcnt <=0;
							vecregs_waddr = decoded_rd;
							vecregs_raddr1 = decoded_rd; //specifies v register holding store data
							vecregs_raddr2 = decoded_rs2;
							//Updating mem_bits depending on vcsr_vl and v_enc_width(i.e instr[14:12])
							v_membits <= vcsr_vl * ((v_enc_width==3'b000)? 8:(v_enc_width==3'b101)?16:(v_enc_width==3'b110)?32:SEW); 
						end
					endcase
				end
				cpu_state_exec: begin
					
				end
				cpu_state_ldmem: begin
					latched_vstore <= 1;
					mem_addr <= reg_op1;
					if(vecldstrcnt != 0) begin
						if(v_membits%32==0) begin
							mem_wordsize <= 0;
							v_membits <= v_membits -32;
							reg_op1 <= reg_op1 + 4;
							set_mem_do_rdata = 1;
							vecldstrcnt <= vecldstrcnt -1;							
						end
						if(mem_ready == 1)	begin
							case(vecldstrcnt)
								15: vreg_op1[1*32-1: 0*32] = mem_rdata_word;
								14: vreg_op1[2*32-1: 1*32] = mem_rdata_word;
								13: vreg_op1[3*32-1: 2*32] = mem_rdata_word;
								12: vreg_op1[4*32-1: 3*32] = mem_rdata_word;
								11: vreg_op1[5*32-1: 4*32] = mem_rdata_word;
								10: vreg_op1[6*32-1: 5*32] = mem_rdata_word;
								9: vreg_op1[7*32-1: 6*32] = mem_rdata_word;
								8: vreg_op1[8*32-1: 7*32] = mem_rdata_word;
								7: vreg_op1[9*32-1: 8*32] = mem_rdata_word;
								6: vreg_op1[10*32-1: 9*32] = mem_rdata_word;
								5: vreg_op1[11*32-1: 10*32] = mem_rdata_word;
								4: vreg_op1[12*32-1: 11*32] = mem_rdata_word;
								3: vreg_op1[13*32-1: 12*32] = mem_rdata_word;
								2: vreg_op1[14*32-1: 13*32] = mem_rdata_word;							16: vreg_op1[1*32-1: 0*32] = mem_rdata_word;
								1: vreg_op1[15*32-1: 14*32] = mem_rdata_word;
								0: vreg_op1[16*32-1: 15*32] = mem_rdata_word;
							endcase
						end
					end
					else if(vecldstrcnt == 0) begin
						if(mem_ready == 1) begin
							0: vreg_op1[16*32-1: 15*32] = mem_rdata_word;
							mem_valid <= 0;
							cpu_state <= cpu_state_fetch;
						end
					end
				end
				cpu_state_stmem: begin
					
				end
			endcase
		end
		if (set_mem_do_rdata)
			mem_do_rdata <= 1;
		if (set_mem_do_wdata)
			mem_do_wdata <= 1;
	end

	// // localparam no_elem = BUS_WIDTH/SEW; //8 elements everytime
	// //pcpi_insn[14:12] ---> Encodes the operand type 
	// //pcpi_insn[31:26] ---> Encodes the operation
	// //pcpi_insn[6:0]  ----> Notifies that it is an arithmetic operation
	// // wire pcpi_insn_vecadd = resetn ? (pcpi_valid && pcpi_insn[31:26]== 6'b000000 && pcpi_insn[14:12] == 3'b000 && pcpi_insn[6:0]==7'b1010111):0; //If not reset, it will get value
	// // wire pcpi_insn_vecdot = resetn ? (pcpi_valid && pcpi_insn[31:26]== 6'b111001 && pcpi_insn[14:12] == 3'b000 && pcpi_insn[6:0]==7'b1010111):0; //If not reset, it will get value
	// wire pcpi_arth_insn =  resetn ? (instr_vadd || instr_vdot) :0; //If not reset, it will get value

	// always@(posedge clk)
	// 	begin
	// 		if(pcpi_insn_valid) begin
	// 			if(elem_n*BUS_WIDTH < vlen) begin
	// 				vec_res_ready <= 0;
	// 				pcpi_wait <= 1; //Since the execution of vect instrn takes more than 2 clock cycles
	// 				$display("Instruction inside co-processor: 0x%x, pcpi_valid: %b, elem_n: %d",pcpi_insn, pcpi_valid, elem_n);
	// 				$display("rs1 data inside co-processor: 0x%x",pcpi_vecrs1);
	// 				$display("rs2 data inside co-processor: 0x%x",pcpi_vecrs2);
	// 				//since param is compile time constant, the unused branch in each instantiation will be optimized out leading to no additional hardware.
	// 				if(BUS_WIDTH == 8'B00100000) begin //If the bus width is 32 bits
	// 					if(pcpi_insn_vecadd) begin
	// 						//Assuming that there are 4 ALUs in the co-processor
	// 						pcpi_rd[31:24] <= pcpi_vecrs1[31:24] + pcpi_vecrs2[31:24];
	// 						pcpi_rd[23:16] <= pcpi_vecrs1[23:16] + pcpi_vecrs2[23:16];
	// 						pcpi_rd[15:8]  <= pcpi_vecrs1[15:8]  + pcpi_vecrs2[15:8];
	// 						pcpi_rd[7:0]   <= pcpi_vecrs1[7:0]   + pcpi_vecrs2[7:0];
	// 					end
	// 					else if(pcpi_insn_vecdot) begin
	// 						//Assuming that there are 4 ALUs in the co-processor
	// 						pcpi_rd[31:24] <= pcpi_vecrs1[31:24] * pcpi_vecrs2[31:24];
	// 						pcpi_rd[23:16] <= pcpi_vecrs1[23:16] * pcpi_vecrs2[23:16];
	// 						pcpi_rd[15:8]  <= pcpi_vecrs1[15:8]  * pcpi_vecrs2[15:8];
	// 						pcpi_rd[7:0]   <= pcpi_vecrs1[7:0]   * pcpi_vecrs2[7:0];
	// 					//Since the execution time is one clock cycle
	// 					end
	// 					//Since the execution time is one clock cycle
	// 					pcpi_wr <= 1;
	// 					pcpi_ready <= 1;
	// 					$display("pcpi output = 0x%x",pcpi_rd);
	// 				end
	// 				else if(BUS_WIDTH == 8'B01000000) begin //If the bus width is 64 bits
	// 					if(pcpi_insn_vecadd) begin
	// 						//Assuming that there are 8 ALUs in the co-processor
	// 						pcpi_rd[63:56] <= pcpi_vecrs1[63:56] + pcpi_vecrs2[63:56];
	// 						pcpi_rd[55:48] <= pcpi_vecrs1[55:48] + pcpi_vecrs2[55:48];
	// 						pcpi_rd[47:40] <= pcpi_vecrs1[47:40] + pcpi_vecrs2[47:40];
	// 						pcpi_rd[39:32] <= pcpi_vecrs1[39:32] + pcpi_vecrs2[39:32];
	// 						pcpi_rd[31:24] <= pcpi_vecrs1[31:24] + pcpi_vecrs2[31:24];
	// 						pcpi_rd[23:16] <= pcpi_vecrs1[23:16] + pcpi_vecrs2[23:16];
	// 						pcpi_rd[15:8]  <= pcpi_vecrs1[15:8]  + pcpi_vecrs2[15:8];
	// 						pcpi_rd[7:0]   <= pcpi_vecrs1[7:0]   + pcpi_vecrs2[7:0];
	// 					end
	// 					else if(pcpi_insn_vecdot) begin
	// 						//Assuming that there are 8 ALUs in the co-processor
	// 						pcpi_rd[63:56] <= pcpi_vecrs1[63:56] * pcpi_vecrs2[63:56];
	// 						pcpi_rd[55:48] <= pcpi_vecrs1[55:48] * pcpi_vecrs2[55:48];
	// 						pcpi_rd[47:40] <= pcpi_vecrs1[47:40] * pcpi_vecrs2[47:40];
	// 						pcpi_rd[39:32] <= pcpi_vecrs1[39:32] * pcpi_vecrs2[39:32];
	// 						pcpi_rd[31:24] <= pcpi_vecrs1[31:24] * pcpi_vecrs2[31:24];
	// 						pcpi_rd[23:16] <= pcpi_vecrs1[23:16] * pcpi_vecrs2[23:16];
	// 						pcpi_rd[15:8]  <= pcpi_vecrs1[15:8]  * pcpi_vecrs2[15:8];
	// 						pcpi_rd[7:0]   <= pcpi_vecrs1[7:0]   * pcpi_vecrs2[7:0];
	// 					end
	// 					//Since the execution time is one clock cycle
	// 					pcpi_wr <= 1;
	// 					pcpi_ready <= 1;
	// 					$display("pcpi output = 0x%x",pcpi_rd);
	// 				end
	// 				else if(BUS_WIDTH == 8'B10000000) begin //If the bus width is 128 bits
	// 					if(pcpi_insn_vecadd) begin
	// 						//Assuming that there are 16 ALUs in the co-processor
	// 						pcpi_rd[127:120] <= pcpi_vecrs1[127:120] + pcpi_vecrs2[127:120];
	// 						pcpi_rd[119:112] <= pcpi_vecrs1[119:112] + pcpi_vecrs2[119:112];
	// 						pcpi_rd[111:104] <= pcpi_vecrs1[111:104] + pcpi_vecrs2[111:104];
	// 						pcpi_rd[103:96]  <= pcpi_vecrs1[103:96] + pcpi_vecrs2[103:96];
	// 						pcpi_rd[95:88]   <= pcpi_vecrs1[95:88] + pcpi_vecrs2[95:88];
	// 						pcpi_rd[87:80]   <= pcpi_vecrs1[87:80] + pcpi_vecrs2[87:80];
	// 						pcpi_rd[79:72]   <= pcpi_vecrs1[79:72] + pcpi_vecrs2[79:72];
	// 						pcpi_rd[71:64]   <= pcpi_vecrs1[71:64] + pcpi_vecrs2[71:64];
	// 						pcpi_rd[63:56]   <= pcpi_vecrs1[63:56] + pcpi_vecrs2[63:56];
	// 						pcpi_rd[55:48]   <= pcpi_vecrs1[55:48] + pcpi_vecrs2[55:48];
	// 						pcpi_rd[47:40]   <= pcpi_vecrs1[47:40] + pcpi_vecrs2[47:40];
	// 						pcpi_rd[39:32]   <= pcpi_vecrs1[39:32] + pcpi_vecrs2[39:32];
	// 						pcpi_rd[31:24]   <= pcpi_vecrs1[31:24] + pcpi_vecrs2[31:24];
	// 						pcpi_rd[23:16]   <= pcpi_vecrs1[23:16] + pcpi_vecrs2[23:16];
	// 						pcpi_rd[15:8]    <= pcpi_vecrs1[15:8]  + pcpi_vecrs2[15:8];
	// 						pcpi_rd[7:0]     <= pcpi_vecrs1[7:0]   + pcpi_vecrs2[7:0];
	// 					end
	// 					else if (pcpi_insn_vecdot) begin
	// 						//Assuming that there are 16 ALUs in the co-processor
	// 						pcpi_rd[127:120] <= pcpi_vecrs1[127:120] * pcpi_vecrs2[127:120];
	// 						pcpi_rd[119:112] <= pcpi_vecrs1[119:112] * pcpi_vecrs2[119:112];
	// 						pcpi_rd[111:104] <= pcpi_vecrs1[111:104]   * pcpi_vecrs2[111:104];
	// 						pcpi_rd[103:96]  <= pcpi_vecrs1[103:96]  * pcpi_vecrs2[103:96];
	// 						pcpi_rd[95:88]   <= pcpi_vecrs1[95:88] * pcpi_vecrs2[95:88];
	// 						pcpi_rd[87:80]   <= pcpi_vecrs1[87:80] * pcpi_vecrs2[87:80];
	// 						pcpi_rd[79:72]   <= pcpi_vecrs1[79:72] * pcpi_vecrs2[79:72];
	// 						pcpi_rd[71:64]   <= pcpi_vecrs1[71:64] * pcpi_vecrs2[71:64];
	// 						pcpi_rd[63:56]   <= pcpi_vecrs1[63:56] * pcpi_vecrs2[63:56];
	// 						pcpi_rd[55:48]   <= pcpi_vecrs1[55:48] * pcpi_vecrs2[55:48];
	// 						pcpi_rd[47:40]   <= pcpi_vecrs1[47:40] * pcpi_vecrs2[47:40];
	// 						pcpi_rd[39:32]   <= pcpi_vecrs1[39:32] * pcpi_vecrs2[39:32];
	// 						pcpi_rd[31:24]   <= pcpi_vecrs1[31:24] * pcpi_vecrs2[31:24];
	// 						pcpi_rd[23:16]   <= pcpi_vecrs1[23:16] * pcpi_vecrs2[23:16];
	// 						pcpi_rd[15:8]    <= pcpi_vecrs1[15:8]  * pcpi_vecrs2[15:8];
	// 						pcpi_rd[7:0]     <= pcpi_vecrs1[7:0]   * pcpi_vecrs2[7:0];
	// 					end
	// 					//Since the execution time is one clock cycle
	// 					pcpi_wr <= 1;
	// 					pcpi_ready <= 1;
	// 					$display("pcpi output = 0x%x",pcpi_rd);
	// 				end
	// 			end
	// 			if((elem_n)*BUS_WIDTH == vlen) begin 
	// 				pcpi_wait <= 0; //Making it 0 after the execution
	// 				vec_res_ready <= 1;
	// 			end
					
	// 		end
	// 		else begin
	// 			pcpi_rd <= 512'bx;
	// 			pcpi_wr <= 0;
	// 			pcpi_ready <= 0;
	// 		end
	// 	end
endmodule
