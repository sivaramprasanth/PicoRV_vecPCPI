
/***************************************************************
 * picorv32_pcpi_vec: A PCPI core that implements the vector instructions
 ***************************************************************/
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
						// $display("Inside mem_interface, mem_wdata:%x, time:%d", vreg_rdata1_latched,$time);
						if(instr_vstore || instr_vstore_str) begin
							// mem_addr <= reg_op1; //This should work only is the instr is store
							mem_wdata <= vreg_rdata1_latched; //changed here
							mem_wstrb <= 4'b1111 & {4{mem_do_wdata}}; //mem_la_wstrb & {4{mem_la_write}}
						end
						if(instr_vload || instr_vload_str) begin
							mem_rdata_word <= mem_rdata; //reads 32 bits
						end
						mem_str_ready <= 1; //str_ready will be 1 irrespective of the instruction
					end
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
					mem_str_ready <= 0;
					if(mem_ready == 1) begin
						if(instr_vload || instr_vload_str) begin
							case(mem_str_state)
								2'b00: begin
											// $display("inside mem_interface state0, mem_rdata:%x, time:%d", mem_rdata,$time);
											case (mem_addr[1:0])
												2'b00: mem_rdata_word[7:0] <= mem_rdata[7: 0];
												2'b01: mem_rdata_word[7:0] <= mem_rdata[15: 8];
												2'b10: mem_rdata_word[7:0] <= mem_rdata[23:16];
												2'b11: mem_rdata_word[7:0] <= mem_rdata[31:24];
											endcase
											mem_str_state <= 2'b01;
											mem_str_ready <= 0;
										end
								2'b01:  begin	
											// $display("inside mem_interface state1, mem_rdata:%x, time:%d", mem_rdata,$time);
											case (mem_addr[1:0])
												2'b00: mem_rdata_word[15:8] <= mem_rdata[7: 0];
												2'b01: mem_rdata_word[15:8] <= mem_rdata[15: 8];
												2'b10: mem_rdata_word[15:8] <= mem_rdata[23:16];
												2'b11: mem_rdata_word[15:8] <= mem_rdata[31:24];
											endcase
											mem_str_state <= 2'b10;	
											mem_str_ready <= 0;							
										end
								2'b10:  begin
											// $display("inside mem_interface state2, mem_rdata:%x, time:%d", mem_rdata,$time);
											case (mem_addr[1:0])
												2'b00: mem_rdata_word[23:16] <= mem_rdata[7: 0];
												2'b01: mem_rdata_word[23:16] <= mem_rdata[15: 8];
												2'b10: mem_rdata_word[23:16] <= mem_rdata[23:16];
												2'b11: mem_rdata_word[23:16] <= mem_rdata[31:24];
											endcase
											mem_str_state <= 2'b11;
											mem_str_ready <= 0;
										end
								2'b11: 	begin
											// $display("inside mem_interface state3, mem_rdata:%x, time:%d", mem_rdata,$time);
											case (mem_addr[1:0])
												2'b00: mem_rdata_word[31:24] <= mem_rdata[7: 0];
												2'b01: mem_rdata_word[31:24] <= mem_rdata[15: 8];
												2'b10: mem_rdata_word[31:24] <= mem_rdata[23:16];
												2'b11: mem_rdata_word[31:24] <= mem_rdata[31:24];
											endcase
											mem_str_state <= 2'b00;
											mem_str_ready <= 1;
										end	
							endcase
						end
						if(instr_vstore || instr_vstore_str) begin
							if(vstore_bit < 2)
								mem_str_ready <= 1;
							mem_wdata <= vreg_rdata1_latched; //changed here
							if(vstore_bit >= 1) begin
								case(mem_str_state)
									2'b00: begin
												// $display("inside mem_interface state0, mem_wstrb:%b, mem_wdata:%x, time:%d", mem_wstrb, mem_wdata,$time);
												case (mem_addr[1:0])
													2'b00: mem_wstrb <= 4'b0001 & {4{mem_do_wdata}};
													2'b01: mem_wstrb <= 4'b0010 & {4{mem_do_wdata}};
													2'b10: mem_wstrb <= 4'b0100 & {4{mem_do_wdata}};
													2'b11: mem_wstrb <= 4'b1000 & {4{mem_do_wdata}};
												endcase
												mem_str_state <= 2'b01;													
												//This is required to change the mem_addr i.e reg_op1 in mem_ready
												if(vstore_bit == 2) begin
													mem_str_ready <= 0;
												end
											end
									2'b01:  begin	
												// $display("inside mem_interface state1, mem_addr:%d, mem_wstrb:%b, mem_wdata:%x, time:%d", mem_addr, mem_wstrb, mem_wdata,$time);
												case (mem_addr[1:0])
													2'b00: mem_wstrb <= 4'b0001 & {4{mem_do_wdata}};
													2'b01: mem_wstrb <= 4'b0010 & {4{mem_do_wdata}};
													2'b10: mem_wstrb <= 4'b0100 & {4{mem_do_wdata}};
													2'b11: mem_wstrb <= 4'b1000 & {4{mem_do_wdata}};
												endcase
												mem_str_state <= 2'b10;	
												mem_str_ready <= 0;							
											end
									2'b10:  begin
												// $display("inside mem_interface state2, mem_wstrb:%b, mem_wdata:%x, time:%d", mem_wstrb, mem_wdata,$time);
												case (mem_addr[1:0])
													2'b00: mem_wstrb <= 4'b0001 & {4{mem_do_wdata}};
													2'b01: mem_wstrb <= 4'b0010 & {4{mem_do_wdata}};
													2'b10: mem_wstrb <= 4'b0100 & {4{mem_do_wdata}};
													2'b11: mem_wstrb <= 4'b1000 & {4{mem_do_wdata}};
												endcase
												mem_str_state <= 2'b11;
												mem_str_ready <= 0;
											end
									2'b11: 	begin
												// $display("inside mem_interface state3, mem_wstrb:%b, mem_wdata:%x, time:%d", mem_wstrb, mem_wdata,$time);
												case (mem_addr[1:0])
													2'b00: mem_wstrb <= 4'b0001 & {4{mem_do_wdata}};
													2'b01: mem_wstrb <= 4'b0010 & {4{mem_do_wdata}};
													2'b10: mem_wstrb <= 4'b0100 & {4{mem_do_wdata}};
													2'b11: mem_wstrb <= 4'b1000 & {4{mem_do_wdata}};
												endcase
												mem_str_state <= 2'b00;
												mem_str_ready <= 1;
											end	
								endcase
							end
						end
					end
				end
				3: begin
					mem_str_ready <= 0;
					if(mem_ready == 1) begin
						if(instr_vload || instr_vload_str) begin
							//When stride value is 0
							if(reg_op2 == 0) begin
								if(mem_addr[1:0] == 2'b00) begin
									// $display("mem_addr: 00, mem_rdata: %x, time:%d", mem_rdata, $time);
									mem_rdata_word <= {mem_rdata[7:0],mem_rdata[7:0],mem_rdata[7:0],mem_rdata[7:0]};
									mem_str_ready <= 1;
									mem_str_state <= 2'b00;
								end
								else if(mem_addr[1:0] == 2'b01) begin
									// $display("mem_addr: 01, mem_rdata: %x, time:%d", mem_rdata, $time);
									mem_rdata_word <= {mem_rdata[15:8],mem_rdata[15:8],mem_rdata[15:8],mem_rdata[15:8]};
									mem_str_ready <= 1;
									mem_str_state <= 2'b00;
								end
								else if(mem_addr[1:0] == 2'b10) begin
									// $display("mem_addr: 10, mem_rdata: %x, time:%d", mem_rdata, $time);
									mem_rdata_word <= {mem_rdata[23:16],mem_rdata[23:16],mem_rdata[23:16],mem_rdata[23:16]};
									mem_str_ready <= 1;
									mem_str_state <= 2'b00;
								end
								else if(mem_addr[1:0] == 2'b11) begin
									// $display("mem_addr: 11, mem_rdata: %x, time:%d", mem_rdata, $time);
									mem_rdata_word <= {mem_rdata[31:24],mem_rdata[31:24],mem_rdata[31:24],mem_rdata[31:24]};
									mem_str_ready <= 1;
									mem_str_state <= 2'b00;
								end
							end
							//When stride value is 1
							else if(reg_op2 == 1) begin
								if(mem_addr[1:0] == 2'b00) begin
									// $display("mem_addr: 00, mem_rdata: %x, time:%d", mem_rdata, $time);
									mem_rdata_word <= mem_rdata;
									mem_str_ready <= 1;
									mem_str_state <= 2'b00;
								end
								else if(mem_addr[1:0] == 2'b01) begin
									case(mem_str_state)
										2'b00: begin
											// $display("mem_addr: 01, in state 00, mem_rdata: %x,time:%d", mem_rdata, $time);
											mem_rdata_word_next[23:0] <= mem_rdata[31:8]; //Imp stepp
											mem_str_ready <= 0;
											mem_str_state <= 2'b01;
										end
										2'b01: begin
											// $display("mem_addr: 01, in state 01, mem_rdata: %x,time:%d", mem_rdata, $time);
											mem_rdata_word[31:24] <= mem_rdata[7:0];
											mem_rdata_word[23:0] <= mem_rdata_word_next[23:0];
											mem_rdata_word_next[23:0] <= mem_rdata[31:8];
											mem_str_ready <= 1;
											mem_str_state <= 2'b01;
										end
									endcase
								end
								else if(mem_addr[1:0] == 2'b10) begin
									case(mem_str_state)
										2'b00: begin
											// $display("mem_addr: 10, in state 00, mem_rdata: %x,time:%d", mem_rdata, $time);
											mem_rdata_word_next[15:0] <= mem_rdata[31:16];
											mem_str_ready <= 0;
											mem_str_state <= 2'b01;
										end
										2'b01: begin
											// $display("mem_addr: 10, in state 00, mem_rdata: %x,time:%d", mem_rdata, $time);
											mem_rdata_word[31:16] <= mem_rdata[15:0];
											mem_rdata_word[15:0] <= mem_rdata_word_next[15:0];
											mem_rdata_word_next[15:0] <= mem_rdata[31:16];
											mem_str_ready <= 1;
											mem_str_state <= 2'b01;
										end
									endcase
								end
								else if(mem_addr[1:0] == 2'b11) begin
									case(mem_str_state)
										2'b00: begin
											// $display("mem_addr: 11, reg_op2: %d, mem_addr[1:0]: %b", reg_op2, mem_addr[1:0]);
											mem_rdata_word_next[7:0] <= mem_rdata[31:24];
											mem_str_ready <= 0;
											mem_str_state <= 2'b01;
										end
										2'b01: begin
											// $display("mem_addr: 11, reg_op2: %d, mem_addr[1:0]: %b", reg_op2, mem_addr[1:0]);
											mem_rdata_word[31:8] <= mem_rdata[23:0];
											mem_rdata_word[7:0] <= mem_rdata_word_next[7:0];
											mem_rdata_word_next[7:0] <= mem_rdata[31:24];
											mem_str_ready <= 1;
											mem_str_state <= 2'b01;
										end
									endcase
								end
							end
							//When stride is 2
							else if(reg_op2 == 2) begin
								if(mem_addr[1:0] == 2'b00) begin
									case(mem_str_state)
										2'b00: begin
											// $display("mem_addr: 00, in state 00, mem_rdata: %x,time:%d", mem_rdata, $time);
											mem_rdata_word[15:0] <= {mem_rdata[23:16],mem_rdata[7:0]}; //Imp stepp
											mem_str_ready <= 0;
											mem_str_state <= 2'b01;
										end
										2'b01: begin
											// $display("mem_addr: 00, in state 01, mem_rdata: %x,time:%d", mem_rdata, $time);
											mem_rdata_word[31:16] <= {mem_rdata[23:16],mem_rdata[7:0]}; //Imp stepp
											mem_str_ready <= 1;
											mem_str_state <= 2'b00;
										end
									endcase
								end
								else if(mem_addr[1:0] == 2'b01) begin
									case(mem_str_state)
										2'b00: begin
											// $display("mem_addr: 01, in state 00, mem_rdata: %x,time:%d", mem_rdata, $time);
											mem_rdata_word[15:0] <= {mem_rdata[31:24],mem_rdata[15:8]}; //Imp stepp
											mem_str_ready <= 0;
											mem_str_state <= 2'b01;
										end
										2'b01: begin
											// $display("mem_addr: 01, in state 01, mem_rdata: %x,time:%d", mem_rdata, $time);
											mem_rdata_word[31:16] <= {mem_rdata[31:24],mem_rdata[15:8]}; //Imp stepp
											mem_str_ready <= 1;
											mem_str_state <= 2'b00;
										end
									endcase
								end
								else if(mem_addr[1:0] == 2'b10) begin
									case(mem_str_state)
										2'b00: begin
											// $display("mem_addr: 10, in state 00, mem_rdata: %x,time:%d", mem_rdata, $time);
											mem_rdata_word_next[7:0] <= mem_rdata[23:16]; //Imp step
											mem_str_ready <= 0;
											mem_str_state <= 2'b01;
										end
										2'b01: begin
											// $display("mem_addr: 10, in state 01, mem_rdata: %x,time:%d", mem_rdata, $time);
											mem_rdata_word[23:8] <= {mem_rdata[23:16],mem_rdata[7:0]};
											mem_rdata_word[7:0] <= mem_rdata_word_next[7:0];
											mem_str_ready <= 0;
											mem_str_state <= 2'b10;
										end
										2'b10: begin
											// $display("mem_addr: 10, in state 10, mem_rdata: %x,time:%d", mem_rdata, $time);
											mem_rdata_word[31:24] <= mem_rdata[7:0];
											mem_rdata_word_next[7:0] <= mem_rdata[23:16];
											mem_str_ready <= 1;
											mem_str_state <= 2'b01;
										end
									endcase
								end
								else if(mem_addr[1:0] == 2'b11) begin
									case(mem_str_state)
										2'b00: begin
											// $display("mem_addr: 11, in state 00, mem_rdata: %x,time:%d", mem_rdata, $time);
											mem_rdata_word_next[7:0] <= mem_rdata[31:24]; //Imp step
											mem_str_ready <= 0;
											mem_str_state <= 2'b01;
										end
										2'b01: begin
											// $display("mem_addr: 11, in state 01, mem_rdata: %x,time:%d", mem_rdata, $time);
											mem_rdata_word[23:8] <= {mem_rdata[31:24],mem_rdata[15:8]};
											mem_rdata_word[7:0] <= mem_rdata_word_next[7:0];
											mem_str_ready <= 0;
											mem_str_state <= 2'b10;
										end
										2'b10: begin
											// $display("mem_addr: 11, in state 10, mem_rdata: %x,time:%d", mem_rdata, $time);
											mem_rdata_word[31:24] <= mem_rdata[15:8];
											mem_rdata_word_next[7:0] <= mem_rdata[31:24];
											mem_str_ready <= 1;
											mem_str_state <= 2'b01;
										end
									endcase
								end
							end
							//When stride is 3
							else if(reg_op2 == 3) begin
								if(mem_addr[1:0] == 2'b00) begin
									case(mem_str_state)
										2'b00: begin
											// $display("mem_addr: 00, in state 00, mem_rdata: %x,time:%d", mem_rdata, $time);
											mem_rdata_word[15:0] <= {mem_rdata[31:24],mem_rdata[7:0]}; //Imp stepp
											mem_str_ready <= 0;
											mem_str_state <= 2'b01;
										end
										2'b01: begin
											// $display("mem_addr: 00, in state 01, mem_rdata: %x,time:%d", mem_rdata, $time);
											mem_rdata_word[23:16] <= mem_rdata[23:16]; //Imp stepp
											mem_str_ready <= 0;
											mem_str_state <= 2'b10;
										end
										2'b10: begin
											// $display("mem_addr: 00, in state 01, mem_rdata: %x,time:%d", mem_rdata, $time);
											mem_rdata_word[31:24] <= mem_rdata[15:8]; //Imp stepp
											mem_str_ready <= 1;
											mem_str_state <= 2'b00;
										end
									endcase
								end
								else if(mem_addr[1:0] == 2'b01) begin
									case(mem_str_state)
										2'b00: begin
											// $display("mem_addr: 01, in state 01, mem_rdata: %x,time:%d", mem_rdata, $time);
											mem_rdata_word[7:0] <= mem_rdata[15:8]; //Imp stepp
											mem_str_ready <= 0;
											mem_str_state <= 2'b01;
										end
										2'b01: begin
											// $display("mem_addr: 01, in state 01, mem_rdata: %x,time:%d", mem_rdata, $time);
											mem_rdata_word[23:8] <= {mem_rdata[31:24],mem_rdata[7:0]}; //Imp stepp
											mem_str_ready <= 0;
											mem_str_state <= 2'b10;
										end
										2'b10: begin
											// $display("mem_addr: 01, in state 01, mem_rdata: %x,time:%d", mem_rdata, $time);
											mem_rdata_word[31:24] <= mem_rdata[23:16]; //Imp stepp
											mem_str_ready <= 1;
											mem_str_state <= 2'b00;
										end
									endcase
								end
								else if(mem_addr[1:0] == 2'b10) begin
									case(mem_str_state)
										2'b00: begin
											$display("mem_addr: 10, in state 00, mem_rdata: %x,time:%d", mem_rdata, $time);
											mem_rdata_word[7:0] <= mem_rdata[23:16]; //Imp stepp
											mem_str_ready <= 0;
											mem_str_state <= 2'b01;
										end
										2'b01: begin
											$display("mem_addr: 10, in state 01, mem_rdata: %x,time:%d", mem_rdata, $time);
											mem_rdata_word[15:8] <= mem_rdata[15:8]; //Imp stepp
											mem_str_ready <= 0;
											mem_str_state <= 2'b10;
										end
										2'b10: begin
											$display("mem_addr: 10, in state 10, mem_rdata: %x,time:%d", mem_rdata, $time);
											mem_rdata_word[31:16] <= {mem_rdata[31:24],mem_rdata[7:0]}; //Imp stepp
											mem_str_ready <= 1;
											mem_str_state <= 2'b00;
										end
									endcase
								end
								else if(mem_addr[1:0] == 2'b11) begin
									case(mem_str_state)
										2'b00: begin
											$display("mem_addr: 11, in state 00, mem_rdata: %x,time:%d", mem_rdata, $time);
											mem_rdata_word_next[7:0] <= mem_rdata[31:24]; //Imp stepp
											mem_str_ready <= 0;
											mem_str_state <= 2'b01;
										end
										2'b01: begin
											$display("mem_addr: 11, in state 01, mem_rdata: %x,time:%d", mem_rdata, $time);
											mem_rdata_word[15:8] <= mem_rdata[23:16]; //Imp stepp
											mem_rdata_word[7:0] <= mem_rdata_word_next[7:0];
											mem_str_ready <= 0;
											mem_str_state <= 2'b10;
										end
										2'b10: begin
											$display("mem_addr: 11, in state 10, mem_rdata: %x,time:%d", mem_rdata, $time);
											mem_rdata_word[23:16] <= mem_rdata[15:8]; //Imp stepp
											mem_str_ready <= 0;
											mem_str_state <= 2'b11;
										end
										2'b11: begin
											$display("mem_addr: 11, in state 11, mem_rdata: %x,time:%d", mem_rdata, $time);
											mem_rdata_word[31:24] <= mem_rdata[7:0]; //Imp stepp
											mem_rdata_word_next[7:0] <= mem_rdata[31:24]; //Imp stepp
											mem_str_ready <= 1;
											mem_str_state <= 2'b01;
										end
									endcase
								end
							end
						end
						if(instr_vstore || instr_vstore_str) begin
							if(vstore_bit < 2)
								mem_str_ready <= 1;
							if(vstore_bit >= 1) begin
								// //When stride value is 0
								if(reg_op2 == 0) begin
									$display("inside mem_interface state0, mem_wstrb:%b, mem_wdata:%x, time:%d", mem_wstrb, mem_wdata,$time);
									mem_wdata <= {vreg_rdata1_latched[31:24], 24'b0}; //changed here
									case (mem_addr[1:0])
										2'b00: mem_wstrb <= 4'b0001 & {4{mem_do_wdata}};
										2'b01: mem_wstrb <= 4'b0010 & {4{mem_do_wdata}};
										2'b10: mem_wstrb <= 4'b0100 & {4{mem_do_wdata}};
										2'b11: mem_wstrb <= 4'b1000 & {4{mem_do_wdata}};
									endcase
									mem_str_state <= 2'b01;													
									mem_str_ready <= 1;
								end
								//When stride value is 1
								if(reg_op2 == 1) begin
									if(mem_addr[1:0] == 2'b00) begin
										// $display("mem_addr: 00, mem_wdata: %x, time:%d", vreg_rdata1_latched, $time);
										mem_wdata <= vreg_rdata1_latched; //changed here
										mem_wstrb <= 4'b1111 & {4{mem_do_wdata}};
										mem_str_ready <= 1;
										mem_str_state <= 2'b00;
										if(vecldstrcnt == var_vlen)
											mem_wstrb <= 4'b0;	
									end
									else if(mem_addr[1:0] == 2'b01) begin
										case(mem_str_state)
										 	2'b00: begin
												$display("mem_addr: 01, in state 00, mem_rdata: %x,time:%d", vreg_rdata1_latched, $time);
												mem_wdata <= {vreg_rdata1_latched[23:0], 8'b0}; //changed here
												mem_wdata_word_next <= {24'b0, vreg_rdata1_latched[31:24]};
												mem_wstrb <= 4'b1110 & {4{mem_do_wdata}};
												mem_str_ready <= 1;
												mem_str_state <= 2'b01;
											end
											2'b01: begin
												$display("mem_addr: 01, in state 01, mem_rdata: %x,time:%d", vreg_rdata1_latched, $time);
												mem_wdata[7:0] <= mem_wdata_word_next[7:0];
												mem_wdata[31:8] <= vreg_rdata1_latched[23:0]; //changed here
												mem_wdata_word_next[7:0] <= vreg_rdata1_latched[31:24];
												mem_wstrb <= 4'b1111 & {4{mem_do_wdata}};
												mem_str_ready <= 1;
												if(vecldstrcnt == (var_vlen + 1))
													mem_str_state <= 2'b10;
												else
													mem_str_state <= 2'b01;
											end
											2'b10: begin
												$display("mem_addr: 01, in state 10, mem_rdata: %x,time:%d", vreg_rdata1_latched, $time);
												mem_wdata[7:0] <= mem_wdata_word_next[7:0];
												mem_wstrb <= 4'b0001 & {4{mem_do_wdata}};
												mem_str_ready <= 1;
												mem_str_state <= 2'b00;
											end
										endcase
									end
									else if(mem_addr[1:0] == 2'b10) begin
										case(mem_str_state)
											2'b00: begin
												// $display("mem_addr: 10, in state 00, mem_rdata: %x,time:%d", vreg_rdata1_latched, $time);
												mem_wdata <= {vreg_rdata1_latched[15:0], 16'b0}; //changed here
												mem_wdata_word_next <= {16'b0, vreg_rdata1_latched[31:16]};
												mem_wstrb <= 4'b1100 & {4{mem_do_wdata}};
												mem_str_ready <= 1;
												mem_str_state <= 2'b01;
											end
											2'b01: begin
												// $display("mem_addr: 10, in state 01, mem_rdata: %x,time:%d", vreg_rdata1_latched, $time);
												mem_wdata[15:0] <= mem_wdata_word_next[15:0];
												mem_wdata[31:16] <= vreg_rdata1_latched[16:0]; //changed here
												mem_wdata_word_next[15:0] <= vreg_rdata1_latched[31:16];
												mem_wstrb <= 4'b1111 & {4{mem_do_wdata}};
												mem_str_ready <= 1;
												if(vecldstrcnt == (var_vlen + 1))
													mem_str_state <= 2'b10;
												else
													mem_str_state <= 2'b01;
											end
											2'b10: begin
												// $display("mem_addr: 10, in state 10, mem_rdata: %x,time:%d", vreg_rdata1_latched, $time);
												mem_wdata[15:0] <= mem_wdata_word_next[15:0];
												mem_wstrb <= 4'b0011 & {4{mem_do_wdata}};
												mem_str_ready <= 1;
												mem_str_state <= 2'b00;
											end
										endcase
									end
									else if(mem_addr[1:0] == 2'b11) begin
										case(mem_str_state)
											2'b00: begin
												// $display("mem_addr: 11, in state 00, mem_rdata: %x,time:%d", vreg_rdata1_latched, $time);
												mem_wdata <= {vreg_rdata1_latched[7:0], 24'b0}; //changed here
												mem_wdata_word_next <= {8'b0, vreg_rdata1_latched[31:8]};
												mem_wstrb <= 4'b1000 & {4{mem_do_wdata}};
												mem_str_ready <= 1;
												mem_str_state <= 2'b01;
											end
											2'b01: begin
												// $display("mem_addr: 11, in state 01, mem_rdata: %x,time:%d", vreg_rdata1_latched, $time);
												mem_wdata[23:0] <= mem_wdata_word_next[23:0];
												mem_wdata[31:24] <= vreg_rdata1_latched[7:0]; //changed here
												mem_wdata_word_next[23:0] <= vreg_rdata1_latched[31:8];
												mem_wstrb <= 4'b1111 & {4{mem_do_wdata}};
												mem_str_ready <= 1;
												if(vecldstrcnt == (var_vlen + 1))
													mem_str_state <= 2'b10;
												else
													mem_str_state <= 2'b01;
											end
											2'b10: begin
												// $display("mem_addr: 11, in state 10, mem_rdata: %x,time:%d", vreg_rdata1_latched, $time);
												mem_wdata[23:0] <= mem_wdata_word_next[23:0];
												mem_wstrb <= 4'b0111 & {4{mem_do_wdata}};
												mem_str_ready <= 1;
												mem_str_state <= 2'b00;
											end
										endcase
									end
								end
							end
						end
					end
				end
			endcase
		end
	end

	//Instruction decoder
	reg instr_vsetvli,instr_vsetvl; //Vec instrn to set the csr reg values
	reg instr_vload,instr_vload_str,instr_vstore, instr_vstore_str;   //Vec load and store instr
	reg instr_vdot,instr_vadd; //For dot product and addition
	wire is_vec_instr; //To check whether the forwarded instruction to coprocessor is vector instruction or not
	
	assign is_vec_instr = |{instr_vsetvli,instr_vsetvl,instr_vload,instr_vload_str,instr_vstore, instr_vstore_str, instr_vdot,instr_vadd};

	reg [4:0] decoded_vs1, decoded_vs2, decoded_vd; //For vect instrns
	reg [10:0] decoded_vimm; //For vect instrns

	//Instruction decoder
	always@(posedge clk) begin
		if (!resetn || !pcpi_valid) begin
			instr_vsetvl <= 0;
			instr_vsetvli <= 0;
			instr_vload <= 0;
			instr_vload_str <= 0;
			instr_vstore <= 0;
			instr_vstore_str <= 0; //For strided store
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
			instr_vstore  <= (pcpi_insn[24:20]==5'b00000) && (pcpi_insn[28:26]==3'b000) && (pcpi_insn[6:0] == 7'b0100111); // only unit stride supported,NF not supported 
			instr_vstore_str <= (pcpi_insn[28:26]==3'b010) && (pcpi_insn[14:12]==3'b111) && (pcpi_insn[6:0] == 7'b0100111); //Strided store, works for 32 bit SEW
			instr_vsetvl  <= (pcpi_insn[14:12]==3'b111) && (pcpi_insn[31]==1) && (pcpi_insn[6:0] == 7'b1010111); 
			instr_vsetvli <= (pcpi_insn[14:12]==3'b111) && (pcpi_insn[31]==0) && (pcpi_insn[6:0] == 7'b1010111);
			instr_vdot    <= (pcpi_insn[31:26]==6'b111001) && (pcpi_insn[14:12] == 3'b000) && (pcpi_insn[6:0]==7'b1010111);
			instr_vadd    <= (pcpi_insn[31:26]==6'b000000) && (pcpi_insn[14:12] == 3'b000) && (pcpi_insn[6:0]==7'b1010111);
			v_enc_width   <= (instr_vload || instr_vstore || instr_vload_str || instr_vstore_str)? pcpi_insn[14:12]:0;
		end
	end
	
	//For vector instructions
	wire [31:0]vcsr_vlenb = 32'h00000080; //vector reg length in bytes
	reg [31:0]vcsr_vlen;
	reg [31:0]vcsr_vtype; //Can be updated by vsetvli or vsetvl instr
	reg [31:0]vcsr_vl;    //No of elements to be updated by a vector instrn
	wire [2:0]vsew  = vcsr_vtype[4:2]; //Encoding for the no of bits in an element (part of vtype)
	wire [1:0]vlmul = vcsr_vtype[1:0]; //Encoding for the no of vec regs in a group (part of vtype)
	wire [9:0] SEW;
	assign SEW  = (4*(2<<vsew));
	wire [31:0]LMUL = (1 << (vlmul)); //No of vector regs in a group (2^vlmul)
	wire [31:0]VLMAX = (512/SEW)*LMUL; //Represents the max no of elements that can be operated on with a vec instrn

	//Variables used by vload and vstore
	reg [5:0] vecldstrcnt;
	reg [5:0] var_vlen; //Used to load the vector reg partially
	reg [31:0] vreg_op1;  //Data from v1 for all vector instr
	reg [31:0] vreg_op2;  //Data from v2 for all vector instr
	reg [31:0] vreg_op3;  //Data from vd for dot product instruction
	reg [31:0] vreg_rdata1_latched; //data readfrom vector regs, used for vstore
	reg [2:0] v_enc_width;
	reg [10:0] v_membits; //Contains the number of bits to be loaded from memory
	reg [15:0] vecregs_wstrb_temp; //Used to store write strobe
	
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
					// case(1'b1)
					// 	//latched_vstore will be 1 for 1 clk cycle
					// 	latched_vstore: begin
					// 		pcpi_ready <= 1;
					// 	end
					// endcase
					cpu_state <= cpu_state_ld_rs1;
				end
				cpu_state_ld_rs1: begin
					case(1'b1)
						(instr_vsetvli || instr_vsetvl):  begin	
							vcsr_vl <= (decoded_vs1!=5'b00000) ? pcpi_cpurs1:vcsr_vl;
							vcsr_vtype <= (instr_vsetvl) ? pcpi_cpurs2:decoded_vimm;
							// $display("pcpi_cpurs1: %d",pcpi_cpurs1);
							// $display("Inside coproc, vcsr_vl = %d, vcsr_vtype = %b, decoded_vs1 = %b", vcsr_vl,vcsr_vtype, decoded_vs1);
							cpu_state <= cpu_state_ld_rs1;
							pcpi_ready <= 1;
							pcpi_wait <= 0;
						end
						(instr_vload): begin
							$display("Inside v_load condition, time:%d", $time);
							// mem_valid <= 1;
							cpu_state <= cpu_state_ldmem;
							vecldstrcnt <= 16; 
							var_vlen <= 17 - ((vcsr_vl * ((v_enc_width==3'b000)? 8:(v_enc_width==3'b101)?16:(v_enc_width==3'b110)?32:SEW)) >> 5);//right shift to divide with 32
							vecregs_waddr <= decoded_vd;
							vecregs_raddr1 <= decoded_vs1;
							mem_str_ready <= 0; //Initial value of mem_str_ready
							mem_str_state <= 2'b00; //Initial state for strided load
							//Updating mem_bits depending on vcsr_vl and v_enc_width(i.e instr[14:12])
							v_membits <= vcsr_vl * ((v_enc_width==3'b000)? 8:(v_enc_width==3'b101)?16:(v_enc_width==3'b110)?32:SEW); 
							//Number many bits to read in each cycle
							if(SEW == 10'b0000100000)
								mem_wordsize = 0;
							if((SEW == 10'b0000001000)) begin
								reg_op2 = 1; //Converting as strided load with stride 1
								mem_wordsize = 3;
							end
						end
						(instr_vload_str): begin
							$display("Inside v_load_stride condition, time:%d", $time);
							// mem_valid <= 1;
							cpu_state <= cpu_state_ldmem;
							vecldstrcnt <= 16; 
							var_vlen <= 17 - ((vcsr_vl * ((v_enc_width==3'b000)? 8:(v_enc_width==3'b101)?16:(v_enc_width==3'b110)?32:SEW)) >> 5);//right shift to divide with 32
							vecregs_waddr <= decoded_vd;
							vecregs_raddr1 <= decoded_vs1;
							mem_str_ready <= 0; //Initial value of mem_str_ready
							mem_str_state <= 2'b00; //Initial state for strided load
							//Updating mem_bits depending on vcsr_vl and v_enc_width(i.e instr[14:12])
							v_membits <= vcsr_vl * ((v_enc_width==3'b000)? 8:(v_enc_width==3'b101)?16:(v_enc_width==3'b110)?32:SEW); 
							//Number many bits to read in each cycle
							if(SEW == 10'b0000100000)
								mem_wordsize = 0;
							if((SEW == 10'b0000001000) && reg_op2 >= 4)
								mem_wordsize = 2;
							if((SEW == 10'b0000001000) && reg_op2 < 4)
								mem_wordsize = 3;
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

				// cpu_state_exec: begin
				// 	if(elem_n*BUS_WIDTH < vlen) begin
				// 		// $display("elem:%d, vecrs1: %x, vecrs2: %x,vlen:%d",elem_n,vecrs1,vecrs2, vlen);
				// 		case(elem_n)
				// 			0: begin
				// 					vecrs1 <= vreg_op1[1*BUS_WIDTH-1:0*BUS_WIDTH];
				// 					vecrs2 <= vreg_op2[1*BUS_WIDTH-1:0*BUS_WIDTH];
				// 					vecrs3  <= vreg_op3[1*BUS_WIDTH-1:0*BUS_WIDTH];
				// 				end
				// 			1: begin
				// 					vecrs1 <= vreg_op1[2*BUS_WIDTH-1:1*BUS_WIDTH];
				// 					vecrs2 <= vreg_op2[2*BUS_WIDTH-1:1*BUS_WIDTH];
				// 					vecrs3  <= vreg_op3[2*BUS_WIDTH-1:1*BUS_WIDTH];
				// 				end
				// 			2: begin
				// 					vecrs1 <= vreg_op1[3*BUS_WIDTH-1:2*BUS_WIDTH];
				// 					vecrs2 <= vreg_op2[3*BUS_WIDTH-1:2*BUS_WIDTH];
				// 					vecrs3  <= vreg_op3[3*BUS_WIDTH-1:2*BUS_WIDTH];
				// 					valu_out[1*BUS_WIDTH-1:0*BUS_WIDTH] <= vecrd;
				// 				end
				// 			3: begin
				// 					vecrs1 <= vreg_op1[4*BUS_WIDTH-1:3*BUS_WIDTH];
				// 					vecrs2 <= vreg_op2[4*BUS_WIDTH-1:3*BUS_WIDTH];
				// 					vecrs3  <= vreg_op3[4*BUS_WIDTH-1:3*BUS_WIDTH];
				// 					valu_out[2*BUS_WIDTH-1:1*BUS_WIDTH] <= vecrd;								
				// 				end
				// 			4: begin
				// 					vecrs1 <= vreg_op1[5*BUS_WIDTH-1:4*BUS_WIDTH];
				// 					vecrs2 <= vreg_op2[5*BUS_WIDTH-1:4*BUS_WIDTH];
				// 					vecrs3  <= vreg_op3[5*BUS_WIDTH-1:4*BUS_WIDTH];
				// 					valu_out[3*BUS_WIDTH-1:2*BUS_WIDTH] <= vecrd;	
				// 				end
				// 			5: begin
				// 					vecrs1 <= vreg_op1[6*BUS_WIDTH-1:5*BUS_WIDTH];
				// 					vecrs2 <= vreg_op2[6*BUS_WIDTH-1:5*BUS_WIDTH];
				// 					vecrs3 <= vreg_op3[6*BUS_WIDTH-1:5*BUS_WIDTH];
				// 					valu_out[4*BUS_WIDTH-1:3*BUS_WIDTH] <= vecrd;	
				// 				end
				// 			6: begin
				// 					vecrs1 <= vreg_op1[7*BUS_WIDTH-1:6*BUS_WIDTH];
				// 					vecrs2 <= vreg_op2[7*BUS_WIDTH-1:6*BUS_WIDTH];
				// 					vecrs3  <= vreg_op3[7*BUS_WIDTH-1:6*BUS_WIDTH];
				// 					valu_out[5*BUS_WIDTH-1:4*BUS_WIDTH] <= vecrd;	
				// 				end
				// 			7: begin
				// 					vecrs1 <= vreg_op1[8*BUS_WIDTH-1:7*BUS_WIDTH];
				// 					vecrs2 <= vreg_op2[8*BUS_WIDTH-1:7*BUS_WIDTH];
				// 					vecrs3  <= vreg_op3[8*BUS_WIDTH-1:7*BUS_WIDTH];
				// 					valu_out[6*BUS_WIDTH-1:5*BUS_WIDTH] <= vecrd;	
				// 				end
				// 			8: begin
				// 					vecrs1 <= vreg_op1[9*BUS_WIDTH-1:8*BUS_WIDTH];
				// 					vecrs2 <= vreg_op2[9*BUS_WIDTH-1:8*BUS_WIDTH];
				// 					vecrs3  <= vreg_op3[9*BUS_WIDTH-1:8*BUS_WIDTH];
				// 					valu_out[7*BUS_WIDTH-1:6*BUS_WIDTH] <= vecrd;	
				// 				end
				// 			9: begin
				// 					vecrs1 <= vreg_op1[10*BUS_WIDTH-1:9*BUS_WIDTH];
				// 					vecrs2 <= vreg_op2[10*BUS_WIDTH-1:9*BUS_WIDTH];
				// 					vecrs3  <= vreg_op3[10*BUS_WIDTH-1:9*BUS_WIDTH];
				// 					valu_out[8*BUS_WIDTH-1:7*BUS_WIDTH] <= vecrd;	
				// 				end
				// 			10: begin
				// 					vecrs1 <= vreg_op1[11*BUS_WIDTH-1:10*BUS_WIDTH];
				// 					vecrs2 <= vreg_op2[11*BUS_WIDTH-1:10*BUS_WIDTH];
				// 					vecrs3  <= vreg_op3[11*BUS_WIDTH-1:10*BUS_WIDTH];
				// 					valu_out[9*BUS_WIDTH-1:8*BUS_WIDTH] <= vecrd;	
				// 				end
				// 			11: begin
				// 					vecrs1 <= vreg_op1[12*BUS_WIDTH-1:11*BUS_WIDTH];
				// 					vecrs2 <= vreg_op2[12*BUS_WIDTH-1:11*BUS_WIDTH];
				// 					vecrs3  <= vreg_op3[12*BUS_WIDTH-1:11*BUS_WIDTH];
				// 					valu_out[10*BUS_WIDTH-1:9*BUS_WIDTH] <= vecrd;	
				// 				end
				// 			12: begin
				// 					vecrs1 <= vreg_op1[13*BUS_WIDTH-1:12*BUS_WIDTH];
				// 					vecrs2 <= vreg_op2[13*BUS_WIDTH-1:12*BUS_WIDTH];
				// 					vecrs3  <= vreg_op3[13*BUS_WIDTH-1:12*BUS_WIDTH];
				// 					valu_out[11*BUS_WIDTH-1:10*BUS_WIDTH] <= vecrd;	
				// 				end
				// 			13: begin
				// 					vecrs1 <= vreg_op1[14*BUS_WIDTH-1:13*BUS_WIDTH];
				// 					vecrs2 <= vreg_op2[14*BUS_WIDTH-1:13*BUS_WIDTH];
				// 					vecrs3 <= vreg_op3[14*BUS_WIDTH-1:13*BUS_WIDTH];
				// 					valu_out[12*BUS_WIDTH-1:11*BUS_WIDTH] <= vecrd;	
				// 				end
				// 			14: begin
				// 					vecrs1 <= vreg_op1[15*BUS_WIDTH-1:14*BUS_WIDTH];
				// 					vecrs2 <= vreg_op2[15*BUS_WIDTH-1:14*BUS_WIDTH];
				// 					vecrs3  <= vreg_op3[15*BUS_WIDTH-1:14*BUS_WIDTH];
				// 					valu_out[13*BUS_WIDTH-1:12*BUS_WIDTH] <= vecrd;	
				// 				end
				// 			15: begin
				// 					vecrs1 <= vreg_op1[16*BUS_WIDTH-1:15*BUS_WIDTH];
				// 					vecrs2 <= vreg_op2[16*BUS_WIDTH-1:15*BUS_WIDTH];
				// 					vecrs3  <= vreg_op3[16*BUS_WIDTH-1:15*BUS_WIDTH];
				// 					valu_out[14*BUS_WIDTH-1:13*BUS_WIDTH] <= vecrd;	
				// 				end
				// 			default: begin
				// 						vecrs1 <= vecrs1;
				// 						vecrs2 <= vecrs2;
				// 						vecrs3 <= vecrs3;
				// 					 end
				// 		endcase
				// 		elem_n <= elem_n + 1;
				//     end
				// 	else if(elem_n < 18) begin 
				// 		case(elem_n)
				// 			16: begin
				// 					vecrs1 <= vreg_op1[16*BUS_WIDTH-1:15*BUS_WIDTH];
				// 					vecrs2 <= vreg_op2[16*BUS_WIDTH-1:15*BUS_WIDTH];
				// 					vecrs3  <= vreg_op3[16*BUS_WIDTH-1:15*BUS_WIDTH];
				// 					valu_out[15*BUS_WIDTH-1:14*BUS_WIDTH] <= vecrd;	
				// 				end
				// 			17: begin
				// 					vecrs1 <= vreg_op1[16*BUS_WIDTH-1:15*BUS_WIDTH];
				// 					vecrs2 <= vreg_op2[16*BUS_WIDTH-1:15*BUS_WIDTH];
				// 					vecrs3  <= vreg_op3[16*BUS_WIDTH-1:15*BUS_WIDTH];
				// 					valu_out[16*BUS_WIDTH-1:15*BUS_WIDTH] <= vecrd;	
				// 					pcpi_ready <= 1;
				// 					pcpi_wait <= 0; //Making the wait flag 0 after execution
				// 					latched_vstore <= 1;
				// 					latched_stalu <= 1; //To store the valu_out in vreg
				// 					cpu_state <= cpu_state_fetch;
				// 				end
				// 		endcase
				// 		elem_n <= elem_n + 1;
				// 	end	
				// 	pcpi_wait <= 1; //Since the execution of vect instrn takes more than 2 clock cycles
				// 	// $display("Instruction inside co-processor: 0x%x, pcpi_valid: %b, elem_n: %d",pcpi_insn, pcpi_valid, elem_n);
				// 	// $display("rs1 data inside co-processor: 0x%x",vecrs1);
				// 	// $display("rs2 data inside co-processor: 0x%x",vecrs2); 
				// 	// $display("Valu_out: 0x%x",valu_out);
				// 	//since param is compile time constant, the unused branch in each instantiation will be optimized out leading to no additional hardware.
				// 	if(BUS_WIDTH == 8'B00100000) begin //If the bus width is 32 bits
				// 		//If SEW is 8 
				// 		if(SEW == 10'b0000001000) begin
				// 			if(instr_vadd) begin
				// 				//Assuming that there are 4 ALUs in the co-processor
				// 				vecrd[31:24] <= vecrs1[31:24] + vecrs2[31:24];
				// 				vecrd[23:16] <= vecrs1[23:16] + vecrs2[23:16];
				// 				vecrd[15:8]  <= vecrs1[15:8]  + vecrs2[15:8];
				// 				vecrd[7:0]   <= vecrs1[7:0]   + vecrs2[7:0];
				// 			end
				// 			else if(instr_vdot) begin
				// 				//Assuming that there are 4 ALUs in the co-processor
				// 				vecrd[31:24] <= vecrs1[31:24] * vecrs2[31:24] + vecrs3[31:24];
				// 				vecrd[23:16] <= vecrs1[23:16] * vecrs2[23:16] + vecrs3[23:16];
				// 				vecrd[15:8]  <= vecrs1[15:8]  * vecrs2[15:8] + vecrs3[15:8];
				// 				vecrd[7:0]   <= vecrs1[7:0]   * vecrs2[7:0] + vecrs3[7:0];
				// 			end
				// 		end
				// 		//If SEW is 16 
				// 		else if(SEW == 10'b0000010000) begin
				// 			if(instr_vadd) begin
				// 				//Assuming that there are 2 16-bit ALUs in the co-processor
				// 				vecrd[31:16] <= vecrs1[31:16] + vecrs2[31:16];
				// 				vecrd[15:0]  <= vecrs1[15:0]  + vecrs2[15:0];
				// 			end
				// 			else if(instr_vdot) begin
				// 				//Assuming that there are 2 16-bit in the co-processor
				// 				vecrd[31:16] <= vecrs1[31:16] * vecrs2[31:16] + vecrs3[31:16];
				// 				vecrd[15:0]  <= vecrs1[15:0]  * vecrs2[15:0] + vecrs3[15:0];
				// 			end
				// 		end
				// 		//If SEW is 32
				// 		else if(SEW == 10'b0000100000) begin
				// 			if(instr_vadd) begin
				// 				//Assuming that there are 1 32-bit ALU in the co-processor
				// 				vecrd[31:0] <= vecrs1[31:0] + vecrs2[31:0];
				// 			end
				// 			else if(instr_vdot) begin
				// 				//Assuming that there are1 32-bit ALU in the co-processor
				// 				// $display("vecrd: %d", vecrd[31:0]);
				// 				vecrd[31:0] <= vecrs1[31:0] * vecrs2[31:0] + vecrs3[31:0];
				// 				// $display("vecrd: %d", vecrd[31:0]);
				// 			end
				// 		end
				// 	end

				// 	else if(BUS_WIDTH == 8'B01000000) begin //If the bus width is 64 bits
				// 		//If SEW is 8
				// 		if(SEW == 10'b 0000001000) begin
				// 			if(instr_vadd) begin
				// 				//Assuming that there are 8 ALUs in the co-processor
				// 				vecrd[63:56] <= vecrs1[63:56] + vecrs2[63:56];
				// 				vecrd[55:48] <= vecrs1[55:48] + vecrs2[55:48];
				// 				vecrd[47:40] <= vecrs1[47:40] + vecrs2[47:40];
				// 				vecrd[39:32] <= vecrs1[39:32] + vecrs2[39:32];
				// 				vecrd[31:24] <= vecrs1[31:24] + vecrs2[31:24];
				// 				vecrd[23:16] <= vecrs1[23:16] + vecrs2[23:16];
				// 				vecrd[15:8]  <= vecrs1[15:8]  + vecrs2[15:8];
				// 				vecrd[7:0]   <= vecrs1[7:0]   + vecrs2[7:0];
				// 			end
				// 			else if(instr_vdot) begin
				// 				//Assuming that there are 8 ALUs in the co-processor
				// 				vecrd[63:56] <= vecrs1[63:56] * vecrs2[63:56];
				// 				vecrd[55:48] <= vecrs1[55:48] * vecrs2[55:48];
				// 				vecrd[47:40] <= vecrs1[47:40] * vecrs2[47:40];
				// 				vecrd[39:32] <= vecrs1[39:32] * vecrs2[39:32];
				// 				vecrd[31:24] <= vecrs1[31:24] * vecrs2[31:24];
				// 				vecrd[23:16] <= vecrs1[23:16] * vecrs2[23:16];
				// 				vecrd[15:8]  <= vecrs1[15:8]  * vecrs2[15:8];
				// 				vecrd[7:0]   <= vecrs1[7:0]   * vecrs2[7:0];
				// 			end
				// 		end
				// 		//If SEW is 16
				// 		else if(SEW == 10'b 0000010000) begin
				// 			if(instr_vadd) begin
				// 				//Assuming that there are 4 16-bit ALUs in the co-processor
				// 				vecrd[63:48] <= vecrs1[63:48] + vecrs2[63:48];
				// 				vecrd[47:32] <= vecrs1[47:32] + vecrs2[47:32];
				// 				vecrd[31:16] <= vecrs1[31:16] + vecrs2[31:16];
				// 				vecrd[15:0]  <= vecrs1[15:0]  + vecrs2[15:0];
				// 			end
				// 			else if(instr_vdot) begin
				// 				//Assuming that there are 4 16-bit ALUs in the co-processor
				// 				vecrd[63:48] <= vecrs1[63:48] * vecrs2[63:48];
				// 				vecrd[47:32] <= vecrs1[47:32] * vecrs2[47:32];
				// 				vecrd[31:16] <= vecrs1[31:16] * vecrs2[31:16];
				// 				vecrd[15:0]  <= vecrs1[15:0]  * vecrs2[15:0];
				// 			end
				// 		end
				// 		//If SEW is 32
				// 		else if(SEW == 10'b 0000100000) begin
				// 			if(instr_vadd) begin
				// 				//Assuming that there are 2 32-bit ALUs in the co-processor
				// 				vecrd[63:32] <= vecrs1[63:32] + vecrs2[63:32];
				// 				vecrd[31:0]  <= vecrs1[31:0] + vecrs2[31:0];
				// 			end
				// 			else if(instr_vdot) begin
				// 				//Assuming that there are 8 ALUs in the co-processor
				// 				vecrd[63:32] <= vecrs1[63:32] * vecrs2[63:32]  + vecrs3[63:32];
				// 				vecrd[31:0]  <= vecrs1[31:0] * vecrs2[31:0]  + vecrs3[31:0];
				// 			end
				// 		end
				// 	end

				// 	else if(BUS_WIDTH == 8'B10000000) begin //If the bus width is 128 bits
				// 		//If SEW is 8
				// 		if(SEW == 10'b 0000001000) begin
				// 			if(instr_vadd) begin
				// 				//Assuming that there are 16 ALUs in the co-processor
				// 				vecrd[127:120] <= vecrs1[127:120] + vecrs2[127:120];
				// 				vecrd[119:112] <= vecrs1[119:112] + vecrs2[119:112];
				// 				vecrd[111:104] <= vecrs1[111:104] + vecrs2[111:104];
				// 				vecrd[103:96]  <= vecrs1[103:96] + vecrs2[103:96];
				// 				vecrd[95:88]   <= vecrs1[95:88] + vecrs2[95:88];
				// 				vecrd[87:80]   <= vecrs1[87:80] + vecrs2[87:80];
				// 				vecrd[79:72]   <= vecrs1[79:72] + vecrs2[79:72];
				// 				vecrd[71:64]   <= vecrs1[71:64] + vecrs2[71:64];
				// 				vecrd[63:56]   <= vecrs1[63:56] + vecrs2[63:56];
				// 				vecrd[55:48]   <= vecrs1[55:48] + vecrs2[55:48];
				// 				vecrd[47:40]   <= vecrs1[47:40] + vecrs2[47:40];
				// 				vecrd[39:32]   <= vecrs1[39:32] + vecrs2[39:32];
				// 				vecrd[31:24]   <= vecrs1[31:24] + vecrs2[31:24];
				// 				vecrd[23:16]   <= vecrs1[23:16] + vecrs2[23:16];
				// 				vecrd[15:8]    <= vecrs1[15:8]  + vecrs2[15:8];
				// 				vecrd[7:0]     <= vecrs1[7:0]   + vecrs2[7:0];
				// 			end
				// 			else if (instr_vdot) begin
				// 				//Assuming that there are 16 ALUs in the co-processor
				// 				vecrd[127:120] <= vecrs1[127:120] * vecrs2[127:120];
				// 				vecrd[119:112] <= vecrs1[119:112] * vecrs2[119:112];
				// 				vecrd[111:104] <= vecrs1[111:104]   * vecrs2[111:104];
				// 				vecrd[103:96]  <= vecrs1[103:96]  * vecrs2[103:96];
				// 				vecrd[95:88]   <= vecrs1[95:88] * vecrs2[95:88];
				// 				vecrd[87:80]   <= vecrs1[87:80] * vecrs2[87:80];
				// 				vecrd[79:72]   <= vecrs1[79:72] * vecrs2[79:72];
				// 				vecrd[71:64]   <= vecrs1[71:64] * vecrs2[71:64];
				// 				vecrd[63:56]   <= vecrs1[63:56] * vecrs2[63:56];
				// 				vecrd[55:48]   <= vecrs1[55:48] * vecrs2[55:48];
				// 				vecrd[47:40]   <= vecrs1[47:40] * vecrs2[47:40];
				// 				vecrd[39:32]   <= vecrs1[39:32] * vecrs2[39:32];
				// 				vecrd[31:24]   <= vecrs1[31:24] * vecrs2[31:24];
				// 				vecrd[23:16]   <= vecrs1[23:16] * vecrs2[23:16];
				// 				vecrd[15:8]    <= vecrs1[15:8]  * vecrs2[15:8];
				// 				vecrd[7:0]     <= vecrs1[7:0]   * vecrs2[7:0];
				// 			end
				// 		end
				// 		//If SEW is 16
				// 		else if(SEW == 10'b 0000010000) begin
				// 			if(instr_vadd) begin
				// 				//Assuming that there are 8 16-bit ALUs in the co-processor
				// 				vecrd[127:112] <= vecrs1[127:112] + vecrs2[127:112];
				// 				vecrd[111:96] <= vecrs1[111:96] + vecrs2[111:96];
				// 				vecrd[95:80]   <= vecrs1[95:80] + vecrs2[95:80];
				// 				vecrd[79:64]   <= vecrs1[79:64] + vecrs2[79:64];
				// 				vecrd[63:48]   <= vecrs1[63:48] + vecrs2[63:48];
				// 				vecrd[47:32]   <= vecrs1[47:32] + vecrs2[47:32];
				// 				vecrd[31:16]   <= vecrs1[31:16] + vecrs2[31:16];
				// 				vecrd[15:0]    <= vecrs1[15:0]  + vecrs2[15:0];
				// 			end
				// 			else if (instr_vdot) begin
				// 				//Assuming that there are 8 16-bit ALUs in the co-processor
				// 				vecrd[127:112] <= vecrs1[127:112] * vecrs2[127:112];
				// 				vecrd[111:96]  <= vecrs1[111:96] * vecrs2[111:96];
				// 				vecrd[95:80]   <= vecrs1[95:80] * vecrs2[95:80];
				// 				vecrd[79:64]   <= vecrs1[79:64] * vecrs2[79:64];
				// 				vecrd[63:48]   <= vecrs1[63:48] * vecrs2[63:48];
				// 				vecrd[47:32]   <= vecrs1[47:32] * vecrs2[47:32];
				// 				vecrd[31:16]   <= vecrs1[31:16] * vecrs2[31:16];
				// 				vecrd[15:0]    <= vecrs1[15:0]  * vecrs2[15:0];
				// 			end
				// 		end
				// 		//If SEW is 32
				// 		else if(SEW == 10'b 0000100000) begin
				// 			if(instr_vadd) begin
				// 				//Assuming that there are 4 32-bit ALUs in the co-processor
				// 				vecrd[127:96] <= vecrs1[127:96] + vecrs2[127:96];
				// 				vecrd[95:64]  <= vecrs1[95:64] + vecrs2[95:64];
				// 				vecrd[63:32]  <= vecrs1[63:32] + vecrs2[63:32];
				// 				vecrd[31:0]   <= vecrs1[31:0] + vecrs2[31:0];
				// 			end
				// 			else if (instr_vdot) begin
				// 				//Assuming that there are 4 32-bit ALUs in the co-processor
				// 				vecrd[127:96] <= vecrs1[127:96] * vecrs2[127:96];
				// 				vecrd[95:64]   <= vecrs1[95:64] * vecrs2[95:64];
				// 				vecrd[63:32]   <= vecrs1[63:32] * vecrs2[63:32];
				// 				vecrd[31:0]   <= vecrs1[31:0] * vecrs2[31:0];
				// 			end
				// 		end
				// 	end
				// end
				
				cpu_state_ldmem: begin
					mem_valid <= 1; //Making the valid bit after changing the address for the first time
					if(vecldstrcnt >= var_vlen) begin
						// $display("Inside ldmem condition. Time: %d, membits: %d, stride: %d, SEW: %d ",$time,v_membits, reg_op2, SEW);
						if(v_membits%32==0) begin 
							//If SEW is 32 bits, then depending on stride, we read the corresponding 32 bits
							if(SEW == 10'b0000100000) begin	
								if(mem_ready == 1) begin //This is how memory works
									if(instr_vload) //For unit stride load
										reg_op1 = reg_op1 + 4;
									//reg_op2 contains the stride
									else if(instr_vload_str) begin
										reg_op1 = reg_op1 + reg_op2; //If it is strided load, increase the PC using stride
									end
								end
								// $display("Inside stride condition, mem_ready: %b vecldct: %d, mem_str_ready: %b, mem_addr: %d, mem_rdata:%x, time = %d",mem_ready, vecldstrcnt, mem_str_ready, reg_op1, mem_rdata, $time);
								if(mem_str_ready == 1) begin
										v_membits <= v_membits-32;
										set_mem_do_rdata = 1;
										vecldstrcnt <= vecldstrcnt -1;	
								end
							end
							//If SEW is 8 bits, there will be many cases depending on the start address and the stride
							else if(SEW == 10'b0000001000) begin   
								if(instr_vload) begin    //For unit stride load (check this)
									if(mem_ready == 1)
										reg_op1 = reg_op1 + 1; //Unit byte stride
									if(mem_str_ready == 1) begin
										// $display("var_length: %d, mem_ready:%d, time:%d",var_vlen, mem_ready, $time);
										v_membits <= v_membits-32;
										vecldstrcnt <= vecldstrcnt -1;	
										set_mem_do_rdata = 1;
									end
								end 
								if(instr_vload_str) begin
									if(reg_op2 >= 4) begin //In this case, 4 data reads are needed for each 32 bit value
										if(mem_ready == 1)		
											reg_op1 = reg_op1 + reg_op2;
										// $display("Inside stride condition, vecldct: %d, mem_str_ready: %b, mem_addr: %d, mem_rdata:%x, time = %d",vecldstrcnt, mem_str_ready, mem_addr, mem_rdata, $time);
										if(mem_str_ready == 1) begin
											// $display("var_length: %d, mem_ready:%d, time:%d",var_vlen, mem_ready, $time);
											v_membits <= v_membits-32;
											vecldstrcnt <= vecldstrcnt -1;	
											set_mem_do_rdata = 1;
										end
									end
									else if(reg_op2 < 4) begin
										// $display("Inside stride condition, vecldct: %d, mem_str_ready: %b, mem_addr: %d, mem_rdata:%x, time = %d",vecldstrcnt, mem_str_ready, mem_addr, mem_rdata, $time);
										if(mem_ready == 1)
											//Read the next memory location every time. Since it is byte addressable, increasing it by 4
											reg_op1 = reg_op1 + 4;
										if(mem_str_ready == 1) begin
											// $display("var_length: %d, mem_ready:%d, time:%d",var_vlen, mem_ready, $time);
											v_membits <= v_membits-32;
											vecldstrcnt <= vecldstrcnt -1;	
											set_mem_do_rdata = 1;
										end
									end
								end
							end
						end
						if((mem_str_ready == 1)) begin  //(mem_ready == 1) && 
							// $display("Inside mem_str_ready,vecldstrcnt:%d, data: %x, time:%d",vecldstrcnt, mem_rdata_word, $time);
							vreg_op1 = mem_rdata_word;
							vecregs_wstrb_temp = 1 << (16 - vecldstrcnt); //left shift by 16-vecldstcnt bits
							latched_vstore <= 1;
							$display("Inside mem_str_ready,vecldstrcnt:%d, data: %x, latched_vs:%d, time:%d",vecldstrcnt, mem_rdata_word,latched_vstore, $time);
							//If we are loading the last word, then make pcpi_ready as 1 
							if(vecldstrcnt == var_vlen) begin
								mem_valid <= 0;
								cpu_state <= cpu_state_fetch;
								pcpi_wait <= 0;
								pcpi_ready <= 1;
							end
						end
					end
					mem_addr = reg_op1; //Updating the mem_addr
				end

				//This state is for store memory instruction
				cpu_state_stmem: begin
					mem_valid <= 1; //Making the valid bit after assigning the data
					// $display("Inside stmem condition. Time: %d, vregs_d1: %x, vreg_rdata_latched:%x",$time,vecregs_rdata1, vreg_rdata1_latched);
					if(vecldstrcnt >= var_vlen) begin
						// $display("Inside stmem condition. Time: %d, membits: %d, vecldsstrcnt:%d",$time,v_membits, vecldstrcnt);
						if(v_membits%32==0) begin
							// mem_wordsize <= 0;
							if(SEW == 10'b0000100000) begin	
								if((mem_ready == 0) && (vstore_bit >= 1)) begin  
									// $display("Inside mem_ready, vecldct: %d, mem_addr: %d, vstore_bit:%d, mem_wdata:%x, time = %d",vecldstrcnt, reg_op1, vstore_bit, mem_wdata, $time);
									if(instr_vstore) //For unit stride store
										reg_op1 = reg_op1 + 4;
									//reg_op2 contains the stride
									else if(instr_vstore_str) 
										reg_op1 = reg_op1 + reg_op2; //If it is strided store, increase the PC using stride
								end
								if(mem_str_ready == 1) begin
									v_membits <= v_membits-32;
									vecldstrcnt <= vecldstrcnt -1;	
								end
							end	
							//If SEW is 8 bits, there will be many cases depending on the start address and the stride
							else if(SEW == 10'b0000001000) begin   
								if(instr_vstore) begin    //For unit stride load (check this)
									if((mem_ready == 1) && (vstore_bit == 2)) 
										reg_op1 = reg_op1 + 1;
									if(mem_str_ready == 1) begin
										// $display("var_length: %d, mem_ready:%d, time:%d",var_vlen, mem_ready, $time);
										v_membits <= v_membits-32;
										vecldstrcnt <= vecldstrcnt -1;	
										// set_mem_do_rdata = 1;
									end
								end 
								if(instr_vstore_str) begin
									if(reg_op2 >= 4) begin //In this case, 4 data reads are needed for each 32 bit value
										if((mem_ready == 0) && (vstore_bit >= 1))	begin
											reg_op1 = reg_op1 + reg_op2;
											// $display("Inside mem_ready, vecldct: %d, mem_addr: %d, vstore_bit:%d, mem_wdata:%x, time = %d",vecldstrcnt, reg_op1, vstore_bit, mem_wdata, $time);
										end
										if(mem_str_ready == 1) begin  
											// $display("var_length: %d, mem_ready:%d, time:%d",var_vlen, mem_ready, $time);
											v_membits <= v_membits-32;
											vecldstrcnt <= vecldstrcnt -1;	
											// set_mem_do_rdata = 1;
										end
									end
									else if(reg_op2 < 4) begin
										// $display("Inside stride condition, vecldct: %d, mem_str_ready: %b, mem_addr: %d, mem_rdata:%x, time = %d",vecldstrcnt, mem_str_ready, mem_addr, mem_rdata, $time);
										if((mem_ready == 0) && (vstore_bit >= 1)) 
											//Read the next memory location every time. Since it is byte addressable, increasing it by 4
											reg_op1 = reg_op1 + 4;
										if(mem_str_ready == 1) begin
											// $display("var_length: %d, mem_ready:%d, time:%d",var_vlen, mem_ready, $time);
											v_membits <= v_membits-32;
											vecldstrcnt <= vecldstrcnt -1;	
											// set_mem_do_rdata = 1;
										end
									end
								end
							end						
						end
						if(mem_str_ready == 1)	begin
							set_mem_do_wdata = 1; //set wstrb to 1 when data is changed 
							if(vstore_bit < 2)
								vstore_bit = vstore_bit + 1;
							vreg_rdata1_latched = vecregs_rdata1;
							if((SEW == 10'b0000001000) && (reg_op2 < 4))
								vecregs_rstrb1 = 1 << (18 - vecldstrcnt + 1); 
							else
								vecregs_rstrb1 = 1 << (17 - vecldstrcnt + 1); 
							$display("Inside mem_str_ready,vecldstrcnt:%d, data: %x, addr:%d, time:%d",vecldstrcnt, vreg_rdata1_latched, reg_op1, $time);
							if(vecldstrcnt == var_vlen) begin
								mem_valid <= 0;  // Making the bit 0 in fetch stage so that the data is written to memory before valid is 0
								cpu_state <= cpu_state_fetch;
								pcpi_wait <= 0;
								// latched_vstore <= 1;
								pcpi_ready <= 1;
							end
						end
					end
					mem_addr <= reg_op1; //Updating the mem_addr
				end

			endcase
		end
		if (set_mem_do_rdata)
			mem_do_rdata <= 1;
		if (set_mem_do_wdata)
			mem_do_wdata <= 1;
	end
endmodule
