// //Used for vector arithmetic instructions
// 	localparam p_instr_vadd__vv = 8'h00 ;
// 	localparam p_instr_vmul__vv = 8'h01 ;
// 	localparam p_instr_vdot__vv = 8'h02 ;
// 	localparam p_instr_vaddvarp = 8'h03 ;
// 	localparam p_instr_vmulvarp = 8'h04 ;
// 	localparam p_instr_vdotvarp = 8'h05 ;

module alu_block(
    input clk,
    input resetn, //low level triggered (i.e if resetn is 0 is reset)
    input [7:0] micro_exec_instr,
    input [9:0] SEW,
    input [3:0] vap,
    input [31:0] opA,
    input [31:0] opB,
    input [31:0] opC,
    output reg [31:0] alu_out,
    output alu_done
);

localparam instr_vadd = 8'h00 ;
localparam instr_vmul = 8'h01 ;
localparam instr_vdot = 8'h02 ;
localparam instr_vaddvarp = 8'h03 ;
localparam instr_vmulvarp = 8'h04 ;
localparam instr_vdotvarp = 8'h05 ;

reg [31:0] new_opA, new_opB, new_opC;
wire [31:0] pe_out;
reg alu_enb, temp_reg=0;
wire is_vec_instr, is_vap_instr, is_port3_instr;
assign is_vec_instr = |{(micro_exec_instr == instr_vdot), (micro_exec_instr == instr_vadd), (micro_exec_instr == instr_vmul),
                        (micro_exec_instr == instr_vaddvarp), (micro_exec_instr == instr_vmulvarp), (micro_exec_instr == instr_vdotvarp)};
assign is_vap_instr = |{(micro_exec_instr == instr_vaddvarp), (micro_exec_instr == instr_vmulvarp), (micro_exec_instr == instr_vdotvarp)}; //To heck whether the instruction is variable bit one
assign is_port3_instr = |{(micro_exec_instr == instr_vdot), (micro_exec_instr == instr_vdotvarp)};
wire done1; 
assign alu_done = &{done1};


    always @(posedge clk) begin
        if(!resetn || temp_reg) begin
            // $display("Entered reset condition, alu_enb:%b, time:%d",alu_enb, $time);
            alu_enb <= 0;
            new_opA <= 0;
            new_opB <= 0;
            new_opC <= 0;
        end
        else if(!alu_enb && is_vec_instr) begin
            if(instr_vadd || instr_vmul || instr_vdot) begin
                // $display("Entered vector condition, alu_enb:%b, time:%d",alu_enb, $time);
                new_opA[31:0]  <= opA[31:0];
                new_opB[31:0]  <= opB[31:0];
                if(is_port3_instr) begin
                    new_opC[31:0]  <= opC[31:0];
                end
                alu_enb <= 1; 
            end
            if(is_vap_instr) begin
                // $display("Entered is_vap_instr condition, alu_enb:%b, time:%d",alu_enb, $time);
                if(vap == 10'b0000000001) begin
                    // $display("vecreg_data: %x, cnt:%d, time:%d", opA, cnt, $time);
                    //Converting sew of 1 to 8 bits to operate on them and zero padding on left side (for better multiplication)
                    new_opA[31:0]    <= {{8{opA[24]}}, {8{opA[16]}}, {8{opA[8]}}, {8{opA[0]}}};
                    new_opB[31:0]    <= {{8{opB[24]}}, {8{opB[16]}}, {8{opB[8]}}, {8{opB[0]}}};
                    // new_opA[31:0]    <= {{8{opA[56]}}, {8{opA[48]}}, {8{opA[40]}}, {8{opA[32]}}, {8{opA[24]}}, {8{opA[16]}}, {8{opA[8]}}, {8{opA[0]}}};
                    // new_opB[31:0]    <= {{8{opB[56]}}, {8{opB[48]}}, {8{opB[40]}}, {8{opB[32]}}, {8{opB[24]}}, {8{opB[16]}}, {8{opB[8]}}, {8{opB[0]}}};
                    if(is_port3_instr) begin
                        new_opC[31:0]     <= {{8{opC[24]}}, {8{opC[16]}}, {8{opC[8]}}, {8{opC[0]}}};
                        // new_opC[31:0]     <= {{8{opC[56]}}, {8{opC[48]}}, {8{opC[40]}}, {8{opC[32]}}, {8{opC[24]}}, {8{opC[16]}}, {8{opC[8]}}, {8{opC[0]}}};
                    end
                    alu_enb <= 1; 
                end
                else if(vap == 10'b0000000010) begin
                    new_opA[31:0]    <= {{6{opA[25]}},opA[25:24],{6{opA[17]}},opA[17:16],{6{opA[9]}},opA[9:8],{6{opA[1]}},opA[1:0]};
                    new_opB[31:0]    <= {opB[25:24],6'b0,opB[17:16],6'b0,opB[9:8],6'b0,opB[1:0],6'b0};
                    if(is_port3_instr) begin
                        new_opC[31:0]    <= {{6{opC[25]}},opC[25:24],{6{opC[17]}},opC[17:16],{6{opC[9]}},opC[9:8],{6{opC[1]}},opC[1:0]};
                    end
                    alu_enb <= 1; 
                end
                else if(vap == 10'b0000000100) begin
                    new_opA[31:0]     <= {{4{opA[27]}},opA[27:24],{4{opA[19]}},opA[19:16],{4{opA[11]}},opA[11:8],{4{opA[3]}},opA[3:0]};
                    new_opB[31:0]     <= {opB[27:24],4'b0,opB[19:16],4'b0,opB[11:8],4'b0,opB[3:0],4'b0};
                    if(is_port3_instr) begin
                        new_opC[31:0]    <= {{4{opC[27]}},opC[27:24],{4{opC[19]}},opC[19:16],{4{opC[11]}},opC[11:8],{4{opC[3]}},opC[3:0]};
                    end
                    alu_enb <= 1; 
                end
                else if(vap == 10'b0000001000) begin
                    // $display("vecreg_data: %x, cnt:%d, time:%d", opA, cnt, $time);
                    new_opA[31:0]  <= opA[31:0];
                    new_opB[31:0]  <= opB[31:0];
                    if(is_port3_instr) begin
                        new_opC[31:0]  <= opC[31:0];
                    end
                    alu_enb <= 1; 
                end
            end
        end
    end

    always @(posedge clk) begin
        if(!resetn) begin
            alu_out <= 0;
        end
        else if(alu_done && is_vec_instr) begin
            $display("Done is ready, pe_out:%h, time:%d", pe_out[31:0], $time);
            alu_out = pe_out;
            temp_reg = 1;
        end
    end 

    vector_processing_element pe1(.clk(clk),.reset(resetn),.instruction(micro_exec_instr),.start(alu_enb),.done(done1),.opA(new_opA[31:0]),.opB(new_opB[31:0]),.opC(new_opC[31:0]),.peout(pe_out[31:0]),.SEW(SEW),.vap(vap));

endmodule

module vector_processing_element(
    input clk,
    input reset,
    
    input [7:0] instruction,
	input start,
	output reg done,
	
    input [31:0] opA,
    input [31:0] opB,
    input [31:0] opC,
    output reg [31:0] peout,

	input [9:0] SEW,
	input [3:0 ] vap
);

reg [3:0] states;
localparam startstate = 4'h0 ;
localparam multstate  = 4'h1 ;
localparam completestate = 4'h2 ;

//Used for vector arithmetic instructions
localparam instr_vadd__vv = 8'h00 ;
localparam instr_vmul__vv = 8'h01 ;
localparam instr_vdot__vv = 8'h02 ;
localparam instr_vaddvarp = 8'h03 ;
localparam instr_vmulvarp = 8'h04 ;
localparam instr_vdotvarp = 8'h05 ;
localparam instr_vsub__vv = 8'h06;
localparam instr_vsubvarp = 8'h07;

reg [31:0] accumulator;  //Stores the final result
reg [7:0]  cycles;
reg first_cmpte;
reg [31:0] copB;

wire is_vap_instr = |{instruction==instr_vaddvarp, instruction==instr_vmulvarp, instruction==instr_vdotvarp, instruction==instr_vsubvarp};

always @(posedge clk) begin
    if(!reset || !start) begin
        states = 0;
        peout  = peout;
        accumulator = 0;
        first_cmpte = 0;
        cycles = 0;
        copB = 0;
        done = 0;
    end
    else if(start) begin
        case(states)
            startstate:begin
                if(start)begin
                    // $display("Entered start state,instr:%b, reset:%d, time:%d",instruction, reset, $time);
                    done <= 0; //Should be this, don't change
                    accumulator <= 0;
                    if(|{instruction == instr_vmul__vv,instruction == instr_vmulvarp,instruction == instr_vdot__vv, instruction == instr_vdotvarp}) 
                    begin
                        states = multstate;
                        //No of clock cycles needed to get the result using bit serial multiplier
                        cycles = (instruction==instr_vmulvarp || instruction == instr_vdotvarp)? ({4'h0,vap}):SEW[7:0]; 
                        first_cmpte = 1;
                    end
                    else begin
                        states = completestate;    
                    end
                end
                else
                    done = 0;
            end
            multstate: begin
                if(SEW==32 && !((instruction == instr_vmulvarp) || (instruction == instr_vdotvarp))) begin
                    if(first_cmpte)begin
                        accumulator = (opB[31])?-opA:0;
                        copB= opB << 1;
                        cycles = cycles -1;
                        first_cmpte = 0;
                        end
                    else begin
                        // $display("Entered else state, accumulator:%b, time:%d", accumulator, $time);
                        accumulator = ((accumulator<<1) + ((copB[31])?opA:0));
                        copB       = copB <<1;
                        cycles = cycles -1;
                    end
                end
                else if(SEW==16 && !((instruction == instr_vmulvarp) || (instruction == instr_vdotvarp))) begin
                    if(first_cmpte)begin
                        accumulator[31:16] = (opB[31])?-opA[31:16]:0;
                        accumulator[15: 0] = (opB[15])?-opA[15: 0]:0;

                        copB[31:16]= opB[31:16] << 1;
                        copB[15: 0]= opB[15: 0] << 1;
                        cycles = cycles -1;
                        first_cmpte = 0;
                        end
                    else begin
                        accumulator[31:16] = ((accumulator[31:16]<<1) + ((copB[31])?opA[31:16]:0));
                        accumulator[15: 0] = ((accumulator[15: 0]<<1) + ((copB[15])?opA[15: 0]:0));
                        copB[31:16]       = copB[31:16] <<1;
                        copB[15: 0]       = copB[15: 0] <<1;
                        cycles = cycles -1;
                    end
                end
                //SEW of 8 is used for vap as well because we are using only 1,2,4,8
                else begin
                    if(first_cmpte)begin
                        if(vap==1) begin
                            accumulator[31:24] = (opB[31])?-opA[31:24]:opA[31:24];
                            accumulator[23:16] = (opB[23])?-opA[23:16]:opA[23:16];
                            accumulator[15:8 ] = (opB[15])?-opA[15:8 ]:opA[15:8 ];
                            accumulator[7 :0 ] = (opB[ 7])?-opA[7 :0 ]:opA[7 :0 ];
                        end
                        else begin
                            // $display("Entered else, opB[7]:%b, -opA[7:0]:%b, time:%d", opB[7:0],-opA[7 :0 ], $time);
                            accumulator[31:24] = (opB[31])?-opA[31:24]:0;
                            accumulator[23:16] = (opB[23])?-opA[23:16]:0;
                            accumulator[15:8 ] = (opB[15])?-opA[15:8 ]:0;
                            accumulator[7 :0 ] = (opB[ 7])?-opA[7:0]:0;
                        end
                        copB[31:24]= opB[31:24] << 1;
                        copB[23:16]= opB[23:16] << 1;
                        copB[15:8 ]= opB[15:8 ] << 1;
                        copB[7 :0 ]= opB[7 :0 ] << 1;
                        cycles = cycles -1;
                        first_cmpte = 0;
                        end
                    else begin
                        // $display("cycles:%d,accumulator:%b, copB:%b, time:%d",cycles, accumulator[7:0], copB[7:0], $time);
                        accumulator[31:24] = ((accumulator[31:24]<<1) + ((copB[31])?opA[31:24]:0));
                        accumulator[23:16] = ((accumulator[23:16]<<1) + ((copB[23])?opA[23:16]:0));
                        accumulator[15: 8] = ((accumulator[15: 8]<<1) + ((copB[15])?opA[15: 8]:0));
                        accumulator[7 : 0] = ((accumulator[7 : 0]<<1) + ((copB[7 ])?opA[7 : 0]:0));
                        
                        copB[31:24]= copB[31:24] << 1;
                        copB[23:16]= copB[23:16] << 1;
                        copB[15:8 ]= copB[15:8 ] << 1;
                        copB[7 :0 ]= copB[7 :0 ] << 1;
                        cycles = cycles -1;
                    end
                end

                if(cycles==0)
                    states=completestate;

            end
            completestate:begin
                if(|{instruction == instr_vadd__vv,instruction==instr_vsub__vv,instruction==instr_vdot__vv})begin
                    if(SEW==32)begin
                        accumulator = ((instruction==instr_vdot__vv)?accumulator : opA) + ((instruction == instr_vadd__vv)?opB:((instruction == instr_vsub__vv)?(-opB):opC));
                    end
                    else if(SEW==16)begin
                        accumulator[31:16] = ((instruction==instr_vdot__vv)?accumulator[31:16] : opA[31:16]) + ((instruction == instr_vadd__vv)?opB[31:16]:((instruction == instr_vsub__vv)?(-opB[31:16]):opC[31:16]));
                        accumulator[15:0]  = ((instruction==instr_vdot__vv)?accumulator[15:0] : opA[15:0])  + ((instruction == instr_vadd__vv)?opB[15:0] :((instruction == instr_vsub__vv)?(-opB[15:0] ):opC[15:0])) ;
                    end
                    else if(SEW==8)begin
                        accumulator[31:24] = ((instruction==instr_vdot__vv)?accumulator[31:24] : opA[31:24]) + ((instruction == instr_vadd__vv)?opB[31:24]:((instruction == instr_vsub__vv)?(-opB[31:24]):opC[31:24]));
                        accumulator[23:16] = ((instruction==instr_vdot__vv)?accumulator[23:16] : opA[23:16]) + ((instruction == instr_vadd__vv)?opB[23:16]:((instruction == instr_vsub__vv)?(-opB[23:16]):opC[23:16]));
                        accumulator[15:8 ] = ((instruction==instr_vdot__vv)?accumulator[15:8] : opA[15:8 ]) + ((instruction == instr_vadd__vv)?opB[15:8 ]:((instruction == instr_vsub__vv)?(-opB[15:8 ]):opC[15:8 ]));
                        accumulator[7:0  ] = ((instruction==instr_vdot__vv)?accumulator[7:0] : opA[7:0  ]) + ((instruction == instr_vadd__vv)?opB[7:0  ]:((instruction == instr_vsub__vv)?(-opB[7:0  ]):opC[7:0  ]));
                    end
                    peout = accumulator;
                end
                else if((|{instruction==instr_vsubvarp,instruction==instr_vaddvarp})) begin
                    accumulator[31:24] = opA[31:24] + ((instruction == instr_vaddvarp)? ((opB[31:24]>>(8-vap)) | ((opB[31])?((8'hFF)<<(vap)):8'h00)) : ((instruction == instr_vsubvarp)?-((opB[31:24]>>(8-vap)) | ((opB[31])?((8'hFF)<<(vap)):8'h00)) :0) );
                    accumulator[23:16] = opA[23:16] + ((instruction == instr_vaddvarp)? ((opB[23:16]>>(8-vap)) | ((opB[23])?((8'hFF)<<(vap)):8'h00)) : ( (instruction == instr_vsubvarp)?-((opB[23:16]>>(8-vap)) | ((opB[23])?((8'hFF)<<(vap)):8'h00)) :0) );
                    accumulator[15:8] = opA[15:8] + ((instruction == instr_vaddvarp)? ((opB[15:8]>>(8-vap)) | ((opB[15])?((8'hFF)<<(vap)):8'h00)) : ( (instruction == instr_vsubvarp)?-((opB[15:8]>>(8-vap)) | ((opB[15])?((8'hFF)<<(vap)):8'h00)) :0) );
                    accumulator[7:0] = opA[7:0] + ((instruction == instr_vaddvarp)? ((opB[7:0]>>(8-vap)) | ((opB[7])?((8'hFF)<<(vap)):8'h00)) : ( (instruction == instr_vsubvarp)?-((opB[7:0]>>(8-vap)) | ((opB[7])?((8'hFF)<<(vap)):8'h00)) :0) );
                    peout = accumulator;
                end
                else if(instruction==instr_vdotvarp)begin 
                    // $display("Inside final condition, instr:%b, time:%d",instruction, $time);
                    peout[7:0] = accumulator[7:0] + opC[7:0];
                    peout[15:8] = accumulator[15:8] + opC[15:8];
                    peout[23:16] = accumulator[23:16] + opC[23:16];
                    peout[31:24] = accumulator[31:24] + opC[31:24];
                end
                else if(|{instruction==instr_vmul__vv,instruction==instr_vmulvarp})begin
                    // $display("Inside final condition, instr:%b, time:%d",instruction, $time);
                    peout = accumulator;
                end
                done = 1;
                states = startstate;
            end

        endcase
    end
end

endmodule
