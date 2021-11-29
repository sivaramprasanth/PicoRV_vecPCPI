
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
reg temp;

wire is_vap_instr = |{instruction==instr_vaddvarp, instruction==instr_vmulvarp, instruction==instr_vdotvarp, instruction==instr_vsubvarp};
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

always @(posedge clk) begin
    // $display("Entered start state, reset:%d, time:%d",reset, $time);
    if(!reset) begin
        states = 0;
        peout  = 0;
        accumulator = 0;
        first_cmpte = 0;
        cycles = 0;
        copB = 0;
        done = 0;
    end
    else begin
        case(states)
            startstate:begin
                if(start)begin
                    done = 0;
                    accumulator = 0;
                    if(|{instruction == instr_vmul__vv,instruction == instr_vmulvarp,instruction == instr_vdot__vv}) begin
                        states = multstate;
                        //No of clock cycles needed to get the result using bit serial multiplier
                        cycles = (instruction==instr_vmulvarp )? ({4'h0,vap}):SEW[7:0]; 

                        first_cmpte = 1;
                        // to retain local copy and not interfere with other changer is depacking module
                    end
                    else begin
                        // $display("Entered start state, time:%d", $time);
                        states = completestate;    
                    end
                end
                else
                    done = 0;
            end
            multstate: begin
                if(SEW==32 && !(instruction == instr_vmulvarp)) begin
                    if(first_cmpte)begin
                        accumulator = (opB[31])?-opA:0;
                        copB= opB << 1;
                        cycles = cycles -1;
                        first_cmpte = 0;
                        end
                    else begin
                        accumulator = ((accumulator<<1) + ((copB[31])?opA:0));
                        copB       = copB <<1;
                        cycles = cycles -1;
                    end
                end
                else if(SEW==16 && !(instruction == instr_vmulvarp)) begin
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
                else if((|{instruction==instr_vsubvarp,instruction==instr_vaddvarp}) && SEW == 16) begin
                    accumulator[31:16] = opA[31:16] + ((instruction == instr_vaddvarp)? ((opB[31:16]>>(15-vap)) | ((opB[31])?((16'hFFFF)<<(vap+1)):16'h0000)) : ( (instruction == instr_vsubvarp)?-((opB[31:16]>>(15-vap)) | ((opB[31])?((16'hFFFF)<<(vap+1)):16'h0000)) :0) );
                    accumulator[15:0] = opA[15:0] + ((instruction == instr_vaddvarp)? ((opB[15:0]>>(15-vap)) | ((opB[15])?((16'hFFFF)<<(vap+1)):16'h0000)) : ( (instruction == instr_vsubvarp)?-((opB[15:0]>>(15-vap)) | ((opB[15])?((16'hFFFF)<<(vap+1)):16'h0000)) :0) );
                    peout = accumulator;
                end
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
                else if(instruction==instr_vmul__vv) begin
                    peout = accumulator;
                end
                temp = 0;
                done =1;
                states = startstate;
            end

        endcase
    end
end

endmodule