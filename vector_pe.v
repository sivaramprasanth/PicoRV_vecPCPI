//This is the ALU module which takes the inputs from coprocessor and returns the output and a done signal
module vector_processing_element(
    input clk,
    input reset,
    
    input [7:0] instruction,
	input start,
	output reg done,
	
    input [31:0] opA,
    input [31:0] opB,
    input [31:0] opC,
    output reg [31:0] peout, //The final output

	input [9:0] SEW,
	input [3:0 ] vap
);

//Used for vector arithmetic instructions
localparam p_instr_vadd__vv = 8'h00 ;
localparam p_instr_vmul__vv = 8'h01 ;
localparam p_instr_vdot__vv = 8'h02 ;
localparam p_instr_vmulvarp = 8'h03 ;
localparam p_instr_vaddvarp = 8'h04 ;
localparam p_instr_vdotvarp = 8'h05 ;

always @(posedge clk) begin
    if(reset) begin
        peout  = 0;
        done = 0;
    end
    else begin
       if(instruction == p_instr_vadd__vv) begin
            if(SEW == 32) begin
                peout <= opA + opB;  //SEW is 32
                done <= 1;
            end
            if(SEW == 16) begin
                peout[15:0] <= opA[15:0] + opB[15:0];  //SEW is 16
                peout[31:16] <= opA[31:16] + opB[31:16]; 
                done <= 1;
            end
            if(SEW == 8) begin
                peout[7:0] <= opA[7:0] + opB[7:0];  //SEW is 8
                peout[15:8] <= opA[15:8] + opB[15:8]; 
                peout[23:16] <= opA[23:16] + opB[23:16]; 
                peout[31:24] <= opA[31:24] + opB[31:24]; 
                done <= 1;
            end
        end 
    end
end

endmodule