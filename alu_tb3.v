// //Used for vector arithmetic instructions
// 	localparam p_instr_vadd__vv = 8'h00 ;
// 	localparam p_instr_vmul__vv = 8'h01 ;
// 	localparam p_instr_vdot__vv = 8'h02 ;
// 	localparam p_instr_vaddvarp = 8'h03 ;
// 	localparam p_instr_vmulvarp = 8'h04 ;
// 	localparam p_instr_vdotvarp = 8'h05 ;

module alu_tb;

    reg clk = 0;
    reg resetn = 0;
    reg [7:0] micro_exec_instr;
    reg [9:0] SEW;
    reg [3:0] vap;
    reg [127:0] opA;
    reg [127:0] opB;
    reg [127:0] opC;
    wire [127:0] alu_out;
    wire alu_done; 

    alu_block dut(
        .clk(clk),
        .resetn(resetn),
        .micro_exec_instr(micro_exec_instr),
        .SEW(SEW),
        .vap(vap),
        .opA(opA),
        .opB(opB),
        .opC(opC),
        .alu_out(alu_out),
        .alu_done(alu_done)
    );

    always #5 clk = ~clk;

	initial begin
        // $dumpfile("alu_module.vcd");
        // $dumpvars(0, alu_tb);
        // $monitor("opA:%h, opB:%h, opC:%h, alu_out:%h, resetn:%b, alu_done:%b, time:%d", opA[127:0], opB[127:0], opC[127:0], alu_out[127:0], resetn, alu_done, $time);
        #10 resetn = 1;

        // #10 micro_exec_instr = 8'h03; vap = 4'h8; 
        // opA = 128'h00000000000000000807060504030201;
        // opB = 128'h00000000000000000807060504030201;
        // //Expected output: 128'h0000000000000000f00e0c0a08060402

        // #10 micro_exec_instr = 8'h04; vap = 4'h8; 
        // opA = 128'h0000000000000000000f0e0d0c0b0a09;
        // opB = 128'h0000000000000000000f0e0d0c0b0a09;
        // //Expected output: 128'h000000000000000000e1c4a990796451

        // #10 micro_exec_instr = 8'h05; vap = 4'h8; 
        // opA = 128'h0000000000040302010f0e0d0c0b0a09;
        // opB = 128'h0000000000040302010f0e0d0c0b0a09;
        // opC = 128'h00000000001011121314151617181920;
        // //Expected output: 128'h0000000000201a1614f5d9bfa7917d71

        // #10 micro_exec_instr = 8'h05; vap = 4'h2; 
        // opA = 128'h00000000000101010100000101010001;
        // opB = 128'h00000000000101010100000101010001;
        // opC = 128'h00000000000001010001010100000100;
        // //Expected output: 128'h00000000000102020101010201010101

        // #10 micro_exec_instr = 8'h01; SEW = 10'b0000100000; 
        // opA = 128'h0000312100001e430000aa2300001111;
        // opB = 128'h0000312100001e430000aa2300001111;
        // //Expected output: 128'h096da6410393c589711280c901234321

        #10 micro_exec_instr = 8'h02; SEW = 10'b0000100000; 
        opA = 128'h0000312100001e430000aa2300001111;
        opB = 128'h0000312100001e430000aa2300001111;
        opC = 128'h22221111222211112222111122221111;
        // //Expected output: 128'h2b8fb75225b5d69a933491da23455432

        #500
		$finish;
	end 

    always@(posedge clk) begin
        if(alu_done || !resetn) begin
            $display("opA:%h, opB:%h, opC:%h, alu_out:%h, resetn:%b, alu_done:%b, time:%d", opA[127:0], opB[127:0], opC[127:0], alu_out[127:0], resetn, alu_done, $time);
        end
    end

endmodule