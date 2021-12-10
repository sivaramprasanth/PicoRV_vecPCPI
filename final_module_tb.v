
`timescale 1 ns / 1 ps

module final_module_tb;
	reg clk = 1;
	reg resetn = 0;

    final_module dut(
        .clk(clk),
        .resetn(resetn)
    );

 	always #5 clk = ~clk;

	initial begin
		if ($test$plusargs("vcd")) begin
			$dumpfile("final_module.vcd");
			$dumpvars(0, final_module_tb);
		end
		repeat (1) @(posedge clk);
		resetn <= 1;
		repeat (500) @(posedge clk);
		$finish;
	end   

endmodule