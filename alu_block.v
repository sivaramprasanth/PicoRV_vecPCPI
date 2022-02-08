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
    input [511:0] opA,
    input [511:0] opB,
    input [511:0] opC,
    output reg [511:0] alu_out,
    output alu_done
);

localparam instr_vadd = 8'h00 ;
localparam instr_vmul = 8'h01 ;
localparam instr_vdot = 8'h02 ;
localparam instr_vaddvarp = 8'h03 ;
localparam instr_vmulvarp = 8'h04 ;
localparam instr_vdotvarp = 8'h05 ;

reg [511:0] new_opA, new_opB, new_opC;
wire [511:0] pe_out;
reg alu_enb, temp_reg=0;
wire is_vec_instr, is_vap_instr, is_port3_instr;
assign is_vec_instr = |{(micro_exec_instr == instr_vdot), (micro_exec_instr == instr_vadd), (micro_exec_instr == instr_vmul),
                        (micro_exec_instr == instr_vaddvarp), (micro_exec_instr == instr_vmulvarp), (micro_exec_instr == instr_vdotvarp)};
assign is_vap_instr = |{(micro_exec_instr == instr_vaddvarp), (micro_exec_instr == instr_vmulvarp), (micro_exec_instr == instr_vdotvarp)}; //To heck whether the instruction is variable bit one
assign is_port3_instr = |{(micro_exec_instr == instr_vdot), (micro_exec_instr == instr_vdotvarp)};
wire done1, done2, done3, done4, done5, done6, done7, done8, done9, done10, done11, done12, done13, done14, done15, done16;
assign alu_done = &{done1, done2, done3, done4, done5, done6, done7, done8, done9, done10, done11, done12, done13, done14, done15, done16};


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
                new_opA[511:0]  <= opA[511:0];
                new_opB[511:0]  <= opB[511:0];
                if(is_port3_instr) begin
                    new_opC[511:0]  <= opC[511:0];
                end
                alu_enb <= 1; 
            end
            if(is_vap_instr) begin
                $display("Entered is_vap_instr condition, alu_enb:%b, time:%d",alu_enb, $time);
                if(vap == 10'b0000000001) begin
                    // $display("vecreg_data: %x, cnt:%d, time:%d", opA, cnt, $time);
                    //Converting sew of 1 to 8 bits to operate on them and zero padding on left side (for better multiplication)
                    new_opA[63:0]    <= {{8{opA[56]}}, {8{opA[48]}}, {8{opA[40]}}, {8{opA[32]}}, {8{opA[24]}}, {8{opA[16]}}, {8{opA[8]}}, {8{opA[0]}}};
                    new_opA[127:64]  <= {{8{opA[120]}}, {8{opA[112]}}, {8{opA[104]}}, {8{opA[96]}}, {8{opA[88]}}, {8{opA[80]}}, {8{opA[72]}}, {8{opA[64]}}};
                    new_opA[191:128] <= {{8{opA[184]}}, {8{opA[176]}}, {8{opA[168]}}, {8{opA[160]}}, {8{opA[152]}}, {8{opA[144]}}, {8{opA[136]}}, {8{opA[128]}}};
                    new_opA[255:192] <= {{8{opA[248]}}, {8{opA[240]}}, {8{opA[232]}}, {8{opA[224]}}, {8{opA[216]}}, {8{opA[208]}}, {8{opA[200]}}, {8{opA[192]}}};
                    new_opA[319:256] <= {{8{opA[312]}}, {8{opA[304]}}, {8{opA[296]}}, {8{opA[288]}}, {8{opA[280]}}, {8{opA[272]}}, {8{opA[264]}}, {8{opA[256]}}};
                    new_opA[383:320] <= {{8{opA[376]}}, {8{opA[368]}}, {8{opA[360]}}, {8{opA[352]}}, {8{opA[344]}}, {8{opA[336]}}, {8{opA[328]}}, {8{opA[320]}}};
                    new_opA[447:384] <= {{8{opA[440]}}, {8{opA[432]}}, {8{opA[424]}}, {8{opA[416]}}, {8{opA[408]}}, {8{opA[400]}}, {8{opA[392]}}, {8{opA[384]}}};
                    new_opA[511:448] <= {{8{opA[504]}}, {8{opA[496]}}, {8{opA[488]}}, {8{opA[480]}}, {8{opA[472]}}, {8{opA[464]}}, {8{opA[456]}}, {8{opA[448]}}};
                    new_opB[63:0]    <= {{8{opB[56]}}, {8{opB[48]}}, {8{opB[40]}}, {8{opB[32]}}, {8{opB[24]}}, {8{opB[16]}}, {8{opB[8]}}, {8{opB[0]}}};
                    new_opB[127:64]  <= {{8{opB[120]}}, {8{opB[112]}}, {8{opB[104]}}, {8{opB[96]}}, {8{opB[88]}}, {8{opB[80]}}, {8{opB[72]}}, {8{opB[64]}}};
                    new_opB[191:128] <= {{8{opB[184]}}, {8{opB[176]}}, {8{opB[168]}}, {8{opB[160]}}, {8{opB[152]}}, {8{opB[144]}}, {8{opB[136]}}, {8{opB[128]}}};
                    new_opB[255:192] <= {{8{opB[248]}}, {8{opB[240]}}, {8{opB[232]}}, {8{opB[224]}}, {8{opB[216]}}, {8{opB[208]}}, {8{opB[200]}}, {8{opB[192]}}};
                    new_opB[319:256] <= {{8{opB[312]}}, {8{opB[304]}}, {8{opB[296]}}, {8{opB[288]}}, {8{opB[280]}}, {8{opB[272]}}, {8{opB[264]}}, {8{opB[256]}}};
                    new_opB[383:320] <= {{8{opB[376]}}, {8{opB[368]}}, {8{opB[360]}}, {8{opB[352]}}, {8{opB[344]}}, {8{opB[336]}}, {8{opB[328]}}, {8{opB[320]}}};
                    new_opB[447:384] <= {{8{opB[440]}}, {8{opB[432]}}, {8{opB[424]}}, {8{opB[416]}}, {8{opB[408]}}, {8{opB[400]}}, {8{opB[392]}}, {8{opB[384]}}};
                    new_opB[511:448] <= {{8{opB[504]}}, {8{opB[496]}}, {8{opB[488]}}, {8{opB[480]}}, {8{opB[472]}}, {8{opB[464]}}, {8{opB[456]}}, {8{opB[448]}}};
                    if(is_port3_instr) begin
                        new_opC[63:0]     <= {{8{opC[56]}}, {8{opC[48]}}, {8{opC[40]}}, {8{opC[32]}}, {8{opC[24]}}, {8{opC[16]}}, {8{opC[8]}}, {8{opC[0]}}};
                        new_opC[127:64]  <= {{8{opC[120]}}, {8{opC[112]}}, {8{opC[104]}}, {8{opC[96]}}, {8{opC[88]}}, {8{opC[80]}}, {8{opC[72]}}, {8{opC[64]}}};
                        new_opC[191:128] <= {{8{opC[184]}}, {8{opC[176]}}, {8{opC[168]}}, {8{opC[160]}}, {8{opC[152]}}, {8{opC[144]}}, {8{opC[136]}}, {8{opC[128]}}};
                        new_opC[255:192] <= {{8{opC[248]}}, {8{opC[240]}}, {8{opC[232]}}, {8{opC[224]}}, {8{opC[216]}}, {8{opC[208]}}, {8{opC[200]}}, {8{opC[192]}}};
                        new_opC[319:256] <= {{8{opC[312]}}, {8{opC[304]}}, {8{opC[296]}}, {8{opC[288]}}, {8{opC[280]}}, {8{opC[272]}}, {8{opC[264]}}, {8{opC[256]}}};
                        new_opC[383:320] <= {{8{opC[376]}}, {8{opC[368]}}, {8{opC[360]}}, {8{opC[352]}}, {8{opC[344]}}, {8{opC[336]}}, {8{opC[328]}}, {8{opC[320]}}};
                        new_opC[447:384] <= {{8{opC[440]}}, {8{opC[432]}}, {8{opC[424]}}, {8{opC[416]}}, {8{opC[408]}}, {8{opC[400]}}, {8{opC[392]}}, {8{opC[384]}}};
                        new_opC[511:448] <= {{8{opC[504]}}, {8{opC[496]}}, {8{opC[488]}}, {8{opC[480]}}, {8{opC[472]}}, {8{opC[464]}}, {8{opC[456]}}, {8{opC[448]}}};
                    end
                    alu_enb <= 1; 
                end
                else if(vap == 10'b0000000010) begin
                    new_opA[31:0]    <= {{6{opA[25]}},opA[25:24],{6{opA[17]}},opA[17:16],{6{opA[9]}},opA[9:8],{6{opA[1]}},opA[1:0]};
                    new_opA[63:32]   <= {{6{opA[57]}},opA[57:56],{6{opA[49]}},opA[49:48],{6{opA[41]}},opA[41:40],{6{opA[33]}},opA[33:32]};
                    new_opA[95:64]   <= {{6{opA[89]}},opA[89:88],{6{opA[81]}},opA[81:80],{6{opA[73]}},opA[73:72],{6{opA[65]}},opA[65:64]};
                    new_opA[127:96]  <= {{6{opA[121]}},opA[121:120],{6{opA[113]}},opA[113:112],{6{opA[105]}},opA[105:104],{6{opA[97]}},opA[97:96]};
                    new_opA[159:128] <= {{6{opA[153]}},opA[153:152],{6{opA[144]}},opA[145:144],{6{opA[137]}},opA[137:136],{6{opA[129]}},opA[129:128]};
                    new_opA[191:160] <= {{6{opA[185]}},opA[185:184],{6{opA[177]}},opA[177:176],{6{opA[169]}},opA[169:168],{6{opA[161]}},opA[161:160]};
                    new_opA[223:192] <= {{6{opA[217]}},opA[217:216],{6{opA[209]}},opA[209:208],{6{opA[201]}},opA[201:200],{6{opA[193]}},opA[193:192]};
                    new_opA[255:224] <= {{6{opA[249]}},opA[249:248],{6{opA[241]}},opA[241:240],{6{opA[233]}},opA[233:232],{6{opA[225]}},opA[225:224]};
                    new_opA[287:256] <= {{6{opA[281]}},opA[281:280],{6{opA[273]}},opA[273:272],{6{opA[265]}},opA[265:264],{6{opA[257]}},opA[257:256]};
                    new_opA[319:288] <= {{6{opA[313]}},opA[313:312],{6{opA[305]}},opA[305:304],{6{opA[297]}},opA[297:296],{6{opA[289]}},opA[289:288]};
                    new_opA[351:320] <= {{6{opA[345]}},opA[345:344],{6{opA[337]}},opA[337:336],{6{opA[329]}},opA[329:328],{6{opA[321]}},opA[321:320]};
                    new_opA[383:352] <= {{6{opA[377]}},opA[377:376],{6{opA[369]}},opA[369:368],{6{opA[361]}},opA[361:360],{6{opA[353]}},opA[353:352]};
                    new_opA[415:384] <= {{6{opA[409]}},opA[409:408],{6{opA[401]}},opA[401:400],{6{opA[393]}},opA[393:392],{6{opA[385]}},opA[385:384]};
                    new_opA[447:416] <= {{6{opA[441]}},opA[441:440],{6{opA[433]}},opA[433:432],{6{opA[425]}},opA[425:424],{6{opA[417]}},opA[417:416]};
                    new_opA[479:448] <= {{6{opA[473]}},opA[473:472],{6{opA[465]}},opA[465:464],{6{opA[457]}},opA[457:456],{6{opA[449]}},opA[449:448]};
                    new_opA[511:480] <= {{6{opA[505]}},opA[505:504],{6{opA[497]}},opA[497:496],{6{opA[489]}},opA[489:488],{6{opA[481]}},opA[481:480]};
                    new_opB[31:0]    <= {opB[25:24],6'b0,opB[17:16],6'b0,opB[9:8],6'b0,opB[1:0],6'b0};
                    new_opB[63:32]   <= {opB[57:56],6'b0,opB[49:48],6'b0,opB[41:40],6'b0,opB[33:32],6'b0};
                    new_opB[95:64]   <= {opB[89:88],6'b0,opB[81:80],6'b0,opB[73:72],6'b0,opB[65:64],6'b0};
                    new_opB[127:96]  <= {opB[121:120],6'b0,opB[113:112],6'b0,opB[105:104],6'b0,opB[97:96],6'b0};
                    new_opB[159:128] <= {opB[153:152],6'b0,opB[145:144],6'b0,opB[137:136],6'b0,opB[129:128],6'b0};
                    new_opB[191:160] <= {opB[185:184],6'b0,opB[177:176],6'b0,opB[169:168],6'b0,opB[161:160],6'b0};
                    new_opB[223:192] <= {opB[217:216],6'b0,opB[209:208],6'b0,opB[201:200],6'b0,opB[193:192],6'b0};
                    new_opB[255:224] <= {opB[249:248],6'b0,opB[241:240],6'b0,opB[233:232],6'b0,opB[225:224],6'b0};
                    new_opB[287:256] <= {opB[281:280],6'b0,opB[273:272],6'b0,opB[265:264],6'b0,opB[257:256],6'b0};
                    new_opB[319:288] <= {opB[313:312],6'b0,opB[305:304],6'b0,opB[297:296],6'b0,opB[289:288],6'b0};
                    new_opB[351:320] <= {opB[345:344],6'b0,opB[337:336],6'b0,opB[329:328],6'b0,opB[321:320],6'b0};
                    new_opB[383:352] <= {opB[377:376],6'b0,opB[369:368],6'b0,opB[361:360],6'b0,opB[353:352],6'b0};
                    new_opB[415:384] <= {opB[409:408],6'b0,opB[401:400],6'b0,opB[393:392],6'b0,opB[385:384],6'b0};
                    new_opB[447:416] <= {opB[441:440],6'b0,opB[433:432],6'b0,opB[425:424],6'b0,opB[417:416],6'b0};
                    new_opB[479:448] <= {opB[473:472],6'b0,opB[465:464],6'b0,opB[457:456],6'b0,opB[449:448],6'b0};
                    new_opB[511:480] <= {opB[505:504],6'b0,opB[497:496],6'b0,opB[489:488],6'b0,opB[481:480],6'b0};
                    if(is_port3_instr) begin
                        new_opC[31:0]    <= {{6{opC[25]}},opC[25:24],{6{opC[17]}},opC[17:16],{6{opC[9]}},opC[9:8],{6{opC[1]}},opC[1:0]};
                        new_opC[63:32]   <= {{6{opC[57]}},opC[57:56],{6{opC[49]}},opC[49:48],{6{opC[41]}},opC[41:40],{6{opC[33]}},opC[33:32]};
                        new_opC[95:64]   <= {{6{opC[89]}},opC[89:88],{6{opC[81]}},opC[81:80],{6{opC[73]}},opC[73:72],{6{opC[65]}},opC[65:64]};
                        new_opC[127:96]  <= {{6{opC[121]}},opC[121:120],{6{opC[113]}},opC[113:112],{6{opC[105]}},opC[105:104],{6{opC[97]}},opC[97:96]};
                        new_opC[159:128] <= {{6{opC[153]}},opC[153:152],{6{opC[144]}},opC[145:144],{6{opC[137]}},opC[137:136],{6{opC[129]}},opC[129:128]};
                        new_opC[191:160] <= {{6{opC[185]}},opC[185:184],{6{opC[177]}},opC[177:176],{6{opC[169]}},opC[169:168],{6{opC[161]}},opC[161:160]};
                        new_opC[223:192] <= {{6{opC[217]}},opC[217:216],{6{opC[209]}},opC[209:208],{6{opC[201]}},opC[201:200],{6{opC[193]}},opC[193:192]};
                        new_opC[255:224] <= {{6{opC[249]}},opC[249:248],{6{opC[241]}},opC[241:240],{6{opC[233]}},opC[233:232],{6{opC[225]}},opC[225:224]};
                        new_opC[287:256] <= {{6{opC[281]}},opC[281:280],{6{opC[273]}},opC[273:272],{6{opC[265]}},opC[265:264],{6{opC[257]}},opC[257:256]};
                        new_opC[319:288] <= {{6{opC[313]}},opC[313:312],{6{opC[305]}},opC[305:304],{6{opC[297]}},opC[297:296],{6{opC[289]}},opC[289:288]};
                        new_opC[351:320] <= {{6{opC[345]}},opC[345:344],{6{opC[337]}},opC[337:336],{6{opC[329]}},opC[329:328],{6{opC[321]}},opC[321:320]};
                        new_opC[383:352] <= {{6{opC[377]}},opC[377:376],{6{opC[369]}},opC[369:368],{6{opC[361]}},opC[361:360],{6{opC[353]}},opC[353:352]};
                        new_opC[415:384] <= {{6{opC[409]}},opC[409:408],{6{opC[401]}},opC[401:400],{6{opC[393]}},opC[393:392],{6{opC[385]}},opC[385:384]};
                        new_opC[447:416] <= {{6{opC[441]}},opC[441:440],{6{opC[433]}},opC[433:432],{6{opC[425]}},opC[425:424],{6{opC[417]}},opC[417:416]};
                        new_opC[479:448] <= {{6{opC[473]}},opC[473:472],{6{opC[465]}},opC[465:464],{6{opC[457]}},opC[457:456],{6{opC[449]}},opC[449:448]};
                        new_opC[511:480] <= {{6{opC[505]}},opC[505:504],{6{opC[497]}},opC[497:496],{6{opC[489]}},opC[489:488],{6{opC[481]}},opC[481:480]};
                    end
                    alu_enb <= 1; 
                end
                else if(vap == 10'b0000000100) begin
                    new_opA[31:0]     <= {{4{opA[27]}},opA[27:24],{4{opA[19]}},opA[19:16],{4{opA[11]}},opA[11:8],{4{opA[3]}},opA[3:0]};
                    new_opA[63:32]    <= {{4{opA[59]}},opA[59:56],{4{opA[51]}},opA[51:48],{4{opA[43]}},opA[43:40],{4{opA[35]}},opA[35:32]};
                    new_opA[95:64]    <= {{4{opA[91]}},opA[91:88],{4{opA[83]}},opA[83:80],{4{opA[75]}},opA[75:72],{4{opA[67]}},opA[67:64]};
                    new_opA[127:96]   <= {{4{opA[123]}},opA[123:120],{4{opA[115]}},opA[115:112],{4{opA[107]}},opA[107:104],{4{opA[99]}},opA[99:96]};
                    new_opA[159:128]  <= {{4{opA[155]}},opA[155:152],{4{opA[145]}},opA[147:144],{4{opA[139]}},opA[139:136],{4{opA[131]}},opA[131:128]};
                    new_opA[191:160]  <= {{4{opA[187]}},opA[187:184],{4{opA[179]}},opA[179:176],{4{opA[171]}},opA[171:168],{4{opA[163]}},opA[163:160]};
                    new_opA[223:192]  <= {{4{opA[219]}},opA[219:216],{4{opA[211]}},opA[211:208],{4{opA[203]}},opA[203:200],{4{opA[195]}},opA[195:192]};
                    new_opA[255:224]  <= {{4{opA[251]}},opA[251:248],{4{opA[243]}},opA[243:240],{4{opA[235]}},opA[235:232],{4{opA[227]}},opA[227:224]};
                    new_opA[287:256]  <= {{4{opA[283]}},opA[283:280],{4{opA[275]}},opA[275:272],{4{opA[267]}},opA[267:264],{4{opA[259]}},opA[259:256]};
                    new_opA[319:288]  <= {{4{opA[315]}},opA[315:312],{4{opA[307]}},opA[307:304],{4{opA[299]}},opA[299:296],{4{opA[291]}},opA[291:288]};
                    new_opA[351:320]  <= {{4{opA[347]}},opA[347:344],{4{opA[339]}},opA[339:336],{4{opA[331]}},opA[331:328],{4{opA[323]}},opA[323:320]};
                    new_opA[383:352]  <= {{4{opA[379]}},opA[379:376],{4{opA[371]}},opA[371:368],{4{opA[363]}},opA[363:360],{4{opA[355]}},opA[355:352]};
                    new_opA[415:384]  <= {{4{opA[411]}},opA[411:408],{4{opA[403]}},opA[403:400],{4{opA[395]}},opA[395:392],{4{opA[387]}},opA[387:384]};
                    new_opA[447:416]  <= {{4{opA[443]}},opA[443:440],{4{opA[435]}},opA[435:432],{4{opA[427]}},opA[427:424],{4{opA[419]}},opA[419:416]};
                    new_opA[479:448]  <= {{4{opA[475]}},opA[475:472],{4{opA[467]}},opA[467:464],{4{opA[459]}},opA[459:456],{4{opA[451]}},opA[451:448]};
                    new_opA[511:480]  <= {{4{opA[507]}},opA[507:504],{4{opA[499]}},opA[499:496],{4{opA[491]}},opA[491:488],{4{opA[483]}},opA[483:480]};
                    new_opB[31:0]     <= {opB[27:24],4'b0,opB[19:16],4'b0,opB[11:8],4'b0,opB[3:0],4'b0};
                    new_opB[63:32]    <= {opB[59:56],4'b0,opB[51:48],4'b0,opB[43:40],4'b0,opB[35:32],4'b0};
                    new_opB[95:64]    <= {opB[91:88],4'b0,opB[83:80],4'b0,opB[75:72],4'b0,opB[67:64],4'b0};
                    new_opB[127:96]   <= {opB[123:120],4'b0,opB[115:112],4'b0,opB[107:104],4'b0,opB[99:96],4'b0};
                    new_opB[159:128]  <= {opB[155:152],4'b0,opB[147:144],4'b0,opB[139:136],4'b0,opB[131:128],4'b0};
                    new_opB[191:160]  <= {opB[187:184],4'b0,opB[179:176],4'b0,opB[171:168],4'b0,opB[163:160],4'b0};
                    new_opB[223:192]  <= {opB[219:216],4'b0,opB[211:208],4'b0,opB[203:200],4'b0,opB[195:192],4'b0};
                    new_opB[255:224]  <= {opB[251:248],4'b0,opB[243:240],4'b0,opB[235:232],4'b0,opB[227:224],4'b0};
                    new_opB[287:256]  <= {opB[283:280],4'b0,opB[275:272],4'b0,opB[267:264],4'b0,opB[259:256],4'b0};
                    new_opB[319:288]  <= {opB[315:312],4'b0,opB[307:304],4'b0,opB[299:296],4'b0,opB[291:288],4'b0};
                    new_opB[351:320]  <= {opB[347:344],4'b0,opB[339:336],4'b0,opB[331:328],4'b0,opB[323:320],4'b0};
                    new_opB[383:352]  <= {opB[379:376],4'b0,opB[371:368],4'b0,opB[363:360],4'b0,opB[355:352],4'b0};
                    new_opB[415:384]  <= {opB[411:408],4'b0,opB[403:400],4'b0,opB[395:392],4'b0,opB[387:384],4'b0};
                    new_opB[447:416]  <= {opB[443:440],4'b0,opB[435:432],4'b0,opB[427:424],4'b0,opB[419:416],4'b0};
                    new_opB[479:448]  <= {opB[475:472],4'b0,opB[467:464],4'b0,opB[459:456],4'b0,opB[451:448],4'b0};
                    new_opB[511:480]  <= {opB[507:504],4'b0,opB[499:496],4'b0,opB[491:488],4'b0,opB[483:480],4'b0};
                    if(is_port3_instr) begin
                        new_opC[31:0]    <= {{4{opC[27]}},opC[27:24],{4{opC[19]}},opC[19:16],{4{opC[11]}},opC[11:8],{4{opC[3]}},opC[3:0]};
                        new_opC[63:32]   <= {{4{opC[59]}},opC[59:56],{4{opC[51]}},opC[51:48],{4{opC[43]}},opC[43:40],{4{opC[35]}},opC[35:32]};
                        new_opC[95:64]   <= {{4{opC[91]}},opC[91:88],{4{opC[83]}},opC[83:80],{4{opC[75]}},opC[75:72],{4{opC[67]}},opC[67:64]};
                        new_opC[127:96]  <= {{4{opC[123]}},opC[123:120],{4{opC[115]}},opC[115:112],{4{opC[107]}},opC[107:104],{4{opC[99]}},opC[99:96]};
                        new_opC[159:128] <= {{4{opC[155]}},opC[155:152],{4{opC[145]}},opC[147:144],{4{opC[139]}},opC[139:136],{4{opC[131]}},opC[131:128]};
                        new_opC[191:160] <= {{4{opC[187]}},opC[187:184],{4{opC[179]}},opC[179:176],{4{opC[171]}},opC[171:168],{4{opC[163]}},opC[163:160]};
                        new_opC[223:192] <= {{4{opC[219]}},opC[219:216],{4{opC[211]}},opC[211:208],{4{opC[203]}},opC[203:200],{4{opC[195]}},opC[195:192]};
                        new_opC[255:224] <= {{4{opC[251]}},opC[251:248],{4{opC[243]}},opC[243:240],{4{opC[235]}},opC[235:232],{4{opC[227]}},opC[227:224]};
                        new_opC[287:256] <= {{4{opC[283]}},opC[283:280],{4{opC[275]}},opC[275:272],{4{opC[267]}},opC[267:264],{4{opC[259]}},opC[259:256]};
                        new_opC[319:288] <= {{4{opC[315]}},opC[315:312],{4{opC[307]}},opC[307:304],{4{opC[299]}},opC[299:296],{4{opC[291]}},opC[291:288]};
                        new_opC[351:320] <= {{4{opC[347]}},opC[347:344],{4{opC[339]}},opC[339:336],{4{opC[331]}},opC[331:328],{4{opC[323]}},opC[323:320]};
                        new_opC[383:352] <= {{4{opC[379]}},opC[379:376],{4{opC[371]}},opC[371:368],{4{opC[363]}},opC[363:360],{4{opC[355]}},opC[355:352]};
                        new_opC[415:384] <= {{4{opC[411]}},opC[411:408],{4{opC[403]}},opC[403:400],{4{opC[395]}},opC[395:392],{4{opC[387]}},opC[387:384]};
                        new_opC[447:416] <= {{4{opC[443]}},opC[443:440],{4{opC[435]}},opC[435:432],{4{opC[427]}},opC[427:424],{4{opC[419]}},opC[419:416]};
                        new_opC[479:448] <= {{4{opC[475]}},opC[475:472],{4{opC[467]}},opC[467:464],{4{opC[459]}},opC[459:456],{4{opC[451]}},opC[451:448]};
                        new_opC[511:480] <= {{4{opC[507]}},opC[507:504],{4{opC[499]}},opC[499:496],{4{opC[491]}},opC[491:488],{4{opC[483]}},opC[483:480]};
                    end
                    alu_enb <= 1; 
                end
                else if(vap == 10'b0000001000) begin
                    // $display("vecreg_data: %x, cnt:%d, time:%d", opA, cnt, $time);
                    new_opA[511:0]  <= opA[511:0];
                    new_opB[511:0]  <= opB[511:0];
                    if(is_port3_instr) begin
                        new_opC[511:0]  <= opC[511:0];
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
            $display("Done is ready, pe_out:%h, time:%d", pe_out[127:0], $time);
            alu_out = pe_out;
            temp_reg = 1;
        end
    end 

    vector_processing_element pe1(.clk(clk),.reset(resetn),.instruction(micro_exec_instr),.start(alu_enb),.done(done1),.opA(new_opA[511:480]),.opB(new_opB[511:480]),.opC(new_opC[511:480]),.peout(pe_out[511:480]),.SEW(SEW),.vap(vap));
    vector_processing_element pe2(.clk(clk),.reset(resetn),.instruction(micro_exec_instr),.start(alu_enb),.done(done2),.opA(new_opA[479:448]),.opB(new_opB[479:448]),.opC(new_opC[479:448]),.peout(pe_out[479:448]),.SEW(SEW),.vap(vap));
    vector_processing_element pe3(.clk(clk),.reset(resetn),.instruction(micro_exec_instr),.start(alu_enb),.done(done3),.opA(new_opA[447:416]),.opB(new_opB[447:416]),.opC(new_opC[447:416]),.peout(pe_out[447:416]),.SEW(SEW),.vap(vap));
    vector_processing_element pe4(.clk(clk),.reset(resetn),.instruction(micro_exec_instr),.start(alu_enb),.done(done4),.opA(new_opA[415:384]),.opB(new_opB[415:384]),.opC(new_opC[415:384]),.peout(pe_out[415:384]),.SEW(SEW),.vap(vap));
    vector_processing_element pe5(.clk(clk),.reset(resetn),.instruction(micro_exec_instr),.start(alu_enb),.done(done5),.opA(new_opA[383:352]),.opB(new_opB[383:352]),.opC(new_opC[383:352]),.peout(pe_out[383:352]),.SEW(SEW),.vap(vap));
    vector_processing_element pe6(.clk(clk),.reset(resetn),.instruction(micro_exec_instr),.start(alu_enb),.done(done6),.opA(new_opA[351:320]),.opB(new_opB[351:320]),.opC(new_opC[351:320]),.peout(pe_out[351:320]),.SEW(SEW),.vap(vap));
    vector_processing_element pe7(.clk(clk),.reset(resetn),.instruction(micro_exec_instr),.start(alu_enb),.done(done7),.opA(new_opA[319:288]),.opB(new_opB[319:288]),.opC(new_opC[319:288]),.peout(pe_out[319:288]),.SEW(SEW),.vap(vap));
    vector_processing_element pe8(.clk(clk),.reset(resetn),.instruction(micro_exec_instr),.start(alu_enb),.done(done8),.opA(new_opA[287:256]),.opB(new_opB[287:256]),.opC(new_opC[287:256]),.peout(pe_out[287:256]),.SEW(SEW),.vap(vap));
    vector_processing_element pe9(.clk(clk),.reset(resetn),.instruction(micro_exec_instr),.start(alu_enb),.done(done9),.opA(new_opA[255:224]),.opB(new_opB[255:224]),.opC(new_opC[255:224]),.peout(pe_out[255:224]),.SEW(SEW),.vap(vap));
    vector_processing_element pe10(.clk(clk),.reset(resetn),.instruction(micro_exec_instr),.start(alu_enb),.done(done10),.opA(new_opA[223:192]),.opB(new_opB[223:192]),.opC(new_opC[223:192]),.peout(pe_out[223:192]),.SEW(SEW),.vap(vap));
    vector_processing_element pe11(.clk(clk),.reset(resetn),.instruction(micro_exec_instr),.start(alu_enb),.done(done11),.opA(new_opA[191:160]),.opB(new_opB[191:160]),.opC(new_opC[191:160]),.peout(pe_out[191:160]),.SEW(SEW),.vap(vap));
    vector_processing_element pe12(.clk(clk),.reset(resetn),.instruction(micro_exec_instr),.start(alu_enb),.done(done12),.opA(new_opA[159:128]),.opB(new_opB[159:128]),.opC(new_opC[159:128]),.peout(pe_out[159:128]),.SEW(SEW),.vap(vap));
    vector_processing_element pe13(.clk(clk),.reset(resetn),.instruction(micro_exec_instr),.start(alu_enb),.done(done13),.opA(new_opA[127:96]),.opB(new_opB[127:96]),.opC(new_opC[127:96]),.peout(pe_out[127:96]),.SEW(SEW),.vap(vap));
    vector_processing_element pe14(.clk(clk),.reset(resetn),.instruction(micro_exec_instr),.start(alu_enb),.done(done14),.opA(new_opA[95:64]),.opB(new_opB[95:64]),.opC(new_opC[95:64]),.peout(pe_out[95:64]),.SEW(SEW),.vap(vap));
    vector_processing_element pe15(.clk(clk),.reset(resetn),.instruction(micro_exec_instr),.start(alu_enb),.done(done15),.opA(new_opA[63:32]),.opB(new_opB[63:32]),.opC(new_opC[63:32]),.peout(pe_out[63:32]),.SEW(SEW),.vap(vap));
    vector_processing_element pe16(.clk(clk),.reset(resetn),.instruction(micro_exec_instr),.start(alu_enb),.done(done16),.opA(new_opA[31:0]),.opB(new_opB[31:0]),.opC(new_opC[31:0]),.peout(pe_out[31:0]),.SEW(SEW),.vap(vap));

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
