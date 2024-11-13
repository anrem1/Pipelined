module RISCV_pipeline ( input rst, input clk, input [1:0] ledSel, input [3:0] ssdSel, output reg [15:0] led, output reg [12:0] ssd  );
// wires declarations
wire [31:0] instr;
wire [31:0] pc_out;
wire [31:0] imm_out;
wire branch, mr, mwrite, alusrc, regwr, jal, jalr; 
wire [1:0] mtoreg;      // for 4x1 mux
wire [1:0] aluop; 
wire [31:0] rdata1  , rdata2;

wire [31:0] B;   // inputs to ALU
wire [3:0] alusel;
wire [31:0] alures; 
wire zero;  
wire [31:0] data_mem_out;   // data memory output
wire [31:0] data_mux_out; 
wire [31:0] shift_out; 
wire [31:0] temp_pc; 
wire [31:0] temp_pc2;
wire overflow;          // for adder after shift
wire pcsrc;              // to jump after branch or not
wire [31:0] result_pc;
wire load = 1; // for pc reg
wire cf, zf, vf, sf;
wire [4:0] shamt;
wire branch_taken;

wire [31:0] IF_ID_PC, IF_ID_Inst;

wire [31:0] ID_EX_PC, ID_EX_RegR1, ID_EX_RegR2, ID_EX_Imm;
// 3 control signals, first 1 (WB) is 2 bits (regwr and memtoreg), 2nd is 3 bits (Branch, Mread, Mwrite), ex is alusrc (1 bit) and aluop (2 bits) [8 bits total]
wire [10:0] ID_EX_Ctrl;
wire [3:0] ID_EX_Func;
// addresses of regs for forwarding
wire [4:0] ID_EX_Rs1, ID_EX_Rs2, ID_EX_Rd;


// Rs1 and Rs2 are needed later for the forwarding unit
wire [31:0] EX_MEM_BranchAddOut, EX_MEM_ALU_out, EX_MEM_RegR2;
wire [7:0] EX_MEM_Ctrl;
wire [4:0] EX_MEM_Rd;
wire EX_MEM_Zero;
wire [31:0] EX_MEM_PC;
wire [31:0] EX_MEM_temp_pc;

wire [31:0] MEM_WB_Mem_out, MEM_WB_ALU_out;
wire [4:0] MEM_WB_Ctrl;
wire [4:0] MEM_WB_Rd;
wire [31:0] MEM_WB_PC;

// forward 
wire [1:0] forwardA, forwardB;
wire [31:0] fmux_out, smux_out;

wire [31:0] auipc;
wire [31:0] up_imm;     // result of mux that chooses between lui and auipc
wire [31:0] EX_MEM_Jal_tar, EX_MEM_Jalr_tar;        // holds values of jal and jalr 

wire stall;

//reg halt;
//nreg #32 PC((halt ? pc_out : result_pc), pc_out, load, rst, clk);

nreg #32 PC(result_pc, pc_out, load, rst, clk);

//InstMem inst(.addr(pc_out[7:2]), .data_out(instr));

    // IF/ID REG 
nreg #(64) IF_ID (
    .D({pc_out, instr[31:0]}),     
    .Q({IF_ID_PC, IF_ID_Inst}), 
    .load(load), 
    .rst(rst), 
    .clk(~clk));        // takes not clock   
    
    
       
// WHERE IS JAL AND JALR COMING FROM 
// changed location of mtoreg 
control ctrl( IF_ID_Inst[6:2], branch, mr, mwrite, alusrc, regwr, aluop, mtoreg, jal, jalr);
// in, output reg branch, mr, mwrite, alusrc, regwr, output reg [1:0] aluop, output reg [1:0] mtoreg, output reg jal, jalr
// put in reg file 
 regfile regs( .wsig(MEM_WB_Ctrl[1]) ,  .clk(clk),  .rst(rst), .radd1(IF_ID_Inst[19:15]) , .radd2(IF_ID_Inst[24:20]), .wadd(MEM_WB_Rd), 
  .wdata(data_mux_out),  .rdata1(rdata1) , .rdata2(rdata2));
  
  // put in imm gen
  ImmGen immgen(.gen_out(imm_out), .inst(IF_ID_Inst));
  
///////////////////////////////////////////////////////////////////////

// 11 + 32 + 32 + 32 + 32 + 4 + 5 + 5 + 5
// ID/EX REG
 nreg #(158) ID_EX(
    // order of control in picture: reg write, memtoreg, branch, aluop, alusrc
    // order of control in this implementation:
    // branch[10], mem read[9], mem to reg[8:7], mem write[6], alusrc[5], reg write[4], aluop[3:2], jal[1], jalr[0]
    .D({{branch, mr, mtoreg, mwrite, alusrc, regwr, aluop, jal, jalr}, IF_ID_PC, rdata1, rdata2, imm_out, {IF_ID_Inst[14:12], IF_ID_Inst[30]}, IF_ID_Inst[19:15], IF_ID_Inst[24:20], IF_ID_Inst[11:7]}),
    .Q({ID_EX_Ctrl,ID_EX_PC,ID_EX_RegR1,ID_EX_RegR2, ID_EX_Imm, ID_EX_Func,ID_EX_Rs1,ID_EX_Rs2,ID_EX_Rd}),    
    .load(load), 
    .rst(rst), 
    .clk(clk));

nshift  #(32) shifter(ID_EX_Imm, shift_out);
rca rca_inst( ID_EX_PC, shift_out, temp_pc2, overflow);

  
  forward forward(
        ID_EX_Rs1, 
        ID_EX_Rs2, 
        EX_MEM_Rd, 
        MEM_WB_Rd,
       /*  EX_MEM_Ctrl[7]*/ 1'b0,        // changed
        /* MEM_WB_Ctrl[4]*/ 1'b0,        // changed
        forwardA,
        forwardB);
 
    mux4x1 #(32) first_mux4x1(
    forwardA,                // 2-bit select line
   ID_EX_RegR1, data_mux_out, EX_MEM_ALU_out, 32'b0, 
    fmux_out        
);


   mux4x1 #(32) second_mux4x1(
    forwardB,                // 2-bit select line
    ID_EX_RegR2, EX_MEM_ALU_out, data_mux_out, 32'b0,             // switched order of data and alu output
    smux_out        
);

  // put in mux after regfile (32 input?) (make sure order is correct)
  // alusrc is selector 
  nmux2x1#(32) mux_reg(ID_EX_Ctrl[5], smux_out, ID_EX_Imm, B);


// AUIPC????
 // add imm to pc and store in upper 20 bits 
//    assign auipc = imm_out + {pc_out, 12'b0};
//    nmux2x1#(32) upper_imm_mux(instr[5], auipc , imm_out, up_imm);


// changed control 
  aluctrl aluctrl(ID_EX_Ctrl[3:2], ID_EX_Func[3:1], ID_EX_Func[0] , alusel );

  alu #(32) aluinst(fmux_out,  B, alusel, alures, cf, zf, vf, sf, ID_EX_Rs2, ID_EX_Func[3:1], branch_taken );
  
  // compute jal and jalr target in ex stage and then send it to next stage 
   wire [31:0] jal_target = ID_EX_PC + ID_EX_Imm;
   wire [31:0] jalr_target = (ID_EX_RegR1 + ID_EX_Imm) & ~1;
  
 //////////////////////////////////////////////////////////////
 
  // 8 + 32 + 1 + 32 + 32 + 5 + 32 + 32
  nreg #(174) EX_MEM(
  // order of control: reg wr, mem to reg, branch, mem read, memwrite
  // coming out: reg wr[7], mem to reg[6:5], branch[4], mem read[3], mem write[2], jal[1], jalr[0] 
  .D({{ID_EX_Ctrl[4], ID_EX_Ctrl[8:7], ID_EX_Ctrl[10], ID_EX_Ctrl[9], ID_EX_Ctrl[6], ID_EX_Ctrl[1], ID_EX_Ctrl[0]}, ID_EX_PC, temp_pc2, branch_taken, alures, ID_EX_RegR2, ID_EX_Rd, jal_target, jalr_target}),
  .Q({EX_MEM_Ctrl, EX_MEM_PC, EX_MEM_BranchAddOut, EX_MEM_Zero, EX_MEM_ALU_out, EX_MEM_RegR2, EX_MEM_Rd, EX_MEM_Jal_tar, EX_MEM_Jalr_tar}), 
  .load(load), 
  .rst(rst), 
  .clk(~clk));      // takes not clock 
  
  // put in data mem
  // added 3 bit func
  
    
 // HOW TO CHOOSE INSTR VS DATA MEM  (clk ???? or if it's writing or reading from data memory then access data mem?) 
 // EX_MEM_Ctrl[3] || EX_MEM_Ctrl[2] data read / write 
// nmux2x1#(32) mem_choose(clk, EX_MEM_PC, );
  // what to put for addr and data in? for EX_MEM_ALU_out[7:2] & EX_MEM_RegR2
  DataMem data_mem( .clk(clk), .MemRead(EX_MEM_Ctrl[3]) , .MemWrite(EX_MEM_Ctrl[2]), .funct3(ID_EX_Func[3:1]), .addr(EX_MEM_ALU_out[7:2]), .data_in(EX_MEM_RegR2), .data_out(data_mem_out));
  
  // and branch signal and zero flag
  assign pcsrc = (EX_MEM_Ctrl[4] & EX_MEM_Zero);
  
 
 ////////////////////////////////////////////////

  
  // 5 + 32 + 32 +5
  // MEM WB
  nreg #(74) MEM_WB(
  // Control out: regwr[4], memtoreg[3:2], jal[1], jalr[0]
  .D({{EX_MEM_Ctrl[7], EX_MEM_Ctrl[6:5], EX_MEM_Ctrl[1], EX_MEM_Ctrl[0]}, EX_MEM_PC, data_mem_out, EX_MEM_ALU_out, EX_MEM_Rd}),
  .Q({MEM_WB_Ctrl, MEM_WB_PC, MEM_WB_Mem_out, MEM_WB_ALU_out, MEM_WB_Rd}),     
  .load(load), 
  .rst(rst), 
  .clk(clk));
  // put in mux 
//   nmux2x1#(32) nmux_inst(MEM_WB_Ctrl[0], MEM_WB_ALU_out, MEM_WB_Mem_out, data_mux_out);
 
 assign EX_MEM_temp_pc = EX_MEM_PC + 4; 


 // UPDATE THIS !!!!  put auipc/lui stuff later  *****
   mux4x1 #(32) data_mux ( MEM_WB_Ctrl[3:2], MEM_WB_temp_pc, MEM_WB_ALU_out, MEM_WB_Mem_out,  /*up_imm*/ 32'b0,  data_mux_out);

 

    nmux2x1#(32) pc_mux(pcsrc,  MEM_WB_temp_pc, EX_MEM_BranchAddOut, result_pc);

//nmux2x1#(32) pc_mux(branch_taken, temp_pc, temp_pc2, result_pc);
  

  
//  assign result_pc = (jal) ? jal_target : 
//                     (jalr) ? jalr_target : 
//                     (branch_taken) ? temp_pc2 : 
//                     (pc_out + 4);
 
//always @(*) begin
//      halt = 1'b0;  
//      case (instr[6:0])
//          7'b1110011: begin  // ECALL 
//              if (instr[31:20] == 12'b0) begin
//                  halt = 1'b1;  // ECALL - Halt execution
//              end
//          end
//          7'b0001111: begin  // FENCE, FENCE.TSO
//              // No operation - leave halt as 0 to proceed to next instruction
//          end
//          default: halt = 1'b0;
//      endcase
//  end

 
 always @ (*) begin
case(ledSel)
2'b00 : led = instr [15:0];
2'b01: led = instr [31:16];
2'b10: led = {2'b0, aluop, alusel, zero, pcsrc, branch, mr, mtoreg, mwrite, alusrc, regwr };       // rest of ctrl, in what order?
endcase
case(ssdSel) 
4'b0000: ssd = pc_out [12:0];   // pc output
4'b0001: ssd = temp_pc [12:0];  // pc+4
4'b0010: ssd = temp_pc2 [12:0]; // branch target add output?
4'b0011: ssd = result_pc [12:0];    // pc input
4'b0100: ssd = rdata1 [12:0];       // data read reg file rs1
4'b0101: ssd = rdata2 [12:0];       // data read reg file rs2
4'b0110: ssd = data_mux_out [12:0]; // data input to reg file rs2
4'b0111: ssd = imm_out [12:0];  //imm gen out
4'b1000: ssd = shift_out [12:0];    // shift left out
4'b1001: ssd = B [12:0];        // output of alu 2nd source mux
4'b1010: ssd = alures [12:0];       // output of alu
4'b1011: ssd = data_mem_out [12:0]; // memory output
4'b1100: ssd = MEM_WB_Rd;
endcase

end
endmodule