module RISCV_pipeline ( input rst, input clk, input [1:0] ledSel, input [3:0] ssdSel, output reg [15:0] led, output reg [12:0] ssd  );
// wires declarations
wire [31:0] instr;
wire [31:0] pc_out;
wire [31:0] imm_out;
wire branch, mr, mtoreg, mwrite, alusrc, regwr; // reg?
wire [1:0] aluop; // reg?
wire [31:0] rdata1, rdata2;

wire [31:0] B;   // inputs to ALU
wire [3:0] alusel;
wire [31:0] alures; // reg?
wire zero;  // reg?
wire [31:0] data_mem_out;   // data memory output
wire [31:0] data_mux_out; 
wire [31:0] shift_out; 
wire [31:0] temp_pc; 
wire [31:0] temp_pc2;
wire overflow;          // for adder after shift
wire pcsrc;              // to jump after branch or not
wire [31:0] result_pc;
wire load = 1; // for pc reg

wire [31:0] IF_ID_PC, IF_ID_Inst;

wire [31:0] ID_EX_PC, ID_EX_RegR1, ID_EX_RegR2, ID_EX_Imm;
// 3 control signals, first 1 (WB) is 2 bits (regwr and memtoreg), 2nd is 3 bits (Branch, Mread, Mwrite), ex is alusrc (1 bit) and aluop (2 bits) [8 bits total]
wire [7:0] ID_EX_Ctrl;
wire [3:0] ID_EX_Func;
// addresses of regs for forwarding
wire [4:0] ID_EX_Rs1, ID_EX_Rs2, ID_EX_Rd;


// Rs1 and Rs2 are needed later for the forwarding unit
wire [31:0] EX_MEM_BranchAddOut, EX_MEM_ALU_out, EX_MEM_RegR2;
wire [4:0] EX_MEM_Ctrl;
wire [4:0] EX_MEM_Rd;
wire EX_MEM_Zero;


wire [31:0] MEM_WB_Mem_out, MEM_WB_ALU_out;
wire [1:0] MEM_WB_Ctrl;
wire [4:0] MEM_WB_Rd;

// forward 
wire [1:0] forwardA, forwardB;
wire [31:0] fmux_out, smux_out;

nreg #32 PC(result_pc, pc_out, load, rst, clk);

InstMem inst(.addr(pc_out[7:2]), .data_out(instr));

// IF/ID REG 
nreg #(64) IF_ID (
.D({pc_out, instr[31:0]}),     
.Q({IF_ID_PC, IF_ID_Inst}), 
.load(load), 
.rst(rst), 
.clk(clk));

// control
control ctrl( IF_ID_Inst[6:2],  branch, mr, mtoreg, mwrite, alusrc, regwr, aluop);

// put in reg file (what to put in write data?)
 regfile regs( .wsig(MEM_WB_Ctrl[1]) ,  .clk(clk),  .rst(rst), .radd1(IF_ID_Inst[19:15]) , .radd2(IF_ID_Inst[24:20]), .wadd(IF_ID_Inst[11:7]), 
  .wdata(data_mux_out),  .rdata1(rdata1) , .rdata2(rdata2));
  
  // put in imm gen
  ImmGen immgen(.gen_out(imm_out), .inst(IF_ID_Inst));
  
// 8 + 32 + 32 + 32 + 32 + 4 + 5 + 5 + 5
// ID/EX REG
nreg #(155) ID_EX(
// order of control in picture: reg write, memtoreg, branch, aluop, alusrc
// order of control in this implementation:
// branch[7], mem read[6], mem to reg[5], mem write[4], alusrc[3], reg write[2], aluop[1:0]
.D({{branch, mr, mtoreg, mwrite, alusrc, regwr, aluop}, IF_ID_PC, rdata1, rdata2, imm_out, {instr[14:12], instr[30]}, IF_ID_Inst[19:15], IF_ID_Inst[24:20], IF_ID_Inst[11:7]}),
.Q({ID_EX_Ctrl,ID_EX_PC,ID_EX_RegR1,ID_EX_RegR2, ID_EX_Imm, ID_EX_Func,ID_EX_Rs1,ID_EX_Rs2,ID_EX_Rd}),      // rs1 and rs2 put later?
.load(load), 
.rst(rst), 
.clk(clk));

nshift  #(32) shifter(ID_EX_Imm, shift_out);
rca rca_inst( ID_EX_PC, shift_out, temp_pc2, overflow);

  // put in mux after regfile (32 input?) (make sure order is correct)
  nmux2x1#(32) mux_reg(ID_EX_Ctrl[3], ID_EX_RegR2, ID_EX_Imm, B);
  
  forward forward(
        ID_EX_Rs1, 
        ID_EX_Rs2, 
        EX_MEM_Rd, 
        MEM_WB_Rd,
         EX_MEM_Ctrl[4], 
         MEM_WB_Ctrl[1],
        forwardA,
        forwardB);
 
    mux4x1 #(32) first_mux4x1(
forwardA,                // 2-bit select line
   ID_EX_RegR1, data_mux_out, EX_MEM_ALU_out, 32'b0, 
    fmux_out        
);


   mux4x1 #(32) second_mux4x1(
forwardB,                // 2-bit select line
    B, data_mux_out, EX_MEM_ALU_out, 32'b0, 
    smux_out        
);

  aluctrl aluctrl(ID_EX_Ctrl[1:0], ID_EX_Func[2:1], ID_EX_Func[0] , alusel );

  alu #(32) aluinst(fmux_out,  smux_out, alusel, alures, zero);
  
  // 5 + 32 + 1 + 32 + 32 + 5
  nreg #(107) EX_MEM(
  // order of control: reg wr, mem to reg, branch, mem read, memwrite
  // coming out: reg wr[4], mem to reg[3], branch[2], mem read[1], mem write[0]
  .D({{ID_EX_Ctrl[2], ID_EX_Ctrl[5], ID_EX_Ctrl[7], ID_EX_Ctrl[6], ID_EX_Ctrl[4]}, temp_pc2, zero, alures, ID_EX_RegR2, ID_EX_Rd}),
  .Q({EX_MEM_Ctrl, EX_MEM_BranchAddOut, EX_MEM_Zero, EX_MEM_ALU_out, EX_MEM_RegR2, EX_MEM_Rd}), 
  .load(load), 
  .rst(rst), 
  .clk(clk));
  
  // put in data mem, divide by 4?
  DataMem data_mem( clk, EX_MEM_Ctrl[1] , EX_MEM_Ctrl[0], EX_MEM_ALU_out[7:2], EX_MEM_RegR2, data_mem_out);
  
  // and branch signal and zero flag
  assign pcsrc = (EX_MEM_Ctrl[2] & EX_MEM_Zero);
  
  // 2 + 32 + 32 +5
  // MEM WB
  nreg #(71) MEM_WB(
  // Control out: regwr[1], memtoreg[0]
  .D({{EX_MEM_Ctrl[4], EX_MEM_Ctrl[3]}, data_mem_out, EX_MEM_ALU_out, EX_MEM_Rd}),
  .Q({MEM_WB_Ctrl, MEM_WB_Mem_out, MEM_WB_ALU_out, MEM_WB_Rd}),     
  .load(load), 
  .rst(rst), 
  .clk(clk));
  // put in mux 
   nmux2x1#(32) nmux_inst(MEM_WB_Ctrl[0], MEM_WB_ALU_out, MEM_WB_Mem_out, data_mux_out);
 
 
assign temp_pc = pc_out + 4; 
nmux2x1#(32) pc_mux(pcsrc, temp_pc, EX_MEM_BranchAddOut, result_pc);

 
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
