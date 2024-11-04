

module main(input rst, input clk, input [1:0] ledSel, input [3:0] ssdSel, output reg [15:0] led, output reg [12:0] ssd );
// divide by 4??? for inst mem
//wire [31: 0] data_out;
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
wire jump;              // to jump after branch or not
wire [31:0] result_pc;
wire load = 1; // for pc reg

nreg #32 PC(result_pc, pc_out, load, rst, clk);
InstMem inst(.addr(pc_out[7:2]), .data_out(instr));
control ctrl( instr[6:2],  branch, mr, mtoreg, mwrite, alusrc, regwr, aluop);

// put in reg file (what to put in write data?)
 regfile regs( .wsig(regwr) ,  .clk(clk),  .rst(rst), .radd1(instr[19:15]) , .radd2(instr[24:20]), .wadd(instr[11:7]), 
  .wdata(data_mux_out),  .rdata1(rdata1) , .rdata2(rdata2));
  
  // put in imm gen
  ImmGen immgen(.gen_out(imm_out), .inst(instr));
  
  // put in mux after regfile (32 input?) (make sure order is correct)
  nmux2x1#(32) mux_reg(alusrc, rdata2 , imm_out, B);
  
  // inst input should be [14:12] and 30?
  aluctrl aluctrl(aluop, instr[14:12], instr[30], alusel );
  // need to get data from 1st reg in reg file into A
  alu #(32) aluinst(rdata1,  B, alusel, alures, zero);
  
  // put in data mem, divide by 4?
  DataMem data_mem( clk, mr , mwrite, alures[7:2], rdata2, data_mem_out);
  
  // put in mux 
   nmux2x1#(32) nmux_inst(mtoreg, alures, data_mem_out, data_mux_out);
   
   // write data to registers how? regfile inst again? or always block?
   // 32 bits for left shift?
   nshift  #(32) shifter(imm_out, shift_out);
   // put in adder, rca or full adder? 
   
  rca rca_inst( pc_out, shift_out, temp_pc2, overflow);
assign temp_pc = pc_out + 4; 
  // and branch signal and zero flag
  assign jump = (branch & zero);
  nmux2x1#(32) pc_mux(jump, temp_pc, temp_pc2, result_pc);
 
 always @ (*) begin
case(ledSel)
2'b00 : led = instr [15:0];
2'b01: led = instr [31:16];
2'b10: led = {2'b0, aluop, alusel, zero, jump, branch, mr, mtoreg, mwrite, alusrc, regwr };       // rest of ctrl, in what order?
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
endcase

end
endmodule
