`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/05/2024 12:17:54 PM
// Design Name: 
// Module Name: forward
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module forward(
input [4:0] ID_EX_rs1, 
[4:0] ID_EX_rs2, 
[4:0] EX_MEM_rd, 
[4:0] MEM_WB_rd,
 input EX_MEM_regwr, 
 input MEM_WB_regwr,
 output reg [1:0] forwardA,
 output reg [1:0] forwardB);
 
 
 // EX HAZARD
 always @ (*) begin
 if( (EX_MEM_regwr && EX_MEM_rd != 0) && EX_MEM_rd == ID_EX_rs1)
 forwardA = 2'b10;
 else 
 forwardA = 2'b00;
 end 
 
 always @ (*) begin
 if( (EX_MEM_regwr && EX_MEM_rd != 0) && EX_MEM_rd == ID_EX_rs2)
 forwardB = 2'b10;
 else 
 forwardB = 2'b00;
 end 
  
 
 // MEM HAZARD
always @ (*) begin
if((( MEM_WB_regwr && MEM_WB_rd != 0) && ( MEM_WB_rd == ID_EX_rs1)) 
&& !(EX_MEM_regwr && (EX_MEM_rd != 0) && (EX_MEM_rd == ID_EX_rs1)))
 forwardA = 2'b01;
 else 
 forwardA = 2'b00;
 end
 
 always @ (*) begin 
 if((MEM_WB_regwr && (MEM_WB_rd != 0)
 && (MEM_WB_rd == ID_EX_rs2))
 && !(EX_MEM_regwr && EX_MEM_rd != 0 && EX_MEM_rd == ID_EX_rs2))
    forwardB = 2'b01;
  else 
 forwardB = 2'b00;
 end
endmodule
