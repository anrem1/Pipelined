`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/15/2024 11:19:34 AM
// Design Name: 
// Module Name: main_ssd
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


module main_ssd(input rst, input clk, input [1:0] ledSel, input [3:0] ssdSel, output [15:0] led,
input ssd_clk ,output  [3:0] Anode,  output  [6:0] LED_out);
wire [12:0] ssd;
main main_inst( rst,  clk,  ledSel,  ssdSel, led, ssd );

Four_Digit_Seven_Segment_Driver_Optimized ssd_inst(
 ssd_clk,
ssd,
 Anode,
 LED_out
);


endmodule
