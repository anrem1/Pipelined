module nreg #(parameter n = 8) (input [n-1:0] D, output [n-1:0] Q, input load, input rst, input clk);
wire [n-1:0] out; 
genvar i;
generate
for (i=0; i<n; i=i+1) begin: mygenloop
mux2x1 minst( .s(load),  .in1(Q[i]), .in2(D[i]), .out(out[i]));
DFlipFlop dinst( .clk(clk),  .rst(rst),  .D(out[i]), .Q(Q[i]));
end
endgenerate


endmodule

