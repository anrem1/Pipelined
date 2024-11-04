module nmux2x1#(parameter n=8)(input s, input [n-1:0] in1, input [n-1:0] in2, output [n-1:0] out);
genvar i;
generate
for (i=0; i<n; i=i+1) begin: mygenloop
mux2x1 minst( .s(s),  .in1(in1[i]), .in2(in2[i]), .out(out[i]));
end
endgenerate


endmodule

