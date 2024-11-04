 module mux2x1( input s, input in1, input in2, output out);
assign out = (s? in2 : in1);
endmodule
