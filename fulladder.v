module fulladder(input A, input B, input cin, output cout, output sum );
assign {cout, sum} = A + B + cin;
endmodule
