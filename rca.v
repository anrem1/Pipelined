
module rca(
    A, B,
    sum,
    overflow
);
    parameter n=32;
    input [n-1:0] A, B;
    output [n:0] sum;
    output overflow; 
    
    wire [n:0] c;
    wire [31:0] sum_temp;
    genvar i;
    
    assign c[0] = 0;

    generate
        for (i=0; i<=31; i=i+1) begin: mygenloop
            fulladder fa(.A(A[i]), .B(B[i]), .cin(c[i]), .sum(sum_temp[i]), .cout(c[i+1]));
        end
    endgenerate

    assign overflow = (A[31] == B[31]) && (sum_temp[31] != A[31]);
    
    assign sum = sum_temp;
endmodule
