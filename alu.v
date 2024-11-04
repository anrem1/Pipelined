module alu #(parameter n = 32)(
input [n-1:0] A, input [n-1:0] B, input [3:0] sel, output reg [n-1:0] result, output reg zero
    );
    wire [n-1:0] Bout;
    wire [n-1:0] sum;
    wire ovflw;
    wire [n-1:0] AND;
    wire [n-1:0] OR;
    nmux2x1 #(32)mux (.s(sel[2]), .in1(B),  .in2(~B+1), .out(Bout));
    rca rca_inst(.A(A), .B(Bout),  .sum(sum), .overflow(ovflw));
    assign AND = A & B;
    assign OR = A | B;
    

    always @ (*) begin
    case(sel)
    4'b0010:  result = sum;
    4'b0110:  result = sum;
    4'b0000:  result = AND;
    4'b0001:  result = OR;
    default:  result = 32'b0;
    endcase

    if(result == 0)
    zero = 1;
    else 
    zero = 0;
    end
endmodule
