module alu #(parameter n = 32)(
    input [n-1:0] A,
    input [n-1:0] B,
    input [3:0] sel,
    output reg [n-1:0] result,
    output wire cf, zf, vf, sf,
    input [4:0] shamt,
    input [2:0] funct3,
    output reg branch_taken 
);

    wire [n-1:0] AND = A & B;
    wire [n-1:0] OR = A | B;
    wire [n-1:0] XOR = A ^ B;
    wire [n-1:0] SLL = A << shamt;
    wire [n-1:0] SRL = A >> shamt;
    wire [n-1:0] SRA = A >>> shamt;
    wire [n-1:0] add, sub;
    wire [n-1:0] SLT = ($signed(A) < $signed(B)) ? 32'b1 : 32'b0;    // Signed comparison for SLT
    wire [n-1:0] SLTU = (A < B) ? 32'b1 : 32'b0;                      // Unsigned comparison for SLTU

    // Carry-out and Overflow Detection
//    assign {cf, add} = sel == 4'b00_10 ? (A - B) : (A + B);
    rca adder( A, B, add, cf);
    rca subber( A, B, sub, cf);
     

    assign zf = (result == 0);
    assign sf = result[n-1];
    assign vf = (A[n-1] ^ B[n-1]) ^ result[n-1] ^ cf;
    
    always @(*) begin
        branch_taken = 1'b0; 
        case(sel)
            4'b0000: result = AND;
            4'b0001: result = OR;
            4'b0010: result = add;
            4'b0100: result = sub;
            4'b0011: result = XOR;
            4'b0100: result = A - B;
            4'b0101: result = SLL;
            4'b0110: result = SRL;
            4'b0111: result = SRA;
            4'b1000: result = SLT;
            4'b1001: result = SLTU;
            4'b1010: begin // Branching conditions
                case (funct3)
                    3'b000: branch_taken = (A == B);                // BEQ
                    3'b001: branch_taken = (A != B);                // BNE
                    3'b100: branch_taken = ($signed(A) < $signed(B)); // BLT
                    3'b101: branch_taken = ($signed(A) >= $signed(B)); // BGE
                    3'b110: branch_taken = (A < B);                 // BLTU
                    3'b111: branch_taken = (A >= B);                // BGEU
                    default: branch_taken = 1'b0;
                endcase
            end
            default: result = 32'b0;
        endcase
    end
endmodule

//module alu #(parameter n = 32)(
//input [n-1:0] A, input [n-1:0] B, input [3:0] sel, output reg [n-1:0] result, output reg zero
//    );
//    wire [n-1:0] Bout;
//    wire [n-1:0] sum;
//    wire ovflw;
//    wire [n-1:0] AND;
//    wire [n-1:0] OR;
//    nmux2x1 #(32)mux (.s(sel[2]), .in1(B),  .in2(~B+1), .out(Bout));
//    rca rca_inst(.A(A), .B(Bout),  .sum(sum), .overflow(ovflw));
//    assign AND = A & B;
//    assign OR = A | B;
    

//    always @ (*) begin
//    case(sel)
//    4'b0010:  result = sum;
//    4'b0110:  result = sum;
//    4'b0000:  result = AND;
//    4'b0001:  result = OR;
//    default:  result = 32'b0;
//    endcase

//    if(result == 0)
//    zero = 1;
//    else 
//    zero = 0;
//    end
//endmodule
