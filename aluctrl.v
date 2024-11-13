//module aluctrl(input [1:0] aluop, input [2:0] inst, input in, output reg [3:0] alusel
//    );
//    always @ (*) begin
//    casez({aluop, inst, in})
//    6'b00zzzz: alusel = 4'b0010;
//    6'b01zzzz: alusel = 4'b0110;
//    6'b100000: alusel = 4'b0010;
//    6'b100001: alusel = 4'b0110;
//    6'b101110: alusel = 4'b0000;
//    6'b101100: alusel = 4'b0001;
    
//    endcase
//    end
//endmodule

// need to fix 
// old 01 -> 0110, new 01 -> 1010 (Done) 
// old 100001 -> 0110, new 100001-> 0100 (done)
// changed 10110? to 101100 for 0001 (done)
module aluctrl(input [1:0] aluop, input [2:0] inst, input in, output reg [3:0] alusel);
    always @(*) begin
        casez ({aluop, inst, in})
            6'b00????: alusel = 4'b0010; // Load/Store: ADD (aluop = 00, ignoring inst and in)
            6'b01????: alusel = 4'b1010; // Branch (aluop = 01, ignoring inst and in)
            6'b100000: alusel = 4'b0010; // ADD/ADDI (aluop = 10, inst = 000, in = 0)
            6'b100001: alusel = 4'b0100; // SUB (aluop = 10, inst = 000, in = 1)
            6'b10010?: alusel = 4'b1000; // SLTI (aluop = 10, inst = 010, ignoring in)
            6'b10011?: alusel = 4'b1001; // SLTIU (aluop = 10, inst = 011, ignoring in)
            6'b10111?: alusel = 4'b0000; // ANDI (aluop = 10, inst = 111, ignoring in)
            6'b10100?: alusel = 4'b0011; // XORI (aluop = 10, inst = 100, ignoring in)
            6'b10110?: alusel = 4'b0001; // ORI or OR(aluop = 10, inst = 110, ignoring in)
            6'b101110: alusel = 4'b0000; // AND (aluop = 10, inst = 111, in = 0)
//            6'b101101: alusel = 4'b0001; // OR (aluop = 10, inst = 101, in = 1)
            6'b100010: alusel = 4'b0101; // SLL (aluop = 10, inst = 001, in = 0)
            6'b101010: alusel = 4'b0110; // SRL (aluop = 10, inst = 010, in = 0)
            6'b101011: alusel = 4'b0111; // SRA (aluop = 10, inst = 011, in = 1)
            default: alusel = 4'b0000;   // Default case
        endcase
    end
endmodule