module mux4x1 #(parameter n = 8) (
    input [1:0] s,                // 2-bit select line
    input [n-1:0] in0, in1, in2, in3, 
    output reg [n-1:0] out         
);
    always @(*) begin
        case (s)
            2'b00: out = in0;
            2'b01: out = in1;
            2'b10: out = in2;
            2'b11: out = in3;
            default: out = {n{1'b0}};
        endcase
    end
endmodule