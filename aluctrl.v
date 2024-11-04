module aluctrl(input [1:0] aluop, input [2:0] inst, input in, output reg [3:0] alusel
    );
    always @ (*) begin
    casez({aluop, inst, in})
    6'b00zzzz: alusel = 4'b0010;
    6'b01zzzz: alusel = 4'b0110;
    6'b100000: alusel = 4'b0010;
    6'b100001: alusel = 4'b0110;
    6'b101110: alusel = 4'b0000;
    6'b101100: alusel = 4'b0001;
    
    endcase
    end
endmodule
