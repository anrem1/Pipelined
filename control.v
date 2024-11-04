module control(input [4:0] in, output reg branch, mr, mtoreg, mwrite, alusrc, regwr, output reg [1:0] aluop
    );
    
    always @ (*) 
    begin
    case(in)
    5'b01100: begin  branch = 0; mr = 0; mtoreg = 0; aluop = 10; mwrite = 0; alusrc = 0; regwr = 1; end
    5'b00000: begin branch = 0; mr = 1; mtoreg = 1; aluop = 00; mwrite = 0; alusrc = 1; regwr = 1; end
    5'b01000: begin branch = 0; mr = 0; mtoreg = 1'bX; aluop = 00; mwrite = 1; alusrc = 1; regwr = 0; end
    5'b11000: begin branch = 1; mr = 0; mtoreg = 1'bX; aluop = 01; mwrite = 0; alusrc = 0; regwr = 0; end
    endcase
    end
endmodule

