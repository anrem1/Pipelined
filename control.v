//module control(input [4:0] in, output reg branch, mr, mtoreg, mwrite, alusrc, regwr, output reg [1:0] aluop
//    );
    
//    always @ (*) 
//    begin
//    case(in)
//    5'b01100: begin  branch = 0; mr = 0; mtoreg = 0; aluop = 10; mwrite = 0; alusrc = 0; regwr = 1; end
//    5'b00000: begin branch = 0; mr = 1; mtoreg = 1; aluop = 00; mwrite = 0; alusrc = 1; regwr = 1; end
//    5'b01000: begin branch = 0; mr = 0; mtoreg = 1'bX; aluop = 00; mwrite = 1; alusrc = 1; regwr = 0; end
//    5'b11000: begin branch = 1; mr = 0; mtoreg = 1'bX; aluop = 01; mwrite = 0; alusrc = 0; regwr = 0; end
//    endcase
//    end
//endmodule


module control(input rst, input [4:0] in, output reg branch, mr, mwrite, alusrc, regwr, output reg [1:0] aluop, output reg [1:0] mtoreg, output reg jal, jalr);
    always @ (*) begin
    if (rst) {branch, mr, mtoreg, aluop, mwrite, alusrc, regwr, jal, jalr} = 11'd0;
    else
        case (in)
            5'b01100: begin // R-type (add, sub, etc.)
                branch = 0; mr = 0; mtoreg = 2'b01; aluop = 2'b10; mwrite = 0; alusrc = 0; regwr = 1; jal = 0; jalr =0 ;
            end
            5'b00000: begin // Load (e.g., LW, LB, LH, etc.)
                branch = 0; mr = 1; mtoreg = 2'b10; aluop = 2'b00; mwrite = 0; alusrc = 1; regwr = 1; jal = 0; jalr =0 ;
            end
            5'b01000: begin // Store (e.g., SW, SB, SH)
                branch = 0; mr = 0; mtoreg = 2'bXX; aluop = 2'b00; mwrite = 1; alusrc = 1; regwr = 0; jal = 0; jalr =0 ;
            end
            5'b11000: begin // Branch (e.g., BEQ, BNE, BLT, etc.)
                branch = 1; mr = 0; mtoreg = 2'bXX; aluop = 2'b01; mwrite = 0; alusrc = 0;  regwr = 0; jal = 0; jalr =0 ;
            end
            5'b11011: begin // JAL
                branch = 0; mr = 0; mtoreg = 2'b01; aluop = 2'bXX; mwrite = 0; alusrc = 1;  regwr = 1; jal = 1; jalr = 0;
            end
            5'b11001: begin // JALR
                branch = 0; mr = 0; mtoreg = 2'b00; aluop = 2'bXX; mwrite = 0; alusrc = 1; regwr = 1; jalr = 1; jal = 0;
            end
            5'b00101: begin // AUIPC
                branch = 0; mr = 0; mtoreg = 2'b11; aluop = 2'b10; mwrite = 0; alusrc = 1; regwr = 1; jal = 0; jalr =0 ;
            end
            5'b01101: begin // LUI
                branch = 0; mr = 0; mtoreg = 2'b11; aluop = 2'bXX; mwrite = 0; alusrc = 1;  regwr = 1; jal = 0; jalr =0 ;
            end
            5'b00100: begin // Immediate
                branch = 0; mr = 0; mtoreg = 2'b01 ; aluop = 2'b10 ; mwrite = 0; alusrc = 1;  regwr = 1; jal = 0; jalr =0 ;
            end
            5'b00011: begin // fence 
                branch = 0; mr = 0; mtoreg = 2'b00 ; aluop = 2'b00 ; mwrite = 0; alusrc = 0;  regwr = 0; jal = 0; jalr = 0 ;
                end
            5'b11100: begin // ebreak 
                branch = 0; mr = 0; mtoreg = 2'b00 ; aluop = 2'b00 ; mwrite = 0; alusrc = 0;  regwr = 0; jal = 0; jalr = 0 ;
                end
            default: begin 
                branch = 0; mr = 0; mtoreg = 2'b00; aluop = 2'b00; mwrite = 0; alusrc = 0;  regwr = 0; jal = 0; jalr =0 ;
            end
        endcase
    end
endmodule