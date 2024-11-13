module DataMem(
    input clk, 
    input MemRead, 
    input MemWrite, 
    input [2:0] funct3,
    input [5:0] addr,       // keep it 6 bits? 
    input [31:0] data_in, 
    output reg [31:0] data_out
);
    reg [31:0] mem [0:63];

    always @(*) begin
        if (MemRead) begin
            case (funct3)
                3'b000: data_out = {{24{mem[addr][7]}}, mem[addr][7:0]};     // LB
                3'b001: data_out = {{16{mem[addr][15]}}, mem[addr][15:0]};   // LH
                3'b010: data_out = mem[addr];                                // LW
                3'b100: data_out = {24'b0, mem[addr][7:0]};                  // LBU
                3'b101: data_out = {16'b0, mem[addr][15:0]};                 // LHU
                default: data_out = 32'b0;
            endcase
        end else begin
            data_out = 32'bz;
        end
    end

    always @(posedge clk) begin
        if (MemWrite) begin
            case (funct3)
                3'b000: mem[addr][7:0] = data_in[7:0];                       // SB
                3'b001: mem[addr][15:0] = data_in[15:0];                     // SH
                3'b010: mem[addr] = data_in;                                 // SW
            endcase
        end
    end

    initial begin
        mem[53] = 32'd17;
        mem[54] = 32'd9;
        mem[55] = 32'd25;
    end
    
    initial begin 
    mem[0]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    //added to be skipped since PC starts with 4 after reset
    mem[1]=32'b000000000000_00000_010_00001_0000011 ; //lw x1, 0(x0)        // 17 in x1
    mem[2]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[3]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[4]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[5]=32'b000000000100_00000_010_00010_0000011 ; //lw x2, 4(x0)        // 9 in x2
    mem[6]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[7]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[8]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[9]=32'b000000001000_00000_010_00011_0000011 ; //lw x3, 8(x0)        // 25 in x3
    mem[10]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[11]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[12]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[13]=32'b0000000_00010_00001_110_00100_0110011 ; //or x4, x1, x2     // 9 or 17 = 25 in x4
    //          1098765 43210 98765 432 10987 6543210    
    mem[14]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[15]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[16]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[17]=32'b0_000001_00011_00100_000_0000_0_1100011; //beq x4, x3, 16   // x4 = x3, move pc + 16 (skip 4 inst)
    mem[18]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[19]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[20]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[21]=32'b0000000_00010_00001_000_00011_0110011 ; //add x3, x1, x2    // 25 in x3
    mem[22]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[23]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[24]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[25]=32'b0000000_00010_00011_000_00101_0110011 ; //add x5, x3, x2    // 34 in x5
    mem[26]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[27]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[28]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[29]=32'b0000000_00101_00000_010_01100_0100011; //sw x5, 12(x0)      // mem[3] = 34
    mem[30]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[31]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[32]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[33]=32'b000000001100_00000_010_00110_0000011 ; //lw x6, 12(x0)      // x6 = 34
    mem[34]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[35]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[36]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[37]=32'b0000000_00001_00110_111_00111_0110011 ; //and x7, x6, x1    // x7 = 51
    mem[38]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[39]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[40]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[41]=32'b0100000_00010_00001_000_01000_0110011 ; //sub x8, x1, x2    // x8 = 8
    mem[42]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[43]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[44]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[45]=32'b0000000_00010_00001_000_00000_0110011 ; //add x0, x1, x2    // x0 = 0 
    mem[46]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[47]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[48]=32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0
    mem[49]=32'b0000000_00001_00000_000_01001_0110011 ; //add x9, x0, x1    // x9 = 17
    end
endmodule

//module DataMem(input clk, input MemRead, input MemWrite,
//input [5:0] addr, input [31:0] data_in, output [31:0] data_out);
//// made addr 32 bits instead of 6 
//reg [31:0] mem [0:63];
//assign data_out = (MemRead? mem[addr] : data_out);
//always @ (posedge clk) begin
//if(MemWrite == 1) 
//mem[addr] = data_in;
//end

//initial begin 
//mem[0]=32'd17;
//mem[1]=32'd9;
//mem[2]=32'd25;

////mem[0]=32'd30;
////mem[1]=32'd25;
////mem[2]=32'd5;

//end
//endmodule
