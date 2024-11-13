module StallUnit(input [4:0] IF_ID_rs1_addr, IF_ID_rs2_addr, ID_EX_rd_addr, input ID_EX_MemRead, output reg stall);

always @* begin
    if((IF_ID_rs1_addr == ID_EX_rd_addr | IF_ID_rs2_addr == ID_EX_rd_addr) & ID_EX_MemRead & (ID_EX_rd_addr != 0 ))
        stall = 1;
    else
        stall = 0; 
end

endmodule
