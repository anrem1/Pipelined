module regfile(
input wsig, input clk, input rst, input [4:0] radd1 , input [4:0] radd2, input [4:0] wadd, 
 input [31:0] wdata,  output  [31:0] rdata1 , output  [31:0] rdata2 
    );
    integer i;
    reg [31:0] register [31:0];
  assign    rdata1 = register[radd1];
  assign  rdata2 = register[radd2];
  
    always @ (negedge clk) begin

    if(rst == 0) begin
  

    if(wsig == 1 & wadd != 0 ) 
    register[wadd] = wdata;
    
    end
   else // if rst = 1 
   begin
    for(i = 0; i<32; i=i+1) 
    register[i] = 0;
    end
    end

endmodule

