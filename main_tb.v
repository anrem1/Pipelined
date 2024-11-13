
module main_tb(
    );
    reg rst, clk;
    reg [1:0] ledSel;
    reg [3:0] ssdSel;
    wire [15:0] led;
    wire [12:0] ssd;
   RISCV_pipeline DUT( rst, clk, ledSel, ssdSel,  led,  ssd);
    initial begin
   clk = 1;
   forever #5 clk = ~clk;
   end

    initial begin
    rst = 1;
    
    #10 
    rst = 0;    
    ssdSel = 4'b0000;
    #500
    $finish;
    end
    
endmodule
