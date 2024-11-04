
module ImmGen (output reg [31:0] gen_out, input [31:0] inst);
always @ (*) begin
if(inst[6] == 1)
 gen_out = {{21{inst[31]}}, inst[7], inst[30:25], inst[11:8]};
else
if(inst[5] == 1) 
gen_out = {{20{inst[31]}}, inst[31:25], inst[11:7]};
if(inst[5] == 0)
     gen_out = (inst[31]? {20'b1 ,inst[31:20]}: {20'b0 ,inst[31:20]});

end
endmodule

