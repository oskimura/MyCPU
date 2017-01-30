`default_nettype none
module alu(src_a,src_b,alu_control,
           alu_result,alu_flags);
    input [31:0] src_a;
    input [31:0] src_b;
    input [1:0] alu_control;
    output reg  [31:0] alu_result;
    output [3:0] alu_flags;

    wire zero,overflow,carry,neg;
    reg cout;

    always @(*) begin
        case (alu_control)
            2'b00: {cout,alu_result} <=  src_a + src_b;
            2'b01: {cout,alu_result} <=  src_a - src_b;
                    
            2'b10: alu_result <=  src_a & src_b;
            2'b11: alu_result <=  src_a | src_b;
        endcase
    end

    assign zero = (alu_result==0)? 1 : 0;
    assign neg = (alu_result<0)? 1 : 0;
    assign carry = ~(alu_control[1]&cout);
    //assign overflow = ((~src_a[31] && ~src_b[31] && alu_result[31]) || (src_a[31] && src_b[31] && ~alu_result[31])) ? 1 :0;
    assign overflow = ((~(alu_control[0]) & (src_a[31] ~^ src_b[31]) || 
                         (alu_control[0]) & (src_a[31] ^ src_b[31])) &&
                        (src_a[31] ^ alu_result[31]) &&
                       ~(alu_control[1])) ?1:0;

    assign alu_flags = {neg,zero,carry,overflow};    

endmodule