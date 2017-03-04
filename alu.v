`default_nettype none
module alu(src_ina,
           src_inb,
           alu_control,
           alu_result,
           alu_flags,
           swap);
    input [31:0] src_ina;
    input [31:0] src_inb;
    input [2:0] alu_control;
    output reg  [31:0] alu_result;
    output [3:0] alu_flags;
    input swap;

    wire zero,overflow,carry,neg;
    reg cout;
    wire [31:0] src_a,src_b;

    assign src_a = swap? src_inb: src_ina;
    assign src_b = swap? src_ina: src_inb;

    always @(*) begin
        case (alu_control)

            // add
            3'b000: {cout,alu_result} <=  src_a + src_b;
            // sub
            3'b001: {cout,alu_result} <=  src_a - src_b;

            // and
            3'b010: alu_result <=  src_a & src_b;
            // or
            3'b011: alu_result <=  src_a | src_b;

            // addc
            3'b100: {cout,alu_result} <= src_a + src_b + carry;
            // subc
            3'b101: {cout,alu_result} <= src_a - src_b - carry;

            // xor
            3'b111: alu_result <=  src_a ^ src_b;

            // shift
        endcase
    end

    assign zero = (alu_result==0)? 1 : 0;
    assign neg = alu_result[31]? 1 : 0;
    //assign overflow = ((~src_a[31] && ~src_b[31] && alu_result[31]) || (src_a[31] && src_b[31] && ~alu_result[31])) ? 1 :0;
    assign overflow = ((~(alu_control[0]) & (src_a[31] ~^ src_b[31]) || 
                         (alu_control[0]) & (src_a[31] ^ src_b[31])) &&
                        (src_a[31] ^ alu_result[31]) &&
                       ~(alu_control[1])) ?1:0;
    assign carry = (~alu_control[1]&cout);

    assign alu_flags = {neg,zero,carry,overflow};    

endmodule