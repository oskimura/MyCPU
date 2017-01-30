module cond_logic(clk,pcs,reg_w,mem_w,flag_w,cond,alu_flag,
    input clk,pcs,reg_w,mem_w;
                 pc_src,reg_write,mem_write,no_write);
    input [1:0] flag_w;
    input [3:0] alu_flag,cond;

    output pc_src,reg_write,mem_write;
    input no_write;

    wire cond_ex;
    
    reg [3:0] flags;
    wire flag_write;

    assign flag_write = flag_w & cond_ex;
    always @(clk or flag_write) begin
        if (flag_write) flags <= 0;
        else
            flags <= alu_flag; 
    end
    cond_check cond_check_u(.cond(cond),.flags(flags),.cond_ex(cond_ex));

    assign pc_src = pcs & cond_ex;
    assign reg_write = reg_w & cond_ex;
    assign mem_write = mem_w; 

endmodule


module cond_check(cond,flags,cond_ex);
    input [3:0] cond,flags;
    output reg cond_ex;

    wire neg,zero,carry,overflow;
    assign {neg,zero,carry,overflow} = flags;
    always @(*) begin
        case (cond)
            4'b0000: cond_ex <= zero;
            4'b0001: cond_ex <= ~zero;
            4'b0010: cond_ex <= carry;
            4'b0011: cond_ex <= ~carry;
            4'b0100: cond_ex <= neg;
            4'b0101: cond_ex <= ~neg;
            4'b0110: cond_ex <= overflow;
            4'b0111: cond_ex <= ~overflow;
            4'b1000: cond_ex <= ~zero & carry;
            4'b1001: cond_ex <= zero | ~carry;
            4'b1010: cond_ex <= ~(neg ^ overflow);
            4'b1011: cond_ex <= neg ^ overflow;
            4'b1100: cond_ex <= ~zero & ~(neg ^ overflow);
            4'b1101: cond_ex <= zero | (neg ^ overflow);
            4'b1110: cond_ex <= 1'b1;
            default: cond_ex <= 1'bx;
        endcase
    end
endmodule