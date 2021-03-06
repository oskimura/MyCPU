`default_nettype none
module cond_logic(clk,reset,pcs,reg_w,mem_w,flag_w,cond,alu_flag,
                 branch,
                 pc_src,reg_write,mem_write,no_write,
                 branch_take);
    input clk,reset,pcs,reg_w,mem_w;
    input [1:0] flag_w;
    input [3:0] alu_flag,cond;
    input branch;

    output pc_src,reg_write,mem_write;
    input no_write;
    output branch_take;

    wire cond_ex;
    
    wire [3:0] flags;
    wire [1:0] flag_write;

    // always @(clk or reset or flag_write) begin
    //     if (reset) flags <= 4'b0;
    //     else if (flag_write) flags <= alu_flag;
    // end

    ff  #(2)ff_h(.clk(clk),
        .reset(reset),
        .en(flag_write[1]),
        .d(alu_flag[3:2]),
        .q(flags[3:2]));

    ff  #(2)ff_l(.clk(clk),
        .reset(reset),
        .en(flag_write[0]),
        .d(alu_flag[1:0]),
        .q(flags[1:0]));

    cond_check cond_check_u(.cond(cond),.flags(flags),.cond_ex(cond_ex));
    //assign flag_write = flag_w & {2{cond_ex}};
    assign flag_write = flag_w;
    
    assign pc_src = pcs & cond_ex;
    assign reg_write = reg_w & cond_ex & ~no_write;
    assign mem_write = (mem_w & cond_ex)?1:0;
    assign branch_take = branch & cond_ex;

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


module ff #(parameter W = 8)
          (clk,reset,en,d,q);
 input clk,reset,en;
 input [W-1:0] d;
 output reg [W-1:0] q;

    always @(posedge clk or posedge reset) begin
        if (reset) q<=0;
        else if (en) q<=d;
    end
endmodule