`default_nettype none
module contorol(clk,reset,op,cond,funct,rd,

                 pc_src,
                 reg_write,
                 mem_to_reg,
                 mem_write,
                 alu_control,
                 branch,
                 alu_src,
                 //flag_write,
                 imm_src,
                 reg_src,
                 
                 alu_flag,
                 shift_flag,
                 swap);
    input clk,reset;
    input [1:0] op;
    input [3:0] cond;
    input [5:0] funct;
    input [3:0] rd;
    output pc_src,
           reg_write,
           mem_to_reg,
           mem_write,      
           branch,
           alu_src;
    //output [1:0] flag_write;
    output [1:0] imm_src;
    output [2:0] alu_control;
    output [1:0] reg_src;

    input [3:0] alu_flag;
    output shift_flag;
    output swap;

    //wire [1:0] flag_write;
    wire pcs;
    wire reg_w;
    wire mem_w;
    wire branch;
    wire [3:0]alu_flag;

    wire [1:0] flag_write;
    wire no_write;

    decoder decoder_u(.op(op), 
    .funct(funct), 
    .rd(rd),

                       .pcs(pcs),
                       .reg_w(reg_w),
                       .mem_w(mem_w),
                       .mem_to_reg(mem_to_reg),
                       .alu_src(alu_src),
                       .imm_src(imm_src),
                       .reg_src(reg_src),
                       .alu_control(alu_control),
                       .flag_w(flag_write),
                       .no_write(no_write),
                       .shift_flag(shift_flag),
                       .swap(swap));

    cond_logic cond_logic_u(.clk(clk),
    .reset(reset),
    .pcs(pcs),
    .reg_w(reg_w),
    .mem_w(mem_w),

                        .flag_w(flag_write),
                        .cond(cond),
                        .alu_flag(alu_flag),
                        .pc_src(pc_src),
                        .reg_write(reg_write),
                        .mem_write(mem_write),
                        .no_write(no_write));


endmodule