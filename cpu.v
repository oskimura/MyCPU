`default_nettype none
module cpu(clk,reset,

           instr,pc,

           alu_result,
           write_data,
           read_data);

input clk,reset;

input [31:0] instr;
output [31:0] pc;

output [31:0] alu_result;
output [31:0] write_data;
input [31:0] read_data;


wire pc_src,
   mem_to_reg,
   mem_write,
   
   alu_src,
   reg_write,
   swap
   ;

     
     wire [3:0] cond;
     wire [1:0] op;
     wire [5:0] funct;
     wire [3:0] rd;
     wire [2:0] alu_control;
     wire [1:0] imm_src,reg_src;
     
     wire shift_flag;

     assign cond = instr[31:28];
     assign op = instr[27:26];
     assign funct = instr[25:20];
     assign rd = instr[15:12];

contorol contorol_u(.clk(clk),
                  .op(op),
                  .funct(funct),
                  .rd(rd),

                 .pc_src(pc_src),
                 .reg_write(reg_write),
                 .mem_to_reg(mem_to_reg),
                 .mem_write(mem_write),
                 .alu_control(alu_control),
                 .alu_src(alu_src),
                 //.flag_write(flag_write),
                 .imm_src(imm_src),
                 .reg_src(reg_src),
                 .shift_flag(shift_flag),
                 .swap(swap));

data_path data_path_u(.pc_src(pc_src),
                      .reg_write(reg_write),
                      .mem_to_reg(mem_to_reg),
                      .mem_write(mem_write),
                      .alu_control(alu_control),
                      .alu_src(alu_src),
                 
                 
                 .imm_src(imm_src),
                 .reg_src(reg_src),
                 .pc(pc),
                 .instr(instr),
                 .alu_result(alu_result),
                 .write_data(write_data),
                 .read_data(read_data),
                 .shift_flag(shift_flag),
                 .swap(swap));

endmodule