`default_nettype none
module data_path(clk,reset,
                 pc_src,
                 reg_write,
                 mem_to_reg,
                 mem_write,
                 alu_control,
                 alu_src,
         
                
                 imm_src,
                 reg_src,
                 pc,
                 instr,
                 alu_result,
                 write_data,
                 read_data,
                 cond,op,funct,rd,
                 a,wd,
                 shift_flag,
                 swap);

    input clk,reset,
        pc_src,
        reg_write,
        mem_to_reg,
        mem_write,
        
        alu_src
        ;
    output [3:0]cond;
    output [1:0] op;
    output [5:0] funct;
    output [3:0] rd;
    input [2:0] alu_control;

       
    input [1:0] reg_src;
    output reg [31:0] pc;
    input [31:0] instr;
    output [31:0] alu_result;
    input [31:0] read_data;
    output [31:0] write_data;
    input [1:0] imm_src;
    output [31:0]a,wd;
    input shift_flag;
    input swap;

    wire [3:0] ra1,ra2;

    wire [31:0] result;
    assign cond = instr[31:28];
    assign op = instr[27:26];
    assign funct = instr[25:20];
    assign rd = instr[15:12];
    //assign  = instr[11:5];
    assign  ra1 = (reg_src[0]) ? 4'd15 :instr[19:16];
    assign  ra2 = (reg_src[1]) ? instr[15:12] : instr[3:0];
    //assign  = instr[23:0];


   wire [31:0] ext_imm;
   extend extend_u(.instr(instr[23:0]),.imm_src(imm_src),
              .ext_imm(ext_imm));

    // reg file
    wire [31:0] src_a,src_b;
    reg_file reg_file_u(.clk(clk),
                        .we3(reg_write),
                        .a1(ra1),
                        .a2(ra2),
                        .a3(instr[15:12]),
                        .wd3(result),
                        .r15(pc_plus8),
                        .rd1(src_a),
                        .rd2(write_data));


    wire [31:0] shift_result;
    shift shift_u(.instr(instr[11:0]), .rd2(write_data),.shift_result(shift_result));
    assign src_b = (alu_src)? ext_imm :  shift_result;


    //wire [31:0] alu_result;



   wire [31:0] pc_next;
   wire [31:0] pc_result4;
   wire [31:0] pc_plus8;

   // pc
   assign pc_next = (pc_src)? result : pc_result4;
   always @(posedge clk or posedge reset) begin
         if (reset) pc <= 32'b0;
         else pc <= #1 pc_next;
   end

   assign pc_result4 = pc + 4;
   assign pc_plus8 = pc_result4 + 4;

    // alu
    alu alu_u(.src_ina(src_a),
    .src_inb(src_b),
    .alu_control(alu_control),
           .alu_result(alu_result),
    .swap(swap));
   assign a = shift_flag? src_b :alu_result; 
   assign wd =  write_data;
   assign result = (mem_to_reg)? rd : alu_result;

endmodule

module extend(instr,imm_src,
              ext_imm);
    input [23:0]instr ;
    input [1:0] imm_src;
    output reg [31:0]ext_imm;
    always @(*) begin
        case (imm_src) 
            2'b00: ext_imm <= {24'b0,instr[7:0]};
            2'b01: ext_imm <= {20'b0,instr[11:0]};
            2'b10: ext_imm <= {{6{instr[23]}},instr[23:0],2'b00};
            default: ext_imm <= 32'bx;
        endcase      
    end        
endmodule


module reg_file(clk,we3,a1,a2,a3,wd3,r15,
                rd1,rd2);
    input clk,reset,we3;
    input [3:0] a1,a2,a3;
    input [31:0] wd3,r15;
    output [31:0] rd1,rd2;

    reg [31:0] r[14:0];

    always @(clk) begin
      if (we3) r[a3] <= wd3;
    end

    assign rd1 = (a1==4'b1111)? r15 : r[a1];
    assign rd2 = (a2==4'b1111)? r15 : r[a2];
endmodule

module shift(instr, rd2,shift_result);

    input [11:0] instr;
    input [31:0] rd2;
    output reg [31:0] shift_result;

    wire [1:0] sh;
    wire [3:0] rm;
    wire [5:0] shamt;
    wire flag;

    assign {shamt,sh,flag,rm} = instr;

    always @(*) begin
        case (sh) 
            //lsl
            2'b00: if (instr[11:4]!=0) shift_result <= rd2 << shamt;
            // lsr
            2'b01: shift_result <= shamt << rd2;
        endcase
    end

endmodule