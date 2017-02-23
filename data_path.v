`default_nettype none

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

endmodule///////////////////////////////////////////
///////////////////////////////////////////
// fetch
module fetch(
    input clk,
    input reset, 
    // ALU result
    input [31:0] result_w,
    // flag
    input pc_src_w,
    // input 
    input instr,

    // stall
    input stall_f,

    // deley
    input branch_take_e,
    input [31:0]alu_result_e,

    // instuction  Memory
    output reg [31:0] instr_f,
    // program counter
    output reg [31:0] pc_f,
    output [31:0] pc_plus4_f    
);
    wire [31:0] pc_next;
    wire [31:0] pc;

   assign pc_next = (pc_src_w)? result_w : pc_plus4_f;
   assign pc = branch_take_e ? alu_result_e : pc_next;


   always @(posedge clk or posedge reset) begin
         if (reset) begin 
             pc_f <= 32'b0;
             instr_f <=32'b0;
         end
         else if (stall_f) begin
            pc_f<=pc_f;
            instr_f<=instr_f;
         end
         else begin 
            //pc_f <= #1 pc_next;
            pc_f <= #1 pc;
            instr_f <= instr;
         end
   end

   assign pc_plus4_f = pc_f + 4;

endmodule


///////////////////////////////////////////
// decode
module decode(
    input clk,
    input reset,

    // input instructin 
    input [31:0] instr_f,
    // input for R15
    input [31:0] pc_plus4_f,
    // input reg writer 
    input we3,
    // input for A3
    input [31:0] wa3_w,

    // stall
    input stall_d,
    input flush_d,



    // control output
    output pc_src_d,
    output reg_write_d,
    output mem_to_reg_d,
    output mem_write_d,
    output [2:0] alu_control_d,
    output branch_d,
    output alu_src_d,
    output [1:0] flag_write_d,
    output [1:0] imm_src_d,
    output [1:0] reg_src_d,

    output [3:0] cond_d,

    // alu output
    //output ,
    output [31:0] rd1_d,
    output [31:0] rd2_d,

    // imm
    output [31:0] ext_imm_d    
);

    reg [31:0] instr_d;
    always @(posedge clk) begin
        if (reset | flush_d) 
        instr_d <=32'b0;
        else if (stall_d) 
            instr_d<=32'b0;
        else
        instr_d <= instr_f;
    end


    wire [1:0] op;
    wire [5:0] funct;
    wire [3:0] rd;
    wire [3:0] ra1;
    wire [3:0] ra2;


    assign cond_d = instr_d[31:28];
    assign op = instr_d[27:26];
    assign funct = instr_d[25:20];
    assign rd = instr_d[15:12];
    assign  ra1 = (reg_src_d[0]) ? 4'd15 :instr_d[19:16];
    assign  ra2 = (reg_src_d[1]) ? instr_d[15:12] : instr_d[3:0];


    // ext imm
   wire [31:0] ext_imm;
   extend extend_u(.instr(instr_d[23:0]),.imm_src(imm_src_d),
              .ext_imm(ext_imm_d));

    // reg file
    wire [31:0] pc_plus8_d;
    assign pc_plus8_d = pc_plus4_f + 4;
    wire [31:0] src_a,src_b;
    reg_file reg_file_u(.clk(clk),
                        .we3(reg_write_d),
                        .a1(ra1),
                        .a2(ra2),
                        .a3(instr_d[15:12]),
                        .wd3(wa3_w),
                        .r15(pc_plus8_d),
                        .rd1(rd1_d),
                        .rd2(rd2_d));



    wire no_write_d;
    wire shift_flag_d;
    wire swap_d;

    decoder decoder_u(.op(op),
                .funct(funct),
                .rd(rd),

                .pcs(pc_src_d),
                .reg_w(reg_write_d),
                .mem_w(mem_write_d),
                .mem_to_reg(mem_write_d),
                .alu_src(alu_src_d),
                .imm_src(imm_src_d),
                .reg_src(reg_src_d),
                .alu_control(alu_control_d),
                .flag_w(flag_write_d),
                .no_write(no_write_d),
                .shift_flag(shift_flag_d),
                .swap(swap_d));

endmodule

///////////////////////////////////////////
// execute
module execute(
    input clk,
    input reset,

    // cond input
    input pc_src_d,
    input reg_write_d,
    input mem_to_reg_d,
    
    input mem_write_d,
    input branch_d,
    input [1:0] flag_write_d,
    input [3:0] cond_d,

    // ALU input
    input [2:0] alu_control_d,
    input alu_src_d,
    
    input [1:0] imm_src_d,
    // fowarding
    input [31:0] result_w,
    input [31:0] alu_out_m,
    input [31:0] wa3_w,
    // fowarding
    input [1:0] forward_a_e,
    input [1:0] forward_b_e,
   
    input [31:0] rd1_d,
    input [31:0] rd2_d,
    //ext immdiate 
    input [31:0] ext_imm_d,


    // stall
    input flush_e,

    // cond OUTPUt
    output pc_src_e,
    output reg_write_e,
    output mem_to_reg_e,
    output mem_write_e,
    
    // ALU OUTPUt
    output [31:0] alu_result_e,
    output [31:0] write_data_e,

    // 
    output [31:0] wa3_e,
    output branch_take_e    
);

    wire [31:0] src_a_e;
    wire [31:0] src_b_e;

    // cond output
    reg pc_src_e;
    reg reg_write_e;
    reg mem_to_reg_e;
    reg mem_write_e;

    reg branch_e;
    reg flag_wirte_e;

    // 
    reg alu_src_e;
    reg ext_imm_e;
    
    reg [2:0] alu_control_e;
    reg [3:0]cond_e;

    reg [1:0] flag_write_e;

    reg rd1_e;
    reg rd2_e;

    reg flag_e;

    reg  flags;

    always @(posedge clk) begin
        if (reset || flush_e) begin
            pc_src_e <=1'b0;
            reg_write_e <=1'b0;
            mem_to_reg_e<=1'b0;
            mem_write_e<=1'b0;
            alu_control_e<=3'b0;
            branch_e<=1'b0;
            alu_src_e<=1'b0;
            flag_wirte_e<=1'b0;
            cond_e<=4'b0;
            flag_e<=2'b0;
            rd1_e<=32'b0;
            rd2_e<=32'b0;
            ext_imm_e<=32'b0;
            cond_e<=4'b0;
        end
        else begin
            pc_src_e <=pc_src_d;
            reg_write_e <=reg_write_d;
            mem_to_reg_e<=mem_to_reg_d;
            mem_write_e<=mem_write_e;
            alu_control_e<=alu_control_d;
            branch_e<=branch_d;
            alu_src_e<=alu_src_d;
            flag_wirte_e<=flag_write_d;
            cond_e<=cond_d;
            flag_e<=flags;
            rd1_e<=rd1_d;
            rd2_e<=rd2_d;
            ext_imm_e<=ext_imm_d;
        end

    end

    wire [3:0] alu_flags;


  
    wire swap;
    // alu

    wire [31:0] src_b;
    
    assign src_a_e = (forward_a_e == 2'b00) ? rd1_e :
                     (forward_a_e == 2'b01) ? result_w :
                     (forward_a_e == 2'b10) ? alu_out_m :
                     rd1_d;

    assign src_b = (forward_b_e == 2'b00) ? rd2_e :
                   (forward_b_e == 2'b01) ? result_w :
                   (forward_b_e == 2'b10) ? alu_out_m :
                   rd2_d;
   
  assign src_b_e = alu_src_e ? src_b : ext_imm_e;



    alu alu_u(
    .src_ina(src_a_e),
    .src_inb(src_b_e),
    .alu_control(alu_control_e),
    .alu_result(alu_result_e),
    .alu_flags(alu_flags),
    .swap(swap));

wire pc_src;
wire reg_write;
wire mem_write;
wire no_write;

cond_logic cond_logic_u(
    .clk(clk),
    .reset(reset),

    .pcs(pc_src_e),
    .reg_w(reg_write_e),
    .mem_w(mem_write_e),
    .flag_w(flag_write_e),
    .cond(cond_e),
    .alu_flag(alu_flags),
    //output
    .pc_src(pc_src),
    .reg_write(reg_write),
    .mem_write(mem_write),
    .no_write(no_write));

    // delay
    assign branch_take_e = branch_e;

endmodule

///////////////////////////////////////////
// memory
module mem(
    input clk,
    input reset,

    // input Condition OUT
    input pc_src_e,
    input reg_write_e,
    input mem_to_reg_e,
    input mem_write_e,

    // ALU OUT
    input [31:0] alu_result_e,

    input [31:0] write_data_e,
    input [31:0] wa3_e,

    // output 
    output reg pc_src_m,
    output reg reg_write_m,
    output reg mem_to_reg_m,
    // Memory OUT
    output reg mem_write_m,

    // ALU OUT
    output [31:0] alu_out_m,
    
    // Memory OUT
    output reg [31:0] write_data_m,

    output reg [31:0] wa3_m
);


    always @(clk) begin
        if (reset) begin
            pc_src_m<=1'b0;
            reg_write_m<=1'b0;
            mem_to_reg_m<=1'b0;
            mem_write_m<=1'b0;

            write_data_m<=32'b0;
            wa3_m<=32'b0;
        end
        else begin
            pc_src_m <= pc_src_e;
            reg_write_m <= reg_write_e;
            mem_to_reg_m <= mem_to_reg_e;
            mem_write_m <= mem_write_e;

            write_data_m <= write_data_e;
            wa3_m <= wa3_e;

        end
    end

endmodule




///////////////////////////////////////////
// Write Back
module wb(

    input clk,
    input reset,

    input pc_src_m,
    input reg_write_m,
    input mem_to_reg_m,

    input [31:0] rd_m,
    input [31:0] alu_out_m,
    input [31:0] wa3_m,

    output reg pc_src_w,
    output reg reg_write_w,
  
    output [31:0] result_w,
    output reg [31:0] wa3_w

);

reg read_draw_w;
reg alu_out_w;
reg mem_to_reg_w;

    always @(clk) begin
        if (reset) begin
            pc_src_w<=1'b0;
            reg_write_w<=1'b0;
            mem_to_reg_w<=1'b0;

            read_draw_w<=0;
            alu_out_w<=32'b0;

            wa3_w <=32'b0;
        end
        else begin
            pc_src_w<=pc_src_m;
            reg_write_w<=reg_write_m;
            mem_to_reg_w<=mem_to_reg_m;

            read_draw_w<=rd_m;
            alu_out_w<=alu_out_m;

            wa3_w <=wa3_m;
        end
    end

    assign result_w = mem_to_reg_w ? read_draw_w : alu_out_w;
endmodule

////////////////////////
// Data path
module data_path (
    input clk,
    input reset,
   
    //  instruction memory input
    input instr,
     //  data memory  input
    input [31:0] read_data,

    // instruction memory output
    output pc,

    // data memory output 
    output addr_data,
    output write_data,
    // data memory werite enable output
    output we
    );



    wire [31:0] result_w;

    //alu 
    wire [31:0] alu_result;

    wire pc_src_w;

    wire [31:0] instr_f;

    wire [31:0] pc_f;
    wire [31:0] pc_plus4_f;


    // instruction memory IN OUT
    assign pc = pc_f;

    // deley
    wire branch_take_e;

    fetch fetch_u(
     .clk(clk),
     .reset(reset), 
     
    // inpt ALU result
    .result_w(result_w),
    // input flag
     .pc_src_w(pc_src_w),
     // input insturuction
     .instr(instr),

     // stall
     .stall_f(ldr_stall | pc_write_pending_f),

      // deley
     .branch_take_e(branch_take_e),
     .alu_result_e(alu_result_e),

    // output instuction  Memory
     .instr_f(instr_f),
    // output program counter
      .pc_f(pc_f),
    // output R15
      .pc_plus4_f(pc_plus4_f)
    );

    wire pc_src_d;
    wire reg_write_d;
    wire mem_to_reg_d;
    wire mem_write_d;

    wire [2:0] alu_control_d;
    wire branch_d;
    wire [1:0] flag_write_d;
    wire [1:0] imm_src_d;
    wire [1:0] reg_src_d;
    wire [3:0] cond_d;
    wire [31:0] instr_d;
    wire [31:0] rd1_d;
    wire [31:0] rd2_d;
    wire [31:0] ext_imm_d;

    wire [31:0] wa3_w;


    wire reg_write_w;

    wire alu_src_d;


    ////////////////////////////////
    // Decode
    decode decode_u(
        .clk(clk),
        .reset(reset),

        // input instructin 
         .instr_f(instr_f),
        // input for R15
        .pc_plus4_f(pc_plus4_f),
        // input reg writer 
        .we3(reg_write_w),
        // input for A3
        .wa3_w(wa3_w),

        // stall
        .stall_d(ldr_stall),
        .flush_d(pc_write_pending_f | pc_src_w | branch_take_e),

        // control output
        .pc_src_d(pc_src_d),
        .reg_write_d(reg_write_d),
        .mem_to_reg_d(mem_to_reg_d),
        .mem_write_d(mem_write_d),
        .alu_control_d(alu_control_d),
        .branch_d(branch_d),
        .alu_src_d(alu_src_d),
        .flag_write_d(flag_write_d),
        .imm_src_d(imm_src_d),
        .reg_src_d(reg_src_d),

        .cond_d(cond_d),

        // alu output
        .rd1_d(rd1_d),
        .rd2_d(rd2_d),

        // output imm
        .ext_imm_d(ext_imm_d)    
    );


     wire pc_src_e;
     wire reg_write_e;
     wire mem_to_reg_e;
     wire mem_write_e;
  
     wire [31:0] alu_result_e;
     wire [31:0] write_data_e;
     wire [31:0] wa3_e;

     wire [1:0] forward_a_e;
     wire [1:0] forward_b_e;

    ////////////////////////////////
    // Execute
    execute execute_u(
        .clk(clk),
        .reset(reset),

        // cond input
        .pc_src_d(pc_src_d),
        .reg_write_d(reg_write_d),
        .mem_to_reg_d(mem_to_reg_d),
        
        .mem_write_d(mem_write_d),
        .branch_d(branch_d),
        .flag_write_d(flag_write_d),
        .cond_d(cond_d),
        //input flag_d,

        // ALU input
        .alu_control_d(alu_control_d),
        .alu_src_d(alu_src_d),
        
        .imm_src_d(imm_src_d),


        // fowarding
        .result_w(result_w),
        .alu_out_m(alu_out_m),
        .wa3_w(wa3_w),
        // fowarding
        .forward_a_e(forward_a_e),
        .forward_b_e(forward_b_e),
  

        .rd1_d(rd1_d),
        .rd2_d(rd2_d),
        //ext immdiate 
        .ext_imm_d(ext_imm_d),

        // stall
        .flush_e(ldr_stall | branch_take_e),

        // cond OUTPUT
        .pc_src_e(pc_src_e),
        .reg_write_e(reg_write_e),
        .mem_to_reg_e(mem_to_reg_e),
        .mem_write_e(mem_write_e),

        
        // ALU OUTPUt
        .alu_result_e(alu_result_e),
        .write_data_e(write_data_e),

        // 
        .wa3_e(wa3_e),
        .branch_take_e(branch_take_e)
    );
                 //rd,
                 //shift_flag,
                 //swap
     wire pc_src_m;
     wire reg_write_m;
     wire mem_to_reg_m;
    // Memory OUT
     wire  mem_write_m;
     wire [31:0] read_data_m;
    // ALU OUT
     wire [31:0] alu_out_m;
     wire [31:0] write_data_m;
     wire [31:0] wa3_m;

    ////////////////////////////////
    // Memory 
    mem mem_u(
        .clk(clk),
        .reset(reset),

        // input Condition OUT
        .pc_src_e(pc_src_e),
        .reg_write_e(reg_write_e),
        .mem_to_reg_e(mem_to_reg_e),
        .mem_write_e(mem_write_e),

        // ALU OUT
        .alu_result_e(alu_result_e),

        .write_data_e(write_data_e),
        .wa3_e(wa3_e),

        // output condition 
        .pc_src_m(pc_src_m),
        .reg_write_m(reg_write_m),
        .mem_to_reg_m(mem_to_reg_m),
        .mem_write_m(mem_write_m),

        // Memory OUT
        // ALU OUT
        .alu_out_m(alu_out_m),
        
        // Memory OUT 
        // Memory OUT
        .write_data_m(write_data_m),
        .wa3_m(wa3_m)
    );

    assign we = mem_write_m;


    // data  memory IN OUT
    assign addr_data = alu_out_m;
    assign write_data = write_data_m;


    ////////////////////////////////
    // Write Back 
    wb wb_u(
        .clk(clk),
        .reset(reset),

        .pc_src_m(pc_src_m),
        .reg_write_m(reg_write_m),
        .mem_to_reg_m(mem_to_reg_m),

        .rd_m(read_data),
        .alu_out_m(alu_out_m),
        .wa3_m(wa3_m),

        .pc_src_w(pc_src_w),
        .reg_write_w(reg_write_w),
    
        .result_w(result_w),
        .wa3_w(wa3_w)
    );



    // fowarding
    assign forward_a_e = ((wa3_m == rd1_d) & reg_write_m) ? 2'b10:
                  ((wa3_w == rd1_d) & reg_write_w) ? 2'b01:
                                                 2'b00;

    assign forward_b_e = (wa3_m == rd2_d) & (reg_write_m) ? 2'b10:
                         (wa3_w == rd2_d) & (reg_write_w) ? 2'b01:
                                                            2'b00;                                  

    // stall
    wire stall_d;
    wire stall_f;
    wire flush_e;

    wire ldr_stall;
    assign ldr_stall = (rd1_d == wa3_e) | (rd2_d == wa3_e) & mem_to_reg_e;

    // delay 
    wire pc_write_pending_f;
    assign pc_write_pending_f = pc_src_d | pc_src_e | pc_src_m;


endmodule

