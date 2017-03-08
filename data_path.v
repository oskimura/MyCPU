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

module reg_file(clk,
                reset,
                we3,
                // Rn
                a1,
                // Rd/Rm
                a2,
                // Rd
                a3,
                // write data
                wd3,
                r15,
                // Rn
                rd1,
                // Rd/Rm
                rd2,
                // Rd
                rd3);
    input clk,reset,we3;
    input [3:0] a1,a2,a3;
    input [31:0] wd3,r15;
    output [31:0] rd1,rd2,rd3;

    reg [31:0] r[14:0];

    always @(posedge clk) begin
      if (we3) r[a3] <= wd3;
      else if (reset) begin
        r[0] <= 0;
        r[1] <= 0;
        r[2] <= 0;
        r[3] <= 0;
        r[4] <= 0;
        r[5] <= 0;
        r[6] <= 0;
        r[7] <= 0;
        r[8] <= 0;
        r[9] <= 0;
        r[10] <= 0;
        r[11] <= 0;
        r[12] <= 0;
        r[13] <= 0;
        r[14] <= 0;
      end
    end

    assign rd1 = (a1==4'b1111)? r15 : r[a1];
    assign rd2 = (a2==4'b1111)? r15 : r[a2];

    assign rd3 = (a3==4'b1111)? r15 : r[a3];

endmodule


///////////////////////////////////////////
// shift
module shift(alu_src, instr, rd2, rd3, imm, shift_result);

    input  alu_src;
    input [11:0] instr;
    
    // Rd/Rm
    input [31:0] rd2;
    // Rs
    input [31:0] rd3;
    input [31:0] imm;
    output reg [31:0] shift_result;

    wire [1:0] sh;
    //wire [3:0] rm;
    wire [5:0] shamt;
    wire flag;

    assign {shamt,sh,flag} = instr[11:4];

    wire [31:0] src;

    assign src = alu_src? imm :
                 instr[4]? rd3 :
                 shamt;

    wire [31:0] ror_out;

    wire carry=1'b0;


    always @(*) begin
        if (alu_src==1'b1) begin
            if (instr[11:4]==8'b0) begin
            // mov
                shift_result <= src;
            end
        end 
        else
            case (sh)
                // mov
                    //shift_result <= instr;
                //lsl
                2'b00: if (instr[11:4]!=0) 
                        shift_result <= rd2 << src;
                // lsr
                2'b01: shift_result <= rd2 >> src;
                // asr
                2'b10:
                    shift_result <= rd2 >> src;
                2'b11:
                    // rrx
                    if (instr[11:4]==0) begin
                        shift_result <= ror_out;
                    // ror
                    end 
                    else begin
                        shift_result <= ror_out;
                    end 
             endcase 
    end 
    

// ror


assign ror_out = 
                 src[7:0] == 8'd 0  ? {carry, rd2                  } :  // fall through case
                 
                 src[4:0] == 5'd 0  ? {rd2[31], rd2                    } :  // Rs > 31
                 src[4:0] == 5'd 1  ? {rd2[ 0], rd2[    0], rd2[31: 1]} :
                 src[4:0] == 5'd 2  ? {rd2[ 1], rd2[ 1: 0], rd2[31: 2]} :
                 src[4:0] == 5'd 3  ? {rd2[ 2], rd2[ 2: 0], rd2[31: 3]} :
                 src[4:0] == 5'd 4  ? {rd2[ 3], rd2[ 3: 0], rd2[31: 4]} :
                 src[4:0] == 5'd 5  ? {rd2[ 4], rd2[ 4: 0], rd2[31: 5]} :
                 src[4:0] == 5'd 6  ? {rd2[ 5], rd2[ 5: 0], rd2[31: 6]} :
                 src[4:0] == 5'd 7  ? {rd2[ 6], rd2[ 6: 0], rd2[31: 7]} :
                 src[4:0] == 5'd 8  ? {rd2[ 7], rd2[ 7: 0], rd2[31: 8]} :
                 src[4:0] == 5'd 9  ? {rd2[ 8], rd2[ 8: 0], rd2[31: 9]} :
                    
                 src[4:0] == 5'd10  ? {rd2[ 9], rd2[ 9: 0], rd2[31:10]} :
                 src[4:0] == 5'd11  ? {rd2[10], rd2[10: 0], rd2[31:11]} :
                 src[4:0] == 5'd12  ? {rd2[11], rd2[11: 0], rd2[31:12]} :
                 src[4:0] == 5'd13  ? {rd2[12], rd2[12: 0], rd2[31:13]} :
                 src[4:0] == 5'd14  ? {rd2[13], rd2[13: 0], rd2[31:14]} :
                 src[4:0] == 5'd15  ? {rd2[14], rd2[14: 0], rd2[31:15]} :
                 src[4:0] == 5'd16  ? {rd2[15], rd2[15: 0], rd2[31:16]} :
                 src[4:0] == 5'd17  ? {rd2[16], rd2[16: 0], rd2[31:17]} :
                 src[4:0] == 5'd18  ? {rd2[17], rd2[17: 0], rd2[31:18]} :
                 src[4:0] == 5'd19  ? {rd2[18], rd2[18: 0], rd2[31:19]} :

                 src[4:0] == 5'd20  ? {rd2[19], rd2[19: 0], rd2[31:20]} :
                 src[4:0] == 5'd21  ? {rd2[20], rd2[20: 0], rd2[31:21]} :
                 src[4:0] == 5'd22  ? {rd2[21], rd2[21: 0], rd2[31:22]} :
                 src[4:0] == 5'd23  ? {rd2[22], rd2[22: 0], rd2[31:23]} :
                 src[4:0] == 5'd24  ? {rd2[23], rd2[23: 0], rd2[31:24]} :
                 src[4:0] == 5'd25  ? {rd2[24], rd2[24: 0], rd2[31:25]} :
                 src[4:0] == 5'd26  ? {rd2[25], rd2[25: 0], rd2[31:26]} :
                 src[4:0] == 5'd27  ? {rd2[26], rd2[26: 0], rd2[31:27]} :
                 src[4:0] == 5'd28  ? {rd2[27], rd2[27: 0], rd2[31:28]} :
                 src[4:0] == 5'd29  ? {rd2[28], rd2[28: 0], rd2[31:29]} :

                 src[4:0] == 5'd30  ? {rd2[29], rd2[29: 0], rd2[31:30]} :
                                                 {rd2[30], rd2[30: 0], rd2[31:31]} ;

endmodule

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
    input [31:0] instr,

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
   assign pc_plus4_f = pc_f + 4;


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
    input [3:0] wa3_w,

    // stall
    input stall_d,
    input flush_d,
    
    input [31:0] alu_out_m,

    output [3:0] ra1_d,
    output [3:0] ra2_d,

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
    output [31:0] rd1_d,
    output [31:0] rd2_d,
    output swap_d,

    // imm
    output [31:0] ext_imm_d,
    // shift 
    output shift_flag_d,
    output [31:0] shift_result_d,

    output [3:0] wa3_d
);

    reg [31:0] instr_d;
    always @(posedge clk) begin
        if (reset) 
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
    wire [3:0] ra3;

    assign cond_d = instr_d[31:28];
    assign op = instr_d[27:26];
    assign funct = instr_d[25:20];

    assign wa3_d = instr_d[15:12];

    // input regfile 
    assign rd = instr_d[15:12];
    // Rn
    assign ra1 = (reg_src_d[0]) ? 4'd15 :instr_d[19:16];
    // Rd/Rm
    assign ra2 = (reg_src_d[1]) ? instr_d[15:12] : instr_d[3:0];
    // Rs
    assign ra3 = instr_d[11:8];
    wire [31:0] rd3;


    // ext imm
   wire [31:0] ext_imm;
   extend extend_u(.instr(instr_d[23:0]),.imm_src(imm_src_d),
              .ext_imm(ext_imm_d));

    assign ra1_d = ra1;
    assign ra2_d = ra2;
    // reg file
    wire [31:0] pc_plus8_d;
    assign pc_plus8_d = pc_plus4_f + 4;
    wire [31:0] src_a,src_b;
    reg_file reg_file_u(.clk(clk),
                        .reset(reset),
                        .we3(reg_write_d),
                        // Rn
                        .a1(ra1),
                        // Rd/Rm
                        .a2(ra2),
                        // Rd
                        .a3(wa3_w),

                        .wd3(alu_out_m),
                        .r15(pc_plus8_d),
                        // Rn
                        .rd1(rd1_d),
                        // Rd/Rm
                        .rd2(rd2_d),
                        // Rd
                        .rd3(rd3));



    wire no_write_d;
    //wire shift_flag_d;
    //wire swap_d;

    wire [31:0] shift_result;

    //rd1, rd2, imm
    shift shift_u(.alu_src(alu_src_d), 
                .instr(instr_d[11:0]), 
                .rd2(rd1_d), 
                .rd3(rd3), 
                .imm(ext_imm_d), 
                .shift_result(shift_result_d));


    decoder decoder_u(.op(op),
                .funct(funct),
                .rd(rd),

                .pcs(pc_src_d),
                .reg_w(reg_write_d),
                .mem_w(mem_write_d),
                .mem_to_reg(mem_to_reg_d),
                .alu_src(alu_src_d),
                .imm_src(imm_src_d),
                .reg_src(reg_src_d),
                .alu_control(alu_control_d),
                .flag_w(flag_write_d),
                .no_write(no_write_d),
                .shift_flag(shift_flag_d),
                .swap(swap_d),
                .branch(branch_d));

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
    input swap_d,
    input alu_src_d,
    
    input [1:0] imm_src_d,
    // fowarding
    input [31:0] result_w,
    input [31:0] alu_out_m,
    input [3:0] wa3_w,
    // fowarding
    input [1:0] forward_a_e,
    input [1:0] forward_b_e,
   
    input [31:0] rd1_d,
    input [31:0] rd2_d,
    //ext immdiate 
    input [31:0] ext_imm_d,

    // sfhit
    input shift_flag_d,
    input [31:0] shift_result_d,

    // stall
    input flush_e,

    input [3:0] wa3_d,

    // cond OUTPUt
    output pc_src_e,
    output reg_write_e,
    output mem_to_reg_e,
    output mem_write_e,
    
    // ALU OUTPUt
    output [31:0] alu_result_e,
    output [31:0] write_data_e,

    // 
    output reg [3:0] wa3_e,
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
    reg [31:0] ext_imm_e;
    
    reg [2:0] alu_control_e;
    reg [3:0] cond_e;

    reg [1:0] flag_write_e;

    reg [31:0] rd1_e;
    reg [31:0] rd2_e;

    reg flag_e;

    reg  flags;

    reg shift_flag_e;
    reg [31:0] shift_result_e;
    reg swap_e;

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
            wa3_e <= 4'b0;
            shift_flag_e <=0;
            shift_result_e <= 32'b0;
            swap_e <= 1'b0;
        end
        else begin
            pc_src_e <=pc_src_d;
            reg_write_e <=reg_write_d;
            mem_to_reg_e<=mem_to_reg_d;
            mem_write_e<=mem_write_d;
            alu_control_e<=alu_control_d;
            branch_e<=branch_d;
            alu_src_e<=alu_src_d;
            flag_wirte_e<=flag_write_d;
            cond_e<=cond_d;
            flag_e<=flags;
            rd1_e<=rd1_d;
            rd2_e<=rd2_d;
            ext_imm_e<=ext_imm_d;
            wa3_e <= wa3_d;
            shift_flag_e <= shift_flag_d;
            shift_result_e <= shift_result_d;
            swap_e <= swap_d;
        end

    end

    wire [3:0] alu_flags;

    //wire swap;
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
   assign write_data_e = src_b;
   assign src_b_e = alu_src_e ? ext_imm_e : src_b;

    wire [31:0] alu_result;
    alu alu_u(
    .src_ina(src_a_e),
    .src_inb(src_b_e),
    .alu_control(alu_control_e),
    .alu_result(alu_result),
    .alu_flags(alu_flags),
    .swap(swap_e));


    assign alu_result_e = shift_flag_e ? shift_result_e : alu_result;

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
    input [3:0] wa3_e,

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

    output reg [3:0] wa3_m
);

    reg [31:0] alu_result_m;

    always @(posedge clk) begin
        if (reset) begin
            pc_src_m<=1'b0;
            reg_write_m<=1'b0;
            mem_to_reg_m<=1'b0;
            mem_write_m<=1'b0;

            write_data_m<=32'b0;
            wa3_m<=32'b0;
            alu_result_m=32'b0;
        end
        else begin
            pc_src_m <= pc_src_e;
            reg_write_m <= reg_write_e;
            mem_to_reg_m <= mem_to_reg_e;
            mem_write_m <= mem_write_e;

            write_data_m <= write_data_e;
            wa3_m <= wa3_e;
            alu_result_m <= alu_result_e;

        end
    end
    assign alu_out_m = alu_result_m;

endmodule




///////////////////////////////////////////
// Write Back
module wb(

    input clk,
    input reset,

    input pc_src_m,
    input reg_write_m,
    input mem_to_reg_m,

    // input memory
    input [31:0] rd_m,
    input [31:0] alu_out_m,
    input [3:0] wa3_m,

    output reg pc_src_w,
    output reg reg_write_w,
  
    output [31:0] result_w,
    output reg [3:0] wa3_w

);

reg read_draw_w;
reg [31:0] alu_out_w;
reg mem_to_reg_w;

    always @(posedge clk) begin
        if (reset) begin
            pc_src_w<=1'b0;
            reg_write_w<=1'b0;
            mem_to_reg_w<=1'b0;

            read_draw_w<=0;
            alu_out_w<=32'b0;

            wa3_w <=4'b0;
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
    input [31:0] instr,
     //  data memory  input
    input [31:0] read_data,

    // instruction memory output
    output [31:0] pc,

    // data memory output 
    output [31:0] addr_data,
    output [31:0] write_data,
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

    wire [3:0] wa3_w;


    wire reg_write_w;

    wire alu_src_d;

    wire [31:0] shift_result;

    wire shift_flag;

    wire [3:0] wa3_d;

    wire [3:0] ra1_d;
    wire [3:0] ra2_d;

    wire swap_d;

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
        
        .alu_out_m(alu_out_m),
        .ra1_d(ra1_d),
        .ra2_d(ra2_d),


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
        .swap_d(swap_d),

        // output imm
        .ext_imm_d(ext_imm_d),
        .shift_flag_d(shift_flag),
        .shift_result_d(shift_result),
        .wa3_d(wa3_d)
    );


     wire pc_src_e;
     wire reg_write_e;
     wire mem_to_reg_e;
     wire mem_write_e;
  
     wire [31:0] alu_result_e;
     wire [31:0] write_data_e;
     wire [3:0] wa3_e;

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
        .swap_d(swap_d),
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

            // sfhit
            .shift_flag_d(shift_flag),
            .shift_result_d(shift_result),

        // stall
        .flush_e(ldr_stall | branch_take_e),

        .wa3_d(wa3_d),

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
     wire [3:0] wa3_m;


    // data  memory IN OUT
    assign we = mem_write_m;
    assign addr_data = alu_out_m;
    assign write_data = write_data_m;

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
    assign forward_a_e = ((wa3_m == ra1_d) & reg_write_m) ? 2'b10:
                         ((wa3_w == ra1_d) & reg_write_w) ? 2'b01:
                                                            2'b00;

    assign forward_b_e = (wa3_m == ra2_d) & (reg_write_m) ? 2'b10:
                         (wa3_w == ra2_d) & (reg_write_w) ? 2'b01:
                                                            2'b00;                                  

    // stall
    wire stall_d;
    wire stall_f;
    wire flush_e;

    wire ldr_stall;
    assign ldr_stall = ((ra1_d == wa3_e) | (ra2_d == wa3_e)) & mem_to_reg_e;

    // delay 
    wire pc_write_pending_f;
    assign pc_write_pending_f = pc_src_d | pc_src_e | pc_src_m;


endmodule

