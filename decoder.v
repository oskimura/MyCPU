`default_nettype none
module decoder(op,
               funct,
               rd,
               pcs,
               reg_w,
               mem_w,
               mem_to_reg,
               alu_src,
               imm_src,
               reg_src,
               alu_control,
               flag_w,
               no_write,
               shift_flag,
               swap);

    input [1:0] op;
    input [5:0] funct;
    input [3:0] rd;
    output pcs,reg_w,mem_w,mem_to_reg,alu_src;
    output [1:0] imm_src,reg_src,flag_w;
            
    output reg [2:0] alu_control;
    output no_write;
    output shift_flag;
    output swap;

    reg [9:0] control;
    wire branch,alu_op;

    always @(*) begin
        case (op)
            2'd0:
                // dp reb
                if (funct[5]) begin
                    control <= 10'b0001001x01;
                end
                // dp imm
                else begin
                    control <= 10'b0000xx1001; 
                end 
            2'd1:
                // ldr
                if (funct[0]) begin
                    control <= 10'b0101011x00;                   
                // str
                end else begin
                    control <= 10'b0x11010100;
                end
            2'd2:
                // b
                control <= 10'b1001100x10;
        endcase
    end

    assign {branch,mem_to_reg,mem_w,alu_src,imm_src, reg_w,reg_src,alu_op} = control;
    wire [3:0] cmd;
    assign cmd = funct[4:1];
    always @(*) begin
        if (alu_op) begin
            
            case (cmd) 
                //add
                4'b0100: alu_control <= 3'b000;
                //sub
                4'b0010: alu_control <= 3'b001;
                //and
                4'b0000: alu_control <= 3'b010;
                //or
                4'b1100: alu_control <= 3'b011;
                // cmp
                4'b1010: alu_control <= 3'b001;
                // tst
                4'b1000: alu_control <= 3'b010;
                // lsl
                4'b1101: alu_control <= 3'b0xx;
                // cmn
                4'b1011: alu_control <= 3'b000;
                // adc
                4'b0101: alu_control <= 3'b100;
                // eor
                4'b0001: alu_control <=  3'b111;
                // lsr
                4'b0001: alu_control <=  3'b0xx;
                // teq
                4'b1001: alu_control <= 3'b111;
                // rsb
                4'b0011: alu_control <= 3'b001;
            endcase
        
        end 
        else begin
            alu_control <= 2'b00;
        end
    end

    assign flag_w[1] = alu_op & funct[0];
    assign flag_w[0] = alu_op & funct[0] & 
                            (alu_control == 3'b00 || 
                             alu_control == 3'b01);
    assign no_write = (cmd == 4'b1010 || 
                       cmd == 4'b1011 || 
                       cmd == 4'b1001 ||
                       cmd == 4'b111) && alu_op 
                      ? 1'b1 : 1'b0;
    assign shift_flag = (cmd==4'b1101 )? 1'b1 : 1'b0;

    assign pcs = ((rd==4'd15)&reg_w)|branch;
    assign swap = (cmd==4'b0011)?1:0;
endmodule
`define Fetch 5'd0
`define Decode 5'd1
`define MemAddr 5'd2
`define ExecuteR 5'd3
`define ExecuteL 5'd4
`define Branch 5'd5
`define MemRead 5'd6
`define MemWrite 5'd7
`define AluWB 5'd8
`define MemWB 5'd9

module fsm(clk,
reset,
op,
funct,

reg_w,
mem_w,
ir_write,
next_pc,
adr_src,
result_src,
alu_src_a,
alu_src_b,
branch,
alu_op);

    input clk, reset;
    input [1:0] op;
    input [5:0] funct;
           

    output reg reg_w,
    mem_w,
    ir_write,
    next_pc,
    adr_src,
    
    alu_src_a,
    branch,
    alu_op;

    output reg [1:0] result_src,reg_src,
    alu_src_b;

    reg [4:0] state,next_state;
    reg cotrol;

    always @(clk or reset) begin
        if (reset) state <= `Fetch;
        else state <= next_state;  
    end
   
    always @(clk) begin
        case (state)
            `Fetch:
                begin
                    adr_src<=1'b0;
                    alu_src_a=1'b1;
                    alu_src_b<=2'b10;
                    alu_op<=1'b0;
                    result_src<=2'b10;
                    ir_write<=1'b1;
                    next_pc<=1'b1;

                    next_state <= `Decode;
                end
            `Decode:
                begin
                    alu_src_a <= 1'b1;
                    alu_src_b <= 2'b10;
                    alu_op<=1'b0;
                    result_src<=2'b10;

                    if (op==2'b01) next_state <= `MemAddr;
                    else if (op==2'b00 && funct[5]==1'b0) next_state <= `ExecuteR;
                    else if (op==2'b00 && funct[5]==1'b1) next_state <= `ExecuteL;
                    else if (op==2'b10) next_state <= `Branch;
                end
            `MemAddr:
                begin
                
                    alu_src_a <= 1'b0;
                    alu_src_b <= 2'b01;
                    alu_op <= 1'b0;

                    if (funct[0]==1'b1) next_state <= `MemRead;
                    else if (funct[0]==1'b0) next_state <= `MemWrite;
                    
                end
            `ExecuteR:
               begin
                    alu_src_a<=1'b0;
                    alu_src_b<=2'b00;
                    alu_op<=1'b1;

                    next_state = `AluWB;
                end
            `ExecuteL:
                begin
                    alu_src_a<=1'b0;
                    alu_src_b<=2'b01;
                    alu_op<=1'b1;

                    next_state <= `AluWB;
                end
            `Branch:
                begin
                    alu_src_a <= 1'b0;
                    alu_src_b <= 2'b01;
                    result_src <= 2'b10;
                    branch <= 1'b1;

                    next_state <= `Fetch;
                end
            `MemRead:
                begin
                    result_src <= 2'b00;
                    adr_src <= 1'b1;

                    next_state <= `MemWB;
                end
            `MemWrite:
                begin
                    reg_src<=2'b00;
                    adr_src<=1'b1;
                    mem_w<=1'b1;

                    next_state <= `Fetch;
                end
            `AluWB:
                begin
                    result_src <= 2'b00;
                    reg_w <= 1'b1;

                    next_state <= `Fetch;
                end
            `MemWB:
                begin
                    result_src <= 2'b01;
                    reg_w<=1'b1;

                    next_state <= `Fetch;
                end
            default:
                next_state <= `Fetch;
        endcase
    end


    //assign alu_src_a = () 1;
    //assign {reg_w,mem_w,ir_write,next_pc,adr_src,result_src,alu_src_a,alu_src_b} = control;

