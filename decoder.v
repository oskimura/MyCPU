`default_nettype none
module decoder(op,funct,rd,
              pcs,reg_w,mem_w,mem_to_reg,alu_src,imm_src,reg_src,alu_control,flag_w,no_write,shift_flag);
    input [1:0] op;
    input [5:0] funct;
    input [3:0] rd;
    output pcs,reg_w,mem_w,mem_to_reg,alu_src;
    output [1:0] imm_src,reg_src,flag_w;
            
    output reg [2:0] alu_control;
    output no_write;
    output shift_flag;

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
            endcase
        
        end 
        else begin
            alu_control <= 2'b00;
        end
    end

    assign flag_w[1] = alu_op & funct[0];
    assign flag_w[0] = alu_op & funct[0] & (alu_control == 2'b00 || alu_control == 2'b01);
    assign no_write = (cmd == 4'b1010 || cmd == 4'b1011 || cmd == 4'b100) && alu_op 
                      ? 1'b1 : 1'b0;
    assign shift_flag = (cmd==4'b1101 )? 1'b1 : 1'b0;

    assign pcs = ((rd==4'd15)&reg_w)|branch;
endmodule