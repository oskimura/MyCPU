`default_nettype none
module decoder(op,funct,rd,
              pcs,reg_w,mem_w,mem_to_reg,alu_src,imm_src,reg_src,alu_control,flag_w);
    input [1:0] op;
    input [5:0] funct;
    input [3:0] rd;
    output pcs,reg_w,mem_w,mem_to_reg,alu_src;
    output [1:0] imm_src,reg_src,flag_w;
    output reg [1:0] alu_control;
            
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
                // str
                if (funct[0]) begin
                    control <= 10'b0x11010100;
                // ldr
                end else begin
                    control <= 10'b0101011x00;
                end
            2'd2:
                // b
                control <= 10'b1001100x10;
        endcase
    end

    assign {branch,mem_to_reg,mem_w,alu_src,imm_src, reg_w,reg_src,alu_op} = control;
    always @(*) begin
        if (alu_op) begin
            
            case (cmd) 
                //add
                4'b0100: alu_control <= 2'b00;
                //sub
                4'b0010: alu_control <= 2'b01;
                //and
                4'b0000: alu_control <= 2'b10;
                //or
                4'b1100: alu_control <= 2'b11;
                // cmp
                4'b1010: alu_control <= 2'b01;
            endcase
        
        end 
        else begin
            alu_control <= 2'b00;
        end
    end

    assign flag_w[1] = alu_op & funct[0];
    assign flag_w[0] = alu_op & funct[0] & (alu_control == 2'b00 || alu_control == 2'b01);

    assign pcs = ((rd==4'd15)&reg_w)|branch;
endmodule