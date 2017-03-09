module data_path_tb;

reg              clk,
                 reset;

    
    reg [31:0] instr;         


    wire [31:0] pc;
    // data memory output 
    wire [31:0] addr_data;
    wire [31:0] write_data;
    // data memory input
    wire [31:0] read_data;
    // data memory werite enable output
    wire  we;




initial begin
        $dumpfile("wave.vcd");
        $dumpvars(0, data_path_u);
    
    $monitor("instr=%b,read_data=%b,pc=%b,addr_data=%b,write_data=%b,we=%b",instr,read_data,pc,addr_data,write_data,we);


end



    initial begin
        clk = 1;
        forever begin 
            #5 clk <= ~clk;
            end
    end

    initial begin
        repeat (1000) @(clk);
         $finish;
    end


dmem dmem_u( .clk(clk),
             .we(we),
             .addr(addr_data),
             .write_data(write_data),
             .read_data(read_data));

data_path data_path_u(
     .clk(clk),
     .reset(reset),
   
    //  instruction memory input
     .instr(instr),
     //  data memory  input
     .read_data(read_data),

    // instruction memory output
     .pc(pc),

    // data memory output 
     .addr_data(addr_data),
     .write_data(write_data),
    // data memory werite enable output
     .we(we)
    );
//                  .wd(wd));


initial begin
    #1 reset=0; 

         
     #10 reset = 1;
     #50 reset = 0;
     
    // // mov
    //instr=32'b0000000110100000001100000000001;
   
    // mov r3 2
        instr=32'b00000011101000000011000000000010;
    // add r3 r3 #1
    #50 instr=32'b00000010100000110011000000000001;
    // sub r4 r3 #1
    #50 instr=32'b00000010010000110100000000000001;
    // add r3 r3 #1
    #50 instr=32'b00000000100000110011000000000011;
    // str r3 [r5] #-26
    //#50 instr=32'b1110 01 000000 0101 0011 0000 00011010
      #50 instr=32'b11100100000001010011000000011010; 
    // ldr r3 [r5] #-26
    //#50 instr=32'b1110 01 000001 0101 0011 0000 00011010
      #50 instr=32'b11100100000101010011000000011010;

    // b 1
    //#50 instr=32'b1110 10 00000000000000000000000000
      #50 instr=32'b11101000000000000000000000000000;
      
    // bl 
 
    //#50 instr=32'b1110 10 01000000000000000000100000
      #50 instr=32'b11101001000000000000001000000000;
         $display("bl 32");
    // bx rm
    // bx r14

    //#50 instr=32'b000000111010 1110 1111 0000 0000 0010;                         
      #50 instr=32'b00000001001000000000000000001110;
          $display("bx r14");
end

endmodule


module dmem(input clk,
            input we,
            input [31:0] addr,
            input [31:0] write_data,
            output [31:0] read_data);

    reg [31:0] ram[63:0];
    assign read_data = ram[addr[31:2]];

    always @(posedge clk) begin
        if (we)
            ram[addr[31:2]] <= write_data;
    end
endmodule