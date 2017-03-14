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
    reg irq=1'b0;
    reg firq=1'b0;




initial begin
        $dumpfile("wave.vcd");
        $dumpvars(0, data_path_u);
    
    $monitor("instr=%b,read_data=%b,pc=%x,addr_data=%b,write_data=%b,we=%b",instr,read_data,pc,addr_data,write_data,we);


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

     .irq(irq),
     .firq(firq),

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
     #10 reset = 0;
     
    // // mov
        //instr=32'b0000000110100000001100000000001;
   
    // mov r3 2
        instr=32'b00000011101000000011000000000010;
    $display("mov r6 #2");
    // add r3 r3 #1
    #10 instr=32'b00000010100000110011000000000001;
    $display("sub r4 r3 #1");
    // sub r4 r3 #1
    #10 instr=32'b00000010010000110100000000000001;
    $display("add r4 r3 #1");
    // add r3 r3 #1
    #10 instr=32'b00000000100000110011000000000011;
    $display("add r3 r3 #1");
    // str r3 [r5] #-26
    //#50 instr=32'b1110 01 000000 0101 0011 0000 00011010
    #10 instr=32'b11100100000001010011000000011010; 
    $display("str r3 [r5] #-26");

    // ldr r3 [r5] #-26
    //#50 instr=32'b1110 01 000001 0101 0011 0000 00011010
    #10 instr=32'b11100100000101010011000000011010;
    $display("ldr r3 [r5] #-26");

    // b 1
    //#50 instr=32'b1110 10 00000000000000000000000000
    #10 instr=32'b11101000000000000000000100000000;

    $display("b 100000000");
    // mov r3 2
    #10 instr=32'b00000011101000000011000000000010;
    $display("mov r6 2");
    
    // bl 
    //#50 instr=32'b1110 10 01000000000000000000100000
    #10 instr=32'b11101011000000000000001000000000;
    $display("bl 1000000");
    #10 instr=32'b00000011101000000011000000000010;
    $display("mov r6 2");
         
    // bx rm
    // bx r14

    //#50 instr=32'b000000 111010 1110 1111 0000 0000 0010;                         
                
    #10 instr=32'b00000001001000000000000000011110;
    $display("bx r14");
    #10 instr=32'b00000011101000000011000000000010;
    #10 $display("mov r6 2");
    // swi
    #10 instr=32'b00001111000000000000000000000000;
    $display("swi");
    #10 instr=32'b00000011101000000011000000000010;
    #10 $display("mov r6 2");

    //adc
    //add
    //and 
    //b 
    //bic
    //cdp
    //cmn
    //eor
    //ldc
    //ldm
    //ldr
    //ldrb
    //mcr
    //#10 instr=32'b 0000 1110 000 0 1111 0010 1111 000 1 0000;
    #10 instr=32'b00001110000011110010111100010000;


    //mlar
    //mov
    //mrc

    //mul
    //mvn
    //orr
    //rsb
    //rsc
    //sbc
    //stc
    //stm
    //str
    //strb
    //swi
    //swp
    //swpb
    //teq
    //tst


  

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