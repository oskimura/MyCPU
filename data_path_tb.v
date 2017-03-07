module data_path_tb;

reg              clk,
                 reset;

    reg [31:0] read_data;
    reg [31:0] instr;         


    wire [31:0] pc;
    // data memory output 
    wire [31:0] addr_data;
    wire [31:0] write_data;
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

                  .wd(wd));
initial begin
    #1 reset=0; 

         
     #10 reset = 1;
     #50 reset = 0;
     
     read_data = 0;
    // // mov
    //instr=32'b0000000110100000001100000000001;
   
    // mov r3 2
    instr=32'b00000011101000000011000000000010;
    // add r3 r3 
    #50 instr=32'b00000010100000110011000000000001;
end

endmodule