module serial_select(
    input [1:0] sel,
    output [2:0] csn
    );

   // no outputs active when sel == 2'b00
   assign csn[0] = ~(sel == 2'b01);
   assign csn[1] = ~(sel == 2'b10);
   assign csn[2] = ~(sel == 2'b11);

endmodule
