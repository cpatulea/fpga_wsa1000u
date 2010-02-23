//
// TODO: Why isn't the OBUFT16 macro recognized?
//
// ERROR:HDLCompilers:87 - "wsa.v" line 117 Could not find module/primitive 'OBUFT16'
//

module xilinx_obuft16(
   input wire [15:0] I,
   output wire [15:0] O,
   input wire T
);

   genvar i;
   generate for (i = 0; i < 16; i = i + 1) begin : one_obuft
      OBUFT the_obuft(.I(I[i]), .O(O[i]), .T(T));
   end endgenerate

endmodule
