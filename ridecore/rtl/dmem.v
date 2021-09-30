`include "constants.vh"
`default_nettype none
//8KB Single PORT synchronous BRAM
module dmem
  (
   input wire 		      clk,
   input wire [`ADDR_LEN-1:0] addr,
   input wire [`DATA_LEN-1:0] wdata,
   input wire 		      we,
   output reg [`DATA_LEN-1:0] rdata
   );
 
`ifdef FV_RIDECORE_INSERT_FV
// reduce the DMEM size to speed up
   reg [`DATA_LEN-1:0] 	      mem [0:((`DMEM_SIZE/4)-1)];
`else
   reg [`DATA_LEN-1:0] 	      mem [0:2047];
`endif
   
   always @ (posedge clk) begin
      rdata <= mem[addr[10:0]];
      if (we)
	mem[addr] <= wdata;
   end
endmodule // dmem
`default_nettype wire
