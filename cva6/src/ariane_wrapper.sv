
`include "config.vh"

//import ariane_pkg::*;
//import ariane_axi::*;

module ariane_wrapper
`ifdef FV_DUT_INSERT_FV
    #(parameter RAM_DATA_WIDTH = `FV_DUT_MEM_DATA_WIDTH,
`else
    #(parameter RAM_DATA_WIDTH = 128,
`endif
`ifdef FV_DUT_INSERT_FV
      parameter BOOT_ADDR = `FV_DUT_INSTR_ADDRESS_BASE,
`else
      parameter BOOT_ADDR = 'h80,
`endif
      parameter PULP_SECURE = 1)
    (input logic clk_i,
     input logic  rst_ni
     );

    // signals connecting core to memory
    ariane_axi::req_t             core_axi_req,  mem_axi_req;
    ariane_axi::resp_t            core_axi_resp, mem_axi_resp;

    // instantiate the core
    ariane
// non-default parameters to be added
    ariane_core_i
        (
         .clk_i       (clk_i),
         .rst_ni      (rst_ni),
	 .boot_addr_i (BOOT_ADDR),  // reset boot address
	 .hart_id_i   (64'h0),  // hart id in a multicore environment (reflected in a CSR)
	 .irq_i       (2'b0),   // level sensitive IR lines, mip & sip (async)
	 .ipi_i       (1'b0),   // inter-processor interrupts (async)
	 .time_irq_i  (1'b0),   // timer interrupt in (async)
	 .debug_req_i (1'b0),   // debug request (async)
	 .axi_req_o   (core_axi_req),
	 .axi_resp_i  (core_axi_resp)
	 );

    localparam AxiNumWords = (ICACHE_LINE_WIDTH/64) * (ICACHE_LINE_WIDTH  > DCACHE_LINE_WIDTH)  +
                             (DCACHE_LINE_WIDTH/64) * (ICACHE_LINE_WIDTH <= DCACHE_LINE_WIDTH) ;
    
   fv_ariane_axi_atomics
      #(
	.AXI_ADDR_WIDTH(64),
	.AXI_DATA_WIDTH(RAM_DATA_WIDTH),
	.AXI_ID_WIDTH($size(core_axi_resp.r.id)),
	.AXI_USER_WIDTH($size(core_axi_resp.r.user)),
	.AXI_MAX_WRITE_TXNS(4), // Note: correct?
	.RISCV_WORD_WIDTH(64)
	)
    ariane_axi_atomics_i
      (
       .clk_i        (clk_i),
       .rst_ni       (rst_ni),
       .slv_i        (core_axi_req),
       .slv_o        (core_axi_resp),
       .mst_o        (mem_axi_req),
       .mst_i        (mem_axi_resp)
       );

    fv_ariane_axi_memory 
        #(
	  .RAM_DATA_WIDTH (RAM_DATA_WIDTH),
	  .AxiNumWords    (AxiNumWords),
	  .AxiIdWidth     ($size(mem_axi_resp.r.id)),
	  .BOOT_ADDR      (BOOT_ADDR),
	  .MEM_SIZE       (`DMEM_SIZE)
	  )
    ariane_axi_mem_i 
        (
	 .clk_i        (clk_i),
         .rst_ni       (rst_ni),
	 .axi_req_in   (mem_axi_req),
	 .axi_resp_out (mem_axi_resp)
	 );

`ifdef FV_DUT_INSERT_FV
      fv fv();
`endif
      
endmodule // ariane_wrapper
