// Copyright 2018 Robert Balas <balasr@student.ethz.ch>
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

// Wrapper for a RI5CY testbench, containing RI5CY, Memory and stdout peripheral
// Contributor: Robert Balas <balasr@student.ethz.ch>

`include "config.vh"

module riscv_wrapper
`ifdef USE_FOR_ZERORISCY
    #(parameter INSTR_RDATA_WIDTH = 32,
`else
 `ifdef FV_RISCY_INSTR_RDATA_WIDTH
    #(parameter INSTR_RDATA_WIDTH = `FV_RISCY_INSTR_RDATA_WIDTH,
 `else
    #(parameter INSTR_RDATA_WIDTH = 128,
 `endif
`endif
`ifdef FV_RISCY_INSERT_FV
      parameter RAM_ADDR_WIDTH = $clog2(`DMEM_SIZE),
`else
      parameter RAM_ADDR_WIDTH = 20,
`endif
      parameter BOOT_ADDR = 'h80,
      parameter PULP_SECURE = 1)
    (input logic clk_i,
     input logic  rst_ni,

     input logic fetch_enable_i,
     output logic tests_passed_o,
     output logic tests_failed_o);

    // signals connecting core to memory
    logic                        instr_req;
    logic                        instr_gnt;
    logic                        instr_rvalid;
    logic [31:0]                 instr_addr;
    logic [INSTR_RDATA_WIDTH-1:0] instr_rdata;

    logic                        data_req;
    logic                        data_gnt;
    logic                        data_rvalid;
    logic [31:0]                 data_addr;
    logic                        data_we;
    logic [3:0]                  data_be;
    logic [31:0]                 data_rdata;
    logic [31:0]                 data_wdata;

    // signals to debug unit
    logic                        debug_req_i;

    // irq signals (not used)
    logic                        irq;
    logic [0:4]                  irq_id_in;
    logic                        irq_ack;
    logic [0:4]                  irq_id_out;
    logic                        irq_sec;


    // interrupts (only timer for now)
    assign irq_sec = '0;

`ifdef USE_FOR_ZERORISCY
    // instantiate the core
    zeroriscy_core
        #(.N_EXT_PERF_COUNTERS(0),
          .RV32E(`RV32E),
          .RV32M(`RV32M))
    riscv_core_i
        (
         .clk_i                  ( clk_i                 ),
         .rst_ni                 ( rst_ni                ),

         .clock_en_i             ( '1                    ),
         .test_en_i              ( '0                    ),

         .boot_addr_i            ( BOOT_ADDR             ),
         .core_id_i              ( 4'h0                  ),
         .cluster_id_i           ( 6'h0                  ),

         .instr_addr_o           ( instr_addr            ),
         .instr_req_o            ( instr_req             ),
         .instr_rdata_i          ( instr_rdata           ),
         .instr_gnt_i            ( instr_gnt             ),
         .instr_rvalid_i         ( instr_rvalid          ),

         .data_addr_o            ( data_addr             ),
         .data_wdata_o           ( data_wdata            ),
         .data_we_o              ( data_we               ),
         .data_req_o             ( data_req              ),
         .data_be_o              ( data_be               ),
         .data_rdata_i           ( data_rdata            ),
         .data_gnt_i             ( data_gnt              ),
         .data_rvalid_i          ( data_rvalid           ),
	 .data_err_i             ( '0                    ), // not in riscy
/* not in zeroriscy
         .apu_master_req_o       (                       ),
         .apu_master_ready_o     (                       ),
         .apu_master_gnt_i       (                       ),
         .apu_master_operands_o  (                       ),
         .apu_master_op_o        (                       ),
         .apu_master_type_o      (                       ),
         .apu_master_flags_o     (                       ),
         .apu_master_valid_i     (                       ),
         .apu_master_result_i    (                       ),
         .apu_master_flags_i     (                       ),
 */
         .irq_i                  ( irq                   ),
         .irq_id_i               ( irq_id_in             ),
         .irq_ack_o              ( irq_ack               ),
         .irq_id_o               ( irq_id_out            ),
//         .irq_sec_i              ( irq_sec               ), not in zeroriscy

//         .sec_lvl_o              ( sec_lvl_o             ), not in zeroriscy

         .debug_req_i            ( '0                    ),
	 .debug_gnt_o            (                       ),
	 .debug_rvalid_o         (                       ),
	 .debug_addr_i           ( '0                    ),
	 .debug_we_i             ( '0                    ),
	 .debug_wdata_i          ( '0                    ),
	 .debug_rdata_o          (                       ),
	 .debug_halted_o         (                       ),
	 .debug_halt_i           ( '0                    ),
	 .debug_resume_i         ( '0                    ),
	 
         .fetch_enable_i         ( '1                    ),
//         .core_busy_o            ( core_busy_o           ), not in zeroriscy

         .ext_perf_counters_i    ( '0                    )
//         .fregfile_disable_i     ( 1'b0                  ) not in zeroriscy
	 );
`else

    // instantiate the core
    riscv_core
        #(.INSTR_RDATA_WIDTH (INSTR_RDATA_WIDTH),
          .PULP_SECURE(PULP_SECURE),
`ifdef FV_INCLUDE_RVF
          .FPU(1),
	  .FP_DIVSQRT(1)
`else
          .FPU(0)
`endif
	  )
    riscv_core_i
        (
         .clk_i                  ( clk_i                 ),
         .rst_ni                 ( rst_ni                ),

         .clock_en_i             ( '1                    ),
         .test_en_i              ( '0                    ),

         .boot_addr_i            ( BOOT_ADDR             ),
         .core_id_i              ( 4'h0                  ),
         .cluster_id_i           ( 6'h0                  ),

         .instr_addr_o           ( instr_addr            ),
         .instr_req_o            ( instr_req             ),
         .instr_rdata_i          ( instr_rdata           ),
         .instr_gnt_i            ( instr_gnt             ),
         .instr_rvalid_i         ( instr_rvalid          ),

         .data_addr_o            ( data_addr             ),
         .data_wdata_o           ( data_wdata            ),
         .data_we_o              ( data_we               ),
         .data_req_o             ( data_req              ),
         .data_be_o              ( data_be               ),
         .data_rdata_i           ( data_rdata            ),
         .data_gnt_i             ( data_gnt              ),
         .data_rvalid_i          ( data_rvalid           ),

         .apu_master_req_o       (                       ),
         .apu_master_ready_o     (                       ),
         .apu_master_gnt_i       (                       ),
         .apu_master_operands_o  (                       ),
         .apu_master_op_o        (                       ),
         .apu_master_type_o      (                       ),
         .apu_master_flags_o     (                       ),
         .apu_master_valid_i     (                       ),
         .apu_master_result_i    (                       ),
         .apu_master_flags_i     (                       ),

         .irq_i                  ( irq                   ),
         .irq_id_i               ( irq_id_in             ),
         .irq_ack_o              ( irq_ack               ),
         .irq_id_o               ( irq_id_out            ),
         .irq_sec_i              ( irq_sec               ),

         .sec_lvl_o              ( sec_lvl_o             ),

         .debug_req_i            ( '0                    ),

         .fetch_enable_i         ( '1                    ),
         .core_busy_o            ( core_busy_o           ),

         .ext_perf_counters_i    (                       ),
         .fregfile_disable_i     ( 1'b0                  ));
`endif // !`ifdef USE_FOR_ZERORISCY

    // this handles read to RAM and memory mapped pseudo peripherals
    mm_ram
        #(.RAM_ADDR_WIDTH (RAM_ADDR_WIDTH),
	  .RAM_DATA_WIDTH (INSTR_RDATA_WIDTH)
	  )
    ram_i
        (.clk_i          ( clk_i                          ),
         .rst_ni         ( rst_ni                         ),
`ifdef FV_INCLUDE_IF_STAGE
         .instr_req_i    ( 1'b0                           ),
         .instr_addr_i   ( {RAM_ADDR_WIDTH{1'b0}}         ),
         .instr_rdata_o  (                                ),
         .instr_rvalid_o (                                ),
         .instr_gnt_o    (                                ),
`else
         .instr_req_i    ( instr_req                      ),
         .instr_addr_i   ( instr_addr[RAM_ADDR_WIDTH-1:0] ),
         .instr_rdata_o  ( instr_rdata[INSTR_RDATA_WIDTH-1:0]),
         .instr_rvalid_o ( instr_rvalid                   ),
         .instr_gnt_o    ( instr_gnt                      ),
`endif
         .data_req_i     ( data_req                       ),
         .data_addr_i    ( data_addr                      ),
         .data_we_i      ( data_we                        ),
         .data_be_i      ( data_be                        ),
         .data_wdata_i   ( data_wdata                     ),
         .data_rdata_o   ( data_rdata                     ),
         .data_rvalid_o  ( data_rvalid                    ),
         .data_gnt_o     ( data_gnt                       ),

         .irq_id_i       ( irq_id_out                     ),
         .irq_ack_i      ( irq_ack                        ),
         .irq_id_o       ( irq_id_in                      ),
         .irq_o          ( irq                            ),

         .tests_passed_o ( tests_passed_o                 ),
         .tests_failed_o ( tests_failed_o                 ));

`ifdef FV_RISCY_INSERT_FV
      fv fv();
`endif
      
endmodule // riscv_wrapper
