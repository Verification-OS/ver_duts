
`include "config.vh"

module ariane_ara_wrapper import axi_pkg::*; import ara_pkg::*;
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
      parameter PULP_SECURE = 1,

// ============
// from ara_soc
    // RVV Parameters
    parameter  int           unsigned NrLanes      = `NR_LANES, // Number of parallel vector lanes.
    // Support for floating-point data types
    parameter  fpu_support_e          FPUSupport   = FPUSupportHalfSingleDouble,
    // AXI Interface
    parameter  int           unsigned AxiDataWidth = 32*NrLanes,
    parameter  int           unsigned AxiAddrWidth = 64,
    parameter  int           unsigned AxiUserWidth = 1,
    parameter  int           unsigned AxiIdWidth   = 6,
    // Dependant parameters. DO NOT CHANGE!
    localparam type                   axi_data_t   = logic [AxiDataWidth-1:0],
    localparam type                   axi_strb_t   = logic [AxiDataWidth/8-1:0],
    localparam type                   axi_addr_t   = logic [AxiAddrWidth-1:0],
    localparam type                   axi_user_t   = logic [AxiUserWidth-1:0],
    localparam type                   axi_id_t     = logic [AxiIdWidth-1:0]

      )
    (input logic clk_i,
     input logic rst_ni
     );

  // Scan chain (tie-off input and dangle output)
  logic scan_enable_i, scan_data_i, scan_data_o;
  assign scan_enable_i = 0;
  assign scan_data_i = 0;
      
  //============================

  localparam NrAXIMasters = 2; // Actually masters, but slaves on the crossbar
                               // Ariane and Ara

/* Note: didn't build with the following; try again or tied UART and CTRL is enough?
 
  typedef enum int unsigned {
    DRAM = 0,
    ERR = 1
  } axi_slaves_e;
  localparam NrAXISlaves  = ERR + 1;

  localparam logic [63:0] DRAMBase   = 64'h00000000;
  localparam logic [63:0] DRAMLength = 64'h0_0100_0000_0000; // max for a 40-bit address
  localparam logic [63:0] ERRBase    = 64'h0_0100_0000_0000;
  localparam logic [63:0] ERRLength  = 64'hf_fe00_0000_0000; // max for a 40-bit address

  */

  typedef enum int unsigned {
    L2MEM = 0,
    UART  = 1,
    CTRL  = 2
  } axi_slaves_e;
  localparam NrAXISlaves = CTRL + 1;

  // Memory Map
//  localparam logic [63:0] DRAMLength = 64'h40000000;
  localparam logic [63:0] DRAMLength = 64'h2_0000; // matching data from 0 to 0x0_ffff and instruction from 0x1_0000 to 0x1_ffff
  localparam logic [63:0] UARTLength = 64'h1000;
  localparam logic [63:0] CTRLLength = 64'h1000;

  typedef enum logic [63:0] {
//    DRAMBase = 64'h8000_0000,
    DRAMBase = `FV_DUT_DATA_ADDRESS_BASE,
    UARTBase = 64'hC000_0000,
    CTRLBase = 64'hD000_0000
  } soc_bus_start_e;

  ///////////
  //  AXI  //
  ///////////

  `include "axi/assign.svh"
  `include "axi/typedef.svh"

  // Ariane's AXI port data width
  localparam AxiNarrowDataWidth = 64;
  localparam AxiNarrowStrbWidth = AxiNarrowDataWidth / 8;
  // Ara's AXI port data width
  localparam AxiWideDataWidth   = AxiDataWidth;
  localparam AXiWideStrbWidth   = AxiWideDataWidth / 8;

  localparam AxiSocIdWidth  = AxiIdWidth - $clog2(NrAXIMasters);
  localparam AxiCoreIdWidth = AxiSocIdWidth - 1;

  // AXI Typedefs

  // These are used for ara_soc's AXI interface
  `AXI_TYPEDEF_AR_CHAN_T(ar_chan_t, axi_addr_t, axi_id_t, axi_user_t)
  `AXI_TYPEDEF_R_CHAN_T(r_chan_t, axi_data_t, axi_id_t, axi_user_t)
  `AXI_TYPEDEF_AW_CHAN_T(aw_chan_t, axi_addr_t, axi_id_t, axi_user_t)
  `AXI_TYPEDEF_W_CHAN_T(w_chan_t, axi_data_t, axi_strb_t, axi_user_t)
  `AXI_TYPEDEF_B_CHAN_T(b_chan_t, axi_id_t, axi_user_t)
  `AXI_TYPEDEF_REQ_T(axi_req_t, aw_chan_t, w_chan_t, ar_chan_t)
  `AXI_TYPEDEF_RESP_T(axi_resp_t, b_chan_t, r_chan_t)

  // Internal types
  typedef logic [AxiNarrowDataWidth-1:0] axi_narrow_data_t;
  typedef logic [AxiNarrowStrbWidth-1:0] axi_narrow_strb_t;
  typedef logic [AxiSocIdWidth-1:0] axi_soc_id_t;
  typedef logic [AxiCoreIdWidth-1:0] axi_core_id_t;

  `AXI_TYPEDEF_W_CHAN_T(axi_wide_w_chan_t, axi_data_t, axi_strb_t, axi_user_t)
  `AXI_TYPEDEF_W_CHAN_T(axi_narrow_w_chan_t, axi_narrow_data_t, axi_narrow_strb_t, axi_user_t)

  `AXI_TYPEDEF_AR_CHAN_T(axi_soc_ar_chan_t, axi_addr_t, axi_soc_id_t, axi_user_t)
  `AXI_TYPEDEF_R_CHAN_T(axi_soc_wide_r_chan_t, axi_data_t, axi_soc_id_t, axi_user_t)
  `AXI_TYPEDEF_R_CHAN_T(axi_soc_narrow_r_chan_t, axi_narrow_data_t, axi_soc_id_t, axi_user_t)
  `AXI_TYPEDEF_AW_CHAN_T(axi_soc_aw_chan_t, axi_addr_t, axi_soc_id_t, axi_user_t)
  `AXI_TYPEDEF_B_CHAN_T(axi_soc_b_chan_t, axi_soc_id_t, axi_user_t)
  `AXI_TYPEDEF_REQ_T(axi_soc_wide_req_t, axi_soc_aw_chan_t, axi_wide_w_chan_t, axi_soc_ar_chan_t)
  `AXI_TYPEDEF_RESP_T(axi_soc_wide_resp_t, axi_soc_b_chan_t, axi_soc_wide_r_chan_t)
  `AXI_TYPEDEF_REQ_T(axi_soc_narrow_req_t, axi_soc_aw_chan_t, axi_narrow_w_chan_t,
    axi_soc_ar_chan_t)
  `AXI_TYPEDEF_RESP_T(axi_soc_narrow_resp_t, axi_soc_b_chan_t, axi_soc_narrow_r_chan_t)

  `AXI_TYPEDEF_AR_CHAN_T(axi_core_ar_chan_t, axi_addr_t, axi_core_id_t, axi_user_t)
  `AXI_TYPEDEF_R_CHAN_T(axi_core_wide_r_chan_t, axi_data_t, axi_core_id_t, axi_user_t)
  `AXI_TYPEDEF_R_CHAN_T(axi_core_narrow_r_chan_t, axi_narrow_data_t, axi_core_id_t, axi_user_t)
  `AXI_TYPEDEF_AW_CHAN_T(axi_core_aw_chan_t, axi_addr_t, axi_core_id_t, axi_user_t)
  `AXI_TYPEDEF_B_CHAN_T(axi_core_b_chan_t, axi_core_id_t, axi_user_t)
  `AXI_TYPEDEF_REQ_T(axi_core_wide_req_t, axi_core_aw_chan_t, axi_wide_w_chan_t, axi_core_ar_chan_t)
  `AXI_TYPEDEF_RESP_T(axi_core_wide_resp_t, axi_core_b_chan_t, axi_core_wide_r_chan_t)
  `AXI_TYPEDEF_REQ_T(axi_core_narrow_req_t, axi_core_aw_chan_t, axi_narrow_w_chan_t,
    axi_core_ar_chan_t)
  `AXI_TYPEDEF_RESP_T(axi_core_narrow_resp_t, axi_core_b_chan_t, axi_core_narrow_r_chan_t)
// Note: axi_lite_* not used? if not, delete the following 
  `AXI_LITE_TYPEDEF_AW_CHAN_T(axi_lite_soc_narrow_aw_t, axi_addr_t)
  `AXI_LITE_TYPEDEF_W_CHAN_T(axi_lite_soc_narrow_w_t, axi_narrow_data_t, axi_narrow_strb_t)
  `AXI_LITE_TYPEDEF_B_CHAN_T(axi_lite_soc_narrow_b_t)
  `AXI_LITE_TYPEDEF_AR_CHAN_T(axi_lite_soc_narrow_ar_t, axi_addr_t)
  `AXI_LITE_TYPEDEF_R_CHAN_T(axi_lite_soc_narrow_r_t, axi_narrow_data_t)
  `AXI_LITE_TYPEDEF_REQ_T(axi_lite_soc_narrow_req_t, axi_lite_soc_narrow_aw_t,
    axi_lite_soc_narrow_w_t, axi_lite_soc_narrow_ar_t)
  `AXI_LITE_TYPEDEF_RESP_T(axi_lite_soc_narrow_resp_t, axi_lite_soc_narrow_b_t,
    axi_lite_soc_narrow_r_t)

  // Buses
  axi_core_narrow_req_t  ariane_narrow_axi_req;
  axi_core_narrow_resp_t ariane_narrow_axi_resp;
  axi_core_wide_req_t    ariane_axi_req;
  axi_core_wide_resp_t   ariane_axi_resp;
  axi_core_wide_req_t    ara_axi_req;
  axi_core_wide_resp_t   ara_axi_resp;
  axi_core_wide_req_t    ara_axi_req_inval;
  axi_core_wide_resp_t   ara_axi_resp_inval;

  axi_soc_wide_req_t    [NrAXISlaves-1:0] periph_wide_axi_req;
  axi_soc_wide_resp_t   [NrAXISlaves-1:0] periph_wide_axi_resp;
  axi_soc_narrow_req_t  [NrAXISlaves-1:0] periph_narrow_axi_req;
  axi_soc_narrow_resp_t [NrAXISlaves-1:0] periph_narrow_axi_resp;

  ////////////////
  //  Crossbar  //
  ////////////////

  localparam axi_pkg::xbar_cfg_t XBarCfg = '{
    NoSlvPorts        : NrAXIMasters,
    NoMstPorts        : NrAXISlaves,
    MaxMstTrans       : 4,
    MaxSlvTrans       : 4,
    FallThrough       : 1'b0,
    LatencyMode       : axi_pkg::CUT_MST_PORTS,
    AxiIdWidthSlvPorts: AxiCoreIdWidth,
    AxiIdUsedSlvPorts : AxiCoreIdWidth,
    UniqueIds         : 1'b0,
    AxiAddrWidth      : AxiAddrWidth,
    AxiDataWidth      : AxiWideDataWidth,
    NoAddrRules       : NrAXISlaves
  };

  axi_pkg::xbar_rule_64_t [NrAXISlaves-1:0] routing_rules;
/*
  assign routing_rules = '{
    '{idx: DRAM, start_addr: DRAMBase, end_addr: DRAMBase + DRAMLength},
    '{idx: ERR,  start_addr: ERRBase,  end_addr: ERRBase + ERRLength}
  };
*/
  assign routing_rules = '{
    '{idx: CTRL, start_addr: CTRLBase, end_addr: CTRLBase + CTRLLength},
    '{idx: UART, start_addr: UARTBase, end_addr: UARTBase + UARTLength},
    '{idx: L2MEM, start_addr: DRAMBase, end_addr: DRAMBase + DRAMLength}
  };
      
  axi_xbar #(
    .Cfg          (XBarCfg                ),
    .slv_aw_chan_t(axi_core_aw_chan_t     ),
    .mst_aw_chan_t(axi_soc_aw_chan_t      ),
    .w_chan_t     (axi_wide_w_chan_t      ),
    .slv_b_chan_t (axi_core_b_chan_t      ),
    .mst_b_chan_t (axi_soc_b_chan_t       ),
    .slv_ar_chan_t(axi_core_ar_chan_t     ),
    .mst_ar_chan_t(axi_soc_ar_chan_t      ),
    .slv_r_chan_t (axi_core_wide_r_chan_t ),
    .mst_r_chan_t (axi_soc_wide_r_chan_t  ),
    .slv_req_t    (axi_core_wide_req_t    ),
    .slv_resp_t   (axi_core_wide_resp_t   ),
    .mst_req_t    (axi_soc_wide_req_t     ),
    .mst_resp_t   (axi_soc_wide_resp_t    ),
    .rule_t       (axi_pkg::xbar_rule_64_t)
  ) i_soc_xbar (
    .clk_i                (clk_i                                ),
    .rst_ni               (rst_ni                               ),
    .test_i               (1'b0                                 ),
    .slv_ports_req_i      ({ariane_axi_req, ara_axi_req_inval}  ),
    .slv_ports_resp_o     ({ariane_axi_resp, ara_axi_resp_inval}),
    .mst_ports_req_o      (periph_wide_axi_req                  ),
    .mst_ports_resp_i     (periph_wide_axi_resp                 ),
    .addr_map_i           (routing_rules                        ),
    .en_default_mst_port_i('0                                   ),
    .default_mst_port_i   ('0                                   )
  );

  // AXI interface with the DRAM
  // Note: ifdef the choice?
  // -- wide data --
  // axi_req_t  ram_w_amo_axi_req;
  // axi_resp_t ram_w_amo_axi_resp;
  // -- narrow data --
  ariane_axi::req_t  mem_w_amo_axi_req,  mem_axi_req;
  ariane_axi::resp_t mem_w_amo_axi_resp, mem_axi_resp;
  // -- above is same narrow data as below --
  // axi_core_narrow_req_t  ram_w_amo_axi_req;
  // axi_core_narrow_resp_t ram_w_amo_axi_resp;
  axi_soc_narrow_req_t  mem_soc_w_amo_axi_req;
  axi_soc_narrow_resp_t mem_soc_w_amo_axi_resp;

  // convert from SoC AXI interface to ariane_axi interfaces to be compatible with
  // interfaces and modules inherited from ariane_wrapper
  `AXI_ASSIGN_REQ_STRUCT(mem_w_amo_axi_req, mem_soc_w_amo_axi_req)
  `AXI_ASSIGN_RESP_STRUCT(mem_soc_w_amo_axi_resp, mem_w_amo_axi_resp)

  // No L2 cache (ara_soc has L2) but we need to convert from wide to narrow datawidth
  axi_dw_converter #(
    .AxiSlvPortDataWidth(AxiWideDataWidth       ),
    .AxiMstPortDataWidth(AxiNarrowDataWidth     ),
    .AxiAddrWidth       (AxiAddrWidth           ),
    .AxiIdWidth         (AxiSocIdWidth          ),
    .AxiMaxReads        (2                      ), // Note: increase?
    .ar_chan_t          (axi_soc_ar_chan_t      ),
    .mst_r_chan_t       (axi_soc_narrow_r_chan_t),
    .slv_r_chan_t       (axi_soc_wide_r_chan_t  ),
    .aw_chan_t          (axi_soc_aw_chan_t      ),
    .b_chan_t           (axi_soc_b_chan_t       ),
    .mst_w_chan_t       (axi_narrow_w_chan_t    ),
    .slv_w_chan_t       (axi_wide_w_chan_t      ),
    .axi_mst_req_t      (axi_soc_narrow_req_t   ),
    .axi_mst_resp_t     (axi_soc_narrow_resp_t  ),
    .axi_slv_req_t      (axi_soc_wide_req_t     ),
    .axi_slv_resp_t     (axi_soc_wide_resp_t    )
  ) i_axi_slave_dram_dwc (
    .clk_i     (clk_i                       ),
    .rst_ni    (rst_ni                      ),
//    .slv_req_i (periph_wide_axi_req[DRAM]   ),
//    .slv_resp_o(periph_wide_axi_resp[DRAM]  ),
    .slv_req_i (periph_wide_axi_req[L2MEM]   ),
    .slv_resp_o(periph_wide_axi_resp[L2MEM]  ),
    .mst_req_o (mem_soc_w_amo_axi_req ),
    .mst_resp_i(mem_soc_w_amo_axi_resp)
  );
      
  // NOTE: No axi_cut to avoid an extra flopping and clock cycle delay
   
  // NOTE: No UART
  assign periph_wide_axi_resp[UART].aw_ready = 1'b0;
  assign periph_wide_axi_resp[UART].w_ready  = 1'b0;
  assign periph_wide_axi_resp[UART].b_valid  = 1'b0;
  assign periph_wide_axi_resp[UART].b        = '0;
  assign periph_wide_axi_resp[UART].ar_ready = 1'b0;
  assign periph_wide_axi_resp[UART].r_valid  = 1'b0;
  assign periph_wide_axi_resp[UART].r        = '0;

  // NOTE: No CTRL
  assign periph_wide_axi_resp[CTRL].aw_ready = 1'b0;
  assign periph_wide_axi_resp[CTRL].w_ready  = 1'b0;
  assign periph_wide_axi_resp[CTRL].b_valid  = 1'b0;
  assign periph_wide_axi_resp[CTRL].b        = '0;
  assign periph_wide_axi_resp[CTRL].ar_ready = 1'b0;
  assign periph_wide_axi_resp[CTRL].r_valid  = 1'b0;
  assign periph_wide_axi_resp[CTRL].r        = '0;
   
  //////////////////////
  //  Ara and Ariane  //
  //////////////////////

  import ariane_pkg::accelerator_req_t;
  import ariane_pkg::accelerator_resp_t;

  localparam ariane_pkg::ariane_cfg_t ArianeAraConfig = '{
    RASDepth             : 2,
    BTBEntries           : 32,
    BHTEntries           : 128,
    // idempotent region
    NrNonIdempotentRules : 2,
    NonIdempotentAddrBase: {64'b0, 64'b0},
    NonIdempotentLength  : {64'b0, 64'b0},
    NrExecuteRegionRules : 3,
    //                      DRAM,       Boot ROM,   Debug Module
    ExecuteRegionAddrBase: {DRAMBase, 64'h1_0000, 64'h0},
    ExecuteRegionLength  : {DRAMLength, 64'h10000, 64'h1000},
    // cached region
    NrCachedRegionRules  : 1,
    CachedRegionAddrBase : {DRAMBase},
    CachedRegionLength   : {DRAMLength},
    //  cache config
    Axi64BitCompliant    : 1'b1,
    SwapEndianess        : 1'b0,
    // debug
    DmBaseAddress        : 64'h0,
    NrPMPEntries         : 0
  };

    // signals connecting core to memory
    // ariane_axi::req_t             core_axi_req,  mem_axi_req;
    // ariane_axi::resp_t            core_axi_resp, mem_axi_resp;

  // Accelerator ports
  accelerator_req_t                     acc_req;
  logic                                 acc_req_valid;
  logic                                 acc_req_ready;
  accelerator_resp_t                    acc_resp;
  logic                                 acc_resp_valid;
  logic                                 acc_resp_ready;
  logic                                 acc_cons_en;
  logic              [AxiAddrWidth-1:0] inval_addr;
  logic                                 inval_valid;
  logic                                 inval_ready;

    // instantiate the core
    ariane #(
      .ArianeCfg(ArianeAraConfig)
// non-default parameters to be added
    ) ariane_core_i
        (
         .clk_i       (clk_i),
         .rst_ni      (rst_ni),
//	 .boot_addr_i (DRAMBase),  // reset boot address
	 .boot_addr_i (BOOT_ADDR),  // reset boot address
	 .hart_id_i   (64'h0),  // hart id in a multicore environment (reflected in a CSR)
	 .irq_i       (2'b0),   // level sensitive IR lines, mip & sip (async)
	 .ipi_i       (1'b0),   // inter-processor interrupts (async)
	 .time_irq_i  (1'b0),   // timer interrupt in (async)
	 .debug_req_i (1'b0),   // debug request (async)
//	 .axi_req_o   (core_axi_req),
//	 .axi_resp_i  (core_axi_resp),
	 .axi_req_o        (ariane_narrow_axi_req ),
	 .axi_resp_i       (ariane_narrow_axi_resp),
	 // Accelerator ports
	 .acc_req_o        (acc_req               ),
	 .acc_req_valid_o  (acc_req_valid         ),
	 .acc_req_ready_i  (acc_req_ready         ),
	 .acc_resp_i       (acc_resp              ),
	 .acc_resp_valid_i (acc_resp_valid        ),
	 .acc_resp_ready_o (acc_resp_ready        ),
	 .acc_cons_en_o    (acc_cons_en           ),
	 .inval_addr_i     (inval_addr            ),
	 .inval_valid_i    (inval_valid           ),
	 .inval_ready_o    (inval_ready           )
	 );

  axi_dw_converter #(
    .AxiSlvPortDataWidth(AxiNarrowDataWidth      ),
    .AxiMstPortDataWidth(AxiWideDataWidth        ),
    .AxiAddrWidth       (AxiAddrWidth            ),
    .AxiIdWidth         (AxiCoreIdWidth          ),
    .AxiMaxReads        (4                       ),
    .ar_chan_t          (axi_core_ar_chan_t      ),
    .mst_r_chan_t       (axi_core_wide_r_chan_t  ),
    .slv_r_chan_t       (axi_core_narrow_r_chan_t),
    .aw_chan_t          (axi_core_aw_chan_t      ),
    .b_chan_t           (axi_core_b_chan_t       ),
    .mst_w_chan_t       (axi_wide_w_chan_t       ),
    .slv_w_chan_t       (axi_narrow_w_chan_t     ),
    .axi_mst_req_t      (axi_core_wide_req_t     ),
    .axi_mst_resp_t     (axi_core_wide_resp_t    ),
    .axi_slv_req_t      (axi_core_narrow_req_t   ),
    .axi_slv_resp_t     (axi_core_narrow_resp_t  )
  ) i_ariane_axi_dwc (
    .clk_i     (clk_i                 ),
    .rst_ni    (rst_ni                ),
    .slv_req_i (ariane_narrow_axi_req ),
    .slv_resp_o(ariane_narrow_axi_resp),
    .mst_req_o (ariane_axi_req        ),
    .mst_resp_i(ariane_axi_resp       )
  );

  axi_inval_filter #(
    .MaxTxns    (4                   ),
    .AddrWidth  (AxiAddrWidth        ),
    .L1LineWidth(16                  ),
    .aw_chan_t  (axi_core_aw_chan_t  ),
    .req_t      (axi_core_wide_req_t ),
    .resp_t     (axi_core_wide_resp_t)
  ) i_axi_inval_filter (
    .clk_i        (clk_i             ),
    .rst_ni       (rst_ni            ),
    .en_i         (acc_cons_en       ),
    .slv_req_i    (ara_axi_req       ),
    .slv_resp_o   (ara_axi_resp      ),
    .mst_req_o    (ara_axi_req_inval ),
    .mst_resp_i   (ara_axi_resp_inval),
    .inval_addr_o (inval_addr        ),
    .inval_valid_o(inval_valid       ),
    .inval_ready_i(inval_ready       )
  );

  ara #(
    .NrLanes     (NrLanes               ),
    .FPUSupport  (FPUSupport            ),
    .AxiDataWidth(AxiWideDataWidth      ),
    .AxiAddrWidth(AxiAddrWidth          ),
    .axi_ar_t    (axi_core_ar_chan_t    ),
    .axi_r_t     (axi_core_wide_r_chan_t),
    .axi_aw_t    (axi_core_aw_chan_t    ),
    .axi_w_t     (axi_wide_w_chan_t     ),
    .axi_b_t     (axi_core_b_chan_t     ),
    .axi_req_t   (axi_core_wide_req_t   ),
    .axi_resp_t  (axi_core_wide_resp_t  )
  ) i_ara (
    .clk_i           (clk_i         ),
    .rst_ni          (rst_ni        ),
    .scan_enable_i   (scan_enable_i ),
    .scan_data_i     (1'b0          ),
    .scan_data_o     (/* Unused */  ),
    .acc_req_i       (acc_req       ),
    .acc_req_valid_i (acc_req_valid ),
    .acc_req_ready_o (acc_req_ready ),
    .acc_resp_o      (acc_resp      ),
    .acc_resp_valid_o(acc_resp_valid),
    .acc_resp_ready_i(acc_resp_ready),
    .axi_req_o       (ara_axi_req   ),
    .axi_resp_i      (ara_axi_resp  )
  );
      
   //===============================
      
    localparam AxiNumWords = (ICACHE_LINE_WIDTH/64) * (ICACHE_LINE_WIDTH  > DCACHE_LINE_WIDTH)  +
                             (DCACHE_LINE_WIDTH/64) * (ICACHE_LINE_WIDTH <= DCACHE_LINE_WIDTH) ;
    
   fv_ariane_axi_atomics
      #(
	.AXI_ADDR_WIDTH(64),
	.AXI_DATA_WIDTH(RAM_DATA_WIDTH),
//	.AXI_ID_WIDTH($size(core_axi_resp.r.id)),
//	.AXI_USER_WIDTH($size(core_axi_resp.r.user)),
	.AXI_ID_WIDTH($size(mem_w_amo_axi_resp.r.id)),
	.AXI_USER_WIDTH($size(mem_w_amo_axi_resp.r.user)),
	.AXI_MAX_WRITE_TXNS(4), // Note: correct?
	.RISCV_WORD_WIDTH(64)
	)
    ariane_axi_atomics_i
      (
       .clk_i        (clk_i),
       .rst_ni       (rst_ni),
//       .slv_i        (core_axi_req),
//       .slv_o        (core_axi_resp),
       .slv_i        (mem_w_amo_axi_req),
       .slv_o        (mem_w_amo_axi_resp),
       .mst_o        (mem_axi_req),
       .mst_i        (mem_axi_resp)
       );

    fv_ariane_axi_memory 
        #(
	  .RAM_DATA_WIDTH (RAM_DATA_WIDTH),
	  .AxiNumWords    (AxiNumWords), // unused
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
      
endmodule // ariane_ara_wrapper
      
