// ======================================================================
// LED STRIP TEST.v generated from TopDesign.cysch
// 07/05/2017 at 11:46
// This file is auto generated. ANY EDITS YOU MAKE MAY BE LOST WHEN THIS FILE IS REGENERATED!!!
// ======================================================================

/* -- WARNING: The following section of defines are deprecated and will be removed in a future release -- */
`define CYDEV_CHIP_DIE_LEOPARD 1
`define CYDEV_CHIP_REV_LEOPARD_PRODUCTION 3
`define CYDEV_CHIP_REV_LEOPARD_ES3 3
`define CYDEV_CHIP_REV_LEOPARD_ES2 1
`define CYDEV_CHIP_REV_LEOPARD_ES1 0
`define CYDEV_CHIP_DIE_TMA4 2
`define CYDEV_CHIP_REV_TMA4_PRODUCTION 17
`define CYDEV_CHIP_REV_TMA4_ES 17
`define CYDEV_CHIP_REV_TMA4_ES2 33
`define CYDEV_CHIP_DIE_PSOC4A 3
`define CYDEV_CHIP_REV_PSOC4A_PRODUCTION 17
`define CYDEV_CHIP_REV_PSOC4A_ES0 17
`define CYDEV_CHIP_DIE_PSOC5LP 4
`define CYDEV_CHIP_REV_PSOC5LP_PRODUCTION 0
`define CYDEV_CHIP_REV_PSOC5LP_ES0 0
`define CYDEV_CHIP_DIE_PSOC5TM 5
`define CYDEV_CHIP_REV_PSOC5TM_PRODUCTION 1
`define CYDEV_CHIP_REV_PSOC5TM_ES1 1
`define CYDEV_CHIP_REV_PSOC5TM_ES0 0
`define CYDEV_CHIP_DIE_VOLANS 6
`define CYDEV_CHIP_REV_VOLANS_PRODUCTION 0
`define CYDEV_CHIP_DIE_BERRYPECKER 7
`define CYDEV_CHIP_REV_BERRYPECKER_PRODUCTION 0
`define CYDEV_CHIP_DIE_CRANE 8
`define CYDEV_CHIP_REV_CRANE_PRODUCTION 0
`define CYDEV_CHIP_DIE_FM3 9
`define CYDEV_CHIP_REV_FM3_PRODUCTION 0
`define CYDEV_CHIP_DIE_FM4 10
`define CYDEV_CHIP_REV_FM4_PRODUCTION 0
`define CYDEV_CHIP_DIE_EXPECT 3
`define CYDEV_CHIP_REV_EXPECT 17
`define CYDEV_CHIP_DIE_ACTUAL 3
/* -- WARNING: The previous section of defines are deprecated and will be removed in a future release -- */
`define CYDEV_CHIP_FAMILY_PSOC3 1
`define CYDEV_CHIP_FAMILY_PSOC4 2
`define CYDEV_CHIP_FAMILY_PSOC5 3
`define CYDEV_CHIP_FAMILY_FM0P 4
`define CYDEV_CHIP_FAMILY_FM3 5
`define CYDEV_CHIP_FAMILY_FM4 6
`define CYDEV_CHIP_FAMILY_UNKNOWN 0
`define CYDEV_CHIP_MEMBER_UNKNOWN 0
`define CYDEV_CHIP_MEMBER_3A 1
`define CYDEV_CHIP_REVISION_3A_PRODUCTION 3
`define CYDEV_CHIP_REVISION_3A_ES3 3
`define CYDEV_CHIP_REVISION_3A_ES2 1
`define CYDEV_CHIP_REVISION_3A_ES1 0
`define CYDEV_CHIP_MEMBER_4G 2
`define CYDEV_CHIP_REVISION_4G_PRODUCTION 17
`define CYDEV_CHIP_REVISION_4G_ES 17
`define CYDEV_CHIP_REVISION_4G_ES2 33
`define CYDEV_CHIP_MEMBER_4U 3
`define CYDEV_CHIP_REVISION_4U_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4E 4
`define CYDEV_CHIP_REVISION_4E_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4O 5
`define CYDEV_CHIP_REVISION_4O_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4N 6
`define CYDEV_CHIP_REVISION_4N_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4Q 7
`define CYDEV_CHIP_REVISION_4Q_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4D 8
`define CYDEV_CHIP_REVISION_4D_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4J 9
`define CYDEV_CHIP_REVISION_4J_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4K 10
`define CYDEV_CHIP_REVISION_4K_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4H 11
`define CYDEV_CHIP_REVISION_4H_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4A 12
`define CYDEV_CHIP_REVISION_4A_PRODUCTION 17
`define CYDEV_CHIP_REVISION_4A_ES0 17
`define CYDEV_CHIP_MEMBER_4F 13
`define CYDEV_CHIP_REVISION_4F_PRODUCTION 0
`define CYDEV_CHIP_REVISION_4F_PRODUCTION_256K 0
`define CYDEV_CHIP_REVISION_4F_PRODUCTION_256DMA 0
`define CYDEV_CHIP_MEMBER_4P 14
`define CYDEV_CHIP_REVISION_4P_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4M 15
`define CYDEV_CHIP_REVISION_4M_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4L 16
`define CYDEV_CHIP_REVISION_4L_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4I 17
`define CYDEV_CHIP_REVISION_4I_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4C 18
`define CYDEV_CHIP_REVISION_4C_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_5B 19
`define CYDEV_CHIP_REVISION_5B_PRODUCTION 0
`define CYDEV_CHIP_REVISION_5B_ES0 0
`define CYDEV_CHIP_MEMBER_5A 20
`define CYDEV_CHIP_REVISION_5A_PRODUCTION 1
`define CYDEV_CHIP_REVISION_5A_ES1 1
`define CYDEV_CHIP_REVISION_5A_ES0 0
`define CYDEV_CHIP_MEMBER_PDL_FM0P_TYPE1 21
`define CYDEV_CHIP_REVISION_PDL_FM0P_TYPE1_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_PDL_FM0P_TYPE2 22
`define CYDEV_CHIP_REVISION_PDL_FM0P_TYPE2_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_PDL_FM0P_TYPE3 23
`define CYDEV_CHIP_REVISION_PDL_FM0P_TYPE3_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_FM3 24
`define CYDEV_CHIP_REVISION_FM3_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_FM4 25
`define CYDEV_CHIP_REVISION_FM4_PRODUCTION 0
`define CYDEV_CHIP_FAMILY_USED 2
`define CYDEV_CHIP_MEMBER_USED 12
`define CYDEV_CHIP_REVISION_USED 17
// Component: ZeroTerminal
`ifdef CY_BLK_DIR
`undef CY_BLK_DIR
`endif

`ifdef WARP
`define CY_BLK_DIR "C:\Program Files (x86)\Cypress\PSoC Creator\4.0\PSoC Creator\psoc\content\cyprimitives\CyPrimitives.cylib\ZeroTerminal"
`include "C:\Program Files (x86)\Cypress\PSoC Creator\4.0\PSoC Creator\psoc\content\cyprimitives\CyPrimitives.cylib\ZeroTerminal\ZeroTerminal.v"
`else
`define CY_BLK_DIR "C:\Program Files (x86)\Cypress\PSoC Creator\4.0\PSoC Creator\psoc\content\cyprimitives\CyPrimitives.cylib\ZeroTerminal"
`include "C:\Program Files (x86)\Cypress\PSoC Creator\4.0\PSoC Creator\psoc\content\cyprimitives\CyPrimitives.cylib\ZeroTerminal\ZeroTerminal.v"
`endif

// Component: demux_v1_10
`ifdef CY_BLK_DIR
`undef CY_BLK_DIR
`endif

`ifdef WARP
`define CY_BLK_DIR "C:\Program Files (x86)\Cypress\PSoC Creator\4.0\PSoC Creator\psoc\content\cyprimitives\CyPrimitives.cylib\demux_v1_10"
`include "C:\Program Files (x86)\Cypress\PSoC Creator\4.0\PSoC Creator\psoc\content\cyprimitives\CyPrimitives.cylib\demux_v1_10\demux_v1_10.v"
`else
`define CY_BLK_DIR "C:\Program Files (x86)\Cypress\PSoC Creator\4.0\PSoC Creator\psoc\content\cyprimitives\CyPrimitives.cylib\demux_v1_10"
`include "C:\Program Files (x86)\Cypress\PSoC Creator\4.0\PSoC Creator\psoc\content\cyprimitives\CyPrimitives.cylib\demux_v1_10\demux_v1_10.v"
`endif

// Component: CyControlReg_v1_70
`ifdef CY_BLK_DIR
`undef CY_BLK_DIR
`endif

`ifdef WARP
`define CY_BLK_DIR "C:\Program Files (x86)\Cypress\PSoC Creator\4.0\PSoC Creator\psoc\content\cyprimitives\CyPrimitives.cylib\CyControlReg_v1_70"
`include "C:\Program Files (x86)\Cypress\PSoC Creator\4.0\PSoC Creator\psoc\content\cyprimitives\CyPrimitives.cylib\CyControlReg_v1_70\CyControlReg_v1_70.v"
`else
`define CY_BLK_DIR "C:\Program Files (x86)\Cypress\PSoC Creator\4.0\PSoC Creator\psoc\content\cyprimitives\CyPrimitives.cylib\CyControlReg_v1_70"
`include "C:\Program Files (x86)\Cypress\PSoC Creator\4.0\PSoC Creator\psoc\content\cyprimitives\CyPrimitives.cylib\CyControlReg_v1_70\CyControlReg_v1_70.v"
`endif

// Component: B_WS2811_v1_3
`ifdef CY_BLK_DIR
`undef CY_BLK_DIR
`endif

`ifdef WARP
`define CY_BLK_DIR "C:\Users\a85495\Downloads\StripLightLib\StripLightLib.cylib\B_WS2811_v1_3"
`include "C:\Users\a85495\Downloads\StripLightLib\StripLightLib.cylib\B_WS2811_v1_3\B_WS2811_v1_3.v"
`else
`define CY_BLK_DIR "C:\Users\a85495\Downloads\StripLightLib\StripLightLib.cylib\B_WS2811_v1_3"
`include "C:\Users\a85495\Downloads\StripLightLib\StripLightLib.cylib\B_WS2811_v1_3\B_WS2811_v1_3.v"
`endif

// StripLights_v2_2(Channels=1, ClockSpeedKhz=400, Display_Memory=0, LEDs_per_Strip=144, Speed=0, Transfer_Method=1, WS281x_Type=1, CY_API_CALLBACK_HEADER_INCLUDE=#include "cyapicallbacks.h", CY_COMPONENT_NAME=StripLights_v2_2, CY_CONST_CONFIG=true, CY_CONTROL_FILE=<:default:>, CY_DATASHEET_FILE=<:default:>, CY_FITTER_NAME=StripLights, CY_INSTANCE_SHORT_NAME=StripLights, CY_MAJOR_VERSION=2, CY_MINOR_VERSION=2, CY_PDL_DRIVER_NAME=, CY_PDL_DRIVER_REQ_VERSION=, CY_PDL_DRIVER_SUBGROUP=, CY_PDL_DRIVER_VARIANT=, CY_REMOVE=false, CY_SUPPRESS_API_GEN=false, CY_VERSION=PSoC Creator  4.0 Update 1, INSTANCE_NAME=StripLights, )
module StripLights_v2_2_0 (
    rdy,
    S0,
    S1,
    S2,
    S3,
    S4,
    S5,
    S6,
    S7,
    S8,
    S9,
    S10,
    S11,
    S12,
    S13,
    S14,
    S15);
    output      rdy;
    output      S0;
    output      S1;
    output      S2;
    output      S3;
    output      S4;
    output      S5;
    output      S6;
    output      S7;
    output      S8;
    output      S9;
    output      S10;
    output      S11;
    output      S12;
    output      S13;
    output      S14;
    output      S15;


          wire  Net_163;
          wire  Net_144;
          wire  Net_143;
          wire  Net_142;
          wire  Net_141;
          wire  Net_140;
          wire  Net_139;
          wire  Net_166;
          wire  Net_159;
          wire [3:0] saddr;
          wire  Net_64;
          wire  Net_18;
          wire  Net_7;

    ZeroTerminal ZeroTerminal_1 (
        .z(Net_7));


	cy_clock_v1_0
		#(.id("b67055fe-072b-4820-ac11-df2efc888043/e530a9ee-27b4-444b-91b5-72801be8ada4"),
		  .source_clock_id("413DE2EF-D9F2-4233-A808-DFAF137FD877"),
		  .divisor(0),
		  .period("0"),
		  .is_direct(1),
		  .is_digital(0))
		HFCLK
		 (.clock_out(Net_18));



	cy_isr_v1_0
		#(.int_type(2'b00))
		cisr
		 (.int_signal(rdy));



	cy_isr_v1_0
		#(.int_type(2'b00))
		fisr
		 (.int_signal(Net_159));


    // -- De Mux start --
    if (1)
    begin : demux_1
        reg  tmp__demux_1_0_reg;
        reg  tmp__demux_1_1_reg;
        reg  tmp__demux_1_2_reg;
        reg  tmp__demux_1_3_reg;
        reg  tmp__demux_1_4_reg;
        reg  tmp__demux_1_5_reg;
        reg  tmp__demux_1_6_reg;
        reg  tmp__demux_1_7_reg;
        reg  tmp__demux_1_8_reg;
        reg  tmp__demux_1_9_reg;
        reg  tmp__demux_1_10_reg;
        reg  tmp__demux_1_11_reg;
        reg  tmp__demux_1_12_reg;
        reg  tmp__demux_1_13_reg;
        reg  tmp__demux_1_14_reg;
        reg  tmp__demux_1_15_reg;
        always @(Net_64 or saddr)
        begin
            case (saddr[3:0])
                4'b0000:
                begin
                    tmp__demux_1_0_reg = Net_64;
                    tmp__demux_1_1_reg = 1'b0;
                    tmp__demux_1_2_reg = 1'b0;
                    tmp__demux_1_3_reg = 1'b0;
                    tmp__demux_1_4_reg = 1'b0;
                    tmp__demux_1_5_reg = 1'b0;
                    tmp__demux_1_6_reg = 1'b0;
                    tmp__demux_1_7_reg = 1'b0;
                    tmp__demux_1_8_reg = 1'b0;
                    tmp__demux_1_9_reg = 1'b0;
                    tmp__demux_1_10_reg = 1'b0;
                    tmp__demux_1_11_reg = 1'b0;
                    tmp__demux_1_12_reg = 1'b0;
                    tmp__demux_1_13_reg = 1'b0;
                    tmp__demux_1_14_reg = 1'b0;
                    tmp__demux_1_15_reg = 1'b0;
                end
                4'b0001:
                begin
                    tmp__demux_1_0_reg = 1'b0;
                    tmp__demux_1_1_reg = Net_64;
                    tmp__demux_1_2_reg = 1'b0;
                    tmp__demux_1_3_reg = 1'b0;
                    tmp__demux_1_4_reg = 1'b0;
                    tmp__demux_1_5_reg = 1'b0;
                    tmp__demux_1_6_reg = 1'b0;
                    tmp__demux_1_7_reg = 1'b0;
                    tmp__demux_1_8_reg = 1'b0;
                    tmp__demux_1_9_reg = 1'b0;
                    tmp__demux_1_10_reg = 1'b0;
                    tmp__demux_1_11_reg = 1'b0;
                    tmp__demux_1_12_reg = 1'b0;
                    tmp__demux_1_13_reg = 1'b0;
                    tmp__demux_1_14_reg = 1'b0;
                    tmp__demux_1_15_reg = 1'b0;
                end
                4'b0010:
                begin
                    tmp__demux_1_0_reg = 1'b0;
                    tmp__demux_1_1_reg = 1'b0;
                    tmp__demux_1_2_reg = Net_64;
                    tmp__demux_1_3_reg = 1'b0;
                    tmp__demux_1_4_reg = 1'b0;
                    tmp__demux_1_5_reg = 1'b0;
                    tmp__demux_1_6_reg = 1'b0;
                    tmp__demux_1_7_reg = 1'b0;
                    tmp__demux_1_8_reg = 1'b0;
                    tmp__demux_1_9_reg = 1'b0;
                    tmp__demux_1_10_reg = 1'b0;
                    tmp__demux_1_11_reg = 1'b0;
                    tmp__demux_1_12_reg = 1'b0;
                    tmp__demux_1_13_reg = 1'b0;
                    tmp__demux_1_14_reg = 1'b0;
                    tmp__demux_1_15_reg = 1'b0;
                end
                4'b0011:
                begin
                    tmp__demux_1_0_reg = 1'b0;
                    tmp__demux_1_1_reg = 1'b0;
                    tmp__demux_1_2_reg = 1'b0;
                    tmp__demux_1_3_reg = Net_64;
                    tmp__demux_1_4_reg = 1'b0;
                    tmp__demux_1_5_reg = 1'b0;
                    tmp__demux_1_6_reg = 1'b0;
                    tmp__demux_1_7_reg = 1'b0;
                    tmp__demux_1_8_reg = 1'b0;
                    tmp__demux_1_9_reg = 1'b0;
                    tmp__demux_1_10_reg = 1'b0;
                    tmp__demux_1_11_reg = 1'b0;
                    tmp__demux_1_12_reg = 1'b0;
                    tmp__demux_1_13_reg = 1'b0;
                    tmp__demux_1_14_reg = 1'b0;
                    tmp__demux_1_15_reg = 1'b0;
                end
                4'b0100:
                begin
                    tmp__demux_1_0_reg = 1'b0;
                    tmp__demux_1_1_reg = 1'b0;
                    tmp__demux_1_2_reg = 1'b0;
                    tmp__demux_1_3_reg = 1'b0;
                    tmp__demux_1_4_reg = Net_64;
                    tmp__demux_1_5_reg = 1'b0;
                    tmp__demux_1_6_reg = 1'b0;
                    tmp__demux_1_7_reg = 1'b0;
                    tmp__demux_1_8_reg = 1'b0;
                    tmp__demux_1_9_reg = 1'b0;
                    tmp__demux_1_10_reg = 1'b0;
                    tmp__demux_1_11_reg = 1'b0;
                    tmp__demux_1_12_reg = 1'b0;
                    tmp__demux_1_13_reg = 1'b0;
                    tmp__demux_1_14_reg = 1'b0;
                    tmp__demux_1_15_reg = 1'b0;
                end
                4'b0101:
                begin
                    tmp__demux_1_0_reg = 1'b0;
                    tmp__demux_1_1_reg = 1'b0;
                    tmp__demux_1_2_reg = 1'b0;
                    tmp__demux_1_3_reg = 1'b0;
                    tmp__demux_1_4_reg = 1'b0;
                    tmp__demux_1_5_reg = Net_64;
                    tmp__demux_1_6_reg = 1'b0;
                    tmp__demux_1_7_reg = 1'b0;
                    tmp__demux_1_8_reg = 1'b0;
                    tmp__demux_1_9_reg = 1'b0;
                    tmp__demux_1_10_reg = 1'b0;
                    tmp__demux_1_11_reg = 1'b0;
                    tmp__demux_1_12_reg = 1'b0;
                    tmp__demux_1_13_reg = 1'b0;
                    tmp__demux_1_14_reg = 1'b0;
                    tmp__demux_1_15_reg = 1'b0;
                end
                4'b0110:
                begin
                    tmp__demux_1_0_reg = 1'b0;
                    tmp__demux_1_1_reg = 1'b0;
                    tmp__demux_1_2_reg = 1'b0;
                    tmp__demux_1_3_reg = 1'b0;
                    tmp__demux_1_4_reg = 1'b0;
                    tmp__demux_1_5_reg = 1'b0;
                    tmp__demux_1_6_reg = Net_64;
                    tmp__demux_1_7_reg = 1'b0;
                    tmp__demux_1_8_reg = 1'b0;
                    tmp__demux_1_9_reg = 1'b0;
                    tmp__demux_1_10_reg = 1'b0;
                    tmp__demux_1_11_reg = 1'b0;
                    tmp__demux_1_12_reg = 1'b0;
                    tmp__demux_1_13_reg = 1'b0;
                    tmp__demux_1_14_reg = 1'b0;
                    tmp__demux_1_15_reg = 1'b0;
                end
                4'b0111:
                begin
                    tmp__demux_1_0_reg = 1'b0;
                    tmp__demux_1_1_reg = 1'b0;
                    tmp__demux_1_2_reg = 1'b0;
                    tmp__demux_1_3_reg = 1'b0;
                    tmp__demux_1_4_reg = 1'b0;
                    tmp__demux_1_5_reg = 1'b0;
                    tmp__demux_1_6_reg = 1'b0;
                    tmp__demux_1_7_reg = Net_64;
                    tmp__demux_1_8_reg = 1'b0;
                    tmp__demux_1_9_reg = 1'b0;
                    tmp__demux_1_10_reg = 1'b0;
                    tmp__demux_1_11_reg = 1'b0;
                    tmp__demux_1_12_reg = 1'b0;
                    tmp__demux_1_13_reg = 1'b0;
                    tmp__demux_1_14_reg = 1'b0;
                    tmp__demux_1_15_reg = 1'b0;
                end
                4'b1000:
                begin
                    tmp__demux_1_0_reg = 1'b0;
                    tmp__demux_1_1_reg = 1'b0;
                    tmp__demux_1_2_reg = 1'b0;
                    tmp__demux_1_3_reg = 1'b0;
                    tmp__demux_1_4_reg = 1'b0;
                    tmp__demux_1_5_reg = 1'b0;
                    tmp__demux_1_6_reg = 1'b0;
                    tmp__demux_1_7_reg = 1'b0;
                    tmp__demux_1_8_reg = Net_64;
                    tmp__demux_1_9_reg = 1'b0;
                    tmp__demux_1_10_reg = 1'b0;
                    tmp__demux_1_11_reg = 1'b0;
                    tmp__demux_1_12_reg = 1'b0;
                    tmp__demux_1_13_reg = 1'b0;
                    tmp__demux_1_14_reg = 1'b0;
                    tmp__demux_1_15_reg = 1'b0;
                end
                4'b1001:
                begin
                    tmp__demux_1_0_reg = 1'b0;
                    tmp__demux_1_1_reg = 1'b0;
                    tmp__demux_1_2_reg = 1'b0;
                    tmp__demux_1_3_reg = 1'b0;
                    tmp__demux_1_4_reg = 1'b0;
                    tmp__demux_1_5_reg = 1'b0;
                    tmp__demux_1_6_reg = 1'b0;
                    tmp__demux_1_7_reg = 1'b0;
                    tmp__demux_1_8_reg = 1'b0;
                    tmp__demux_1_9_reg = Net_64;
                    tmp__demux_1_10_reg = 1'b0;
                    tmp__demux_1_11_reg = 1'b0;
                    tmp__demux_1_12_reg = 1'b0;
                    tmp__demux_1_13_reg = 1'b0;
                    tmp__demux_1_14_reg = 1'b0;
                    tmp__demux_1_15_reg = 1'b0;
                end
                4'b1010:
                begin
                    tmp__demux_1_0_reg = 1'b0;
                    tmp__demux_1_1_reg = 1'b0;
                    tmp__demux_1_2_reg = 1'b0;
                    tmp__demux_1_3_reg = 1'b0;
                    tmp__demux_1_4_reg = 1'b0;
                    tmp__demux_1_5_reg = 1'b0;
                    tmp__demux_1_6_reg = 1'b0;
                    tmp__demux_1_7_reg = 1'b0;
                    tmp__demux_1_8_reg = 1'b0;
                    tmp__demux_1_9_reg = 1'b0;
                    tmp__demux_1_10_reg = Net_64;
                    tmp__demux_1_11_reg = 1'b0;
                    tmp__demux_1_12_reg = 1'b0;
                    tmp__demux_1_13_reg = 1'b0;
                    tmp__demux_1_14_reg = 1'b0;
                    tmp__demux_1_15_reg = 1'b0;
                end
                4'b1011:
                begin
                    tmp__demux_1_0_reg = 1'b0;
                    tmp__demux_1_1_reg = 1'b0;
                    tmp__demux_1_2_reg = 1'b0;
                    tmp__demux_1_3_reg = 1'b0;
                    tmp__demux_1_4_reg = 1'b0;
                    tmp__demux_1_5_reg = 1'b0;
                    tmp__demux_1_6_reg = 1'b0;
                    tmp__demux_1_7_reg = 1'b0;
                    tmp__demux_1_8_reg = 1'b0;
                    tmp__demux_1_9_reg = 1'b0;
                    tmp__demux_1_10_reg = 1'b0;
                    tmp__demux_1_11_reg = Net_64;
                    tmp__demux_1_12_reg = 1'b0;
                    tmp__demux_1_13_reg = 1'b0;
                    tmp__demux_1_14_reg = 1'b0;
                    tmp__demux_1_15_reg = 1'b0;
                end
                4'b1100:
                begin
                    tmp__demux_1_0_reg = 1'b0;
                    tmp__demux_1_1_reg = 1'b0;
                    tmp__demux_1_2_reg = 1'b0;
                    tmp__demux_1_3_reg = 1'b0;
                    tmp__demux_1_4_reg = 1'b0;
                    tmp__demux_1_5_reg = 1'b0;
                    tmp__demux_1_6_reg = 1'b0;
                    tmp__demux_1_7_reg = 1'b0;
                    tmp__demux_1_8_reg = 1'b0;
                    tmp__demux_1_9_reg = 1'b0;
                    tmp__demux_1_10_reg = 1'b0;
                    tmp__demux_1_11_reg = 1'b0;
                    tmp__demux_1_12_reg = Net_64;
                    tmp__demux_1_13_reg = 1'b0;
                    tmp__demux_1_14_reg = 1'b0;
                    tmp__demux_1_15_reg = 1'b0;
                end
                4'b1101:
                begin
                    tmp__demux_1_0_reg = 1'b0;
                    tmp__demux_1_1_reg = 1'b0;
                    tmp__demux_1_2_reg = 1'b0;
                    tmp__demux_1_3_reg = 1'b0;
                    tmp__demux_1_4_reg = 1'b0;
                    tmp__demux_1_5_reg = 1'b0;
                    tmp__demux_1_6_reg = 1'b0;
                    tmp__demux_1_7_reg = 1'b0;
                    tmp__demux_1_8_reg = 1'b0;
                    tmp__demux_1_9_reg = 1'b0;
                    tmp__demux_1_10_reg = 1'b0;
                    tmp__demux_1_11_reg = 1'b0;
                    tmp__demux_1_12_reg = 1'b0;
                    tmp__demux_1_13_reg = Net_64;
                    tmp__demux_1_14_reg = 1'b0;
                    tmp__demux_1_15_reg = 1'b0;
                end
                4'b1110:
                begin
                    tmp__demux_1_0_reg = 1'b0;
                    tmp__demux_1_1_reg = 1'b0;
                    tmp__demux_1_2_reg = 1'b0;
                    tmp__demux_1_3_reg = 1'b0;
                    tmp__demux_1_4_reg = 1'b0;
                    tmp__demux_1_5_reg = 1'b0;
                    tmp__demux_1_6_reg = 1'b0;
                    tmp__demux_1_7_reg = 1'b0;
                    tmp__demux_1_8_reg = 1'b0;
                    tmp__demux_1_9_reg = 1'b0;
                    tmp__demux_1_10_reg = 1'b0;
                    tmp__demux_1_11_reg = 1'b0;
                    tmp__demux_1_12_reg = 1'b0;
                    tmp__demux_1_13_reg = 1'b0;
                    tmp__demux_1_14_reg = Net_64;
                    tmp__demux_1_15_reg = 1'b0;
                end
                4'b1111:
                begin
                    tmp__demux_1_0_reg = 1'b0;
                    tmp__demux_1_1_reg = 1'b0;
                    tmp__demux_1_2_reg = 1'b0;
                    tmp__demux_1_3_reg = 1'b0;
                    tmp__demux_1_4_reg = 1'b0;
                    tmp__demux_1_5_reg = 1'b0;
                    tmp__demux_1_6_reg = 1'b0;
                    tmp__demux_1_7_reg = 1'b0;
                    tmp__demux_1_8_reg = 1'b0;
                    tmp__demux_1_9_reg = 1'b0;
                    tmp__demux_1_10_reg = 1'b0;
                    tmp__demux_1_11_reg = 1'b0;
                    tmp__demux_1_12_reg = 1'b0;
                    tmp__demux_1_13_reg = 1'b0;
                    tmp__demux_1_14_reg = 1'b0;
                    tmp__demux_1_15_reg = Net_64;
                end
            endcase
        end
        assign S0 = tmp__demux_1_0_reg;
        assign S1 = tmp__demux_1_1_reg;
        assign S2 = tmp__demux_1_2_reg;
        assign S3 = tmp__demux_1_3_reg;
        assign S4 = tmp__demux_1_4_reg;
        assign S5 = tmp__demux_1_5_reg;
        assign S6 = tmp__demux_1_6_reg;
        assign S7 = tmp__demux_1_7_reg;
        assign S8 = tmp__demux_1_8_reg;
        assign S9 = tmp__demux_1_9_reg;
        assign S10 = tmp__demux_1_10_reg;
        assign S11 = tmp__demux_1_11_reg;
        assign S12 = tmp__demux_1_12_reg;
        assign S13 = tmp__demux_1_13_reg;
        assign S14 = tmp__demux_1_14_reg;
        assign S15 = tmp__demux_1_15_reg;
    end
    // -- De Mux end --

    CyControlReg_v1_70 StringSel (
        .control_1(saddr[1]),
        .control_2(saddr[2]),
        .control_3(saddr[3]),
        .control_0(saddr[0]),
        .control_4(Net_139),
        .control_5(Net_140),
        .control_6(Net_141),
        .control_7(Net_142),
        .clock(1'b0),
        .reset(1'b0));
    defparam StringSel.Bit0Mode = 0;
    defparam StringSel.Bit1Mode = 0;
    defparam StringSel.Bit2Mode = 0;
    defparam StringSel.Bit3Mode = 0;
    defparam StringSel.Bit4Mode = 0;
    defparam StringSel.Bit5Mode = 0;
    defparam StringSel.Bit6Mode = 0;
    defparam StringSel.Bit7Mode = 0;
    defparam StringSel.BitValue = 0;
    defparam StringSel.BusDisplay = 0;
    defparam StringSel.ExtrReset = 0;
    defparam StringSel.NumOutputs = 4;

    B_WS2811_v1_3 B_WS2811 (
        .clk(Net_18),
        .reset(Net_7),
        .sout(Net_64),
        .firq(Net_159),
        .cntl(Net_163),
        .cirq(rdy));



endmodule

// top
module top ;

          wire  Net_17;
          wire  Net_16;
          wire  Net_15;
          wire  Net_14;
          wire  Net_13;
          wire  Net_12;
          wire  Net_11;
          wire  Net_10;
          wire  Net_9;
          wire  Net_8;
          wire  Net_7;
          wire  Net_6;
          wire  Net_5;
          wire  Net_4;
          wire  Net_3;
          wire  Net_39;
          wire  Net_43;

    StripLights_v2_2_0 StripLights (
        .rdy(Net_39),
        .S0(Net_43),
        .S1(Net_3),
        .S2(Net_4),
        .S3(Net_5),
        .S4(Net_6),
        .S5(Net_7),
        .S6(Net_8),
        .S7(Net_9),
        .S8(Net_10),
        .S9(Net_11),
        .S10(Net_12),
        .S11(Net_13),
        .S12(Net_14),
        .S13(Net_15),
        .S14(Net_16),
        .S15(Net_17));

	wire [0:0] tmpOE__Pin_1_net;
	wire [0:0] tmpFB_0__Pin_1_net;
	wire [0:0] tmpIO_0__Pin_1_net;
	wire [0:0] tmpINTERRUPT_0__Pin_1_net;
	electrical [0:0] tmpSIOVREF__Pin_1_net;

	cy_psoc3_pins_v1_10
		#(.id("e851a3b9-efb8-48be-bbb8-b303b216c393"),
		  .drive_mode(3'b110),
		  .ibuf_enabled(1'b1),
		  .init_dr_st(1'b1),
		  .input_clk_en(0),
		  .input_sync(1'b1),
		  .input_sync_mode(1'b0),
		  .intr_mode(2'b00),
		  .invert_in_clock(0),
		  .invert_in_clock_en(0),
		  .invert_in_reset(0),
		  .invert_out_clock(0),
		  .invert_out_clock_en(0),
		  .invert_out_reset(0),
		  .io_voltage(""),
		  .layout_mode("CONTIGUOUS"),
		  .oe_conn(1'b0),
		  .oe_reset(0),
		  .oe_sync(1'b0),
		  .output_clk_en(0),
		  .output_clock_mode(1'b0),
		  .output_conn(1'b1),
		  .output_mode(1'b0),
		  .output_reset(0),
		  .output_sync(1'b0),
		  .pa_in_clock(-1),
		  .pa_in_clock_en(-1),
		  .pa_in_reset(-1),
		  .pa_out_clock(-1),
		  .pa_out_clock_en(-1),
		  .pa_out_reset(-1),
		  .pin_aliases(""),
		  .pin_mode("O"),
		  .por_state(4),
		  .sio_group_cnt(0),
		  .sio_hyst(1'b1),
		  .sio_ibuf(""),
		  .sio_info(2'b00),
		  .sio_obuf(""),
		  .sio_refsel(""),
		  .sio_vtrip(""),
		  .sio_hifreq(""),
		  .sio_vohsel(""),
		  .slew_rate(1'b0),
		  .spanning(0),
		  .use_annotation(1'b0),
		  .vtrip(2'b10),
		  .width(1),
		  .ovt_hyst_trim(1'b0),
		  .ovt_needed(1'b0),
		  .ovt_slew_control(2'b00),
		  .input_buffer_sel(2'b00))
		Pin_1
		 (.oe(tmpOE__Pin_1_net),
		  .y({Net_43}),
		  .fb({tmpFB_0__Pin_1_net[0:0]}),
		  .io({tmpIO_0__Pin_1_net[0:0]}),
		  .siovref(tmpSIOVREF__Pin_1_net),
		  .interrupt({tmpINTERRUPT_0__Pin_1_net[0:0]}),
		  .in_clock({1'b0}),
		  .in_clock_en({1'b1}),
		  .in_reset({1'b0}),
		  .out_clock({1'b0}),
		  .out_clock_en({1'b1}),
		  .out_reset({1'b0}));

	assign tmpOE__Pin_1_net = (`CYDEV_CHIP_MEMBER_USED == `CYDEV_CHIP_MEMBER_3A && `CYDEV_CHIP_REVISION_USED < `CYDEV_CHIP_REVISION_3A_ES3) ? ~{1'b1} : {1'b1};



endmodule

