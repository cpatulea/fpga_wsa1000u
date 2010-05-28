// -*- verilog -*-
//
//  USRP - Universal Software Radio Peripheral
//
//  Copyright (C) 2003,2004 Matt Ettus
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin Street, Boston, MA  02110-1301  USA
//

//
// Top-level module for ThinkRF WSA1000.
//

`include "config.vh"
`include "../../../firmware/include/fpga_regs_common.v"
`include "../../../firmware/include/fpga_regs_standard.v"

module wsa(
   input clk50_in,
   output clk100,
   input reset_pin,

   // Front end control
   output VSWA,
   output VSWB,
   output VSWC,
   output VSWD,
   
   // VCO controller
   output VCO_LE,
   input VCO_MUXOUT,
   
   // Filter controller
   output FILTER_A0,
   output FILTER_A1,

   // USB interface
   input USB_IFCLK,
   output [15:0] USB_FD,
   output USB_RDY0,
   output USB_RDY1,
   input USB_CTL0,
   input USB_CTL1,
   input USB_CTL2,
   input USB_PA2_CSN,
   input USB_PA3_FRDY,
   input USB_PA4_SCLK,
   output USB_PA5_SDO,
   input USB_PA6_SDI,

   // LEDs
   output LD0,
   output LD1,
   output LD2,
   output LD3,

   // ADC
   input ADC_DCOA,
   input ADC_DCOB, 
   input [13:0] ADC_DA,
   input [13:0] ADC_DB,
   inout ADC_SMI_SCLK,
   inout ADC_SMI_SDO,
   output ADC_CS_N,

   // SPI to ADC and serial flash
   output SDIO,
   output SCLK,
   output SPI_FLASH_CS,

   // RAM and StrataFlash
   output SD_CKE,
   output SF_CE0,
   output SF_OE,

   // Debugging/misc
   output [31:0] PROTO_PORT,
   output GNDpin29,
   output GNDpin30,
   output GNDpin39
);
   wire [15:0] debugdata,debugctrl;
   
   // Pin assignments restrict the concurrent use of some signals as clocks
   // because of BUFGMUX sharing. Either use only one of the clocks in each
   // BUFGMUX or route some of the clocks through a DCM, which bypasses
   // BUFGMUXes.
   //
   // Signals(pin)                              BUFGMUX/Quadrant clock line
   // ADC_DCOA(GCLK11@D9), USB_IFCLK(GCLK7@A10) X1Y10/H
   // clk50_in(GCLK8@B8), ADC_DCOB(GCLK4@D10)   X2Y11/E
   //
   // See also UCF file for note about pin assignment for (unused) ADC_DCOA.
   wire adcclk = ADC_DCOB;   // 50 MHz
   wire usbclk = USB_IFCLK;  // 48 MHz
   
   wire RD = USB_CTL1;
   wire OE = USB_CTL2;

   wire have_pkt_rdy;
   assign USB_RDY1 = have_pkt_rdy;

   wire   rx_overrun;    
   wire   clear_status = USB_CTL0;
   assign USB_RDY0 = rx_overrun;
      
   wire [15:0] usbdata_out;
   
   wire [11:0] rx_a_a = ADC_DA[13:2];     // TODO: widen
   wire [11:0] rx_b_a = ADC_DB[13:2];     // TODO: widen
   
   assign ADC_SMI_SCLK = 0;
   assign ADC_SMI_SDO = 0;
   
   wire [3:0]  rx_numchan;
   
   wire [7:0]  decim_rate;
   wire [31:0] rx_debugbus;
   
   wire        enable_rx;
   wire        rx_dsp_reset, rx_bus_reset;
   wire [7:0]  settings;
   
   // Tri-state output buffer
   xilinx_obuft16 usbdata_tri(.I(usbdata_out),.O(USB_FD),.T(~OE));

   wire [15:0] ch0rx,ch1rx;
   
   wire        serial_strobe;
   wire [6:0]  serial_addr;
   wire [31:0] serial_data;
   wire [2:0]  serial_csn;

   reg [15:0] debug_counter;
   
   // SPI bus via CPLD to ADC and d'board (J4)
   assign SCLK = USB_PA4_SCLK;
   assign SDIO = USB_PA6_SDI; // never read from ADC
   assign ADC_CS_N = serial_csn[1]; // active low
   assign VCO_LE = serial_csn[2]; // PLL on RFE0440

   // SPI bus to Patrick's ADF4350 board.
   assign PROTO_PORT[1] = USB_PA4_SCLK;
   assign PROTO_PORT[3] = USB_PA6_SDI;
   assign PROTO_PORT[5] = serial_csn[2];

   // TODO: Hang these off a SPI register.
   // VCO_MUXOUT

   // Misc
   assign clk100 = 0;
   assign SPI_FLASH_CS = 1; // active low
   assign SD_CKE = 0; // active high
   assign SF_CE0 = 1; // active low
   assign SF_OE = 1; // active low
   assign GNDpin30 = 0;
   assign GNDpin29 = 0;
   assign GNDpin39 = 0;
   assign LD0 = enable_rx;
   assign LD1 = rx_overrun;
   assign LD2 = 0;
   assign LD3 = 0;

   /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   // Receive Side
   wire        rx_sample_strobe,strobe_decim,hb_strobe;
   wire [15:0] bb_rx_i0,bb_rx_q0;

   wire counter = settings[1];

   always @(posedge adcclk)
     if(rx_dsp_reset)
       debug_counter <= #1 16'd0;
     else if(~enable_rx)
       debug_counter <= #1 16'd0;
     else if(hb_strobe)
       debug_counter <=#1 debug_counter + 16'd2;
   
   assign ch0rx = counter ? debug_counter : bb_rx_i0;
   assign ch1rx = counter ? debug_counter + 16'd1 : bb_rx_q0;

   wire [15:0] ddc0_in_i,ddc0_in_q;
   wire [31:0] rssi_0;
   
   adc_interface adc_interface(.clock(adcclk),.reset(rx_dsp_reset),.enable(1'b1),
			       .serial_addr(serial_addr),.serial_data(serial_data),.serial_strobe(serial_strobe),
			       .rx_a_a(rx_a_a),.rx_b_a(rx_b_a),
			       .rssi_0(rssi_0),
			       .ddc0_in_i(ddc0_in_i),.ddc0_in_q(ddc0_in_q),.rx_numchan(rx_numchan) );
   
   rx_buffer rx_buffer
     ( .usbclk(usbclk),.bus_reset(rx_bus_reset),.reset(rx_dsp_reset),
       .reset_regs(rx_dsp_reset),
       .usbdata(usbdata_out),.RD(RD),.have_pkt_rdy(have_pkt_rdy),.rx_overrun(rx_overrun),
       .channels(rx_numchan),
       .ch_0(ch0rx),.ch_1(ch1rx),
       .rxclk(adcclk),.rxstrobe(hb_strobe),
       .clear_status(clear_status),
       .serial_addr(serial_addr),.serial_data(serial_data),.serial_strobe(serial_strobe),
       .debugbus(rx_debugbus) );
   
   rx_chain #(`FR_RX_FREQ_0,`FR_RX_PHASE_0) rx_chain_0
     ( .clock(adcclk),.reset(1'b0),.enable(enable_rx),
       .decim_rate(decim_rate),.sample_strobe(rx_sample_strobe),.decimator_strobe(strobe_decim),.hb_strobe(hb_strobe),
       .serial_addr(serial_addr),.serial_data(serial_data),.serial_strobe(serial_strobe),
       .i_in(ddc0_in_i),.q_in(ddc0_in_q),.i_out(bb_rx_i0),.q_out(bb_rx_q0),.debugdata(debugdata),.debugctrl(debugctrl));

   ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   // Control Functions

   wire [31:0] capabilities;
   assign      capabilities[7] =   `TX_CAP_HB;
   assign      capabilities[6:4] = `TX_CAP_NCHAN;
   assign      capabilities[3] =   `RX_CAP_HB;
   assign      capabilities[2:0] = `RX_CAP_NCHAN;

   serial_select serial_select
     ( .sel({USB_PA3_FRDY, USB_PA2_CSN}),
       .csn(serial_csn)
       );

   wire [15:0] reg_1;
   serial_io serial_io
     ( .master_clk(usbclk),.serial_clock(USB_PA4_SCLK),.serial_data_in(USB_PA6_SDI),
       .enable(~serial_csn[0]),.reset(1'b0),.serial_data_out(USB_PA5_SDO),
       .serial_addr(serial_addr),.serial_data(serial_data),.serial_strobe(serial_strobe),
       .readback_0({reg_1, 16'b0}),
       .readback_1(0),.readback_2(capabilities),.readback_3(32'hf0f0931a),
       .readback_4(rssi_0)
       );

   master_control master_control
     ( .master_clk(adcclk),.usbclk(usbclk),
       .serial_addr(serial_addr),.serial_data(serial_data),.serial_strobe(serial_strobe),
       .rx_bus_reset(rx_bus_reset),
       .rx_dsp_reset(rx_dsp_reset),
       .enable_rx(enable_rx),
       .decim_rate(decim_rate),
       .rx_sample_strobe(rx_sample_strobe),.strobe_decim(strobe_decim),
       .tx_empty(tx_empty),
       .debug_0(rx_a_a),.debug_1(ddc0_in_i),
       .debug_2(rx_debugbus[15:0]),.debug_3(rx_debugbus[31:16]),
       .reg_1(reg_1));
   
   wire [9:0] io_rx_a_nc;
   io_pins io_pins
     (.io_1({io_rx_a_nc, FILTER_A1, FILTER_A0, VSWD, VSWC, VSWB, VSWA}), .reg_1(reg_1),
      .clock(adcclk),.rx_reset(rx_dsp_reset),
      .serial_addr(serial_addr),.serial_data(serial_data),.serial_strobe(serial_strobe));

   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   // Misc Settings
   setting_reg #(`FR_MODE) sr_misc(.clock(adcclk),.reset(rx_dsp_reset),.strobe(serial_strobe),.addr(serial_addr),.in(serial_data),.out(settings));

endmodule
