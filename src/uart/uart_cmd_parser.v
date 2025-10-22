/*-----------------------------------------------------------------------
								 \\\|///
							   \\  - -  //
								(  @ @  )
+-----------------------------oOOo-(_)-oOOo-----------------------------+
CONFIDENTIAL IN CONFIDENCE
This confidential and proprietary software may be only used as authorized
by a licensing agreement from CrazyBingo (Thereturnofbingo).
In the event of publication, the following notice is applicable:
Copyright (C) 2013-20xx CrazyBingo Corporation
The entire notice above must be reproduced on all authorized copies.
Author				:		CrazyBingo
Technology blogs 	: 		www.crazyfpga.com
Email Address 		: 		crazyfpga@vip.qq.com
Filename			:		PC2FPGA_UART_Test.v
Date				:		2013-10-31
Description			:		Test UART Communication between PC and FPGA.
Modification History	:
Date			By			Version			Change Description
=========================================================================
13/10/31		CrazyBingo	1.0				Original
-------------------------------------------------------------------------
|                                     Oooo								|
+------------------------------oooO--(   )-----------------------------+
                              (   )   ) /
                               \ (   (_/
                                \_)
----------------------------------------------------------------------*/   

`timescale 1ns/1ns
module uart_cmd_parser
(
	//global clock
//	input				clk,                //24MHz
//	input				rst_n,
    input               pll_inst1_CLKOUT0,  //96MHz
    input               pll_inst1_LOCKED,
	
	//user interface
	input				fpga_rxd,		//pc 2 fpga uart receiver
	output				fpga_txd,		//fpga 2 pc uart transfer	
    output  reg         w_filter_enable
);
wire    clk_ref = pll_inst1_CLKOUT0;    //96MHz
wire    sys_rst_n = pll_inst1_LOCKED;


//------------------------------------
//Precise clk divider
wire	divide_clken;
integer_divider	
#(
	.DEVIDE_CNT	(52)	//115200bps * 16
//	.DEVIDE_CNT	(625)	//9600bps * 16
)
u_integer_devider
(
	//global
	.clk				(clk_ref),		//96MHz clock
	.rst_n				(sys_rst_n),    //global reset
	
	//user interface
	.divide_clken		(divide_clken)
);


wire	clken_16bps = divide_clken;
//---------------------------------
//Data receive for PC to FPGA.
wire			rxd_flag;
wire	[7:0]	rxd_data;
uart_receiver	u_uart_receiver
(
	//gobal clock
	.clk			(clk_ref),
	.rst_n			(sys_rst_n),
	
	//uart interface
	.clken_16bps	(clken_16bps),	//clk_bps * 16
	.rxd			(fpga_rxd),		//uart txd interface
	
	//user interface
	.rxd_data		(rxd_data),		//uart data receive
	.rxd_flag		(rxd_flag)  	//uart data receive done
);

//---------------------------------
//String command parser
wire	[7:0]	filter_pattern;
wire			cmd_valid;
wire	[1:0]	cmd_type;
wire			response_ready;
wire	[7:0]	response_data;
wire			response_flag;
wire            txd_flag;
string_parser	u_string_parser
(
	//global clock
	.clk			(clk_ref),
	.rst_n			(sys_rst_n),
	
	//uart interface
	.clken_16bps	(clken_16bps),	//clk_bps * 16
	.rxd_data		(rxd_data),		//uart data receive
	.rxd_flag		(rxd_flag),		//uart data receive done
	
	//command interface
	.filter_pattern	(filter_pattern),	//LED pattern to display
    .tx_done        (txd_flag),
	.cmd_valid		(cmd_valid),	//command is valid
	.cmd_type		(cmd_type),		//command type
	.response_ready	(response_ready),//response is ready to send
	.response_data	(response_data),//response data to send
	.response_flag	(response_flag)	//response data valid
);

//---------------------------------
//Data transfer for FPGA to PC.
uart_transfer	u_uart_transfer
(
	//gobal clock
	.clk			(clk_ref),
	.rst_n			(sys_rst_n),
	
	//uaer interface
	.clken_16bps	(clken_16bps),	//clk_bps * 16
	.txd			(fpga_txd),  	//uart txd interface
           
	//user interface   
	.txd_en			(response_flag),//uart data transfer enable
	.txd_data		(response_data),//uart transfer data	
	.txd_flag		(txd_flag) 			    //uart data transfer done
);

//---------------------------------
//滤波控制逻辑 - 通过字符串指令控制滤波使能w_filter_enable和
always@(posedge clk_ref or negedge sys_rst_n)
begin
    if(!sys_rst_n)  // 复位时滤波关闭
        w_filter_enable <= 1'b0;
    else if(cmd_valid)  // 接收到有效指令时，更新滤波状态
        w_filter_enable <= filter_pattern;
        
end
endmodule

