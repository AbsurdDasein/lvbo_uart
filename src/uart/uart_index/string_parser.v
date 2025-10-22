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
Filename			:		string_parser.v
Date				:		2024-01-01
Description			:		String command parser for LED control
Modification History	:
Date			By			Version			Change Description
=========================================================================
24/01/01		CrazyBingo	1.0				Original
-------------------------------------------------------------------------
|                                     Oooo								|
+------------------------------oooO--(   )-----------------------------+
                              (   )   ) /
                               \ (   (_/
                                \_)
----------------------------------------------------------------------*/   

`timescale 1ns/1ns
module string_parser
(
	//global clock
	input				clk,
	input				rst_n,
	
	//uart interface
	input				clken_16bps,	//clk_bps * 16
	input		[7:0]	rxd_data,		//uart data receive
	input				rxd_flag,		//uart data receive done
	
	//command interface
	output	reg	[7:0]	filter_pattern,	//filter pattern to display
    input               tx_done,           // 新增：UART发送完成信号（来自UART发送模块
	output	reg			cmd_valid,		//command is valid
	output	reg	[1:0]	cmd_type,		//command type: 0=close, 1=on, 2=invaild
	output	reg			response_ready,	//response is ready to send
	output	reg	[7:0]	response_data,	//response data to send
	output	reg			response_flag	//response data valid
);

//---------------------------------------
//parameter definition
//命令状态cmd_type
localparam	cmd_off	=	2'd0;	//滤波关闭
localparam	cmd_on	=	2'd1;	//滤波开启  
localparam	CMD_INVALID		=	2'd2;	//命令无效

localparam	filter_off	=	1'd0;	//滤波关闭
localparam	filter_on	=	1'd1;	//滤波开启  

//state machine
localparam	IDLE		=	4'd0;	//idle state
localparam	RECEIVE		=	4'd1;	//receiving characters
localparam	PARSE		=	4'd2;	//parsing command
localparam	RESPONSE	=	4'd3;	//sending response
localparam	WAIT_END	=	4'd4;	//wait for command end

//---------------------------------------
//internal signals
reg	[3:0]	state;
reg	[7:0]	cmd_buffer[0:15];	//command buffer (max 16 chars)
reg	[3:0]	cmd_len;			//command length
reg	[3:0]	char_cnt;			//character counter
reg	[7:0]	hex_value;			//hex value for LED_XX command
reg	[4:0]	response_cnt;		//response character counter
reg	[7:0]	response_buffer[0:31]; //response buffer
reg	[4:0]	response_len;		//response length
reg			cmd_end_flag;		//command end flag
// 在模块内定义一个中间寄存器
reg [7:0] hex_value_reg;

//---------------------------------------
//command end detection (detect '\n' or '\r')
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
		cmd_end_flag <= 1'b0;
	else if(rxd_flag && (rxd_data == 8'd10 || rxd_data == 8'd13)) // '\n' or '\r'
		cmd_end_flag <= 1'b1;
	else if(state == PARSE)
		cmd_end_flag <= 1'b0;
end

//---------------------------------------
//main state machine
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
		begin
		state <= IDLE;
		cmd_len <= 0;
		char_cnt <= 0;
		cmd_valid <= 1'b0;
		cmd_type <= 0;
		filter_pattern <= filter_off;
		response_ready <= 1'b0;
		response_flag <= 1'b0;
		response_cnt <= 0;
		response_len <= 0;
        hex_value_reg <= 8'h00;  // 中间寄存器复位
		end
	else
		begin
		case(state)
		IDLE:
			begin
			cmd_valid <= 1'b0;
			response_ready <= 1'b0;
			response_flag <= 1'b0;
			response_cnt <= 0;
            filter_pattern  <= filter_pattern;
			if(rxd_flag && rxd_data != 8'd10 && rxd_data != 8'd13) //not '\n' or '\r'
				begin
				state <= RECEIVE;
				cmd_len <= 1;
				char_cnt <= 0;
				cmd_buffer[0] <= rxd_data;
				end
			else
                begin
                cmd_type <= CMD_INVALID;
				state <= IDLE;
                end
			end
			
		RECEIVE:
			begin
			if(rxd_flag)
				begin
				if(rxd_data == 8'd10 || rxd_data == 8'd13) //command end
					begin
					state <= PARSE;
					end
				else if(cmd_len < 15) //buffer not full
					begin
					cmd_len <= cmd_len + 1;
					cmd_buffer[cmd_len] <= rxd_data;
					end
				else //buffer full, invalid command
					begin
                    
					state <= IDLE;
					end
				end
			else
				state <= RECEIVE;
			end
			
		PARSE:
			begin
			//parse command
			if(cmd_len >= 10 && 
			   cmd_buffer[0] == "f" && cmd_buffer[1] == "i" && cmd_buffer[2] == "l" && cmd_buffer[3] == "t" &&
			   cmd_buffer[4] == "e" && cmd_buffer[5] == "r" && cmd_buffer[6] == "_" && cmd_buffer[7] == "o" &&
			   cmd_buffer[8] == "f" && cmd_buffer[9] == "f")
				begin
				//filter_close command
				cmd_type <= cmd_off;
				cmd_valid <= 1'b1;
				filter_pattern <= filter_off; //all LEDs off
				state <= RESPONSE;
				//prepare response: "filter_already_off"
				response_buffer[0] <= "f"; response_buffer[1] <= "i"; response_buffer[2] <= "l"; response_buffer[3] <= "t";
				response_buffer[4] <= "e"; response_buffer[5] <= "r"; response_buffer[6] <= "_"; response_buffer[7] <= "a";
				response_buffer[8] <= "l"; response_buffer[9] <= "r"; response_buffer[10] <= "e"; response_buffer[11] <= "a";
				response_buffer[12] <= "d"; response_buffer[13] <= "y"; response_buffer[14] <= "_"; response_buffer[15] <= "o";
				response_buffer[16] <= "f"; response_buffer[17] <= "f";
				response_len <= 18;
				end
			else if(cmd_len >= 9 && 
					cmd_buffer[0] == "f" && cmd_buffer[1] == "i" && cmd_buffer[2] == "l" && cmd_buffer[3] == "t" &&
                    cmd_buffer[4] == "e" && cmd_buffer[5] == "r" && cmd_buffer[6] == "_" && cmd_buffer[7] == "o" &&
                    cmd_buffer[8] == "n")
				begin
				//filter_on command
				cmd_type <= cmd_on;
				cmd_valid <= 1'b1;
				filter_pattern <= filter_on; //all LEDs on
				state <= RESPONSE;
				//prepare response: "filter_already_on"
				response_buffer[0] <= "f"; response_buffer[1] <= "i"; response_buffer[2] <= "l"; response_buffer[3] <= "t";
				response_buffer[4] <= "e"; response_buffer[5] <= "r"; response_buffer[6] <= "_"; response_buffer[7] <= "a";
				response_buffer[8] <= "l"; response_buffer[9] <= "r"; response_buffer[10] <= "e"; response_buffer[11] <= "a";
				response_buffer[12] <= "d"; response_buffer[13] <= "y"; response_buffer[14] <= "_"; response_buffer[15] <= "o";
				response_buffer[16] <= "n";
				response_len <= 17;
				end
            
			else//无效命令
				begin
				//invalid command
				cmd_type <= CMD_INVALID;
				cmd_valid <= 1'b0;
                cmd_len <= 0;  // 清除命令长度
				state <= RESPONSE;
                //prepare response: "cmd_invalid"
				response_buffer[0] <= "c"; response_buffer[1] <= "m"; response_buffer[2] <= "d"; response_buffer[3] <= "_";
				response_buffer[4] <= "i"; response_buffer[5] <= "n"; response_buffer[6] <= "v"; response_buffer[7] <= "a";
				response_buffer[8] <= "l"; response_buffer[9] <= "i"; response_buffer[10] <= "d";
				response_len <= 11;
				end
			end
			
		RESPONSE:
			begin
			response_ready <= 1'b1;
			if(response_cnt < response_len)
				begin
                    if(!response_flag)
                    begin
                        response_data <= response_buffer[response_cnt];
                        response_flag <= 1'b1;
                    end
                    else if(tx_done)  // 等待UART发送完成
                    begin
                        response_flag <= 1'b0;
                        response_cnt <= response_cnt + 1;
                    end
                end
			else
				begin
				response_flag <= 1'b0;
                response_ready <= 1'b0;
				state <= IDLE;
				end
			end
			
		default:
			state <= IDLE;
		endcase
		end
end

endmodule
