/*-------------------------------------------------------------------------
This confidential and proprietary software may be only used as authorized
by a licensing agreement from CrazyBingo.www.cnblogs.com/crazybingo
(C) COPYRIGHT 2012 CrazyBingo. ALL RIGHTS RESERVED
Filename            :       I2C_SC130GS_12801024_Config.v
Author              :       CrazyBingo
Date                :       2019-08-03
Version             :       1.0
Description         :       I2C Configure Data of AR0135.
Modification History    :
Date            By          Version         Change Description
===========================================================================
19/08/03        CrazyBingo  1.0             Original
--------------------------------------------------------------------------*/

`timescale 1ns/1ns

module  I2C_AD2020_1280960_FPS60_1Lane_Config  //60Hz
(
    input       [8:0]   LUT_INDEX,
    output  reg [23:0]  LUT_DATA,
    output      [8:0]   LUT_SIZE
);
// assign  LUT_SIZE = 92;
assign  LUT_SIZE = 14;

//-----------------------------------------------------------------
/////////////////////   Config Data LUT   //////////////////////////    
always@(*)
begin

    case(LUT_INDEX)
0:	LUT_DATA = {16'h358B, 8'h0A}; 
1:	LUT_DATA = {16'h356F, 8'h03}; 
2:	LUT_DATA = {16'h36DA, 8'h02}; 
3:	LUT_DATA = {16'h3808, 8'h25}; 
4:	LUT_DATA = {16'h380A, 8'h02}; 
5:	LUT_DATA = {16'h38dc, 8'h00}; 
6:	LUT_DATA = {16'h38ca, 8'h01}; 
7:	LUT_DATA = {16'h36d8, 8'h06}; 
8:	LUT_DATA = {16'h36d9, 8'h06}; 
9:	LUT_DATA = {16'h3169, 8'h00}; 
10:	LUT_DATA = {16'h38a8, 8'h03}; 
11:	LUT_DATA = {16'h38a9, 8'h1b}; 
12:	LUT_DATA = {16'h38ea, 8'h01}; 
13:	LUT_DATA = {16'h3000, 8'h00}; 


		default:LUT_DATA    =   {16'h0000, 8'h00};
    endcase
end

endmodule
