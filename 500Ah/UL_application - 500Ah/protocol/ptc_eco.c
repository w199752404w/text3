/* Created by h00205922, 2024.09.06 for multiple protocols compatible */

#include "ptc_eco.h"
#include "config.h"
#include "local.h"
#include "bsp_can.h"
#include "bsp_sh367309.h"
#include "eco.h"
#include "history.h"

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <systick.h>

const CAN_VAL_CFG_S g_astCanValCfg[E_PTC_ECO_CV_NUM] = {		//const global variable use ROM space, to save RAM
//ByteInversion	WordInversion	StartBit	BitNum	Signed	FacA		FacB
	//Message ID 1
	{0, 					0, 						0, 				16, 		0, 			0.10, 		0},
	{0, 					0, 						16, 			8, 			0, 			1, 			-40},
	{0, 					0, 						32, 			16, 		0, 			0.1, 		0},
	{0, 					0, 						48, 			16, 		0, 			0.1, 		0},
	//Message ID 2
	{0, 					0, 						0, 				16, 		0, 			0.01, 	0},
	{0, 					0, 						16, 			8, 			0, 			1, 			0},
	{0, 					0, 						32, 			16, 		0, 			0.1, 		0},
	{0, 					0, 						48, 			16, 		0, 			0.1, 		0},
	//Message ID 3
	{0, 					0, 						0, 				1, 			0, 			1, 			0},
	{0, 					0, 						1, 				1, 			0, 			1, 			0},
	{0, 					0, 						2, 				1, 			0, 			1, 			0},
	{0, 					0, 						4, 				1, 			0, 			1, 			0},
	{0, 					0, 						16, 			8, 			0, 			1, 			0},
	{0, 					0, 						32, 			16, 		0, 			0.1, 		0},
	{0, 					0, 						48, 			16, 		0, 			0.1, 		0},
	//Message ID 4
	{0, 					0, 						0, 				16, 		0, 			0.01, 	0},
	{0, 					0, 						16, 			16, 		1, 			0.01, 	0},
	{0, 					0, 						32, 			16, 		1, 			0.1, 		0},
	{0, 					0, 						48, 			16, 		1, 			0.1, 		0},
	//Message ID 5
	{0, 					0, 						0, 				8, 			1, 			1, 			0},
	{0, 					0, 						8, 				8, 			1, 			1, 			0},
	{0, 					0, 						16, 			16, 		0, 			0.1, 		0},
	{0, 					0, 						32, 			16, 		0, 			0.1, 		0},
	{0, 					0, 						48, 			16, 		0, 			0.1, 		0},
	//Message ID 6
	{0, 					0, 						0, 				16, 		0, 			1, 			0},
	{0, 					0, 						16, 			16, 		0, 			1, 			0},
	{0, 					0, 						32, 			16, 		0, 			1, 			0},
	{0, 					0, 						48, 			16, 		0, 			1, 			0},
	//Message ID 7
	{0, 					0, 						0, 				16, 		0, 			1, 			0},
	{0, 					0, 						16, 			16, 		0, 			1, 			0},
	{0, 					0, 						32, 			16, 		0, 			1, 			0},
	{0, 					0, 						48, 			16, 		0, 			1, 			0},
	//Message ID 8
	{0, 					0, 						0, 				16, 		0, 			1, 			0},
	{0, 					0, 						16, 			16, 		0, 			1, 			0},
	{0, 					0, 						32, 			16, 		0, 			1, 			0},
	{0, 					0, 						48, 			16, 		0, 			1, 			0},
	//Message ID 9
	{0, 					0, 						0, 				16, 		0, 			1, 			0},
	{0, 					0, 						16, 			16, 		0, 			1, 			0},
	{0, 					0, 						32, 			16, 		0, 			1, 			0},
	{0, 					0, 						48, 			16, 		0, 			1, 			0},
	//Message ID 10
	{0, 					0, 						0, 				16, 		0, 			1, 			0},
	{0, 					0, 						16, 			16, 		0, 			1, 			0},
	{0, 					0, 						32, 			16, 		0, 			1, 			0},
	{0, 					0, 						48, 			16, 		0, 			1, 			0},
	//Message ID 11
	{0, 					0, 						0, 				16, 		0, 			1, 			0},
	{0, 					0, 						16, 			16, 		0, 			1, 			0},
	{0, 					0, 						32, 			16, 		0, 			1, 			0},
	{0, 					0, 						48, 			16, 		0, 			1, 			0},
	//Message ID 12
	{0, 					0, 						0, 				16, 		0, 			1, 			0},
	{0, 					0, 						16, 			16, 		0, 			1, 			0},
	//Message ID 13
	{0, 					0, 						0, 				2, 			0, 			1, 			0},
	{0, 					0, 						2, 				2, 			0, 			1, 			0},
	{0, 					0, 						4, 				2, 			0, 			1, 			0},
	{0, 					0, 						6, 				2, 			0, 			1, 			0},
	{0, 					0, 						8, 				2, 			0, 			1, 			0},
	{0, 					0, 						10,				2, 			0, 			1, 			0},
	{0, 					0, 						12,				2, 			0, 			1, 			0},
	{0, 					0, 						14,				2, 			0, 			1, 			0},
	{0, 					0, 						16,				2, 			0, 			1, 			0},
	{0, 					0, 						18,				2, 			0, 			1, 			0},
	{0, 					0, 						20,				2, 			0, 			1, 			0},
	{0, 					0, 						22,				2, 			0, 			1, 			0},
	{0, 					0, 						24,				2, 			0, 			1, 			0},
	{0, 					0, 						26,				2, 			0, 			1, 			0},
	{0, 					0, 						28,				2, 			0, 			1, 			0},
	{0, 					0, 						30,				2, 			0, 			1, 			0},
	{0, 					0, 						32,				2, 			0, 			1, 			0},
	{0, 					0, 						34,				1, 			0, 			1, 			0},
	//Message ID 14
	{0, 					0, 						0, 				1, 			0, 			1, 			0},
	{0, 					0, 						4, 				1, 			0, 			1, 			0},
	{0, 					0, 						8, 				8, 			0, 			1, 			0},
	{0, 					0, 						32, 			16,			0, 			0.1,		0},
	{0, 					0, 						48, 			8, 			0, 			1, 			0},
	{0, 					0, 						56, 			8, 			0, 			1, 			0},
	//Message ID 15
	{0, 					0, 						0, 				16, 		0, 			0.001,	0},
	{0, 					0, 						16, 			16, 		0, 			0.001,	0},
	{0, 					0, 						32, 			8, 			0, 			1, 			-40},
	{0, 					0, 						40, 			8, 			0, 			1, 			-40},
	//Message ID 16
	{0, 					0, 						0, 				16, 		1, 			0.1,		0},
	{0, 					0, 						16, 			16, 		1, 			0.1,		0},
	//Message ID 17
	{1, 					0, 						0, 				8,	 		0, 			1,			0},
	{1, 					0, 						16, 			8,	 		0, 			1,			0},
	{1, 					0, 						24, 			16,	 		0, 			0.1,		-32000},
	{1, 					0, 						40, 			16, 		0, 			0.1, 		0},
	{1, 					0, 						56, 			8, 			0, 			1, 			0},
	//Message ID 18
	{1, 					0, 						0, 				16,	 		0, 			0.001,	0},
	{1, 					0, 						16, 			16,	 		0, 			0.001,	0},
	{1, 					0, 						32, 			8,	 		0, 			1,			-40},
	{1, 					0, 						40, 			8,	 		0, 			1, 			-40},
	//Message ID 19
	{1, 					0, 						0, 				8,	 		0, 			1,			0},
	{1, 					0, 						8, 				16,	 		0, 			0.1,		-10000},
	{1, 					0, 						24, 			8,	 		0, 			1,			0},
	{1, 					0, 						32, 			8,	 		0, 			1,			-40},
	{1, 					0, 						40, 			8,	 		0, 			1, 			-40},
	{1, 					0, 						48, 			2,	 		0, 			1,			0},
	{1, 					0, 						50, 			2,	 		0, 			1,			0},
	{1, 					0, 						52, 			2,	 		0, 			1,			0},
	{1, 					0, 						54, 			2,	 		0, 			1,			0},
	{1, 					0, 						56, 			2,	 		0, 			1,			0},
	{1, 					0, 						58, 			2,	 		0, 			1,			0},
	{1, 					0, 						60, 			2,	 		0, 			1,			0},
	{1, 					0, 						62, 			2,	 		0, 			1,			0},
	//Message ID20
	{1, 					0, 						0, 				16,	 		0, 			0.001,			0},
	{1, 					0, 						16, 			8,	 		0, 			1,			0},
	{1, 					0, 						24, 			16,	 		0, 			0.001,			0},
	{1, 					0, 						40, 			8,	 		0, 			1,			0},
	{1, 					0, 						48, 			16,	 		0, 			0.01, 	0},
	//Message ID21
	{1, 					0, 						32,				8,	 		0, 			1,			0},
	//Message ID22
	{1, 					0, 						0, 				16,	 		0, 			0.1,		0},
	{1, 					0, 						16,				16,	 		0, 			0.1,		-1000},
	{1, 					0, 						32, 			8,	 		0, 			1,			0},
	{1, 					0, 						40, 			8,	 		0, 			1,			0},
	{1, 					0, 						48, 			8,	 		0, 			1,			0},
	//Message ID23
	{0, 					0, 						0, 				16,	 		0, 			0.001,			0},
	{0, 					0, 						16,				8,	 		0, 			1,			0},
	{0, 					0, 						24,				16,	 		0, 			0.001,			0},
	{0, 					0, 						40, 			8,	 		0, 			1,			0},
	//Message ID24
	{0, 					0, 						0, 				16,	 		0, 			0.1,		0},
	{0, 					0, 						32,				16,	 		0, 			0.1,		-30000},
	{0, 					0, 						48, 			16,	 		0, 			0.1,		0},
	//Message ID25
	{1, 					0, 						0, 				16,	 		0, 			0.1,		-1000},
	{1, 					0, 						16,				16,	 		0, 			0.1,		-1000},
	{1, 					0, 						32,				16,	 		0, 			0.1,		-1000},
	//Message ID26
	{1, 					0, 						0, 				16,	 		0, 			0.001,	0},
	{1, 					0, 						16,				16,	 		0, 			0.001,	0},
	{1, 					0, 						32,				8,	 		0, 			0.1,		-40},
	{1, 					0, 						40,				8,	 		0, 			0.1,		-40},
	//Message ID27
	{1, 					0, 						0, 				1,	 		0, 			1,			0},
	{1, 					0, 						1, 				1,	 		0, 			1,			0},
	{1, 					0, 						2, 				1,	 		0, 			1,			0},
	{1, 					0, 						3, 				1,	 		0, 			1,			0},
	{1, 					0, 						4, 				1,	 		0, 			1,			0},
	{1, 					0, 						5, 				1,	 		0, 			1,			0},
	{1, 					0, 						8, 				8,	 		0, 			1,			0},
	{1, 					0, 						32,				16,	 		0, 			0.1,		0},
	{1, 					0, 						48,				8,	 		0, 			1,			0},
	{1, 					0, 						56,				8,	 		0, 			1,			0},
	//Message ID28
	{1, 					0, 						0, 				16,	 		0, 			0.001,	0},
	{1, 					0, 						16, 			8,	 		0, 			1,			0},
	{1, 					0, 						24, 			8,	 		0, 			1,			0},
	{1, 					0, 						32, 			16,	 		0, 			0.001,	0},
	{1, 					0, 						48, 			8,	 		0, 			1,			0},
	{1, 					0, 						56, 			8,	 		0, 			1,			0},
	//Message ID29
	{1, 					0, 						0, 				8,	 		1, 			1,			-40},
	{1, 					0, 						8, 				8,	 		0, 			1,			0},
	{1, 					0, 						16, 			8,	 		0, 			1,			0},
	{1, 					0, 						24, 			8,	 		1, 			1,			-40},
	{1, 					0, 						32, 			8,	 		0, 			1,			0},
	{1, 					0, 						40, 			8,	 		0, 			1,			0},
	//Message ID30
	{1, 					0, 						0, 				8,	 		0, 			1,			0},
	{1, 					0, 						8, 				8,	 		0, 			1,			0},
	{1, 					0, 						16, 			8,	 		0, 			1,			0},
	{1, 					0, 						24, 			8,	 		0, 			1,			0},
	{1, 					0, 						32, 			8,	 		0, 			1,			0},
	{1, 					0, 						40, 			8,	 		0, 			1,			0},
	//Message ID31
	{1, 					0, 						0, 				16,	 		0, 			0.1,		-1000},
	{1, 					0, 						16, 			16,	 		0, 			0.1,		-1000},
	{1, 					0, 						32, 			16,	 		0, 			0.1,		-1000},
	{1, 					0, 						48, 			16,	 		0, 			1,			0},
	//Message ID32
	{1, 					0, 						0, 				2,	 		0, 			1,			0},
	{1, 					0, 						2, 				2,	 		0, 			1,			0},
	{1, 					0, 						4, 				2,	 		0, 			1,			0},
	{1, 					0, 						6, 				2,	 		0, 			1,			0},
	{1, 					0, 						8, 				2,	 		0, 			1,			0},
	{1, 					0, 						10,				2,	 		0, 			1,			0},
	{1, 					0, 						12,				2,	 		0, 			1,			0},
	{1, 					0, 						14,				2,	 		0, 			1,			0},
	{1, 					0, 						16,				2,	 		0, 			1,			0},
	{1, 					0, 						18,				2,	 		0, 			1,			0},
	{1, 					0, 						20,				2,	 		0, 			1,			0},
	{1, 					0, 						22,				2,	 		0, 			1,			0},
	{1, 					0, 						24,				2,	 		0, 			1,			0},
	{1, 					0, 						26,				2,	 		0, 			1,			0},
	{1, 					0, 						28,				2,	 		0, 			1,			0},
	{1, 					0, 						32,				2,	 		0, 			1,			0},
	{1, 					0, 						40,				2,	 		0, 			1,			0},
	//Message ID33
	{1, 					0, 						0, 				16,	 		0, 			1,			0},
	{1, 					0, 						16,				8,	 		0, 			1,			0},
	{1, 					0, 						24,				8,	 		0, 			1,			0},
	{1, 					0, 						32,				8,	 		0, 			1,			0},
	{1, 					0, 						40,				8,	 		0, 			1,			0},
	//Message ID34
	{1, 					0, 						0, 				16,	 		0, 			0.1,		0},
	{1, 					0, 						16, 			16,	 		0, 			0.01,		0},
	{1, 					0, 						32, 			16,	 		0, 			0.1,		0},
	{1, 					0, 						48, 			8,	 		0, 			1,			0},
	{1, 					0, 						56, 			1,	 		0, 			1,			0},
	{1, 					0, 						57, 			1,	 		0, 			1,			0},
	//Message ID35
	{1, 					0, 						0, 				16,	 		0, 			1,			0},
	{1, 					0, 						16, 			8,	 		0, 			1,			0},
	{1, 					0, 						24, 			8,	 		0, 			1,			0},
	{1, 					0, 						32, 			8,	 		0, 			1,			0},
	{1, 					0, 						40, 			8,	 		0, 			1,			0},
	{1, 					0, 						48, 			16,	 		0, 			1,			0},
	//Message ID36
	{0, 					0, 						0, 				8,	 		0, 			1,			0},
	{1, 					0, 						8, 				8,	 		0, 			1,			0},
	{1, 					0, 						16, 			8,	 		0, 			1,			0},
	{1, 					0, 						24, 			8,	 		0, 			1,			0},
	{1, 					0, 						32, 			8,	 		0, 			1,			0},
	{1, 					0, 						40, 			8,	 		0, 			1,			0},
	{1, 					0, 						48, 			8,	 		0, 			1,			0},
	{1, 					0, 						56, 			8,	 		0, 			1,			0},
	//Message ID37
	{1, 					0, 						0, 				8,	 		0, 			1,			0},
	{1, 					0, 						8, 				8,	 		0, 			1,			0},
	{1, 					0, 						16, 			8,	 		0, 			1,			0},
	{1, 					0, 						24, 			8,	 		0, 			1,			0},
	{1, 					0, 						32, 			8,	 		0, 			1,			0},
	{1, 					0, 						40, 			8,	 		0, 			1,			0},
	{1, 					0, 						48, 			8,	 		0, 			1,			0},
	{1, 					0, 						56, 			8,	 		0, 			1,			0},
	//Message ID38
	{1, 					0, 						0, 				8,	 		0, 			1,			0},
	{1, 					0, 						8, 				8,	 		0, 			1,			0},
	{1, 					0, 						16, 			8,	 		0, 			1,			0},
	{1, 					0, 						24, 			8,	 		0, 			1,			0},
	{1, 					0, 						32, 			8,	 		0, 			1,			0},
	{1, 					0, 						40, 			8,	 		0, 			1,			0},
	{1, 					0, 						48, 			8,	 		0, 			1,			0},
	{1, 					0, 						56, 			8,	 		0, 			1,			0},
	//Message ID39
	{0, 					0, 						0, 				16,	 		1, 			0.1,		0},
	{0, 					0, 						16, 			16,	 		1, 			0.1,		-1000},
	{0, 					0, 						32, 			16,	 		0, 			0.1,		0},
	{0, 					0, 						48, 			16,	 		0, 			0.1,		0},
	//Message ID40
	{0, 					0, 						8, 				8,	 		0, 			1,			0},
	//Message ID41
	{1, 					0, 						0, 				1,	 		0, 			1,			0},
	{1, 					0, 						1, 				1,	 		0, 			1,			0},
	{1, 					0, 						2, 				1,	 		0, 			1,			0},
	{1, 					0, 						3, 				1,	 		0, 			1,			0},
	{1, 					0, 						4, 				1,	 		0, 			1,			0},
	{1, 					0, 						5, 				1,	 		0, 			1,			0},
	{1, 					0, 						6, 				1,	 		0, 			1,			0},
	{1, 					0, 						7, 				1,	 		0, 			1,			0},
	{1, 					0, 						8, 				1,	 		0, 			1,			0},
	{1, 					0, 						9, 				1,	 		0, 			1,			0},
	{1, 					0, 						10, 			1,	 		0, 			1,			0},
	{1, 					0, 						12, 			1,	 		0, 			1,			0},
	{1, 					0, 						13, 			1,	 		0, 			1,			0},
	{1, 					0, 						14, 			1,	 		0, 			1,			0},
	{1, 					0, 						16, 			1,	 		0, 			1,			0},
	{1, 					0, 						17, 			1,	 		0, 			1,			0},
	{1, 					0, 						18, 			1,	 		0, 			1,			0},
	{1, 					0, 						19, 			1,	 		0, 			1,			0},
	{1, 					0, 						20, 			1,	 		0, 			1,			0},
	{1, 					0, 						21, 			1,	 		0, 			1,			0},
	{1, 					0, 						22, 			1,	 		0, 			1,			0},
	{1, 					0, 						23, 			1,	 		0, 			1,			0},
	{1, 					0, 						24, 			1,	 		0, 			1,			0},
	{1, 					0, 						25, 			1,	 		0, 			1,			0},
	{1, 					0, 						32, 			1,	 		0, 			1,			0},
	{1, 					0, 						33, 			1,	 		0, 			1,			0},
	{1, 					0, 						34, 			1,	 		0, 			1,			0},
	{1, 					0, 						35, 			1,	 		0, 			1,			0},
	{1, 					0, 						37, 			1,	 		0, 			1,			0},
	{1, 					0, 						38, 			1,	 		0, 			1,			0},
	{1, 					0, 						39, 			1,	 		0, 			1,			0},
	{1, 					0, 						40, 			1,	 		0, 			1,			0},
	{1, 					0, 						41, 			1,	 		0, 			1,			0},
	{1, 					0, 						42, 			1,	 		0, 			1,			0},
	{1, 					0, 						43, 			1,	 		0, 			1,			0},
	{1, 					0, 						44, 			1,	 		0, 			1,			0},
	{1, 					0, 						45, 			1,	 		0, 			1,			0},
	{1, 					0, 						48, 			4,	 		0, 			1,			0},
	{1, 					0, 						52, 			4,	 		0, 			1,			0},
	{1, 					0, 						56, 			4,	 		0, 			1,			0},
	//Message ID42
	{1, 					0, 						0, 				16,	 		0, 			0.1,		0},
	{1, 					0, 						16, 			16,	 		0, 			0.1,		-500},
	{1, 					0, 						32, 			8,	 		0, 			1,			0},
	{1, 					0, 						40, 			8,	 		0, 			1,			-50},
	{1, 					0, 						48, 			8,	 		0, 			1,			-50},
	{1, 					0, 						56, 			8,	 		0, 			1,			-50},
	//Message ID43
	{1, 					0, 						0, 				2,	 		0, 			1,			0},
	{1, 					0, 						4, 				1,	 		0, 			1,			0},
	{1, 					0, 						5, 				1,	 		0, 			1,			0},
	{1, 					0, 						8, 				3,	 		0, 			1,			0},
	{1, 					0, 						16, 			3,	 		0, 			1,			0},
	{1, 					0, 						24, 			16,	 		0, 			1,			0},
	{1, 					0, 						40, 			8,	 		0, 			1,			0},
	//Message ID44
	{0, 					1, 						0, 				16,	 		0, 			0.01,		0},
	{0, 					1, 						16, 			16,	 		1, 			0.01,		0},
	{0, 					1, 						32, 			16,	 		1, 			0.1,		0},
	{0, 					1, 						48, 			16,	 		1, 			0.1,		0},
	//Message ID45
	{0, 					1, 						0, 				8,	 		1, 			1,			0},
	{0, 					1, 						8, 				8,	 		1, 			1,			0},
	{0, 					1, 						16, 			16,	 		0, 			0.01,		0},
	{0, 					1, 						32, 			16,	 		0, 			0.01,		0},
	{0, 					1, 						48, 			16,	 		0, 			0.01,		0},
	//Message ID46
	{0, 					1, 						24, 			8,	 		0, 			1,			0},
	{0, 					1, 						32, 			16,	 		0, 			0.1,		0},
	{0, 					1, 						48, 			16,	 		0, 			0.1,		0},
	//Message ID47
	{0, 					1, 						0, 				16,	 		0, 			0.1,		0},
	{0, 					1, 						16, 			16,	 		0, 			0.1,		0},
	{0, 					1, 						32, 			16,	 		0, 			0.1,		0},
	{0, 					1, 						48, 			16,	 		0, 			0.1,		0},
	//Message ID48
	{0, 					1, 						0, 				16,	 		0, 			0.1,		0},
	{0, 					1, 						16, 			16,	 		0, 			0.1,		0},
	{0, 					1, 						32, 			16,	 		0, 			0.1,		0},
	{0, 					1, 						48, 			16,	 		0, 			0.1,		0},
	//Message ID49
	{0, 					1, 						0, 				16,	 		0, 			0.1,		0},
	{0, 					1, 						16, 			16,	 		0, 			0.1,		0},
	{0, 					1, 						32, 			16,	 		0, 			0.1,		0},
	{0, 					1, 						48, 			16,	 		0, 			0.1,		0},
	//Message ID50
	{0, 					1, 						0, 				16,	 		0, 			0.1,		0},
	{0, 					1, 						16, 			16,	 		0, 			0.1,		0},
	{0, 					1, 						32, 			16,	 		0, 			0.1,		0},
	{0, 					1, 						48, 			16,	 		0, 			0.1,		0},
	//Message ID51
	{0, 					1, 						0, 				16,	 		0, 			0.1,		0},
	{0, 					1, 						16, 			16,	 		0, 			0.1,		0},
	{0, 					1, 						32, 			16,	 		0, 			0.1,		0},
	{0, 					1, 						48, 			16,	 		0, 			0.1,		0},
	//Message ID52
	{0, 					1, 						0, 				16,	 		0, 			0.1,		0},
	{0, 					1, 						16, 			16,	 		0, 			0.1,		0},
	{0, 					1, 						32, 			16,	 		0, 			0.1,		0},
	{0, 					1, 						48, 			16,	 		0, 			0.1,		0},
	//Message ID53
	{0, 					1, 						0, 				8,	 		0, 			1,			-40},
	{0, 					1, 						32, 			16,	 		0, 			0.1,		0},
	{0, 					1, 						48, 			16,	 		0, 			0.1,		0},
	//Message ID54
	{0, 					1, 						32, 			16,	 		0, 			0.1,		0},
	{0, 					1, 						48, 			16,	 		0, 			0.1,		0},
	//Message ID55
	{0, 					0, 						0, 				16,	 		0, 			1,			0},
	{0, 					0, 						16, 			16,	 		0, 			1,			0},
	{0, 					0, 						32, 			16,	 		0, 			1,			0},
	{0, 					0, 						48, 			16,	 		0, 			1,			0},
	//Message ID56
	{0, 					0, 						0, 				16,	 		0, 			1,			0},
	{0, 					0, 						16, 			16,	 		0, 			1,			0},
	{0, 					0, 						32, 			16,	 		0, 			1,			0},
	{0, 					0, 						48, 			16,	 		0, 			1,			0},
	//Message ID57
	{0, 					0, 						0, 				16,	 		0, 			1,			0},
	{0, 					0, 						16, 			16,	 		0, 			1,			0},
	{0, 					0, 						32, 			16,	 		0, 			1,			0},
	{0, 					0, 						48, 			16,	 		0, 			1,			0},
	//Message ID58
	{0, 					0, 						0, 				16,	 		0, 			1,			0},
	{0, 					0, 						16, 			16,	 		0, 			1,			0},
	{0, 					0, 						32, 			16,	 		0, 			1,			0},
	{0, 					0, 						48, 			16,	 		0, 			1,			0},
	//Message ID59
	{1, 					0, 						0, 				16,	 		0, 			1,			0},
	{1, 					0, 						16, 			16,	 		0, 			1,			0},
	//Message ID60
	{1, 					0, 						0, 				16,	 		0, 			1,			0},
	{1, 					0, 						16, 			16,	 		0, 			1,			0},
	{1, 					0, 						32, 			8,	 		0, 			1,			-40},
	{1, 					0, 						40, 			8,	 		0, 			1,			-40},
	{1,           0,            48,       16,     0,      1,      0},
	//Message ID61
	{1, 					0, 						7, 				1,	 		0, 			1,			0},
	{1, 					0, 						8, 				8,	 		0, 			1,			0},
	{1, 					0, 						16, 			16,	 		0, 			0.1,		-500},
	{1, 					0, 						32, 			16,	 		0, 			0.1,		0},
	//Message ID62
	{1, 					0, 						0, 			16,	 		0, 			0.1,		-500},
	{1, 					0, 						16, 		16,	 		0, 			1,			0},
	{1, 					0, 						32, 		16,	 		0, 			0,		0},
	{1, 					0, 						48, 		8,	 		0, 			0,		0},
	//Message ID63
	{1, 					0, 						0, 			16,	 		0, 			0.1,		0},
	{1, 					0, 						16, 		16,	 		0, 			0.1,		0},
	//Message ID64
	{0, 					0, 						0, 				16,	 		0, 			1,			0},
	{0, 					0, 						16, 			16,	 		0, 			1,			0},
	{0, 					0, 						32, 			16,	 		0, 			1,			0},
	{0, 					0, 						48, 			16,	 		0, 			1,			0},
	//Message ID65
	{0, 					0, 						0, 				16,	 		0, 			1,			0},
	{0, 					0, 						16, 			16,	 		0, 			1,			0},
	{0, 					0, 						32, 			16,	 		0, 			1,			0},
	{0, 					0, 						48, 			16,	 		0, 			1,			0},
};

const CAN_FRAME_CFG_S g_astCanFrameCfg[] = {//原500ms改为250才是准的500ms发送
//CanId					DataLen		Period
	{0xE005, 			8, 				80},
	{0x1806e5f4, 	8, 				80},
	{0x18ff50e5, 	8, 				80},
	{0xE000, 			8, 				80},
	{0xE004, 			8, 				80},
	{0xE006, 			8, 				80},
	{0xE007, 			8, 				80},
	{0xE008, 			8, 				80},
	{0xE009, 			8, 				80},
	{0xE010, 			8, 				80},
	{0xE011, 			8, 				80},
	{0xE012, 			8, 				80},
	{0xF001, 			8, 				80},
	{0x18FF28F4,	8,				80},
	{0x18FE28F4,	8, 				80},
	{0x18FD28F4,	8,				80},
	{0x4d4,				8,				80},
	{0x4d5,				8,				80},
	{0x108,				8,				80},
	{0x110,				8,				80},
	{0x111,				8,				80},
	{0x18F810F3,	8,				80},
	{0x18914010,	8,				80},
	{0x18904010,	8,				80},
	{0x18FA28F4,	8,				80},
	{0x18FB28F4,	8,				80},
	{0x18FC28F4,	8,				80},
	{0x18F811F3,	8,				500},
	{0x18F812F3,	8,				500},
	{0x18F813F3,	8,				500},
	{0x18F814F3,	8,				500},
	{0x18F815F3,	8,				80},
	{0x18F880F3,	8,				500},//33
	{0x273,				8,				80},
	{0xF000,			8,				500},
	{0xE013,			8,				500},
	{0xE014,			8,				500},
	{0xE015,			8,				500},
	{0x1E1,				8,				80},
	{0x1F5,				8,				80},
	{0x0800A6A9,	8,				80},
	{0x1000A6A9, 	8, 				80},
	{0x1C00A6A9,	8,				80},
	{0xE000,			8,				80},
	{0xE001,			8,				80},
	{0xE002,			8,				80},
	{0xE015,			8,				80},
	{0xE016,			8,				80},
	{0xE017,			8,				80},
	{0xE018,			8,				80},
	{0xE019,			8,				80},
	{0xE020,			8,				80},
	{0xE005,			8,				80},
	{0xE006,			8,				80},
	{0x18C828F4,	8,				80},
	{0x18C928F4,	8,				80},
	{0x18CA28F4,	8,				80},
	{0x18CB28F4,	8,				80},
	{0x18FD28F4,	8,				80},
	{0x18FE28F4,	8,				80},
	{0x18FF28F4,	8,				80},
	{0x18FFFFFF,	8,				80},
	{0x18FFE5F4,	8,				80},
	{0x18CC28F4,	8,				80},
	{0x18CD28F4,	8,				80}
};

uint8_t g_aaucSBuf[sizeof(g_astCanFrameCfg) / sizeof(CAN_FRAME_CFG_S)][8] = {0};

const uint64_t g_aullCanFramesCfg[][2] = {
	{0x000007FFFFFFFFFF, 0x0000000000000000},
	{0x003FF8000000F802, 0x0000000000000000},
	{0x1FC00000000011E2, 0x0000000000000000}
};

float ptc_eco_get(const uint8_t* const  pucData, const enum ePtcEcoCanVal eIdx) {
	if(NULL == pucData || eIdx >= E_PTC_ECO_CV_NUM) {
		return 0;
	}
	CAN_VAL_CFG_S stCanValCfg = g_astCanValCfg[eIdx];
	uint64_t ullVal = ((*(uint64_t*)pucData) >> stCanValCfg.usStartBit) & ~(0xFFFFFFFFFFFFFFFF << stCanValCfg.usBitNum);
	if(stCanValCfg.usByteInversion != 0) {
		ullVal =  (ullVal & 0xFF) << 8
						| (ullVal & 0xFF00) >> 8
						| (ullVal & 0xFF0000) << 8
						| (ullVal & 0xFF000000) >> 8
						| (ullVal & 0xFF00000000) >> 8
						| (ullVal & 0xFF0000000000) << 8
						| (ullVal & 0xFF000000000000) >> 8
						| (ullVal & 0xFF00000000000000) << 8;
	}
	if(stCanValCfg.usWordInversion != 0) {
		ullVal = 	(ullVal & 0xFFFF) << 16
						| (ullVal & 0xFFFF0000) >> 16
						|	(ullVal & 0xFFFF00000000) << 16
						| (ullVal & 0xFFFF000000000000) >> 16;
	}
	if(stCanValCfg.usSigned != 0) {
		if((ullVal >> (stCanValCfg.usStartBit + stCanValCfg.usBitNum -1) & 0x01) != 0) {
			ullVal &= ~(0x0000000000000001 << (stCanValCfg.usStartBit + stCanValCfg.usBitNum - 1));
			ullVal |= 0x8000000000000000;
		}
	}
	return ullVal * stCanValCfg.fFacA + stCanValCfg.fFacB;
}

void ptc_eco_set(const float fVal, const enum ePtcEcoCanVal eIdx, uint16_t sIdx) {
	if(eIdx >= E_PTC_ECO_CV_NUM) {
		return;
	}
	CAN_VAL_CFG_S stCanValCfg = g_astCanValCfg[eIdx];
	if(stCanValCfg.fFacA == 0) {
		stCanValCfg.fFacA = 1;
		stCanValCfg.fFacB = 0;
	}
	uint64_t ullVal;
	if(stCanValCfg.usSigned != 0) {
		if(fVal - stCanValCfg.fFacB >= 0) {
			ullVal = (fVal - stCanValCfg.fFacB) / stCanValCfg.fFacA;
		} else {
			ullVal = (stCanValCfg.fFacB - fVal) / stCanValCfg.fFacA;
			ullVal = ~ullVal + 1;
//			ullVal |= (0x0000000000000001 << (stCanValCfg.usStartBit + stCanValCfg.usBitNum - 1));
		}
	} else if(eIdx == E_PTC_ECO_CV17_24 || eIdx == E_PTC_ECO_CV17_40 || eIdx == E_PTC_ECO_CV19_8 || eIdx == E_PTC_ECO_CV20_48 || eIdx == E_PTC_ECO_CV17_40 || eIdx == E_PTC_ECO_CV22_0
		|| eIdx == E_PTC_ECO_CV22_16 || eIdx == E_PTC_ECO_CV24_32 || eIdx == E_PTC_ECO_CV24_48 || eIdx == E_PTC_ECO_CV31_0 || eIdx == E_PTC_ECO_CV31_16 || eIdx == E_PTC_ECO_CV31_32
		|| eIdx == E_PTC_ECO_CV31_48 || eIdx == E_PTC_ECO_CV33_0 || eIdx == E_PTC_ECO_CV33_16 || eIdx == E_PTC_ECO_CV33_24 || eIdx == E_PTC_ECO_CV33_32 || eIdx == E_PTC_ECO_CV33_40
		) {
		ullVal = fVal / stCanValCfg.fFacA - stCanValCfg.fFacB;
	}	else {
		ullVal = (fVal - stCanValCfg.fFacB) / stCanValCfg.fFacA;
	}
	if(stCanValCfg.usBitNum >= 16 && stCanValCfg.usByteInversion == 0 && stCanValCfg.usWordInversion == 0) {
		uint8_t ucVala = ullVal & 0xFF;
		uint8_t ucValb = ullVal >> 8 & 0xFF;
		ullVal = ucVala * 256 + ucValb;
	}
	if(stCanValCfg.usWordInversion == 0) {
	//	uint64_t ullSData = ((uint64_t)g_aaucSBuf[sIdx][0] << 56)
	//										| ((uint64_t)g_aaucSBuf[sIdx][1] << 48)
	//										| ((uint64_t)g_aaucSBuf[sIdx][2] << 40)
	//										| ((uint64_t)g_aaucSBuf[sIdx][3] << 32)
	//										| ((uint64_t)g_aaucSBuf[sIdx][4] << 24)
	//										| ((uint64_t)g_aaucSBuf[sIdx][5] << 16)
	//										| ((uint64_t)g_aaucSBuf[sIdx][6] << 8)
	//										| ((uint64_t)g_aaucSBuf[sIdx][7]);
		uint64_t ullSData = ((uint64_t)g_aaucSBuf[sIdx][0])
											| ((uint64_t)g_aaucSBuf[sIdx][1] << 8)
											| ((uint64_t)g_aaucSBuf[sIdx][2] << 16)
											| ((uint64_t)g_aaucSBuf[sIdx][3] << 24)
											| ((uint64_t)g_aaucSBuf[sIdx][4] << 32)
											| ((uint64_t)g_aaucSBuf[sIdx][5] << 40)
											| ((uint64_t)g_aaucSBuf[sIdx][6] << 48)
											| ((uint64_t)g_aaucSBuf[sIdx][7] << 56);
		if(eIdx == E_PTC_ECO_CV61_7) {
			ullSData &= ~(~(0xFFFFFFFFFFFFFFFF << stCanValCfg.usBitNum));
			ullSData |= (ullVal & (~(0xFFFFFFFFFFFFFFFF << stCanValCfg.usBitNum)));
		} else {
			ullSData &= ~(~(0xFFFFFFFFFFFFFFFF << stCanValCfg.usBitNum) << stCanValCfg.usStartBit);
			ullSData |= ((ullVal & (~(0xFFFFFFFFFFFFFFFF << stCanValCfg.usBitNum))) << stCanValCfg.usStartBit);
		}
	//	g_aaucSBuf[sIdx][0] = ullSData >> 56;
		g_aaucSBuf[sIdx][0] = ullSData & 0xFF;
	//	g_aaucSBuf[sIdx][1] = ullSData >> 48 & 0xFF;
		g_aaucSBuf[sIdx][1] = ullSData >> 8 & 0xFF;
	//	g_aaucSBuf[sIdx][2] = ullSData >> 40 & 0xFF;
		g_aaucSBuf[sIdx][2] = ullSData >> 16 & 0xFF;
	//	g_aaucSBuf[sIdx][3] = ullSData >> 32 & 0xFF;
		g_aaucSBuf[sIdx][3] = ullSData >> 24 & 0xFF;
	//	g_aaucSBuf[sIdx][4] = ullSData >> 24 & 0xFF;
		g_aaucSBuf[sIdx][4] = ullSData >> 32 & 0xFF;
	//	g_aaucSBuf[sIdx][5] = ullSData >> 16 & 0xFF;
		g_aaucSBuf[sIdx][5] = ullSData >> 40 & 0xFF;
	//	g_aaucSBuf[sIdx][6] = ullSData >> 8 & 0xFF;
		g_aaucSBuf[sIdx][6] = ullSData >> 48 & 0xFF;
	//	g_aaucSBuf[sIdx][7] = ullSData & 0xFF;
		g_aaucSBuf[sIdx][7] = ullSData >> 56;
	}
	if(stCanValCfg.usWordInversion == 1) {
		uint64_t ullSData = ((uint64_t)g_aaucSBuf[sIdx][0] << 56)
										| ((uint64_t)g_aaucSBuf[sIdx][1] << 48)
										| ((uint64_t)g_aaucSBuf[sIdx][2] << 40)
										| ((uint64_t)g_aaucSBuf[sIdx][3] << 32)
										| ((uint64_t)g_aaucSBuf[sIdx][4] << 24)
										| ((uint64_t)g_aaucSBuf[sIdx][5] << 16)
										| ((uint64_t)g_aaucSBuf[sIdx][6] << 8)
										| ((uint64_t)g_aaucSBuf[sIdx][7]);
		ullSData &= ~(~(0xFFFFFFFFFFFFFFFF << stCanValCfg.usBitNum) << stCanValCfg.usStartBit);
		ullSData |= ((ullVal & (~(0xFFFFFFFFFFFFFFFF << stCanValCfg.usBitNum))) << stCanValCfg.usStartBit);
		g_aaucSBuf[sIdx][0] = ullSData >> 56;
		g_aaucSBuf[sIdx][1] = ullSData >> 48 & 0xFF;
		g_aaucSBuf[sIdx][2] = ullSData >> 40 & 0xFF;
		g_aaucSBuf[sIdx][3] = ullSData >> 32 & 0xFF;
		g_aaucSBuf[sIdx][4] = ullSData >> 24 & 0xFF;
		g_aaucSBuf[sIdx][5] = ullSData >> 16 & 0xFF;
		g_aaucSBuf[sIdx][6] = ullSData >> 8 & 0xFF;
		g_aaucSBuf[sIdx][7] = ullSData & 0xFF;
	}
}

void ptc_eco_init(void) {
}

void ptc_eco_proc(void) {
	if(g_stCfg.stCom.ucCanPtcType >= sizeof(g_aullCanFramesCfg) / sizeof(uint64_t) * 2) {
		return;
	}
	
	/* put variables into can send buff, a lot... */
	LOCAL_PACK_RVAL_S* pstPackRVal = g_stLocalArrayRVal.astPackRVal + g_stPrl.ucSelfId;
	{
		ptc_eco_set(pstPackRVal->fPeakLmtDsgI, E_PTC_ECO_CV1_0, 0);
		ptc_eco_set(pstPackRVal->fMosT, E_PTC_ECO_CV1_16, 0);
		ptc_eco_set(pstPackRVal->fLmtDsgI, E_PTC_ECO_CV1_32, 0);
		ptc_eco_set(pstPackRVal->fLmtChgI, E_PTC_ECO_CV1_48, 0);
		ptc_eco_set(pstPackRVal->fPackSoc, E_PTC_ECO_CV2_0, 1);
		ptc_eco_set(pstPackRVal->fPackSoc, E_PTC_ECO_CV4_0, 3);
		ptc_eco_set(pstPackRVal->fPackSoc, E_PTC_ECO_CV4_16, 3);
		ptc_eco_set(pstPackRVal->fPackU, E_PTC_ECO_CV4_32, 3);
		ptc_eco_set(pstPackRVal->fPackCur, E_PTC_ECO_CV4_48, 3);
		ptc_eco_set(pstPackRVal->fCellTMax, E_PTC_ECO_CV5_0, 4);
		ptc_eco_set(pstPackRVal->fCellTMin, E_PTC_ECO_CV5_8, 4);
		ptc_eco_set(pstPackRVal->fPackRealAH, E_PTC_ECO_CV5_16, 4);
		ptc_eco_set(g_stCfg.stLocal.usDesignAH, E_PTC_ECO_CV5_32, 4);
		ptc_eco_set(pstPackRVal->fPackLeftAH, E_PTC_ECO_CV5_48, 4);
		ptc_eco_set(pstPackRVal->afCellU[0] * 1000, E_PTC_ECO_CV6_0, 5);
		ptc_eco_set(pstPackRVal->afCellU[1] * 1000, E_PTC_ECO_CV6_16, 5);
		ptc_eco_set(pstPackRVal->afCellU[2] * 1000, E_PTC_ECO_CV6_32, 5);
		ptc_eco_set(pstPackRVal->afCellU[3] * 1000, E_PTC_ECO_CV6_48, 5);
		ptc_eco_set(pstPackRVal->afCellU[4] * 1000, E_PTC_ECO_CV7_0, 6);
		ptc_eco_set(pstPackRVal->afCellU[5] * 1000, E_PTC_ECO_CV7_16, 6);
		ptc_eco_set(pstPackRVal->afCellU[6] * 1000, E_PTC_ECO_CV7_32, 6);
		ptc_eco_set(pstPackRVal->afCellU[7] * 1000, E_PTC_ECO_CV7_48, 6);
		ptc_eco_set(pstPackRVal->afCellU[8] * 1000, E_PTC_ECO_CV8_0, 7);
		ptc_eco_set(pstPackRVal->afCellU[9] * 1000, E_PTC_ECO_CV8_16, 7);
		ptc_eco_set(pstPackRVal->afCellU[10] * 1000, E_PTC_ECO_CV8_32, 7);
		ptc_eco_set(pstPackRVal->afCellU[11] * 1000, E_PTC_ECO_CV8_48, 7);
		ptc_eco_set(pstPackRVal->afCellU[12] * 1000, E_PTC_ECO_CV9_0, 8);
		ptc_eco_set(pstPackRVal->afCellU[13] * 1000, E_PTC_ECO_CV9_16, 8);
		ptc_eco_set(pstPackRVal->afCellU[14] * 1000, E_PTC_ECO_CV9_32, 8);
		ptc_eco_set(pstPackRVal->afCellU[15] * 1000, E_PTC_ECO_CV9_48, 8);
		ptc_eco_set(0, E_PTC_ECO_CV10_0, 9);
		ptc_eco_set(0, E_PTC_ECO_CV10_16, 9);
		ptc_eco_set(0, E_PTC_ECO_CV10_32, 9);
		ptc_eco_set(0, E_PTC_ECO_CV10_48, 9);
		ptc_eco_set(0, E_PTC_ECO_CV11_0, 10);
		ptc_eco_set(0, E_PTC_ECO_CV11_16, 10);
		ptc_eco_set(0, E_PTC_ECO_CV11_32, 10);
		ptc_eco_set(0, E_PTC_ECO_CV11_48, 10);
		ptc_eco_set(pstPackRVal->fPackU * 1000 / CFG_CELL_NUM, E_PTC_ECO_CV12_0, 11);
		ptc_eco_set((pstPackRVal->fCellUMax - pstPackRVal->fCellUMin) * 1000, E_PTC_ECO_CV12_16, 11);
		if(GET_ALM0_CODE(1)) {
			ptc_eco_set(0b01, E_PTC_ECO_CV13_0, 12);
		} else {
			ptc_eco_set(0b00, E_PTC_ECO_CV13_0, 12);
		}
		if(GET_ALM0_CODE(2)) {
			ptc_eco_set(0b01, E_PTC_ECO_CV13_2, 12);
		} else {
			ptc_eco_set(0b00, E_PTC_ECO_CV13_2, 12);
		}
		if(GET_ALM0_CODE(3)) {
			ptc_eco_set(0b01, E_PTC_ECO_CV13_4, 12);
		} else {
			ptc_eco_set(0b00, E_PTC_ECO_CV13_4, 12);
		}
		if(GET_ALM0_CODE(4)) {
			ptc_eco_set(0b10, E_PTC_ECO_CV13_6, 12);
		} else {
			ptc_eco_set(0b00, E_PTC_ECO_CV13_6, 12);
		}
		if(GET_ALM0_CODE(5)) {
			ptc_eco_set(0b10, E_PTC_ECO_CV13_8, 12);
		} else {
			ptc_eco_set(0b00, E_PTC_ECO_CV13_8, 12);
		}
//		if(GET_ALM0_CODE(6)) {
//			ptc_eco_set(0b10, E_PTC_ECO_CV13_10, 12);
//		} else {
//			ptc_eco_set(0b00, E_PTC_ECO_CV13_10, 12);
//		}
		if(GET_ALM0_CODE(7)) {
			ptc_eco_set(0b10, E_PTC_ECO_CV13_12, 12);
		} else {
			ptc_eco_set(0b00, E_PTC_ECO_CV13_12, 12);
		}
		if(GET_ALM0_CODE(8)) {
			ptc_eco_set(0b11, E_PTC_ECO_CV13_14, 12);
		} else {
			ptc_eco_set(0b00, E_PTC_ECO_CV13_14, 12);
		}
		if(GET_ALM0_CODE(9)) {
			ptc_eco_set(0b11, E_PTC_ECO_CV13_16, 12);
		} else {
			ptc_eco_set(0b00, E_PTC_ECO_CV13_16, 12);
		}
		if(GET_ALM0_CODE(10)) {
			ptc_eco_set(0b11, E_PTC_ECO_CV13_18, 12);
		} else {
			ptc_eco_set(0b00, E_PTC_ECO_CV13_18, 12);
		}
		if(GET_ALM0_CODE(11)) {
			ptc_eco_set(0b11, E_PTC_ECO_CV13_20, 12);
		} else {
			ptc_eco_set(0b00, E_PTC_ECO_CV13_20, 12);
		}
		if(GET_ALM0_CODE(12)) {
			ptc_eco_set(0b11, E_PTC_ECO_CV13_22, 12);
		} else {
			ptc_eco_set(0b00, E_PTC_ECO_CV13_22, 12);
		}
		if(GET_ALM0_CODE(13)) {
			ptc_eco_set(0b11, E_PTC_ECO_CV13_24, 12);
		} else {
			ptc_eco_set(0b00, E_PTC_ECO_CV13_24, 12);
		}
		if(GET_ALM0_CODE(14)) {
			ptc_eco_set(0b11, E_PTC_ECO_CV13_26, 12);
		} else {
			ptc_eco_set(0b00, E_PTC_ECO_CV13_26, 12);
		}
		if(GET_ALM0_CODE(15)) {
			ptc_eco_set(0b11, E_PTC_ECO_CV13_28, 12);
		} else {
			ptc_eco_set(0b00, E_PTC_ECO_CV13_28, 12);
		}
		if(GET_ALM0_CODE(16)) {
			ptc_eco_set(0b11, E_PTC_ECO_CV13_30, 12);
		} else {
			ptc_eco_set(0b00, E_PTC_ECO_CV13_30, 12);
		}
		if(GET_ALM0_CODE(17)) {
			ptc_eco_set(0b11, E_PTC_ECO_CV13_32, 12);
		} else {
			ptc_eco_set(0b00, E_PTC_ECO_CV13_32, 12);
		}
		ptc_eco_set(0, E_PTC_ECO_CV13_34, 12);
		if(pstPackRVal->uBaseStat.stBaseStat.ucChgerON) {
			ptc_eco_set(1, E_PTC_ECO_CV14_0, 13);
		} else {
			ptc_eco_set(0, E_PTC_ECO_CV14_0, 13);
		}
		if(g_stAfe.uRam.stCode.DSG_FET && pstPackRVal->ucDsgForceEn == 0) {
			ptc_eco_set(1, E_PTC_ECO_CV14_4, 13);
		} else {
			ptc_eco_set(0, E_PTC_ECO_CV14_4, 13);
		}
		ptc_eco_set(pstPackRVal->fPackSoc, E_PTC_ECO_CV14_8, 13);
		ptc_eco_set(pstPackRVal->fPackU, E_PTC_ECO_CV14_32, 13);
		ptc_eco_set(g_ucAlmLeve3, E_PTC_ECO_CV14_48, 13);
		{
			static uint8_t aucBuf = 0;
			static uint8_t s_uctick = 1, uc_tick = 0;
			for(uint8_t i=s_uctick; i<65; i++){
				if(i == 6 || i == 12 || (i > 17 && i < 53) || (i > 57 && i < 64)) {
					continue;
				}
				if(GET_ALM1_CODE(i)) {
					aucBuf = i;
					ptc_eco_set(i, E_PTC_ECO_CV14_56, 13);
					delay_1ms(2);	
					s_uctick = i;
					uc_tick++;
					if(uc_tick > 10){
						uc_tick = 0;
						s_uctick = i + 1;
					}
					if(s_uctick == 65){
						s_uctick = 1;
					}
					break;
				}		
			}
			if(aucBuf == 0) {
				if(s_uctick == 1){
					ptc_eco_set(0, E_PTC_ECO_CV14_56, 13);
					delay_1ms(2);
				}
				s_uctick = 1;
			}
			aucBuf = 0;
		}
		ptc_eco_set(pstPackRVal->fCellUMax, E_PTC_ECO_CV15_0, 14);
		ptc_eco_set(pstPackRVal->fCellUMin, E_PTC_ECO_CV15_16, 14);
		ptc_eco_set(pstPackRVal->fCellTMax, E_PTC_ECO_CV15_32, 14);
		ptc_eco_set(pstPackRVal->fCellTMin, E_PTC_ECO_CV15_40, 14);
		ptc_eco_set(pstPackRVal->fPeakLmtDsgI, E_PTC_ECO_CV16_0, 15);
		ptc_eco_set(pstPackRVal->fLmtChgI, E_PTC_ECO_CV16_16, 15);
		{
			static uint8_t aucBuf = 0;
			static uint8_t s_uctick = 1, uc_tick = 0;
			for(uint8_t i=s_uctick; i<65; i++){
				if(i == 6 || i == 12 || (i > 17 && i < 53) || (i > 57 && i < 64)) {
					continue;
				}
				if(GET_ALM1_CODE(i)) {
					aucBuf = i;
					ptc_eco_set(i, E_PTC_ECO_CV17_0, 16);
					delay_1ms(2);	
					s_uctick = i;
					uc_tick++;
					if(uc_tick > 10){
						uc_tick = 0;
						s_uctick = i + 1;
					}
					if(s_uctick == 65){
						s_uctick = 1;
					}
					break;
				}		
			}
			if(aucBuf == 0) {
				if(s_uctick == 1){
					ptc_eco_set(0, E_PTC_ECO_CV17_0, 16);
					delay_1ms(2);
				}
				s_uctick = 1;
			}
			aucBuf = 0;
		}
		ptc_eco_set(pstPackRVal->fPackSoc, E_PTC_ECO_CV17_16, 16);
		ptc_eco_set(pstPackRVal->fPackCur, E_PTC_ECO_CV17_24, 16);
		ptc_eco_set(pstPackRVal->fPackU, E_PTC_ECO_CV17_40, 16);
		if(g_bMChgerComAct == true) {
			ptc_eco_set(1, E_PTC_ECO_CV17_56, 16);
		} else {
			ptc_eco_set(3, E_PTC_ECO_CV17_56, 16);
		}
		ptc_eco_set(pstPackRVal->fCellUMin, E_PTC_ECO_CV18_0, 17);
		ptc_eco_set(pstPackRVal->fCellUMax, E_PTC_ECO_CV18_16, 17);
		ptc_eco_set(pstPackRVal->fCellTMin, E_PTC_ECO_CV18_32, 17);
		ptc_eco_set(pstPackRVal->fCellTMax, E_PTC_ECO_CV18_40, 17);
		ptc_eco_set(1, E_PTC_ECO_CV19_0, 18);
		ptc_eco_set(pstPackRVal->fPackCur, E_PTC_ECO_CV19_8, 18);
		ptc_eco_set(pstPackRVal->fPackSoc, E_PTC_ECO_CV19_24, 18);
		ptc_eco_set(pstPackRVal->fCellTMax, E_PTC_ECO_CV19_32, 18);
		ptc_eco_set(pstPackRVal->fCellTMin, E_PTC_ECO_CV19_40, 18);
		if(pstPackRVal->ucChgEn == 0x55 && (pstPackRVal->ucChgForceEn == 0x55 || pstPackRVal->ucChgForceEn == 0)) {
			ptc_eco_set(0b01, E_PTC_ECO_CV19_48, 18);
		} else {
			ptc_eco_set(0b00, E_PTC_ECO_CV19_48, 18);
		}
		if(pstPackRVal->ucDsgEn == 0x55 && (pstPackRVal->ucDsgForceEn == 0x55 || pstPackRVal->ucDsgForceEn == 0)) {
			ptc_eco_set(0b01, E_PTC_ECO_CV19_50, 18);
		} else {
			ptc_eco_set(0b00, E_PTC_ECO_CV19_50, 18);
		}
		if(pstPackRVal->uBaseStat.stBaseStat.ucHeating) {
			ptc_eco_set(0b01, E_PTC_ECO_CV19_52, 18);
		} else {
			ptc_eco_set(0b00, E_PTC_ECO_CV19_52, 18);
		}
		if(pstPackRVal->uBaseStat.stBaseStat.ucDCOut1) {
			ptc_eco_set(0b01, E_PTC_ECO_CV19_54, 18);
		} else {
			ptc_eco_set(0b00, E_PTC_ECO_CV19_54, 18);
		}
		if(prl_host()) {
			ptc_eco_set(0b01, E_PTC_ECO_CV19_48, 18);
			for(uint8_t i=0;i<g_stPrl.ucDevNum;i++) {
				if(g_ausPrlComTick[i] == 0) {
					ptc_eco_set(0b11, E_PTC_ECO_CV19_48, 18);
					break;
				}
			}
		}
		ptc_eco_set(0b00, E_PTC_ECO_CV19_56, 18);
		ptc_eco_set(0b00, E_PTC_ECO_CV19_58, 18);
		ptc_eco_set(0b00, E_PTC_ECO_CV19_60, 18);
		if(pstPackRVal->uBaseStat.stBaseStat.ucChgerON) {
			ptc_eco_set(0b11, E_PTC_ECO_CV19_62, 18);
		} else {
			ptc_eco_set(0b00, E_PTC_ECO_CV19_62, 18);
		}
		ptc_eco_set(pstPackRVal->fCellUMax, E_PTC_ECO_CV20_0, 19);
		ptc_eco_set(pstPackRVal->ucCellUMaxId + 1, E_PTC_ECO_CV20_16, 19);
		ptc_eco_set(pstPackRVal->fCellUMin, E_PTC_ECO_CV20_24, 19);
		ptc_eco_set(pstPackRVal->ucCellUMinId + 1, E_PTC_ECO_CV20_40, 19);
		ptc_eco_set(pstPackRVal->fPackU, E_PTC_ECO_CV20_48, 19);
		ptc_eco_set(0, E_PTC_ECO_CV21_32, 20);
		{
			for(uint8_t i=1; i<17; i++){
				if(i==4 || i==6 || i==8 || i==9 || i==12){
					continue;
				}
				if(GET_ALM0_CODE(i)){
					ptc_eco_set(0xFF, E_PTC_ECO_CV21_32, 20);
				}
			}
		}
		ptc_eco_set(pstPackRVal->fPackU, E_PTC_ECO_CV22_0, 21);
		ptc_eco_set(pstPackRVal->fPackCur, E_PTC_ECO_CV22_16, 21);
		ptc_eco_set(pstPackRVal->fPackSoc, E_PTC_ECO_CV22_32, 21);
		ptc_eco_set(pstPackRVal->fPackSoh, E_PTC_ECO_CV22_40, 21);
		if(pstPackRVal->uBaseStat.stBaseStat.ucChgerON == 0) {
			ptc_eco_set(1, E_PTC_ECO_CV22_48, 21);
		} else {
			ptc_eco_set(0, E_PTC_ECO_CV22_48, 21);
		}
		ptc_eco_set(pstPackRVal->fCellUMax, E_PTC_ECO_CV23_0, 22);
		ptc_eco_set(pstPackRVal->ucCellUMaxId + 1, E_PTC_ECO_CV23_16, 22);
		ptc_eco_set(pstPackRVal->fCellUMin, E_PTC_ECO_CV23_24, 22);
		ptc_eco_set(pstPackRVal->ucCellUMinId + 1, E_PTC_ECO_CV23_40, 22);
		ptc_eco_set(pstPackRVal->fPackU, E_PTC_ECO_CV24_0, 23);
		ptc_eco_set(pstPackRVal->fPackCur, E_PTC_ECO_CV24_32, 23);
		ptc_eco_set(pstPackRVal->fPackSoc, E_PTC_ECO_CV24_48, 23);
		ptc_eco_set(pstPackRVal->fPeakLmtDsgI, E_PTC_ECO_CV25_0, 24);
		ptc_eco_set(pstPackRVal->fLmtChgI, E_PTC_ECO_CV25_16, 24);
		ptc_eco_set(pstPackRVal->fPackCur, E_PTC_ECO_CV25_32, 24);
		ptc_eco_set(pstPackRVal->fCellUMax, E_PTC_ECO_CV26_0, 25);
		ptc_eco_set(pstPackRVal->fCellUMin, E_PTC_ECO_CV26_16, 25);
		ptc_eco_set(pstPackRVal->fCellTMax, E_PTC_ECO_CV26_32, 25);
		ptc_eco_set(pstPackRVal->fCellTMin, E_PTC_ECO_CV26_40, 25);
		if(pstPackRVal->uBaseStat.stBaseStat.ucChgerON) {
			ptc_eco_set(1, E_PTC_ECO_CV27_0, 26);
		} else {
			ptc_eco_set(0, E_PTC_ECO_CV27_0, 26);
		}
		if(pstPackRVal->uBaseStat.stBaseStat.ucChg) {
			ptc_eco_set(1, E_PTC_ECO_CV27_1, 26);
		} else {
			ptc_eco_set(0, E_PTC_ECO_CV27_1, 26);
		}
		if(pstPackRVal->fCellUMin < 2.6) {
			ptc_eco_set(1, E_PTC_ECO_CV27_2, 26);
		} else {
			ptc_eco_set(0, E_PTC_ECO_CV27_2, 26);
		}
		if(g_stLocalArrayRVal.eLocalStat == eLocalStatRun) {
			ptc_eco_set(1, E_PTC_ECO_CV27_3, 26);
		} else {
			ptc_eco_set(0, E_PTC_ECO_CV27_3, 26);
		}
		if(g_stAfe.uRam.stCode.DSG_FET && pstPackRVal->ucDsgForceEn == 0) {
			ptc_eco_set(1, E_PTC_ECO_CV27_4, 26);
		} else {
			ptc_eco_set(0, E_PTC_ECO_CV27_4, 26);
		}
		if(g_stAfe.uRam.stCode.CHG_FET && pstPackRVal->ucChgForceEn == 0) {
			ptc_eco_set(1, E_PTC_ECO_CV27_5, 26);
		} else {
			ptc_eco_set(0, E_PTC_ECO_CV27_5, 26);
		}
		ptc_eco_set(pstPackRVal->fPackSoc, E_PTC_ECO_CV27_8, 26);
		ptc_eco_set(pstPackRVal->fPackU, E_PTC_ECO_CV27_32, 26);
		ptc_eco_set(g_ucAlmLeve3, E_PTC_ECO_CV27_48, 26);
		{
			static uint8_t aucBuf = 0;
			static uint8_t s_uctick = 1, uc_tick = 0;
			for(uint8_t i=s_uctick; i<65; i++){
				if(i == 6 || i == 12 || (i > 17 && i < 53) || (i > 57 && i < 64)) {
					continue;
				}
				if(GET_ALM1_CODE(i)) {
					aucBuf = i;
					ptc_eco_set(i, E_PTC_ECO_CV27_56, 26);
					delay_1ms(2);	
					s_uctick = i;
					uc_tick++;
					if(uc_tick > 10){
						uc_tick = 0;
						s_uctick = i + 1;
					}
					if(s_uctick == 65){
						s_uctick = 1;
					}
					break;
				}		
			}
			if(aucBuf == 0) {
				if(s_uctick == 1){
					ptc_eco_set(0, E_PTC_ECO_CV27_56, 26);
					delay_1ms(2);
				}
				s_uctick = 1;
			}
			aucBuf = 0;
		}
		ptc_eco_set(g_stLocalArrayRVal.fCellUMax, E_PTC_ECO_CV28_0, 27);
		ptc_eco_set(g_stLocalArrayRVal.usCellUMaxId + 1, E_PTC_ECO_CV28_16, 27);
		ptc_eco_set((g_stLocalArrayRVal.usCellUMaxId >> 8) + 1, E_PTC_ECO_CV28_24, 27);
		ptc_eco_set(g_stLocalArrayRVal.fCellUMin, E_PTC_ECO_CV28_32, 27);
		ptc_eco_set(g_stLocalArrayRVal.usCellUMinId + 1, E_PTC_ECO_CV28_48, 27);
		ptc_eco_set((g_stLocalArrayRVal.usCellUMinId >> 8) + 1, E_PTC_ECO_CV28_56, 27);
		ptc_eco_set(g_stLocalArrayRVal.fPackTMax, E_PTC_ECO_CV29_0, 28);
		ptc_eco_set(g_stLocalArrayRVal.usPackTMaxId + 1, E_PTC_ECO_CV29_8, 28);
		ptc_eco_set((g_stLocalArrayRVal.usPackTMaxId >> 8) + 1, E_PTC_ECO_CV29_16, 28);
		ptc_eco_set(g_stLocalArrayRVal.fPackTMin, E_PTC_ECO_CV29_24, 28);
		ptc_eco_set(g_stLocalArrayRVal.usPackTMinId + 1, E_PTC_ECO_CV29_32, 28);
		ptc_eco_set((g_stLocalArrayRVal.usPackTMinId >> 8) + 1, E_PTC_ECO_CV29_40, 28);
		if(g_bMChgerComAct || g_bSChgerComAct) {
			ptc_eco_set(0, E_PTC_ECO_CV30_0, 29);
		} else {
			ptc_eco_set(1, E_PTC_ECO_CV30_0, 29);
		}
		ptc_eco_set(1, E_PTC_ECO_CV30_8, 29);
		ptc_eco_set(0, E_PTC_ECO_CV30_16, 29);
		if(pstPackRVal->uBaseStat.stBaseStat.ucChg) {
			ptc_eco_set(1, E_PTC_ECO_CV30_16, 29);
		}
		if(g_ucChgFull == 1) {
			ptc_eco_set(2, E_PTC_ECO_CV30_16, 29);
		}
		if(g_stAfe.uRam.stCode.DSG_FET && pstPackRVal->ucDsgForceEn == 0) {
			ptc_eco_set(1, E_PTC_ECO_CV30_24, 29);
			ptc_eco_set(1, E_PTC_ECO_CV30_32, 29);
		} else {
			ptc_eco_set(0, E_PTC_ECO_CV30_24, 29);
			ptc_eco_set(0, E_PTC_ECO_CV30_32, 29);
		}
		if(g_stAfe.uRam.stCode.CHG_FET && pstPackRVal->ucChgForceEn == 0) {
			ptc_eco_set(1, E_PTC_ECO_CV30_40, 29);
		} else {
			ptc_eco_set(0, E_PTC_ECO_CV30_40, 29);
		}
		ptc_eco_set(pstPackRVal->fReqChgI, E_PTC_ECO_CV31_0, 30);
		ptc_eco_set(pstPackRVal->fPeakLmtDsgI, E_PTC_ECO_CV31_16, 30);
		ptc_eco_set(pstPackRVal->fLmtChgI, E_PTC_ECO_CV31_32, 30);
		ptc_eco_set(g_stCfg.stLocal.usDesignAH, E_PTC_ECO_CV31_48, 30);
		if(GET_ALM1_CODE(56)) {
			ptc_eco_set(0b10, E_PTC_ECO_CV32_0, 31);
		} else {
			ptc_eco_set(0b00, E_PTC_ECO_CV32_0, 31);
		}
		if(GET_ALM1_CODE(7)) {
			ptc_eco_set(0b11, E_PTC_ECO_CV32_2, 31);
		} else if(GET_ALM1_CODE(57)) {
			ptc_eco_set(0b10, E_PTC_ECO_CV32_2, 31);
		} else {
			ptc_eco_set(0b00, E_PTC_ECO_CV32_2, 31);
		}
		if(GET_ALM1_CODE(4)) {
			ptc_eco_set(0b11, E_PTC_ECO_CV32_4, 31);
		} else if(GET_ALM1_CODE(54)) {
			ptc_eco_set(0b10, E_PTC_ECO_CV32_4, 31);
		} else {
			ptc_eco_set(0b00, E_PTC_ECO_CV32_4, 31);
		}
		if(GET_ALM1_CODE(5)) {
			ptc_eco_set(0b11, E_PTC_ECO_CV32_6, 31);
		} else if(GET_ALM1_CODE(55)) {
			ptc_eco_set(0b10, E_PTC_ECO_CV32_6, 31);
		} else {
			ptc_eco_set(0b00, E_PTC_ECO_CV32_6, 31);
		}
		if(GET_ALM1_CODE(3)) {
			ptc_eco_set(0b11, E_PTC_ECO_CV32_8, 31);
		} else if(GET_ALM1_CODE(53)) {
			ptc_eco_set(0b10, E_PTC_ECO_CV32_8, 31);
		} else {
			ptc_eco_set(0b00, E_PTC_ECO_CV32_8, 31);
		}
		if(GET_ALM1_CODE(10) || GET_ALM1_CODE(11)) {
			ptc_eco_set(0b11, E_PTC_ECO_CV32_10, 31);
		} else {
			ptc_eco_set(0b00, E_PTC_ECO_CV32_10, 31);
		}
//		if(GET_ALM1_CODE(12) || GET_ALM1_CODE(13)) {
//			ptc_eco_set(0b11, E_PTC_ECO_CV32_12, 31);
//		} else {
			ptc_eco_set(0b00, E_PTC_ECO_CV32_12, 31);
//		}
		if(pstPackRVal->fCellTMax - pstPackRVal->fCellTMin > 20) {
			ptc_eco_set(0b11, E_PTC_ECO_CV32_14, 31);
		} else {
			ptc_eco_set(0b00, E_PTC_ECO_CV32_14, 31);
		}
		if(GET_ALM0_CODE(9)) {
			ptc_eco_set(0b11, E_PTC_ECO_CV32_16, 31);
		} else {
			ptc_eco_set(0b00, E_PTC_ECO_CV32_16, 31);
		}
		if(GET_ALM0_CODE(8)) {
			ptc_eco_set(0b11, E_PTC_ECO_CV32_18, 31);
		} else {
			ptc_eco_set(0b00, E_PTC_ECO_CV32_18, 31);
		}
		ptc_eco_set(0b00, E_PTC_ECO_CV32_20, 31);
		if(GET_ALM0_CODE(15) || GET_ALM1_CODE(15)) {
			ptc_eco_set(0b11, E_PTC_ECO_CV32_22, 31);
		} else {
			ptc_eco_set(0b00, E_PTC_ECO_CV32_22, 31);
		}
		ptc_eco_set(0b00, E_PTC_ECO_CV32_24, 31);
		ptc_eco_set(0b00, E_PTC_ECO_CV32_26, 31);
		ptc_eco_set(0b00, E_PTC_ECO_CV32_28, 31);
		ptc_eco_set(0b00, E_PTC_ECO_CV32_26, 31);
		ptc_eco_set(CFG_CELL_NUM, E_PTC_ECO_CV32_32, 31);
		ptc_eco_set(CFG_TMP_NUM, E_PTC_ECO_CV32_40, 31);
		{
			static uint16_t aucBuf = 0;
			if(g_BalanceFlag == 1) {
				switch(pstPackRVal->ucCellUMaxId + 1){
					case 1:aucBuf = 0x0001;break;
					case 2:aucBuf = 0x0002;break;
					case 3:aucBuf = 0x0004;break;
					case 4:aucBuf = 0x0008;break;
					case 5:aucBuf = 0x0010;break;
					case 6:aucBuf = 0x0020;break;
					case 7:aucBuf = 0x0040;break;
					case 8:aucBuf = 0x0080;break;
					case 9:aucBuf = 0x0100;break;
					case 10:aucBuf = 0x0200;break;
					case 11:aucBuf = 0x0400;break;
					case 12:aucBuf = 0x0800;break;
					case 13:aucBuf = 0x1000;break;
					case 14:aucBuf = 0x2000;break;
					case 15:aucBuf = 0x4000;break;
					case 16:aucBuf = 0x8000;break;
				}
			} else {
				aucBuf = 0;
			}
			ptc_eco_set(aucBuf, E_PTC_ECO_CV35_0, 34);
			ptc_eco_set(0, E_PTC_ECO_CV35_16, 34);
			ptc_eco_set(0, E_PTC_ECO_CV35_24, 34);
			ptc_eco_set(0, E_PTC_ECO_CV35_32, 34);
			ptc_eco_set(0, E_PTC_ECO_CV35_40, 34);
			ptc_eco_set(0, E_PTC_ECO_CV35_48, 34);
		}
		{
			static uint8_t aucData[14];
			memcpy(aucData, g_stCfg.stLocal.aucBleName, 14);
			ptc_eco_set(aucData[0], E_PTC_ECO_CV36_0, 35);
			ptc_eco_set(aucData[1], E_PTC_ECO_CV36_8, 35);
			ptc_eco_set(aucData[2], E_PTC_ECO_CV36_16, 35);
			ptc_eco_set(aucData[3], E_PTC_ECO_CV36_24, 35);
			ptc_eco_set(aucData[4], E_PTC_ECO_CV36_32, 35);
			ptc_eco_set(aucData[5], E_PTC_ECO_CV36_40, 35);
			ptc_eco_set(aucData[6], E_PTC_ECO_CV36_48, 35);
			ptc_eco_set(aucData[7], E_PTC_ECO_CV36_56, 35);
			ptc_eco_set(aucData[8], E_PTC_ECO_CV37_0, 36);
			ptc_eco_set(aucData[9], E_PTC_ECO_CV37_8, 36);
			ptc_eco_set(aucData[10], E_PTC_ECO_CV37_16, 36);
			ptc_eco_set(aucData[11], E_PTC_ECO_CV37_24, 36);
			ptc_eco_set(aucData[12], E_PTC_ECO_CV37_32, 36);
			ptc_eco_set(aucData[13], E_PTC_ECO_CV37_40, 36);
			ptc_eco_set(0, E_PTC_ECO_CV37_48, 36);
			ptc_eco_set(0, E_PTC_ECO_CV37_56, 36);
			ptc_eco_set(0, E_PTC_ECO_CV38_0, 37);
			ptc_eco_set(0, E_PTC_ECO_CV38_8, 37);
			ptc_eco_set(0, E_PTC_ECO_CV38_16, 37);
			ptc_eco_set(0, E_PTC_ECO_CV38_24, 37);
			ptc_eco_set(0, E_PTC_ECO_CV38_32, 37);
			ptc_eco_set(0, E_PTC_ECO_CV38_40, 37);
			ptc_eco_set(0, E_PTC_ECO_CV38_48, 37);
			ptc_eco_set(0x03, E_PTC_ECO_CV38_56, 37);
		}
		ptc_eco_set(pstPackRVal->fPackU, E_PTC_ECO_CV39_0, 38);
		ptc_eco_set(pstPackRVal->fPackCur , E_PTC_ECO_CV39_16, 38);
		ptc_eco_set(pstPackRVal->fPackSoc, E_PTC_ECO_CV39_32, 38);
		ptc_eco_set(pstPackRVal->fPackSoc, E_PTC_ECO_CV39_48, 38);
		ptc_eco_set(g_ucAlmLevel2, E_PTC_ECO_CV40_8, 39);
		{
			static uint8_t s_ucF_I_Num = 0;
			static uint8_t s_ucF_II_Num = 0;
			static uint8_t s_ucF_III_Num = 0;
			if(GET_ALM0_CODE(9)) {
				ptc_eco_set(1, E_PTC_ECO_CV41_32, 40);
				s_ucF_III_Num++;
			} else {
				ptc_eco_set(0, E_PTC_ECO_CV41_32, 40);
			}
			if(GET_ALM0_CODE(8)) {
				ptc_eco_set(1, E_PTC_ECO_CV41_33, 40);
				s_ucF_III_Num++;
			} else {
				ptc_eco_set(0, E_PTC_ECO_CV41_33, 40);
			}
			if(GET_ALM0_CODE(6) || GET_ALM1_CODE(6)) {
				ptc_eco_set(1, E_PTC_ECO_CV41_34, 40);
				s_ucF_III_Num++;
			} 
//			else if(GET_ALM1_CODE(56)) {
//				ptc_eco_set(1, E_PTC_ECO_CV41_18, 40);
//				s_ucF_II_Num += 1;
//			} 
			else {
				ptc_eco_set(0, E_PTC_ECO_CV41_34, 40);
//				ptc_eco_set(0, E_PTC_ECO_CV41_18, 40);
			}
			if(GET_ALM0_CODE(7) || GET_ALM1_CODE(7)) {
				ptc_eco_set(1, E_PTC_ECO_CV41_35, 40);
				s_ucF_III_Num++;
			} 
//			else if(GET_ALM1_CODE(57)) {
//				ptc_eco_set(1, E_PTC_ECO_CV41_19, 40);
//				s_ucF_II_Num += 1;
//			} 
			else {
				ptc_eco_set(0, E_PTC_ECO_CV41_35, 40);
//				ptc_eco_set(0, E_PTC_ECO_CV41_19, 40);
			}
			ptc_eco_set(0, E_PTC_ECO_CV41_4, 40);
			if(GET_ALM0_CODE(15) || GET_ALM1_CODE(15)) {
				ptc_eco_set(1, E_PTC_ECO_CV41_37, 40);
				s_ucF_III_Num++;
			} else {
				ptc_eco_set(0, E_PTC_ECO_CV41_37, 40);
			}
			if(GET_ALM0_CODE(5) || GET_ALM1_CODE(5)) {
				ptc_eco_set(1, E_PTC_ECO_CV41_38, 40);
				s_ucF_III_Num++;
			} 
//			else if(GET_ALM1_CODE(55)) {
//				ptc_eco_set(1, E_PTC_ECO_CV41_22, 40);
//				s_ucF_II_Num += 1;
//			} 
			else {
				ptc_eco_set(0, E_PTC_ECO_CV41_38, 40);
//				ptc_eco_set(0, E_PTC_ECO_CV41_22, 40);
			}
			if(GET_ALM0_CODE(4) || GET_ALM1_CODE(4) || GET_ALM1_CODE(54)) {
				ptc_eco_set(1, E_PTC_ECO_CV41_39, 40);
				s_ucF_III_Num++;
			} 
//			else if(GET_ALM1_CODE(54)) {
//				ptc_eco_set(1, E_PTC_ECO_CV41_23, 40);
//				s_ucF_II_Num += 1;
//			} 
			else {
				ptc_eco_set(0, E_PTC_ECO_CV41_39, 40);
//				ptc_eco_set(0, E_PTC_ECO_CV41_23, 40);
			}
//			if(GET_ALM0_CODE(12) || GET_ALM1_CODE(12) || GET_ALM0_CODE(13) || GET_ALM1_CODE(13)) {
//				ptc_eco_set(1, E_PTC_ECO_CV41_8, 40);
//				s_ucF_III_Num += 1;
//			} else {
//				ptc_eco_set(0, E_PTC_ECO_CV41_8, 40);
//			}
			if(GET_ALM0_CODE(10) || GET_ALM1_CODE(10) || GET_ALM0_CODE(11) || GET_ALM1_CODE(11)) {
				ptc_eco_set(1, E_PTC_ECO_CV41_40, 40);
				s_ucF_III_Num++;
			} else {
				ptc_eco_set(0, E_PTC_ECO_CV41_40, 40);
			}
			ptc_eco_set(0, E_PTC_ECO_CV41_10, 40);
			if(GET_ALM0_CODE(17) || GET_ALM1_CODE(17)) {
				ptc_eco_set(1, E_PTC_ECO_CV41_45, 40);
				s_ucF_III_Num++;
			} else {
				ptc_eco_set(0, E_PTC_ECO_CV41_45, 40);
			}
			if(pstPackRVal->uErrCode.stErrCode.bTempSensor == 1) {
				ptc_eco_set(1, E_PTC_ECO_CV41_12, 40);
				s_ucF_I_Num++;
			} else {
				ptc_eco_set(0, E_PTC_ECO_CV41_12, 40);
			}
			ptc_eco_set(0, E_PTC_ECO_CV41_13, 40);
			if(pstPackRVal->uErrCode.stErrCode.bVoltSensor == 1) {
				ptc_eco_set(1, E_PTC_ECO_CV41_14, 40);
				s_ucF_I_Num++;
			} else {
				ptc_eco_set(0, E_PTC_ECO_CV41_14, 40);
			}
			ptc_eco_set(0, E_PTC_ECO_CV41_0, 40);
			ptc_eco_set(0, E_PTC_ECO_CV41_1, 40);
			ptc_eco_set(0, E_PTC_ECO_CV41_2, 40);
			ptc_eco_set(0, E_PTC_ECO_CV41_3, 40);
			ptc_eco_set(0, E_PTC_ECO_CV41_4, 40);
			ptc_eco_set(0, E_PTC_ECO_CV41_5, 40);
			ptc_eco_set(0, E_PTC_ECO_CV41_6, 40);
			ptc_eco_set(0, E_PTC_ECO_CV41_7, 40);
			ptc_eco_set(0, E_PTC_ECO_CV41_8, 40);
			ptc_eco_set(0, E_PTC_ECO_CV41_9, 40);
			ptc_eco_set(0, E_PTC_ECO_CV41_10, 40);
			ptc_eco_set(0, E_PTC_ECO_CV41_16, 40);
			ptc_eco_set(0, E_PTC_ECO_CV41_17, 40);
			ptc_eco_set(0, E_PTC_ECO_CV41_18, 40);
			ptc_eco_set(0, E_PTC_ECO_CV41_19, 40);
			ptc_eco_set(0, E_PTC_ECO_CV41_20, 40);
			ptc_eco_set(0, E_PTC_ECO_CV41_21, 40);
			ptc_eco_set(0, E_PTC_ECO_CV41_24, 40);
			ptc_eco_set(0, E_PTC_ECO_CV41_25, 40);
			ptc_eco_set(0, E_PTC_ECO_CV41_41, 40);
			ptc_eco_set(0, E_PTC_ECO_CV41_42, 40);
			ptc_eco_set(0, E_PTC_ECO_CV41_43, 40);
			ptc_eco_set(0, E_PTC_ECO_CV41_44, 40);
//			if(GET_ALM0_CODE(9)) {
//				ptc_eco_set(1, E_PTC_ECO_CV41_32, 40);
//				s_ucF_I_Num += 1;
//			} else {
//				ptc_eco_set(0, E_PTC_ECO_CV41_32, 40);
//			}
//			if(GET_ALM0_CODE(8)) {
//				ptc_eco_set(1, E_PTC_ECO_CV41_33, 40);
//				s_ucF_I_Num += 1;
//			} else {
//				ptc_eco_set(0, E_PTC_ECO_CV41_33, 40);
//			}
		ptc_eco_set(s_ucF_I_Num, E_PTC_ECO_CV41_48, 40);
		ptc_eco_set(s_ucF_II_Num, E_PTC_ECO_CV41_52, 40);
		ptc_eco_set(s_ucF_III_Num, E_PTC_ECO_CV41_56, 40);
		s_ucF_I_Num = 0;
		s_ucF_II_Num = 0;
		s_ucF_III_Num = 0;
		}
		ptc_eco_set(pstPackRVal->fPackU, E_PTC_ECO_CV42_0, 41);
		ptc_eco_set(pstPackRVal->fPackCur, E_PTC_ECO_CV42_16, 41);
		ptc_eco_set(pstPackRVal->fPackSoc, E_PTC_ECO_CV42_32, 41);
		ptc_eco_set(pstPackRVal->fCellTMax, E_PTC_ECO_CV42_40, 41);
		ptc_eco_set(pstPackRVal->fCellTMin, E_PTC_ECO_CV42_48, 41);
		{
			static uint16_t usVal = 0;
			for(uint8_t i = 0; i < CFG_TMP_NUM; i++) {
				usVal += pstPackRVal->afCellT[i];
			}
			ptc_eco_set(usVal / CFG_TMP_NUM, E_PTC_ECO_CV42_56, 41);
			usVal = 0;
		}
		if(pstPackRVal->uBaseStat.stBaseStat.ucChgerON == 1) {
			ptc_eco_set(0b10, E_PTC_ECO_CV43_0, 42);
		} else {
			ptc_eco_set(0b00, E_PTC_ECO_CV43_0, 42);
		}
		if(pstPackRVal->ucDsgEn == 0x55 || pstPackRVal->ucDsgForceEn == 0x55) {
			ptc_eco_set(1, E_PTC_ECO_CV43_4, 42);
		} else {
			ptc_eco_set(0, E_PTC_ECO_CV43_4, 42);
		}
		if(pstPackRVal->ucChgEn == 0x55 || pstPackRVal->ucChgForceEn ==0x55) {
			ptc_eco_set(1, E_PTC_ECO_CV43_5, 42);
		} else {
			ptc_eco_set(0, E_PTC_ECO_CV43_5, 42);
		}
		if(pstPackRVal->uBaseStat.stBaseStat.ucChgerON == 1 && pstPackRVal->uBaseStat.stBaseStat.ucHeating != 0) {
			ptc_eco_set(1, E_PTC_ECO_CV43_8, 42);
		} else {
			ptc_eco_set(0, E_PTC_ECO_CV43_8, 42);
		}
		if(g_eBaseStat == eBStatWorking && pstPackRVal->uBaseStat.stBaseStat.ucChgerON == 1) {
			ptc_eco_set(0x02, E_PTC_ECO_CV43_16, 42);
		} else if(g_eBaseStat == eBStatWorking && pstPackRVal->uBaseStat.stBaseStat.ucChgerON != 1) {
			ptc_eco_set(0x01, E_PTC_ECO_CV43_16, 42);
		} else {
			ptc_eco_set(0, E_PTC_ECO_CV43_16, 42);
		}
		if(pstPackRVal->uBaseStat.stBaseStat.ucChgerON == 1) {
			ptc_eco_set((pstPackRVal->fPackRealAH - pstPackRVal->fPackLeftAH) / pstPackRVal->fPackCur * 60, E_PTC_ECO_CV43_24, 42);
		} else {
			ptc_eco_set(0, E_PTC_ECO_CV43_24, 42);
		}
		ptc_eco_set(0x03, E_PTC_ECO_CV43_40, 42);
		ptc_eco_set(pstPackRVal->fPackSoc, E_PTC_ECO_CV44_0, 43);
		ptc_eco_set(pstPackRVal->fPackSoc, E_PTC_ECO_CV44_16, 43);
		ptc_eco_set(pstPackRVal->fPackU, E_PTC_ECO_CV44_32, 43);
		ptc_eco_set(pstPackRVal->fPackCur, E_PTC_ECO_CV44_48, 43);
		ptc_eco_set(pstPackRVal->fCellTMin, E_PTC_ECO_CV45_0, 44);
		ptc_eco_set(pstPackRVal->fCellTMax, E_PTC_ECO_CV45_8, 44);
		{
			static uint16_t usVal = 0;
			for(uint8_t i = 0; i < CFG_CELL_NUM; i++) {
				usVal += pstPackRVal->afCellU[i] * 100;
			}
			ptc_eco_set(usVal / CFG_CELL_NUM, E_PTC_ECO_CV45_16, 44);
			usVal = 0;
		}
		ptc_eco_set(pstPackRVal->fCellUMax * 100, E_PTC_ECO_CV45_32, 44);
		ptc_eco_set(pstPackRVal->fCellUMin * 100, E_PTC_ECO_CV45_48, 44);
		if(pstPackRVal->fPackSoc < 100) {
			ptc_eco_set(1, E_PTC_ECO_CV46_24, 45);
			ptc_eco_set(0, E_PTC_ECO_CV46_32, 45);
		} else {
			ptc_eco_set(0, E_PTC_ECO_CV46_24, 45);
			ptc_eco_set(pstPackRVal->fReqChgI, E_PTC_ECO_CV46_32, 45);
		}
		if(round(pstPackRVal->fCellTMin) < g_stCfg.stLocal.sCellUCTTVThr1 || pstPackRVal->uErrCode.stErrCode.bTempSensor) {
			ptc_eco_set(0, E_PTC_ECO_CV46_24, 45);
			ptc_eco_set(0, E_PTC_ECO_CV46_32, 45);
		}
		if(g_bSetMChgerAct && pstPackRVal->uBaseStat.stBaseStat.ucHeating != 0) {
			ptc_eco_set(1, E_PTC_ECO_CV46_24, 45);
			ptc_eco_set(5, E_PTC_ECO_CV46_32, 45);
			if(pstPackRVal->fCellTMin > 2) {
				ptc_eco_set(pstPackRVal->fReqChgI, E_PTC_ECO_CV46_32, 1);
			}
		}
		if(g_bSChgerComAct && !g_bMChgerComAct) {
			ptc_eco_set(0, E_PTC_ECO_CV46_24, 45);
			ptc_eco_set(0, E_PTC_ECO_CV46_32, 45);
		}
		ptc_eco_set(56, E_PTC_ECO_CV46_48, 45);
		ptc_eco_set(pstPackRVal->afCellU[0] * 1000, E_PTC_ECO_CV47_48, 46);
		ptc_eco_set(pstPackRVal->afCellU[1] * 1000, E_PTC_ECO_CV47_32, 46);
		ptc_eco_set(pstPackRVal->afCellU[2] * 1000, E_PTC_ECO_CV47_16, 46);
		ptc_eco_set(pstPackRVal->afCellU[3] * 1000, E_PTC_ECO_CV47_0, 46);
		ptc_eco_set(pstPackRVal->afCellU[4] * 1000, E_PTC_ECO_CV48_48, 47);
		ptc_eco_set(pstPackRVal->afCellU[5] * 1000, E_PTC_ECO_CV48_32, 47);
		ptc_eco_set(pstPackRVal->afCellU[6] * 1000, E_PTC_ECO_CV48_16, 47);
		ptc_eco_set(pstPackRVal->afCellU[7] * 1000, E_PTC_ECO_CV48_0, 47);
		ptc_eco_set(pstPackRVal->afCellU[8] * 1000, E_PTC_ECO_CV49_48, 48);
		ptc_eco_set(pstPackRVal->afCellU[9] * 1000, E_PTC_ECO_CV49_32, 48);
		ptc_eco_set(pstPackRVal->afCellU[10] * 1000, E_PTC_ECO_CV49_16, 48);
		ptc_eco_set(pstPackRVal->afCellU[11] * 1000, E_PTC_ECO_CV49_0, 48);
		ptc_eco_set(pstPackRVal->afCellU[12] * 1000, E_PTC_ECO_CV50_48, 49);
		ptc_eco_set(pstPackRVal->afCellU[13] * 1000, E_PTC_ECO_CV50_32, 49);
		ptc_eco_set(pstPackRVal->afCellU[14] * 1000, E_PTC_ECO_CV50_16, 49);
		ptc_eco_set(pstPackRVal->afCellU[15] * 1000, E_PTC_ECO_CV50_0, 49);
		ptc_eco_set(0 * 1000, E_PTC_ECO_CV51_48, 50);
		ptc_eco_set(0 * 1000, E_PTC_ECO_CV51_32, 50);
		ptc_eco_set(0 * 1000, E_PTC_ECO_CV51_16, 50);
		ptc_eco_set(0 * 1000, E_PTC_ECO_CV51_0, 50);
		ptc_eco_set(0 * 1000, E_PTC_ECO_CV52_48, 51);
		ptc_eco_set(0 * 1000, E_PTC_ECO_CV52_32, 51);
		ptc_eco_set(0 * 1000, E_PTC_ECO_CV52_16, 51);
		ptc_eco_set(0 * 1000, E_PTC_ECO_CV52_0, 51);
		ptc_eco_set(pstPackRVal->fMosT, E_PTC_ECO_CV53_0, 52);
		ptc_eco_set(pstPackRVal->fPeakLmtDsgI, E_PTC_ECO_CV53_32, 52);
		ptc_eco_set(pstPackRVal->fLmtChgI, E_PTC_ECO_CV53_48, 52);
		ptc_eco_set(pstPackRVal->fPeakLmtDsgI, E_PTC_ECO_CV54_32, 53);
		ptc_eco_set(pstPackRVal->fLmtChgI, E_PTC_ECO_CV54_48, 53);
		ptc_eco_set(pstPackRVal->afCellU[0] * 1000, E_PTC_ECO_CV55_0, 54);
		ptc_eco_set(pstPackRVal->afCellU[1] * 1000, E_PTC_ECO_CV55_16, 54);
		ptc_eco_set(pstPackRVal->afCellU[2] * 1000, E_PTC_ECO_CV55_32, 54);
		ptc_eco_set(pstPackRVal->afCellU[3] * 1000, E_PTC_ECO_CV55_48, 54);
		ptc_eco_set(pstPackRVal->afCellU[4] * 1000, E_PTC_ECO_CV56_0, 55);
		ptc_eco_set(pstPackRVal->afCellU[5] * 1000, E_PTC_ECO_CV56_16, 55);
		ptc_eco_set(pstPackRVal->afCellU[6] * 1000, E_PTC_ECO_CV56_32, 55);
		ptc_eco_set(pstPackRVal->afCellU[7] * 1000, E_PTC_ECO_CV56_48, 55);
		ptc_eco_set(pstPackRVal->afCellU[8] * 1000, E_PTC_ECO_CV57_0, 56);
		ptc_eco_set(pstPackRVal->afCellU[9] * 1000, E_PTC_ECO_CV57_16, 56);
		ptc_eco_set(pstPackRVal->afCellU[10] * 1000, E_PTC_ECO_CV57_32, 56);
		ptc_eco_set(pstPackRVal->afCellU[11] * 1000, E_PTC_ECO_CV57_48, 56);
		ptc_eco_set(pstPackRVal->afCellU[12] * 1000, E_PTC_ECO_CV58_0, 57);
		ptc_eco_set(pstPackRVal->afCellU[13] * 1000, E_PTC_ECO_CV58_16, 57);
		ptc_eco_set(pstPackRVal->afCellU[14] * 1000, E_PTC_ECO_CV58_32, 57);
		ptc_eco_set(pstPackRVal->afCellU[15] * 1000, E_PTC_ECO_CV58_48, 57);
		ptc_eco_set(0, E_PTC_ECO_CV64_0, 63);
		ptc_eco_set(0, E_PTC_ECO_CV64_16, 63);
		ptc_eco_set(0, E_PTC_ECO_CV64_32, 63);
		ptc_eco_set(0, E_PTC_ECO_CV64_48, 63);
		ptc_eco_set(0, E_PTC_ECO_CV65_0, 64);
		ptc_eco_set(0, E_PTC_ECO_CV65_16, 64);
		ptc_eco_set(0, E_PTC_ECO_CV65_32, 64);
		ptc_eco_set(0, E_PTC_ECO_CV65_48, 64);
		ptc_eco_set(pstPackRVal->fPeakLmtDsgI, E_PTC_ECO_CV59_0, 58);
		ptc_eco_set(pstPackRVal->fLmtChgI, E_PTC_ECO_CV59_16, 58);
		ptc_eco_set(pstPackRVal->fCellUMax * 1000, E_PTC_ECO_CV60_0, 59);
		ptc_eco_set(pstPackRVal->fCellUMin * 1000, E_PTC_ECO_CV60_16, 59);
		ptc_eco_set(pstPackRVal->fCellTMin, E_PTC_ECO_CV60_32, 59);
		ptc_eco_set(pstPackRVal->fCellTMax, E_PTC_ECO_CV60_40, 59);
		ptc_eco_set(0x7017, E_PTC_ECO_CV60_48, 59);
		if(g_bMChgerComAct || g_bSChgerComAct) {
			ptc_eco_set(1, E_PTC_ECO_CV61_7, 60);
		} else {
			ptc_eco_set(0, E_PTC_ECO_CV61_7, 60);
		}
		ptc_eco_set(pstPackRVal->fPackSoc, E_PTC_ECO_CV61_8, 60);
		ptc_eco_set(pstPackRVal->fPackCur, E_PTC_ECO_CV61_16, 60);
		ptc_eco_set(pstPackRVal->fPackU, E_PTC_ECO_CV61_32, 60);
		ptc_eco_set(pstPackRVal->fPackCur,E_PTC_ECO_CV62_0,61);
		ptc_eco_set(pstPackRVal->fCellUMin * 1000,E_PTC_ECO_CV62_16,61);
		ptc_eco_set(pstPackRVal->fCellUMax * 1000,E_PTC_ECO_CV62_32,61);
		ptc_eco_set(pstPackRVal->fPackSoc,E_PTC_ECO_CV62_48,61);
		if(pstPackRVal->fPackSoc < 100) {
			ptc_eco_set(56, E_PTC_ECO_CV63_0, 62);
			ptc_eco_set(pstPackRVal->fReqChgI, E_PTC_ECO_CV63_16, 62);
		} else if(g_bSetMChgerAct && pstPackRVal->uBaseStat.stBaseStat.ucHeating != 0) {
			ptc_eco_set(56, E_PTC_ECO_CV63_0, 62);
			ptc_eco_set(5, E_PTC_ECO_CV63_16, 62);
		} else {
			ptc_eco_set(0, E_PTC_ECO_CV63_0, 62);
			ptc_eco_set(0, E_PTC_ECO_CV63_16, 62);
		}
	}
	{
		static uint8_t s_ucTick = 0;
		if(pstPackRVal->fPackSoc < 100) {
			ptc_eco_set(1, E_PTC_ECO_CV2_16, 1);
			ptc_eco_set(pstPackRVal->fReqChgI, E_PTC_ECO_CV2_32, 1);
		} else {
			ptc_eco_set(0, E_PTC_ECO_CV2_16, 1);
			ptc_eco_set(pstPackRVal->fReqChgI, E_PTC_ECO_CV2_32, 1);
		}
		if(round(pstPackRVal->fCellTMin) < g_stCfg.stLocal.sCellUCTTVThr1 || pstPackRVal->uErrCode.stErrCode.bTempSensor || GET_ALM0_CODE(9)) {
			ptc_eco_set(0, E_PTC_ECO_CV2_16, 1);
			ptc_eco_set(0, E_PTC_ECO_CV2_32, 1);
		}
		if(g_bSetMChgerAct && pstPackRVal->uBaseStat.stBaseStat.ucHeating != 0) {
			ptc_eco_set(1, E_PTC_ECO_CV2_16, 1);
			ptc_eco_set(5, E_PTC_ECO_CV2_32, 1);
			if(pstPackRVal->fCellTMin > 2) {
				ptc_eco_set(pstPackRVal->fReqChgI, E_PTC_ECO_CV2_32, 1);
			}
		}
		if(g_bSChgerComAct && !g_bMChgerComAct) {
			ptc_eco_set(0, E_PTC_ECO_CV2_16, 1);
			ptc_eco_set(0, E_PTC_ECO_CV2_32, 1);
		}
		if(g_bSChgerComAct && g_bMChgerComAct) {
			if(s_ucTick * g_stCfg.stLocal.usCyclePeriod < 5000) {
				s_ucTick++;
				ptc_eco_set(0, E_PTC_ECO_CV2_16, 1);
				ptc_eco_set(0, E_PTC_ECO_CV2_32, 1);
			} else {
				s_ucTick = 0;
			}
		}
		if(GET_ALM0_CODE(14) || GET_ALM1_CODE(14)) {
			ptc_eco_set(0, E_PTC_ECO_CV2_16, 1);
		}
		ptc_eco_set(56, E_PTC_ECO_CV2_48, 1);
		ptc_eco_set(pstPackRVal->fReqChgI, E_PTC_ECO_CV34_0, 33);
		ptc_eco_set(56, E_PTC_ECO_CV34_16, 33);
		ptc_eco_set(pstPackRVal->fPackU, E_PTC_ECO_CV34_32, 33);
		ptc_eco_set(0, E_PTC_ECO_CV34_56, 33);
		ptc_eco_set(0, E_PTC_ECO_CV34_57, 33);
		if(g_bMChgerComAct) {
			ptc_eco_set(1, E_PTC_ECO_CV34_57, 33);
		} 
		if(pstPackRVal->fReqChgI > 0 && g_bSChgerComAct) {
			ptc_eco_set(1, E_PTC_ECO_CV34_56, 33);
		}
		if(g_bMChgerComAct && !g_bSChgerComAct) {
			ptc_eco_set(0, E_PTC_ECO_CV34_0, 33);
		}
		if(g_bSChgerComAct && g_bMChgerComAct) {
			ptc_eco_set(0, E_PTC_ECO_CV34_0, 33);
			ptc_eco_set(1, E_PTC_ECO_CV34_57, 33);
		} else {
			s_ucTick = 0;
		}
		if(!g_bSChgerComAct && !g_bMChgerComAct) {
			ptc_eco_set(1, E_PTC_ECO_CV34_56, 33);
		}
		if(pstPackRVal->fPackSoc == 100 || pstPackRVal->uErrCode.stErrCode.bTempSensor || round(pstPackRVal->fCellTMin) < g_stCfg.stLocal.sCellUCTTVThr1 || GET_ALM0_CODE(9)) {
			ptc_eco_set(0, E_PTC_ECO_CV34_0, 33);
			ptc_eco_set(0, E_PTC_ECO_CV34_56, 33);
			ptc_eco_set(0, E_PTC_ECO_CV34_57, 33);
		}
	}
	//...
	
	/* send can frame according to config */
	static uint16_t s_ausTick[sizeof(g_astCanFrameCfg) / sizeof(CAN_FRAME_CFG_S)] = {0};
	if(g_stCfg.stCom.ucCanPtcType == 1) {
		for(int i = 0;i < sizeof(g_astCanFrameCfg) / sizeof(CAN_FRAME_CFG_S); i++) {
			if(i == 0  || i == 1 /*|| i == 2 */|| i == 3 || i == 4 || i == 5 || i == 6 || i == 7 || i == 8 || i == 9 || i == 10 || i == 11 || i == 12 || i == 13 || i == 14 
				 || i == 15 || i == 16 || i == 17 || i == 18 || i == 19 || i == 20 || i == 21 || i == 22 || i == 23 || i == 24 || i == 25 || i == 26 || i == 27 || i == 28 
				 || i == 29 || i == 30 || i == 31 /*|| i == 32 */|| i == 33 || i == 34 || i == 35 || i == 36 || i == 37 || i == 38 || i == 39 || i == 40 || i == 41 || i == 42) {
				s_ausTick[i] += g_stCfg.stLocal.usCyclePeriod;
				if(s_ausTick[i] >= g_astCanFrameCfg[i].uiPeriod) {
					s_ausTick[i] = 0;
					CAN0_SendMsg(g_astCanFrameCfg[i].uiCanId, g_aaucSBuf[i], g_astCanFrameCfg[i].uiDataLen);
					delay_1ms(1);
				}
			}
		}
	} else if(g_stCfg.stCom.ucCanPtcType == 3) {
		for(int i = 0;i < sizeof(g_astCanFrameCfg) / sizeof(CAN_FRAME_CFG_S); i++) {
			if(i == 43 || i == 44 || i == 45 || i == 46 || i == 47 || i == 48 || i == 49 || i == 50 || i == 51 || i == 52 || i == 53 || i == 1 || i == 11 || i == 12 || i == 12
				 || i == 13 || i == 14 || i == 15) {
				s_ausTick[i] += g_stCfg.stLocal.usCyclePeriod;
				if(s_ausTick[i] >= g_astCanFrameCfg[i].uiPeriod) {
					s_ausTick[i] = 0;
					CAN0_SendMsg(g_astCanFrameCfg[i].uiCanId, g_aaucSBuf[i], g_astCanFrameCfg[i].uiDataLen);
					delay_1ms(2);
				}
			}
		}
	} else if(g_stCfg.stCom.ucCanPtcType == 4) {
		for(int i = 0;i < sizeof(g_astCanFrameCfg) / sizeof(CAN_FRAME_CFG_S); i++) {
			if(i == 0 || i == 54 ||i == 55 || i == 56 || i == 57 || i == 58 || i == 59 || i == 60 || i == 61 || i== 62 || i== 63 || i== 64 || i == 1 || i == 5 || i == 6 || i == 7 || i == 8 || i == 12) {
				s_ausTick[i] += g_stCfg.stLocal.usCyclePeriod;
				if(s_ausTick[i] >= g_astCanFrameCfg[i].uiPeriod) {
					s_ausTick[i] = 0;
					if(i == 62) {
						if(pstPackRVal->fPackSoc < 100 || (g_bSetMChgerAct && pstPackRVal->uBaseStat.stBaseStat.ucHeating != 0)) {
							CAN0_SendMsg(g_astCanFrameCfg[i].uiCanId, g_aaucSBuf[i], g_astCanFrameCfg[i].uiDataLen);
						}
					} else {
						CAN0_SendMsg(g_astCanFrameCfg[i].uiCanId, g_aaucSBuf[i], g_astCanFrameCfg[i].uiDataLen);
					}
					delay_1ms(2);
				}
			}
		}
	}
//	for(int i = 0;i < sizeof(g_astCanFrameCfg) / sizeof(CAN_FRAME_CFG_S); i++) {
//		if((g_aullCanFramesCfg[g_stCfg.stCom.ucCanPtcType][i / 64] >> (i %64) & 0x0000000000000001) == 0) {
//			continue;
//		}
//		s_ausTick[i] += g_stCfg.stLocal.usCyclePeriod;
//		if(s_ausTick[i] >= g_astCanFrameCfg[i].uiPeriod) {
//			s_ausTick[i] = 0;
//			CAN0_SendMsg(g_astCanFrameCfg[i].uiCanId, g_aaucSBuf[i], g_astCanFrameCfg[i].uiDataLen);
//		}
//	}
}
