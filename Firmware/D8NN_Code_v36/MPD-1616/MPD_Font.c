/*
 * Title	: Font
 * Version	: 1.0
 * Created	: 2017-05-30
 * Author	: 이 재훈
 */


#include <stdio.h>
#include "MPD-1616.h"

/*

	2023-04-03
		1. PCB 변경으로 폰트 Colume 순서 변경

*/

/*------------------------------------------------------*/
//
//			TABLE
//
/*------------------------------------------------------*/

const WORD TIMER_BAR[6][16] = {
	{0x0000, 0x0000, 0x0000, 0x0000, 0xEEEE, 0xEEEE, 0xEEEE, 0xEEEE, 0xEEEE, 0xEEEE, 0xEEEE, 0xEEEE, 0x0000, 0x0000, 0x0000, 0x0000},	// 0 (100%)
	{0x0000, 0x0000, 0x0000, 0x0000, 0xEEE0, 0xEEE0, 0xEEE0, 0xEEE0, 0xEEE0, 0xEEE0, 0xEEE0, 0xEEE0, 0x0000, 0x0000, 0x0000, 0x0000},	// 1 ( 75%)
	{0x0000, 0x0000, 0x0000, 0x0000, 0xEE00, 0xEE00, 0xEE00, 0xEE00, 0xEE00, 0xEE00, 0xEE00, 0xEE00, 0x0000, 0x0000, 0x0000, 0x0000},	// 2 ( 50%)
	{0x0000, 0x0000, 0x0000, 0x0000, 0xE000, 0xE000, 0xE000, 0xE000, 0xE000, 0xE000, 0xE000, 0xE000, 0x0000, 0x0000, 0x0000, 0x0000},	// 3 ( 25%)	
	{0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000}	// 4 (  0%)
};
const WORD TIMER_BAR2[6][16] = {
	{0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},	// 4 (  0%)
	{0x0000, 0x0000, 0x0000, 0x0000, 0xE000, 0xE000, 0xE000, 0xE000, 0xE000, 0xE000, 0xE000, 0xE000, 0x0000, 0x0000, 0x0000, 0x0000},	// 3 ( 25%)
	{0x0000, 0x0000, 0x0000, 0x0000, 0xEE00, 0xEE00, 0xEE00, 0xEE00, 0xEE00, 0xEE00, 0xEE00, 0xEE00, 0x0000, 0x0000, 0x0000, 0x0000},	// 2 ( 50%)		
	{0x0000, 0x0000, 0x0000, 0x0000, 0xEEE0, 0xEEE0, 0xEEE0, 0xEEE0, 0xEEE0, 0xEEE0, 0xEEE0, 0xEEE0, 0x0000, 0x0000, 0x0000, 0x0000},	// 1 ( 75%)
	{0x0000, 0x0000, 0x0000, 0x0000, 0xEEEE, 0xEEEE, 0xEEEE, 0xEEEE, 0xEEEE, 0xEEEE, 0xEEEE, 0xEEEE, 0x0000, 0x0000, 0x0000, 0x0000}	// 0 (100%)
};

const WORD FONT_NUMBER1616[10][16] = {
	{0x0000, 0x07E0, 0x0FF0, 0x1818, 0x1C18, 0x1E18, 0x1B18, 0x1998, 0x1998, 0x18D8, 0x1878, 0x1838, 0x1818, 0x0FF0, 0x07E0, 0x0000},	// 0
	{0x0000, 0x1FF8, 0x1FF8, 0x0180, 0x0180, 0x0180, 0x0180, 0x0180, 0x0180, 0x0180, 0x0180, 0x0D80, 0x0F80, 0x0780, 0x0380, 0x0000},	// 1
	{0x0000, 0x1FF8, 0x1FF8, 0x0E00, 0x0700, 0x0380, 0x01C0, 0x00E0, 0x0070, 0x0038, 0x1818, 0x1818, 0x1C38, 0x0FF0, 0x07E0, 0x0000},	// 2
	{0x0000, 0x1FE0, 0x1FF0, 0x0018, 0x0018, 0x0018, 0x0010, 0x0FE0, 0x0FE0, 0x0010, 0x0018, 0x0018, 0x0018, 0x1FF0, 0x1FE0, 0x0000},	// 3
	{0x0000, 0x0060, 0x0060, 0x0060, 0x3FFC, 0x3FFC, 0x3060, 0x3060, 0x1860, 0x0C60, 0x0660, 0x0360, 0x01E0, 0x00E0, 0x0060, 0x0000},	// 4
	{0x0000, 0x0FE0, 0x1FF0, 0x0018, 0x0018, 0x0018, 0x0018, 0x1FF0, 0x1FE0, 0x1800, 0x1800, 0x1800, 0x1800, 0x1FF0, 0x1FF0, 0x0000},	// 5
	{0x0000, 0x07E0, 0x0FF0, 0x1C38, 0x1818, 0x1818, 0x1C38, 0x1FF0, 0x1FE0, 0x1800, 0x1800, 0x1800, 0x1C00, 0x0FF0, 0x07E0, 0x0000},	// 6
	{0x0000, 0x0180, 0x0180, 0x0180, 0x0180, 0x0180, 0x01C0, 0x00E0, 0x0070, 0x0038, 0x1818, 0x1818, 0x1818, 0x1FF8, 0x1FF8, 0x0000},	// 7
	{0x0000, 0x07E0, 0x0FF0, 0x1C38, 0x1818, 0x1818, 0x1C38, 0x0FF0, 0x0FF0, 0x1C38, 0x1818, 0x1818, 0x1C38, 0x0FF0, 0x07E0, 0x0000},	// 8
	{0x0000, 0x0FE0, 0x0FF0, 0x0018, 0x0018, 0x0018, 0x07F8, 0x0FF8, 0x1C38, 0x1818, 0x1818, 0x1818, 0x1C38, 0x0FF0, 0x07E0, 0x0000} 	// 9
};


const WORD FONT_CHARACTER[9][16] =
{
	{0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},	// 0, Blank (No Direction)
	{0x2004, 0x700E, 0xF81F, 0x7C3E, 0x3E7C, 0x1FF8, 0x0FF0, 0x07E0, 0x07E0, 0x0FF0, 0x1FF8, 0x3E7C, 0x7C3E, 0xF81F, 0x700E, 0x2004}, 	// 1, X	
	{0x01C0, 0x01C0, 0x01C0, 0x01C0, 0x01C0, 0x01C0, 0x31C6, 0x39CE, 0x3DDE, 0x1FFC, 0x0FF8, 0x07F0, 0x03E0, 0x01C0, 0x0080, 0x0000},	// 2, Arrow UP_v.2.0
	{0x0000, 0x0080, 0x01C0, 0x03E0, 0x07F0, 0x0FF8, 0x1FFC, 0x3DDE, 0x39CE, 0x31C6, 0x01C0, 0x01C0, 0x01C0, 0x01C0, 0x01C0, 0x01C0},	// 3, Arrow DOWN_v.2.0
	{0x1C00, 0x0E00, 0x0700, 0x8300, 0xC300, 0xE700, 0x7F80, 0x3FC0, 0x03FC, 0x01FE, 0x00E7, 0x00C3, 0x00C1, 0x00E0, 0x0070, 0x0038}, 	// 4, INSP	
	{0x03C0, 0x0FF0, 0x1C38, 0x300C, 0x6006, 0x6006, 0xC003, 0xDFFB, 0xDFFB, 0xC003, 0x6006, 0x6006, 0x300C, 0x1C38, 0x0FF0, 0x03C0},	// 5, DONT _v.3.0
	{0x03C0, 0x0FF0, 0x1C38, 0x300C, 0x6006, 0x6006, 0xC003, 0xC003, 0xC003, 0xC003, 0x6006, 0x6006, 0x300C, 0x1C38, 0x0FF0, 0x03C0},	// 6, DONT _v.3.1
	{0x0F80, 0x18C0, 0x1760, 0x17A0, 0x17A0, 0x17A0, 0x1B60, 0x0CC0, 0x0680, 0x1390, 0x0300, 0x2204, 0x0020, 0x0800, 0x2080, 0x0000},	// 7, FIRE
	{0x0000, 0x0000, 0x0700, 0x0780, 0x0780, 0x0780, 0x0300, 0x0000, 0x0000, 0x1010, 0x0000, 0x2004, 0x0020, 0x0800, 0x2080, 0x0000}	// 8, FIRE_Mix	
	
	/*
	{0x07C0, 0x07C0, 0x07C0, 0x87C2, 0xC7C6, 0xE7CE, 0xF7DE, 0xFFFE, 0x7FFC, 0x3FF8, 0x1FF0, 0x0FE0, 0x07C0, 0x0380, 0x0100, 0x0000},	// Arrow UP_v.1.0
	{0x0000, 0x0100, 0x0380, 0x07C0, 0x0FE0, 0x1FF0, 0x3FF8, 0x7FFC, 0xFFFE, 0xF7DE, 0xE7CE, 0xC7C6, 0x87C2, 0x07C0, 0x07C0, 0x07C0},	// Arrow DOWN_v.1.0
	{0x07E0, 0x1FF8, 0x300C, 0x6006, 0x4002, 0xC003, 0xC003, 0xFFFF, 0xFFFF, 0xC003, 0xC003, 0x4002, 0x6006, 0x300C, 0x1FF8, 0x07E0},	// DONT
	{0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x3FFC, 0x3FFC, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},	// DONT_Mix
	{0x0FF0, 0x3FFC, 0x700E, 0x6006, 0xC003, 0xC003, 0xC003, 0xDFFB, 0xDFFB, 0xC003, 0xC003, 0xC003, 0x6006, 0x700E, 0x3FFC, 0x0FF0},	// DONT _v.2.0
	{0x0FF0, 0x3FFC, 0x700E, 0x6006, 0xC003, 0xC003, 0xC003, 0xC003, 0xC003, 0xC003, 0xC003, 0xC003, 0x6006, 0x700E, 0x3FFC, 0x0FF0},	// DONT _v.2.1
	{0xFFFF, 0xFFFF, 0xC003, 0xC003, 0xC003, 0xC003, 0xC003, 0xC003, 0xC003, 0xC003, 0xC003, 0xC003, 0xC003, 0xC003, 0xFFFF, 0xFFFF},	// STOP
	*/
};

const WORD FONT_NUMBER0816[15][16] =
{
	{0x00, 0x00, 0x00, 0x3C, 0x7E, 0x66, 0x66, 0x6E, 0x76, 0x66, 0x66, 0x7E, 0x3C, 0x00, 0x00, 0x00}, // 0
	{0x00, 0x00, 0x00, 0x7E, 0x7E, 0x18, 0x18, 0x18, 0x18, 0x18, 0x1C, 0x1C, 0x18, 0x00, 0x00, 0x00}, // 1
	{0x00, 0x00, 0x00, 0x7E, 0x7E, 0x0C, 0x18, 0x30, 0x60, 0x60, 0x66, 0x7E, 0x3C, 0x00, 0x00, 0x00}, // 2
	{0x00, 0x00, 0x00, 0x3C, 0x7E, 0x66, 0x60, 0x3C, 0x3C, 0x60, 0x66, 0x7E, 0x3C, 0x00, 0x00, 0x00}, // 3
	{0x00, 0x00, 0x00, 0x30, 0x30, 0x30, 0x7E, 0x7E, 0x32, 0x34, 0x38, 0x30, 0x20, 0x00, 0x00, 0x00}, // 4
	{0x00, 0x00, 0x00, 0x3C, 0x7E, 0x66, 0x60, 0x7E, 0x7E, 0x06, 0x06, 0x7E, 0x7E, 0x00, 0x00, 0x00}, // 5
	{0x00, 0x00, 0x00, 0x3C, 0x7E, 0x46, 0x46, 0x7E, 0x3E, 0x06, 0x66, 0x7E, 0x3C, 0x00, 0x00, 0x00}, // 6
	{0x00, 0x00, 0x00, 0x18, 0x18, 0x18, 0x18, 0x30, 0x60, 0x66, 0x66, 0x7E, 0x7E, 0x00, 0x00, 0x00}, // 7
	{0x00, 0x00, 0x00, 0x3C, 0x7E, 0x66, 0x66, 0x3C, 0x3C, 0x66, 0x66, 0x7E, 0x3C, 0x00, 0x00, 0x00}, // 8
	{0x00, 0x00, 0x00, 0x3C, 0x7E, 0x66, 0x60, 0x7C, 0x7E, 0x66, 0x66, 0x7E, 0x3C, 0x00, 0x00, 0x00}, // 9
	{0x00, 0x00, 0x00, 0xFD, 0xFC, 0x30, 0x30, 0x30, 0x30, 0x30, 0x70, 0x70, 0x30, 0x00, 0x00, 0x00}, // 1.
	{0x00, 0x00, 0x00, 0x18, 0x3C, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x00, 0x00, 0x00}, // V
	{0x00, 0x00, 0x00, 0x7E, 0x7E, 0x06, 0x06, 0x7E, 0x7E, 0x06, 0x06, 0x7E, 0x7E, 0x00, 0x00, 0x00}, // E
	{0x00, 0x00, 0x00, 0x66, 0x66, 0x36, 0x1E, 0x1E, 0x3E, 0x66, 0x66, 0x7E, 0x3E, 0x00, 0x00, 0x00}, // R
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}	
};

const WORD FONT_ERROR[16] =
{
	0x0000,	0x0000,	0x0000,	0x007E,	0x007E,	0x0006,	0x0006,	0x3C7E,	0x3C7E,	0x0006,	0x0006,	0x007E,	0x007E,	0x0000,	0x0000,	0x0000
};

const WORD FONT_CHECK[16] =
{
	0xFFFF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000
};

