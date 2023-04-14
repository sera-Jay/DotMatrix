/*
	 TITLE  	: MPD-1616_header
	 AUTHOR		: 이 재 훈
	 MPU		: NUC130SC2AE	 
	 CREATED	: 2017. 05. 30
	 VERSION	: 1.0
 */ 


#ifndef __MPD_1616_H__
#define __MPD_1616_H__

/*------------------------------------------------------*/
//
//			TYPE DEFINE
//
/*------------------------------------------------------*/

typedef unsigned char 	BYTE;
typedef BYTE			BOOL;
typedef unsigned int 	WORD;
typedef	unsigned long 	DWORD;

/*------------------------------------------------------*/
//
//			DEFINE
//
/*------------------------------------------------------*/

#define FALSE			0
#define TRUE			1

#define TOP				1
#define BOTTOM			0

#define GRN				1
#define RED				2
#define YELLOW			3

#define TEXT_BK			14
#define TEXT_V			11
#define TEXT_E			12
#define TEXT_R			13

#define CAN_BIT_UP		0x01
#define CAN_BIT_DOWN	0x02
#define CAN_BIT_AUTO	0x04
#define CAN_BIT_INSP	0x08
#define CAN_BIT_ERROR	0x10
#define CAN_BIT_FIRE	0x20
#define CAN_BIT_UBZ		0x40
#define CAN_BIT_DBZ		0x80


#define SINPUT_NOTHING	0x0000
#define SINPUT_BIT_DIR	0x0001
#define SINPUT_BIT_NE	0x0002
#define SINPUT_BIT_AUT	0x0004
#define SINPUT_BIT_INS	0x0008
#define SINPUT_BIT_ERR	0x0010
#define SINPUT_BIT_FIRE 0x0020
#define SINPUT_BIT_BUZ	0x0040
#define SINPUT_BIT_STP	0x0080

#define FONT_BLK		0
#define FONT_X			1
#define FONT_UP			2
#define FONT_DN			3
#define FONT_INSP		4
#define FONT_DONT		5
#define FONT_DONTsub	6
#define FONT_FIRE		7
#define FONT_FIREsub	8


#define DIPSW1			GPB_0
#define DIPSW2			GPB_1
#define DIPSW3			GPB_2
#define DIPSW4			GPB_3

#define Latch_G			GPB_9
#define Latch_R			GPB_10	

#define IN_DIR			GPC_8
#define IN_NE			GPC_9
#define IN_STP			GPC_10

#define IN_AUT			GPB_4
#define IN_INS			GPB_5
#define IN_ERR			GPB_6
#define IN_FIRE			GPB_7

#define IN_BUZZER		GPF_5
#define OUT_BUZZER		GPF_4
/*
#define IN_UP			GPC_8
#define IN_DOWN			GPC_9

#define IN_AUTO			GPB_4
#define IN_INSP			GPB_5
#define IN_ERROR		GPB_6
#define IN_FIRE			GPB_7

#define IN_BUZZER		GPF_5
#define OUT_BUZZER		GPF_4
*/

// CAN ID

#define ReqCommTest_ID			0x0100
#define STATUSMSG_ID			0x0500
/*------------------------------------------------------*/
//
//			ENUM
//
/*------------------------------------------------------*/
typedef enum _DISPLAY_MODE
{
	/* 0 */Display_NOTHING = 0,
	/* 1 */Display_UP,
	/* 2 */Dispaly_DOWN,
	/* 3 */Display_DONT,
	/* 4 */Display_INSP,
	/* 5 */Display_FIRE,
	/* 6 */Display_ERROR,
	/* 7 */Display_ERROR_CODE
} DISPLAY_MODE;

/*------------------------------------------------------*/
//
//			STRUCT
//
/*------------------------------------------------------*/

// Status Escalator MSG : 운행정보 0x0500
typedef struct _MSG_STATUS_ESCALATOR
{
	BYTE bMode;			// 0x01
	BYTE Status[3];
	BYTE bSavedErrorCount;
	BYTE bCurrentErrorCode;
	BYTE bPessraeErrorCode;
} MSG_STATUS_ESCALATOR;

typedef struct _DISPLAY_INFO
{
	DISPLAY_MODE 	eMode;
	BYTE			bColor;
} DISPALY_INFO;


typedef struct _CAN_DATA
{
	BYTE fDIR				: 1;
	BYTE fNE				: 1;
	BYTE fAuto				: 1;
	BYTE fInsp				: 1;
	BYTE fError				: 1;
	BYTE fFire				: 1;
	BYTE fUBZ				: 1;
	BYTE fDBZ				: 1;
} CAN_DATA;


typedef union _GLOBAL_FLAG
{
	struct _GLOBAL_BIT
	{
		WORD fPosition		: 1;	// 1 : TOP,  0 : BOTTON
		WORD fUpDown		: 1;	// 1 : DOWN, 0 : UP			(DIPSW_1)
		WORD fScrollFuntion : 1;	// 1 : 
		WORD fDisplayMix	: 1;
		WORD fYellowBlink	: 1;
		WORD fFadeMode		: 1;
		WORD fFadeInOut		: 1;	// 1 : IN,   0 : OUT
		
		WORD fStart			: 1;
	} bit;
	
	WORD wReg;
} GLOBAL_FLAG;

typedef union _INPUT_FLAG
{
	struct _INPUT_BIT
	{
		WORD fDIR			: 1;
		WORD fNE			: 1;
		WORD fAuto			: 1;
		WORD fInsp			: 1;
		WORD fError			: 1;
		WORD fFire			: 1;
		WORD fBuzzer		: 1;
		WORD fStp			: 1;
		
		WORD 				: 8;
	} bit;
	
	WORD wReg;
} INPUT_FLAG;


/*------------------------------------------------------*/
//
//			MACRO
//
/*------------------------------------------------------*/

#define wdt_reset() 				WDT->WTCR.WTR = 1	// SYS->u32RSTSRC |= 0x2

#define CAN_LED_OFF()				GPC_14 = 1
#define CAN_LED_ON()				GPC_14 = 0
#define CAN_LED_Toggle()			GPC_14 ^= 1

#define STATUS_LED_OFF()			GPC_15 = 1
#define STATUS_LED_ON()				GPC_15 = 0
#define STATUS_LED_Toggle()			GPC_15 ^= 1

#define Latch_OFF() 	{Latch_G = 0; Latch_R = 0; GPB_11 = 1; GPE_5 = 1;}

#define GRN_ON() 		{GPB_11 = 0; GPE_5 = 1; Latch_G = 1; Latch_R = 0;}
#define RED_ON() 		{GPB_11 = 1; GPE_5 = 0; Latch_G = 0; Latch_R = 1;}
#define YEL_ON() 		{GPB_11 = 0; GPE_5 = 0; Latch_G = 1; Latch_R = 1;}
#define OFF()			{GPB_11 = 1; GPE_5 = 1; Latch_G = 0; Latch_R = 0;}

/*------------------------------------------------------*/
//
//			PWM
//
/*------------------------------------------------------*/

#define PWM_CH_4_MASK              (0x10UL)   /*!< PWM channel 4 mask \hideinitializer */
#define PWM_CH_5_MASK              (0x20UL)   /*!< PWM channel 5 mask \hideinitializer */

#define PWM_SET_PRESCALER(pwm, u32ChannelNum, u32Prescaler) (*(__IO uint32_t *) (&((pwm)->CLKPSC0_1) + ((u32ChannelNum) >> 1)) = (u32Prescaler))
#define PWM_SET_CMR(pwm, u32ChannelNum, u32CMR) ((pwm)->CMPDAT[(u32ChannelNum)] = (u32CMR))
#define PWM_SET_CNR(pwm, u32ChannelNum, u32CNR)  ((pwm)->PERIOD[(((u32ChannelNum) >> 1) << 1)] = (u32CNR))

#endif


