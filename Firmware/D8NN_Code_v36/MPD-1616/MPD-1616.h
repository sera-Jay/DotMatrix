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

#define ERROR			0
#define ERROR_CODE		20
#define UP				1
#define DOWN			2			// 어느 방향에서든 진입 하는 방향으로 화살표 표시
#define	INSP			3
#define DONT			4
#define DONT_sub 		5
#define FIRE			6
// FIRE_sub				7
#define STOP			8
#define AUTO			10


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


// CAN data Flag
typedef struct _CAN_FLAG
{
	volatile WORD fUp		:1;
	volatile WORD fDown		:1;
	volatile WORD fAuto		:1;
	volatile WORD fInsp		:1;
	volatile WORD fError	:1;
	volatile WORD fFire		:1;
	volatile WORD fU_Buzz	:1;
	volatile WORD fD_Buzz	:1;
	
	volatile WORD fMove		:1;
	
	WORD                    :7;
} CAN_FLAG;

// Status Flag
typedef struct _STS_FLAG
{
	volatile BYTE fPosition  :1;
	volatile BYTE fUp 		 :1;
	volatile BYTE fDown 	 :1;
	volatile BYTE fAUTO	 	 :1;
	volatile BYTE fINSP 	 :1;
	volatile BYTE fError 	 :1;
	volatile BYTE fFire 	 :1;
	volatile BYTE fBuzzer 	 :1;	
} STS_FLAG;


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


