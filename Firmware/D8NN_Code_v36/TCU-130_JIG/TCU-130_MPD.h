/*
	TITLE  	: Test CAN UART - NUC 130
	AUTHOR	: 이 재 훈
	MPU		: NUC130
	CLOCK	: 14.7456MHz			
	CREATED	: 2017. 06. 08
	VERSION	: 1.0
	
	Test	: MPD-1616 ( Multi Post Dotmatrix - 16 * 16 )
*/

#ifndef __TCU_130_h__
#define __TCU_130_h__
/*------------------------------------------------------*/
/*														*/
/*			TYPE DEFINE									*/
/*														*/
/*------------------------------------------------------*/

typedef 	unsigned char 	BYTE;
typedef 	BYTE				BOOL;
typedef 	unsigned int 		WORD;
typedef	unsigned long 	DWORD;

/*------------------------------------------------------*/
/*														*/
/*			DEFINE										*/
/*														*/
/*------------------------------------------------------*/
#define FALSE				0
#define TRUE				1

// CAN ID
#define ReqCommTest_ID		0x0100
#define STATUSMSG_ID		0x0500

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

typedef struct _MSG_REQ_ESCALATOR
{
	BYTE bDeviceAddr;
	BYTE bPosition;	
} MSG_REQ_ESCALATOR;

/*------------------------------------------------------*/
/*																																																*/
/*			MACRO																																							*/
/*																																																*/
/*------------------------------------------------------*/

#define sleep()				asm volatile("sleep"::)
#define nop()				asm volatile("nop"::)
	
#define wdt_reset()	WDT->WTCR.WTR = 1
//#define WDT_RESET_COUNTER	(		)	   (WDT->WTCR = (WDT->WTCR & ~(WDT_WTCR_WTIF_Msk | WDT_WTCR_WTWKF_Msk | WDT_WTCR_WTRF_Msk)) | WDT_WTCR_WTR_Msk)

#define CAN_Tx_LED_ON()			GPC_0 = 0
#define CAN_Tx_LED_OFF()		GPC_0 = 1
#define CAN_Tx_LED_Toggle()		GPC_0 ^= 1

#define CAN_Rx_LED_ON()			GPC_1 = 0
#define CAN_Rx_LED_OFF()		GPC_1 = 1
#define CAN_Rx_LED_Toggle()		GPC_1 ^= 1

#define STS_LED_ON()			GPC_2 = 0
#define STS_LED_OFF()			GPC_2 = 1
#define STS_LED_Toggle()		GPC_2 ^= 1

#define ERR_LED_ON()			GPC_3 = 0
#define ERR_LED_OFF()			GPC_3 = 1
#define ERR_LED_Toggle()		GPC_3 ^= 1

#endif


