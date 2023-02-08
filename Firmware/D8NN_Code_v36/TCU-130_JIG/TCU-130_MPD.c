/*
	TITLE  	: Test CAN UART - NUC 130
	AUTHOR	: 이 재 훈
	MPU		: NUC130
	CLOCK	: 14.7456MHz			
	CREATED	: 2017. 06. 08
	VERSION	: 1.0
	
	Test	: MPD-1616 ( Multi Post Dotmatrix - 16 * 16 )
	CAM		: 115.2KBps
*/



/*
	[	CAN	  ] - 115.2 KBPS
				- tx : E/S Status
				
	[  DIPSW  ] - SW1 : TX E/S Status SETTING
				- SW2 : SET AUTO
				- SW3 : SET UP
				- SW4 : SET DOWN
*/



#include <stdio.h>
#include "NUC1xx.h"
#include "DrvGPIO.h"
#include "DrvUART.h"
#include "DrvCAN.h"
#include "DrvSYS.h"
#include "DrvTIMER.h"

#include "TCU-130_MPD.h"
                                            
extern char GetChar(void);

/*------------------------------------------------------*/
/*																																																*/
/*			FUNCTION PROTOTYPE DEFINE																					*/
/*																																																*/
/*------------------------------------------------------*/
void Delay_ms(int time);

// statistics of all the interrupts
void CAN_ShowMsg(STR_CANMSG_T* Msg);
void Input_Msg(STR_CANMSG_T* Msg, volatile uint8_t array[]);

void Tx_ES_Status(void);

void Recive_CAN(void);

void OUTPUT(BYTE btemp);
void LED_Check(void);

/*------------------------------------------------------*/
/*																																																*/
/*			VARIABLE																																						*/
/*																																																*/
/*------------------------------------------------------*/

STR_CANMSG_T rrMsg;		// CAN Recive
STR_CANMSG_T msg1;   	// for CAN 구조체 선언

STR_UART_T param;

MSG_STATUS_ESCALATOR 	MsgSTES;
MSG_REQ_ESCALATOR		MsgReqES;

BYTE bDipsw ;
int i,j;

// CAN용..
BYTE Rx_Req[4] = {0, 0, 0, 0};

volatile BYTE bCan_delay    = 0, bTimer0Cnt= 0;

BYTE fCan_TxStart = FALSE;
BYTE fTimer_500ms = FALSE;

/*------------------------------------------------------//
//
//			TIMER
//
//------------------------------------------------------*/

void Delay_ms(int time)
{	
	int i = 0;
	for(i = 0; i < time; i ++)
	{		
		DrvSYS_Delay(1000);
	}
}

/*------------------------------------------------------
//				TIMER _ initialization
//-----------------------------------------------------*/
void TIMER_Init()
{	
	DrvSYS_SelectIPClockSource(E_SYS_TMR0_CLKSRC, 7);		// Timer 0,1 clock source - internal 22.1184 MHz
	DrvSYS_SelectIPClockSource(E_SYS_TMR1_CLKSRC, 7);
	DrvTIMER_Init();
	DrvTIMER_Open(E_TMR0, 400, E_PERIODIC_MODE);			// Using Timer0 in PERIODIC_MODE, 1/400 = 2.5 ms

	DrvTIMER_Open(E_TMR1, 20, E_PERIODIC_MODE);				// Using Timer0 in PERIODIC_MODE, 1/20 = 50 ms			
	
	// Enable TIMER0 interrupt
	DrvTIMER_EnableInt(E_TMR0);
	DrvTIMER_EnableInt(E_TMR1);			
	
	DrvTIMER_Start(E_TMR0);
	DrvTIMER_Start(E_TMR1);			
}

/*------------------------------------------------------
//				TIMER0 _ 2.5ms
//-----------------------------------------------------*/
void TMR0_Callback(void) 
{
	bTimer0Cnt++;
	
	if(bTimer0Cnt >= 40)		// 2.5 ms * 40 = 100 ms
	{
		fTimer_500ms = TRUE;
		bTimer0Cnt = 0;
	}	
	
	if(fCan_TxStart)		
		bCan_delay++;
}

/*------------------------------------------------------
//				TIMER1 _ 500ms
//-----------------------------------------------------*/
void TMR1_Callback(void)
{
	STS_LED_Toggle();
}


/*-------------------------------------------------------
//
//			CAN
//
//------------------------------------------------------*/

void CAN_CallbackFn(uint32_t u32IIDR)
{
	CAN_Rx_LED_ON();	
	switch(u32IIDR) {		
		case (2 + 1) :				// Req Communication - MOb 2			
			printf("Msg-2 INT and Callback - Req Communication\n");

			DrvCAN_ReadMsgObj((u32IIDR - 1),TRUE,&rrMsg);
			
			MsgReqES.bDeviceAddr = rrMsg.Data[0];
			MsgReqES.bPosition	 = rrMsg.Data[1];			
			// TOP = 1, BOTTOM = 0
			CAN_ShowMsg(&rrMsg);			
			
			break;		
		default : break;		
	}
	CAN_Rx_LED_OFF();
}

/*------------------------------------------------------
//								CAN _ RX
//-----------------------------------------------------*/
void Recive_CAN(void){
	//ReceiveCan_ReqCommunicationTest 
    if(DrvCAN_SetRxMsgObj(MSG(2),CAN_STD_ID, ReqCommTest_ID,TRUE) < 0)
	{
        printf("Set Rx Msg Object failed\n");
        return;
    }
	
	DrvCAN_EnableInt(CAN_CON_IE);
	DrvCAN_InstallCallback(CALLBACK_MSG, (CAN_CALLBACK)CAN_CallbackFn);
}

void Input_Msg(STR_CANMSG_T* Msg, volatile uint8_t array[])		// Input Rx Msg into the Array
{
	for(i = 0; i < Msg->DLC; i++)    
		array[i] = Msg->Data[i];
}

void CAN_ShowMsg(STR_CANMSG_T* Msg)
{
	uint8_t i;
	printf("Read ID=%4X, Type=%s, DLC=%d, Data=", Msg->Id, Msg->IdType?"EXT":"STD", Msg->DLC);
	for(i=0;i<Msg->DLC;i++)    
			printf("0x%02X,",Msg->Data[i]);
	printf("\n\n");
}

/*------------------------------------------------------
//				CAN _ TX
//-----------------------------------------------------*/
void Tx_ES_Status(void)
{
	STR_CANMSG_T t_Msg;
	
	CAN_Tx_LED_ON();
	DrvCAN_DisableInt(CAN_CON_IE | CAN_CON_SIE);
		
/* Message 1 */
	t_Msg.FrameType = DATA_FRAME;
	t_Msg.IdType 	= CAN_STD_ID;
	t_Msg.Id 		= STATUSMSG_ID;
	t_Msg.DLC 		= 8;

	t_Msg.Data[0] =	MsgSTES.bMode;				// mode
	t_Msg.Data[1] =	MsgSTES.Status[0];			// status[0]
	t_Msg.Data[2] =	MsgSTES.Status[1];			// status[1]
	t_Msg.Data[3] =	MsgSTES.Status[2];			// status[2]
	t_Msg.Data[4] =	MsgSTES.bSavedErrorCount;	// SaveErrorCount
	t_Msg.Data[5] =	MsgSTES.bCurrentErrorCode;	// CurrentErrorCode
	t_Msg.Data[6] =	MsgSTES.bPessraeErrorCode;	// SubErrorCode
	t_Msg.Data[7] = 0x88;
	
	if(DrvCAN_SetTxMsgObj(MSG(1),&t_Msg) < 0)     // Set MsgObj and configure
    {
        printf("Set Tx Msg Object failed\n");
        return;
    }
	
	bCan_delay = 0;
	fCan_TxStart = TRUE;
	
	while(!(DrvCAN_SetTxRqst(MSG(1))))		// Transmit request bit
	{
		wdt_reset();						// Watchdog Timer Reset
		if(bCan_delay >= 16)		// 2.5 ms * 16 = 40 ms
			break;
	}
	
	fCan_TxStart = FALSE;
	
	CAN_ShowMsg(&t_Msg);
	CAN_Tx_LED_OFF();
	DrvCAN_EnableInt(CAN_CON_IE | CAN_CON_SIE);		
}


/*------------------------------------------------------//
//
//			PORT
//
//------------------------------------------------------*/

void Port_Init()
{
	DrvGPIO_Open(E_GPA,  4, E_IO_INPUT);
	DrvGPIO_Open(E_GPA,  5, E_IO_INPUT);
	DrvGPIO_Open(E_GPA,  6, E_IO_INPUT);
	DrvGPIO_Open(E_GPA,  7, E_IO_INPUT);
	DrvGPIO_Open(E_GPA,  8, E_IO_INPUT);
	DrvGPIO_Open(E_GPA,  9, E_IO_INPUT);
	DrvGPIO_Open(E_GPA, 10, E_IO_INPUT);
	DrvGPIO_Open(E_GPA, 11, E_IO_INPUT);
	
	//	DIPSW
	DrvGPIO_Open(E_GPA, 12, E_IO_INPUT);
	DrvGPIO_Open(E_GPA, 13, E_IO_INPUT);
	DrvGPIO_Open(E_GPA, 14, E_IO_INPUT);
	DrvGPIO_Open(E_GPA, 15, E_IO_INPUT);
	
	//GPIOA->DOUT = 0x0000;
			
	DrvGPIO_Open(E_GPB,  0, E_IO_INPUT);
	DrvGPIO_Open(E_GPB,  1, E_IO_INPUT);
	DrvGPIO_Open(E_GPB,  2, E_IO_INPUT);
	DrvGPIO_Open(E_GPB,  3, E_IO_INPUT);
	DrvGPIO_Open(E_GPB,  4, E_IO_INPUT);
	DrvGPIO_Open(E_GPB,  5, E_IO_INPUT);
	DrvGPIO_Open(E_GPB,  6, E_IO_INPUT);
	DrvGPIO_Open(E_GPB,  7, E_IO_INPUT);
	DrvGPIO_Open(E_GPB,  8, E_IO_INPUT);
	DrvGPIO_Open(E_GPB,  9, E_IO_INPUT);
	DrvGPIO_Open(E_GPB, 10, E_IO_INPUT);
	DrvGPIO_Open(E_GPB, 11, E_IO_INPUT);
	DrvGPIO_Open(E_GPB, 12, E_IO_INPUT);
	DrvGPIO_Open(E_GPB, 13, E_IO_INPUT);
	DrvGPIO_Open(E_GPB, 14, E_IO_INPUT);
	DrvGPIO_Open(E_GPB, 15, E_IO_INPUT);
	

	//	LED
	DrvGPIO_Open(E_GPC,  0, E_IO_OUTPUT);	// CAN_Tx
	DrvGPIO_Open(E_GPC,  1, E_IO_OUTPUT);	// CAN_Rx
	DrvGPIO_Open(E_GPC,  2, E_IO_OUTPUT);	// STS
	DrvGPIO_Open(E_GPC,  3, E_IO_OUTPUT);	// ERROR
	
	//	OUTPUT
	DrvGPIO_Open(E_GPC,  6, E_IO_OUTPUT);
	DrvGPIO_Open(E_GPC,  7, E_IO_OUTPUT);
	DrvGPIO_Open(E_GPC,  8, E_IO_OUTPUT);
	DrvGPIO_Open(E_GPC,  9, E_IO_OUTPUT);
	DrvGPIO_Open(E_GPC, 10, E_IO_OUTPUT);
	DrvGPIO_Open(E_GPC, 11, E_IO_OUTPUT);
	DrvGPIO_Open(E_GPC, 14, E_IO_OUTPUT);
	DrvGPIO_Open(E_GPC, 15, E_IO_OUTPUT);
	
	DrvGPIO_SetPortBits(E_GPC, 0xF);
}

void OUTPUT(BYTE btemp)
{	
	GPC_15 = (btemp & 0x80)? 1 : 0;
	GPC_14 = (btemp & 0x40)? 1 : 0;
	GPC_11 = (btemp & 0x20)? 1 : 0;
	GPC_10 = (btemp & 0x10)? 1 : 0;
	GPC_9  = (btemp & 0x08)? 1 : 0;
	GPC_8  = (btemp & 0x04)? 1 : 0;
	GPC_7  = (btemp & 0x02)? 1 : 0;
	GPC_6  = btemp & 0x01;
}

void LED_Check()
{
	int i = 0;
	
	for(i = 0; i < 5; i++)
	{
		GPIOC->DOUT = (~(1 << i)) & 0xF;
		Delay_ms(100);
	}
	
	for(i = 0; i < 9; i++)
	{
		OUTPUT(0x01 << i);
		Delay_ms(100);
	}
	
	GPIOC->DOUT = 0xF;
}

/*------------------------------------------------------*/
//
//			MAIN PROGRAM
//
/*------------------------------------------------------*/

int main (void){
	int bTemp;
/*------------------------------------------------------
//					Clock _ Set
//-----------------------------------------------------*/
	UNLOCKREG();

	DrvSYS_SetOscCtrl(E_SYS_OSC22M, 1);
	DrvSYS_Delay(20000);

	while(!SYSCLK->CLKSTATUS.OSC22M_STB);
	
	DrvSYS_SelectPLLSource(E_SYS_INTERNAL_22M);
	DrvSYS_Open(25000000);
	DrvSYS_Delay(20000);
	SYSCLK->PLLCON.OE = 0;		// Fout Enable(0)
	
	// 0 : Ex 4 ~ 24 MHz,	 1 : Ex 32 K,		2 : PLL,		3 : In 10 K,		7 : In 22 MHz
	DrvSYS_SelectHCLKSource(7);

	DrvSYS_SelectIPClockSource(E_SYS_UART_CLKSRC,0x3);			// 	UART  : Internal 22.1184MHz
	DrvSYS_SelectIPClockSource(E_SYS_WDT_CLKSRC, 0x2);			// 	WDT	  : HCLK / 2048 Clock
	DrvSYS_SelectIPClockSource(E_SYS_TMR0_CLKSRC,0x7);			// TIMER0 : Internal 22.1184 MHz
	DrvSYS_SelectIPClockSource(E_SYS_TMR1_CLKSRC,0x7);			// TIMER1 : Internal 22.1184 MHz
	
	DrvWDT_Open(E_WDT_LEVEL5);
	DrvWDT_Ioctl(E_WDT_IOC_ENABLE_INT, 0);
	DrvWDT_Ioctl(E_WDT_IOC_START_TIMER, 0);				// Watchdog Start

	LOCKREG();
/*------------------------------------------------------*/
	Port_Init();

	TIMER_Init();
	// Install callback "TMRn_Callback" and trigger callback when Interrupt happen always
	(void)DrvTIMER_SetTimerEvent(E_TMR0, 1, (TIMER_CALLBACK)TMR0_Callback, 0);	// 2.5ms
	(void)DrvTIMER_SetTimerEvent(E_TMR1,10, (TIMER_CALLBACK)TMR1_Callback, 0);	// 10 * 50 = 500ms

	// Init GPIO and configure UART0	
	DrvGPIO_InitFunction(E_FUNC_UART2);
	param.u32BaudRate 		= 115200;
	param.u8cDataBits 		= DRVUART_DATABITS_8;
	param.u8cStopBits 		= DRVUART_STOPBITS_1;
	param.u8cParity 		= DRVUART_PARITY_NONE;
	param.u8cRxTriggerLevel = DRVUART_FIFO_1BYTES;
	param.u8TimeOut 		= 0;
	DrvUART_Open(UART_PORT2, &param);			

	// Select CAN Multi-Function
	DrvGPIO_InitFunction(E_FUNC_CAN0);	
	DrvCAN_Init();

	// CAN Send Tx Msg by basic mode Function (without Message RAM)	
	/*Tseg2, Tseg1, Sjw, Brp*/			
	//DrvCAN_SetTiming(3,3,1,29);		
	// (3,3,0,2) - 819.2 KBPS/ (3,3,1,29) - 81.92 KBPS/ (4, 5, 1, 15) - 115.2 KBPS
	DrvCAN_SetTiming(4, 5, 1, 15);
	
	printf("/************** CAN SETTING **************/\n");
	printf("Tseg1 : %d \tTseg2 : %d \tSJW : %d \tBpr : %d \n", CAN->BTIME.TSEG1,CAN->BTIME.TSEG2 ,CAN->BTIME.SJW ,(CAN->BTIME.BRP) | (CAN->BRPE.BPRE <<6));
	printf("CAN HCLKFreq : %d  [Hz]\n",DrvSYS_GetHCLKFreq());
	printf("PLL Clock    : %d  [Hz]\n",DrvSYS_GetPLLClockFreq());
	printf("CAN BITRATE  : %8d  [kbps]\n",DrvCAN_GetCANBitRate());
	printf("CAN BRP      : %d\n", (CAN->BTIME.BRP) | (CAN->BRPE.BPRE <<6));
	printf("/*****************************************/\n");

	// Module interrupt, Status Change interrupt enable
	DrvCAN_EnableInt(CAN_CON_IE | CAN_CON_SIE);			

	// Dipsw 읽기-----------------------------------------------------
	bDipsw = ((DrvGPIO_GetPortBits(E_GPB) >> 8) & 0xF);	
	
	DrvGPIO_InitFunction(E_FUNC_CLKO);
	
	LED_Check();
	
	Recive_CAN();
	
	MsgSTES.bMode			  = 0x01;
	MsgSTES.Status[0]		  = 0x0;
	MsgSTES.Status[2]		  = 0x0;
	MsgSTES.bSavedErrorCount  = 0x0;
	MsgSTES.bCurrentErrorCode = 0x0;
	MsgSTES.bPessraeErrorCode = 0x0;
				
//------------------------------------------------------------------	
	while(1)
	{
		wdt_reset();
		
		if(GPB_0 == 0)
			MsgSTES.Status[2] = 0x01;
		else
			MsgSTES.Status[2] = 0;

		if(GPA_15 == 1)
		{
			printf("****** Please select a status. ******\n(2 : AUTO, 3 : INSP, 4 : ERROR, 5 : FIRE, 6 : Up_BUZZ, 7 : Down_BUZZ) \n : ");
			scanf("%d", &bTemp);
			printf("%d \n",bTemp);
			
			if(2 == bTemp)
			{
				MsgSTES.Status[1] = 1 << bTemp;
				printf("****** Choose UP and DOWN. ******   0 : UP, 1 : DOWN \n : ");
				scanf("%d", &bTemp);
				printf("%d \n",bTemp);
				MsgSTES.Status[1] |= 1 << bTemp;
			}
			else
				MsgSTES.Status[1] = 1 << bTemp;
		}
		else
		{
			GPA_12 ? (MsgSTES.Status[1] |= 0x01) : (MsgSTES.Status[1] &= ~0x01);
			GPA_13 ? (MsgSTES.Status[1] |= 0x02) : (MsgSTES.Status[1] &= ~0x02);
			GPA_14 ? (MsgSTES.Status[1] |= 0x04) : (MsgSTES.Status[1] &= ~0x04);
		}
		
		if(fTimer_500ms)		// 100 ms
		{
			OUTPUT(MsgSTES.Status[1]);
			printf("\n\nTx_ES_Status - \n");
			Tx_ES_Status();
			
			printf("\n\n\n");

			fTimer_500ms = FALSE;
		}
	}
}

