/*
	TITLE  	: MPD-1616
	AUTHOR	: 이 재훈
	MPU		: NUC130SC2AE
	CLOCK	: 22.1184 MHz
	CREATED	: 2017. 05. 30
	VERSION	: 1.0
*/

/*
	[   CAN	   ] - 115.2 KBps
			RX ) - STATUS_ESCALATOR_MSG
	
	[ PARALLEL ] - UP / DOWN / ERROR / TEST / FIRE / AUTO / BUZZER
				 - SIGNAL : ( - ), N24
				 - COM	  : ( + ), P24
				 
	[   DIPSW  ] - 0b_1000 : S_Flag.fPosition ( 1 : TOP, 0 : BOTTOM)
				 - 0b_X111 : ID ( TOP : RED, BOTTOM : GRN)
				 - 0b_0100 : FADE ON/OFF (After ID Setting and starting main loop, Control the DIPSW2)
				 
	[ DISPLAY  ] - SDM5169_URUG ( 16 * 16, 2 + 1 Color, DotMatrix)
		   ROW ) - P-Channel MOSFET(Current Control) Array , PIN	// Anode
		   COL ) - 74HC595 + ULN2803 , SPI	// Cathode
		   
				(1 sec / 60 Frame) -> (0.016666... sec / 1 Frame) 
				 => (16 Line / 16 msec) -> (1 Line / 1 msec)
				 
	[   LED	   ] - ( ERROR ) : 0.2s toggle
 				 - (  FIRE ) : 0.5s toggle
				 - (  TEST ) : Always ON
				 - (  AUTO ) : 0.4s ON / 0.6s OFF
				 - (Up/Down) : irregular toggle
				 
	[   MAIN   ]
	 
	 (led_check)   1. GRN, RED, YELLOW 1Line check
				   2. ID check - TOP : RED / BOTTOM : GRN
				   3. CAN, STS Led Check
	 
	 ( while() )   Save Rx_CAN_Data into the Flag.
				   Read State and Display.
*/

/*

	[17.10.12] - 김포 공장 테스트
		1. 화살표 모양 변경 (타업체와 동일하게 : 보편적인것이 좋다....)
		2. 대기상태(방향 신호가 안들어 올때) 아무것도 표시 안하게 (CHARACTER 배열 마지막 STOP)

	[17.10.13] - 추가 요청
		1. Dipsw 설정으로 UP & DOWN 설정 할 수 있도록 (기존에는 TOP, BOTTOM 동일하게 UP 출력)
		2. Dipsw 설정으로 대기상태(STOP)에서 방향에 따른 화살표 표시 (깜빡임 기능 추가)
				
		-+--------+--------+--------+--------+--------+-
         |  DIPSW |    1   |    2   |    3   |    4   |
		-+--------+--------+--------+--------+--------+-
		 |  기 능 |   UP   | Scroll |  Fade  |   TOP  |
		 |        |  DOWN  |        | IN,OUT | BOTTOM |
		-+--------+--------+--------+--------+--------+-
	[17.10.20]
		1. version 표시 변경 (우 -> 좌) 스크롤
		 - 숫자 조합은 가능하나 알파벳(10.20/ V,E,R)한정 및 1.1 사이 '.'(dot) 은 고정
		 
	[23.02.15]
		ver 1.2
		 - PCB 구조 변경 (회로 동일)
		 - 접점 변경
		  1. UP		- 	DIR
		  2. DOWN	- 	N.E
		  3. ERR	- 	AUT
		  4. TEST 	- 	INS
		  5. FIRE	-	ERR
		  6. AUTO	-	FR
		  7. BUZZ	-	STP		// Scroll Stop signal
		  
		  
	[23.03.20]
		ver 1.3
		 - Error code 표시 추가 (Only Can통신)
*/

#include <stdio.h>
#include "NUC1xx.h"
#include "DrvGPIO.h"
#include "DrvCAN.h"
#include "DrvSYS.h"
#include "DrvTIMER.h"
#include "DrvSPI.h"

#include "DrvUART.h"

#include "MPD-1616.h"

extern char GetChar(void);

/*------------------------------------------------------*/
//
//			FUNCTION PROTOTYPE DEFINE
//
/*------------------------------------------------------*/
void Delay_ms(int time);
	
void TxCAN_Req(void);
//void CAN_ShowMsg(STR_CANMSG_T* Msg);
void RxCAN_Setting(void);

void LED_Check(void);

void Read_Status(void);

void Display_Version_Text(void);
void Display_Version_Number(void);

void Display_Error_code(BYTE bErrorCode);
/*------------------------------------------------------*/
//
//			VARIABLE
//
/*------------------------------------------------------*/

/* external variable */
extern const WORD FONT_NUMBER1616[10][16];
extern const WORD FONT_CHARACTER[9][16];	
extern const WORD FONT_NUMBER0816[15][16];
extern const WORD FONT_CHECK[16];
extern const WORD FONT_ERROR[16];

//UART
STR_UART_T param;

// Status Flag
STS_FLAG S_Flag;

// CAN
STR_CANMSG_T rrMsg;			// CAN Recive
STR_CANMSG_T msg1;   		// for CAN 구조체 선언

MSG_STATUS_ESCALATOR MsgSTES;		// Status Struct
CAN_FLAG C_Flag;					// CAN_Flag Struct
BYTE fCanRxES_ST, fCan_TxStart, bCan_delay;		// CAN Tx flag, delay count

// Dip SW
BYTE bDipsw = 0;
WORD wDeviceAddr;
volatile BYTE fUPDOWN = FALSE, fScrolling = FALSE;

// Display - Arrow Scroll
volatile BYTE fArrowScroll, bArrow_Scroll, fScrollSLOW;
volatile WORD wArrowCnt, wSpeed = 320;

// Display
volatile BYTE fMIX, fYelblink;
volatile BYTE bRowLine, bColor, bDisplay = 8, bPreDir;
WORD wColLineBuf[16];
WORD wColLineBuf_Sub[16];

// LED
volatile BYTE bLED_STS, bPre_LED_STS;

// Fade_In/Out
BYTE fFade_InOut, fFadeIN;
int bFrameCnt = 5;

// Time Count
BYTE b10msCnt;


BYTE gbErrorCode = 0;

BYTE bText[4] = {14, 11, 12, 13};	// { , V, E, R, }
BYTE bVer[2]  = { 1,  3};				// ver 1.3

/*------------------------------------------------------//
//
//			TIMER
//
//------------------------------------------------------*/

void Delay_ms(int time)		// 10ms
{	
	int i = 0;
	for(i = 0; i < time; i ++)
	{		
		DrvSYS_Delay(10000);	// Parameters : [us]
		wdt_reset();
	}
}

/*------------------------------------------------------
//								TIMER _ initialization
//-----------------------------------------------------*/
void TIMER_Init()
{
	// Timer 0,1,2,3 clock source - XTAL 14.7456MHz (internal 22.1184 MHz)
	DrvSYS_SelectIPClockSource(E_SYS_TMR0_CLKSRC, 0);		// XTAL(0) : 14.7456MHz
	DrvSYS_SelectIPClockSource(E_SYS_TMR1_CLKSRC, 0);		// XTAL(0) : 14.7456MHz
	DrvSYS_SelectIPClockSource(E_SYS_TMR3_CLKSRC, 0);		// XTAL(0) : 14.7456MHz

	DrvTIMER_Init();
	
	
	SYSCLK->APBCLK.TMR3_EN = 1;					// Timer3 Clock Enable
	TIMER3->TISR.TIF 	   = 1;					// Timer Interrupt Flag (TDR value matches the TCMP value)
	TIMER3->TCSR.MODE 	   = E_ONESHOT_MODE;	// 00 : One-shot Mode
	TIMER3->TCSR.PRESCALE  = 14;				// clock sourece is divided by (PRESCALE + 1)
	TIMER3->TCMPR 		   = 1000;				// Timer Compared Value
	// Time-Out period = Clock * (8bit)(Prescale + 1) * (24bit)TCMP
	
	// Sys Clock 	: 22,118,400 Hz
	// Prescale  	: 21
	// Timer Clk 	: SYS Clk / (Prescale + 1) = 1,005,382 Hz
	// Timer period : 9.9464 * e-7 = 1 us
	
	// Sys Clock 	: 14,745,600 Hz
	// Prescale  	: 14
	// Timer Clk 	: SYS Clk / (Prescale + 1) = 983,040 Hz
	// Timer period : 9.9464 * e-7 = 1 us
	

	DrvTIMER_Open(E_TMR0, 1000, E_PERIODIC_MODE);		// TIMER0 : 1/1000 =  1ms , Periodic Mode
	DrvTIMER_Open(E_TMR1, 100,	E_PERIODIC_MODE);		// TIMER1 : 1/100  = 10ms , Periodic Mode

	DrvTIMER_EnableInt(E_TMR0);
	DrvTIMER_EnableInt(E_TMR1);
	DrvTIMER_EnableInt(E_TMR3);	

	DrvTIMER_Start(E_TMR0);
	DrvTIMER_Start(E_TMR1);
}

/*------------------------------------------------------
//				Display				TIMER0 _ 1 ms 
//-----------------------------------------------------*/
void TMR0_Callback(void) 
{	
// Arrow scroll
	if(fArrowScroll)
	{
		wArrowCnt++;
		//printf("ACnt = %d, bspeed = %d, Scroll = %d\n",wArrowCnt, wSpeed, bArrow_Scroll);
		if(wArrowCnt == wSpeed)
		{
			wArrowCnt = 0;
			bArrow_Scroll++;
			
			if(bArrow_Scroll >= 16)
			{
				bArrow_Scroll = 0;
			}
			
			if(fScrollSLOW)							// 감속
			{
				if(wSpeed < 240)					// 감속 중.. 
					wSpeed += 50;
				else if(bArrow_Scroll == 1)
				{
					bArrow_Scroll = 0;
					fArrowScroll = FALSE;
					fScrollSLOW = FALSE;					
				}
				bFrameCnt = 200;					// ScrollSLOW 중 TCMPR값 감소 때문
				fFadeIN = TRUE;						// ScrollStop 후 정상상태 유지 후 FadeOut
													//(bFrameCnt > 200, FadeIN => 정상 5초 유지)
			}
			
			else									// 운행 중
			{
				if(wSpeed > 160)					// 가속 중..
					wSpeed -= 30;
			}
		}
	}	
	
// Display
	
	DrvGPIO_SetPortBits(E_GPA, 0xFFFF);		// Display 초기화 -> 잔상 제거	
	Latch_OFF();						// Latch Output at rising edge 	
	
	bRowLine++;	
	
	if(bRowLine >= 16)
	{
		bRowLine = 0;
// Fade_IN / OUT
		if(fFade_InOut)
		{
			bFrameCnt++;			// 95 10 100 255 254 / 190 5 200 510 508

			if(bFrameCnt < 190)    // 16ms * 95 = 1520ms = 1.5s
			{
				if(fFadeIN)			// TIMER3->TCMPR값이 1000 이상이면 display에 영향을 줌 (950 미만 유지)
				{
					TIMER3->TCMPR = 5 * bFrameCnt;
				}
				else				// TIMER3->TCMPR값이 0이면 timer OFF (5 이상 유지)
				{
					TIMER3->TCMPR = 5 * (190 - bFrameCnt);
				}
			}
			
			else if((bFrameCnt <= 510) && fFadeIN)		// 16ms * (255 - 95) = 2560 ms = 2.5s
			{
				TIMER3->TCMPR = 0;		// TIMER3 OFF
				
				if(bFrameCnt == 508)
					fFade_InOut = FALSE;
			}			

			else
			{
				bFrameCnt = 5;				
				fFadeIN ^= 1;
			}
		}
		//printf("Fade_%s, Cnt = %3d,    tcmp = %3d \n", (fFadeIN)?"IN ":"OUT",bFrameCnt, TIMER3->TCMPR);
	}		

	// Col Output -> SPI
	DrvSPI_SingleWrite(eDRVSPI_PORT0, &wColLineBuf[bRowLine]);

	// Row Output
	DrvGPIO_SetPortBits(E_GPA, ~(0x01 << bRowLine));
	
	if(fMIX && (bDisplay == FIRE))		// Yellow color Display
	{				
		Latch_R = 1;
		
		DrvSPI_SingleWrite(eDRVSPI_PORT0, &wColLineBuf_Sub[bRowLine]);
		DrvGPIO_SetPortBits(E_GPA, ~(0x01 << bRowLine));
		
		Latch_G = 1;
	}
	
	// TESTPOINT 1, PWM Testing
	GPB_15 = 1;
	
	// column Output _ Latch ON & OutPut Enable
	switch(bColor)
	{
		case GRN :		GRN_ON();		break;
		case RED :		RED_ON();		break;
		case YELLOW :	YEL_ON();		break;
		default : 
			GPB_11 = 1;
			GPE_5 = 1;
		
			Latch_OFF();	
			break;
	}	

	if(fFade_InOut)
		DrvTIMER_Start(E_TMR3);
}

/*------------------------------------------------------
//				Time Count				TIMER1 _ 10 ms 
//-----------------------------------------------------*/

void TMR1_Callback(void){
	static BYTE bON_Time = 0;
	static BYTE bYellowCnt = 0;
	
	if(fCan_TxStart)					// CAN Tx Overtime Count
	{
		bCan_delay++;
	}
	else
	{
		bCan_delay = 0;
	}	
	
	b10msCnt++;
	bYellowCnt++;

// LED STATUS
	switch(bPre_LED_STS)
	{
		case ERROR :					// ERROR - 0.2s toggle
	
			if(b10msCnt > 20)
			{
				STATUS_LED_Toggle();
				b10msCnt = 0;
			}			
		break;
			
		case FIRE :						// FIRE - 0.5s toggle			
			if(b10msCnt < 50)
			{
				STATUS_LED_ON();				
			}
			else if(b10msCnt < 100)
			{
				STATUS_LED_OFF();
			}
			else
				b10msCnt = 0;			
		break;
		
		case INSP :						// TEST - ALWAYS STS_LED_ON
			STATUS_LED_ON();
			b10msCnt = 0;			
		break;
		
		case UP : //case DOWN :			// UP, DOWN 
			if(b10msCnt < bON_Time)
				STATUS_LED_ON();
			else if(b10msCnt < (100 - bON_Time))
			{
				STATUS_LED_OFF();
			}
			else
			{
				b10msCnt = 0;
				if(bON_Time < 5)
					bON_Time = 49;
				else
					bON_Time -= 5;
			}
		break;
		
		case AUTO :						// AUTO - 0.4s ON / 0.6s OFF
			if(b10msCnt < 40)				//  10ms * 40 = 400ms
			{
				STATUS_LED_ON();
			}
			else if(b10msCnt < 100)		// 	10ms * (100 - 40) = 600ms
			{
				STATUS_LED_OFF();
			}
			else
			{
				b10msCnt = 0;
			}
		break;
		
		default : STATUS_LED_OFF(); break;
	}
	
	if(fMIX)
	{
		if(bYellowCnt <= 50)				// Yellow _ Red Blink - 500 ms
		{		
			fYelblink = TRUE;			
		}
		else if(bYellowCnt <= 100)
		{
			fYelblink = FALSE;
		}
		else
			bYellowCnt = 0;
	}
	else
		bYellowCnt = 0;
}

/*------------------------------------------------------
//				Fade In/Out				TIMER3 _ 1 ms 
//-----------------------------------------------------*/

void TMR3_Callback(void){
	
	if(fFade_InOut)
	{
		DrvGPIO_SetPortBits(E_GPA, 0xFFFF);
		// TESTPOINT 1, PWM testing
		GPB_15 = 0;
	}	
}

/*------------------------------------------------------//
//	
//			CAN
//
//------------------------------------------------------*/

void CAN_CallbackFn(uint32_t u32IIDR){
	
	CAN_LED_Toggle();
	
	switch(u32IIDR)
	{
		case (1 + 1) :

			DrvCAN_ReadMsgObj((u32IIDR - 1), TRUE, &rrMsg);
		
			MsgSTES.bMode 		  	  =	rrMsg.Data[0];	// mode
			MsgSTES.Status[0] 		  =	rrMsg.Data[1];	// status[0]
			MsgSTES.Status[1] 		  =	rrMsg.Data[2];	// status[1]
			MsgSTES.Status[2] 		  =	rrMsg.Data[3];	// status[2]
			MsgSTES.bSavedErrorCount  =	rrMsg.Data[4];	// SaveErrorCount
			MsgSTES.bCurrentErrorCode =	rrMsg.Data[5];	// CurrentErrorCode
			MsgSTES.bPessraeErrorCode =	rrMsg.Data[6];	// SubErrorCode
			
			if(MsgSTES.bMode == 0x01)
				fCanRxES_ST = TRUE;
			//CAN_ShowMsg(&rrMsg);
			break;
		
		default :	break;
	}
	//CAN_LED_OFF();
}

/*------------------------------------------------------
//				CAN _ RX
//-----------------------------------------------------*/

void Rx_CAN_Setting(void)
{
	DrvCAN_SetRxMsgObj(MSG(1), CAN_STD_ID, STATUSMSG_ID, TRUE);
	
	DrvCAN_EnableInt(CAN_CON_IE | CAN_CON_SIE);
	DrvCAN_InstallCallback(CALLBACK_MSG, (CAN_CALLBACK)CAN_CallbackFn);
}
/*
void CAN_ShowMsg(STR_CANMSG_T* Msg)
{
	uint8_t i;
	printf("Read ID=%4X, Type=%s, DLC=%d, Data=", Msg->Id, Msg->IdType?"EXT":"STD", Msg->DLC);
	for(i=0;i<Msg->DLC;i++)
			printf("0x%02X,",Msg->Data[i]);
	printf("\n\n");
}
*/

/*------------------------------------------------------
//				CAN _ TX
//-----------------------------------------------------*/

void TxCAN_Req(void)
{
	STR_CANMSG_T t_Reg_Msg;
	
	DrvCAN_DisableInt(CAN_CON_IE | CAN_CON_SIE);
	
	t_Reg_Msg.FrameType = DATA_FRAME;
	t_Reg_Msg.IdType = CAN_STD_ID;
	t_Reg_Msg.Id = ReqCommTest_ID;			//	ReqCommTest_ID
	t_Reg_Msg.DLC = 2;
	
	t_Reg_Msg.Data[0] = wDeviceAddr;		// ID
	t_Reg_Msg.Data[1] = S_Flag.fPosition;	// TOP = 1, BOTTOM = 0
	
	DrvCAN_SetTxMsgObj(MSG(4), &t_Reg_Msg);
	
	bCan_delay = 0;
	fCan_TxStart = TRUE;
	
	while(!DrvCAN_SetTxRqst(MSG(4)))
	{
		wdt_reset();
		if(bCan_delay >= 4)	// 4 * 10ms = 40ms
			break;
	}
	
	fCan_TxStart = FALSE;
	
	DrvCAN_EnableInt(CAN_CON_IE | CAN_CON_SIE);
}


/*------------------------------------------------------//
//
//			PORT
//
//------------------------------------------------------*/

void Port_Init()
{
/*_______________ PORTA __________________*/	
/*  		0 : 15      PROW			  */
	DrvGPIO_Open(E_GPA,  0, E_IO_OUTPUT);
	DrvGPIO_Open(E_GPA,  1, E_IO_OUTPUT);
	DrvGPIO_Open(E_GPA,  2, E_IO_OUTPUT);
	DrvGPIO_Open(E_GPA,  3, E_IO_OUTPUT);
	DrvGPIO_Open(E_GPA,  4, E_IO_OUTPUT);
	DrvGPIO_Open(E_GPA,  5, E_IO_OUTPUT);
	DrvGPIO_Open(E_GPA,  6, E_IO_OUTPUT);
	DrvGPIO_Open(E_GPA,  7, E_IO_OUTPUT);
	DrvGPIO_Open(E_GPA,  8, E_IO_OUTPUT);
	DrvGPIO_Open(E_GPA,  9, E_IO_OUTPUT);
	DrvGPIO_Open(E_GPA, 10, E_IO_OUTPUT);
	DrvGPIO_Open(E_GPA, 11, E_IO_OUTPUT);
	DrvGPIO_Open(E_GPA, 12, E_IO_OUTPUT);
	DrvGPIO_Open(E_GPA, 13, E_IO_OUTPUT);
	DrvGPIO_Open(E_GPA, 14, E_IO_OUTPUT);
	DrvGPIO_Open(E_GPA, 15, E_IO_OUTPUT);
	
	DrvGPIO_SetPortBits(E_GPA, 0xFFFF);			// PORTA - ROW OUTPUT (P-Channel)
												// 'H' - No Output / 'L' - Output	
//	GPIOA->u32PMD = 0x55555555;
//	GPIOA->DOUT   = 0xFFFF;

/*_______________ PORTB __________________*/	
/*         0 : 3          DIPSW 0:3 	  */
/*         4 : 7            STATUS		  */	
	DrvGPIO_Open(E_GPB,  0, E_IO_INPUT);		// DIPSW0
	DrvGPIO_Open(E_GPB,  1, E_IO_INPUT);		// DIPSW1
	DrvGPIO_Open(E_GPB,  2, E_IO_INPUT);		// DIPSW2
	DrvGPIO_Open(E_GPB,  3, E_IO_INPUT);		// DIPSW3
	
	DrvGPIO_Open(E_GPB,  4, E_IO_INPUT);		// ERROR
	DrvGPIO_Open(E_GPB,  5, E_IO_INPUT);		// TEST
	DrvGPIO_Open(E_GPB,  6, E_IO_INPUT);		// FIRE	
	DrvGPIO_Open(E_GPB,  7, E_IO_INPUT);		// AUTO
	
	DrvGPIO_Open(E_GPB,  8, E_IO_OUTPUT);	// Not Used
	
	DrvGPIO_Open(E_GPB,  9, E_IO_OUTPUT);		// LATCH_CLK_GRN
	DrvGPIO_Open(E_GPB, 10, E_IO_OUTPUT);		// LATCH_CLK_RED
	
	DrvGPIO_Open(E_GPB, 11, E_IO_OUTPUT);	 	// BackLight_GRN(PWM), Active LOW
	
	DrvGPIO_Open(E_GPB, 15, E_IO_OUTPUT);	// TESTPOINT 1
		
	DrvGPIO_SetPortBits(E_GPB, 0x0800);
	
/*_______________ PORTC __________________*/	

	DrvGPIO_Open(E_GPC,  1, E_IO_OUTPUT);		// SPI_CLK		(SPI)
	DrvGPIO_Open(E_GPC,  3, E_IO_OUTPUT);		// SPI_DATA		(SPI)
	
	DrvGPIO_Open(E_GPC,  8, E_IO_INPUT);		// UP
	DrvGPIO_Open(E_GPC,  9, E_IO_INPUT);		// DOWN
	
	DrvGPIO_Open(E_GPC, 10, E_IO_OUTPUT);	// TESTPOINT 2
	DrvGPIO_Open(E_GPC, 11, E_IO_OUTPUT);	// TESTPOINT 3
	
	DrvGPIO_Open(E_GPC, 14, E_IO_OUTPUT);		// CAN_LED
	DrvGPIO_Open(E_GPC, 15, E_IO_OUTPUT);		// STS_LED
	
	DrvGPIO_SetPortBits(E_GPC, 0xC000);
	
/*_______________ PORTE __________________*/	

	DrvGPIO_Open(E_GPE,  5, E_IO_OUTPUT);		// BackLight_RED (PWM), Active LOW
	
	DrvGPIO_SetPortBits(E_GPE, 0x0020);
	
/*_______________ PORTF __________________*/	
	
	DrvGPIO_Open(E_GPF,  4, E_IO_OUTPUT);		// BUZZER_OUT (PWM)
	DrvGPIO_Open(E_GPF,  5, E_IO_INPUT);		// BUZZER_IN
	
	DrvGPIO_SetPortBits(E_GPF, 0x0000);	
	
	// CAN_FLAG Init
	C_Flag.fUp 		= FALSE;
	C_Flag.fDown 	= FALSE;
	C_Flag.fAuto 	= FALSE;
	C_Flag.fInsp 	= FALSE;
	C_Flag.fFire 	= FALSE;
	C_Flag.fError	= FALSE;
	C_Flag.fU_Buzz 	= FALSE;
	C_Flag.fD_Buzz 	= FALSE;
	
	// STS_FLAG Init
	S_Flag.fPosition = FALSE;
	S_Flag.fUp		 = FALSE;
	S_Flag.fDown	 = FALSE;
	S_Flag.fError	 = FALSE;
	S_Flag.fINSP	 = FALSE;
	S_Flag.fFire	 = FALSE;
	S_Flag.fAUTO	 = FALSE;
	S_Flag.fBuzzer	 = FALSE;
}

/*------------------------------------------------------*/
//
//			MAIN PROGRAM
//
/*------------------------------------------------------*/

int main (void)
{
	int i = 0;
	
/*------------------------------------------------------
//					Clock _ Set
//-----------------------------------------------------*/
	UNLOCKREG();

	DrvSYS_SetOscCtrl(E_SYS_OSC22M, 1);
	DrvSYS_Delay(20000);

	while(!SYSCLK->CLKSTATUS.OSC22M_STB);
	
	DrvSYS_SetOscCtrl(E_SYS_XTL12M, 1);
	DrvSYS_Delay(20000);
	
	while(!SYSCLK->CLKSTATUS.XTL12M_STB);

// 0 : Ex 4 ~ 24 MHz,	 1 : Ex 32 K,		2 : PLL,		3 : In 10 K,		7 : In 22 MHz
	DrvSYS_SelectHCLKSource(7);

	DrvSYS_SelectIPClockSource(E_SYS_WDT_CLKSRC, 0x2);			// 	WDT	  : HCLK / 2048 Clock

	DrvSYS_SelectIPClockSource(E_SYS_UART_CLKSRC,0x3);			// 	UART  : Internal 22.1184MHz
	
//	CLK->APBCLK1 |= CLK_APBCLK1_PWM0_EN_Msk | CLK_APBCLK1_PWM1_EN_Msk;
//	CLK->CLKSEL3 |= CLK_CLKSEL3_PWM0_S_Msk | CLK_CLKSEL3_PWM1_S_Msk;

	// Watch Dog Setting
	DrvWDT_Open(E_WDT_LEVEL5);					// Watch Dog Level 5 -> 1.3 ms
	DrvWDT_Ioctl(E_WDT_IOC_ENABLE_INT, 0);
	DrvWDT_Ioctl(E_WDT_IOC_START_TIMER, 0);		// Watchdog Start

	LOCKREG();
/*------------------------------------------------------*/
	Port_Init();
	
	TIMER_Init();
	// Install callback "TMR0_Callback" and trigger callback when Interrupt happen always
	(void)DrvTIMER_SetTimerEvent(E_TMR0, 1, (TIMER_CALLBACK)TMR0_Callback, 0);
	(void)DrvTIMER_SetTimerEvent(E_TMR1, 1, (TIMER_CALLBACK)TMR1_Callback, 0);
	(void)DrvTIMER_SetTimerEvent(E_TMR3, 1, (TIMER_CALLBACK)TMR3_Callback, 0);

	// Select CAN Multi-Function
	DrvGPIO_InitFunction(E_FUNC_CAN0);
	DrvCAN_Init();

	// CAN Send Tx Msg
	/*Tseg2, Tseg1, Sjw, Brp*/
	DrvCAN_SetTiming(4, 5, 1, 15);
	// (3, 3, 0, 2) - 819.2 KBPS/ (3, 3, 1, 29) - 81.92 KBPS/ (4, 5, 1, 15) - 115.2 KBPS

	// Module interrupt, Status Change interrupt enable
	DrvCAN_EnableInt(CAN_CON_IE | CAN_CON_SIE);

	// SPI Setting
	DrvGPIO_InitFunction(E_FUNC_SPI0);
	DrvSPI_Open(eDRVSPI_PORT0, eDRVSPI_MASTER, eDRVSPI_TYPE5, 16);
	DrvSPI_SetEndian(eDRVSPI_PORT0, eDRVSPI_LSB_FIRST);
	// Rising edge - Tx, Falling edge - Rx, data length = 16 bit
	DrvSPI_SetClockFreq(eDRVSPI_PORT0, 20000000, 0);		// 1.83 MHz
	
	// Init GPIO and configure UART0
	DrvGPIO_InitFunction(E_FUNC_UART2);
	param.u32BaudRate 		= 115200;
	param.u8cDataBits 		= DRVUART_DATABITS_8;
	param.u8cStopBits 		= DRVUART_STOPBITS_1;
	param.u8cParity 		= DRVUART_PARITY_NONE;
	param.u8cRxTriggerLevel = DRVUART_FIFO_1BYTES;
	param.u8TimeOut 		= 0;
	DrvUART_Open(UART_PORT2, &param);
	
/*************** Dipsw 읽기*****************************/
	bDipsw = (DrvGPIO_GetPortBits(E_GPB) & 0x000F);
	
	if(bDipsw & 0x8)
		S_Flag.fPosition = TOP;
	else
		S_Flag.fPosition = BOTTOM;
	
	wDeviceAddr = (WORD)(bDipsw & 0x7);			// Device ID	

	LED_Check();
	Rx_CAN_Setting();
	
	//printf("/************** CAN SETTING **************/\n");
	//printf("Tseg1 : %d \tTseg2 : %d \tSJW : %d \tBpr : %d \n", CAN->BTIME.TSEG1,CAN->BTIME.TSEG2 ,CAN->BTIME.SJW ,(CAN->BTIME.BRP) | (CAN->BRPE.BPRE <<6));
	//printf("CAN HCLKFreq : %d  [Hz]\n",DrvSYS_GetHCLKFreq());	
	//printf("CAN BITRATE  : %8d  [kbps]\n",DrvCAN_GetCANBitRate());
	//printf("CAN BRP      : %d\n", (CAN->BTIME.BRP) | (CAN->BRPE.BPRE <<6));
	//printf("/*****************************************/\n");
	
//------------------------------------------------------------------	
	//printf("ID : %d,	Position = %s\n", (bDipsw & 0x07), (S_Flag.fPosition)?"TOP":"BOTTOM");

	while(1){

		wdt_reset();
		
		(GPB_0) ? (fUPDOWN = TRUE)    : (fUPDOWN = FALSE);
		(GPB_1) ? (fScrolling = TRUE) : (fScrolling = FALSE);
		
		if(fCanRxES_ST)
		{
			fCanRxES_ST = FALSE;
			
			C_Flag.fUp 		=  MsgSTES.Status[1] & 0x01;
			C_Flag.fDown 	= (MsgSTES.Status[1] & 0x02) ? TRUE : FALSE;
			C_Flag.fAuto 	= (MsgSTES.Status[1] & 0x04) ? TRUE : FALSE;		// 자동
			C_Flag.fInsp 	= (MsgSTES.Status[1] & 0x08) ? TRUE : FALSE;		// 수동
 			C_Flag.fError	= (MsgSTES.Status[1] & 0x10) ? TRUE : FALSE;
			C_Flag.fFire 	= (MsgSTES.Status[1] & 0x20) ? TRUE : FALSE;
			C_Flag.fU_Buzz  = (MsgSTES.Status[1] & 0x40) ? TRUE : FALSE;		// TOP MultiPost Buzzer
			C_Flag.fD_Buzz  = (MsgSTES.Status[1] & 0x80) ? TRUE : FALSE;		// BOTTOM MultiPost Buzzer
			
			if((MsgSTES.Status[0] & 0x03) || (MsgSTES.Status[2] & 0x03))
				C_Flag.fMove = TRUE;
			else
				C_Flag.fMove = FALSE;
			
			
			if(MsgSTES.bCurrentErrorCode)
			{
				gbErrorCode = MsgSTES.bCurrentErrorCode;
			}
			else
			{
				gbErrorCode = 0;
			}			
		}
		
		Read_Status();		

		// TxCAN_Req();			
		
		// -------------------------------------------------- Display		
		// Up_Scroll
		
		if(fArrowScroll && S_Flag.fUp && (S_Flag.fPosition == BOTTOM))
		{
			fFade_InOut = FALSE;		// 정지 후 다시 시작할때 Fade_InOut이 걸려있음
			
			S_Flag.fUp  = FALSE;
			for(i = 0; i < 16; i++)	
				wColLineBuf[i] = FONT_CHARACTER[bDisplay][((i + bArrow_Scroll) % 16) & 0xF];
				
		}
		// Down_Scroll
		else if(fArrowScroll && S_Flag.fDown && (S_Flag.fPosition == TOP))
		{
			fFade_InOut = FALSE;		// 정지 후 다시 시작할때 Fade_InOut이 걸려있음
			
			S_Flag.fDown = FALSE;
			for(i = 0; i < 16; i++)
			{
				if(fUPDOWN)
					wColLineBuf[i] = FONT_CHARACTER[bDisplay][((i - bArrow_Scroll) % 16) & 0xF];
				else
					wColLineBuf[i] = FONT_CHARACTER[bDisplay][((i + bArrow_Scroll) % 16) & 0xF];
			}
		}

		// else Display		
		else
		{
			if(bDisplay == ERROR_CODE)
			{
				Display_Error_code(gbErrorCode);
			}
			else
			{
				if((!GPB_2) || fMIX)		// MIX일때 FadeInOut X
					fFade_InOut = FALSE;
				else if(GPB_2)				// DIPSW2 _ Fade ON/OFF
					fFade_InOut = TRUE;
			
				for(i = 0; i < 16; i++)
				{
					wColLineBuf[i] = FONT_CHARACTER[bDisplay][i];
					
					if(fMIX && (bDisplay == FIRE))
						wColLineBuf_Sub[i] = FONT_CHARACTER[bDisplay + 1][i];
				}
			}
		}		
	}
}

/*------------------------------------------------------
//		시작시 LED 확인						LED_CHECK()
//-----------------------------------------------------*/
void LED_Check(void)
{ 
	BYTE i = 0, j = 0, k = 1;	
	
	for(i = 0; i < 16; i++)				// line test
	{
		for(j = 0; j < 16; j++)
		{		
			wColLineBuf[j] = FONT_CHECK[((i - j) % 16) & 0xF];
		}		
		for(k = 1; k <= 3; k++)			// color test
		{
			bColor = k;
			Delay_ms(2);
		}		
	}
/*
	 // number check
	for(i = 0; i < 10; i++)
	{
		bColor = YELLOW;
		for(j= 0; j < 16; j++)
			wColLineBuf[j] = FONT_NUMBER1616[i][j];
		
		Delay_ms(200);
	}
*/
/*	
	// version display
	for(i = 0; i < 16; i++)
	{
		if(S_Flag.fPosition == TOP)
			bColor = RED;
		else
			bColor = GRN;
		
		wColLineBuf[i] = FONT_VERSION[VER][i] | (FONT_VERSION[wVer[0]][i] << 8) | FONT_VERSION[wVer[1]][i];
	}	
*/
	if(S_Flag.fPosition == TOP)
		bColor = RED;
	else
		bColor = GRN;		
	
	// version display_move
	Display_Version_Text();
	Display_Version_Number();
	Delay_ms(200);
	
	bColor = GRN;
	// clear
	for(i = 0; i < 16; i++)
	{
		wColLineBuf[i] = 0;
		wColLineBuf_Sub[i] = 0;
	}	

	// LED check
	CAN_LED_ON();
	STATUS_LED_ON();
	
	Delay_ms(100);
	
	CAN_LED_OFF();
	STATUS_LED_OFF();
}


void Read_Status(void)
{	
	// 2023.02.15
	// 접점 방식만 프로그램 진행함
	// CAN통신은 나중에 프로토콜 확정되면 추가 예정
	// --------------------------------------------------------- ERROR
	if(C_Flag.fError || (IN_ERR == 1))
	{	
		bLED_STS = ERROR;
		
		if(gbErrorCode)
		{
			fFade_InOut = FALSE;		// display 변경 할 때 Fade걸려 있음. (확인 필요!!)
			bDisplay = ERROR_CODE;
		}
		else
		{
			bDisplay = ERROR;
		}
		bColor = RED;		
		//printf(" ERROR ");
	}
	
	// --------------------------------------------------------- FIRE	
	if(C_Flag.fFire || (IN_FIRE == 1))
	{		
		bLED_STS = FIRE;
		
		bDisplay = FIRE;
		(fYelblink) ? (bColor = YELLOW) : (bColor = RED);
		fMIX = TRUE;		
		//printf(" FIRE ");
	}
	
	// --------------------------------------------------------- INSP	
	if(C_Flag.fInsp || (IN_INS == 1))
	{
		bLED_STS = INSP;
		
		bDisplay = INSP;
		bColor = RED;
		
		//printf(" TEST ");
	}
	
	if(IN_DIR == 1)
	{
		if(S_Flag.fPosition == BOTTOM)
		{
			bDisplay = UP;
			bColor = GRN;
			
			bLED_STS = AUTO;
			
			S_Flag.fUp = TRUE;
			
			if(IN_STP == 1)
			{
				fScrollSLOW = TRUE;
			}
			else
			{
				fArrowScroll = TRUE;
			}
				
			S_Flag.fUp = TRUE;

			bPreDir = UP;
		}
		else if(S_Flag.fPosition == TOP)
		{				
			(fUPDOWN) ? (bDisplay = DOWN) : (bDisplay = UP);
			bColor = GRN;
			
			bLED_STS = AUTO;
			
			S_Flag.fDown = TRUE;
			
			if(IN_STP == 1)
			{
				fScrollSLOW = TRUE;
			}
			else
			{
				fArrowScroll = TRUE;
			}
			
			(fUPDOWN) ? (bPreDir = DOWN) : (bPreDir = UP);
		}
	}
	
	if(IN_NE == 1)
	{		
		bColor = RED;
		(fYelblink) ? (bDisplay = DONT) : (bDisplay = DONT_sub);
				
		fMIX = TRUE;
		
		bPreDir = DONT;
	}	

	// --------------------------------------------------------- Yellow _ Red Blink Flag init
	
	if((bDisplay != DONT) && (bDisplay != FIRE) && (bDisplay != DONT_sub))
		fMIX = FALSE;
	
	// --------------------------------------------------------- BUZZER
//	if(((C_Flag.fU_Buzz && (S_Flag.fPosition == TOP))	 ||		// CAN Signal (at TOP)
//		(C_Flag.fD_Buzz && (S_Flag.fPosition == BOTTOM)) ||		// CAN Siganl (at BOTTOM)
//		(IN_BUZZER == 1)) && (S_Flag.fBuzzer == FALSE))			// INPUT Signal
//	{
//		S_Flag.fBuzzer = TRUE;				// When the buzz signal is received, keep the buzz ON
//		OUT_BUZZER = TRUE;
//		//printf("Buzz_On  ");		
//	}
//	else if(C_Flag.fU_Buzz == C_Flag.fD_Buzz == IN_BUZZER == 0)
//	{
//		S_Flag.fBuzzer = FALSE;
//		OUT_BUZZER = FALSE;
//	}
	
	// --------------------------------------------------------- LED_STATUS
	if(bLED_STS != bPre_LED_STS)
		bPre_LED_STS = bLED_STS;
}

void Display_Version_Number()
{
	int i = 0, bShift = 0;
	
	WORD wTemp[16] = {0};
	
	for(i = 0; i < 16; i++)
	{
		wTemp[i] = (FONT_NUMBER0816[bVer[0]][i] << 9) | FONT_NUMBER0816[bVer[1]][i];
		wTemp[12] |= 0x0100;
	}
	
	for(bShift = 0; bShift < 17; bShift++)
	{
		for(i = 0; i < 16; i++)
		{
			wColLineBuf[i] = (wColLineBuf_Sub[i] << (bShift + 1)) | wTemp[i] >> (16 - bShift);			
		}
		Delay_ms(5);
	}	
}

void Display_Version_Text(void)
{
	int i = 0, num = 0, bShift = 0;	

	for(num = 1; num <= 2; num++)
	{
		for(bShift = 0; bShift < 8; bShift++)
		{
			for(i = 0; i < 16; i++)
			{
				wColLineBuf[i] = (FONT_NUMBER0816[bText[num - 1]][i]<< (bShift + 8)) |
								 (FONT_NUMBER0816[bText[num]][i] << bShift) |
								 (FONT_NUMBER0816[bText[num + 1]][i] >> (8 - bShift));
				wColLineBuf_Sub[i] = wColLineBuf[i];
			}
			Delay_ms(5);
		}
	}		
}

void Display_Error_code(BYTE bErrorCode)
{
	BYTE i = 0, bShift = 0;
	WORD wErrorBuffer[2][16] = {0};
	
	// Error Buffer에 출력 데이터 저장 ex) 'E-99'
	for(i = 0; i < 16; i++)
	{
		wErrorBuffer[0][i] = FONT_ERROR[i];
		wErrorBuffer[1][i] = (FONT_NUMBER0816[bErrorCode / 10][i] << 8) | (FONT_NUMBER0816[bErrorCode % 10][i]);
	}
	
	// Frame 1. 'E-' -> '99'
	for(bShift = 1; bShift < 17; bShift++)
	{
		for(i = 0; i < 16; i++)
		{
			wColLineBuf[i] = (wErrorBuffer[0][i] << bShift) | (wErrorBuffer[1][i] >> (16 - bShift));
		}
		Delay_ms(15);
	}
	
	// Delay for ErrorCode Display
	Delay_ms(100);
	
	// Frame 2. '99' -> 'E-'
	for(bShift = 0; bShift < 17; bShift++)
	{
		for(i = 0; i < 16; i++)
		{
			wColLineBuf[i] = (wErrorBuffer[1][i] << bShift) | (wErrorBuffer[0][i] >> (16 - bShift));
		}
		Delay_ms(15);
	}
}

