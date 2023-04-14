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
		 - PCB 변경(180도 회전)으로 폰트 및 스크롤 프로그램 변경함
		 - 입력 신호 없을 경우 화면 표시 안함 (공백)
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

// Initialization
void sysclk_init(void);
void port_init(void);
void timer_init(void);
void can_init(void);
void spi_init(void);
void uart_init(void);

// Funtion
void Delay_ms(int time);	
void TxCAN_Req(void);
//void CAN_ShowMsg(STR_CANMSG_T* Msg);
void Rx_CAN_Setting(void);
void LED_Check(void);
void Read_Status(void);
void Display_Version(void);
void Display_Error_code(BYTE bErrorCode);

// Interrupt
void CAN_CallbackFn(uint32_t u32IIDR);
void TMR0_Callback(void);
void TMR1_Callback(void);
void TMR3_Callback(void);

/*------------------------------------------------------*/
//
//			VARIABLE
//
/*------------------------------------------------------*/

/* external variable */
extern const WORD TIMER_BAR[6][16];
extern const WORD TIMER_BAR2[6][16];
extern const WORD FONT_NUMBER1616[10][16];
extern const WORD FONT_CHARACTER[9][16];	
extern const WORD FONT_NUMBER0816[15][16];
extern const WORD FONT_CHECK[16];
extern const WORD FONT_ERROR[16];

//UART
STR_UART_T 		param;

// CAN
STR_CANMSG_T 	rrMsg;			// CAN Recive
STR_CANMSG_T 	msg1;   		// for CAN 구조체 선언

MSG_STATUS_ESCALATOR 	MsgSTES;		// Status Struct

BYTE fCanRxES_ST, fCan_TxStart, bCan_delay;		// CAN Tx flag, delay count

// Dip SW
BYTE bDipsw = 0;
WORD wDeviceAddr;

// Global Flag
GLOBAL_FLAG		gFlag;

// Display
DISPALY_INFO 	gDisplay;
WORD wColLineBuf[16];
WORD wColLineBuf_Sub[16];

// Display - Arrow Scroll
volatile BYTE fArrowScroll, bArrow_Scroll, fScrollSLOW;
volatile WORD wArrowCnt, wSpeed = 320;

// Time Count
WORD gwTimerCnt = 0;
BYTE gbTimerIndex = 0;

BYTE b10msCnt;

BYTE gbErrorCode = 0;

BYTE bVer[2]  = { 2,  0};				// ver 2.0


/*------------------------------------------------------*/
//
//			MAIN PROGRAM
//
/*------------------------------------------------------*/

int main (void)
{
	int i = 0;
	
	sysclk_init();
	
	port_init();	
	timer_init();
	can_init();
	spi_init();
	uart_init();
	
/*************** Dipsw 읽기*****************************/
	bDipsw = (DrvGPIO_GetPortBits(E_GPB) & 0x000F);
	
	if(bDipsw & 0x8)
		gFlag.bit.fPosition = TOP;
	else
		gFlag.bit.fPosition = BOTTOM;
	
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
	printf("ID : %d,	Position = %s\n", (bDipsw & 0x07), (gFlag.bit.fPosition)?"TOP":"BOTTOM");

	while(1){

		wdt_reset();
		
		bDipsw = (DrvGPIO_GetPortBits(E_GPB) & 0x000F);
		
		if(IN_DIR)
		{
			if(gFlag.bit.fStart == FALSE)
				gFlag.bit.fStart = TRUE;
		}
		else
		{
			if(gbTimerIndex >= 5)
				gFlag.bit.fStart = FALSE;
		}
		
		
		if(gFlag.bit.fStart)
		{
			gbTimerIndex = gwTimerCnt / 100;
			
			for(i = 0; i < 16; i++)
			{
				if(bDipsw == 0x00)
				{
					switch(gbTimerIndex)
					{	
						case 0 : case 1 :
							gDisplay.bColor = GRN;					
							wColLineBuf[i] = TIMER_BAR[gbTimerIndex][i];
							break;
						case 2 :
							gDisplay.bColor = YELLOW;
							wColLineBuf[i] = TIMER_BAR[gbTimerIndex][i];
							wColLineBuf_Sub[i] = TIMER_BAR[gbTimerIndex][i];
							break;						
						case 3 :
							gDisplay.bColor = RED;					
							wColLineBuf[i] = TIMER_BAR[gbTimerIndex][i];
							break;
						case 4 : case 5 :
							gDisplay.bColor = RED;
							gFlag.bit.fDisplayMix = TRUE;
						
							if(gFlag.bit.fYellowBlink)
								wColLineBuf[i] = FONT_CHARACTER[FONT_DONT][i];
							else
								wColLineBuf[i] = FONT_CHARACTER[FONT_DONTsub][i];
							break;
					}
				}
				else if(bDipsw == 0x01)
				{
					switch(gbTimerIndex)
					{	
						case 0 : case 1 : case 2 : case 3:
							gDisplay.bColor = YELLOW;
							//wColLineBuf[i] = TIMER_BAR[0][i];
							wColLineBuf_Sub[i] = TIMER_BAR2[gbTimerIndex][i];
							break;
						case 4 : case 5 :
							gDisplay.bColor = RED;
							wColLineBuf[i] = TIMER_BAR[0][i];
							break;
					}
				}
			}
		}
		else
		{
			gFlag.bit.fDisplayMix = FALSE;
			for(i = 0; i < 16; i++)
			{
				wColLineBuf[i] = 0;
				wColLineBuf_Sub[i] = 0;
			}
		}
	}
}
//------------------------------------------------------
//
//		INITIALIZATION
//
//------------------------------------------------------

//------------------------------------------------------
//		System Clock Initialization
//------------------------------------------------------
void sysclk_init(void)
{

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
}

//------------------------------------------------------
//		GPIO Port Initialization
//------------------------------------------------------
void port_init(void)
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
	
	DrvGPIO_Open(E_GPC,  8, E_IO_INPUT);		// DIR
	DrvGPIO_Open(E_GPC,  9, E_IO_INPUT);		// NE	
	DrvGPIO_Open(E_GPC, 10, E_IO_INPUT);		// STP, TESTPOINT 2
	
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
}

//------------------------------------------------------
//		Timer Initialization
//------------------------------------------------------
void timer_init(void)
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
	
	// Install callback "TMR0_Callback" and trigger callback when Interrupt happen always
	(void)DrvTIMER_SetTimerEvent(E_TMR0, 1, (TIMER_CALLBACK)TMR0_Callback, 0);
	(void)DrvTIMER_SetTimerEvent(E_TMR1, 1, (TIMER_CALLBACK)TMR1_Callback, 0);
	(void)DrvTIMER_SetTimerEvent(E_TMR3, 1, (TIMER_CALLBACK)TMR3_Callback, 0);
}

//------------------------------------------------------
//		CAN Initialization
//------------------------------------------------------
void can_init(void)
{
	// Select CAN Multi-Function
	DrvGPIO_InitFunction(E_FUNC_CAN0);
	DrvCAN_Init();

	// CAN Send Tx Msg
	/*Tseg2, Tseg1, Sjw, Brp*/
	DrvCAN_SetTiming(4, 5, 1, 15);
	// (3, 3, 0, 2) - 819.2 KBPS/ (3, 3, 1, 29) - 81.92 KBPS/ (4, 5, 1, 15) - 115.2 KBPS

	// Module interrupt, Status Change interrupt enable
	DrvCAN_EnableInt(CAN_CON_IE | CAN_CON_SIE);
}

//------------------------------------------------------
//		SPI Initialization
//------------------------------------------------------
void spi_init(void)
{
	// SPI Setting
	DrvGPIO_InitFunction(E_FUNC_SPI0);
	DrvSPI_Open(eDRVSPI_PORT0, eDRVSPI_MASTER, eDRVSPI_TYPE5, 16);
	DrvSPI_SetEndian(eDRVSPI_PORT0, eDRVSPI_LSB_FIRST);
	// Rising edge - Tx, Falling edge - Rx, data length = 16 bit
	DrvSPI_SetClockFreq(eDRVSPI_PORT0, 20000000, 0);		// 1.83 MHz
}

//------------------------------------------------------
//		UART Initialization
//------------------------------------------------------
void uart_init(void)
{
	// Init GPIO and configure UART0
	DrvGPIO_InitFunction(E_FUNC_UART2);
	param.u32BaudRate 		= 115200;
	param.u8cDataBits 		= DRVUART_DATABITS_8;
	param.u8cStopBits 		= DRVUART_STOPBITS_1;
	param.u8cParity 		= DRVUART_PARITY_NONE;
	param.u8cRxTriggerLevel = DRVUART_FIFO_1BYTES;
	param.u8TimeOut 		= 0;
	DrvUART_Open(UART_PORT2, &param);
}

//------------------------------------------------------
//
//		FUNTION
//
//------------------------------------------------------

//------------------------------------------------------
//		System Delay : 10ms
//------------------------------------------------------
void Delay_ms(int time)		// 10ms
{	
	int i = 0;
	for(i = 0; i < time; i ++)
	{		
		DrvSYS_Delay(10000);	// Parameters : [us]
		wdt_reset();
	}
}

//------------------------------------------------------
//		Can Rx Setting
//------------------------------------------------------
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

//------------------------------------------------------
//		Can Transmit
//------------------------------------------------------
void TxCAN_Req(void)
{
	STR_CANMSG_T t_Reg_Msg;
	
	DrvCAN_DisableInt(CAN_CON_IE | CAN_CON_SIE);
	
	t_Reg_Msg.FrameType = DATA_FRAME;
	t_Reg_Msg.IdType = CAN_STD_ID;
	t_Reg_Msg.Id = ReqCommTest_ID;				//	ReqCommTest_ID
	t_Reg_Msg.DLC = 2;
	
	t_Reg_Msg.Data[0] = wDeviceAddr;			// ID
	t_Reg_Msg.Data[1] = gFlag.bit.fPosition;	// TOP = 1, BOTTOM = 0
	
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

//------------------------------------------------------
//		LED Check : Check Led before Starting
//------------------------------------------------------
void LED_Check(void)
{ 
	BYTE i = 0, j = 0, k = 1;	
	
	for(i = 0; i < 16; i++)				// line test
	{
		for(j = 0; j < 16; j++)
		{		
			wColLineBuf[j] = FONT_CHECK[((i + j) % 16) & 0xF];
		}		
		for(k = 1; k <= 3; k++)			// color test
		{
			gDisplay.bColor = k;
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
	if(gFlag.bit.fPosition == TOP)
		gDisplay.bColor = RED;
	else
		gDisplay.bColor = GRN;		
	
	// version display_move
	Display_Version();
	Delay_ms(200);
	
	gDisplay.bColor = GRN;
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

//------------------------------------------------------
//		Read Status : Check Input signal
//------------------------------------------------------
// 2023.02.15
// 접점 방식만 프로그램 진행함
// CAN통신은 나중에 프로토콜 확정되면 추가 예정
void Read_Status(void)
{	
	INPUT_FLAG		sInput = {0};
	CAN_DATA		sCan = {0};
	
	if(fCanRxES_ST)
	{
		fCanRxES_ST = FALSE;
		
		if(gFlag.bit.fPosition == TOP)
		{
			if(MsgSTES.Status[1] & CAN_BIT_UP)
				sCan.fNE = TRUE;
			
			if(MsgSTES.Status[1] & CAN_BIT_DOWN)
				sCan.fDIR = TRUE;
		}
		else	// fPosition == BOTTOM
		{
			if(MsgSTES.Status[1] & CAN_BIT_UP)
				sCan.fDIR = TRUE;
			
			if(MsgSTES.Status[1] & CAN_BIT_DOWN)			
				sCan.fNE = TRUE;
		}
		
		sCan.fAuto  = (MsgSTES.Status[1] & CAN_BIT_AUTO ) ? TRUE : FALSE;
		sCan.fInsp  = (MsgSTES.Status[1] & CAN_BIT_INSP ) ? TRUE : FALSE;
		sCan.fError = (MsgSTES.Status[1] & CAN_BIT_ERROR) ? TRUE : FALSE;
		sCan.fFire  = (MsgSTES.Status[1] & CAN_BIT_FIRE ) ? TRUE : FALSE;
		sCan.fUBZ   = (MsgSTES.Status[1] & CAN_BIT_UBZ  ) ? TRUE : FALSE;
		sCan.fDBZ   = (MsgSTES.Status[1] & CAN_BIT_DBZ  ) ? TRUE : FALSE;		
		
		if(MsgSTES.bCurrentErrorCode)
		{
			gbErrorCode = MsgSTES.bCurrentErrorCode;
		}
		else
		{
			gbErrorCode = 0;
		}			
	}
	
	
	
	sInput.bit.fDIR   = (IN_DIR  | sCan.fDIR);
	sInput.bit.fNE 	  = (IN_NE   | sCan.fNE);
	//sInput.bit.fAuto  = (IN_AUT  | sCan.fAuto);
	sInput.bit.fInsp  = (IN_INS  | sCan.fInsp);
	sInput.bit.fError = (IN_ERR  | sCan.fError);
	sInput.bit.fFire  = (IN_FIRE | sCan.fFire);
	//sInput.bit.fBuzzer 	= IN_BUZZER;
	
	if(DIPSW2)	// DIPSW2 CAN-Auto Signal Arrow Scroll
	{
		if(MsgSTES.Status[1])
			sInput.bit.fStp = IN_STP | ~(sCan.fAuto);
		else
			sInput.bit.fStp = IN_STP;
	}
	else
	{
		sInput.bit.fStp = IN_STP;
	}
	
	if(sInput.bit.fDIR == FALSE)
	{
		fArrowScroll = FALSE;
	}
	
	if((sInput.wReg & (SINPUT_BIT_NE | SINPUT_BIT_FIRE)) == 0x0000)
	{
		gFlag.bit.fDisplayMix = FALSE;
	}
	
	if(DIPSW3)
	{
		switch(sInput.wReg)
		{
			case SINPUT_BIT_INS :
			case SINPUT_BIT_ERR :	gFlag.bit.fFadeMode = TRUE;		break;		
			default :				gFlag.bit.fFadeMode = FALSE;	break;
		}
	}
	else
	{
		gFlag.bit.fFadeMode = FALSE;
	}
	
	switch(sInput.wReg & 0x007F)
	{
		case SINPUT_NOTHING : /*default :*/
			gDisplay.eMode = Display_NOTHING;
			break;
		
		case SINPUT_BIT_DIR :
			if(gFlag.bit.fUpDown)
			{
				gDisplay.eMode = Dispaly_DOWN;
			}
			else
			{
				gDisplay.eMode = Display_UP;
			}
			
			gDisplay.bColor   = GRN;
			
			if(sInput.bit.fStp == TRUE)
			{
				fScrollSLOW = TRUE;
			}
			else
			{
				fArrowScroll = TRUE;
			}
			break;
			
		case  SINPUT_BIT_NE :
			gDisplay.eMode = Display_DONT;
			gDisplay.bColor = RED;			
			
			if(sInput.bit.fStp == TRUE)
				gFlag.bit.fDisplayMix = FALSE;
			else
				gFlag.bit.fDisplayMix = TRUE;
			break;
		
		case SINPUT_BIT_ERR :
			if(gbErrorCode)
			{	
				gFlag.bit.fFadeMode = FALSE;
				gDisplay.eMode = Display_ERROR_CODE;
			}
			else
			{
				gDisplay.eMode = Display_ERROR;
			}
			
			gDisplay.bColor = RED;
			break;
			
		case SINPUT_BIT_FIRE :
			gDisplay.eMode 	  = Display_FIRE;
		
			if(gFlag.bit.fYellowBlink)
			{
				gDisplay.bColor = YELLOW;
			}
			else
			{
				gDisplay.bColor = RED;
			}
			
			gFlag.bit.fDisplayMix = TRUE;
			break;
			
		case SINPUT_BIT_INS :
			gDisplay.eMode = Display_INSP;
			gDisplay.bColor = RED;
			break;
	}	// end switch(sInput.wReg)
	
	
	if(IN_BUZZER | sCan.fUBZ | sCan.fDBZ)
	{
		OUT_BUZZER = TRUE;
	}
	else
	{
		OUT_BUZZER = FALSE;
	}	
}

//------------------------------------------------------
//		Display Version
//------------------------------------------------------
void Display_Version(void)
{
	WORD wTemp[3][16] = {0};
	int i = 0, j = 0, bShift = 0;
	
	for(i = 0; i < 16; i++)
	{
		wTemp[0][i] = (FONT_NUMBER0816[TEXT_BK][i]) | (FONT_NUMBER0816[TEXT_V ][i] << 8);
		wTemp[1][i] = (FONT_NUMBER0816[TEXT_E ][i]) | (FONT_NUMBER0816[TEXT_R ][i] << 8);
		wTemp[2][i] = (FONT_NUMBER0816[bVer[0]][i]) | (FONT_NUMBER0816[bVer[1]][i] << 9);
	}
	
	wTemp[2][3] |= 0x0100;
	
	for(j = 0; j < 2; j++)
	{
		for(bShift = 1; bShift < 17; bShift++)
		{
			for(i = 0; i < 16; i++)
			{
				wColLineBuf[i] = (wTemp[j][i] >> bShift) | (wTemp[j + 1][i] << (16 - bShift));
			}
			Delay_ms(5);
		}
	}
}

//------------------------------------------------------
//		Display Error Code
//------------------------------------------------------
void Display_Error_code(BYTE bErrorCode)
{
	BYTE i = 0, bShift = 0;
	WORD wErrorBuffer[2][16] = {0};
	
	// Error Buffer에 출력 데이터 저장 ex) 'E-99'
	for(i = 0; i < 16; i++)
	{
		wErrorBuffer[0][i] = FONT_ERROR[i];
		wErrorBuffer[1][i] = (FONT_NUMBER0816[bErrorCode / 10][i]) | (FONT_NUMBER0816[bErrorCode % 10][i] << 8);
	}
	
	// Frame 1. 'E-' -> '99'
	for(bShift = 1; bShift < 17; bShift++)
	{
		for(i = 0; i < 16; i++)
		{
			wColLineBuf[i] = (wErrorBuffer[0][i] >> bShift) | (wErrorBuffer[1][i] << (16 - bShift));
		}
		Delay_ms(7);
	}
	
	// Delay for ErrorCode Display
	Delay_ms(100);
	
	// Frame 2. '99' -> 'E-'
	for(bShift = 0; bShift < 17; bShift++)
	{
		for(i = 0; i < 16; i++)
		{
			wColLineBuf[i] = (wErrorBuffer[1][i] >> bShift) | (wErrorBuffer[0][i] << (16 - bShift));
		}
		Delay_ms(7);
	}
}


//------------------------------------------------------
//
//		INTERRUPT SERVICE ROUTINE
//
//------------------------------------------------------

//------------------------------------------------------
//		CAN Callback Funtion (Rx : 0x0500 / STATUSMSG_ID)
//------------------------------------------------------
void CAN_CallbackFn(uint32_t u32IIDR)
{	
	CAN_LED_Toggle();
	
	switch(u32IIDR)
	{
		case (1 + 1) :

			DrvCAN_ReadMsgObj((u32IIDR - 1), TRUE, &rrMsg);
			
			if(rrMsg.Data[0] == 0x01)
			{
				MsgSTES.bMode 		  	  =	rrMsg.Data[0];	// mode
				MsgSTES.Status[0] 		  =	rrMsg.Data[1];	// status[0]
				MsgSTES.Status[1] 		  =	rrMsg.Data[2];	// status[1]
				MsgSTES.Status[2] 		  =	rrMsg.Data[3];	// status[2]
				MsgSTES.bSavedErrorCount  =	rrMsg.Data[4];	// SaveErrorCount
				MsgSTES.bCurrentErrorCode =	rrMsg.Data[5];	// CurrentErrorCode
				MsgSTES.bPessraeErrorCode =	rrMsg.Data[6];	// SubErrorCode
				fCanRxES_ST = TRUE;
			}
			//CAN_ShowMsg(&rrMsg);
			break;
		
		default :	break;
	}
	//CAN_LED_OFF();
}

//------------------------------------------------------
//		Timer0 Callback Funtion : 1ms
//------------------------------------------------------
void TMR0_Callback(void) 
{	
	static volatile BYTE 	bRowLine;
	static 			int 	bFrameCnt = 5;
	
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
				gFlag.bit.fFadeInOut = TRUE;		// ScrollStop 후 정상상태 유지 후 FadeOut
													//(bFrameCnt > 200, FadeIN => 정상 5초 유지)
			}			
			else									// 운행 중
			{
				if(wSpeed > 160)					// 가속 중..
					wSpeed -= 30;
			}
		}
	}
	else
	{
		wArrowCnt = 0;
		bArrow_Scroll = 0;
	}
	
// Display
	
	DrvGPIO_SetPortBits(E_GPA, 0xFFFF);		// Display 초기화 -> 잔상 제거	
	Latch_OFF();						// Latch Output at rising edge 	
	
	bRowLine++;	
	
	if(bRowLine >= 16)
	{
		bRowLine = 0;
// Fade_IN / OUT
		if(gFlag.bit.fFadeMode)
		{
			bFrameCnt++;			// 95 10 100 255 254 / 190 5 200 510 508

			if(bFrameCnt < 190)    // 16ms * 95 = 1520ms = 1.5s
			{
				if(gFlag.bit.fFadeInOut)			// TIMER3->TCMPR값이 1000 이상이면 display에 영향을 줌 (950 미만 유지)
				{
					TIMER3->TCMPR = 5 * bFrameCnt;
				}
				else				// TIMER3->TCMPR값이 0이면 timer OFF (5 이상 유지)
				{
					TIMER3->TCMPR = 5 * (190 - bFrameCnt);
				}
			}
			
			else if((bFrameCnt <= 510) && gFlag.bit.fFadeInOut)		// 16ms * (255 - 95) = 2560 ms = 2.5s
			{
				TIMER3->TCMPR = 0;		// TIMER3 OFF
				
				if(bFrameCnt == 508)
					gFlag.bit.fFadeMode = FALSE;
			}			

			else
			{
				bFrameCnt = 5;				
				gFlag.bit.fFadeInOut ^= 1;
			}
		}
		//printf("Fade_%s, Cnt = %3d,    tcmp = %3d \n", (fFadeIN)?"IN ":"OUT",bFrameCnt, TIMER3->TCMPR);
	}		

	// Col Output -> SPI
	DrvSPI_SingleWrite(eDRVSPI_PORT0, &wColLineBuf[bRowLine]);

	// Row Output
	DrvGPIO_SetPortBits(E_GPA, ~(0x01 << bRowLine));
	
	if(gFlag.bit.fDisplayMix && (gDisplay.eMode == Display_FIRE))		// Yellow color Display
	{				
		Latch_R = 1;
		
		DrvSPI_SingleWrite(eDRVSPI_PORT0, &wColLineBuf_Sub[bRowLine]);
		DrvGPIO_SetPortBits(E_GPA, ~(0x01 << bRowLine));
		
		Latch_G = 1;
	}
	
	// TESTPOINT 1, PWM Testing
	GPB_15 = 1;
	
	// column Output _ Latch ON & OutPut Enable
	switch(gDisplay.bColor)
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

	if(gFlag.bit.fFadeMode)
		DrvTIMER_Start(E_TMR3);
}

//------------------------------------------------------
//		Timer1 Callback Funtion : 10ms
//------------------------------------------------------
void TMR1_Callback(void)
{
	static BYTE bON_Time = 0;
	static BYTE bYellowCnt = 0;
	
	if(gFlag.bit.fStart)
	{
		if(gwTimerCnt < 500)		// 500 * 10ms = 5s
			gwTimerCnt++;
	}
	else
		gwTimerCnt = 0;
	
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
	switch(gDisplay.eMode)
	{
		case Display_ERROR :					// ERROR - 0.2s toggle
	
			if(b10msCnt > 20)
			{
				STATUS_LED_Toggle();
				b10msCnt = 0;
			}			
		break;
			
		case Display_FIRE :						// FIRE - 0.5s toggle			
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
		
		case Display_INSP :						// TEST - ALWAYS STS_LED_ON
			STATUS_LED_ON();
			b10msCnt = 0;			
		break;
		
		case Display_UP : case Dispaly_DOWN :
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
		
		case Display_DONT :					// AUTO - 0.4s ON / 0.6s OFF
			if(b10msCnt < 40)				//  10ms * 40 = 400ms
			{
				STATUS_LED_ON();
			}
			else if(b10msCnt < 100)			// 	10ms * (100 - 40) = 600ms
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
	
	if(gFlag.bit.fDisplayMix)
	{
		if(bYellowCnt <= 50)				// Yellow _ Red Blink - 500 ms
		{		
			gFlag.bit.fYellowBlink = TRUE;			
		}
		else if(bYellowCnt <= 100)
		{
			gFlag.bit.fYellowBlink = FALSE;
		}
		else
			bYellowCnt = 0;
	}
	else
		bYellowCnt = 0;
}

//------------------------------------------------------
//		Timer3 Callback Funtion : PWM
//------------------------------------------------------
void TMR3_Callback(void)
{	
	if(gFlag.bit.fFadeMode)
	{
		DrvGPIO_SetPortBits(E_GPA, 0xFFFF);
		// TESTPOINT 1, PWM testing
		GPB_15 = 0;
	}	
}

