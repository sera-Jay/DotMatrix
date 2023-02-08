/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2009 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------------------------------------*/
/* Include related headers                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
#include "NUC1xx.h"
#include "core_cm0.h"
#include "DrvSPI.h"
#include "DrvSYS.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global file scope (static) variables                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
typedef struct
{
    uint8_t bInUse;
    PFN_DRVSPI_CALLBACK pfnOneTransDoneCallBack; /* Function pointer of the one transaction done interrupt */
    uint32_t u32OneTransDoneUserData;
    PFN_DRVSPI_CALLBACK pfn3WireStartCallBack;   /* Function pointer of the 3-wire SPI start interrupt */
    uint32_t u32ThreeWireStartUserData;
} S_DRVSPI_HANDLE;

static S_DRVSPI_HANDLE g_sSpiHandler[4];

static SPI_T * SPI_PORT[4]={SPI0, SPI1, SPI2, SPI3};


/*---------------------------------------------------------------------------------------------------------*/
/* Function: DrvSPI_Open                                                                                   */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*         eSpiPort     [in]: Specify the SPI port                                                         */
/*         eMode        [in]: Specify the operation mode (eDRVSPI_MASTER/eDRVSPI_SLAVE)                    */
/*         eType        [in]: Specify the transfer type (eDRVSPI_TYPE0 ~ eDRVSPI_TYPE7)                    */
/*         i32BitLength [in]: Specify the bit length in a transaction (1~32)                               */
/*                                                                                                         */
/* Returns:                                                                                                */
/*         E_DRVSPI_ERR_INIT: The specified SPI port has been opened before.                               */
/*         E_DRVSPI_ERR_BUSY: The specified SPI port is in busy status.                                    */
/*         E_DRVSPI_ERR_BIT_LENGTH: The specified bit length is out of range.                              */
/*         E_SUCCESS: Success.                                                                             */
/*                                                                                                         */
/* Description:                                                                                            */
/*       Configure the operation mode, transfer type and bit length of a transaction of the specified SPI  */
/*       port.                                                                                             */
/*       The timing waveform types:                                                                        */
/*
DRVSPI_TYPE0:          

    CS    --|          Active state           |---
               _   _   _   _   _   _   _   _  
    CLK   ____| |_| |_| |_| |_| |_| |_| |_| |_____
              
    Tx    ----| 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |---
          
    Rx    --| 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |-----


DRVSPI_TYPE1:          

    CS    --|          Active state           |---
               _   _   _   _   _   _   _   _  
    CLK   ____| |_| |_| |_| |_| |_| |_| |_| |_____
              
    Tx    --| 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |-----
          
    Rx    --| 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |-----


DRVSPI_TYPE2:          

    CS    --|          Active state           |---
               _   _   _   _   _   _   _   _  
    CLK   ____| |_| |_| |_| |_| |_| |_| |_| |_____
              
    Tx    ----| 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |---
          
    Rx    ----| 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |---


DRVSPI_TYPE3:          

    CS    --|          Active state           |---
               _   _   _   _   _   _   _   _  
    CLK   ____| |_| |_| |_| |_| |_| |_| |_| |_____
              
    Tx    --| 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |-----
          
    Rx    ----| 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |---


DRVSPI_TYPE4:          

    CS    --|          Active state           |---
           ___   _   _   _   _   _   _   _   ______ 
    CLK       |_| |_| |_| |_| |_| |_| |_| |_|  
              
    Tx    --| 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |------
          
    Rx    ----| 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |----


DRVSPI_TYPE5:

    CS    --|          Active state           |---
           ___   _   _   _   _   _   _   _   ______ 
    CLK       |_| |_| |_| |_| |_| |_| |_| |_|  
              
    Tx    ----| 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |----
          
    Rx    ----| 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |----


DRVSPI_TYPE6:

    CS    --|          Active state           |---
           ___   _   _   _   _   _   _   _   ______ 
    CLK       |_| |_| |_| |_| |_| |_| |_| |_|  
              
    Tx    --| 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |------
          
    Rx    --| 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |------


DRVSPI_TYPE7:

    CS    --|          Active state           |---
           ___   _   _   _   _   _   _   _   ______ 
    CLK       |_| |_| |_| |_| |_| |_| |_| |_|  
              
    Tx    ----| 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |----
          
    Rx    --| 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |----


Master / Slave Transfer Type Matching Table

    DRVSPI_TYPE0 <==> DRVSPI_TYPE3
    DRVSPI_TYPE1 <==> DRVSPI_TYPE1
    DRVSPI_TYPE2 <==> DRVSPI_TYPE2
    DRVSPI_TYPE3 <==> DRVSPI_TYPE0
    DRVSPI_TYPE4 <==> DRVSPI_TYPE7
    DRVSPI_TYPE5 <==> DRVSPI_TYPE5
    DRVSPI_TYPE6 <==> DRVSPI_TYPE6
    DRVSPI_TYPE7 <==> DRVSPI_TYPE4
*/
/*---------------------------------------------------------------------------------------------------------*/
int32_t DrvSPI_Open(E_DRVSPI_PORT eSpiPort, E_DRVSPI_MODE eMode, E_DRVSPI_TRANS_TYPE eType, int32_t i32BitLength)
{
    int32_t i32TimeOut;
    
    if(g_sSpiHandler[eSpiPort].bInUse)
    {
        return E_DRVSPI_ERR_INIT;
    }
    
    /* Bit length 1 ~ 32 */
    if((i32BitLength <= 0) || (i32BitLength > 32))
    {
        return E_DRVSPI_ERR_BIT_LENGTH;
    }
   
    if(eSpiPort == eDRVSPI_PORT0)
    {
        SYSCLK->APBCLK.SPI0_EN  =1;
        SYS->IPRSTC2.SPI0_RST   =1;
        SYS->IPRSTC2.SPI0_RST   =0;
    }
    else if(eSpiPort == eDRVSPI_PORT1)
    {
        SYSCLK->APBCLK.SPI1_EN  =1;
        SYS->IPRSTC2.SPI1_RST   =1;
        SYS->IPRSTC2.SPI1_RST   =0;
    }
    else if(eSpiPort == eDRVSPI_PORT2)
    {
        SYSCLK->APBCLK.SPI2_EN  =1;
        SYS->IPRSTC2.SPI2_RST   =1;
        SYS->IPRSTC2.SPI2_RST   =0;
    }
    else
    {
        SYSCLK->APBCLK.SPI3_EN  =1;
        SYS->IPRSTC2.SPI3_RST   =1;
        SYS->IPRSTC2.SPI3_RST   =0;
    }
    
    /* Check busy*/
    i32TimeOut = 0x10000;
    while(SPI_PORT[eSpiPort]->CNTRL.GO_BUSY == 1)
    {
        if(i32TimeOut-- <= 0)
            return E_DRVSPI_ERR_BUSY;
    }
    
    g_sSpiHandler[eSpiPort].bInUse = TRUE;
    g_sSpiHandler[eSpiPort].pfnOneTransDoneCallBack = NULL;
    g_sSpiHandler[eSpiPort].u32OneTransDoneUserData = 0;
    g_sSpiHandler[eSpiPort].pfn3WireStartCallBack = NULL;
    g_sSpiHandler[eSpiPort].u32ThreeWireStartUserData = 0;
    
    /* "i32BitLength = 0" means 32 bits */
    if(i32BitLength == 32)
    {
        i32BitLength = 0;
    }
    SPI_PORT[eSpiPort]->CNTRL.TX_BIT_LEN = i32BitLength;
    
    if(eMode == eDRVSPI_SLAVE)
        SPI_PORT[eSpiPort]->CNTRL.SLAVE = 1;
    else
        SPI_PORT[eSpiPort]->CNTRL.SLAVE = 0;
    
    /* Automatic slave select */
    SPI_PORT[eSpiPort]->SSR.AUTOSS = 1;
        
    /* Timing waveform types */
    if(eType==eDRVSPI_TYPE0)
    {
        SPI_PORT[eSpiPort]->CNTRL.CLKP = 0;
        /* Drive data and latch data at the same edge. Not recommend to use this transfer type. */
        SPI_PORT[eSpiPort]->CNTRL.TX_NEG = 0;
        SPI_PORT[eSpiPort]->CNTRL.RX_NEG = 0;
    }
    else if(eType==eDRVSPI_TYPE1)
    {
        SPI_PORT[eSpiPort]->CNTRL.CLKP = 0;
        /* Drive data at falling-edge of serial clock; latch data at rising-edge of serial clock. */
        SPI_PORT[eSpiPort]->CNTRL.TX_NEG = 1;
        SPI_PORT[eSpiPort]->CNTRL.RX_NEG = 0;
    }
    else if(eType==eDRVSPI_TYPE2)
    {
        SPI_PORT[eSpiPort]->CNTRL.CLKP = 0;
        /* Drive data at rising-edge of serial clock; latch data at falling-edge of serial clock. */
        SPI_PORT[eSpiPort]->CNTRL.TX_NEG = 0;
        SPI_PORT[eSpiPort]->CNTRL.RX_NEG = 1;
    }
    else if(eType==eDRVSPI_TYPE3)
    {
        SPI_PORT[eSpiPort]->CNTRL.CLKP = 0;
        /* Drive data and latch data at the same edge. Not recommend to use this transfer type. */
        SPI_PORT[eSpiPort]->CNTRL.TX_NEG = 1;
        SPI_PORT[eSpiPort]->CNTRL.RX_NEG = 1;
    }
    else if(eType==eDRVSPI_TYPE4)
    {
        SPI_PORT[eSpiPort]->CNTRL.CLKP = 1;
        /* Drive data and latch data at the same edge. Not recommend to use this transfer type. */
        SPI_PORT[eSpiPort]->CNTRL.TX_NEG = 0;
        SPI_PORT[eSpiPort]->CNTRL.RX_NEG = 0;
    }
    else if(eType==eDRVSPI_TYPE5)
    {
        SPI_PORT[eSpiPort]->CNTRL.CLKP = 1;
        /* Drive data at falling-edge of serial clock; latch data at rising-edge of serial clock. */
        SPI_PORT[eSpiPort]->CNTRL.TX_NEG = 1;
        SPI_PORT[eSpiPort]->CNTRL.RX_NEG = 0;
    }
    else if(eType==eDRVSPI_TYPE6)
    {
        SPI_PORT[eSpiPort]->CNTRL.CLKP = 1;
        /* Drive data at rising-edge of serial clock; latch data at falling-edge of serial clock. */
        SPI_PORT[eSpiPort]->CNTRL.TX_NEG = 0;
        SPI_PORT[eSpiPort]->CNTRL.RX_NEG = 1;
    }
    else
    {
        SPI_PORT[eSpiPort]->CNTRL.CLKP = 1;
        /* Drive data and latch data at the same edge. Not recommend to use this transfer type. */
        SPI_PORT[eSpiPort]->CNTRL.TX_NEG = 1;
        SPI_PORT[eSpiPort]->CNTRL.RX_NEG = 1;
    }

    return E_SUCCESS;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: DrvSPI_Close                                                                                  */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*    eSpiPort [in]:  Specify the SPI port.                                                                */
/*                                                                                                         */
/* Returns:                                                                                                */
/*    None.                                                                                                */
/*                                                                                                         */
/* Description:                                                                                            */
/*    Close the specified SPI module and disable the SPI interrupt.                                        */
/*---------------------------------------------------------------------------------------------------------*/
void DrvSPI_Close(E_DRVSPI_PORT eSpiPort)
{
    int32_t i32TimeOut;
    
    g_sSpiHandler[eSpiPort].bInUse = FALSE;
    g_sSpiHandler[eSpiPort].pfnOneTransDoneCallBack = NULL;
    g_sSpiHandler[eSpiPort].u32OneTransDoneUserData = 0;
    g_sSpiHandler[eSpiPort].pfn3WireStartCallBack = NULL;
    g_sSpiHandler[eSpiPort].u32ThreeWireStartUserData = 0;

    /* Check SPI state */
    i32TimeOut = 0x10000;
    while(SPI_PORT[eSpiPort]->CNTRL.GO_BUSY == 1)
    {
        if(i32TimeOut-- <= 0)
            break;
    }
   
   if(eSpiPort == eDRVSPI_PORT0)
    {
        NVIC_DisableIRQ(SPI0_IRQn);
        SYS->IPRSTC2.SPI0_RST=1;
        SYS->IPRSTC2.SPI0_RST=0;
        SYSCLK->APBCLK.SPI0_EN=0;
    }
    else
    {
        NVIC_DisableIRQ(SPI1_IRQn);
        SYS->IPRSTC2.SPI1_RST=1;
        SYS->IPRSTC2.SPI1_RST=0;
        SYSCLK->APBCLK.SPI1_EN=0;
    }    
   
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: DrvSPI_SetEndian                                                                              */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*    eSpiPort [in]: Specify the SPI port                                                                  */
/*    eEndian  [in]: Specify LSB first or MSB first (eDRVSPI_LSB_FIRST / eDRVSPI_MSB_FIRST)                */
/*                                                                                                         */
/* Returns:                                                                                                */
/*    None.                                                                                                */
/*                                                                                                         */
/* Description:                                                                                            */
/*    Dertermine to transfer data with LSB first or MSB first                                              */
/*---------------------------------------------------------------------------------------------------------*/
void DrvSPI_SetEndian(E_DRVSPI_PORT eSpiPort, E_DRVSPI_ENDIAN eEndian)
{

    if(eEndian == eDRVSPI_LSB_FIRST)
        SPI_PORT[eSpiPort]->CNTRL.LSB = 1;
    else
        SPI_PORT[eSpiPort]->CNTRL.LSB = 0;

}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: DrvSPI_SetBitLength                                                                           */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*    eSpiPort     [in]: Specify the SPI port                                                              */
/*    i32BitLength [in]: Specify the bit length (1~32 bits)                                                */
/*                                                                                                         */
/* Returns:                                                                                                */
/*    E_SUCCESS:                 Success.                                                                  */
/*    E_DRVSPI_ERR_BIT_LENGTH: The bit length is out of range.                                             */
/*                                                                                                         */
/* Description:                                                                                            */
/*    Set the bit length of SPI transfer.                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
int32_t DrvSPI_SetBitLength(E_DRVSPI_PORT eSpiPort, int32_t i32BitLength)
{

    if((i32BitLength < 1) || (i32BitLength > 32))
    {
        return E_DRVSPI_ERR_BIT_LENGTH;
    }
    if(i32BitLength == 32)
        i32BitLength = 0;

    SPI_PORT[eSpiPort]->CNTRL.TX_BIT_LEN = i32BitLength;
    
    return E_SUCCESS;
}


/*---------------------------------------------------------------------------------------------------------*/
/* Function: DrvSPI_BurstTransfer                                                                          */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*    eSpiPort    [in]: Specify the SPI port                                                               */
/*    i32BurstCnt [in]: Specify the transaction number in one transfer. It could be 1 or 2.                */
/*    i32Interval [in]: Specify the delay clocks between successive transactions. It could be 2~17.        */
/*                                                                                                         */
/* Returns:                                                                                                */
/*    E_DRVSPI_ERR_BURST_CNT: The transaction number is out of range.                                      */
/*    E_DRVSPI_ERR_SUSPEND_INTERVAL: The suspend interval setting is out of range.                         */
/*    E_SUCCESS: Success.                                                                                  */
/*                                                                                                         */
/* Description:                                                                                            */
/*    Configure the burst transfer settings.                                                               */
/*---------------------------------------------------------------------------------------------------------*/
int32_t DrvSPI_BurstTransfer(E_DRVSPI_PORT eSpiPort, int32_t i32BurstCnt, int32_t i32Interval)
{

    if((i32BurstCnt < 1) || (i32BurstCnt > 2))
    {
        return E_DRVSPI_ERR_BURST_CNT;
    }
    
    if((i32Interval < 2) || (i32Interval > 17))
    {
        return E_DRVSPI_ERR_SUSPEND_INTERVAL;
    }

    SPI_PORT[eSpiPort]->CNTRL.TX_NUM = i32BurstCnt-1;
    SPI_PORT[eSpiPort]->CNTRL.SP_CYCLE = i32Interval-2;

    return E_SUCCESS;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: DrvSPI_SetClockFreq                                                                           */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*    eSpiPort  [in]: Specify the SPI port                                                                 */
/*    u32Clock1 [in]: Specify the SPI clock rate in Hz. It's the target clock rate of SPI base clock and   */
/*                    variable clock 1.                                                                    */
/*    u32Clock2 [in]: Specify the SPI clock rate in Hz. It's the target clock rate of variable clock 2.    */
/*                                                                                                         */
/* Returns:                                                                                                */
/*    The actual clock rate of SPI engine clock is returned.                                               */
/*    SPI engine clock rate = APB clock rate / ((DIVIDER + 1) * 2)                                         */
/*    The actual clock rate may be different from the target SPI clock rate.                               */
/*    For example, if the system clock rate is 12MHz and the target SPI base clock rate is 7MHz, the       */
/*    actual SPI clock rate will be 6MHz.                                                                  */
/*                                                                                                         */
/* Description:                                                                                            */
/*    Configure the SPI clock. Only for master mode.                                                       */
/*    If the DIV_ONE bit is set to 1, executing this function is unmeaningful.                             */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t DrvSPI_SetClockFreq(E_DRVSPI_PORT eSpiPort, uint32_t u32Clock1, uint32_t u32Clock2)
{
    uint32_t u32Div;
    uint32_t u32Pclk;
    
    u32Pclk = DrvSYS_GetHCLKFreq();
    
    u32Div = 0xFFFF; /* Initial value */
    
    if(u32Clock2!=0)
    {
        if(u32Clock2>u32Pclk)
            u32Div = 0;
        else
        {
            u32Div = (((u32Pclk / u32Clock2) + 1) >> 1) - 1;
            if(u32Div > 65535)
                u32Div = 65535;
        }
        SPI_PORT[eSpiPort]->DIVIDER.DIVIDER2 = u32Div;
    }
    else
        SPI_PORT[eSpiPort]->DIVIDER.DIVIDER2 = 0xFFFF;
    
    if(u32Clock1!=0)
    {
        if(u32Clock1>u32Pclk)
            u32Div = 0;
        else
        {
            u32Div = (((u32Pclk / u32Clock1) + 1) >> 1) - 1;
            if(u32Div > 0xFFFF)
                u32Div = 0xFFFF;
        }
        SPI_PORT[eSpiPort]->DIVIDER.DIVIDER = u32Div;
    }
    else
        SPI_PORT[eSpiPort]->DIVIDER.DIVIDER = 0xFFFF;

    return ( u32Pclk / ((u32Div+1)*2) );
}


/*---------------------------------------------------------------------------------------------------------*/
/* Function: DrvSPI_SetVariableClockFunction                                                               */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*    eSpiPort   [in]: Specify the SPI port                                                                */
/*    bEnable    [in]: TRUE  -- Enable variable clock function                                             */
/*                     FALSE -- Disable variable clock function                                            */
/*    u32Pattern [in]: Specify the variable clock pattern                                                  */
/*                                                                                                         */
/* Returns:                                                                                                */
/*    None.                                                                                                */
/*                                                                                                         */
/* Description:                                                                                            */
/*    Set the variable clock function. Only for master mode.                                               */
/*    If the bit pattern of VARCLK is '0', the output frequency of SPICLK is according to the value of     */
/*    DIVIDER.                                                                                             */
/*    If the bit pattern of VARCLK is '1', the output frequency of SPICLK is according to the value of     */
/*    DIVIDER2.                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void DrvSPI_SetVariableClockFunction(E_DRVSPI_PORT eSpiPort, uint8_t bEnable, uint32_t u32Pattern)
{
    if(bEnable)
    {
        SPI_PORT[eSpiPort]->CNTRL.VARCLK_EN = 1;
        SPI_PORT[eSpiPort]->VARCLK = u32Pattern;
    }
    else
        SPI_PORT[eSpiPort]->CNTRL.VARCLK_EN = 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: DrvSPI_SingleRead                                                                             */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*    eSpiPort   [in]:  Specify the SPI port                                                               */
/*    pu32Data   [out]: Store the data got from the SPI bus.                                               */
/*                                                                                                         */
/* Returns:                                                                                                */
/*    TRUE:  The data stored in pu32Data is valid.                                                         */
/*    FALSE: The data stored in pu32Data is invalid.                                                       */
/*                                                                                                         */
/* Description:                                                                                            */
/*    Read data from SPI Rx registers and trigger SPI for next transfer.                                   */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t DrvSPI_SingleRead(E_DRVSPI_PORT eSpiPort, uint32_t *pu32Data)
{
    if(SPI_PORT[eSpiPort]->CNTRL.GO_BUSY==1)
        return FALSE;

    *pu32Data = SPI_PORT[eSpiPort]->RX[0];
    SPI_PORT[eSpiPort]->CNTRL.GO_BUSY = 1;
    return TRUE;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: DrvSPI_SingleWrite                                                                            */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*    eSpiPort  [in]:  Specify the SPI port                                                                */
/*    pu32Data  [in]:  Store the data which will be transmitted through the SPI bus.                       */
/*                                                                                                         */
/* Returns:                                                                                                */
/*    TRUE:  The data stored in pu32Data has been transferred.                                             */
/*    FALSE: The SPI is in busy. The data stored in pu32Data has not been transferred.                     */
/*                                                                                                         */
/* Description:                                                                                            */
/*    Write data to SPI bus and trigger SPI to start transfer.                                             */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t DrvSPI_SingleWrite(E_DRVSPI_PORT eSpiPort, uint32_t *pu32Data)
{
    if(SPI_PORT[eSpiPort]->CNTRL.GO_BUSY==1)
        return FALSE;

    SPI_PORT[eSpiPort]->TX[0] = *pu32Data;
    SPI_PORT[eSpiPort]->CNTRL.GO_BUSY = 1;
    return TRUE;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: DrvSPI_BurstRead                                                                              */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*    eSpiPort  [in]:  Specify the SPI port                                                                */
/*    pu32Buf   [out]: Store the data got from the SPI bus.                                                */
/*                                                                                                         */
/* Returns:                                                                                                */
/*    TRUE: The data stored in pu32Buf is valid.                                                           */
/*    FALSE: The data stored in pu32Buf is invalid.                                                        */
/*                                                                                                         */
/* Description:                                                                                            */
/*    Read two words of data from SPI Rx registers and then trigger SPI for next transfer.                 */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t DrvSPI_BurstRead(E_DRVSPI_PORT eSpiPort, uint32_t *pu32Buf)
{
    if(SPI_PORT[eSpiPort]->CNTRL.GO_BUSY==1)
        return FALSE;

    pu32Buf[0] = SPI_PORT[eSpiPort]->RX[0];
    pu32Buf[1] = SPI_PORT[eSpiPort]->RX[1];
    SPI_PORT[eSpiPort]->CNTRL.GO_BUSY = 1;

    return TRUE;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: DrvSPI_BurstWrite                                                                             */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*    eSpiPort  [in]:  Specify the SPI port                                                                */
/*    pu32Buf   [in]:  Store the data which will be transmitted through the SPI bus.                       */
/*                                                                                                         */
/* Returns:                                                                                                */
/*    TRUE: The data stored in pu32Buf has been transferred.                                               */
/*    FALSE: The SPI is in busy. The data stored in pu32Buf has not been transferred.                      */
/*                                                                                                         */
/* Description:                                                                                            */
/*    Write two words of data to SPI bus and then trigger SPI to start transfer.                           */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t DrvSPI_BurstWrite(E_DRVSPI_PORT eSpiPort, uint32_t *pu32Buf)
{
    if(SPI_PORT[eSpiPort]->CNTRL.GO_BUSY==1)
        return FALSE;

    SPI_PORT[eSpiPort]->TX[0] = pu32Buf[0];
    SPI_PORT[eSpiPort]->TX[1] = pu32Buf[1];
    SPI_PORT[eSpiPort]->CNTRL.GO_BUSY = 1;

    return TRUE;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: DrvSPI_SetTxRegister                                                                          */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*    eSpiPort     [in]:  Specify the SPI port                                                             */
/*    pu32Buf      [in]:  Store the data which will be written to Tx registers.                            */
/*    u32DataCount [in]:  The count of data write to Tx registers.                                         */
/*                                                                                                         */
/* Returns:                                                                                                */
/*    The count of data actually written to Tx registers.                                                  */
/*                                                                                                         */
/* Description:                                                                                            */
/*    Write data to Tx registers. This function will not trigger a SPI data transfer.                      */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t DrvSPI_SetTxRegister(E_DRVSPI_PORT eSpiPort, uint32_t *pu32Buf, uint32_t u32DataCount)
{
    uint32_t i;

    if(u32DataCount>2)
        u32DataCount = 2;
    
    for(i=0; i<u32DataCount; i++)
    {
        SPI_PORT[eSpiPort]->TX[i] = pu32Buf[i];
    }

    return u32DataCount;
}
