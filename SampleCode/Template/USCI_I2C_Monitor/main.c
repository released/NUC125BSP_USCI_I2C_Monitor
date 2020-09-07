/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 16/10/25 4:33p $
 * @brief    Show a Master how to access Slave.
 *           This sample code needs to work with USCI_I2C_Slave.
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>

//#define PLLCTL_SETTING  			CLK_PLLCTL_72MHz_HXT
//#define ENABLE_NUC126
#define ENABLE_NUC125

#if defined (ENABLE_NUC125)
#define PLL_CLOCK       			FREQ_50MHZ
#include "NuMicro.h"

#elif defined (ENABLE_NUC126)
#define PLL_CLOCK       			FREQ_72MHZ
#include "NUC126.h"

#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define I2C_DATA_MAX  			24

#define MONITOR_TEST_CNT  		3

volatile uint8_t g_u8RxDataTmp;
volatile uint32_t g_u32ProtOn;

#if defined (ENABLE_NUC125)
volatile uint8_t g_au8MstTxData[I2C_DATA_MAX];
volatile uint8_t g_au8SlvRxData[I2C_DATA_MAX];
volatile uint8_t    g_u8MonRxData[(I2C_DATA_MAX + 1) * 2] = {0};

#elif defined (ENABLE_NUC126)
volatile uint8_t g_au8MstTxData[3];
volatile uint8_t g_au8SlvRxData[3];
volatile uint8_t    g_u8MonRxData[(MONITOR_TEST_CNT + 2) * 9] = {0};

#endif

volatile uint32_t slave_buff_addr;
volatile uint8_t g_au8SlvData[256];

volatile uint8_t g_u8DeviceAddr;

volatile uint8_t g_u8MstRxData;
volatile uint8_t g_u8MstDataLen;
volatile uint8_t g_u8SlvDataLen;
volatile uint8_t g_u8MstEndFlag = 0;

volatile uint8_t g_u8MonDataCnt = 0;

volatile uint8_t g_u8test_data = 0x10 ;	// 5


uint8_t g_u8DataLen_s = 0;
uint8_t g_u8ToMasterLen = 0;
uint8_t g_u8ToMasterData[I2C_DATA_MAX] ={0};
uint8_t g_u8FromMasterLen = 0;
uint8_t g_u8FromMasterData[I2C_DATA_MAX] ={0};

uint8_t slave_register_addr = 0;
uint8_t g_u8temporary = 0;


typedef void (*I2C_FUNC)(uint32_t u32Status);

volatile static I2C_FUNC s_I2C0HandlerFn = NULL;
volatile static I2C_FUNC s_I2C1HandlerFn = NULL;

volatile enum UI2C_SLAVE_EVENT s_Event;

typedef void (*UI2C_FUNC)(uint32_t u32Status);

volatile static UI2C_FUNC s_UI2C0HandlerFn = NULL;

enum
{
	_state_DEFAULT_ = 0 , 
	_state_RECEIVE_ADDRESS_ = 1 ,
	_state_CHECK_RX_OR_TX_ = 2 ,
	_state_RECEIVE_RX_ = 3 ,	
	_state_TRANSMIT_TX_ = 4 ,	
};
	
#define SLAVE_TRANSMIT_REPEAT_START_OR_STOP	(0xA0)
#define SLAVE_TRANSMIT_ADDRESS_ACK 			(0xA8)
#define SLAVE_TRANSMIT_DATA_ACK				(0xB8)
#define SLAVE_TRANSMIT_DATA_NACK             	(0xC0)
#define SLAVE_TRANSMIT_LAST_DATA_ACK         	(0xC8)
#define SLAVE_RECEIVE_ADDRESS_ACK            	(0x60)
#define SLAVE_RECEIVE_ARBITRATION_LOST       	(0x68)
#define SLAVE_RECEIVE_DATA_ACK               	(0x80)
#define SLAVE_RECEIVE_DATA_NACK              	(0x88)
#define GC_MODE_ADDRESS_ACK                	(0x70)
#define GC_MODE_ARBITRATION_LOST             	(0x78)
#define GC_MODE_DATA_ACK                   	(0x90)
#define GC_MODE_DATA_NACK                  	(0x98)
#define ADDRESS_TRANSMIT_ARBITRATION_LOST    	(0xB0)
#define BUS_ERROR                         		(0x00)
/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C0);

    if(I2C_GET_TIMEOUT_FLAG(I2C0))
    {
        /* Clear I2C0 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C0);
    }
    else
    {
        if(s_I2C0HandlerFn != NULL)
            s_I2C0HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C1 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C1_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C1);

    if(I2C_GET_TIMEOUT_FLAG(I2C1))
    {
        /* Clear I2C1 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C1);
    }
    else
    {
        if(s_I2C1HandlerFn != NULL)
            s_I2C1HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C IRQ Handler                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_IRQHandler(void)
{
    uint32_t u32Status;

    //UI2C0 Interrupt
    u32Status = UI2C_GET_PROT_STATUS(UI2C0);

    if (s_UI2C0HandlerFn != NULL)
    {
        s_UI2C0HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 Master Rx Callback Function                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterRx(uint32_t u32Status)
{
    if(u32Status == 0x08)                       /* START has been transmitted and prepare SLA+W */
    {
        I2C_SET_DATA(I2C0, (g_u8DeviceAddr << 1));    /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(I2C0, g_au8MstTxData[g_u8MstDataLen++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
        if (g_u8MstDataLen != (I2C_DATA_MAX - 2))	//(g_u8MstDataLen != 2)
        {
            I2C_SET_DATA(I2C0, g_au8MstTxData[g_u8MstDataLen++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA_SI);
        }
    }
    else if(u32Status == 0x10)                  /* Repeat START has been transmitted and prepare SLA+R */
    {
        I2C_SET_DATA(I2C0, ((g_u8DeviceAddr << 1) | 0x01));   /* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x40)                  /* SLA+R has been transmitted and ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x58)                  /* DATA has been received and NACK has been returned */
    {
//        g_u8MstRxData = (unsigned char) I2C_GET_DATA(I2C0);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
        g_u8MstEndFlag = 1;
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 Master Tx Callback Function                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterTx(uint32_t u32Status)
{
    if(u32Status == 0x08)                       /* START has been transmitted */
    {
        I2C_SET_DATA(I2C0, g_u8DeviceAddr << 1);    /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(I2C0, g_au8MstTxData[g_u8MstDataLen++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
        if (g_u8MstDataLen != I2C_DATA_MAX)	//(g_u8MstDataLen != 3)
        {
            I2C_SET_DATA(I2C0, g_au8MstTxData[g_u8MstDataLen++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            g_u8MstEndFlag = 1;
        }
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C1 Slave TRx Callback Function                                                                       */
/*---------------------------------------------------------------------------------------------------------*/

void I2C_Slave_ReturnTx(void)
{
	uint8_t i = 0;	

	// clear tx buffer
	for (i = 0; i <64 ; i++)
	{
		g_u8ToMasterData[i] = 0x00;
	}
	// swap the data 
	for (i = 0; i < g_u8temporary; i++)
	{
		g_u8ToMasterData[(g_u8temporary-1)-i] = g_u8FromMasterData[i];
//		printf("From : 0x%2X, To : 0x%2X,\r\n" , g_u8FromMasterData[i],g_u8ToMasterData[i]);
	}

	g_u8FromMasterLen = 0;
	g_u8ToMasterLen = 0;
	g_u8temporary = 0;
}


void I2C_Slave_StateMachine(uint32_t res , uint8_t* InData, uint8_t* OutData)
{
//	uint8_t i = 0;	
//	static uint8_t u8Temp = 0;
	static uint8_t cnt = _state_DEFAULT_;
	
	if (res == SLAVE_RECEIVE_ADDRESS_ACK)	//no this ack when not receive register address
	{
		cnt = _state_RECEIVE_ADDRESS_;
		slave_register_addr = 0;
		g_u8FromMasterLen = 0;
		g_u8ToMasterLen = 0;		
	}

	switch(cnt)
	{
		case _state_RECEIVE_ADDRESS_:
			if (res == SLAVE_RECEIVE_DATA_ACK)	
			{
				// first 0x80 ack is register address
				if(g_u8DataLen_s == 1)
				{
					slave_register_addr = slave_buff_addr;
					printf("-----------------\r\n");
					printf("DATA = 0x%2X\r\n" , slave_register_addr);
				}
				cnt = _state_CHECK_RX_OR_TX_;
			}
			break;

		case _state_CHECK_RX_OR_TX_:
			if (res == SLAVE_RECEIVE_DATA_ACK)	
			{
				// RX : contiune with multi 0x80 , end with 0xA0
				// example : 0x60 > 0x80 (address) > 0x80 (data00)> 0x80 (data01) > ...> 0xA0
				g_u8FromMasterData[g_u8FromMasterLen++] =  *InData;
				cnt = _state_RECEIVE_RX_;
			}
			else if (res == SLAVE_TRANSMIT_REPEAT_START_OR_STOP)	
			{
				// if use I2C_ReadMultiBytesOneReg , 
				// ack will act as below , 0x60 > 0x80 > 0xA0 > 0xA8 > 0xB8
				// TX : following with 0xA0 , and 0xA8 (data01) , continue with 0xB8 (Data02)  , end with 0xC0
				// example : 0x60 > 0x80 (address) > 0xA8 (data00) > 0xB8  (data01)> 0xB8  (data02) > ... > 0xC0
				
				cnt = _state_TRANSMIT_TX_;
				
			}
			break;

		case _state_RECEIVE_RX_:
			if (res == SLAVE_TRANSMIT_REPEAT_START_OR_STOP)	
			{
				// end of RX

				printf("g_u8FromMasterLen = %d\r\n" , g_u8FromMasterLen);
				g_u8temporary = g_u8FromMasterLen;

				// if use I2C_ReadMultiBytes , 
				// ack will act as below , 0xA8 > 0xB8 > 0xB8 > ... > 0xC0
				// so change state machine here
				// must follow with 
				// 1) I2C_WriteMultiBytes , I2C_ReadMultiBytes , or
				// 2) I2C_WriteMultiBytesOneReg , , I2C_ReadMultiBytes
				// 3) directlly by using I2C_ReadMultiBytes , will be no data transfer
				
//				cnt = _state_DEFAULT_;	//reset flag
				cnt = _state_TRANSMIT_TX_;
				
			}
			else if (res == SLAVE_RECEIVE_DATA_ACK)
			{
				// continue to get data
				g_u8FromMasterData[g_u8FromMasterLen++] =  *InData;
			}
			break;

		case _state_TRANSMIT_TX_:
			if (res == SLAVE_TRANSMIT_ADDRESS_ACK)	
			{
				I2C_Slave_ReturnTx();
				
				// first TX byte				
				*OutData = g_u8ToMasterData[g_u8ToMasterLen++];
			}
			else if (res == SLAVE_TRANSMIT_DATA_NACK)	
			{
				// end of TX
				printf("g_u8ToMasterLen = %d\r\n" , g_u8ToMasterLen);
				printf("-----------------\r\n\r\n");
				cnt = _state_DEFAULT_;	//reset flag
			}
			else if (res == SLAVE_TRANSMIT_DATA_ACK)
			{
				// continue to send data
				*OutData = g_u8ToMasterData[g_u8ToMasterLen++];
			}
			break;

			
	}
}

void I2C_SlaveTRx(uint32_t u32Status)
{
#if 1
	uint8_t u8RxData = 0;
	uint8_t u8TxData = 0;
	uint8_t u8TempData = 0;	
	uint16_t u16Rxlen = 0;
	
    if(u32Status == SLAVE_RECEIVE_ADDRESS_ACK) //0x60                    	/* Own SLA+W has been receive; ACK has been return */
    {
        g_u8DataLen_s = 0;

		I2C_Slave_StateMachine(u32Status , &u8TempData , &u8TempData);
		
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI | I2C_CTL_AA);

    }
    else if(u32Status == SLAVE_RECEIVE_DATA_ACK) //0x80                 		/* Previously address with own SLA address
																			/* Data has been received; ACK has been returned*/
    {
        u8RxData = (unsigned char) I2C_GET_DATA(I2C1);
		
//		__I2Cx_Slave_LogBuffer__(u32Status , g_u8RxData);		

	    g_u8DataLen_s++;

	    if(g_u8DataLen_s == 1)
	    {
			/*
				Blind spot : register address can not larger than g_u8SlvData array size , 
				due to resister address will be save as array start index to store data
			
			*/
	        slave_buff_addr = u8RxData;	//register
					
	    }
	    else
	    {
			u16Rxlen = slave_buff_addr+(g_u8DataLen_s-2);		//data buffer start	
	        g_au8SlvData[u16Rxlen] = u8RxData;
		
	    }
	
		I2C_Slave_StateMachine(u32Status , &u8RxData , &u8TempData);
				
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI | I2C_CTL_AA);

    }
    else if(u32Status == SLAVE_TRANSMIT_ADDRESS_ACK) //0xA8               	/* Own SLA+R has been receive; ACK has been return */
    {
		u8RxData = g_au8SlvData[slave_buff_addr++];

		I2C_Slave_StateMachine(u32Status , &u8RxData, &u8TxData);	

//		I2C_SET_DATA(I2C1, u8RxData);
     	I2C_SET_DATA(I2C1, u8TxData);	
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI | I2C_CTL_AA);

    }
	else if(u32Status == SLAVE_TRANSMIT_DATA_ACK) //0xB8                 	/* Data byte in I2CDAT has been transmitted
																			/* ACK has been received */
    {
		u8RxData = g_au8SlvData[slave_buff_addr++];

		I2C_Slave_StateMachine(u32Status , &u8RxData, &u8TxData);

//		I2C_SET_DATA(I2C1, u8RxData);
		I2C_SET_DATA(I2C1, u8TxData);	
		I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI | I2C_CTL_AA);
		
    }
    else if(u32Status == SLAVE_TRANSMIT_DATA_NACK) //0xC0                 	/* Data byte or last data in I2CDAT has been transmitted
																			/* Not ACK has been received */
    {
		I2C_Slave_StateMachine(u32Status , &u8TempData , &u8TempData);
	
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI | I2C_CTL_AA);
		

    }
    else if(u32Status == SLAVE_RECEIVE_DATA_NACK) //0x88                 		/* Previously addressed with own SLA address; NOT ACK has
																			/* been returned */
    {
        g_u8DataLen_s = 0;
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI | I2C_CTL_AA);

    }
    else if(u32Status == SLAVE_TRANSMIT_REPEAT_START_OR_STOP) //0xA0    		/* A STOP or repeated START has been received while still
																			/* addressed as Slave/Receiver*/
    {
        g_u8DataLen_s = 0;
		I2C_Slave_StateMachine(u32Status , &u8TempData, &u8TempData);
		
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI | I2C_CTL_AA);

    }
    else if(u32Status == BUS_ERROR) //0x00
    {
		I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO_SI_AA);
		I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
	
    }																	
    else
    {
		#if defined (DEBUG_LOG_SLAVE_LV2)
        /* TO DO */
        printf("I2Cx_SlaveTRx Status 0x%x is NOT processed\n", u32Status);
		#endif
    }
#else

    if(u32Status == 0x60)                       /* Own SLA+W has been receive; ACK has been return */
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0x80)                 /* Previously address with own SLA address
                                                   Data has been received; ACK has been returned*/
    {
        g_au8SlvRxData[g_u8SlvDataLen] = (unsigned char) I2C_GET_DATA(I2C1);
        g_u8SlvDataLen++;

        if(g_u8SlvDataLen == 2)
        {
            slave_buff_addr = (g_au8SlvRxData[0] << 8) + g_au8SlvRxData[1];
        }
        if(g_u8SlvDataLen == 3)
        {
            g_au8SlvData[slave_buff_addr] = g_au8SlvRxData[2];
            g_u8SlvDataLen = 0;
        }

        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xA8)                  /* Own SLA+R has been receive; ACK has been return */
    {
        I2C_SET_DATA(I2C1, g_au8SlvData[slave_buff_addr]);
        slave_buff_addr++;
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xC0)                 /* Data byte or last data in I2CDAT has been transmitted
                                                   Not ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
#endif	
}
void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HXT, CLK_CLKDIV0_HCLK(1));

    /* Enable external XTAL 12MHz clock */
	#if defined (ENABLE_NUC125)
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN);
	#elif defined (ENABLE_NUC126)
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
	#endif

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable IP clock */
    CLK_EnableModuleClock(USCI0_MODULE);

    /* Enable I2C0 clock */
    CLK_EnableModuleClock(I2C0_MODULE);

    /* Enable I2C1 clock */
    CLK_EnableModuleClock(I2C1_MODULE);

    /* Select UART module clock source */
	#if defined (ENABLE_NUC125)
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC_DIV2, CLK_CLKDIV0_UART(1));
	#elif defined (ENABLE_NUC126)	
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC, CLK_CLKDIV0_UART(1));
	#endif

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

	#if defined (ENABLE_NUC125)
    /* Set PB multi-function pins for UART0 RXD(PB.0) and TXD(PB.1) */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk)) | SYS_GPB_MFPL_PB0MFP_UART0_RXD;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk)) | SYS_GPB_MFPL_PB1MFP_UART0_TXD;

    /* Set PC multi-function pins for UI2C SDA and SCL */
    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC3MFP_Msk | SYS_GPC_MFPL_PC0MFP_Msk);
    SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC3MFP_USCI0_DAT0 | SYS_GPC_MFPL_PC0MFP_USCI0_CLK);

    /* Set PC multi-function pins for I2C0 SDA and SCL */
    SYS->GPC_MFPH &= ~(SYS_GPC_MFPH_PC11MFP_Msk | SYS_GPC_MFPH_PC12MFP_Msk);
    SYS->GPC_MFPH |= (SYS_GPC_MFPH_PC11MFP_I2C0_SDA | SYS_GPC_MFPH_PC12MFP_I2C0_SCL);

    /* Set PA multi-function pins for I2C1 SDA and SCL */
    SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA10MFP_Msk | SYS_GPA_MFPH_PA11MFP_Msk);
    SYS->GPA_MFPH |= (SYS_GPA_MFPH_PA10MFP_I2C1_SDA | SYS_GPA_MFPH_PA11MFP_I2C1_SCL);
	#elif defined (ENABLE_NUC126)	
    /* Set PD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD;

    /* Set PC multi-function pins for UI2C0_SDA(PC.5) and UI2C0_SCL(PC.4) */
    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC5MFP_Msk | SYS_GPC_MFPL_PC4MFP_Msk);
    SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC5MFP_USCI0_DAT0 | SYS_GPC_MFPL_PC4MFP_USCI0_CLK);


    /* Set I2C0 multi-function pins */
    SYS->GPA_MFPH = (SYS->GPA_MFPH & ~(SYS_GPA_MFPH_PA9MFP_Msk | SYS_GPA_MFPH_PA8MFP_Msk)) |
                    (SYS_GPA_MFPH_PA9MFP_I2C1_SDA | SYS_GPA_MFPH_PA8MFP_I2C1_SCL);

    /* Set I2C1 multi-function pins */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk)) |
                    (SYS_GPA_MFPL_PA2MFP_I2C0_SDA | SYS_GPA_MFPL_PA3MFP_I2C0_SCL);
	#endif
	
}
void I2C0_Init(void)
{
	#if defined (ENABLE_NUC125)
    SYS_ResetModule(I2C0_RST);
	#endif

    /* Open I2C0 module and set bus clock */
    I2C_Open(I2C0, 100000);

    /* Get I2C0 Bus Clock */
    printf("I2C0 clock %d Hz\n", I2C_GetBusClockFreq(I2C0));

    /* Set I2C0 4 Slave Addresses */
    I2C_SetSlaveAddr(I2C0, 0, 0x15, 0);   /* Slave Address : 0x15 */
    I2C_SetSlaveAddr(I2C0, 1, 0x35, 0);   /* Slave Address : 0x35 */
    I2C_SetSlaveAddr(I2C0, 2, 0x55, 0);   /* Slave Address : 0x55 */
    I2C_SetSlaveAddr(I2C0, 3, 0x75, 0);   /* Slave Address : 0x75 */

    /* Set I2C0 4 Slave Addresses Mask */
    I2C_SetSlaveAddrMask(I2C0, 0, 0x01);
    I2C_SetSlaveAddrMask(I2C0, 1, 0x04);
    I2C_SetSlaveAddrMask(I2C0, 2, 0x01);
    I2C_SetSlaveAddrMask(I2C0, 3, 0x04);

    /* Enable I2C0 interrupt */
    I2C_EnableInt(I2C0);
    NVIC_EnableIRQ(I2C0_IRQn);
}

void I2C1_Init(void)
{
	#if defined (ENABLE_NUC125)
    /* Reset I2C1 */
    SYS_ResetModule(I2C1_RST);
	#endif

    /* Open I2C1 module and set bus clock */
    I2C_Open(I2C1, 100000);

    /* Get I2C1 Bus Clock */
    printf("I2C1 clock %d Hz\n", I2C_GetBusClockFreq(I2C1));

    /* Set I2C1 4 Slave Addresses */
    I2C_SetSlaveAddr(I2C1, 0, 0x16, 0);   /* Slave Address : 0x16 */
    I2C_SetSlaveAddr(I2C1, 1, 0x36, 0);   /* Slave Address : 0x36 */
    I2C_SetSlaveAddr(I2C1, 2, 0x56, 0);   /* Slave Address : 0x56 */
    I2C_SetSlaveAddr(I2C1, 3, 0x76, 0);   /* Slave Address : 0x76 */

    /* Set I2C1 4 Slave Addresses Mask */
    I2C_SetSlaveAddrMask(I2C1, 0, 0x04);
    I2C_SetSlaveAddrMask(I2C1, 1, 0x02);
    I2C_SetSlaveAddrMask(I2C1, 2, 0x04);
    I2C_SetSlaveAddrMask(I2C1, 3, 0x02);

    /* Enable I2C interrupt */
    I2C_EnableInt(I2C1);
    NVIC_EnableIRQ(I2C1_IRQn);
}

void UI2C0_Init(uint32_t u32ClkSpeed)
{
    /* Open USCI_I2C0 and set clock to 100k */
    UI2C_Open(UI2C0, u32ClkSpeed);

    /* Get USCI_I2C0 Bus Clock */
    printf("UI2C0 clock %d Hz\n", UI2C_GetBusClockFreq(UI2C0));

    /* Set USCI_I2C0 Slave Addresses */
    UI2C_SetSlaveAddr(UI2C0, 0, 0x16, UI2C_GCMODE_DISABLE);   /* Slave Address : 0x16 */
    UI2C_SetSlaveAddr(UI2C0, 1, 0x36, UI2C_GCMODE_DISABLE);   /* Slave Address : 0x36 */

    /* Set USCI_I2C0 Slave Addresses Mask */
    UI2C_SetSlaveAddrMask(UI2C0, 0, 0x04);                    /* Slave Address : 0x4 */
    UI2C_SetSlaveAddrMask(UI2C0, 1, 0x02);                    /* Slave Address : 0x2 */

    /* Enable UI2C0 protocol interrupt */
    UI2C_ENABLE_PROT_INT(UI2C0, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk));
    NVIC_EnableIRQ(USCI_IRQn);

}

void I2C0_Close(void)
{
    /* Disable I2C0 interrupt and clear corresponding NVIC bit */
    I2C_DisableInt(I2C0);
    NVIC_DisableIRQ(I2C0_IRQn);

    /* Disable I2C0 and close I2C0 clock */
    I2C_Close(I2C0);
    CLK_DisableModuleClock(I2C0_MODULE);

}

void I2C1_Close(void)
{
    /* Disable I2C1 interrupt and clear corresponding NVIC bit */
    I2C_DisableInt(I2C1);
    NVIC_DisableIRQ(I2C1_IRQn);

    /* Disable I2C1 and close I2C1 clock */
    I2C_Close(I2C1);
    CLK_DisableModuleClock(I2C1_MODULE);
}

int32_t I2C0_Read_Write_Slave(uint8_t slvaddr)
{
#if 1
    uint32_t u32Index;
	
    g_u8DeviceAddr = slvaddr;

    printf("Dump transmitted data:\r\n");

    for (u32Index = 0; u32Index < I2C_DATA_MAX; u32Index++)
    {
        g_au8MstTxData[u32Index] = g_u8test_data + u32Index;
        printf("0x%2X , ", g_au8MstTxData[u32Index]);
		
        if ((u32Index+1)%8 ==0)
        {
            printf("\r\n");
        }
		
    }

	printf("\r\n");

    g_u8MonDataCnt = 0;
    g_u8MstDataLen = 0;
    g_u8MstEndFlag = 0;

    /* I2C function to write data to slave */
    s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterTx;

    /* I2C as master sends START signal */
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);

    /* Wait I2C Tx Finish */
    while (g_u8MstEndFlag == 0);

    g_u8MstEndFlag = 0;

    /* I2C function to read data from slave */
    s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterRx;

    g_u8MstDataLen = 0;
    g_u8DeviceAddr = slvaddr;

    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);

    /* Wait I2C Rx Finish */
    while (g_u8MstEndFlag == 0);

    /* Compare data */
//    if(g_u8MstRxData != g_au8MstTxData[2])
//    {
//        printf("I2C0 Byte Write/Read Failed, Data 0x%x\r\n", g_u8MstRxData);
//        while(1);
//    }

    while (g_u32ProtOn);

    return 0;
#else


    uint32_t i;

    g_u8DeviceAddr = slvaddr;

    for(i = 0; i < MONITOR_TEST_CNT; i++)
    {
        g_au8MstTxData[0] = (uint8_t)((i & 0xFF00) >> 8);
        g_au8MstTxData[1] = (uint8_t)(i & 0x00FF);
        g_au8MstTxData[2] = (uint8_t)(g_au8MstTxData[1] + 3);

        g_u8MstDataLen = 0;
        g_u8MstEndFlag = 0;

        /* I2C0 function to write data to slave */
        s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterTx;

        /* I2C0 as master sends START signal */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);

        /* Wait I2C0 Tx Finish */
        while(g_u8MstEndFlag == 0);
        g_u8MstEndFlag = 0;

        /* I2C0 function to read data from slave */
        s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterRx;

        g_u8MstDataLen = 0;
        g_u8DeviceAddr = slvaddr;

        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);

        /* Wait I2C0 Rx Finish */
        while(g_u8MstEndFlag == 0);

        /* Compare data */
        if(g_u8MstRxData != g_au8MstTxData[2])
        {
            printf("I2C0 Byte Write/Read Failed, Data 0x%x\n", g_u8MstRxData);
            while(1);
        }
    }
    printf("Master Access Slave (0x%X) Test OK\n", slvaddr);
    return 0;
#endif	
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UI2C0 Slave Monitor Callback Function                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void UI2C_SlaveTRx_Monitor(uint32_t u32Status)
{
    uint8_t u8Rxdata;

    if((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
        /* Clear START INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STARIF_Msk);

        if (g_u32ProtOn == 0)
        {
            g_u32ProtOn = 1;
        }

        s_Event = SLAVE_ADDRESS_ACK;
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
        /* Clear ACK INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ACKIF_Msk);

        if(s_Event==SLAVE_ADDRESS_ACK)
        {
            if((UI2C0->PROTSTS & UI2C_PROTSTS_SLAREAD_Msk) == UI2C_PROTSTS_SLAREAD_Msk)
            {
                u8Rxdata = (uint8_t)UI2C_GET_DATA(UI2C0);
                g_u8MonRxData[g_u8MonDataCnt++] = u8Rxdata;
                s_Event = SLAVE_SEND_DATA;				
            }
            else
            {
                u8Rxdata = (uint8_t)UI2C_GET_DATA(UI2C0);;
                g_u8MonRxData[g_u8MonDataCnt++] = u8Rxdata;
                s_Event = SLAVE_GET_DATA;				
            }

            if (((u8Rxdata >> 1) != (UI2C0->DEVADDR0 & 0xFF)) && ((u8Rxdata >> 1) != (UI2C0->DEVADDR1 & 0xFF)))
            {
                /* Check Receive Adddress not match */
                printf("\n[Error]Receive address(0x%x) not match!\n", u8Rxdata);

                while (1);
            }

            s_Event=SLAVE_GET_DATA;
        }
        else if(s_Event==SLAVE_GET_DATA)
        {
            u8Rxdata = (uint8_t)UI2C_GET_DATA(UI2C0);
            g_u8MonRxData[g_u8MonDataCnt++] = u8Rxdata;
        }

        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
        /* Clear NACK INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_NACKIF_Msk);

        u8Rxdata = (uint8_t)UI2C_GET_DATA(UI2C0);
        g_u8MonRxData[g_u8MonDataCnt++] = u8Rxdata;
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if((u32Status & UI2C_PROTSTS_STORIF_Msk) == UI2C_PROTSTS_STORIF_Msk)
    {
        /* Clear STO INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STORIF_Msk);
        g_u32ProtOn = 0;
        s_Event = SLAVE_ADDRESS_ACK;
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
}

int32_t UI2C_Monitor(void)
{
    int32_t i32Err = 0;
    uint32_t u32Index;
	
    g_u32ProtOn = 0;

    i32Err = I2C0_Read_Write_Slave(0x16);

    printf("Dump Monitor data: \r\n");

    for (u32Index = 0; u32Index < (I2C_DATA_MAX + 1); u32Index++)
    {
        if (u32Index == 0)
        {
            printf("Monitor address: [0x%2X] \r\n", g_u8MonRxData[u32Index] >> 1);
        }
        else
        {
            printf("0x%2X , ", g_u8MonRxData[u32Index]);
	        if ((u32Index)%8 ==0)
	        {
	            printf("\r\n");
	        }			
        }
    }

    printf("\r\n");

    for (u32Index = 0; u32Index < I2C_DATA_MAX; u32Index++)
        g_u8MonRxData[u32Index] = 0;

    return i32Err;
}

void UI2C_Init(void)
{
    /* Init USCI_I2C0 */
    UI2C0_Init(100000);

    s_Event = SLAVE_ADDRESS_ACK;

    UI2C0->PROTCTL |= (UI2C_PROTCTL_MONEN_Msk | UI2C_PROTCTL_SCLOUTEN_Msk);

    UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));

    /* UI2C function to Slave receive/transmit data */
    s_UI2C0HandlerFn = UI2C_SlaveTRx_Monitor;

    printf("UI2C0 Monitor Mode is Running.\r\n");
}

int main()
{
    uint32_t i;

    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

//    printf("+-------------------------------------------------------+\n");
//    printf("|  USCI_I2C Driver Sample Code for Monitor Mode         |\n");
//    printf("|  7-bit Monitor mode test                              |\n");
//    printf("|  I2C0(Master)  <----> UI2C0(Monitor) & I2C1(Slave)    |\n");
//    printf("+-------------------------------------------------------+\n");

//    printf("\n");
//    printf("Configure UI2C0 as a monitor mode.\n");
//    printf("The I/O connection for UI2C0:\n");
//    printf("UI2C0_SDA(PC.5), UI2C0_SCL(PC.4)\n");
//    printf("\n");
//    printf("Configure I2C0 as Master, and I2C1 as a slave.\n");
//    printf("The I/O connection I2C0 to I2C1:\n");
//    printf("I2C1_SDA(PA.9), I2C1_SCL(PA.8)\n");
//    printf("I2C0_SDA(PA.2), I2C0_SCL(PA.3)\r\n");
	
    /* Init UI2C0 */
    UI2C_Init();

    /* Init I2C0 */
    I2C0_Init();

    /* Init I2C1 */
    I2C1_Init();

    /* I2C1 enter non address SLV mode */
    I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);

    for(i = 0; i < 0x100; i++)
    {
        g_au8SlvData[i] = 0;
    }

    /* I2C1 function to Slave receive/transmit data */
    s_I2C1HandlerFn = I2C_SlaveTRx;

    printf("\r\n");
    printf("I2C1 Slave Mode is Running\r\n");

    while (1)
    {
        printf("Monitor test ....\r\n");
        UI2C_Monitor();

        printf("Press any key to continue (increase TX data idx 1)\r\n");
        getchar();
		g_u8test_data = (g_u8test_data >= 0xFF) ? (0x10) : (g_u8test_data + 1);

    }
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

