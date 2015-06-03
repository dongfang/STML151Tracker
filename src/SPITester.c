
/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************  
* File Name          : main.c  
* Author             : MCD Application Team  
* Date First Issued  : 05/21/2007  
* Description        : Main program body  
********************************************************************************  
* History:  
* 05/21/2007: V0.3  
********************************************************************************  
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS  
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.  
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,  
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE  
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING  
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.  
*******************************************************************************/   
   
/* Includes ------------------------------------------------------------------*/   
#include "stm32l1xx_conf.h"   
   
/* Local includes ------------------------------------------------------------*/   
/* Private typedef -----------------------------------------------------------*/   
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;   
   
/* Private define ------------------------------------------------------------*/   
#define BufferSize  32   
   
/* Private macro -------------------------------------------------------------*/   
/* Private variables ---------------------------------------------------------*/   
SPI_InitTypeDef  SPI_InitStructure;   

uint16_t SPI1_Buffer_Rx[BufferSize], SPI2_Buffer_Rx[BufferSize];
uint8_t Tx_Idx = 0, Rx_Idx = 0, k = 0;
//vu16 CRC1_Value = 0, CRC2_Value = 0;
volatile TestStatus TransferStatus1 = FAILED, TransferStatus2 = FAILED;   
ErrorStatus HSEStartUpStatus;   
   
/* Private functions ---------------------------------------------------------*/   
// void RCC_Configuration(void);
void GPIO_Configuration(void);   
void NVIC_Configuration(void);   
TestStatus Buffercmp(uint16_t* pBuffer1, uint16_t* pBuffer2, uint16_t BufferLength);
   
/*******************************************************************************  
* Function Name  : main  
* Description    : Main program  
* Input          : None  
* Output         : None  
* Return         : None  
*******************************************************************************/   
int main(void)   
{   
   
  /* System clocks configuration ---------------------------------------------*/   
  //RCC_Configuration();
   
  /* NVIC configuration ------------------------------------------------------*/   
  NVIC_Configuration();   
   
  /* GPIO configuration ------------------------------------------------------*/   
  GPIO_Configuration();   
   
  /* SPI1 configuration ------------------------------------------------------*/   
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;   
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;   
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;   
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;   
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;   
  SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;   
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;   
  SPI_InitStructure.SPI_CRCPolynomial = 7;   
  SPI_Init(SPI1, &SPI_InitStructure);   
   
  /* SPI1 configuration ------------------------------------------------------*/
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_Init(SPI1, &SPI_InitStructure);
   
  /* Disable SPI1 CRC calculation */
  SPI_CalculateCRC(SPI1, DISABLE);
  /* Enable SPI2 CRC calculation */   
  // SPI_CalculateCRC(SPI2, ENABLE);
   
  /* Enable SPI1 */   
  SPI_Cmd(SPI1, ENABLE);   
   
  /* Transfer procedure */   
  while(Tx_Idx < BufferSize-1)   
  {    
    /* Wait for SPI1 Tx buffer empty */    
    while(SPI_GetFlagStatus(SPI1, SPI_FLAG_TXE)==RESET);   
    /* Send SPI1 data */

    uint16_t data = 0;

    SPI_SendData(SPI1, data);
    /* Wait for SPI2 data reception */         
    while(SPI_GetFlagStatus(SPI1, SPI_FLAG_RXNE)==RESET);
    /* Read SPI2 received data */   
    uint8_t yyy = SPI_ReceiveData(SPI1);
  }   
   
  /* Wait for SPI1 Tx buffer empty */   
  while(SPI_GetFlagStatus(SPI1, SPI_FLAG_TXE)==RESET);   
   
  /* Wait for SPI1 last data reception */      
  while(SPI_GetFlagStatus(SPI1, SPI_FLAG_RXNE)==RESET);        
  /* Read SPI1 last received data */   
  SPI1_Buffer_Rx[Rx_Idx] = SPI_ReceiveData(SPI1);    
      
  /* Wait for SPI1 last data reception */          
  while(SPI_GetFlagStatus(SPI2, SPI_FLAG_RXNE)==RESET);   
  /* Read SPI2 last received data */   
  SPI2_Buffer_Rx[Rx_Idx] = SPI_ReceiveData(SPI2);    
   
  /* Wait for SPI1 data reception: CRC transmitted by SPI2 */     
  while(SPI_GetFlagStatus(SPI1, SPI_FLAG_RXNE)==RESET);   
  /* Read SPI1 received CRC value */   
  uint16_t CRC1_Value = SPI_ReceiveData(SPI1);
   
  /* Wait for SPI2 data reception: CRC transmitted by SPI1 */     
  while(SPI_GetFlagStatus(SPI2, SPI_FLAG_RXNE)==RESET);   
  /* Read SPI2 received CRC value */   
  uint16_t CRC2_Value = SPI_ReceiveData(SPI2);
   
//ignore the rest. It was a loopback test.
}
   
/*******************************************************************************  
* Function Name  : RCC_Configuration  
* Description    : Configures the different system clocks.  
* Input          : None  
* Output         : None  
* Return         : None  
*******************************************************************************/   
/*
void RCC_Configuration(void)   
{   
  / * RCC system reset(for debug purpose) * /
  RCC_DeInit();   
   
  / * Enable HSE * /
  RCC_HSEConfig(RCC_HSE_ON);   
   
  / * Wait till HSE is ready * /
  HSEStartUpStatus = RCC_WaitForHSEStartUp();   
   
  if(HSEStartUpStatus == SUCCESS)   
  {   
    / * HCLK = SYSCLK * /
    RCC_HCLKConfig(RCC_SYSCLK_Div1);    
     
    / * PCLK2 = HCLK/2 * /
    RCC_PCLK2Config(RCC_HCLK_Div2);    
   
    / * PCLK1 = HCLK/2 * /
    RCC_PCLK1Config(RCC_HCLK_Div2);   
    
    / * Flash 2 wait state * /
    FLASH_SetLatency(FLASH_Latency_2);   
    /* Enable Prefetch Buffer * /
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);   
   
    / * PLLCLK = 8MHz * 9 = 72 MHz * /
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);   
   
    / * Enable PLL * /
    RCC_PLLCmd(ENABLE);   
   
    / * Wait till PLL is ready * /
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)   
    {   
    }   
   
    / * Select PLL as system clock source * /
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);   
   
    / * Wait till PLL is used as system clock source * /
    while(RCC_GetSYSCLKSource() != 0x08)   
    {   
    }   
  }   
   
/ * Enable peripheral clocks --------------------------------------------------* /
  / * GPIOA, GPIOB and SPI1 clock enable * /
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |   
                         RCC_APB2Periph_SPI1, ENABLE);   
   
  / * SPI2 Periph clock enable * /
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);   
}
*/
   
/*******************************************************************************  
* Function Name  : GPIO_Configuration  
* Description    : Configures the different GPIO ports.  
* Input          : None  
* Output         : None  
* Return         : None  
*******************************************************************************/   
void GPIO_Configuration(void)   
{   
  GPIO_InitTypeDef GPIO_InitStructure;   
   
  /* Configure SPI1 pins: SCK, MISO and MOSI ---------------------------------*/   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;   
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);   
}   
   
/*******************************************************************************  
* Function Name  : NVIC_Configuration  
* Description    : Configure the nested vectored interrupt controller.  
* Input          : None  
* Output         : None  
* Return         : None  
*******************************************************************************/   
void NVIC_Configuration(void)   
{   
#ifdef  VECT_TAB_RAM     
  /* Set the Vector Table base location at 0x20000000 */    
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);    
#else  /* VECT_TAB_FLASH  */   
  /* Set the Vector Table base location at 0x08000000 */    
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);      
#endif   
}   
   
/*******************************************************************************  
* Function Name  : Buffercmp  
* Description    : Compares two buffers.  
* Input          : - pBuffer1, pBuffer2: buffers to be compared.  
*                : - BufferLength: buffer's length  
* Output         : None  
* Return         : PASSED: pBuffer1 identical to pBuffer2  
*                  FAILED: pBuffer1 differs from pBuffer2  
*******************************************************************************/   
TestStatus Buffercmp(uint16_t* pBuffer1, uint16_t* pBuffer2, uint16_t BufferLength)
{   
  while(BufferLength--)   
  {   
    if(*pBuffer1 != *pBuffer2)   
    {   
      return FAILED;   
    }   
       
    pBuffer1++;   
    pBuffer2++;   
  }   
   
  return PASSED;     
}   
   
#ifdef  DEBUG   
/*******************************************************************************  
* Function Name  : assert_failed  
* Description    : Reports the name of the source file and the source line number  
*                  where the assert error has occurred.  
* Input          : - file: pointer to the source file name  
*                  - line: assert error line source number  
* Output         : None  
* Return         : None  
*******************************************************************************/   
void assert_failed(u8* file, u32 line)   
{    
  /* User can add his own implementation to report the file name and line number,  
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */   
   
  /* Infinite loop */   
  while (1)   
  {   
  }   
}   
#endif   
   
/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/   
