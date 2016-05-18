
#include "stm32l1xx.h"
#define VECT_TAB_OFFSET  0x0 /*!< Vector Table base offset field. 
                                  This value must be a multiple of 0x200. */

/** @addtogroup STM32L1xx_System_Private_Variables
 * @{
 */
uint32_t SystemCoreClock = 16000000;
__I uint8_t PLLMulTable[9] = { 3, 4, 6, 8, 12, 16, 24, 32, 48 };
__I uint8_t AHBPrescTable[16] =
		{ 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9 };

int SetSysClock();

/**
 * @brief  Setup the microcontroller system.
 *         Initialize the Embedded Flash Interface, the PLL and update the
 *         SystemCoreClock variable.
 * @param  None
 * @retval None
 */
void SystemInit(void) {
	/*!< Clear the RTCPRE bits (undefined at reset) */
	RCC->CR &= 0x81FFFFFF;
	/*!< Set MSION bit (medium speed internal) (surprisingly it is off at reset time)
	 * Also set RTCPRE to divide by 8 (HSE to RTC clock 1MHz).
	 * */
	RCC->CR |= (uint32_t) 0x400000100;

	// PPRE1 = divide by 2
	/*!< Reset SW[1:0], HPRE[3:0], PPRE1[2:0], PPRE2[2:0], MCOSEL[2:0] and MCOPRE[2:0] bits */
	RCC->CFGR &= (uint32_t) 0x88FFC40C;

	/*!< Reset HSION, HSEON, CSSON and PLLON bits (yes right for STM32L151CB) */
	RCC->CR &= (uint32_t) 0xEEFEFFFE;

	/*!< Reset HSEBYP bit (OK) */
	RCC->CR &= (uint32_t) 0xFFFBFFFF;

	/*!< Reset PLLSRC, PLLMUL[3:0] and PLLDIV[1:0] bits (presumably OK) */
	RCC->CFGR &= (uint32_t) 0xFF02FFFF;

	/*!< Disable all interrupts */
	RCC->CIR = 0x00000000;

	/* Configure the System clock frequency, AHB/APBx prescalers and Flash settings */
	// SetSysClock();

#ifdef VECT_TAB_SRAM
	SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM. */
#else
	SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH. */
#endif
}

/**
 * @brief  Update SystemCoreClock according to Clock Register Values
 *         The SystemCoreClock variable contains the core clock (HCLK), it can
 *         be used by the user application to setup the SysTick timer or configure
 *         other parameters.
 *
 * @note   Each time the core clock (HCLK) changes, this function must be called
 *         to update SystemCoreClock variable value. Otherwise, any configuration
 *         based on this variable will be incorrect.
 *
 * @note   - The system frequency computed by this function is not the real
 *           frequency in the chip. It is calculated based on the predefined
 *           constant and the selected clock source:
 *
 *           - If SYSCLK source is MSI, SystemCoreClock will contain the MSI
 *             value as defined by the MSI range.
 *
 *           - If SYSCLK source is HSI, SystemCoreClock will contain the HSI_VALUE(*)
 *
 *           - If SYSCLK source is HSE, SystemCoreClock will contain the HSE_VALUE(**)
 *
 *           - If SYSCLK source is PLL, SystemCoreClock will contain the HSE_VALUE(**)
 *             or HSI_VALUE(*) multiplied/divided by the PLL factors.
 *
 *         (*) HSI_VALUE is a constant defined in stm32l1xx.h file (default value
 *             16 MHz) but the real value may vary depending on the variations
 *             in voltage and temperature.
 *
 *         (**) HSE_VALUE is a constant defined in stm32l1xx.h file (default value
 *              8 MHz), user has to ensure that HSE_VALUE is same as the real
 *              frequency of the crystal used. Otherwise, this function may
 *              have wrong result.
 *
 *         - The result of this function could be not correct when using fractional
 *           value for HSE crystal.
 * @param  None
 * @retval None
void SystemCoreClockUpdate(void) {
	uint32_t tmp = 0, pllmul = 0, plldiv = 0, pllsource = 0, msirange = 0;

	/ * Get SYSCLK source -------------------------------------------------------* /
	tmp = RCC->CFGR & RCC_CFGR_SWS;

	switch (tmp) {
	case 0x00: / * MSI used as system clock * /
		msirange = (RCC->ICSCR & RCC_ICSCR_MSIRANGE) >> 13;
		SystemCoreClock = (32768 * (1 << (msirange + 1)));
		break;
	case 0x04: / * HSI used as system clock * /
		SystemCoreClock = HSI_VALUE;
		break;
	case 0x08: / * HSE used as system clock * /
		SystemCoreClock = HSE_VALUE;
		break;
	case 0x0C: / * PLL used as system clock * /
		/ * Get PLL clock source and multiplication factor ----------------------* /
		pllmul = RCC->CFGR & RCC_CFGR_PLLMUL;
		plldiv = RCC->CFGR & RCC_CFGR_PLLDIV;
		pllmul = PLLMulTable[(pllmul >> 18)];
		plldiv = (plldiv >> 22) + 1;

		pllsource = RCC->CFGR & RCC_CFGR_PLLSRC;

		if (pllsource == 0x00) {
			/ * HSI oscillator clock selected as PLL clock entry * /
			SystemCoreClock = (((HSI_VALUE) * pllmul) / plldiv);
		} else {
			/ * HSE selected as PLL clock entry * /
			SystemCoreClock = (((HSE_VALUE) * pllmul) / plldiv);
		}
		break;
	default: / * MSI used as system clock * /
		msirange = (RCC->ICSCR & RCC_ICSCR_MSIRANGE) >> 13;
		SystemCoreClock = (32768 * (1 << (msirange + 1)));
		break;
	}
	/ * Compute HCLK clock frequency --------------------------------------------* /
	/ * Get HCLK prescaler * /
	tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
	/ * HCLK clock frequency * /
	SystemCoreClock >>= tmp;
}
*/

/**
 * @brief  Configures the System clock frequency, AHB/APBx prescalers and Flash
 *         settings.
 * @note   This function should be called only once the RCC clock configuration
 *         is reset to the default reset state (done in SystemInit() function).
 * @param  None
 * @retval None
 */
int SetSysClock(void) {
	__IO uint32_t StartUpCounter = 0, HSEStatus = 0;

	/* SYSCLK, HCLK, PCLK2 and PCLK1 configuration ---------------------------*/
	/* Enable HSE */
	RCC->CR |= ((uint32_t) RCC_CR_HSEON);

	/* Wait till HSE is ready and if Time out is reached exit */
	do {
		HSEStatus = RCC->CR & RCC_CR_HSERDY;
		StartUpCounter++;
	} while ((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

	if ((RCC->CR & RCC_CR_HSERDY) != RESET) {
		HSEStatus = (uint32_t) 0x01;
	} else {
		HSEStatus = (uint32_t) 0x00;
	}

	if (HSEStatus == (uint32_t) 0x01) {
		/* Enable 64-bit access */
		FLASH->ACR |= FLASH_ACR_ACC64;

		/* Enable Prefetch Buffer */
		FLASH->ACR |= FLASH_ACR_PRFTEN;

		/* Flash 1 wait state */
		FLASH->ACR |= FLASH_ACR_LATENCY;

		/* Power enable */
		RCC->APB1ENR |= RCC_APB1ENR_PWREN;

		/* Select the Voltage Range 1 (1.8 V). Seems to be a MUST. */
		PWR->CR = PWR_CR_VOS_0;

		/* Wait Until the Voltage Regulator is ready */
		while ((PWR->CSR & PWR_CSR_VOSF) != RESET) {
		}

		/* HCLK = SYSCLK /1*/
		RCC->CFGR |= (uint32_t) RCC_CFGR_HPRE_DIV1;

		/* PCLK2 = HCLK /1*/
		RCC->CFGR |= (uint32_t) RCC_CFGR_PPRE2_DIV1;

		/* PCLK1 = HCLK /1*/
		RCC->CFGR |= (uint32_t) RCC_CFGR_PPRE1_DIV1;

		/*  PLL configuration aka kill mul and div, then use mul=12 and div=2 (yes yes that is * 4, 8*4=32. Too much for our 16MHz!! */
		// RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL |
		//                                    RCC_CFGR_PLLDIV));
		// RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMUL12 | RCC_CFGR_PLLDIV3);
		/*  PLL configuration aka kill mul and div, then use mul=6 and div=2 (16 * 6 / 3 = 32) */
		RCC->CFGR &= (uint32_t) ((uint32_t) ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL
				| RCC_CFGR_PLLDIV));
		RCC->CFGR |= (uint32_t) (RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMUL3
		 		| RCC_CFGR_PLLDIV3);

		/* Enable PLL */
		//RCC->CR |= RCC_CR_PLLON;

		/* Wait till PLL is ready */
		//while ((RCC->CR & RCC_CR_PLLRDY) == 0) {
		//}

		/* Wait till HSE is ready. Nah don't bother. It already is. */
		//while((RCC->CR & RCC_CR_HSERDY) == 0)
		//{
		//}
		/* Select PLL as system clock source */
		//RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
		//RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;

		// Nah, use HSE directly.
		RCC->CFGR &= (uint32_t) ((uint32_t) ~(RCC_CFGR_SW));
		RCC->CFGR |= (uint32_t) RCC_CFGR_SW_HSE;

		/* Wait till PLL is used as system clock source */
		//while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL)
		//{
		//}
		/* Wait till HSE is used as system clock source */
		while ((RCC->CFGR & (uint32_t) RCC_CFGR_SWS)
				!= (uint32_t) RCC_CFGR_SWS_HSE) {
		}
		return 1;
	} else {
		/* If HSE fails to start-up, the application will have wrong clock
		 configuration. User can add here some code to deal with this error */
		// trace_printf("setSysClk failed!\n");
		return 0;
	}
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
