/**
  ******************************************************************************
  * @file    system_stm32f4xx.c
  * @author  MCD Application Team
  * @brief   CMSIS Cortex-M4 Device Peripheral Access Layer System Source File.
  *
  *   This file provides two functions and one global variable to be called from
  *   user application:
  *      - SystemInit(): This function is called at startup just after reset and
  *                      before branch to main program. This call is made inside
  *                      the "startup_stm32f4xx.s" file.
  *
  *      - SystemCoreClock variable: Contains the core clock (HCLK), it can be used
  *                                  by the user application to setup the SysTick
  *                                  timer or configure other parameters.
  *
  *      - SystemCoreClockUpdate(): Updates the variable SystemCoreClock and must
  *                                 be called whenever the core clock is changed
  *                                 during program execution.
  *
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32f4xx_system
  * @{
  */  
  
/** @addtogroup STM32F4xx_System_Private_Includes
  * @{
  */


#include "stm32f4xx.h"

#if !defined  (HSE_VALUE) 
#define HSE_VALUE    ((uint32_t)25000000) /*!< Default value of the External oscillator in Hz */
#endif /* HSE_VALUE */

#if !defined  (HSI_VALUE)
#define HSI_VALUE    ((uint32_t)16000000) /*!< Value of the Internal oscillator in Hz*/
#endif /* HSI_VALUE */

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Defines
  * @{
  */

/************************* Miscellaneous Configuration ************************/
/*!< Uncomment the following line if you need to use external SRAM or SDRAM as data memory  */
#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx)\
 || defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)\
 || defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F412Zx) || defined(STM32F412Vx)
/* #define DATA_IN_ExtSRAM */
#endif /* STM32F40xxx || STM32F41xxx || STM32F42xxx || STM32F43xxx || STM32F469xx || STM32F479xx ||\
          STM32F412Zx || STM32F412Vx */
 
#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)\
 || defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx)
/* #define DATA_IN_ExtSDRAM */
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx || STM32F446xx || STM32F469xx ||\
          STM32F479xx */

/*!< Uncomment the following line if you need to relocate your vector Table in
     Internal SRAM. */
/* #define VECT_TAB_SRAM */
#define VECT_TAB_OFFSET  0x00 /*!< Vector Table base offset field. 
                                   This value must be a multiple of 0x200. */
/******************************************************************************/

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Variables
  * @{
  */
  /* This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetHCLKFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency 
         Note: If you use this function to configure the system clock; then there
               is no need to call the 2 first functions listed above, since SystemCoreClock
               variable is updated automatically.
  */
uint32_t SystemCoreClock = 16000000;
const uint8_t AHBPrescTable[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9 };
const uint8_t APBPrescTable[8] = { 0, 0, 0, 0, 1, 2, 3, 4 };
/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_FunctionPrototypes
  * @{
  */

#if defined (DATA_IN_ExtSRAM) || defined (DATA_IN_ExtSDRAM)
static void SystemInit_ExtMemCtl(void);
#endif /* DATA_IN_ExtSRAM || DATA_IN_ExtSDRAM */

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Functions
  * @{
  */

/**
  * @brief  Setup the microcontroller system
  *         Initialize the FPU setting, vector table location and External memory 
  *         configuration.
  * @param  None
  * @retval None
  */
void SystemInit(void)
{
	/* FPU settings ------------------------------------------------------------*/
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
	SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 and CP11 Full Access */
#endif
  /* Reset the RCC clock configuration to the default reset state ------------*/
  /* Set HSION bit */
	RCC->CR |= (uint32_t)0x00000001;

	/* Reset CFGR register */
	RCC->CFGR = 0x00000000;

	/* Reset HSEON, CSSON and PLLON bits */
	RCC->CR &= (uint32_t)0xFEF6FFFF;

	/* Reset PLLCFGR register */
	RCC->PLLCFGR = 0x24003010;

	/* Reset HSEBYP bit */
	RCC->CR &= (uint32_t)0xFFFBFFFF;

	/* Disable all interrupts */
	RCC->CIR = 0x00000000;



	/* Configure the Vector Table location add offset address ------------------*/
#ifdef VECT_TAB_SRAM
	SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
#else
	SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
#endif

	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;

	volatile uint32_t StartUpCounter = 0, HSIStatus = 0;

	//Enable HSI//
	RCC->CR |= RCC_CR_HSION;
	//Wait for HSI to be ready
	do {
		HSIStatus = RCC->CR & RCC_CR_HSIRDY;
		StartUpCounter++;
	} while ((HSIStatus == 0) && (StartUpCounter != 500))
		;
	if ((RCC->CR & RCC_CR_HSIRDY) != 0)
	{
		HSIStatus = 0x01;
	}
	else
	{
		HSIStatus = 0x00;
	}
	if (HSIStatus == 0x01)
	{
		//Enable high performance mode//
		RCC->APB1ENR |= RCC_APB1ENR_PWREN;
		PWR->CR |= PWR_CR_VOS;
		// HCLK = SYSCLK/1
		RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
		// PLCK2 = HCLK / 2
		RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;
		// PLCK1 = HCLK / 4
		RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;

		//Configure the main PLL
#define PLL_M 8
#define PLL_N 180
#define PLL_P 0 // 0x00 = PLL_P == 2
#define PLL_Q 4
			  RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (PLL_P << 16) | (PLL_Q << 24);

		//Enable the main PLL
		RCC->CR |= RCC_CR_PLLON;
		//Wait for PLL to turn on
		while((RCC->CR & RCC_CR_PLLRDY) == 0);

		//Set power over drive
		PWR->CR |= 0x01 << 16;
		//Wait for over drive to become ready
		while((PWR->CSR & (0x01 << 16) != 1));
		PWR->CR |= 0x01 << 17;
		while ((PWR->CSR & (0x01 << 17) != 1))
			;

		//Configure Flash
		FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;

		//Select main PLL as system clock source
		RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
		RCC->CFGR |= RCC_CFGR_SW_PLL;
		//Wait for the main PLL to be used as system clock
		while((RCC->CFGR & (uint32_t) RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);



	}
	else
	{
		//HSE failed to start up
		 while(1);
	}
	//Enable the CCM RAM clock
	RCC->AHB1ENR |= (1 << 20);


}

/**
   * @brief  Update SystemCoreClock variable according to Clock Register Values.
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
  *           - If SYSCLK source is HSI, SystemCoreClock will contain the HSI_VALUE(*)
  *                                              
  *           - If SYSCLK source is HSE, SystemCoreClock will contain the HSE_VALUE(**)
  *                          
  *           - If SYSCLK source is PLL, SystemCoreClock will contain the HSE_VALUE(**) 
  *             or HSI_VALUE(*) multiplied/divided by the PLL factors.
  *         
  *         (*) HSI_VALUE is a constant defined in stm32f4xx_hal_conf.h file (default value
  *             16 MHz) but the real value may vary depending on the variations
  *             in voltage and temperature.   
  *    
  *         (**) HSE_VALUE is a constant defined in stm32f4xx_hal_conf.h file (its value
  *              depends on the application requirements), user has to ensure that HSE_VALUE
  *              is same as the real frequency of the crystal used. Otherwise, this function
  *              may have wrong result.
  *                
  *         - The result of this function could be not correct when using fractional
  *           value for HSE crystal.
  *     
  * @param  None
  * @retval None
  */
void SystemCoreClockUpdate(void)
{
	uint32_t tmp = 0, pllvco = 0, pllp = 2, pllsource = 0, pllm = 2;
  
	/* Get SYSCLK source -------------------------------------------------------*/
	tmp = RCC->CFGR & RCC_CFGR_SWS;

	switch (tmp)
	{
	case 0x00:  /* HSI used as system clock source */
		SystemCoreClock = HSI_VALUE;
		break;
	case 0x04:  /* HSE used as system clock source */
		SystemCoreClock = HSE_VALUE;
		break;
	case 0x08:  /* PLL used as system clock source */

	  /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N
	     SYSCLK = PLL_VCO / PLL_P
	     */
		pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
		pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
      
		if (pllsource != 0)
		{
			/* HSE used as PLL clock source */
			pllvco = (HSE_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
		}
		else
		{
			/* HSI used as PLL clock source */
			pllvco = (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
		}

		pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> 16) + 1) * 2;
		SystemCoreClock = pllvco / pllp;
		break;
	default:
		SystemCoreClock = HSI_VALUE;
		break;
	}
	/* Compute HCLK frequency --------------------------------------------------*/
	/* Get HCLK prescaler */
	tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
	/* HCLK frequency */
	SystemCoreClock >>= tmp;
}
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
