//UPDATED CODE FROM LAB


/** Robert Jan 2023
---------This starter project does not work, when the NB_OF_VAR, in HAL_eeprom.h, is defined as 3 or 4!!!
This starter project: 
1. configured TIM3 as a  base timer .  Every 0.5 second there will ba an  update event (overflow) interrupt, 
2. configured TIM4 (channel 1) as Output Compared interrupt timer. Every 1/1000 second (1 ms) there is a capture interrupt.
3. Configured LCD.
4. configured USER BOTTON in EXTI mode. 
5. configured RNG
6. configured EEPROM emulator.
7. configured PC1 as the pin for EXTI 1. 



*  ******************************************************************************
  * @file    GPIO/GPIO_EXTI/Src/main.c 
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   This example describes how to configure and use GPIOs through 
  *          the STM32F4xx HAL API.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup GPIO_EXTI
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define COLUMN(x) ((x) * (((sFONT *)BSP_LCD_GetFont())->Width))    //see font.h, for defining LINE(X)


/* Private macro -------------------------------------------------------------*/

 

/* Private variables ---------------------------------------------------------*/
HAL_StatusTypeDef Hal_status;  //HAL_ERROR, HAL_TIMEOUT, HAL_OK, of HAL_BUSY 

TIM_HandleTypeDef    Tim3_Handle,Tim4_Handle;
TIM_OC_InitTypeDef Tim4_OCInitStructure;
uint16_t Tim3_PrescalerValue,Tim4_PrescalerValue;

__IO uint16_t Tim4_CCR; // the pulse of the TIM4



__IO uint8_t UBPressed = 0; //if user button is pressed, =1																ON BOARD BUTTON
__IO uint8_t extern_UBPressed=0; // if external button if pressed													BREADBOARD BUTTON
__IO uint16_t OC_Count=0;

uint16_t reactionTime = 0;	//---------------------------------------------Variable created to store user reaction time
int state;									//---------------------------------------------Variable used to store what state the board is in

char lcd_buffer[14];    // LCD display buffer

RNG_HandleTypeDef Rng_Handle;							//-------------------------USE THIS FOR RANDOM NUMBER GENERATION WITH FUNCTION FROM hal_rng.c
uint32_t random;													//-------------------^^^^^^^^^^^^^^^-------------------------------------------

uint16_t VirtAddVarTab[NB_OF_VAR] = {0x5555, 0x6666, 0x7777};
uint16_t EEREAD;  //to practice reading the BESTRESULT save in the EE, for EE read/write, require uint16_t type





/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
static void EXTILine0_Config(void);

void LCD_DisplayString(uint16_t LineNumber, uint16_t ColumnNumber, uint8_t *ptr);
void LCD_DisplayInt(uint16_t LineNumber, uint16_t ColumnNumber, int Number);
void LCD_DisplayFloat(uint16_t LineNumber, uint16_t ColumnNumber, float Number, int DigitAfterDecimalPoint);


void TIM3_Config(void);
void TIM4_Config(void);
void TIM4_OC_Config(void);

static void EXTILine1_Config(void); // configure the exti line1, for exterrnal button, using PB1



/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
	


int main(void)
{
 /* This sample code shows how to use STM32F4xx GPIO HAL API to toggle PG13 
     IOs (connected to LED3 on STM32F429i-Discovery board) 
    in an infinite loop.
    To proceed, 3 steps are required: */

  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
  */
  
		
	HAL_Init();
	
	 /* Configure the system clock to 72 MHz */
  SystemClock_Config();
	
	HAL_InitTick(0x0000); // set systick's priority to the highest, in case HAL_Delay() is used inside other ISR.
	
	//configure use button as exti mode. 
	BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);   // BSP_functions in stm32f429i_discovery.c
																			//Use this line, so do not need extiline0_config()
			
  //Configure LED3 and LED4 ======================================
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);
  BSP_LED_On(LED3);
	BSP_LED_On(LED4);
	
	
	//Configer timer =================================
	TIM3_Config();
	
	TIM4_Config();
	
	Tim4_CCR=500;       //  with clock counter freq as 500,000, this will make OC Freq as 1ms.
	TIM4_OC_Config();
 
	// ===========Config the GPIO for external interupt==============
	EXTILine1_Config();


//Configure LCD==================================================== LCD LCD LCD LCD LCD LCD LCD LCD LCD LCD LCD LCD ================
// Initialization steps :
		//     o Initialize the LCD using the LCD_Init() function.
		//    o Apply the Layer configuration using LCD_LayerDefaultInit() function    
		// 			o Select the LCD layer to be used using LCD_SelectLayer() function.
		//     o Enable the LCD display using LCD_DisplayOn() function.	
	BSP_LCD_Init();

	BSP_LCD_LayerDefaultInit(0, LCD_FRAME_BUFFER);   //LCD_FRAME_BUFFER, defined as 0xD0000000 in _discovery_lcd.h
														// the LayerIndex may be 0 and 1. if is 2, then the LCD is dark.

	BSP_LCD_SelectLayer(0);
	
	//**********************************
	BSP_LCD_SetLayerVisible(0, ENABLE); 
	//**********************************
	// in 2023, this line seems to be essetial to make LCD work reliably. otherwise, LCD is not reliable. 
	
	
	BSP_LCD_Clear(LCD_COLOR_CYAN);  //need this line, otherwise, the screen is ddark	
	BSP_LCD_DisplayOn();
 
	BSP_LCD_SetFont(&Font20);  //the default font,  LCD_DEFAULT_FONT, which is defined in _lcd.h, is Font24
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_SetBackColor(LCD_COLOR_LIGHTGRAY);
	
		
	LCD_DisplayString(1, 2, (uint8_t *)"Mitchel Cox &");
	LCD_DisplayString(2, 3, (uint8_t *)"Ali Umar");
	LCD_DisplayString(5, 2, (uint8_t *)"MT2TA4 Lab2 ");
	//LCD_DisplayString(7, 6, (uint8_t *)"Rxn time");
	//LCD_DisplayInt(8, 6, OC_Count-reactionCount);


 //**************test random number *********************
/*	Rng_Handle.Instance=RNG;  //Everytime declare a Handle, need to assign its Instance a base address. like the timer handles.... 													
	
	HAL_RNG_Init(&Rng_Handle);
	
	//test Random number=======
	Hal_status=HAL_RNG_GenerateRandomNumber(&Rng_Handle, &random); //HAL_RNG_GenerateRandomNumber(RNG_HandleTypeDef *hrng, uint32_t *random32bit);
     //since the randam32bit is type of uint32_t, sometimes it may reture a negative value.
	
		 
	random &=0x000007FF; // the random numberf is 32bits and is too large for this project. 
												//just use  its last 10 bits.
												//the range is 0 to 2047 for unsigned int.
												
	if (Hal_status==HAL_ERROR || Hal_status==HAL_TIMEOUT) // a new rng was NOT generated sucessfully;
		 random=1000; // millisecond-------------------------------------------------------------------------1000 = 1ms --------------for scale for calculations	
	LCD_DisplayString(9, 0, (uint8_t *)"random:");
	LCD_DisplayString(9, 8, (uint8_t *)"         ");
	LCD_DisplayInt(9, 8, random);
*/
	
	
	
//******************* use emulated EEPROM ====================================

//Unlock the Flash Program Erase controller 
	HAL_FLASH_Unlock();
	//LCD_DisplayInt(10, 1, 1);	  //!!!!!the 1, 2, 3, 4 printed here is for debuging.....to check 		
	
// EEPROM Init 
	EE_Init();
	//LCD_DisplayInt(10, 4, 2);
 	
	//test EEPROM----
	//EE_WriteVariable(VirtAddVarTab[0], 999); 
	//LCD_DisplayInt(10, 7, 3);
	
	EE_ReadVariable(VirtAddVarTab[0], &EEREAD);	
	//LCD_DisplayInt(10, 10, 4);

	LCD_DisplayString(11,0,(uint8_t *)"High Score:");
	LCD_DisplayString(11,11,(uint8_t *)"   ");	
	LCD_DisplayInt(11, 11, EEREAD);		
	
	LCD_DisplayString(9, 0, (uint8_t *)"Your Score:");
	//LCD_DisplayString(9, 11, (uint8_t *)"   ");

 
 
 //int state;
 state = 0; //-----------------------------------------------------------Set initial state to 0 (default state)
 
  /* Infinite loop*/
  while (1)
	{	
		 	

}
}	
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
	* 					Oscillator											=HSE
	*    				HSE frequencey 										=8,000,000   (8MHz)
	*      ----However, if the project is created by uVision, the default HSE_VALUE is 25MHz. thereore, need to define HSE_VALUE
	*						PLL Source											=HSE
  *            PLL_M                          = 4
  *            PLL_N                          = 72
  *            PLL_P                          = 2
  *            PLL_Q                          = 3
  *        --->therefore, PLLCLK =8MHz X N/M/P=72MHz   
	*            System Clock source            = PLL (HSE)
  *        --> SYSCLK(Hz)                     = 72,000,000
  *            AHB Prescaler                  = 1
	*        --> HCLK(Hz)                       = 72 MHz
  *            APB1 Prescaler                 = 2
	*        --> PCLK1=36MHz,  -->since TIM2, TIM3, TIM4 TIM5...are on APB1, these TIMs CLK is 36X2=72MHz
							 	
  *            APB2 Prescaler                 = 1
	*        --> PCLK2=72MHz 
	*       
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Activate the Over-Drive mode */
  HAL_PWREx_EnableOverDrive();
 
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}





void LCD_DisplayString(uint16_t LineNumber, uint16_t ColumnNumber, uint8_t *ptr)
{  
  //here the LineNumber and the ColumnNumber are NOT  pixel numbers!!!
		while (*ptr!=NULL)
    {
				BSP_LCD_DisplayChar(COLUMN(ColumnNumber),LINE(LineNumber), *ptr); //new version of this function need Xpos first. so COLUMN() is the first para.
				ColumnNumber++;
			 //to avoid wrapping on the same line and replacing chars 
				if ((ColumnNumber+1)*(((sFONT *)BSP_LCD_GetFont())->Width)>=BSP_LCD_GetXSize() ){
					ColumnNumber=0;
					LineNumber++;
				}
					
				ptr++;
		}
}

void LCD_DisplayInt(uint16_t LineNumber, uint16_t ColumnNumber, int Number)
{  
  //here the LineNumber and the ColumnNumber are NOT  pixel numbers!!!
		char lcd_buffer[15];
		sprintf(lcd_buffer,"%d",Number);
	
		LCD_DisplayString(LineNumber, ColumnNumber, (uint8_t *) lcd_buffer);
}

void LCD_DisplayFloat(uint16_t LineNumber, uint16_t ColumnNumber, float Number, int DigitAfterDecimalPoint)
{  
  //here the LineNumber and the ColumnNumber are NOT  pixel numbers!!!
		char lcd_buffer[15];
		
		sprintf(lcd_buffer,"%.*f",DigitAfterDecimalPoint, Number);  //6 digits after decimal point, this is also the default setting for Keil uVision 4.74 environment.
	
		LCD_DisplayString(LineNumber, ColumnNumber, (uint8_t *) lcd_buffer);
}



// set up timer 3

void  TIM3_Config(void)
{

		/* -----------------------------------------------------------------------
    In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1.   
      TIM3CLK = 2 * PCLK1  
      PCLK1 = HCLK / 2 
      => TIM3CLK = HCLK  = SystemCoreClock
    To get TIM3 counter clock at 10 KHz, the Prescaler is computed as following:
    Prescaler = (TIM3CLK / TIM3 counter clock) - 1
    Prescaler = (SystemCoreClock /10 KHz) - 1
       
    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock 
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency  
  ----------------------------------------------------------------------- */  
  
  /* Compute the prescaler value to have TIM3 counter clock equal to 10 KHz */
  Tim3_PrescalerValue = (uint32_t) (SystemCoreClock / 10000) - 1;
  
  /* Set TIM3 instance */
  Tim3_Handle.Instance = TIM3; //TIM3 is defined in stm32f429xx.h
   
  /* Initialize TIM3 peripheral as follows:
       + Period = 5000 - 1
       + Prescaler = (SystemCoreClock/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
  Tim3_Handle.Init.Period = 5000 - 1;     // takes 0.5sec
  Tim3_Handle.Init.Prescaler = Tim3_PrescalerValue;
  Tim3_Handle.Init.ClockDivision = 0;
  Tim3_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&Tim3_Handle) != HAL_OK) // this line need to call the callback function _MspInit() in stm32f4xx_hal_msp.c to set up peripheral clock and NVIC..
  {
    Error_Handler();
  }
  
  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  if(HAL_TIM_Base_Start_IT(&Tim3_Handle) != HAL_OK)   //the TIM_XXX_Start_IT function enable IT, and also enable Timer
																											//so do not need HAL_TIM_BASE_Start() any more.
  {
    Error_Handler();
  }
	
}


// configure Timer4 base.
void  TIM4_Config(void)
{

		/* -----------------------------------------------------------------------
    In this example TIM4 input clock (TIM4CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1.   
      TIM4CLK = 2 * PCLK1  
      PCLK1 = HCLK / 2
      => TIM4CLK = HCLK = SystemCoreClock
    To get TIM4 counter clock at 500 KHz, the Prescaler is computed as following:
    Prescaler = (TIM4CLK / TIM4 counter clock) - 1
    Prescaler = (SystemCoreClock  /500 KHz) - 1
       
    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock 
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency  
  ----------------------------------------------------------------------- */  
  
  /* -----------------------------------------------------------------------
    TIM4 Configuration: Output Compare Timing Mode:
                                               
    (if CCR1_Val=500, then every 1 ms second will have an interrupt. If count 500 times of interrupt, thta is  0.5 seconds.
		 ----------------------------------------------------------------------- */ 	

/* Compute the prescaler value to have TIM4 counter clock equal to 500 KHz */
  Tim4_PrescalerValue = (uint32_t) (SystemCoreClock  / 500000) - 1;
  
  /* Set TIM3 instance */
  Tim4_Handle.Instance = TIM4; //TIM3 is defined in stm32f429xx.h
   
  /* Initialize TIM4 peripheral as follows:
       + Period = 65535
       + Prescaler = ((SystemCoreClock/2)/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
	
	Tim4_Handle.Init.Period = 65535;  //does no matter, so set it to max.
  Tim4_Handle.Init.Prescaler = Tim4_PrescalerValue;
  Tim4_Handle.Init.ClockDivision = 0;
  Tim4_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  //if(HAL_TIM_Base_Init(&Tim4_Handle) != HAL_OK)
  //{
    /* Initialization Error */
  //  Error_Handler();
  //} 
}



void  TIM4_OC_Config(void)
{
		Tim4_OCInitStructure.OCMode=  TIM_OCMODE_TIMING;
		Tim4_OCInitStructure.Pulse=Tim4_CCR;    // 500, this means every 1/1000 second generate an inerrupt
		Tim4_OCInitStructure.OCPolarity=TIM_OCPOLARITY_HIGH;
		
		HAL_TIM_OC_Init(&Tim4_Handle); // if the TIM4 has not been set, then this line will call the callback function _MspInit() 
													//in stm32f4xx_hal_msp.c to set up peripheral clock and NVIC.
	
		HAL_TIM_OC_ConfigChannel(&Tim4_Handle, &Tim4_OCInitStructure, TIM_CHANNEL_1); //must add this line to make OC work.!!!
	
	   /* **********see the top part of the hal_tim.c**********
		++ HAL_TIM_OC_Init and HAL_TIM_OC_ConfigChannel: to use the Timer to generate an 
              Output Compare signal. 
			similar to PWD mode and Onepulse mode!!!
	
	*******************/
	
	 	HAL_TIM_OC_Start_IT(&Tim4_Handle, TIM_CHANNEL_1); //this function enable IT and enable the timer. so do not need
				//HAL_TIM_OC_Start() any more
				
		
}
	

static void EXTILine1_Config(void)  //for STM32f429_DISCO board, can not use PA1, PB1 and PD1,---PC1 is OK!!!!
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOB clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull =GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_1;
	//GPIO_InitStructure.Speed=GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)   //see  stm32fxx_hal_tim.c for different callback function names. 
																															//for timer 3 , Timer 3 use update event initerrupt
{
		if ((*htim).Instance==TIM3){    //since only one timer use this interrupt, this line is actually not needed
			if (state == 0){						//if in state 0, LEDs will toggle 
				
				BSP_LED_Toggle(LED3);			//GREEN LED
				BSP_LED_Toggle(LED4);	 		//RED LED
			}
		}
}


void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef * htim) //see  stm32fxx_hal_tim.c for different callback function names. 
{																																//for timer4

		if ((*htim).Instance==TIM4 && state == 1) {   //be careful, every 1/1000 second there is a interrupt with the current configuration for TIM4								
				OC_Count=OC_Count+1;											//OC_Count increments every interrupt (from above)
						
			if (OC_Count==random)  {   //random is from RNG in HAL_GPIO_EXTI_Callback function (in state 1)
																//when OC_Count is equal to the random number generated, the LEDs turn on
						BSP_LED_On(LED3);
						BSP_LED_On(LED4);
						
							
						
			}		
		}
		else if ((*htim).Instance==TIM4 && state == 3){						//waiting for external user button to be pressed after playing without cheating
				LCD_DisplayString(7, 1, (uint8_t *)"Extern to reset");
				UBPressed = 0;
				OC_Count = 0;
			}

			//clear the timer counter!  in stm32f4xx_hal_tim.c, the counter is not cleared after  OC interrupt
		__HAL_TIM_SET_COUNTER(htim, 0x0000);   //this maro is defined in stm32f4xx_hal_tim.h
		
		
		
		
		
		
}	
/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) //------------------------------------------THIS CAUSES THE INTERUPTS FROM BUTTON PRESS
{	
	
	

	
  if(GPIO_Pin == KEY_BUTTON_PIN)  //GPIO_PIN_0 (on board button)
  {
		
		
    UBPressed=1;
		if (state == 0 && UBPressed==1){		//changes from state 0 to state 1
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);					
			BSP_LED_Off(LED3);															//turns green led off
			BSP_LED_Off(LED4);															//turns red led off
			
			Rng_Handle.Instance=RNG;  //Everytime declare a Handle, need to assign its Instance a base address. like the timer handles.... 													
			HAL_RNG_Init(&Rng_Handle);
			Hal_status=HAL_RNG_GenerateRandomNumber(&Rng_Handle, &random); //HAL_RNG_GenerateRandomNumber(RNG_HandleTypeDef *hrng, uint32_t *random32bit)
			random = 1500 + random%2000; //random number can be very big, to reduce it and keep it around 2 seconds we have it as 1.5 sec + random number mod 2000
		
												
			if (Hal_status==HAL_ERROR || Hal_status==HAL_TIMEOUT){ // a new rng was NOT generated sucessfully;
					random=1000; // millisecond-------------------------------------------------------------------------1000 = 1ms --------------for scale for calculations
				}
			
			state = 1;											//setting state to 1
			UBPressed=0;
			OC_Count = 0;
		}
		
		else if(state == 1 && UBPressed ==1){							
					reactionTime = OC_Count - random;  //calculating reaction time 
					if (OC_Count < random){		//played with cheating
						BSP_LCD_SetTextColor(LCD_COLOR_RED);
						//LCD_DisplayString(3, 5, (uint8_t *)"CHEATER");
						LCD_DisplayString(9, 11, (uint8_t *)"CHT");
						//LCD_DisplayString(7, 1, (uint8_t *)"Extern to reset");
						UBPressed = 0;
						state = 0;
						OC_Count = 0;
						
					}
					else {
			
			
					if ((reactionTime)<EEREAD){		//played without cheating							
						EE_WriteVariable(VirtAddVarTab[0], reactionTime);	//writes to EEREAD variable if reactionCount is lower than EEREAD
	
						EE_ReadVariable(VirtAddVarTab[0], &EEREAD);				//reads the value from EEREAD

					}
					LCD_DisplayInt(11, 11, EEREAD);
					
					LCD_DisplayInt(9, 11, reactionTime);
					
					LCD_DisplayString(7, 1, (uint8_t *)"Extern to reset");
					OC_Count = 0;
					state = 3;									//setting state to 3 (post without cheating)
					
					
				}
		}

		
		
		
	}
	
	
	if(GPIO_Pin == GPIO_PIN_1)
  {
			extern_UBPressed=1;
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			if (extern_UBPressed == 1){							//resets LCD when extern button is pressed
				BSP_LCD_SetBackColor(LCD_COLOR_CYAN);
				BSP_LCD_ClearStringLine(3);
				BSP_LCD_ClearStringLine(7);
				BSP_LCD_ClearStringLine(9);
				BSP_LCD_SetBackColor(LCD_COLOR_LIGHTGRAY);
				LCD_DisplayString(9, 0, (uint8_t *)"Your Score:");
				
				state = 0;													//returning to default state (flashing LEDs)
				extern_UBPressed=0;
				BSP_LCD_SetBackColor(LCD_COLOR_LIGHTGRAY);
			}
			
	}
 
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED4 on */
  BSP_LED_On(LED4);
  while(1)
  {
  }
}


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
