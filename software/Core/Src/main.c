///////////////////////////////////////////////////////////////////////////////////////////////////
// main.c
// Copyright © 2021 Jeffrey Mathews All rights reserved.
///////////////////////////////////////////////////////////////////////////////////////////////////


#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "main.h"


void SystemClock_Config(void);
static void MX_GPIO_Init(void);

#define HAL_GPIO_ReadPin(port,pin)          LL_GPIO_IsInputPinSet(port,pin)
#define HAL_GPIO_WritePin(port,pin,value)   do { if(value) LL_GPIO_SetOutputPin(port,pin); else LL_GPIO_ResetOutputPin(port,pin); } while(0)

#define HAL_GetTick()                       (systick)
// tuned at 10s, assumes 32MHz sysclock
#define delayUs(us)                         for (int i=0;i<us*6;i++){ asm("nop"); }

// tuned on 6/1/2021
#define rodInchesPerRotation                0.316
#define stepsPerRotation                    200
#define stepsPerInch                        (stepsPerRotation / rodInchesPerRotation)
#define stepsPerLevel                       ((3.5/2) * stepsPerInch )
#define stepsPerCamDegree                   (stepsPerLevel / 90)

#define STEP_RAMP                           4
#define STEP_SIZE                           400



static volatile uint32_t systick = 0;
// called from stm32l0xx_it.c weak link ISR
void mySysTick_Handler(void) {
  systick++;
}

bool fault = false;

void myIRQ_0_1(void) {
  // #define S_NFLT_Pin LL_GPIO_PIN_0
  // #define S_NFLT_GPIO_Port GPIOA
  // #define S_NFLT_EXTI_IRQn EXTI0_1_IRQn
  fault = true;
}

void myIRQ_4_15(void) {  
  // #define LIMIT_Pin LL_GPIO_PIN_9
  // #define LIMIT_GPIO_Port GPIOB
  // #define LIMIT_EXTI_IRQn EXTI4_15_IRQn
  //fault = true;
}


typedef enum {
    left    = 0,
    right   = 1,
    limit   = 2
} button_t;

typedef enum {
    cw = 1,
    ccw = -1
} direction_t;

static uint8_t readPin(button_t id) {
  switch ( id ) {
    case left:
      return HAL_GPIO_ReadPin( SW0_GPIO_Port, SW0_Pin ); 
    case right: 
      return HAL_GPIO_ReadPin( SW1_GPIO_Port, SW1_Pin ); 
    case limit: 
      return HAL_GPIO_ReadPin( LIMIT_GPIO_Port, LIMIT_Pin ); 
  }
  return 0;
}

static bool debouncedSwitch(button_t id) {
    static uint32_t lastDebounceTime[3] = {0,0,0};
    static uint8_t buttonState[3] = {1,1,1};
    static uint8_t lastButtonState[3] = {1,1,1};
    uint32_t millis = HAL_GetTick();
    uint8_t reading = readPin( id );
    if ( reading != lastButtonState[id] ) {
      lastDebounceTime[id] = millis;
    }
    uint32_t debounceDelay = buttonState[id] ? 25 : 250;
    if ( (millis - lastDebounceTime[id]) > debounceDelay ) {
      if ( reading != buttonState[id] ) {
        buttonState[id] = reading;
        if ( reading == 0 ) { // button is active low
          lastButtonState[id] = reading;
          return true;
        }
      }
    }
    lastButtonState[id] = reading;
    return false; // button released or button hasn't changed
}

// static float camAngle = 0;


static void stepDirection(direction_t direction) {
  HAL_GPIO_WritePin( S_DIR_GPIO_Port, S_DIR_Pin, (direction==cw) ? 0:1 );
}

static void stepSingle(void) {
  delayUs( STEP_SIZE );
  LL_GPIO_SetOutputPin( S_STEP_GPIO_Port, S_STEP_Pin );
  delayUs( STEP_SIZE );
  LL_GPIO_ResetOutputPin( S_STEP_GPIO_Port, S_STEP_Pin );
}

static int stepSize(uint8_t sz) {
  HAL_GPIO_WritePin( S_M0_GPIO_Port, S_M0_Pin, sz&0x01 );
  HAL_GPIO_WritePin( S_M1_GPIO_Port, S_M1_Pin, sz&0x02 );
  HAL_GPIO_WritePin( S_M2_GPIO_Port, S_M2_Pin, sz&0x04 );
  // 0=full, 1=1/2 step, 2=1/4 step, 3=1/8th step, 4=1/16th, 5,6,7=32th
  const int m[] = {1,2,4,8,16,32};
  return m[sz];
}

static void move(direction_t direction, int steps) {
  stepDirection( direction );

  // accelerate...
  for (int j=5; j>=0; j--) {
    int m = stepSize( j );
    for (int i=0; i<m*STEP_RAMP; i++) {
      stepSingle();
    }
  }

  // flat
  for (int i=STEP_RAMP*6*2; i<steps; i++) {
    stepSingle();
  }

  // decelerate...
  for (int j=0; j<=5; j++) {
    int m = stepSize( j );
    for (int i=0; i<m*STEP_RAMP; i++) {
      stepSingle();
    }
  }

  //camAngle += (steps * direction) / stepsPerCamDegree;
}

static void writeWilliams(int direction, int position) {
  static int ix = 0;
  // convert the motor position to something the factory williams game
  // thinks is the rotating cam and home opto input.
  // cw -> 
  // ccw <- 
  uint8_t values[] = { 0, // first floor 
                       1,1,1,1,1,1,1, // second floor
                       0,0,0,0,0,0,0, // thrid floor 
                       1,0,0,0,0,1,1,1,1, // second floor
                       0,0,0,0, 1,1,1 // first floor 
                     };
  ix += direction;
  if ( ix < 0 ) {
    ix = sizeof(values)-1;
  } else {
    if ( ix >= sizeof(values) ) {
      ix = 0;
    }
  }
  HAL_GPIO_WritePin( HOME_GPIO_Port, HOME_Pin, values[ix] );
}

static void readWilliams(void) {
  int enable = HAL_GPIO_ReadPin( EN_GPIO_Port, EN_Pin );
  int direction = HAL_GPIO_ReadPin( DIR_GPIO_Port, DIR_Pin );

  if ( enable ) {
      if ( direction ) {
        move( cw, 1000 );
      } else {
        move( ccw, 1000 );
      }
      // and while that's going, start sending writeWilliams values 
  }
}




int main(void)
{
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  SystemClock_Config();
  SysTick_Config(SystemCoreClock / 1000);
  MX_GPIO_Init();
  
  // NVIC_SetPriority(LIMIT_EXTI_IRQn, 1, 0);
  NVIC_EnableIRQ(LIMIT_EXTI_IRQn);
  // NVIC_SetPriority(S_NFLT_EXTI_IRQn, 1, 0);
  NVIC_EnableIRQ(S_NFLT_EXTI_IRQn);
  
  HAL_GPIO_WritePin( S_NEN_GPIO_Port, S_NEN_Pin, 0 );
  HAL_GPIO_WritePin( S_NRST_GPIO_Port, S_NRST_Pin, 1 );

  // move off the switch (we're probably on it)
  int m = stepSize( 4 );
  stepDirection( ccw );
  for (int i=0; i<stepsPerRotation*m; i++) {
    stepSingle();
  }

  // find the switch at slow speed
  stepSize( 4 );
  stepDirection( cw );
  int ct = 0;
  do {
    stepSingle();
    if ( HAL_GPIO_ReadPin(LIMIT_GPIO_Port,LIMIT_Pin) == 1 ) { ct++; } else { ct=0; }
  } while ( ct < 4 ); // debounce
    
  fault = false;
  
  while (1) {
    
    uint32_t tick = HAL_GetTick();
    static uint32_t lasttick = 0;

    if ( fault ) {
      if (tick-lasttick > 500) {
        lasttick = tick;
        HAL_GPIO_WritePin( S_NEN_GPIO_Port, S_NEN_Pin, ! HAL_GPIO_ReadPin(S_NEN_GPIO_Port,S_NEN_Pin) );
      }
    }
    
    if ( debouncedSwitch( left ) ) {
      // move up a bit
      move( cw, stepsPerLevel );
    }
    if ( debouncedSwitch( right ) ) {
      // move down a bit
      move( ccw, stepsPerLevel );
    }
      
  }
  
}




/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_1)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_4, LL_RCC_PLL_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }

  LL_Init1msTick(32000000);

  LL_SetSystemCoreClock(32000000);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  /**/
  LL_GPIO_ResetOutputPin(HOME_GPIO_Port, HOME_Pin);

  /**/
  LL_GPIO_ResetOutputPin(S_STEP_GPIO_Port, S_STEP_Pin);

  /**/
  LL_GPIO_ResetOutputPin(S_DIR_GPIO_Port, S_DIR_Pin);

  /**/
  LL_GPIO_ResetOutputPin(S_NEN_GPIO_Port, S_NEN_Pin);

  /**/
  LL_GPIO_ResetOutputPin(S_M0_GPIO_Port, S_M0_Pin);

  /**/
  LL_GPIO_ResetOutputPin(S_M1_GPIO_Port, S_M1_Pin);

  /**/
  LL_GPIO_ResetOutputPin(S_M2_GPIO_Port, S_M2_Pin);

  /**/
  LL_GPIO_ResetOutputPin(S_NRST_GPIO_Port, S_NRST_Pin);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE9);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);

  /**/
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_9, LL_GPIO_PULL_NO);

  /**/
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_0, LL_GPIO_PULL_NO);

  /**/
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_9, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_9;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_0;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  GPIO_InitStruct.Pin = SW0_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(SW0_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = SW1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(SW1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = HOME_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(HOME_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = S_STEP_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(S_STEP_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = S_DIR_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(S_DIR_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = S_NEN_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(S_NEN_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = S_M0_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(S_M0_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = S_M1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(S_M1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = S_M2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(S_M2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = EN_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(EN_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = S_NRST_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(S_NRST_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = DIR_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(DIR_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
