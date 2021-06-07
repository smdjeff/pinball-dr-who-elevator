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
#define HAL_GPIO_TogglePin(port,pin)        do { HAL_GPIO_WritePin(port, pin, !HAL_GPIO_ReadPin(port,pin) ); } while(0)

#define HAL_GetTick()                       (systick)
// tuned at 10s, assumes 32MHz sysclock
#define delayUs(us)                         for (int i=0;i<us*6;i++){ asm("nop"); }
#define delayMs(ms)                         do { delayUs((ms)*1000); } while(0)
#define SECONDS_TO_TICKS(s)                 ((s)*1000)
#define MINUTES_TO_TICKS(s)                 (SECONDS_TO_TICKS(s)*60)

// tuned on 6/1/2021
/*
stepsPerRotation:200
stepsPerInch:632.911392
stepsPerLevel:1107.594937
stepsPerCamDegree:12.306610
stepsPerPercentOfLevel: 11.07594937
*/
#define rodInchesPerRotation                0.316
#define stepsPerRotation                    200
#define stepsPerInch                        (stepsPerRotation / rodInchesPerRotation) 
#define stepsPerLevel                       ((3.5/2) * stepsPerInch )
#define stepsPerCamDegree                   (stepsPerLevel / 90)
#define stepsPerPercentOfLevel              (stepsPerLevel / 100)

#define STEP_RAMP                           4
#define STEP_SIZE                           500



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
    but_left    = 0,
    but_right   = 1,
    but_limit   = 2
} button_t;

typedef enum {
    step_enable = 0,
    step_disable = 1
} step_enabled_t;

typedef enum {
    step_assert = 0,
    step_deassert = 1
} step_reset_t;

typedef enum {
    step_dir_up = 0,
    step_dir_down = 1
} step_dir_t;

typedef enum {
    step_size_full  = 0,
    step_size_half  = 1,
    step_size_4th   = 2,
    step_size_8th   = 3,
    step_size_16th  = 4,
    step_size_32nd  = 5
}  step_size_t;
 
typedef enum {
    motor_dir_cw = 1,
    motor_dir_ccw = 0
} motor_dir_t;

typedef enum {
    motor_enable = 0,
    motor_disable = 1
} motor_enable_t;

typedef enum {
    opto_open = 0,
    opto_closed = 1
} opto_t;

typedef enum {
    limit_off = 0,
    limit_hit = 1,
} limit_t;

typedef enum {
    level_down  = 0,
    level_mid_r = 1,
    level_up    = 2,
    level_mid_l = 3,
} level_t;

static int step_percentage = 0;
static int step_percentage_toggle = 0;
static level_t current_level = 0;


static uint8_t readPin(button_t id) {
  switch ( id ) {
    case but_left:
      return HAL_GPIO_ReadPin( SW0_GPIO_Port, SW0_Pin ); 
    case but_right: 
      return HAL_GPIO_ReadPin( SW1_GPIO_Port, SW1_Pin ); 
    case but_limit: 
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
    uint32_t debounceDelay = buttonState[id] ? 25 : 50;
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


static void stepSingle(void) {
  delayUs( STEP_SIZE );
  LL_GPIO_SetOutputPin( S_STEP_GPIO_Port, S_STEP_Pin );
  delayUs( STEP_SIZE );
  LL_GPIO_ResetOutputPin( S_STEP_GPIO_Port, S_STEP_Pin );
  
  static int steps = 0;
  if ( steps++ >= stepsPerPercentOfLevel ) {
    steps = 0;
    step_percentage++;
    if ( step_percentage >= step_percentage_toggle ) {
      step_percentage_toggle = 999;
      HAL_GPIO_TogglePin( HOME_GPIO_Port, HOME_Pin );
    }
  }
}

static int stepSize(step_size_t sz) {
  HAL_GPIO_WritePin( S_M0_GPIO_Port, S_M0_Pin, sz&0x01 );
  HAL_GPIO_WritePin( S_M1_GPIO_Port, S_M1_Pin, sz&0x02 );
  HAL_GPIO_WritePin( S_M2_GPIO_Port, S_M2_Pin, sz&0x04 );
  // 0=full, 1=1/2 step, 2=1/4 step, 3=1/8th step, 4=1/16th, 5,6,7=32th
  const int m[] = {1,2,4,8,16,32};
  return m[sz];
}

static void move(step_dir_t dir, int steps, int opto_toggle) {
  step_percentage = 0;
  step_percentage_toggle = opto_toggle;

  HAL_GPIO_WritePin( S_NEN_GPIO_Port, S_NEN_Pin, step_enable );
  HAL_GPIO_WritePin( S_DIR_GPIO_Port, S_DIR_Pin, dir );
  delayUs(10);
  
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
  
  HAL_GPIO_WritePin( S_NEN_GPIO_Port, S_NEN_Pin, step_enable );
  HAL_GPIO_WritePin( S_NRST_GPIO_Port, S_NRST_Pin, step_deassert );

  // move off the switch
  if ( HAL_GPIO_ReadPin(LIMIT_GPIO_Port,LIMIT_Pin) == limit_hit ) {
    int m = stepSize( step_size_4th );
    HAL_GPIO_WritePin( S_DIR_GPIO_Port, S_DIR_Pin, step_dir_up );
    for (int i=0; i<stepsPerRotation*m; i++) {
      stepSingle();
    }
  }

  // find the switch at slow speed
  stepSize( step_size_8th );
  HAL_GPIO_WritePin( S_DIR_GPIO_Port, S_DIR_Pin, step_dir_down );
  int ct = 0;
  do {
    stepSingle();
    if ( HAL_GPIO_ReadPin(LIMIT_GPIO_Port,LIMIT_Pin) == limit_hit ) { ct++; } else { ct=0; }
  } while ( ct < 2 ); // debounce
  
  // once we zero the stepper, we can allow the machine to "home the cam"
  // machine will try to home the to cam CW down, where opto is open at complete
  // just to the right of the little nub on the cam
  current_level = level_down;
  HAL_GPIO_WritePin( HOME_GPIO_Port, HOME_Pin, opto_open );
  
  fault = false;
  
  while (1) {
    
    uint32_t tick = HAL_GetTick();
    static uint32_t inactivityTimer = 0;

    // note: wpc89 is 6809 @2MHz, 2 instructions per 1us
    motor_enable_t enable = HAL_GPIO_ReadPin( EN_GPIO_Port, EN_Pin );
    delayUs(10);

    if ( enable == motor_enable ) {
      inactivityTimer = tick;

      motor_dir_t direction = HAL_GPIO_ReadPin( DIR_GPIO_Port, DIR_Pin );
      
    // if ( debouncedSwitch( but_left ) ) {
    //   enable = true;
    //   direction = motor_dir_ccw;
    // }
    // if ( debouncedSwitch( but_right ) ) {
    //   enable = true;
    //   direction = motor_dir_cw;
    // }
    
        static motor_dir_t last_direction = motor_dir_cw;
        if ( direction != last_direction ) {
          HAL_GPIO_TogglePin( HOME_GPIO_Port, HOME_Pin );
          last_direction = direction;
        }
        
        if ( direction == motor_dir_ccw ) {
    
          // adjustments to percentages made to account for delays in moving
          // a timer would probably work better than a percentage of the step motion
          
          switch ( current_level ) {
            case level_down:
              move( step_dir_up, stepsPerLevel, 5/*perc of move to open opto*/);
              current_level = level_mid_r;
              break;
            case level_mid_r:
              move( step_dir_up, stepsPerLevel, 98/*perc of move to open opto*/);
              current_level = level_up;
              break;
            case level_mid_l:
              move( step_dir_down, stepsPerLevel, 66/*perc of move to open opto*/);
              current_level = level_down;
              break;
            case level_up:
              move( step_dir_down, stepsPerLevel, 45/*perc of move to open opto*/);
              current_level = level_mid_l;
              break;
          }
          
        } else {

          switch ( current_level ) {
            case level_down:
              move( step_dir_up, stepsPerLevel, 33/*perc of move to close opto*/);
              current_level = level_mid_l;
              break;
            case level_mid_l:
              move( step_dir_up, stepsPerLevel, 85/*perc of move to close opto*/);
              current_level = level_up;
              break;
            case level_up:
              move( step_dir_down, stepsPerLevel, 5/*perc of move to close opto*/);
              current_level = level_mid_r;
              break;
            case level_mid_r:
              move( step_dir_down, stepsPerLevel, 98/*perc of move to close opto*/);
              current_level = level_down;
              break;
          }
        }

        // signal complete to the wpc89
        HAL_GPIO_TogglePin( HOME_GPIO_Port, HOME_Pin );
        
        // stall a bit for the williams cpu to see that we completed the move
        // allowing it to properly decide to disable or enable the dc motor enable signal
        delayMs( 10 );
          
    }

    if ( fault || tick-inactivityTimer > MINUTES_TO_TICKS(1)) {
      inactivityTimer = tick;
      HAL_GPIO_WritePin( S_NEN_GPIO_Port, S_NEN_Pin, step_disable );
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
