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
#define HAL_GPIO_WritePin(port,pin,value)   do { if(value) LL_GPIO_SetOutputPin(port,pin); else LL_GPIO_ResetOutputPin(port,pin); } while(0);
#define HAL_GetTick()                       (systick * 1000 / SystemTickUs)
#define HAL_Delay(ms)                       delayMs(ms)
#define delayMs(ms)                         LL_mDelay(((ms) * 1000) / SystemTickUs)
#define delayUs(us)                         LL_mDelay((us) / SystemTickUs)

#define SystemCoreClock                     2097000

// #define SystemTickUs                        1000U
#define SystemTickUs                        50

//#define rodInchesPerRotation                0.1
#define rodInchesPerRotation                0.3

#define stepsPerRotation                    200.0
#define stepsPerInch                        (stepsPerRotation / rodInchesPerRotation)
#define stepsPerLevel                       ((3.5/2.0) * stepsPerInch )
#define stepsPerCamDegree                   (stepsPerLevel / 90.0)


static volatile uint32_t systick = 0;
// called from stm32l0xx_it.c weak link ISR
void SysTick_Handler(void) {
  systick++;
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

const int slope = 50000;
const int ramp[] = { 
  12800,  // 78Hz
  6400,   // 156Hz
  3200,   // 312Hz
  1600,   // 625Hz
  800,    // 1.25kHz
  400,    // 2.5kHz
  200,    // 5kHz
  100,    // 10kHz
  50      // 20kHz
};

static float camAngle = 0;

static void move(direction_t direction, int steps) {
  static int position = 0;
  HAL_GPIO_WritePin( S_DIR_GPIO_Port, S_DIR_Pin, (direction==cw) ? 0:1 );
  int ix = 0;
  int iy = slope / ramp[ 0 ] ; 
  for (int i=0; i<steps; i++) {
    if ( iy-- == 0 ) {
      if ( ix < (sizeof(ramp)/sizeof(ramp[0]))-1 ) {
        iy = slope / ramp[ ix++ ];
      }
    }
    delayUs( ramp[ ix ] );
    HAL_GPIO_WritePin( S_STEP_GPIO_Port, S_STEP_Pin, 1 );
    delayUs( ramp[ ix ] );
    HAL_GPIO_WritePin( S_STEP_GPIO_Port, S_STEP_Pin, 0 );
  }
  camAngle += (steps * direction) / stepsPerCamDegree;
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



int main(void) {
  
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  SystemClock_Config();
  SysTick_Config(SystemCoreClock / 1000);
  MX_GPIO_Init();

  HAL_GPIO_WritePin( S_NEN_GPIO_Port, S_NEN_Pin, 0 );
  HAL_GPIO_WritePin( S_NRST_GPIO_Port, S_NRST_Pin, 1 );
  
  // full steps
  HAL_GPIO_WritePin( S_M0_GPIO_Port, S_M0_Pin, 0 );
  HAL_GPIO_WritePin( S_M1_GPIO_Port, S_M1_Pin, 0 );
  HAL_GPIO_WritePin( S_M2_GPIO_Port, S_M2_Pin, 0 );
  // 
  // // 1/2 step
  // HAL_GPIO_WritePin( S_M0_GPIO_Port, S_M0_Pin, 1 );
  // HAL_GPIO_WritePin( S_M1_GPIO_Port, S_M1_Pin, 0 );
  // HAL_GPIO_WritePin( S_M2_GPIO_Port, S_M2_Pin, 0 );
  // 
  // // 1/4 step
  // HAL_GPIO_WritePin( S_M0_GPIO_Port, S_M0_Pin, 0 );
  // HAL_GPIO_WritePin( S_M1_GPIO_Port, S_M1_Pin, 1 );
  // HAL_GPIO_WritePin( S_M2_GPIO_Port, S_M2_Pin, 0 );
  // 
  // // 1/8 step
  // HAL_GPIO_WritePin( S_M0_GPIO_Port, S_M0_Pin, 1 );
  // HAL_GPIO_WritePin( S_M1_GPIO_Port, S_M1_Pin, 1 );
  // HAL_GPIO_WritePin( S_M2_GPIO_Port, S_M2_Pin, 0 );
  // 
  // // 1/16 step
  // HAL_GPIO_WritePin( S_M0_GPIO_Port, S_M0_Pin, 0 );
  // HAL_GPIO_WritePin( S_M1_GPIO_Port, S_M1_Pin, 0 );
  // HAL_GPIO_WritePin( S_M2_GPIO_Port, S_M2_Pin, 1 );
  // 
  // // 1/32 step
  // HAL_GPIO_WritePin( S_M0_GPIO_Port, S_M0_Pin, 0 );
  // HAL_GPIO_WritePin( S_M1_GPIO_Port, S_M1_Pin, 1 );
  // HAL_GPIO_WritePin( S_M2_GPIO_Port, S_M2_Pin, 1 );


  // // home stepper
  // HAL_GPIO_WritePin( S_DIR_GPIO_Port, S_DIR_Pin, 0 );
  // do {
  //     HAL_GPIO_WritePin( S_STEP_GPIO_Port, S_STEP_Pin, 1 );
  //     HAL_Delay(50);
  //     HAL_GPIO_WritePin( S_STEP_GPIO_Port, S_STEP_Pin, 0 );
  //     HAL_Delay(50);
  // } while ( HAL_GPIO_ReadPin( LIMIT_GPIO_Port, LIMIT_Pin ) == 1 );
  // 

  while (1) {
    
    uint32_t tick = HAL_GetTick();
    static uint32_t lasttick = 0;
    
    if (tick-lasttick > 500) {
      lasttick = tick;
      //HAL_GPIO_WritePin( S_NEN_GPIO_Port, S_NEN_Pin, ! HAL_GPIO_ReadPin(S_NEN_GPIO_Port,S_NEN_Pin) );
    }
      
    // if ( HAL_GPIO_ReadPin( S_NFLT_GPIO_Port, S_NFLT_Pin ) ) {
    //   printf("stepper driver fault detected!");
    //   HAL_GPIO_WritePin( S_NEN_GPIO_Port, S_NEN_Pin, 1 );
    //   for (;;) { }
    // }

    
    // 
    // if ( HAL_GPIO_ReadPin( S_NFLT_GPIO_Port, S_NFLT_Pin ) ) {
    //   printf("stepper driver fault detected!");
    //   HAL_GPIO_WritePin( S_NEN_GPIO_Port, S_NEN_Pin, 1 );
    //   for (;;) { }
    // }
    // 
    // 
    // if ( debouncedSwitch( left ) && debouncedSwitch( right ) ) {
    //   // record position
    // } else {
    // 
      if ( debouncedSwitch( left ) ) {
          // move up a bit
          move( cw, stepsPerInch*1.0 );
          //HAL_GPIO_WritePin( S_NEN_GPIO_Port, S_NEN_Pin, 0);
      }
    // 
      if ( debouncedSwitch( right ) ) {
          // move down a bit
          move( ccw, stepsPerInch*1.0 );
          //HAL_GPIO_WritePin( S_NEN_GPIO_Port, S_NEN_Pin, 1);
      }
    // 
    // }

    
  }
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_MSI_Enable();

   /* Wait till MSI is ready */
  while(LL_RCC_MSI_IsReady() != 1)
  {

  }
  LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_5);
  LL_RCC_MSI_SetCalibTrimming(0);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_MSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_MSI)
  {

  }

  LL_InitTick(SystemCoreClock, (uint32_t)SystemTickUs);

  LL_SetSystemCoreClock(SystemCoreClock);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
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
  GPIO_InitStruct.Pin = LIMIT_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(LIMIT_GPIO_Port, &GPIO_InitStruct);

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
  GPIO_InitStruct.Pin = S_NFLT_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(S_NFLT_GPIO_Port, &GPIO_InitStruct);

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
