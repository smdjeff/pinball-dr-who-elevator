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


const int stepsPerLevel = 1000;

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

static int move(direction_t direction, int steps) {
  static int position = 0;
  HAL_GPIO_WritePin( S_DIR_GPIO_Port, S_DIR_Pin, (direction==cw) ? 0:1 );
  for (int i=0; i<steps; i++) {
    HAL_GPIO_WritePin( S_STEP_GPIO_Port, S_STEP_Pin, 1 );
    HAL_Delay(50);
    HAL_GPIO_WritePin( S_STEP_GPIO_Port, S_STEP_Pin, 0 );
    HAL_Delay(50);
  }
  position += steps * direction;
  return position;
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
  
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  HAL_GPIO_WritePin( S_NEN_GPIO_Port, S_NEN_Pin, 0 );
  HAL_GPIO_WritePin( S_NRST_GPIO_Port, S_NRST_Pin, 0 );
  
  // full steps
  HAL_GPIO_WritePin( S_M0_GPIO_Port, S_M0_Pin, 0 );
  HAL_GPIO_WritePin( S_M1_GPIO_Port, S_M1_Pin, 0 );
  HAL_GPIO_WritePin( S_M2_GPIO_Port, S_M2_Pin, 0 );


  // home stepper
  HAL_GPIO_WritePin( S_DIR_GPIO_Port, S_DIR_Pin, 0 );
  do {
      HAL_GPIO_WritePin( S_STEP_GPIO_Port, S_STEP_Pin, 1 );
      HAL_Delay(50);
      HAL_GPIO_WritePin( S_STEP_GPIO_Port, S_STEP_Pin, 0 );
      HAL_Delay(50);
  } while ( HAL_GPIO_ReadPin( LIMIT_GPIO_Port, LIMIT_Pin ) == 1 );


  while (1) {
    
    uint32_t tick = HAL_GetTick();
    
    if ( HAL_GPIO_ReadPin( S_NFLT_GPIO_Port, S_NFLT_Pin ) ) {
      printf("stepper driver fault detected!");
      HAL_GPIO_WritePin( S_NEN_GPIO_Port, S_NEN_Pin, 1 );
      for (;;) { }
    }
      
    
    if ( debouncedSwitch( left ) && debouncedSwitch( right ) ) {
      // record position
    } else {

      if ( debouncedSwitch( left ) ) {
          // move up a bit
      }
  
      if ( debouncedSwitch( right ) ) {
          // move down a bit
      }
      
    }

    
  }
}




/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, HOME_Pin|S_STEP_Pin|S_DIR_Pin|S_NEN_Pin
                          |S_M0_Pin|S_M1_Pin|S_M2_Pin|S_NRST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LIMIT_Pin */
  GPIO_InitStruct.Pin = LIMIT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LIMIT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SW0_Pin SW1_Pin */
  GPIO_InitStruct.Pin = SW0_Pin|SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : S_NFLT_Pin DIR_Pin */
  GPIO_InitStruct.Pin = S_NFLT_Pin|DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : HOME_Pin S_STEP_Pin S_DIR_Pin S_NEN_Pin
                           S_M0_Pin S_M1_Pin S_M2_Pin S_NRST_Pin */
  GPIO_InitStruct.Pin = HOME_Pin|S_STEP_Pin|S_DIR_Pin|S_NEN_Pin
                          |S_M0_Pin|S_M1_Pin|S_M2_Pin|S_NRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : EN_Pin */
  GPIO_InitStruct.Pin = EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EN_GPIO_Port, &GPIO_InitStruct);

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
