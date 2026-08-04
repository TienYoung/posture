
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_azure_rtos.c
  * @author  MCD Application Team
  * @brief   azure_rtos application implementation file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "app_azure_rtos.h"
#include "app_azure_rtos_config.h"
#include "main.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_gpio.h"
#include "tx_api.h"
#include "tx_port.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f3discovery_bus.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* STM32F3-Discovery LSM303DLHC/LSM303AGR accelerometer (SA0 pulled high). */
#define LEVEL_ACC_I2C_ADDRESS        (0x19U << 1)
#define LEVEL_ACC_CTRL_REG1_A        0x20U
#define LEVEL_ACC_CTRL_REG4_A        0x23U
#define LEVEL_ACC_OUT_X_L_A          0x28U
#define LEVEL_ACC_AUTO_INCREMENT     0x80U

#define LEVEL_SAMPLE_TICKS           (TX_TIMER_TICKS_PER_SECOND / 20U)
#define LEVEL_ERROR_BLINK_TICKS      (TX_TIMER_TICKS_PER_SECOND / 5U)

/* tan(3 degrees) ~= 0.052; tan(4 degrees) ~= 0.070. */
#define LEVEL_ENTER_RATIO_PER_1000   52L
#define LEVEL_EXIT_RATIO_PER_1000    70L
#define LEVEL_MIN_VERTICAL_MG        700L
#define LEVEL_MAX_VERTICAL_MG        1300L
#define LEVEL_DIAGONAL_RATIO_PER_1000 414L
#define LEVEL_ANIMATION_SAMPLES      10U

#define LEVEL_RED_LEDS               (LD3_Pin | LD10_Pin)
#define LEVEL_ALL_LEDS               (LD3_Pin | LD4_Pin | LD5_Pin | LD6_Pin | \
                                      LD7_Pin | LD8_Pin | LD9_Pin | LD10_Pin)
#define LEVEL_NON_RED_LEDS           (LEVEL_ALL_LEDS & ~LEVEL_RED_LEDS)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN TX_Pool_Buffer */
static UCHAR tx_byte_pool_buffer[TX_APP_MEM_POOL_SIZE];
static TX_BYTE_POOL tx_app_byte_pool;
/* USER CODE END TX_Pool_Buffer */

/* USER CODE BEGIN thread_0 */
TX_THREAD thread_0;
static int32_t filtered_acc_mg[3];
static uint8_t filter_initialized;
/* USER CODE END thread_0 */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static int32_t LevelSensor_Init(void);
static int32_t LevelSensor_Read(int32_t acceleration_mg[3]);
static void LevelLeds_Set(uint16_t leds);
static uint8_t LevelDetector_Update(const int32_t acceleration_mg[3], uint8_t was_level);
static uint16_t LevelDirection_GetLed(void);
VOID level_thread(ULONG thread_input);
/* USER CODE END PFP */

/**
  * @brief  Define the initial system.
  * @param  first_unused_memory : Pointer to the first unused memory
  * @retval None
  */
VOID tx_application_define(VOID *first_unused_memory)
{
  /* USER CODE BEGIN  tx_application_define */

  /* USER CODE END  tx_application_define */

  VOID *memory_ptr;
  CHAR *thread_ptr = TX_NULL;

  if (tx_byte_pool_create(&tx_app_byte_pool, "Tx App memory pool", tx_byte_pool_buffer, TX_APP_MEM_POOL_SIZE) != TX_SUCCESS)
  {
    /* USER CODE BEGIN TX_Byte_Pool_Error */

    /* USER CODE END TX_Byte_Pool_Error */
  }
  else
  {
    /* USER CODE BEGIN TX_Byte_Pool_Success */

    /* USER CODE END TX_Byte_Pool_Success */

    memory_ptr = (VOID *)&tx_app_byte_pool;

    if (App_ThreadX_Init(memory_ptr) != TX_SUCCESS)
    {
      /* USER CODE BEGIN  App_ThreadX_Init_Error */

      /* USER CODE END  App_ThreadX_Init_Error */
    }

    /* USER CODE BEGIN  App_ThreadX_Init_Success */
    UINT result = tx_byte_allocate(&tx_app_byte_pool, (VOID **) &thread_ptr, TX_APP_STACK_SIZE, TX_NO_WAIT);
    if(result == TX_SUCCESS)
    {
      result = tx_thread_create(&thread_0, "level detector", level_thread, 0, thread_ptr, TX_APP_STACK_SIZE, 1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
      if(result == TX_SUCCESS)
      {

      }
    }

    /* USER CODE END  App_ThreadX_Init_Success */

  }

}

/* USER CODE BEGIN  0 */
static int32_t LevelSensor_Init(void)
{
  uint8_t value;

  if (BSP_I2C1_Init() != BSP_ERROR_NONE)
  {
    return BSP_ERROR_BUS_FAILURE;
  }

  /* 100 Hz, normal power mode, X/Y/Z axes enabled. */
  value = 0x57U;
  if (BSP_I2C1_WriteReg(LEVEL_ACC_I2C_ADDRESS, LEVEL_ACC_CTRL_REG1_A,
                        &value, 1U) != BSP_ERROR_NONE)
  {
    return BSP_ERROR_COMPONENT_FAILURE;
  }

  /* Block-data update, high-resolution mode, +/-2 g full scale. */
  value = 0x88U;
  if (BSP_I2C1_WriteReg(LEVEL_ACC_I2C_ADDRESS, LEVEL_ACC_CTRL_REG4_A,
                        &value, 1U) != BSP_ERROR_NONE)
  {
    return BSP_ERROR_COMPONENT_FAILURE;
  }

  tx_thread_sleep(LEVEL_SAMPLE_TICKS);
  return BSP_ERROR_NONE;
}

static int32_t LevelSensor_Read(int32_t acceleration_mg[3])
{
  uint8_t data[6];
  int16_t raw;
  uint32_t axis;

  if (BSP_I2C1_ReadReg(LEVEL_ACC_I2C_ADDRESS,
                       LEVEL_ACC_OUT_X_L_A | LEVEL_ACC_AUTO_INCREMENT,
                       data, sizeof(data)) != BSP_ERROR_NONE)
  {
    return BSP_ERROR_COMPONENT_FAILURE;
  }

  for (axis = 0U; axis < 3U; axis++)
  {
    raw = (int16_t)((uint16_t)data[(axis * 2U) + 1U] << 8U |
                    data[axis * 2U]);

    /* In high-resolution +/-2 g mode the 12-bit result is 1 mg/LSB. */
    acceleration_mg[axis] = (int32_t)(raw >> 4);
  }

  return BSP_ERROR_NONE;
}

static void LevelLeds_Set(uint16_t leds)
{
  HAL_GPIO_WritePin(GPIOE, LEVEL_ALL_LEDS, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, leds, GPIO_PIN_SET);
}

static uint8_t LevelDetector_Update(const int32_t acceleration_mg[3], uint8_t was_level)
{
  int32_t abs_x;
  int32_t abs_y;
  int32_t abs_z;
  int32_t threshold_ratio;
  uint32_t axis;

  if (filter_initialized == 0U)
  {
    for (axis = 0U; axis < 3U; axis++)
    {
      filtered_acc_mg[axis] = acceleration_mg[axis];
    }
    filter_initialized = 1U;
  }
  else
  {
    for (axis = 0U; axis < 3U; axis++)
    {
      filtered_acc_mg[axis] += (acceleration_mg[axis] - filtered_acc_mg[axis]) / 8L;
    }
  }

  abs_x = (filtered_acc_mg[0] < 0L) ? -filtered_acc_mg[0] : filtered_acc_mg[0];
  abs_y = (filtered_acc_mg[1] < 0L) ? -filtered_acc_mg[1] : filtered_acc_mg[1];
  abs_z = (filtered_acc_mg[2] < 0L) ? -filtered_acc_mg[2] : filtered_acc_mg[2];

  if ((abs_z < LEVEL_MIN_VERTICAL_MG) || (abs_z > LEVEL_MAX_VERTICAL_MG))
  {
    return 0U;
  }

  threshold_ratio = (was_level != 0U) ? LEVEL_EXIT_RATIO_PER_1000
                                      : LEVEL_ENTER_RATIO_PER_1000;

  return (((abs_x * 1000L) <= (abs_z * threshold_ratio)) &&
          ((abs_y * 1000L) <= (abs_z * threshold_ratio))) ? 1U : 0U;
}

static uint16_t LevelDirection_GetLed(void)
{
  int32_t x = filtered_acc_mg[0];
  int32_t y = filtered_acc_mg[1];
  int32_t abs_x = (x < 0L) ? -x : x;
  int32_t abs_y = (y < 0L) ? -y : y;

  /* Split the X/Y gravity projection into eight 45-degree sectors. */
  if ((abs_x * 1000L) <= (abs_y * LEVEL_DIAGONAL_RATIO_PER_1000))
  {
    return (y < 0L) ? LD3_Pin : LD10_Pin;  /* North / South */
  }

  if ((abs_y * 1000L) <= (abs_x * LEVEL_DIAGONAL_RATIO_PER_1000))
  {
    return (x > 0L) ? LD7_Pin : LD6_Pin;   /* East / West */
  }

  if (x > 0L)
  {
    return (y < 0L) ? LD5_Pin : LD9_Pin;   /* North-East / South-East */
  }

  return (y < 0L) ? LD4_Pin : LD8_Pin;     /* North-West / South-West */
}

VOID level_thread(ULONG thread_input)
{
  int32_t acceleration_mg[3];
  uint8_t is_level = 0U;
  uint8_t level_animation_on = 1U;
  uint8_t level_animation_samples = 0U;

  (void)thread_input;
  LevelLeds_Set(LEVEL_ALL_LEDS);

  if (LevelSensor_Init() != BSP_ERROR_NONE)
  {
    LevelLeds_Set(0U);
    while (1)
    {
      HAL_GPIO_TogglePin(GPIOE, LEVEL_RED_LEDS);
      tx_thread_sleep(LEVEL_ERROR_BLINK_TICKS);
    }
  }

  while (1)
  {
    if (LevelSensor_Read(acceleration_mg) == BSP_ERROR_NONE)
    {
      is_level = LevelDetector_Update(acceleration_mg, is_level);

      if (is_level != 0U)
      {
        level_animation_samples++;
        if (level_animation_samples >= LEVEL_ANIMATION_SAMPLES)
        {
          level_animation_samples = 0U;
          level_animation_on = (level_animation_on == 0U) ? 1U : 0U;
        }
        LevelLeds_Set((level_animation_on != 0U) ? LEVEL_ALL_LEDS : 0U);
      }
      else
      {
        level_animation_samples = 0U;
        level_animation_on = 1U;
        LevelLeds_Set(LevelDirection_GetLed());
      }
    }
    else
    {
      HAL_GPIO_WritePin(GPIOE, LEVEL_NON_RED_LEDS, GPIO_PIN_RESET);
      HAL_GPIO_TogglePin(GPIOE, LEVEL_RED_LEDS);
    }

    tx_thread_sleep(LEVEL_SAMPLE_TICKS);
  }
}
/* USER CODE END  0 */
