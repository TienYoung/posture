
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
#include "custom_motion_sensors.h"
#include "custom_motion_sensors_ex.h"
#include <stddef.h>
#include "motion_fx.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FUSION_SAMPLE_HZ             100U
#define FUSION_SAMPLE_PERIOD_MS      (1000U / FUSION_SAMPLE_HZ)
#define FUSION_SAMPLE_TICKS          (TX_TIMER_TICKS_PER_SECOND / FUSION_SAMPLE_HZ)
#define FUSION_DRDY_TIMEOUT_TICKS    (TX_TIMER_TICKS_PER_SECOND / 2U)
#define FUSION_EVENT_GYRO_DRDY       (1UL << 0)
#define FUSION_UART_DECIMATION       1U
#define FUSION_STATE_SIZE_BYTES      2432U
#define FUSION_ACC_INSTANCE          CUSTOM_LSM303AGR_ACC_0
#define FUSION_MAG_INSTANCE          CUSTOM_LSM303AGR_MAG_0
#define FUSION_GYRO_INSTANCE         CUSTOM_A3G4250D_0
#define FUSION_SENSOR_ODR_HZ         100.0f
#define FUSION_ACC_FULL_SCALE_G      2
#define FUSION_GYRO_FULL_SCALE_DPS   245
#define FUSION_MG_TO_G               0.001f
#define FUSION_MDPS_TO_DPS           0.001f
#define FUSION_MGAUSS_TO_UT50        0.002f
#define FUSION_DEFAULT_DELTA_S       0.010f
#define FUSION_MIN_DELTA_S           0.001f
#define FUSION_MAX_DELTA_S           0.050f

/* Raw positive axes expressed in the board frame: east, south and up. */
#define FUSION_ACC_ORIENTATION       "esu"
#define FUSION_GYRO_ORIENTATION      "esu"
#define FUSION_MAG_ORIENTATION       "esu"

#define FUSION_GBIAS_ACC_TH_SC       (2.0f * 0.000765f)
#define FUSION_GBIAS_GYRO_TH_SC      (2.0f * 0.002000f)
#define FUSION_GBIAS_MAG_TH_SC       (2.0f * 0.001500f)

#define LEVEL_ERROR_BLINK_TICKS      (TX_TIMER_TICKS_PER_SECOND / 5U)
#define LEVEL_UART_BUFFER_SIZE       128U

/* tan(3 degrees) ~= 0.052; tan(4 degrees) ~= 0.070. */
#define LEVEL_ENTER_RATIO_PER_1000   52L
#define LEVEL_EXIT_RATIO_PER_1000    70L
#define LEVEL_MIN_VERTICAL_MG        700L
#define LEVEL_MAX_VERTICAL_MG        1300L
#define LEVEL_DIAGONAL_RATIO_PER_1000 414L
#define LEVEL_ANIMATION_SAMPLES      50U

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

/* USER CODE BEGIN PV */
TX_THREAD thread_0;
static TX_EVENT_FLAGS_GROUP fusion_events;
/* MotionFX contains double-word accesses, so its opaque state must be 8-byte aligned. */
static uint64_t motionfx_state[(FUSION_STATE_SIZE_BYTES + sizeof(uint64_t) - 1U) / sizeof(uint64_t)];
static MFX_MagCal_quality_t mag_cal_quality = MFX_MAGCALUNKNOWN;
static uint8_t uart_dma_message[LEVEL_UART_BUFFER_SIZE];
static uint32_t uart_sample_sequence;
static volatile uint32_t gyro_drdy_irq_count;
static volatile uint8_t fusion_events_ready;
extern UART_HandleTypeDef huart1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static int32_t FusionSensors_Init(void);
static int32_t FusionGyroDataReady_Enable(void);
static int32_t FusionSensors_Read(MFX_input_t *input, uint32_t timestamp_ms);
static int32_t FusionEngine_Init(void);
static void FusionEngine_Run(MFX_input_t *input, MFX_output_t *output, float delta_time_s);
static void FusionUart_PrintQuaternion(const MFX_output_t *output, uint32_t drdy_irq_count);
static int32_t FusionFloatToMicro(float value);
static void LevelLeds_Set(uint16_t leds);
static uint8_t LevelDetector_Update(const float gravity[3], uint8_t was_level);
static uint16_t LevelDirection_GetLed(const float gravity[3]);
char MotionFX_LoadMagCalFromNVM(unsigned short int data_size, unsigned int *data);
char MotionFX_SaveMagCalInNVM(unsigned short int data_size, unsigned int *data);
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
    UINT result = tx_event_flags_create(&fusion_events, "fusion events");
    if (result == TX_SUCCESS)
    {
      fusion_events_ready = 1U;
      result = tx_byte_allocate(&tx_app_byte_pool, (VOID **) &thread_ptr, TX_APP_STACK_SIZE, TX_NO_WAIT);
    }
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
static int32_t FusionSensors_Init(void)
{
  if ((CUSTOM_MOTION_SENSOR_Init(FUSION_ACC_INSTANCE, MOTION_ACCELERO) != BSP_ERROR_NONE) ||
      (CUSTOM_MOTION_SENSOR_Init(FUSION_MAG_INSTANCE, MOTION_MAGNETO) != BSP_ERROR_NONE) ||
      (CUSTOM_MOTION_SENSOR_Init(FUSION_GYRO_INSTANCE, MOTION_GYRO) != BSP_ERROR_NONE))
  {
    return BSP_ERROR_COMPONENT_FAILURE;
  }

  if ((CUSTOM_MOTION_SENSOR_SetOutputDataRate(FUSION_ACC_INSTANCE, MOTION_ACCELERO,
                                               FUSION_SENSOR_ODR_HZ) != BSP_ERROR_NONE) ||
      (CUSTOM_MOTION_SENSOR_SetOutputDataRate(FUSION_MAG_INSTANCE, MOTION_MAGNETO,
                                               FUSION_SENSOR_ODR_HZ) != BSP_ERROR_NONE) ||
      (CUSTOM_MOTION_SENSOR_SetOutputDataRate(FUSION_GYRO_INSTANCE, MOTION_GYRO,
                                               FUSION_SENSOR_ODR_HZ) != BSP_ERROR_NONE))
  {
    return BSP_ERROR_COMPONENT_FAILURE;
  }

  if ((CUSTOM_MOTION_SENSOR_SetFullScale(FUSION_ACC_INSTANCE, MOTION_ACCELERO,
                                          FUSION_ACC_FULL_SCALE_G) != BSP_ERROR_NONE) ||
      (CUSTOM_MOTION_SENSOR_SetFullScale(FUSION_GYRO_INSTANCE, MOTION_GYRO,
                                          FUSION_GYRO_FULL_SCALE_DPS) != BSP_ERROR_NONE))
  {
    return BSP_ERROR_COMPONENT_FAILURE;
  }

  tx_thread_sleep(FUSION_SAMPLE_TICKS);
  return BSP_ERROR_NONE;
}

static int32_t FusionGyroDataReady_Enable(void)
{
  a3g4250d_ctrl_reg3_t interrupt_route = {0};

  if (CUSTOM_MOTION_SENSOR_Read_Register(FUSION_GYRO_INSTANCE, A3G4250D_CTRL_REG3,
                                         (uint8_t *)&interrupt_route) != BSP_ERROR_NONE)
  {
    return BSP_ERROR_COMPONENT_FAILURE;
  }

  interrupt_route.i2_drdy = 1U;
  __HAL_GPIO_EXTI_CLEAR_IT(MEMS_INT2_Pin);

  if (CUSTOM_MOTION_SENSOR_Write_Register(FUSION_GYRO_INSTANCE, A3G4250D_CTRL_REG3,
                                          *((uint8_t *)&interrupt_route)) != BSP_ERROR_NONE)
  {
    return BSP_ERROR_COMPONENT_FAILURE;
  }

  return BSP_ERROR_NONE;
}

static int32_t FusionSensors_Read(MFX_input_t *input, uint32_t timestamp_ms)
{
  CUSTOM_MOTION_SENSOR_Axes_t acc;
  CUSTOM_MOTION_SENSOR_Axes_t gyro;
  CUSTOM_MOTION_SENSOR_Axes_t mag;
  MFX_MagCal_input_t mag_cal_input;
  MFX_MagCal_output_t mag_cal_output;
  uint32_t axis;

  if ((CUSTOM_MOTION_SENSOR_GetAxes(FUSION_ACC_INSTANCE, MOTION_ACCELERO, &acc) != BSP_ERROR_NONE) ||
      (CUSTOM_MOTION_SENSOR_GetAxes(FUSION_GYRO_INSTANCE, MOTION_GYRO, &gyro) != BSP_ERROR_NONE) ||
      (CUSTOM_MOTION_SENSOR_GetAxes(FUSION_MAG_INSTANCE, MOTION_MAGNETO, &mag) != BSP_ERROR_NONE))
  {
    return BSP_ERROR_COMPONENT_FAILURE;
  }

  input->acc[0] = (float)acc.x * FUSION_MG_TO_G;
  input->acc[1] = (float)acc.y * FUSION_MG_TO_G;
  input->acc[2] = (float)acc.z * FUSION_MG_TO_G;

  input->gyro[0] = (float)gyro.x * FUSION_MDPS_TO_DPS;
  input->gyro[1] = (float)gyro.y * FUSION_MDPS_TO_DPS;
  input->gyro[2] = (float)gyro.z * FUSION_MDPS_TO_DPS;

  mag_cal_input.mag[0] = (float)mag.x * FUSION_MGAUSS_TO_UT50;
  mag_cal_input.mag[1] = (float)mag.y * FUSION_MGAUSS_TO_UT50;
  mag_cal_input.mag[2] = (float)mag.z * FUSION_MGAUSS_TO_UT50;
  mag_cal_input.time_stamp = (int)timestamp_ms;
  MotionFX_MagCal_run(&mag_cal_input);
  MotionFX_MagCal_getParams(&mag_cal_output);
  mag_cal_quality = mag_cal_output.cal_quality;

  for (axis = 0U; axis < MFX_NUM_AXES; axis++)
  {
    input->mag[axis] = mag_cal_input.mag[axis];
    if (mag_cal_output.cal_quality >= MFX_MAGCALOK)
    {
      input->mag[axis] -= mag_cal_output.hi_bias[axis];
    }
  }

  return BSP_ERROR_NONE;
}

static int32_t FusionEngine_Init(void)
{
  MFX_knobs_t knobs;
  uint32_t axis;
  const char acc_orientation[] = FUSION_ACC_ORIENTATION;
  const char gyro_orientation[] = FUSION_GYRO_ORIENTATION;
  const char mag_orientation[] = FUSION_MAG_ORIENTATION;

  if (MotionFX_GetStateSize() > sizeof(motionfx_state))
  {
    return BSP_ERROR_NO_INIT;
  }

  MotionFX_initialize((MFXState_t)motionfx_state);
  MotionFX_getKnobs((MFXState_t)motionfx_state, &knobs);

  for (axis = 0U; axis < MFX_NUM_AXES; axis++)
  {
    knobs.acc_orientation[axis] = acc_orientation[axis];
    knobs.gyro_orientation[axis] = gyro_orientation[axis];
    knobs.mag_orientation[axis] = mag_orientation[axis];
  }
  knobs.acc_orientation[MFX_NUM_AXES] = '\0';
  knobs.gyro_orientation[MFX_NUM_AXES] = '\0';
  knobs.mag_orientation[MFX_NUM_AXES] = '\0';
  knobs.gbias_acc_th_sc = FUSION_GBIAS_ACC_TH_SC;
  knobs.gbias_gyro_th_sc = FUSION_GBIAS_GYRO_TH_SC;
  knobs.gbias_mag_th_sc = FUSION_GBIAS_MAG_TH_SC;
  knobs.output_type = MFX_ENGINE_OUTPUT_ENU;
  knobs.LMode = 1U;
  knobs.modx = 1U;
  knobs.start_automatic_gbias_calculation = 1;

  MotionFX_setKnobs((MFXState_t)motionfx_state, &knobs);
  MotionFX_enable_6X((MFXState_t)motionfx_state, MFX_ENGINE_DISABLE);
  MotionFX_enable_9X((MFXState_t)motionfx_state, MFX_ENGINE_ENABLE);
  MotionFX_MagCal_init((int)FUSION_SAMPLE_PERIOD_MS, 1U);

  return BSP_ERROR_NONE;
}

static void FusionEngine_Run(MFX_input_t *input, MFX_output_t *output, float delta_time_s)
{
  MotionFX_propagate((MFXState_t)motionfx_state, output, input, &delta_time_s);
  MotionFX_update((MFXState_t)motionfx_state, output, input, &delta_time_s, NULL);
}

static int32_t FusionFloatToMicro(float value)
{
  if (value != value)
  {
    return 0;
  }
  if (value > 2.0f)
  {
    value = 2.0f;
  }
  else if (value < -2.0f)
  {
    value = -2.0f;
  }
  return (int32_t)(value * 1000000.0f);
}

static void FusionUart_PrintQuaternion(const MFX_output_t *output, uint32_t drdy_irq_count)
{
  int32_t scaled[MFX_QNUM_AXES];
  uint32_t absolute[MFX_QNUM_AXES];
  char sign[MFX_QNUM_AXES];
  int32_t length;
  uint32_t axis;
  uint32_t sequence = uart_sample_sequence++;

  if (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
  {
    return;
  }

  for (axis = 0U; axis < MFX_QNUM_AXES; axis++)
  {
    scaled[axis] = FusionFloatToMicro(output->quaternion[axis]);
    sign[axis] = (scaled[axis] < 0) ? '-' : '+';
    absolute[axis] = (scaled[axis] < 0) ? (uint32_t)(-scaled[axis]) : (uint32_t)scaled[axis];
  }

  length = snprintf((char *)uart_dma_message, sizeof(uart_dma_message),
                    "Q,%c%lu.%06lu,%c%lu.%06lu,%c%lu.%06lu,%c%lu.%06lu,M,%u,S,%lu,T,%lu,I,%lu\r\n",
                    sign[0], (unsigned long)(absolute[0] / 1000000U), (unsigned long)(absolute[0] % 1000000U),
                    sign[1], (unsigned long)(absolute[1] / 1000000U), (unsigned long)(absolute[1] % 1000000U),
                    sign[2], (unsigned long)(absolute[2] / 1000000U), (unsigned long)(absolute[2] % 1000000U),
                    sign[3], (unsigned long)(absolute[3] / 1000000U), (unsigned long)(absolute[3] % 1000000U),
                    (unsigned int)mag_cal_quality,
                    (unsigned long)sequence,
                    (unsigned long)HAL_GetTick(),
                    (unsigned long)drdy_irq_count);

  if (length > 0)
  {
    uint16_t transmit_length = (length < (int32_t)sizeof(uart_dma_message))
                               ? (uint16_t)length
                               : (uint16_t)(sizeof(uart_dma_message) - 1U);
    (void)HAL_UART_Transmit_DMA(&huart1, uart_dma_message, transmit_length);
  }
}

static void LevelLeds_Set(uint16_t leds)
{
  HAL_GPIO_WritePin(GPIOE, LEVEL_ALL_LEDS, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, leds, GPIO_PIN_SET);
}

static uint8_t LevelDetector_Update(const float gravity[3], uint8_t was_level)
{
  float abs_x = (gravity[0] < 0.0f) ? -gravity[0] : gravity[0];
  float abs_y = (gravity[1] < 0.0f) ? -gravity[1] : gravity[1];
  float abs_z = (gravity[2] < 0.0f) ? -gravity[2] : gravity[2];
  float threshold_ratio;

  if ((abs_z < 0.7f) || (abs_z > 1.3f))
  {
    return 0U;
  }

  threshold_ratio = (was_level != 0U) ? ((float)LEVEL_EXIT_RATIO_PER_1000 / 1000.0f)
                                      : ((float)LEVEL_ENTER_RATIO_PER_1000 / 1000.0f);

  return ((abs_x <= (abs_z * threshold_ratio)) &&
          (abs_y <= (abs_z * threshold_ratio))) ? 1U : 0U;
}

static uint16_t LevelDirection_GetLed(const float gravity[3])
{
  /* The LED ring's east/west direction is opposite to the fusion board X axis. */
  float x = -gravity[0];
  float y = gravity[1];
  float abs_x = (x < 0.0f) ? -x : x;
  float abs_y = (y < 0.0f) ? -y : y;

  /* Split the fused gravity projection into eight 45-degree sectors. */
  if ((abs_x * 1000.0f) <= (abs_y * (float)LEVEL_DIAGONAL_RATIO_PER_1000))
  {
    return (y < 0.0f) ? LD3_Pin : LD10_Pin;  /* North / South */
  }

  if ((abs_y * 1000.0f) <= (abs_x * (float)LEVEL_DIAGONAL_RATIO_PER_1000))
  {
    return (x > 0.0f) ? LD7_Pin : LD6_Pin;   /* East / West */
  }

  if (x > 0.0f)
  {
    return (y < 0.0f) ? LD5_Pin : LD9_Pin;   /* North-East / South-East */
  }

  return (y < 0.0f) ? LD4_Pin : LD8_Pin;     /* North-West / South-West */
}

char MotionFX_LoadMagCalFromNVM(unsigned short int data_size, unsigned int *data)
{
  (void)data_size;
  (void)data;
  return (char)1;
}

char MotionFX_SaveMagCalInNVM(unsigned short int data_size, unsigned int *data)
{
  (void)data_size;
  (void)data;
  return (char)1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == MEMS_INT2_Pin)
  {
    gyro_drdy_irq_count++;
    if (fusion_events_ready != 0U)
    {
      (void)tx_event_flags_set(&fusion_events, FUSION_EVENT_GYRO_DRDY, TX_OR);
    }
  }
}

VOID level_thread(ULONG thread_input)
{
  MFX_input_t fusion_input = {0};
  MFX_output_t fusion_output = {0};
  uint8_t is_level = 0U;
  uint8_t level_animation_on = 1U;
  uint8_t level_animation_samples = 0U;
  uint8_t uart_decimation = 0U;
  uint32_t previous_tick;

  (void)thread_input;
  LevelLeds_Set(LEVEL_ALL_LEDS);

  if ((FusionSensors_Init() != BSP_ERROR_NONE) ||
      (FusionEngine_Init() != BSP_ERROR_NONE) ||
      (FusionGyroDataReady_Enable() != BSP_ERROR_NONE))
  {
    LevelLeds_Set(0U);
    while (1)
    {
      HAL_GPIO_TogglePin(GPIOE, LEVEL_RED_LEDS);
      tx_thread_sleep(LEVEL_ERROR_BLINK_TICKS);
    }
  }

  previous_tick = HAL_GetTick();

  while (1)
  {
    ULONG actual_events;
    UINT event_status = tx_event_flags_get(&fusion_events, FUSION_EVENT_GYRO_DRDY,
                                           TX_OR_CLEAR, &actual_events,
                                           FUSION_DRDY_TIMEOUT_TICKS);

    if (event_status != TX_SUCCESS)
    {
      HAL_GPIO_WritePin(GPIOE, LEVEL_NON_RED_LEDS, GPIO_PIN_RESET);
      HAL_GPIO_TogglePin(GPIOE, LEVEL_RED_LEDS);
      continue;
    }

    uint32_t drdy_irq_snapshot = gyro_drdy_irq_count;
    uint32_t current_tick = HAL_GetTick();
    uint32_t elapsed_ms = current_tick - previous_tick;
    float delta_time_s = (elapsed_ms == 0U) ? FUSION_DEFAULT_DELTA_S
                                            : (float)elapsed_ms * 0.001f;
    previous_tick = current_tick;

    if (delta_time_s < FUSION_MIN_DELTA_S)
    {
      delta_time_s = FUSION_MIN_DELTA_S;
    }
    else if (delta_time_s > FUSION_MAX_DELTA_S)
    {
      delta_time_s = FUSION_MAX_DELTA_S;
    }

    if (FusionSensors_Read(&fusion_input, current_tick) == BSP_ERROR_NONE)
    {
      FusionEngine_Run(&fusion_input, &fusion_output, delta_time_s);

      uart_decimation++;
      if (uart_decimation >= FUSION_UART_DECIMATION)
      {
        uart_decimation = 0U;
        FusionUart_PrintQuaternion(&fusion_output, drdy_irq_snapshot);
      }

      is_level = LevelDetector_Update(fusion_output.gravity, is_level);

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
        LevelLeds_Set(LevelDirection_GetLed(fusion_output.gravity));
      }
    }
    else
    {
      HAL_GPIO_WritePin(GPIOE, LEVEL_NON_RED_LEDS, GPIO_PIN_RESET);
      HAL_GPIO_TogglePin(GPIOE, LEVEL_RED_LEDS);
    }

  }
}
/* USER CODE END  0 */
