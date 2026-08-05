/**
  ******************************************************************************
  * @file    custom_mems_conf.h
  * @author  MEMS Software Solutions Team
  * @brief   This file contains definitions of the MEMS components bus interfaces for custom boards
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CUSTOM_MEMS_CONF_H
#define CUSTOM_MEMS_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "stm32f3discovery_bus.h"
#include "stm32f3discovery_errno.h"

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

#define USE_CUSTOM_MOTION_SENSOR_LSM303AGR_ACC_0  1U
#define USE_CUSTOM_MOTION_SENSOR_LSM303AGR_MAG_0  1U

#define USE_CUSTOM_MOTION_SENSOR_A3G4250D_0       1U

#define CUSTOM_LSM303AGR_0_I2C_Init BSP_I2C1_Init
#define CUSTOM_LSM303AGR_0_I2C_DeInit BSP_I2C1_DeInit
#define CUSTOM_LSM303AGR_0_I2C_ReadReg BSP_I2C1_ReadReg
#define CUSTOM_LSM303AGR_0_I2C_WriteReg BSP_I2C1_WriteReg

#define CUSTOM_A3G4250D_0_SPI_Init BSP_SPI1_Init
#define CUSTOM_A3G4250D_0_SPI_DeInit BSP_SPI1_DeInit
#define CUSTOM_A3G4250D_0_SPI_Send BSP_SPI1_Send
#define CUSTOM_A3G4250D_0_SPI_Recv BSP_SPI1_Recv

#define CUSTOM_A3G4250D_0_CS_PORT GPIOE
#define CUSTOM_A3G4250D_0_CS_PIN GPIO_PIN_3

#ifdef __cplusplus
}
#endif

#endif /* CUSTOM_MEMS_CONF_H*/
