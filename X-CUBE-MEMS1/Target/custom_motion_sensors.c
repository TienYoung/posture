/**
  ******************************************************************************
  * @file    custom_motion_sensors.c
  * @author  MEMS Software Solutions Team
  * @brief   This file provides BSP Motion Sensors interface for custom boards
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

/* Includes ------------------------------------------------------------------*/
#include "custom_motion_sensors.h"

/** @addtogroup BSP BSP
  * @{
  */

/** @addtogroup CUSTOM CUSTOM
  * @{
  */

/** @defgroup CUSTOM_MOTION_SENSOR CUSTOM MOTION SENSOR
  * @{
  */

/** @defgroup CUSTOM_MOTION_SENSOR_Exported_Variables CUSTOM MOTION SENSOR Exported Variables
  * @{
  */

extern void
*MotionCompObj[CUSTOM_MOTION_INSTANCES_NBR]; /* This "redundant" line is here to fulfil MISRA C-2012 rule 8.4 */
void *MotionCompObj[CUSTOM_MOTION_INSTANCES_NBR];

/**
  * @}
  */

/** @defgroup CUSTOM_MOTION_SENSOR_Private_Variables CUSTOM MOTION SENSOR Private Variables
  * @{
  */

/* We define a jump table in order to get the correct index from the desired function. */
/* This table should have a size equal to the maximum value of a function plus 1.      */
/* But due to MISRA it has to be increased to 7 + 1. */
static uint32_t FunctionIndex[] = {0, 0, 1, 1, 2, 2, 2, 2};
static MOTION_SENSOR_FuncDrv_t *MotionFuncDrv[CUSTOM_MOTION_INSTANCES_NBR][CUSTOM_MOTION_FUNCTIONS_NBR];
static MOTION_SENSOR_CommonDrv_t *MotionDrv[CUSTOM_MOTION_INSTANCES_NBR];
static CUSTOM_MOTION_SENSOR_Ctx_t MotionCtx[CUSTOM_MOTION_INSTANCES_NBR];

/**
  * @}
  */

/** @defgroup CUSTOM_MOTION_SENSOR_Private_Function_Prototypes CUSTOM MOTION SENSOR Private Function Prototypes
  * @{
  */

#if (USE_CUSTOM_MOTION_SENSOR_LSM303AGR_ACC_0 == 1)
static int32_t LSM303AGR_ACC_0_Probe(uint32_t Functions);
#endif /* USE_CUSTOM_MOTION_SENSOR_LSM303AGR_ACC_0 */
#if (USE_CUSTOM_MOTION_SENSOR_LSM303AGR_MAG_0 == 1)
static int32_t LSM303AGR_MAG_0_Probe(uint32_t Functions);
#endif /* USE_CUSTOM_MOTION_SENSOR_LSM303AGR_MAG_0 */
#if (USE_CUSTOM_MOTION_SENSOR_A3G4250D_0 == 1)
static int32_t A3G4250D_0_Probe(uint32_t Functions);
#endif /* USE_CUSTOM_MOTION_SENSOR_A3G4250D_0 */

#if (USE_CUSTOM_MOTION_SENSOR_A3G4250D_0 == 1)
static int32_t CUSTOM_A3G4250D_0_Init(void);
static int32_t CUSTOM_A3G4250D_0_DeInit(void);
static int32_t CUSTOM_A3G4250D_0_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pdata, uint16_t len);
static int32_t CUSTOM_A3G4250D_0_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pdata, uint16_t len);
#endif /* USE_CUSTOM_MOTION_SENSOR_A3G4250D_0 */

/**
  * @}
  */

/** @defgroup CUSTOM_MOTION_SENSOR_Exported_Functions CUSTOM MOTION SENSOR Exported Functions
  * @{
  */

/**
  * @brief  Initializes the motion sensors
  * @param  Instance Motion sensor instance
  * @param  Functions Motion sensor functions. Could be :
  *         - MOTION_GYRO
  *         - MOTION_ACCELERO
  *         - MOTION_MAGNETO
  * @retval BSP status
  */
int32_t CUSTOM_MOTION_SENSOR_Init(uint32_t Instance, uint32_t Functions)
{
  int32_t ret = BSP_ERROR_NONE;
  uint32_t function = MOTION_GYRO;
  uint32_t i;
  uint32_t component_functions = 0;
  CUSTOM_MOTION_SENSOR_Capabilities_t cap;

  switch (Instance)
  {
#if (USE_CUSTOM_MOTION_SENSOR_LSM303AGR_ACC_0 == 1)
    case CUSTOM_LSM303AGR_ACC_0:
      if (LSM303AGR_ACC_0_Probe(Functions) != BSP_ERROR_NONE)
      {
        return BSP_ERROR_NO_INIT;
      }
      if (MotionDrv[Instance]->GetCapabilities(MotionCompObj[Instance], (void *)&cap) != BSP_ERROR_NONE)
      {
        return BSP_ERROR_UNKNOWN_COMPONENT;
      }
      if (cap.Acc == 1U)
      {
        component_functions |= MOTION_ACCELERO;
      }
      if (cap.Gyro == 1U)
      {
        component_functions |= MOTION_GYRO;
      }
      if (cap.Magneto == 1U)
      {
        component_functions |= MOTION_MAGNETO;
      }
      break;
#endif /* USE_CUSTOM_MOTION_SENSOR_LSM303AGR_ACC_0 */
#if (USE_CUSTOM_MOTION_SENSOR_LSM303AGR_MAG_0 == 1)
    case CUSTOM_LSM303AGR_MAG_0:
      if (LSM303AGR_MAG_0_Probe(Functions) != BSP_ERROR_NONE)
      {
        return BSP_ERROR_NO_INIT;
      }
      if (MotionDrv[Instance]->GetCapabilities(MotionCompObj[Instance], (void *)&cap) != BSP_ERROR_NONE)
      {
        return BSP_ERROR_UNKNOWN_COMPONENT;
      }
      if (cap.Acc == 1U)
      {
        component_functions |= MOTION_ACCELERO;
      }
      if (cap.Gyro == 1U)
      {
        component_functions |= MOTION_GYRO;
      }
      if (cap.Magneto == 1U)
      {
        component_functions |= MOTION_MAGNETO;
      }
      break;
#endif /* USE_CUSTOM_MOTION_SENSOR_LSM303AGR_MAG_0 */
#if (USE_CUSTOM_MOTION_SENSOR_A3G4250D_0 == 1)
    case CUSTOM_A3G4250D_0:
      if (A3G4250D_0_Probe(Functions) != BSP_ERROR_NONE)
      {
        return BSP_ERROR_NO_INIT;
      }
      if (MotionDrv[Instance]->GetCapabilities(MotionCompObj[Instance], (void *)&cap) != BSP_ERROR_NONE)
      {
        return BSP_ERROR_UNKNOWN_COMPONENT;
      }
      if (cap.Acc == 1U)
      {
        component_functions |= MOTION_ACCELERO;
      }
      if (cap.Gyro == 1U)
      {
        component_functions |= MOTION_GYRO;
      }
      if (cap.Magneto == 1U)
      {
        component_functions |= MOTION_MAGNETO;
      }
      break;
#endif /* USE_CUSTOM_MOTION_SENSOR_A3G4250D_0 */
    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  if (ret != BSP_ERROR_NONE)
  {
    return ret;
  }

  for (i = 0; i < CUSTOM_MOTION_FUNCTIONS_NBR; i++)
  {
    if (((Functions & function) == function) && ((component_functions & function) == function))
    {
      if (MotionFuncDrv[Instance][FunctionIndex[function]]->Enable(MotionCompObj[Instance]) != BSP_ERROR_NONE)
      {
        return BSP_ERROR_COMPONENT_FAILURE;
      }
    }
    function = function << 1;
  }

  return ret;
}

/**
  * @brief  Deinitialize Motion sensor
  * @param  Instance Motion sensor instance
  * @retval BSP status
  */
int32_t CUSTOM_MOTION_SENSOR_DeInit(uint32_t Instance)
{
  int32_t ret;

  if (Instance >= CUSTOM_MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (MotionDrv[Instance]->DeInit(MotionCompObj[Instance]) != BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief  Get motion sensor instance capabilities
  * @param  Instance Motion sensor instance
  * @param  Capabilities pointer to motion sensor capabilities
  * @retval BSP status
  */
int32_t CUSTOM_MOTION_SENSOR_GetCapabilities(uint32_t Instance, CUSTOM_MOTION_SENSOR_Capabilities_t *Capabilities)
{
  int32_t ret;

  if (Instance >= CUSTOM_MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (MotionDrv[Instance]->GetCapabilities(MotionCompObj[Instance], Capabilities) != BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief  Get WHOAMI value
  * @param  Instance Motion sensor instance
  * @param  Id WHOAMI value
  * @retval BSP status
  */
int32_t CUSTOM_MOTION_SENSOR_ReadID(uint32_t Instance, uint8_t *Id)
{
  int32_t ret;

  if (Instance >= CUSTOM_MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (MotionDrv[Instance]->ReadID(MotionCompObj[Instance], Id) != BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief  Enable Motion sensor
  * @param  Instance Motion sensor instance
  * @param  Function Motion sensor function. Could be :
  *         - MOTION_GYRO
  *         - MOTION_ACCELERO
  *         - MOTION_MAGNETO
  * @retval BSP status
  */
int32_t CUSTOM_MOTION_SENSOR_Enable(uint32_t Instance, uint32_t Function)
{
  int32_t ret;

  if (Instance >= CUSTOM_MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((MotionCtx[Instance].Functions & Function) == Function)
    {
      if (MotionFuncDrv[Instance][FunctionIndex[Function]]->Enable(MotionCompObj[Instance]) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }

  return ret;
}

/**
  * @brief  Disable Motion sensor
  * @param  Instance Motion sensor instance
  * @param  Function Motion sensor function. Could be :
  *         - MOTION_GYRO
  *         - MOTION_ACCELERO
  *         - MOTION_MAGNETO
  * @retval BSP status
  */
int32_t CUSTOM_MOTION_SENSOR_Disable(uint32_t Instance, uint32_t Function)
{
  int32_t ret;

  if (Instance >= CUSTOM_MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((MotionCtx[Instance].Functions & Function) == Function)
    {
      if (MotionFuncDrv[Instance][FunctionIndex[Function]]->Disable(MotionCompObj[Instance]) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }

  return ret;
}

/**
  * @brief  Get motion sensor axes data
  * @param  Instance Motion sensor instance
  * @param  Function Motion sensor function. Could be :
  *         - MOTION_GYRO
  *         - MOTION_ACCELERO
  *         - MOTION_MAGNETO
  * @param  Axes pointer to axes data structure
  * @retval BSP status
  */
int32_t CUSTOM_MOTION_SENSOR_GetAxes(uint32_t Instance, uint32_t Function, CUSTOM_MOTION_SENSOR_Axes_t *Axes)
{
  int32_t ret;

  if (Instance >= CUSTOM_MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((MotionCtx[Instance].Functions & Function) == Function)
    {
      if (MotionFuncDrv[Instance][FunctionIndex[Function]]->GetAxes(MotionCompObj[Instance], Axes) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }

  return ret;
}

/**
  * @brief  Get motion sensor axes raw data
  * @param  Instance Motion sensor instance
  * @param  Function Motion sensor function. Could be :
  *         - MOTION_GYRO
  *         - MOTION_ACCELERO
  *         - MOTION_MAGNETO
  * @param  Axes pointer to axes raw data structure
  * @retval BSP status
  */
int32_t CUSTOM_MOTION_SENSOR_GetAxesRaw(uint32_t Instance, uint32_t Function, CUSTOM_MOTION_SENSOR_AxesRaw_t *Axes)
{
  int32_t ret;

  if (Instance >= CUSTOM_MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((MotionCtx[Instance].Functions & Function) == Function)
    {
      if (MotionFuncDrv[Instance][FunctionIndex[Function]]->GetAxesRaw(MotionCompObj[Instance], Axes) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }

  return ret;
}

/**
  * @brief  Get motion sensor sensitivity
  * @param  Instance Motion sensor instance
  * @param  Function Motion sensor function. Could be :
  *         - MOTION_GYRO
  *         - MOTION_ACCELERO
  *         - MOTION_MAGNETO
  * @param  Sensitivity pointer to sensitivity read value
  * @retval BSP status
  */
int32_t CUSTOM_MOTION_SENSOR_GetSensitivity(uint32_t Instance, uint32_t Function, float_t *Sensitivity)
{
  int32_t ret;

  if (Instance >= CUSTOM_MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((MotionCtx[Instance].Functions & Function) == Function)
    {
      if (MotionFuncDrv[Instance][FunctionIndex[Function]]->GetSensitivity(MotionCompObj[Instance],
                                                                           Sensitivity) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }

  return ret;
}

/**
  * @brief  Get motion sensor Output Data Rate
  * @param  Instance Motion sensor instance
  * @param  Function Motion sensor function. Could be :
  *         - MOTION_GYRO
  *         - MOTION_ACCELERO
  *         - MOTION_MAGNETO
  * @param  Odr pointer to Output Data Rate read value
  * @retval BSP status
  */
int32_t CUSTOM_MOTION_SENSOR_GetOutputDataRate(uint32_t Instance, uint32_t Function, float_t *Odr)
{
  int32_t ret;

  if (Instance >= CUSTOM_MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((MotionCtx[Instance].Functions & Function) == Function)
    {
      if (MotionFuncDrv[Instance][FunctionIndex[Function]]->GetOutputDataRate(MotionCompObj[Instance], Odr) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }

  return ret;
}

/**
  * @brief  Get motion sensor Full Scale
  * @param  Instance Motion sensor instance
  * @param  Function Motion sensor function. Could be :
  *         - MOTION_GYRO and/or MOTION_ACCELERO for instance 0
  *         - MOTION_ACCELERO for instance 1
  *         - MOTION_MAGNETO for instance 2
  * @param  Fullscale pointer to Fullscale read value
  * @retval BSP status
  */
int32_t CUSTOM_MOTION_SENSOR_GetFullScale(uint32_t Instance, uint32_t Function, int32_t *Fullscale)
{
  int32_t ret;

  if (Instance >= CUSTOM_MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((MotionCtx[Instance].Functions & Function) == Function)
    {
      if (MotionFuncDrv[Instance][FunctionIndex[Function]]->GetFullScale(MotionCompObj[Instance],
                                                                         Fullscale) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }

  return ret;
}

/**
  * @brief  Set motion sensor Output Data Rate
  * @param  Instance Motion sensor instance
  * @param  Function Motion sensor function. Could be :
  *         - MOTION_GYRO
  *         - MOTION_ACCELERO
  *         - MOTION_MAGNETO
  * @param  Odr Output Data Rate value to be set
  * @retval BSP status
  */
int32_t CUSTOM_MOTION_SENSOR_SetOutputDataRate(uint32_t Instance, uint32_t Function, float_t Odr)
{
  int32_t ret;

  if (Instance >= CUSTOM_MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((MotionCtx[Instance].Functions & Function) == Function)
    {
      if (MotionFuncDrv[Instance][FunctionIndex[Function]]->SetOutputDataRate(MotionCompObj[Instance], Odr) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }

  return ret;
}

/**
  * @brief  Set motion sensor Full Scale
  * @param  Instance Motion sensor instance
  * @param  Function Motion sensor function. Could be :
  *         - MOTION_GYRO
  *         - MOTION_ACCELERO
  *         - MOTION_MAGNETO
  * @param  Fullscale Fullscale value to be set
  * @retval BSP status
  */
int32_t CUSTOM_MOTION_SENSOR_SetFullScale(uint32_t Instance, uint32_t Function, int32_t Fullscale)
{
  int32_t ret;

  if (Instance >= CUSTOM_MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((MotionCtx[Instance].Functions & Function) == Function)
    {
      if (MotionFuncDrv[Instance][FunctionIndex[Function]]->SetFullScale(MotionCompObj[Instance],
                                                                         Fullscale) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }

  return ret;
}

/**
  * @}
  */

/** @defgroup CUSTOM_MOTION_SENSOR_Private_Functions CUSTOM MOTION SENSOR Private Functions
  * @{
  */

#if (USE_CUSTOM_MOTION_SENSOR_LSM303AGR_ACC_0 == 1)
/**
  * @brief  Register Bus IOs for LSM303AGR accelerometer instance
  * @param  Functions Motion sensor functions. Could be :
  *         - MOTION_ACCELERO
  * @retval BSP status
 */
static int32_t LSM303AGR_ACC_0_Probe(uint32_t Functions)
{
  LSM303AGR_IO_t                io_ctx;
  uint8_t                       id;
  static LSM303AGR_ACC_Object_t lsm303agr_acc_obj_0;
  LSM303AGR_Capabilities_t      cap;
  int32_t                       ret = BSP_ERROR_NONE;

  /* Configure the driver */
  io_ctx.BusType     = LSM303AGR_I2C_BUS; /* I2C */
  io_ctx.Address     = LSM303AGR_I2C_ADD_XL;
  io_ctx.Init        = CUSTOM_LSM303AGR_0_I2C_Init;
  io_ctx.DeInit      = CUSTOM_LSM303AGR_0_I2C_DeInit;
  io_ctx.ReadReg     = CUSTOM_LSM303AGR_0_I2C_ReadReg;
  io_ctx.WriteReg    = CUSTOM_LSM303AGR_0_I2C_WriteReg;
  io_ctx.GetTick     = BSP_GetTick;
  io_ctx.Delay       = HAL_Delay;

  if (LSM303AGR_ACC_RegisterBusIO(&lsm303agr_acc_obj_0, &io_ctx) != LSM303AGR_OK)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else if (LSM303AGR_ACC_ReadID(&lsm303agr_acc_obj_0, &id) != LSM303AGR_OK)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else if (id != (uint8_t)LSM303AGR_ID_XL)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else
  {
    (void)LSM303AGR_ACC_GetCapabilities(&lsm303agr_acc_obj_0, &cap);
    MotionCtx[CUSTOM_LSM303AGR_ACC_0].Functions = ((uint32_t)cap.Gyro) | ((uint32_t)cap.Acc << 1) | ((uint32_t)cap.Magneto << 2);

    MotionCompObj[CUSTOM_LSM303AGR_ACC_0] = &lsm303agr_acc_obj_0;
    /* The second cast (void *) is added to bypass Misra R11.3 rule */
    MotionDrv[CUSTOM_LSM303AGR_ACC_0] = (MOTION_SENSOR_CommonDrv_t *)(void *)&LSM303AGR_ACC_COMMON_Driver;

    if ((ret == BSP_ERROR_NONE) && ((Functions & MOTION_GYRO) == MOTION_GYRO))
    {
      /* Return an error if the application try to initialize a function not supported by the component */
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    if ((ret == BSP_ERROR_NONE) && ((Functions & MOTION_ACCELERO) == MOTION_ACCELERO) && (cap.Acc == 1U))
    {
      /* The second cast (void *) is added to bypass Misra R11.3 rule */
      MotionFuncDrv[CUSTOM_LSM303AGR_ACC_0][FunctionIndex[MOTION_ACCELERO]] = (MOTION_SENSOR_FuncDrv_t *)(
                                                                               void *)&LSM303AGR_ACC_Driver;

      if (MotionDrv[CUSTOM_LSM303AGR_ACC_0]->Init(MotionCompObj[CUSTOM_LSM303AGR_ACC_0]) != LSM303AGR_OK)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    if ((ret == BSP_ERROR_NONE) && ((Functions & MOTION_MAGNETO) == MOTION_MAGNETO))
    {
      /* Return an error if the application try to initialize a function not supported by the component */
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
  }

  return ret;
}
#endif /* USE_CUSTOM_MOTION_SENSOR_LSM303AGR_ACC_0 */

#if (USE_CUSTOM_MOTION_SENSOR_LSM303AGR_MAG_0 == 1)
/**
  * @brief  Register Bus IOs for LSM303AGR magnetometer instance
  * @param  Functions Motion sensor functions. Could be :
  *         - MOTION_MAGNETO
  * @retval BSP status
  */
static int32_t LSM303AGR_MAG_0_Probe(uint32_t Functions)
{
  LSM303AGR_IO_t                io_ctx;
  uint8_t                       id;
  static LSM303AGR_MAG_Object_t lsm303agr_mag_obj_0;
  LSM303AGR_Capabilities_t      cap;
  int32_t                       ret = BSP_ERROR_NONE;

  /* Configure the driver */
  io_ctx.BusType     = LSM303AGR_I2C_BUS; /* I2C */
  io_ctx.Address     = LSM303AGR_I2C_ADD_MG;
  io_ctx.Init        = CUSTOM_LSM303AGR_0_I2C_Init;
  io_ctx.DeInit      = CUSTOM_LSM303AGR_0_I2C_DeInit;
  io_ctx.ReadReg     = CUSTOM_LSM303AGR_0_I2C_ReadReg;
  io_ctx.WriteReg    = CUSTOM_LSM303AGR_0_I2C_WriteReg;
  io_ctx.GetTick     = BSP_GetTick;
  io_ctx.Delay       = HAL_Delay;

  if (LSM303AGR_MAG_RegisterBusIO(&lsm303agr_mag_obj_0, &io_ctx) != LSM303AGR_OK)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else if (LSM303AGR_MAG_ReadID(&lsm303agr_mag_obj_0, &id) != LSM303AGR_OK)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else if (id != (uint8_t)LSM303AGR_ID_MG)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else
  {
    (void)LSM303AGR_MAG_GetCapabilities(&lsm303agr_mag_obj_0, &cap);
    MotionCtx[CUSTOM_LSM303AGR_MAG_0].Functions = ((uint32_t)cap.Gyro) | ((uint32_t)cap.Acc << 1) | ((uint32_t)cap.Magneto << 2);

    MotionCompObj[CUSTOM_LSM303AGR_MAG_0] = &lsm303agr_mag_obj_0;
    /* The second cast (void *) is added to bypass Misra R11.3 rule */
    MotionDrv[CUSTOM_LSM303AGR_MAG_0] = (MOTION_SENSOR_CommonDrv_t *)(void *)&LSM303AGR_MAG_COMMON_Driver;

    if ((ret == BSP_ERROR_NONE) && ((Functions & MOTION_GYRO) == MOTION_GYRO))
    {
      /* Return an error if the application try to initialize a function not supported by the component */
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    if ((ret == BSP_ERROR_NONE) && ((Functions & MOTION_ACCELERO) == MOTION_ACCELERO))
    {
      /* Return an error if the application try to initialize a function not supported by the component */
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    if ((ret == BSP_ERROR_NONE) && ((Functions & MOTION_MAGNETO) == MOTION_MAGNETO) && (cap.Magneto == 1U))
    {
      /* The second cast (void *) is added to bypass Misra R11.3 rule */
      MotionFuncDrv[CUSTOM_LSM303AGR_MAG_0][FunctionIndex[MOTION_MAGNETO]] = (MOTION_SENSOR_FuncDrv_t *)(
                                                                              void *)&LSM303AGR_MAG_Driver;

      if (MotionDrv[CUSTOM_LSM303AGR_MAG_0]->Init(MotionCompObj[CUSTOM_LSM303AGR_MAG_0]) != LSM303AGR_OK)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
  }

  return ret;
}
#endif /* USE_CUSTOM_MOTION_SENSOR_LSM303AGR_MAG_0 */

#if (USE_CUSTOM_MOTION_SENSOR_A3G4250D_0 == 1)
/**
  * @brief  Register Bus IOs for A3G4250D instance
  * @param  Functions Motion sensor functions. Could be :
  *         - MOTION_GYRO
  * @retval BSP status
  */
static int32_t A3G4250D_0_Probe(uint32_t Functions)
{
  A3G4250D_IO_t            io_ctx;
  uint8_t                  id;
  static A3G4250D_Object_t a3g4250d_obj_0;
  A3G4250D_Capabilities_t  cap;
  int32_t                  ret = BSP_ERROR_NONE;

  /* Configure the driver */
  io_ctx.BusType     = A3G4250D_SPI_4WIRES_BUS; /* SPI 4-Wires */
  io_ctx.Address     = 0x0;
  io_ctx.Init        = CUSTOM_A3G4250D_0_Init;
  io_ctx.DeInit      = CUSTOM_A3G4250D_0_DeInit;
  io_ctx.ReadReg     = CUSTOM_A3G4250D_0_ReadReg;
  io_ctx.WriteReg    = CUSTOM_A3G4250D_0_WriteReg;
  io_ctx.GetTick     = BSP_GetTick;
  io_ctx.Delay       = HAL_Delay;

  if (A3G4250D_RegisterBusIO(&a3g4250d_obj_0, &io_ctx) != A3G4250D_OK)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else if (A3G4250D_ReadID(&a3g4250d_obj_0, &id) != A3G4250D_OK)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else if (id != (uint8_t)A3G4250D_ID)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else
  {
    (void)A3G4250D_GetCapabilities(&a3g4250d_obj_0, &cap);
    MotionCtx[CUSTOM_A3G4250D_0].Functions = ((uint32_t)cap.Gyro) | ((uint32_t)cap.Acc << 1) | ((uint32_t)cap.Magneto << 2);

    MotionCompObj[CUSTOM_A3G4250D_0] = &a3g4250d_obj_0;
    /* The second cast (void *) is added to bypass Misra R11.3 rule */
    MotionDrv[CUSTOM_A3G4250D_0] = (MOTION_SENSOR_CommonDrv_t *)(void *)&A3G4250D_COMMON_Driver;

    if ((ret == BSP_ERROR_NONE) && ((Functions & MOTION_GYRO) == MOTION_GYRO) && (cap.Gyro == 1U))
    {
      /* The second cast (void *) is added to bypass Misra R11.3 rule */
      MotionFuncDrv[CUSTOM_A3G4250D_0][FunctionIndex[MOTION_GYRO]] = (MOTION_SENSOR_FuncDrv_t *)(
                                                                      void *)&A3G4250D_GYRO_Driver;

      if (MotionDrv[CUSTOM_A3G4250D_0]->Init(MotionCompObj[CUSTOM_A3G4250D_0]) != A3G4250D_OK)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    if ((ret == BSP_ERROR_NONE) && ((Functions & MOTION_ACCELERO) == MOTION_ACCELERO))
    {
      /* Return an error if the application try to initialize a function not supported by the component */
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    if ((ret == BSP_ERROR_NONE) && ((Functions & MOTION_MAGNETO) == MOTION_MAGNETO))
    {
      /* Return an error if the application try to initialize a function not supported by the component */
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
  }

  return ret;
}

/**
  * @brief  Initialize SPI bus for A3G4250D
  * @retval BSP status
  */
static int32_t CUSTOM_A3G4250D_0_Init(void)
{
  int32_t ret = BSP_ERROR_UNKNOWN_FAILURE;

  if(CUSTOM_A3G4250D_0_SPI_Init() == BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief  DeInitialize SPI bus for A3G4250D
  * @retval BSP status
  */
static int32_t CUSTOM_A3G4250D_0_DeInit(void)
{
  int32_t ret = BSP_ERROR_UNKNOWN_FAILURE;

  if(CUSTOM_A3G4250D_0_SPI_DeInit() == BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief  Write register by SPI bus for A3G4250D
  * @param  Addr not used, it is only for BSP compatibility
  * @param  Reg the starting register address to be written
  * @param  pdata the pointer to the data to be written
  * @param  len the length of the data to be written
  * @retval BSP status
  */
static int32_t CUSTOM_A3G4250D_0_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pdata, uint16_t len)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t dataReg = (uint8_t)Reg;

  /* CS Enable */
  HAL_GPIO_WritePin(CUSTOM_A3G4250D_0_CS_PORT, CUSTOM_A3G4250D_0_CS_PIN, GPIO_PIN_RESET);

  if (CUSTOM_A3G4250D_0_SPI_Send(&dataReg, 1) != BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  if (CUSTOM_A3G4250D_0_SPI_Send(pdata, len) != BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  /* CS Disable */
  HAL_GPIO_WritePin(CUSTOM_A3G4250D_0_CS_PORT, CUSTOM_A3G4250D_0_CS_PIN, GPIO_PIN_SET);

  return ret;
}

/**
  * @brief  Read register by SPI bus for A3G4250D
  * @param  Addr not used, it is only for BSP compatibility
  * @param  Reg the starting register address to be read
  * @param  pdata the pointer to the data to be read
  * @param  len the length of the data to be read
  * @retval BSP status
  */
static int32_t CUSTOM_A3G4250D_0_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pdata, uint16_t len)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t dataReg = (uint8_t)Reg;

  dataReg |= 0x80;

  /* CS Enable */
  HAL_GPIO_WritePin(CUSTOM_A3G4250D_0_CS_PORT, CUSTOM_A3G4250D_0_CS_PIN, GPIO_PIN_RESET);

  if (CUSTOM_A3G4250D_0_SPI_Send(&dataReg, 1) != BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  if (CUSTOM_A3G4250D_0_SPI_Recv(pdata, len) != BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  /* CS Disable */
  HAL_GPIO_WritePin(CUSTOM_A3G4250D_0_CS_PORT, CUSTOM_A3G4250D_0_CS_PIN, GPIO_PIN_SET);

  return ret;
}
#endif /* USE_CUSTOM_MOTION_SENSOR_A3G4250D_0 */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
