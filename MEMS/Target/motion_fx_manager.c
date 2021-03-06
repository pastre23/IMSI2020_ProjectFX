/**
 ******************************************************************************
 * @file    motion_fx_manager.c
 * @author  MEMS Software Solutions Team
 * @brief   This file contains Datalog Fusion interface functions
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed under Software License Agreement
 * SLA0077, (the "License"). You may not use this file except in compliance
 * with the License. You may obtain a copy of the License at:
 *
 *     www.st.com/content/st_com/en/search.html#q=SLA0077-t=keywords-page=1
 *
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "motion_fx_manager.h"
#include "iks01a2_mems_control_ex.h"

/** @addtogroup MOTION_APPLICATIONS MOTION APPLICATIONS
 * @{
 */

/** @addtogroup DATALOG_FUSION DATALOG FUSION
 * @{
 */

/* Extern variables ----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define SAMPLETODISCARD                 15

#define GBIAS_ACC_TH_SC_6X              (2.0f*0.000765f)
#define GBIAS_GYRO_TH_SC_6X             (2.0f*0.002f)
#define GBIAS_MAG_TH_SC_6X              (2.0f*0.001500f)
#define GBIAS_ACC_TH_SC_9X              (2.0f*0.000765f)
#define GBIAS_GYRO_TH_SC_9X             (2.0f*0.002f)
#define GBIAS_MAG_TH_SC_9X              (2.0f*0.001500f)

#define DECIMATION                      1U

/* Private variables ---------------------------------------------------------*/
static MFX_knobs_t iKnobs;
static MFX_knobs_t *ipKnobs = &iKnobs;

static volatile int sampleToDiscard = SAMPLETODISCARD;
static int discardedCount = 0;

/* Private typedef -----------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
 * @brief  Initialize the MotionFX engine
 * @param  None
 * @retval None
 */
void MotionFX_manager_init(void)
{
  MotionFX_initialize();

  MotionFX_getKnobs(ipKnobs);

  BSP_SENSOR_ACC_GetOrientation(ipKnobs->acc_orientation);
  BSP_SENSOR_GYR_GetOrientation(ipKnobs->gyro_orientation);
  BSP_SENSOR_MAG_GetOrientation(ipKnobs->mag_orientation);

  ipKnobs->gbias_acc_th_sc_6X = GBIAS_ACC_TH_SC_6X;
  ipKnobs->gbias_gyro_th_sc_6X = GBIAS_GYRO_TH_SC_6X;
  ipKnobs->gbias_mag_th_sc_6X = GBIAS_MAG_TH_SC_6X;

  ipKnobs->gbias_acc_th_sc_9X = GBIAS_ACC_TH_SC_9X;
  ipKnobs->gbias_gyro_th_sc_9X = GBIAS_GYRO_TH_SC_9X;
  ipKnobs->gbias_mag_th_sc_9X = GBIAS_MAG_TH_SC_9X;

  ipKnobs->output_type = MFX_ENGINE_OUTPUT_ENU;
  ipKnobs->LMode = 1;
  ipKnobs->modx = DECIMATION;

  MotionFX_setKnobs(ipKnobs);

  MotionFX_enable_6X(MFX_ENGINE_ENABLE);
  MotionFX_enable_9X(MFX_ENGINE_ENABLE);
}

/**
 * @brief  Run Motion Sensor Data Fusion algorithm
 * @param  data_in  Structure containing input data
 * @param  data_out Structure containing output data
 * @param  delta_time Delta time
 * @retval None
 */
void MotionFX_manager_run(MFX_input_t *data_in, MFX_output_t *data_out, float delta_time)
{
  if (discardedCount == sampleToDiscard)
  {
    MotionFX_propagate(data_out, data_in, &delta_time);
    MotionFX_update(data_out, data_in, &delta_time, NULL);
  }
  else
  {
    discardedCount++;
  }
}

/**
 * @brief  Start 6 axes MotionFX engine
 * @param  None
 * @retval None
 */
void MotionFX_manager_start_6X(void)
{
  MotionFX_enable_6X(MFX_ENGINE_ENABLE);
}

/**
 * @brief  Stop 6 axes MotionFX engine
 * @param  None
 * @retval None
 */
void MotionFX_manager_stop_6X(void)
{
  MotionFX_enable_6X(MFX_ENGINE_DISABLE);
}

/**
 * @brief  Start 9 axes MotionFX engine
 * @param  None
 * @retval None
 */
void MotionFX_manager_start_9X(void)
{
  MotionFX_enable_9X(MFX_ENGINE_ENABLE);
}

/**
 * @brief  Stop 9 axes MotionFX engine
 * @param  None
 * @retval None
 */
void MotionFX_manager_stop_9X(void)
{
  MotionFX_enable_9X(MFX_ENGINE_DISABLE);
}

/**
 * @brief  Get the library version
 * @param  version  Library version string (must be array of 35 char)
 * @param  length  Library version string length
 * @retval None
 */
void MotionFX_manager_get_version(char *version, int *length)
{
  *length = (int)MotionFX_GetLibVersion(version);
}

/**
 * @brief  Run magnetometer calibration algorithm
 * @param  None
 * @retval None
 */
void MotionFX_manager_MagCal_run(MFX_MagCal_input_t *data_in, MFX_MagCal_output_t *data_out)
{
  MotionFX_MagCal_run(data_in);
  MotionFX_MagCal_getParams(data_out);
}

/**
 * @brief  Start magnetometer calibration
 * @param  None
 * @retval None
 */
void MotionFX_manager_MagCal_start(int sampletime)
{
  MotionFX_MagCal_init(sampletime, 1);
}

/**
 * @brief  Stop magnetometer calibration
 * @param  None
 * @retval None
 */
void MotionFX_manager_MagCal_stop(int sampletime)
{
  MotionFX_MagCal_init(sampletime, 0);
}

/**
 * @brief  Load calibration parameter from memory
 * @param  dataSize length ot the data
 * @param  data pointer to the data
 * @retval (1) fail, (0) success
 */
char MotionFX_LoadMagCalFromNVM(unsigned short int dataSize, unsigned int *data)
{
  return (char)1;
}

/**
 * @brief  Save calibration parameter to memory
 * @param  dataSize length ot the data
 * @param  data pointer to the data
 * @retval (1) fail, (0) success
 */
char MotionFX_SaveMagCalInNVM(unsigned short int dataSize, unsigned int *data)
{
  return (char)1;
}

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
