/**
  ******************************************************************************
  * File Name          : stmicroelectronics_x-cube-mems1_7_1_0.c
  * Description        : This file provides code for the configuration
  *                      of the STMicroelectronics.X-CUBE-MEMS1.7.1.0 instances.
  ******************************************************************************
  *
  * COPYRIGHT 2020 STMicroelectronics
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  ******************************************************************************
  */

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "app_x-cube-mems1.h"
#include "main.h"
#include <stdio.h>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"
#include "com.h"
#include "demo_serial.h"
#include "motion_fx_manager.h"
#include "bsp_ip_conf.h"
#include "fw_version.h"
#include "gatt_db.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ALGO_FREQ    100U /* Algorithm frequency [Hz] */

#define ALGO_PERIOD  10   /* Algorithm period [ms] */

#define MOTION_FX_ENGINE_DELTATIME  0.01f

#define FROM_MG_TO_G         0.001f
#define FROM_G_TO_MG         1000.0f
#define FROM_MDPS_TO_DPS     0.001f
#define FROM_DPS_TO_MDPS     1000.0f
#define FROM_MGAUSS_TO_UT50  (0.1f/50.0f)
#define FROM_UT50_TO_MGAUSS  500.0f

/* Public variables ----------------------------------------------------------*/
volatile uint8_t DataLoggerActive = 0;
volatile uint32_t SensorsEnabled = 0;
uint8_t Enabled6X = 1;

char lib_version[35];
int lib_version_len;

/* Extern variables ----------------------------------------------------------*/
extern AxesRaw_t x_axes;
extern AxesRaw_t g_axes;
extern AxesRaw_t m_axes;
extern AxesRaw_t q_axes;

float mems_pressure;
float mems_humidity;
float mems_temperature;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static MOTION_SENSOR_Axes_t AccValue;
static MOTION_SENSOR_Axes_t GyrValue;
static MOTION_SENSOR_Axes_t MagValue;
static MOTION_SENSOR_Axes_t MagOffset;

static uint32_t MagTimeStamp = 0;
static uint8_t MagCalStatus = 0;

/* Private function prototypes -----------------------------------------------*/
static void MX_DataLogFusion_Init(void);
static void MX_DataLogFusion_Process(void);

static void Init_Sensors(void);
static void RTC_Handler();
static void FX_Data_Handler();
static void Accelero_Sensor_Handler();
static void Gyro_Sensor_Handler();
static void Magneto_Sensor_Handler();
static void Pressure_Sensor_Handler();
static void Humidity_Sensor_Handler();
static void Temperature_Sensor_Handler();
static void TIM_Config(uint32_t Freq);
static void DWT_Start(void);
static uint32_t DWT_Stop(void);


void MX_MEMS_Init(void)
{
  /* USER CODE BEGIN SV */

  /* USER CODE END SV */

  /* USER CODE BEGIN MEMS_Init_PreTreatment */

  /* USER CODE END MEMS_Init_PreTreatment */

  /* Initialize the peripherals and the MEMS components */

  MX_DataLogFusion_Init();

  /* USER CODE BEGIN MEMS_Init_PostTreatment */

  /* USER CODE END MEMS_Init_PostTreatment */
}

/*
 * LM background task
 */
void MX_MEMS_Process(void)
{
  /* USER CODE BEGIN MEMS_Process_PreTreatment */

  /* USER CODE END MEMS_Process_PreTreatment */

  MX_DataLogFusion_Process();

  /* USER CODE BEGIN MEMS_Process_PostTreatment */

  /* USER CODE END MEMS_Process_PostTreatment */
}

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  Period elapsed callback
 * @param  htim pointer to a TIM_HandleTypeDef structure that contains
 *              the configuration information for TIM module
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == BSP_IP_TIM_Handle.Instance)
  {
	  //do nothing!
  }
}

/**
 * @brief  Configures the current date
 * @param  y the year value to be set
 * @param  m the month value to be set
 * @param  d the day value to be set
 * @param  dw the day-week value to be set
 * @retval None
 */
void RTC_DateRegulate(uint8_t y, uint8_t m, uint8_t d, uint8_t dw)
{
  RTC_DateTypeDef sdatestructure;

  sdatestructure.Year    = y;
  sdatestructure.Month   = m;
  sdatestructure.Date    = d;
  sdatestructure.WeekDay = dw;

  if (HAL_RTC_SetDate(&hrtc, &sdatestructure, FORMAT_BIN) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
 * @brief  Configures the current time
 * @param  hh the hour value to be set
 * @param  mm the minute value to be set
 * @param  ss the second value to be set
 * @retval None
 */
void RTC_TimeRegulate(uint8_t hh, uint8_t mm, uint8_t ss)
{
  RTC_TimeTypeDef stimestructure;

  stimestructure.Hours          = hh;
  stimestructure.Minutes        = mm;
  stimestructure.Seconds        = ss;
  stimestructure.SubSeconds     = 0;
  stimestructure.TimeFormat     = RTC_HOURFORMAT12_AM;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

  if (HAL_RTC_SetTime(&hrtc, &stimestructure, FORMAT_BIN) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}


/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initialize the DataLogFusion application
  * @retval None
  */
static void MX_DataLogFusion_Init(void)
{
  float ans_float;

  /* Initialize LED */
  BSP_LED_Init(LED2);

  /* Initialize push button */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize Virtual COM Port */
  BSP_COM_Init(COM1);

  /* Initialize Timer */
  BSP_IP_TIM_Init();

  /* Configure Timer to run with desired algorithm frequency */
  TIM_Config(ALGO_FREQ);

  /* Initialize (disabled) Sensors */
  Init_Sensors();

  /* Sensor Fusion API initialization function */
  MotionFX_manager_init();

  /* LED Blink */
  BSP_LED_On(LED2);
  HAL_Delay(500);
  BSP_LED_Off(LED2);

  /* Enable magnetometer calibration */
  MotionFX_manager_MagCal_start(ALGO_PERIOD);

  /* Test if calibration data are available */
  MFX_MagCal_output_t mag_cal_test;
  MotionFX_MagCal_getParams(&mag_cal_test);

  /* If calibration data are available load HI coeficients */
  if (mag_cal_test.cal_quality == MFX_MAGCALGOOD)
  {
    ans_float = (mag_cal_test.hi_bias[0] * FROM_UT50_TO_MGAUSS);
    MagOffset.x = (int32_t)ans_float;
    ans_float = (mag_cal_test.hi_bias[1] * FROM_UT50_TO_MGAUSS);
    MagOffset.y = (int32_t)ans_float;
    ans_float = (mag_cal_test.hi_bias[2] * FROM_UT50_TO_MGAUSS);
    MagOffset.z = (int32_t)ans_float;

    MagCalStatus = 1;
  }


}

/**
  * @brief  Process of the DataLogFusion application
  * @retval None
  */
static void MX_DataLogFusion_Process(void)
{

   /* Reset magnetometer calibration value*/
   MagCalStatus = 0;
   MagOffset.x = 0;
   MagOffset.y = 0;
   MagOffset.z = 0;

   /* Enable magnetometer calibration */
   MotionFX_manager_MagCal_start(ALGO_PERIOD);


   RTC_Handler();
   Accelero_Sensor_Handler();
   Gyro_Sensor_Handler();
   Magneto_Sensor_Handler();
   Humidity_Sensor_Handler();
   Temperature_Sensor_Handler();
   Pressure_Sensor_Handler();

   /* Sensor Fusion specific part */
   FX_Data_Handler();

}

/**
 * @brief  Initialize all sensors
 * @param  None
 * @retval None
 */
static void Init_Sensors(void)
{
  BSP_SENSOR_ACC_Init();
  BSP_SENSOR_GYR_Init();
  BSP_SENSOR_MAG_Init();
  BSP_SENSOR_PRESS_Init();
  BSP_SENSOR_TEMP_Init();
  BSP_SENSOR_HUM_Init();

  /* Set accelerometer:
   *   - ODR >= 100Hz
   *   - FS   = <-2g, 2g>
   */
  BSP_SENSOR_ACC_SetOutputDataRate(100.0f);
  BSP_SENSOR_ACC_SetFullScale(2);

//  /* Set magnetometer:
//   *   - ODR >= 100Hz
//   *   - FS   = 50Gauss (always) // TODO: Valid for all magnetometers?
//   */
//  BSP_SENSOR_MAG_SetOutputDataRate(100.0f);


  BSP_SENSOR_ACC_Enable();
  BSP_SENSOR_GYR_Enable();
  BSP_SENSOR_MAG_Enable();
  BSP_SENSOR_PRESS_Enable();
  BSP_SENSOR_TEMP_Enable();
  BSP_SENSOR_HUM_Enable();

}

/**
 * @brief  Handles the time+date getting/sending
 * @retval None
 */
static void RTC_Handler()
{
  uint8_t sub_sec = 0;
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructure;
  uint32_t ans_uint32;
  int32_t ans_int32;
  uint32_t RtcSynchPrediv = hrtc.Init.SynchPrediv;

  (void)HAL_RTC_GetTime(&hrtc, &stimestructure, FORMAT_BIN);
  (void)HAL_RTC_GetDate(&hrtc, &sdatestructureget, FORMAT_BIN);

  /* To be MISRA C-2012 compliant the original calculation:
     sub_sec = ((((((int)RtcSynchPrediv) - ((int)stimestructure.SubSeconds)) * 100) / (RtcSynchPrediv + 1)) & 0xFF);
     has been split to separate expressions */
  ans_int32 = (RtcSynchPrediv - (int32_t)stimestructure.SubSeconds) * 100;
  ans_int32 /= RtcSynchPrediv + 1;
  ans_uint32 = (uint32_t)ans_int32 & 0xFFU;
  sub_sec = (uint8_t)ans_uint32;

  printf("Hour: %hu, Minutes: %hu Seconds: %hu, SubSeconds: %hu\r\n",
			(uint8_t) stimestructure.Hours, (uint8_t) stimestructure.Minutes,
			(uint8_t) stimestructure.Seconds, sub_sec);
}

/**
 * @brief  Sensor Fusion data handler
 * @param  Msg the Sensor Fusion data part of the stream
 * @retval None
 */
static void FX_Data_Handler()
{
  uint32_t elapsed_time_us = 0U;
  MFX_input_t data_in;
  MFX_input_t *pdata_in = &data_in;
  MFX_output_t data_out;
  MFX_output_t *pdata_out = &data_out;


  /* Convert angular velocity from [mdps] to [dps] */
  data_in.gyro[0] = (float)GyrValue.x * FROM_MDPS_TO_DPS;
  data_in.gyro[1] = (float)GyrValue.y * FROM_MDPS_TO_DPS;
  data_in.gyro[2] = (float)GyrValue.z * FROM_MDPS_TO_DPS;

  /* Convert acceleration from [mg] to [g] */
  data_in.acc[0] = (float)AccValue.x * FROM_MG_TO_G;
  data_in.acc[1] = (float)AccValue.y * FROM_MG_TO_G;
  data_in.acc[2] = (float)AccValue.z * FROM_MG_TO_G;

  /* Convert magnetic field intensity from [mGauss] to [uT / 50] */
  data_in.mag[0] = (float)MagValue.x * FROM_MGAUSS_TO_UT50;
  data_in.mag[1] = (float)MagValue.y * FROM_MGAUSS_TO_UT50;
  data_in.mag[2] = (float)MagValue.z * FROM_MGAUSS_TO_UT50;

  /* Run Sensor Fusion algorithm */
  BSP_LED_On(LED2);
  DWT_Start();
  MotionFX_manager_run(pdata_in, pdata_out, MOTION_FX_ENGINE_DELTATIME);
  elapsed_time_us = DWT_Stop();
  BSP_LED_Off(LED2);

  if (Enabled6X == 1U)
  {
	  q_axes.AXIS_Z = data_out.rotation_6X[0];
	  q_axes.AXIS_Y = data_out.rotation_6X[1];
	  q_axes.AXIS_X = data_out.rotation_6X[2];
  }
  else
  {
	  q_axes.AXIS_Z = data_out.rotation_9X[0];
	  q_axes.AXIS_Y = data_out.rotation_9X[1];
	  q_axes.AXIS_X = data_out.rotation_9X[2];
  }

	printf("qx: %ld, qy: %ld, qz: %ld\r\n", q_axes.AXIS_X, q_axes.AXIS_Y,
			q_axes.AXIS_Z);

  UNUSED(elapsed_time_us);

}

/**
 * @brief  Handles the ACC axes data getting/sending
 * @retval None
 */
static void Accelero_Sensor_Handler() {
	BSP_SENSOR_ACC_GetAxes(&AccValue);

	x_axes.AXIS_X = AccValue.x;
	x_axes.AXIS_Y = AccValue.y;
	x_axes.AXIS_Z = AccValue.z;
	printf("ax: %ld, ay: %ld, az: %ld\r\n", x_axes.AXIS_X,
			x_axes.AXIS_Y, x_axes.AXIS_Z);

}

/**
 * @brief  Handles the GYR axes data getting/sending
 * @retval None
 */
static void Gyro_Sensor_Handler() {
	BSP_SENSOR_GYR_GetAxes(&GyrValue);
	g_axes.AXIS_X = GyrValue.x;
	g_axes.AXIS_Y = GyrValue.y;
	g_axes.AXIS_Z = GyrValue.z;
	printf("gx: %ld, gy: %ld, gz: %ld\r\n", g_axes.AXIS_X, g_axes.AXIS_Y,
			g_axes.AXIS_Z);
}

/**
 * @brief  Handles the MAG axes data getting/sending
 * @retval None
 */
static void Magneto_Sensor_Handler()
{
  float ans_float;
  MFX_MagCal_input_t mag_data_in;
  MFX_MagCal_output_t mag_data_out;

  BSP_SENSOR_MAG_GetAxes(&MagValue);

  if (MagCalStatus == 0U)
  {
	  mag_data_in.mag[0] = (float)MagValue.x * FROM_MGAUSS_TO_UT50;
      mag_data_in.mag[1] = (float)MagValue.y * FROM_MGAUSS_TO_UT50;
      mag_data_in.mag[2] = (float)MagValue.z * FROM_MGAUSS_TO_UT50;

      mag_data_in.time_stamp = (int)MagTimeStamp;
      MagTimeStamp += (uint32_t)ALGO_PERIOD;

      MotionFX_manager_MagCal_run(&mag_data_in, &mag_data_out);

      if (mag_data_out.cal_quality == MFX_MAGCALGOOD)
      {
        MagCalStatus = 1;

        ans_float = (mag_data_out.hi_bias[0] * FROM_UT50_TO_MGAUSS);
        MagOffset.x = (int32_t)ans_float;
        ans_float = (mag_data_out.hi_bias[1] * FROM_UT50_TO_MGAUSS);
        MagOffset.y = (int32_t)ans_float;
        ans_float = (mag_data_out.hi_bias[2] * FROM_UT50_TO_MGAUSS);
        MagOffset.z = (int32_t)ans_float;

        /* Disable magnetometer calibration */
        MotionFX_manager_MagCal_stop(ALGO_PERIOD);
      }
    }

    MagValue.x = (int32_t)(MagValue.x - MagOffset.x);
    MagValue.y = (int32_t)(MagValue.y - MagOffset.y);
    MagValue.z = (int32_t)(MagValue.z - MagOffset.z);

	m_axes.AXIS_X = MagValue.x;
	m_axes.AXIS_Y = MagValue.y;
	m_axes.AXIS_Z = MagValue.z;
	printf("mx: %ld, my: %ld, mz: %ld\r\n", m_axes.AXIS_X, m_axes.AXIS_Y,
			m_axes.AXIS_Z);

}

/**
 * @brief  Handles the PRESS sensor data getting/sending.
 * @retval None
 */
static void Pressure_Sensor_Handler()
{
  BSP_SENSOR_PRESS_GetValue(&mems_pressure);
}

/**
 * @brief  Handles the TEMP axes data getting/sending
 * @retval None
 */
static void Temperature_Sensor_Handler()
{
  BSP_SENSOR_TEMP_GetValue(&mems_temperature);
}

/**
 * @brief  Handles the HUM axes data getting/sending
 * @retval None
 */
static void Humidity_Sensor_Handler()
{
  BSP_SENSOR_HUM_GetValue(&mems_humidity);
}



/**
 * @brief  Timer configuration
 * @param  Freq the desired Timer frequency
 * @retval None
 */
static void TIM_Config(uint32_t Freq)
{
  const uint32_t tim_counter_clock = 2000; /* TIM counter clock 2 kHz */
  uint32_t prescaler_value = (uint32_t)((SystemCoreClock / tim_counter_clock) - 1);
  uint32_t period = (tim_counter_clock / Freq) - 1;

  BSP_IP_TIM_Handle.Init.Prescaler         = prescaler_value;
  BSP_IP_TIM_Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  BSP_IP_TIM_Handle.Init.Period            = period;
  BSP_IP_TIM_Handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  BSP_IP_TIM_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&BSP_IP_TIM_Handle) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
 * @brief  Start counting clock cycles
 * @param  None
 * @retval None
 */
static void DWT_Start(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0; /* Clear count of clock cycles */
  DWT->CTRL |= 1;  /* Enable counter */
}

/**
 * @brief  Stop counting clock cycles and calculate elapsed time in [us]
 * @param  None
 * @retval Elapsed time in [us]
 */
static uint32_t DWT_Stop(void)
{
  volatile uint32_t cycles_count = 0U;
  uint32_t system_core_clock_mhz = 0U;

  DWT->CTRL = DWT->CTRL ^ 1;  /* Disable counter */
  cycles_count = DWT->CYCCNT; /* Read count of clock cycles */

  /* Calculate elapsed time in [us] */
  system_core_clock_mhz = SystemCoreClock / 1000000U;
  return cycles_count / system_core_clock_mhz;
}

#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
