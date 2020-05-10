/* USER CODE BEGIN Header */
/**
* @file main.h
* @author IMSI2020
* @date 09 05 2020
* @version: 1.0
*/
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include "hci_tl_interface.h"
#include "stm32f4xx_nucleo.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gatt_db.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
void   MX_TIM3_Init(void);
/* USER CODE BEGIN Private defines */

#define BUFFER_LEN 1000		/*!< Dimensione del buffer circolare: MemBuffer */

/**
  * @brief Record MEMS
  *
  * Struttura dati contenente i valori acquisiti dai sensori MEMS e da inviare
  * all'applicazione tramite BLE.
  */
typedef struct {
	int32_t temperature;	/*!< Temperatura ([0;60] +- 1) [°C] */
	int32_t humidity;		/*!< Umidità ([0;100] +- 5) [% rH] */
	int32_t pressure;		/*!< Pressione ([260;1260] +- 1) [hPa] */
	int32_t exten1;		/*!< Estensimetro ADC ch10*/
	int32_t exten2;		/*!< Estensimetro ADC ch11*/
	AxesRaw_t x_axes;		/*!< Accelerometro ([-2g;+2g] +- 40) [mg] */
	AxesRaw_t q_axes;		/*!< Quaternioni*/
} MEMS_Record_t;

/**
  * @brief Buffer circolare MEMS
  *
  * Buffer circolare di elementi MEMS_Record_t di dimensione BUFFER_LEN.
  */
typedef MEMS_Record_t MEMS_buffer_t[BUFFER_LEN];
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
