/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define AVANCE 	GPIO_PIN_SET
#define RECULE  GPIO_PIN_RESET
#define POURCENT 640
#define Seuil_Dist_4 1289 // corespond � 10 cm.
#define Seuil_Dist_3 1093
#define Seuil_Dist_1 1003
#define Seuil_Dist_2 1142
#define V1 38
#define V2 56
#define V3 76
#define Vmax 95
#define T_2_S 1000 //( pwm p�riode = 2 ms )
#define T_200_MS 100
#define T_2000_MS 1000
#define CKp_D 100  //80 Robot1
#define CKp_G 100  //80 Robot1
#define CKi_D 80  //50 Robot1
#define CKi_G 80  //50 Robot1
#define CKd_D 0
#define CKd_G 0
#define DELTA 0x50

enum CMDE {
	START,
	STOP,
	AVANT,
	ARRIERE,
	DROITE,
	GAUCHE,
	PARK,
	MOVPARK
};
volatile enum CMDE CMDE;
enum MODE {
	SLEEP, ACTIF, PARKMODE, GOPARK
};
volatile enum MODE Mode; // Mode du robot en cours (avancer / Reculer etc... ou MOVPARK / PARK)
volatile unsigned char New_CMDE = 0; // Si ya une nouvelle commande
volatile uint16_t Dist_ACS_1, Dist_ACS_2, Dist_ACS_3, Dist_ACS_4, VBat; // Valeurs ADC
volatile unsigned int Time = 0;
volatile unsigned int Tech = 0;
volatile unsigned int Tpark = 0;
volatile unsigned int Tmod = 100;
volatile unsigned int Trotation = 0;
volatile unsigned int T_sonar = 0; // Temps permettant de faire une mesure tous les X ms
uint16_t adc_buffer[10];
uint16_t Buff_Dist[8];
uint8_t BLUE_RX; // Buffer des commandes bluetooth recues
uint8_t ZIGBEE_RX; // ZIGBEE RECUES
char BLUE_TX[100];

uint16_t _DirG, _DirD, DirD, DirG; // Futures directions des chenilles D et G et les actuelles
uint16_t _CVitD, CVitD = 0; // Future vitesse D et actuelle
uint16_t _CVitG, CVitG = 0; // Future vitesse G et actuelle
uint16_t VitD, VitG, DistD, DistG;
uint16_t DistD_old = 0; // Ancienne distance parcourue par D
uint16_t DistG_old = 0;// Ancienne distance parcourue par G
int Cmde_VitD = 0;
int Cmde_VitG = 0;
unsigned long Dist_parcours = 0;
volatile uint32_t Dist_Obst;
uint32_t Dist_Obst_;
uint32_t Dist_Obst_cm;
uint32_t Dist;
uint8_t UNE_FOIS = 1;
uint32_t OV = 0;
uint32_t position_0 [3];
uint16_t zigbee_state;
volatile uint32_t distance_sonar = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void Gestion_Commandes(void);
void regulateur(void);
void controle(void);
void Calcul_Vit(void);
void ACS(void);
void HAL_GPIO_ACQ_SONAR(void);
void HAL_MOV_SERVO(void);
void park (void);
void addon(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  Dist_Obst = 0;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  	HAL_SuspendTick(); // suppresion des Tick interrupt pour le mode sleep.

  	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);  // Start PWM motor
  	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  	CMDE = STOP;
  	New_CMDE = 1;
  	HAL_TIM_Base_Start_IT(&htim2);  // Start IT sur font montant PWM
  	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  	HAL_UART_Receive_IT(&huart3, &BLUE_RX, 1);

  	HAL_ADC_Start_IT(&hadc1);
  	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1); // start interrupt TIM1 sur channel 1
  	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2); // start interrupt TIM1 sur channel 2 - sonar
  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);  //mettre trig_sonar en marchant
  	HAL_TIM_Base_Start_IT(&htim1);

  	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); //lancement de PWM servo moteur

  	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 2300);




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  	HAL_MOV_SERVO();


  while (1)
  {
	  Gestion_Commandes();
	  controle();
	  addon();
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}

/* USER CODE BEGIN 4 */
void Gestion_Commandes(void) {
	enum ETAT {
		VEILLE,
		ARRET,
		AV1,
		AV2,
		AV3,
		RV1,
		RV2,
		RV3,
		DV1,
		DV2,
		DV3,
		GV1,
		GV2,
		GV3
	};
	static enum ETAT Etat = VEILLE;

if (New_CMDE) {
		New_CMDE = 0;
	switch (CMDE) {
		case STOP: {
			_CVitD = _CVitG = 0;
			// Mise en sommeil: STOP mode , r�veil via IT BP1
			Etat = VEILLE;
			Mode = SLEEP;

			break;
		}
		case START: {
			// r�veil syt�me grace � l'IT BP1
			Etat = ARRET;
			Mode = SLEEP;

			break;
		}
		case AVANT: {
			switch (Etat) {
			case VEILLE: {
				Etat = VEILLE;
				Mode = SLEEP;
				break;
			}
			case ARRET: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = AV1;
				Mode = ACTIF;
				break;
			}
			case AV1: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = AV2;
				Mode = ACTIF;
				break;
			}
			case AV2: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3 ;
				Etat = AV3;
				Mode = ACTIF;
				break;
			}
			case AV3: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3 ;
				Etat = AV3;
				Mode = ACTIF;
				break;
			}
			case RV1: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = 0;
				_CVitD = 0;
				Etat = ARRET;
				Mode = SLEEP;

				break;
			}
			case RV2: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = RV1;
				Mode = ACTIF;
				break;
			}
			case RV3: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = RV2;
				Mode = ACTIF;
				break;
			}
			case DV1: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = AV1;
				Mode = ACTIF;
				break;
			}
			case DV2: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = AV2;
				Mode = ACTIF;
				break;
			}
			case DV3: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = AV3;
				Mode = ACTIF;
				break;
			}
			case GV1: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = AV2;
				Mode = ACTIF;
				break;
			}
			case GV2: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = AV2;
				Mode = ACTIF;
				break;
			}
			case GV3: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = AV3;
				Mode = ACTIF;
				break;
			}
			}
			break;
		}
		case ARRIERE: {
			switch (Etat) {
			case VEILLE: {
				Etat = VEILLE;
				Mode = SLEEP;
				break;
			}
			case ARRET: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = RV1;
				Mode = ACTIF;
				break;
			}
			case AV1: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = 0;
				_CVitD = 0;
				Etat = ARRET;
				Mode = SLEEP;

				break;
			}
			case AV2: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = AV1;
				Mode = ACTIF;
				break;
			}
			case AV3: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = AV2;
				Mode = ACTIF;
				break;
			}
			case RV1: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = RV2;
				Mode = ACTIF;
				break;
			}
			case RV2: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = RV3;
				Mode = ACTIF;
				break;
			}
			case RV3: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = RV3;
				Mode = ACTIF;
				break;
			}
			case DV1: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = RV1;
				Mode = ACTIF;
				break;
			}
			case DV2: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = RV2;
				Mode = ACTIF;
				break;
			}
			case DV3: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = RV3;
				Mode = ACTIF;
				break;
			}
			case GV1: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = RV1;
				Mode = ACTIF;
				break;
			}
			case GV2: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = RV2;
				Mode = ACTIF;
				break;
			}
			case GV3: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = RV3;
				Mode = ACTIF;
				break;
			}
			}
			break;
		}
		case DROITE: {
			switch (Etat) {
			case VEILLE: {
				Etat = VEILLE;
				Mode = SLEEP;
				break;
			}
			case ARRET: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = DV1;
				Mode = ACTIF;
				break;
			}
			case AV1: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = DV1;
				Mode = ACTIF;
				break;
			}
			case AV2: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = DV2;
				Mode = ACTIF;
				break;
			}
			case AV3: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = DV3;
				Mode = ACTIF;
				break;
			}
			case RV1: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = DV1;
				Mode = ACTIF;
				break;
			}
			case RV2: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = DV2;
				Mode = ACTIF;
				break;
			}
			case RV3: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = DV3;
				Mode = ACTIF;
				break;
			}
			case DV1: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = DV2;
				Mode = ACTIF;
				break;
			}
			case DV2: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = DV3;
				Mode = ACTIF;
				break;
			}
			case DV3: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = DV3;
				Mode = ACTIF;
				break;
			}
			case GV1: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = 0;
				_CVitD = 0;
				Etat = ARRET;
				Mode = SLEEP;

				break;
			}
			case GV2: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = GV1;
				Mode = ACTIF;
				break;
			}
			case GV3: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = GV2;
				Mode = ACTIF;
				break;
			}
			}
			break;
		}
		case GAUCHE: {
			switch (Etat) {
			case VEILLE: {
				Etat = VEILLE;
				Mode = SLEEP;
				break;
			}
			case ARRET: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = GV1;
				Mode = ACTIF;
				break;
			}
			case AV1: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = GV1;
				Mode = ACTIF;
				break;
			}
			case AV2: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = GV2;
				Mode = ACTIF;
				break;
			}
			case AV3: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = GV3;
				Mode = ACTIF;
				break;
			}
			case RV1: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = GV1;
				Mode = ACTIF;
				break;
			}
			case RV2: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = GV2;
				Mode = ACTIF;
				break;
			}
			case RV3: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = GV3;
				Mode = ACTIF;
				break;
			}
			case DV1: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = 0;
				_CVitD = 0;
				Etat = ARRET;
				Mode = SLEEP;

				break;
			}
			case DV2: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = DV1;
				Mode = ACTIF;
				break;
			}
			case DV3: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = DV2;
				Mode = ACTIF;
				break;
			}
			case GV1: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = GV2;
				Mode = ACTIF;
				break;
			}
			case GV2: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = GV3;
				Mode = ACTIF;
				break;
			}
			case GV3: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = GV3;
				Mode = ACTIF;
				break;
			}
			}
			break;

		}
		case PARK: {
			Etat = ARRET;
			Mode = PARKMODE;
			break;

		}
		case MOVPARK : {
			Etat = ARRET;
			Mode = GOPARK;
			break;
		}
	}
}
}
void controle(void) {

	if (Tech >= T_200_MS) {
		Tech = 0;
		ACS();
		Calcul_Vit();
		regulateur();
	}

}


void ACS(void) {
	enum ETAT {
		ARRET, ACTIFE
	};
	static enum ETAT Etat = ARRET;
	static uint16_t Delta1 = 0;
	static uint16_t Delta2 = 0;
	static uint16_t Delta3 = 0;
	static uint16_t Delta4 = 0;

	switch (Etat) {
	case ARRET: {
		if (Mode == ACTIF )
			Etat = ACTIFE;
		else {
			CVitD = _CVitD;
			CVitG = _CVitG;
			DirD = _DirD;
			DirG = _DirG;
		}
		break;
	}
	case ACTIFE: {
		if (Mode == SLEEP)
			Etat = ARRET;
		if (_DirD == AVANCE && _DirG == AVANCE) {
			if ((Dist_ACS_1 < Seuil_Dist_1 - Delta1)
					&& (Dist_ACS_2 < Seuil_Dist_2 - Delta2)) {
				CVitD = _CVitD;
				CVitG = _CVitG;
				DirD = _DirD;
				DirG = _DirG;
				Delta1 = Delta2 = 0;
			} else if ((Dist_ACS_1 < Seuil_Dist_1)
					&& (Dist_ACS_2 > Seuil_Dist_2)) {
				CVitD = V1;
				CVitG = V1;
				DirG = AVANCE;
				DirD = RECULE;
				Delta2 = DELTA;
			} else if ((Dist_ACS_1 > Seuil_Dist_1)
					&& (Dist_ACS_2 < Seuil_Dist_2)) {
				CVitD = V1;
				CVitG = V1;
				DirD = AVANCE;
				DirG = RECULE;
				Delta1 = DELTA;
			} else if ((Dist_ACS_1 > Seuil_Dist_1)
					&& (Dist_ACS_2 > Seuil_Dist_2)) {
				CVitD = 0;
				CVitG = 0;
				DirD = RECULE;
				DirG = RECULE;
			}
		} else if (_DirD == RECULE && _DirG == RECULE) {
			if ((Dist_ACS_3 < Seuil_Dist_3 - Delta3)
					&& (Dist_ACS_4 < Seuil_Dist_4 - Delta4)) {
				CVitD = _CVitD;
				CVitG = _CVitG;
				DirD = _DirD;
				DirG = _DirG;
				Delta3 = Delta4 = 0;
			} else if ((Dist_ACS_3 > Seuil_Dist_3)
					&& (Dist_ACS_4 < Seuil_Dist_4)) {
				CVitD = V1;
				CVitG = V1;
				DirD = AVANCE;
				DirG = RECULE;
				Delta3 = DELTA;
			} else if ((Dist_ACS_3 < Seuil_Dist_3)
					&& (Dist_ACS_4 > Seuil_Dist_4)) {
				CVitD = V1;
				CVitG = V1;
				DirG = AVANCE;
				DirD = RECULE;
				Delta4 = DELTA;
			} else if ((Dist_ACS_3 > Seuil_Dist_3)
					&& (Dist_ACS_4 > Seuil_Dist_4)) {
				CVitD = 0;
				CVitG = 0;
				DirD = RECULE;
				DirG = RECULE;
			}
		} else {
			CVitD = _CVitD;
			CVitG = _CVitG;
			DirD = _DirD;
			DirG = _DirG;
		}
		break;
	}
	}
}

void Calcul_Vit(void) {
	static int first = 0;

	DistD = __HAL_TIM_GET_COUNTER(&htim3);
	DistG = __HAL_TIM_GET_COUNTER(&htim4);
	if(first == 0){
		DistD = 0;
		DistG = 0;
		first++;
	}
	VitD = abs(DistD - DistD_old);
	VitG = abs(DistG - DistG_old);
	DistD_old = DistD;
	DistG_old = DistG;
	if (DirD == DirG) {
		Dist_parcours = Dist_parcours + ((VitD + VitG) >> 1);
		int cx = snprintf(BLUE_TX, 100, "Dist %lu\n", Dist_parcours);
		HAL_UART_Transmit(&huart3, (uint8_t*) BLUE_TX, cx, HAL_MAX_DELAY);
	}
}

void regulateur(void) {
	enum ETAT {
		ARRET, ACTIFE
	};
	static enum ETAT Etat = ARRET;
	uint16_t Kp_D = CKp_D;
	uint16_t Kp_G = CKp_G;
	uint16_t Ki_D = CKi_D;
	uint16_t Ki_G = CKi_G;
	uint16_t Kd_D = CKd_D;
	uint16_t Kd_G = CKd_G;

	static int16_t ErreurD = 0;
	static int16_t ErreurG = 0;
	static int16_t ErreurD_old = 0;
	static int16_t ErreurG_old = 0;
	static int16_t S_erreursD = 0;
	static int16_t S_erreursG = 0;
	static int16_t V_erreurD = 0;
	static int16_t V_erreurG = 0;

	switch (Etat) {
	case ARRET: {
		if (Mode == ACTIF || Mode == PARKMODE)
			Etat = ACTIFE;
		else {
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(IR3_out_GPIO_Port, IR3_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IR4_out_GPIO_Port, IR4_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IR1_out_GPIO_Port, IR1_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IR2_out_GPIO_Port, IR2_out_Pin, GPIO_PIN_RESET);

			HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON,
					PWR_SLEEPENTRY_WFI);

			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
			Time = 0;
		}
		break;
	}
	case ACTIFE: {
		if ((CVitD != 0) && (CVitG != 0))
			Time = 0;
		if ((Mode == SLEEP) && (VitD == 0) && (VitG == 0) && Time > T_2_S)
			Etat = ARRET;
		else {
			ErreurD = CVitD - VitD;
			ErreurG = CVitG - VitG;
			S_erreursD += ErreurD;
			S_erreursG += ErreurG;
			V_erreurD = ErreurD - ErreurD_old;
			V_erreurG = ErreurG - ErreurG_old;
			ErreurD_old = ErreurD;
			ErreurG_old = ErreurG;
			Cmde_VitD = (unsigned int) Kp_D * (int) (ErreurD)
					+ (unsigned int) Ki_D * ((int) S_erreursD)
					+ (unsigned int) Kd_D * (int) V_erreurD;
			Cmde_VitG = (unsigned int) Kp_G * (int) (ErreurG)
					+ (unsigned int) Ki_G * ((int) S_erreursG)
					+ (unsigned int) Kd_G * (int) V_erreurG;

			//Cmde_VitD = _CVitD*640;
			//Cmde_VitG = _CVitG*640;
			//	DirD = _DirD;
			//	DirG= _DirG;

			if (Cmde_VitD < 0)
				Cmde_VitD = 0;
			if (Cmde_VitG < 0)
				Cmde_VitG = 0;
			if (Cmde_VitD > 100 * POURCENT)
				Cmde_VitD = 100 * POURCENT;
			if (Cmde_VitG > 100 * POURCENT)
				Cmde_VitG = 100 * POURCENT;
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint16_t ) Cmde_VitG);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (uint16_t ) Cmde_VitD);
			HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, (GPIO_PinState) DirD);
			HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, (GPIO_PinState) DirG);

		}
		break;
	}
	}
}

void park(void) {
	enum ETAT {
			ARRET, SERVO_X0, MESURE_X0, VAL_X0, SERVO_Y0, MESURE_Y0, VAL_Y0, SERVO_Z0, MESURE_Z0, VAL_Z0, SEND_ZIGBEE
		};
	static enum ETAT Etat = ARRET;


	switch(Etat) {
		case ARRET : {
			if(Mode == PARKMODE) { // Si on est dans le parkmode on continue
				Etat = SERVO_X0;
				HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_4); // Arrete pour eviter les bugs
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0); // Set to X0
				distance_sonar = 0;
				Dist_parcours = 0;
				Tmod = 100;
			}/*else{
				// AECRIRE
			}*/

			break;
		}

		case SERVO_X0 : {
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_4); // Arrete pour eviter les bugs
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 7100); // Set to X0
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); //lancement de PWM servo moteur

			Etat = MESURE_X0;
			Tmod = 100;
			int cx = snprintf(BLUE_TX, 100, "Etat suivant : MESURE_X0\n");
			HAL_UART_Transmit(&huart3, (uint8_t*) BLUE_TX, cx, HAL_MAX_DELAY);
			break;
		}
		case MESURE_X0 : {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
			Etat = VAL_X0;
			int cx = snprintf(BLUE_TX, 100, "Etat suivant : VAL_X0\n");
			HAL_UART_Transmit(&huart3, (uint8_t*) BLUE_TX, cx, HAL_MAX_DELAY);
			break;
		}
		case VAL_X0 : {
			while(distance_sonar == 0);
			position_0[0] = distance_sonar;
			distance_sonar = 0;
			Etat = SERVO_Y0;
			Tmod = 500;
			int cx = snprintf(BLUE_TX, 100, "Etat suivant : SERVO_Y0\n");
			HAL_UART_Transmit(&huart3, (uint8_t*) BLUE_TX,cx, HAL_MAX_DELAY);
			break;
		}
		case SERVO_Y0 : {
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_4); // Arrete pour eviter les bugs
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 2300); // Set to Y0
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); //lancement de PWM servo moteur

			Etat = MESURE_Y0;
			Tmod = 100;
			int cx = snprintf(BLUE_TX, 100, "Etat suivant : MESURE_Y0\n");
			HAL_UART_Transmit(&huart3, (uint8_t*) BLUE_TX,cx, HAL_MAX_DELAY);
			break;

		}
		case MESURE_Y0 : {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
			Etat = VAL_Y0;
			int cx = snprintf(BLUE_TX, 100, "Etat suivant : VAL_Y0\n");
			HAL_UART_Transmit(&huart3, (uint8_t*) BLUE_TX,cx, HAL_MAX_DELAY);
			break;
		}
		case VAL_Y0 : {
			while(distance_sonar == 0);
			position_0[1] = distance_sonar;
			distance_sonar = 0;
			Etat = SERVO_Z0;
			Tmod = 500;
			int cx = snprintf(BLUE_TX, 100, "Etat suivant : SERVO_Z0\n");
			HAL_UART_Transmit(&huart3, (uint8_t*) BLUE_TX, cx, HAL_MAX_DELAY);
			break;
		}
		case SERVO_Z0 : {
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_4); // Arrete pour eviter les bugs
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 800); // Set to Z0
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); //lancement de PWM servo moteur

			Etat = MESURE_Z0;
			Tmod = 100;
			int cx = snprintf(BLUE_TX, 100, "Etat suivant : MESURE_Z0\n");
			HAL_UART_Transmit(&huart3, (uint8_t*) BLUE_TX, cx, HAL_MAX_DELAY);
			break;

		}
		case MESURE_Z0 : {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
			Etat = VAL_Z0;
			int cx = snprintf(BLUE_TX, 100, "Etat suivant : VAL_Z0\n");
			HAL_UART_Transmit(&huart3, (uint8_t*) BLUE_TX, cx, HAL_MAX_DELAY);
			break;
		}
		case VAL_Z0 : {
			while(distance_sonar == 0);
			position_0[2] = distance_sonar;
			distance_sonar = 0;
			Etat = SEND_ZIGBEE;
			int cx = snprintf(BLUE_TX, 100, "Etat suivant : SEND_ZIGBEE\n");
			HAL_UART_Transmit(&huart3, (uint8_t*) BLUE_TX, cx, HAL_MAX_DELAY);
			break;
		}
		case SEND_ZIGBEE : {
			if(zigbee_state == 1){

			}else{

			}
			Etat = ARRET;
			Mode = STOP;
			int cx = snprintf(BLUE_TX, 100, "Etat suivant : FINI\n");
			HAL_UART_Transmit(&huart3, (uint8_t*) BLUE_TX, cx, HAL_MAX_DELAY);
		}
	}
}

void attentePark(void) {
	enum ETAT {
			ARRET, AVANCE_1, ROTATION_ANTIHORAIRE, SERVO_CENTRE, MESURE_1, VAL_1, RECULE_1, ROTATION_HORAIRE, MESURE_2, VAL_2, AVANCE_FINAL
		};
	static enum ETAT Etat = ARRET;

	switch(Etat) {
		case ARRET : {
			if(Mode == GOPARK) {
				Etat = AVANCE_1;
			}
		case AVANCE_1 : {
			_DirD = AVANCE;
			_DirG = AVANCE;
			_CVitD = V1;
			_CVitG = V1;
			if(Dist_parcours >= 527) {
				_CVitD = _CVitG = 0;
				TRotation = 0;
				Etat = ROTATION_ANTIHORAIRE;
			}
		case ROTATION_ANTIHORAIRE : {
			_DirD = AVANCE;
			_DirG = RECULE;
			_CVitD = _CVitG = V1;
			if(respectTime(TRotation, T_2_S)) {
				_CVitD = _CVitG = 0;
				Etat = SERVO_CENTRE;
			}
		}


		}
		}
	}

}

void addon(void) {// Addon = controleur + ts
	if (Tpark >= Tmod) { // Periode 200ms d'actualisation
		Tpark= 0;
		park();
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if (huart->Instance == USART3) {

		switch (BLUE_RX) {
		case 'F': {
			CMDE = AVANT;
			//New_CMDE = 1;
			break;
		}

		case 'V': {
			CMDE = STOP;
			New_CMDE = 1;
			break;
		}

		case 'L': {
			CMDE = GAUCHE;
			//New_CMDE = 1;
			break;
		}

		case 'R': {
			CMDE = DROITE;
			//New_CMDE = 1;
			break;
		}

		case 'D':{
			// disconnect bluetooth
			break;
		}
		case 'W' : {
			CMDE = PARK;
			New_CMDE = 1;
			break;
		}
		case 'w' : {
			CMDE = PARK;
			New_CMDE = 1;
			break;
				}
		case 'U' : {
			CMDE = MOVPARK;
			New_CMDE = 1;
			break;
				}
		case 'u' : {
			CMDE = MOVPARK;
			New_CMDE = 1;
			break;
				}
		default:
			New_CMDE = 1;
		}
		HAL_UART_Receive_IT(&huart3, &BLUE_RX, 1);


	}/*else if (huart->Instance == USART1) {

		switch (ZIGBEE_RX) {
		case 'F': {
			CMDE = AVANT;
			//New_CMDE = 1;
			break;
		}

		case 'B': {
			CMDE = ARRIERE;
			//New_CMDE = 1;
			break;
		}

		case 'L': {
			CMDE = GAUCHE;
			//New_CMDE = 1;
			break;
		}

		case 'R': {
			CMDE = DROITE;
			//New_CMDE = 1;
			break;
		}

		case 'D':{
			// disconnect bluetooth
			break;
		}
		//default:
		}
		//HAL_UART_Receive_IT(&huart1, &ZIGBEE_RX, 1);


	}*/
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {

	Dist_ACS_3 = adc_buffer[0] - adc_buffer[5];
	Dist_ACS_4 = adc_buffer[3] - adc_buffer[8];
	Dist_ACS_1 = adc_buffer[1] - adc_buffer[6];
	Dist_ACS_2 = adc_buffer[2] - adc_buffer[7];
	VBat = (adc_buffer[4]+adc_buffer[9])/2;
	HAL_ADC_Stop_DMA(hadc);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim) {
	static unsigned char cpt = 0;

	if ( htim->Instance == TIM2) {
		cpt++;
		Time++;
		T_sonar++;
		Tech++;
		Tpark++;
		TRotation ++;

		switch (cpt) {
		case 1: {
			HAL_GPIO_WritePin(IR3_out_GPIO_Port, IR3_out_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(IR4_out_GPIO_Port, IR4_out_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(IR1_out_GPIO_Port, IR1_out_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(IR2_out_GPIO_Port, IR2_out_Pin, GPIO_PIN_SET);
			break;
		}
		case 2: {
			HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc_buffer, 10);
			break;
		}
		case 3: {
			HAL_GPIO_WritePin(IR3_out_GPIO_Port, IR3_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IR4_out_GPIO_Port, IR4_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IR1_out_GPIO_Port, IR1_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IR2_out_GPIO_Port, IR2_out_Pin, GPIO_PIN_RESET);
			break;
		}
		case 4: {
			HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc_buffer, 10);
			break;
		}
		default:
			cpt = 0;
		}
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) { // On est dans le falling-edge

		distance_sonar = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_2); // On obtient la distance en (cm)
		/*
		 * Chaque tick est d�clench� a chaque 56,25 us (64MHz + pre de 36)
		 * sachant que l'on a au plus 37,7 ms de distance, ca donne 667 ticks max.
		 * Sachant donc que 667 correspond a 650 cm alors x cm correspondent a ?
		 * distance(cm) =  x*650/667
		 */


		int cx = snprintf(BLUE_TX, 100, "Distance Sonar : %d\n", (int)distance_sonar);
		HAL_UART_Transmit(&huart3, (uint8_t*) BLUE_TX, cx, HAL_MAX_DELAY);
	}

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	static unsigned char TOGGLE = 0;

	if (TOGGLE) {
		CMDE = STOP;
		int cx = snprintf(BLUE_TX, 100, "Robot STOP\n");
		HAL_UART_Transmit(&huart3, (uint8_t*) BLUE_TX, cx, HAL_MAX_DELAY);
	}else {
		CMDE = START;
		int cx = snprintf(BLUE_TX, 100, "Robot START\n");
		HAL_UART_Transmit(&huart3, (uint8_t*) BLUE_TX, cx, HAL_MAX_DELAY);
	}

	TOGGLE = ~TOGGLE;
	New_CMDE = 1;

}

void HAL_GPIO_ACQ_SONAR(void) {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
}
void HAL_MOV_SERVO(void) {
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 7100);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 2300);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 800);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef * hadc) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
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
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}

#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
