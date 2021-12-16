/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

	/// high-bandwidth 3-phase motor control for robots
	/// Written by Ben Katz, with much inspiration from Bayley Wang, Nick Kirkby, Shane Colton, David Otten, and others
	/// Hardware documentation can be found at build-its.blogspot.com

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
	#include "structs.h"
	#include <stdio.h>
	#include <string.h>

	#include "stm32f4xx_flash.h"
	#include "flash_writer.h"
	#include "position_sensor.h"
	#include "preference_writer.h"
	#include "hw_config.h"
	#include "user_config.h"
	#include "fsm.h"
	#include "drv8323.h"
	#include "foc.h"
	#include "math_ops.h"
	#include "calibration.h"
	#include "observer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
	#define VERSION_NUM 2.1f
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
	/* Flash Registers */
	float __float_reg[64];
	int __int_reg[256];
	PreferenceWriter prefs;

	int count = 0;

	/* Structs for control, etc */

	ControllerStruct controller;
	ObserverStruct observer;
	COMStruct com;
	FSMStruct state;
	EncoderStruct comm_encoder;
	DRVStruct drv;
	CalStruct comm_encoder_cal;
	CANTxMessage can_tx;
	CANRxMessage can_rx;

	/* init but don't allocate calibration arrays */
	int *error_array = NULL;
	int *lut_array = NULL;

	static uint8_t Serial2RxBuffer[1];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_CAN1_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  	/* Time measurement counter */
//  	HAL_TIM_Base_Start(&htim2);
   HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_2);
	/* Load settings from flash */
	preference_writer_init(&prefs, 6);
	preference_writer_load(prefs);

	/* Sanitize configs in case flash is empty*/
	if(E_ZERO==-1){E_ZERO = 0;}
	if(M_ZERO==-1){M_ZERO = 0;}
	if(isnan(I_BW) || I_BW==-1){I_BW = 1000;}
	if(isnan(I_MAX) || I_MAX ==-1){I_MAX=30;}
	if(isnan(I_FW_MAX) || I_FW_MAX ==-1){I_FW_MAX=0;}
	if(CAN_ID==-1){CAN_ID = 1;}
	if(CAN_MASTER==-1){CAN_MASTER = 0;}
	if(CAN_TIMEOUT==-1){CAN_TIMEOUT = 1000;}
	if(isnan(R_NOMINAL) || R_NOMINAL==-1){R_NOMINAL = 0.0f;}
	if(isnan(TEMP_MAX) || TEMP_MAX==-1){TEMP_MAX = 125.0f;}
	if(isnan(I_MAX_CONT) || I_MAX_CONT==-1){I_MAX_CONT = 14.0f;}
	if(isnan(I_CAL)||I_CAL==-1){I_CAL = 3.0f;}
	if(isnan(PPAIRS) || PPAIRS==-1){PPAIRS = 28.0f;}
	if(isnan(GR) || GR==-1){GR = 1.0f;}
	if(isnan(KT) || KT==-1){KT = 1.0f;}
	if(isnan(KP_MAX) || KP_MAX==-1){KP_MAX = 500.0f;}
	if(isnan(KD_MAX) || KD_MAX==-1){KD_MAX = 5.0f;}
	if(isnan(P_MAX)){P_MAX = 12.5f;}
	if(isnan(P_MIN)){P_MIN = -12.5f;}
	if(isnan(V_MAX)){V_MAX = 65.0f;}
	if(isnan(V_MIN)){V_MIN = -65.0f;}

	printf("\r\nFirmware Version Number: %.2f\r\n", VERSION_NUM);

	/* Controller Setup */
	// 	  if(PHASE_ORDER){							// Timer channel to phase mapping
	//
	// 	  }
	// 	  else{
	//
	// 	  }

	init_controller_params(&controller);

	/* calibration "encoder" zeroing */
	memset(&comm_encoder_cal.encoder_p, 0, sizeof(EncoderStruct));

	/* commutation encoder setup */
	comm_encoder.m_zero = M_ZERO;
	comm_encoder.e_zero = E_ZERO;
	comm_encoder.ppairs = PPAIRS;
	ps_warmup(&comm_encoder, 100);			// clear the noisy data when the encoder first turns on

	if(EN_ENC_LINEARIZATION){memcpy(&comm_encoder.offset_lut, &ENCODER_LUT, sizeof(comm_encoder.offset_lut));}	// Copy the linearization lookup table
	else{memset(&comm_encoder.offset_lut, 0, sizeof(comm_encoder.offset_lut));}
	//for(int i = 0; i<128; i++){printf("%d\r\n", comm_encoder.offset_lut[i]);}


 	/* ADCs and DMA configuration */
	if(HAL_ADC_Start(&hadc2)!= HAL_OK)
	{
	  Error_Handler();
	}
	if(HAL_ADC_Start(&hadc3)!= HAL_OK)
	{
	  Error_Handler();
	}

	/* By calling this function - MultiMode will be triggered and DMA Transfer interrupt should be expected
	 * More details: It starts ADC1, triggers start of the ADC2 and ADC3 which triggers DMA
	 * */
	if(HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)controller.adc_data, 3)!= HAL_OK)
	{
	  Error_Handler();
	}

	/* DRV8323 setup */

	HAL_GPIO_WritePin(ENABLE_PIN, GPIO_PIN_SET );
	HAL_Delay(1);
	//drv_calibrate(drv);
	HAL_Delay(1);
	drv_write_DCR(drv, 0x0, DIS_GDF_EN, 0x0, PWM_MODE_3X, 0x0, 0x0, 0x0, 0x0, 0x1);
	HAL_Delay(1);
	drv_write_CSACR(drv, 0x0, 0x1, 0x0, CSA_GAIN_40, 0x0, 0x1, 0x1, 0x1, SEN_LVL_1_0);
	HAL_Delay(10);
	zero_current(&controller);
	HAL_Delay(1);
	drv_write_CSACR(drv, 0x0, 0x1, 0x0, CSA_GAIN_40, 0x1, 0x0, 0x0, 0x0, SEN_LVL_1_0);
	HAL_Delay(1);
	drv_write_OCPCR(drv, TRETRY_50US, DEADTIME_50NS, OCP_NONE, OCP_DEG_8US, VDS_LVL_1_88); //TODO: increase deadtime
	HAL_Delay(1);
	drv_disable_gd(drv);
	HAL_Delay(10);
	// 	  uint16_t reg_check = drv_read_register(drv, DCR);
	// 	  drv_enable_gd(drv);
	printf("ADC A OFFSET: %d     ADC B OFFSET: %d\r\n", controller.adc_ch_i_offset[0], controller.adc_ch_i_offset[1]);

	/* Turn on PWM */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	/* CAN setup */
	CAN_User_Config(&CAN_H, &can_rx, &can_tx);

	/* Start the FSM */
	state.state = SETUP_MODE;
	state.next_state = MENU_MODE;
	state.ready = 1;


	/* Turn on interrupts */
	HAL_UART_Receive_IT(&huart2, (uint8_t *)Serial2RxBuffer, 1);
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC4);	// Sets the CC interrupt for channel 4
	HAL_TIM_Base_Start_IT(&htim1);	// Starts counter and enables interrupts
	reset_foc(&controller);
//	printf("Start program\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	/* Least important processing */
	drv_print_faults(drv);
	HAL_Delay(100);
//	if(state.state==MOTOR_MODE)
//	{
//		  printf("%.2f %.2f %.2f %.2f %.2f\r\n", controller.i_a, controller.i_b, controller.i_d, controller.i_q, controller.dtheta_elec);
//	}
	//	  printf("Can received p_des: %.3f\t  v_des: %.3f\t  kp: %.3f\t  kd: %.3f\t  t_ff: %.3f\t  i_q_ref: %.3f \n\r", controller.p_des, controller.v_des, controller.kp, controller.kd, controller.t_ff, controller.i_q_des);

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	/* Get RX message */
	if (hcan != &CAN_H){
		printf("Check can receiver!");
	}
	if (HAL_CAN_GetRxMessage(&CAN_H, CAN_RX_FIFO0, &can_rx.rx_header, can_rx.data) != HAL_OK)
	{
		/* Reception Error */
		Error_Handler();
	}

	// Read CAN
	uint32_t TxMailbox;
	// Data that can be sent also: controller.i_a, controller.i_b
	pack_reply(&can_tx, CAN_ID,  comm_encoder.angle_multiturn[N_POS_SAMPLES-1]/GR, comm_encoder.velocity, controller.i_q);	// Pack response
//	pack_reply(&can_tx, CAN_ID,  comm_encoder.angle_multiturn[N_POS_SAMPLES-1]/GR, controller.i_a, controller.i_b);
	HAL_CAN_AddTxMessage(&CAN_H, &can_tx.tx_header, can_tx.data, &TxMailbox);	// Send response

	/* Check for special Commands */
	if((can_rx.data[0]==0xFF) && (can_rx.data[1]==0xFF) && (can_rx.data[2]==0xFF) && (can_rx.data[3]==0xFF) && (can_rx.data[4]==0xFF) && (can_rx.data[5]==0xFF) && (can_rx.data[6]==0xFF))
	{
		controller.can_fsm_upd_req_flag = 1;
		switch (can_rx.data[7])
		{
		case 0xFC:
			controller.can_command = MOTOR_CMD;
			break;
		case 0xFD:
			controller.can_command = MENU_CMD;
			break;
		case 0xFE:
			controller.can_command = ZERO_CMD;
			break;		
		default:
			controller.can_fsm_upd_req_flag = 0; //No valid command received
			break;
		}
		if (controller.can_fsm_upd_req_flag)
		{
			update_fsm(&state, controller.can_command);
		}
	}
	else{
		unpack_cmd(can_rx, &controller);		// Unpack commands
		controller.timeout = 0;					// Reset timeout counter
	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	update_fsm(&state, (char) Serial2RxBuffer[0]);
  /* Prevent unused argument(s) compilation warning */
	HAL_UART_Receive_IT(&huart2, Serial2RxBuffer, 1);
}

void adc_data_ready_update(uint8_t val)
{
	controller.adc_data_ready_flag = val;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
