/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <funksjoner_test.h>
#include <funksjoner_thruster.h>
#include <funksjoner_manpu.h>
#include <funksjoner_sensor.h>

#include <globvariabler_dek.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern FDCAN_HandleTypeDef hfdcan1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

	///////////// FOR TESTING

	static uint32_t teller_bryter = 0;
	teller_bryter++;
	if (teller_bryter >= 10){
		teller_bryter = 0;
		sjekk_bryter();		// Sjekker om det har vært et brukertrykk siden sist sjekk.
	}

	///////////// FEILSJEKKING

	static uint32_t teller_siden_marco = 0;
	teller_siden_marco++;

	// MARCO blir aldri satt til 0 noe sted, løkke >= 3000 vil aldri være sann
	if (MARCO){
		teller_siden_marco = 0;
	}

	if (teller_siden_marco >= 3000){
		// sette alle motorer til idle dersom mister kontakt med topside
		null_bidrag_horisontal(&sum_bidrag);
		null_bidrag_vertikal(&sum_bidrag);
		konverter_sum_bidrag_PWM();
		send_thruster_PWM();
	}


	// teller for om CAN styring feiler, denne telleren skal ikke nå et alt for høyt nivå dersom
	// får thrust meldinger kontinuerlig, vil da bli satt til 0 for hver melding.
	static uint32_t teller_feil_CAN_styring = 0;
	teller_feil_CAN_styring++;

	// dersom når feks 60ms, vil si at CAN styring har feilet, ønsker da at alle paadrag skal settes
	// til 0, siden har mistet kontroll over ROV-en

	if(teller_feil_CAN_styring >= 3000){
		// CAN THRUSTER FEILET
		null_bidrag_horisontal(&sum_bidrag);
		null_bidrag_vertikal(&sum_bidrag);
		konverter_sum_bidrag_PWM();
		send_thruster_PWM();

	}


	static uint32_t teller_feil_CAN_manpu = 0;
	teller_feil_CAN_manpu++;

	if(teller_feil_CAN_manpu >= 250){
		// NULL ALLE MANIPULATOR LEDD
	}


	////////// STYRING

	if (MOTTATT_THRUST){
		static uint32_t teller_timeout_send;
		teller_timeout_send++;

		// dersom gått for lang tid, noe som tilsier at sensor CAN kan ha sviktet, eller flagg for
		// deaktivering av regulering er satt, sett flagg for å gå videre til å utføre sjekker og
		// sette PWM.
		// & "OG" med 0x01 vill nulstille alle andre bit enn bit 0.

		// siden REG_AKTIVER initialisert som 0, vil ikke reguleringen starte med en gang
		// roven skrus på, må sendes aktiveringsmelding først.
		// !(REG_AKTIVER & 0x01) -> dersom bit 0 er 0, ingen regulering
		// !(REG_AKTIVER & 0x01)

		//|| !(REG_AKTIVER & 0x01
		if ( (teller_timeout_send >= 250)){
			SUM_BIDRAG_STYRING = 1;

			teller_timeout_send = 0;
			// hopp ut av if
			MOTTATT_THRUST = 0;

			//har mottatt thrust melding en gang dersom her, setter teller for feil i styremelding
			//til 0.
			teller_feil_CAN_styring = 0;
		}
		// else if slik at denne ikke vil bli testet. Dersom if blir utført i tillegg(vil da overskrive forrige sum_bidrag)

		else if (MOTTATT_SENSOR){
			SUM_BIDRAG_INKL_REG = 1;

			teller_timeout_send = 0;
			MOTTATT_SENSOR = 0;
			MOTTATT_THRUST= 0;

			teller_feil_CAN_styring = 0;
		}
	}

	///////////// SENDING OVER CAN

	static uint32_t teller_temp_topside = 0;
	teller_temp_topside++;

	// både hente temperatursensordata og sende til topside på 500ms går fint ()
	if (teller_temp_topside >= 500){ // 500 ms
		teller_temp_topside = 0;
		LES_SEND_TEMP = 1;

		//teller_ID_129 = 0;
	}

	static uint32_t teller_thrust_topside = 0;
	teller_thrust_topside++;
	if (teller_thrust_topside >= 100){
		teller_thrust_topside = 0;
		THRUST_TOPSIDE = 1;
	}



  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles FDCAN1 interrupt 0.
  */
void FDCAN1_IT0_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 0 */

  /* USER CODE END FDCAN1_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan1);
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 1 */

  /* USER CODE END FDCAN1_IT0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
