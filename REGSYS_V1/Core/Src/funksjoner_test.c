/*
 * funksjoner_test.c
 *
 *  Created on: Feb 21, 2023
 *      Author: fridawagner
 */

#include <globvariabler_dek.h>
#include <funksjoner_manpu.h>
#include "main.h"

void sjekk_bryter(void){
	if (GPIOC->IDR & B1_Pin) { 			// Sjekker om bryteren er trykket inn
       if(!bryter_forrige_status) { 				// Var bryteren trykket inn sist kontrollsjekk
    	   bryter_forrige_status = 1;
    	   bryter_trykk = 1;      				// Nytt brytertrykk registrert
       }
	}
	else {
		bryter_forrige_status = 0;
	}


	TIM8 -> CCR1 = 1500;
}


void test_manpu_borstelos(void){
	if (bryter_trykk){
		PWM_bredde++;
		bryter_trykk = 0;

		switch(PWM_bredde){
		case 1:
			TIM8 -> CCR1 = 1900;
			break;
		case 2:
			TIM8 -> CCR1 = 1500;
			break;
		case 3:
			TIM8 -> CCR1 = 1100;
			break;
		case 4:
			TIM8 -> CCR1 = 1500;
			PWM_bredde=0;
			break;
		}
	}
}
