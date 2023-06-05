/*
 * funksjoner_manpu.h
 *
 *  Created on: 14. apr. 2023
 *      Author: fridawagner
 */

#ifndef INC_FUNKSJONER_MANPU_H_
#define INC_FUNKSJONER_MANPU_H_

#include <typedef_structures.h>


///// FOR MANIPULATOR

void sett_rotasjonsretning(float manpulator_variabel,GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
uint16_t beregn_ARR(float manpulator_variabel,float maks_frek, float sensitivitet);
uint16_t beregn_CCR_snurr(float manpulator_variabel,float maks_bredde, float sensitivitet);
uint8_t sjekk_for_forskjellig_fortegn(float var_naa,float var_forrige);
void manipulator_demping(void);

void behandle_gripe(void);
void behandle_sving(void);
void behandle_skyv(void);
void behandle_snurr(void);

void behandle_manipulator(void);


#endif /* INC_FUNKSJONER_MANPU_H_ */
