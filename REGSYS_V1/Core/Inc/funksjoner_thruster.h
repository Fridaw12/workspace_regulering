/*
 * funksjoner_thruster.h
 *
 *  Created on: Mar 25, 2023
 *      Author: fridawagner
 */

#ifndef INC_FUNKSJONER_THRUSTER_H_
#define INC_FUNKSJONER_THRUSTER_H_

#include <typedef_structures.h>

///// FOR THURSTER

// void beregn_generell_PID(PID_var var, PID_var_felles fel);

void null_bidrag_horisontal(thruster_struct *bidrag);
void null_bidrag_vertikal(thruster_struct *bidrag);
//void beregn_generell_PID(PID_var var, PID_var_felles fel);

void null_bidrag_alle_regvar(void);

void beregning_operatorbidrag(void);
void beregn_sum_bidrag(void);

void skalere_ned_saturation(float grense_variabel);
int8_t sjekk_saturation(void);

void horisontal_demping(void);
void vertikal_demping(void);
int differanse_sjekk(float var_forrige, float var_naa,float begrensing);
int horisontal_differanse(void);
int vertikal_differanse(void);

int effekt_sjekk_hoyre(void);
int effekt_sjekk_venstre(void);
void effekt_begrensing();


int16_t konverter_paadrag_PWM(float paadrag);
void konverter_sum_bidrag_PWM(void);
void send_thruster_PWM(void);
void thruster_data_lagring(void);
void behandle_sum_bidrag(void);

#endif /* INC_FUNKSJONER_THRUSTER_H_ */
