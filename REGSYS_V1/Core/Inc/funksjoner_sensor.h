/*
 * funksjoner_sensor.h
 *
 *  Created on: 14. apr. 2023
 *      Author: fridawagner
 */

#ifndef INC_FUNKSJONER_SENSOR_H_
#define INC_FUNKSJONER_SENSOR_H_

#include <typedef_structures.h>
#include "STTS75.h"

/// NB; klikke klikker ikke med deklarasjon av funksjon, med mindre blir kalt i annen fil en
// hvor den er definert. -> må dobbelsjekke at deklarasjon stemmer overens med definisjon,
// gått på en smell flere ganger her:))

//////FOR KRAFT
void konverter_id_kraft(void);

///// FOR SENSOR
void konverter_id_sensor(void);

///// FOR REGULERING
uint8_t sjekk_forste_null(int8_t var_naa,int8_t var_forrige);
float konverter_kg_paadrag(float kg);
void hiv_regulering_init(void);
void stamp_regulering_init(void);
void rull_regulering_init(void);
void hiv_regulering(void);
void stamp_regulering(void);
void rull_regulering(void);

// dersom skal oppdatere verdi i variabel/struct må sende pointer
// dersom bare skal bruke kan sende verdi uten å være pointer
void null_PID_var(PID_var *PID);
void beregn_generell_PID(PID_var *PID, PID_var_felles felles);
void beregn_reguleringbidrag(void);

///// OPPDATERING AV PARAMETRE
void oppdater_parameter(uint32_t param_id, float parameter);


#endif /* INC_FUNKSJONER_SENSOR_H_ */
