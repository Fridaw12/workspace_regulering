/*
 * globvariabler_dek.h
 *
 *  Created on: Feb 21, 2023
 *      Author: fridawagner
 *
 *  Declaration introduces an identifier and describes its type, be it a type, object, or function
 *  A declaration is what the compiler needs to accept references to that identifier.
 */


#ifndef INC_GLOBVARIABLER_DEK__H_
#define INC_GLOBVARIABLER_DEK__H_

#include "main.h"
#include "math.h"
#include <stdio.h>
#include <typedef_structures.h>


////////// FLAGG

extern uint8_t ENDRE_FREKVENS;
extern uint8_t REG_AKTIVER;
extern uint8_t REG_AKTIVER_DYBDE;

extern uint8_t MARCO;

extern uint8_t MOTTATT_THRUST;
extern uint8_t MOTTATT_MANIPULATOR;
extern uint8_t MOTTATT_SENSOR;

extern uint8_t SUM_BIDRAG_STYRING;
extern uint8_t SUM_BIDRAG_INKL_REG;

extern uint8_t THRUST_TOPSIDE;
extern uint8_t LES_SEND_TEMP;

////////// FLAGG REGULERING
extern uint8_t SETT_FORSTE_REF_HIV;
extern int16_t REF_HIV;


//////FOR THRUSTER STYRING

//////// Single parametre
extern float BEGRENSING;

/////// Struct diff parametre

extern motorkarakteristikk_parametre motorkar;
extern lin_modell lowpass_thruster,lowpass_manpu_steg,lowpass_manpu_borste,pos_paa_PWM,neg_paa_PWM,pos_kr_paa,neg_kr_paa;

////// struct int
extern thruster_struct_int8 sum_bidrag_int8;
extern thruster_struct_int16 sum_bidrag_PWM;
extern id_thrust_struct operator_referanse,thrust_sensitivitet;
extern id_thrust_struct_int operator_referanse_int, operator_referanse_int_forrige;

///// struct float
extern thruster_struct operatorbidrag, sum_bidrag, sum_bidrag_forrige1, sum_bidrag_forrige2 \
        ,reguleringbidrag_hiv,reguleringbidrag_stamp, reguleringbidrag_rull;



////////// FOR MANIPULATOR
extern id_manpu_struct operator_manpu, operator_manpu_forrige1, manpu_sensitivitet;
extern id_manpu_struct_int operator_manpu_int, operator_manpu_forrige1_int;
extern manpu_struct_uint16   manu_ARR, manu_CCR, manu_ARR_default;


///// FOR REGULERING
extern PID_var_felles PID_felles;
extern PID_var PID_hiv, PID_stamp, PID_rull;

/////////// FOR SENSOR
extern id_sensor_struct sensordata, REF;
extern id_sensor_struct_int sensordata_int;

////////// FOR KRAFT
extern demping_effekt d_effekt;
extern id_kraft_struct kraftdata;
extern id_kraft_struct_int kraftdata_int;


///////// FOR TESTING
///// testing CAN
extern uint16_t førsteuint8;
extern uint8_t teller_CAN;
extern uint8_t send_melding;
extern uint8_t ny_melding;
extern uint8_t segment_data[2];

////// testing m bryter
extern uint8_t bryter_trykk;
extern uint8_t bryter_forrige_status;
extern uint8_t bryter_antall;
extern uint8_t PWM_bredde;


/////// OPPDATER PARAMETRE

extern can_param ny_param;


#endif /* INC_GLOBVARIABLER_DEK__H_ */
