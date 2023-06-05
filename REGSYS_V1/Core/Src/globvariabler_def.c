/*
 * globvariabler_def.c
 *
 *  Created on: 25. mar. 2023
 *      Author: fridawagner
 *
 */


#include <main.h>
#include "typedef_structures.h"

///////////////  FLAGG


uint8_t ENDRE_FREKVENS = 1;
uint8_t REG_AKTIVER_DYBDE;

uint8_t REG_AKTIVER;

uint8_t MARCO;


uint8_t MOTTATT_THRUST = 0;
uint8_t MOTTATT_MANIPULATOR = 0;
uint8_t MOTTATT_SENSOR = 0;

uint8_t SUM_BIDRAG_STYRING = 0;
uint8_t SUM_BIDRAG_INKL_REG = 0;

uint8_t THRUST_TOPSIDE = 0;
uint8_t LES_SEND_TEMP = 0;

////////// FLAGG REGULERING
uint8_t SETT_FORSTE_REF_HIV ;
int16_t REF_HIV;


/////////////  FOR THRUSTER STYRING

////////////// Single parametre
/////// NBNB; husk å velge init verdi!!
float BEGRENSING = {40};

//////// diff parametre
motorkarakteristikk_parametre motorkar = {-100};
/// MÅ IKKE NEGATIV PWM HA NEGATIVT FORTEGN? -> fortegnet på x variabel er med
// og vil føre til negativt fortegn til a

//// NB ENDRE PÅ VAR FOR SAMMENGENG MELLOM KREFTER OG PAADRAG.. THRUSTERKARAKTERISTIKK
// a = 0.1 gav merkbar forandring

lin_modell lowpass_thruster={0.9,0},lowpass_manpu_steg={0.9,0},lowpass_manpu_borste={0.9,0},\

		pos_paa_PWM = {3.3,1500},neg_paa_PWM={3.3,1500},\
		pos_kr_paa = {35.0,0},neg_kr_paa = {35.0,0};

// pos_paa_PWM = {3.3,1570},neg_paa_PWM={3.3,1430}

/// struct int
thruster_struct_int8 sum_bidrag_int8;
thruster_struct_int16 sum_bidrag_PWM;
id_thrust_struct operator_referanse, thrust_sensitivitet = {1,1,0.5,1,0.5};
id_thrust_struct_int operator_referanse_int, operator_referanse_int_forrige;

/// struct float
thruster_struct operatorbidrag, sum_bidrag, sum_bidrag_forrige1, sum_bidrag_forrige2 \
        ,reguleringbidrag_hiv,reguleringbidrag_stamp, reguleringbidrag_rull;


//////// FOR MANIPULATOR
id_manpu_struct operator_manpu,operator_manpu_forrige1,manpu_sensitivitet = {1,1,1,1};
id_manpu_struct_int operator_manpu_int, operator_manpu_forrige1_int;
/// 5000 gir frekvens på 200Hz
manpu_struct_uint16 manu_ARR, manu_CCR, manu_ARR_default = {5000,5000,5000,5000};


//////// FOR REGULERING
PID_var_felles PID_felles =  {0.05,0.5};
// når CAN-frekvens = 20 Hz, setter Ts = 1/20 = 0.05 (endre dette når vet mer hvor lang tid andre ting tar gjennom syst)
// når Tf = 5*Ts , alpha = 1/6
PID_var PID_hiv, PID_stamp, PID_rull;


///////// FOR SENSOR
// definererer struct [gyrostruct "makro" laget i annen fil]

id_sensor_struct sensordata, REF = {0,0};
id_sensor_struct_int sensordata_int;

////////// FOR KRAFT
demping_effekt d_effekt;
id_kraft_struct kraftdata;
// NB: velg init kraftdata_int.EFFEKT_begrensing!! ..
id_kraft_struct_int kraftdata_int = {0,0};


///////// TESTING
///// for testing CAN
uint16_t førsteuint8;
uint8_t andreuint8;
uint8_t teller_ID_129;
uint8_t send_melding;
uint8_t ny_melding;

uint8_t segment_data[2] = {0x01,0x02};


//// for testing m bryter
uint8_t bryter_trykk;
uint8_t bryter_forrige_status;
uint8_t bryter_antall;
uint8_t PWM_bredde;


/////// OPPDATER PARAMETRE

can_param ny_param;



