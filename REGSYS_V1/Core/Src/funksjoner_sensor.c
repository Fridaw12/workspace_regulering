/*
 * funksjoner_sensor.c
 *
 *  Created on: 14. apr. 2023
 *      Author: fridawagner
 */

#include "main.h"
#include <typedef_structures.h>
#include <globvariabler_dek.h>
#include <funksjoner_thruster.h>
#include "math.h"
#include "STTS75.h"

//// NB: Data fra kommunikasjon sendes på 50 Hz,
//// Data til topside sendes på 10 Hz

//// temperatursensor leses på 500ms

/////////////////////////// REGULERING

void null_PID_var(PID_var *PID){

	 PID -> e_1 = 0;// lagre som forrige avvik
	 PID -> ui_1 = 0; // lagre som forrige integratorbidrag
	 PID -> YF_1 = 0; // lagre som forrige filtrerte verdi
}


void beregn_generell_PID(PID_var *PID, PID_var_felles felles){
    float up,ui,ud;
    (PID->e) = (PID->Ref) - (PID->Y); // beregne avvik, error e

    up = (PID->Kp) * (PID->e); // proportional action
    ui = PID->ui_1 + ( (PID->Ki * felles.Ts) * (PID->e + PID->e_1)/2 ); // intergal action

    if (ui > (PID->uimaks)){ // pga integral action akkumulering av error, setter begrensning; anti-windup
        ui = (PID->uimaks);
    }
    else if(ui < (PID->uimin)){
        ui = (PID->uimin);
    }

    PID->YF = ( felles.a * PID->YF_1 ) + ( (1-felles.a) * PID->Y); // filtere måleverdi for derivering

    ud = - PID->Kd * ( (PID->YF - PID->YF_1) / felles.Ts ); //derivative action

    PID->u = up + ui + ud;
    if (PID->u > (PID->umaks)){ // maksbegrensing
        PID->u = (PID->umaks);
    }
    else if(PID->u < (PID->umin)){
        PID->u = (PID->umin);
    }

    PID -> e_1 = PID -> e;// lagre som forrige avvik
    PID -> ui_1 = ui; // lagre som forrige integratorbidrag
    PID -> YF_1 = PID -> YF; // lagre som forrige filtrerte verdi
}

float konverter_kg_paadrag(float kg){
    if(kg<0) {return (pos_kr_paa.a*kg + pos_kr_paa.b);}
    else if(kg>0) {return (neg_kr_paa.a*kg + neg_kr_paa.b);}
    else {return 0;}
}

void hiv_regulering_init(void){
	// oppdater
	PID_hiv.Kp =  4; //1
	PID_hiv.Ki = 0.8; //2
	PID_hiv.Kd =  3.5; //3
	PID_hiv.Ref = 0;
	PID_hiv.u = 0;
	PID_hiv.ui_1 = 0;
	PID_hiv.e = 0;
	PID_hiv.e_1 = 0;
	PID_hiv.Y = 0;
	PID_hiv.YF = 0;
	PID_hiv.YF_1 = 0;
	// oppdater

	// med 70 går opp til ca 59 på hver
	// med 100 går opp til rundt 80 på hver
	PID_hiv.uimaks = 70;
	PID_hiv.uimin = -70;
	PID_hiv.umaks = 100;
	PID_hiv.umin = -100;
}

// kanskje litt aggresiv (?)

void rull_regulering_init(void){
	// oppdater
	PID_rull.Kp = 0.3; //1
	PID_rull.Ki = 0.2; //2
	PID_rull.Kd = 0.165; //3
	PID_rull.Ref = 0;
	PID_rull.u = 0;
	PID_rull.ui_1 = 0;
	PID_rull.e = 0;
	PID_rull.e_1 = 0;
	PID_rull.Y = 0;
	PID_rull.YF = 0;
	PID_rull.YF_1 = 0;
	// oppdater
	PID_rull.uimaks = 70;
	PID_rull.uimin = -70;
	PID_rull.umaks = 100;
	PID_rull.umin = -100;
}

void stamp_regulering_init(void){
	// oppdater
	PID_stamp.Kp =  0.33; //1
	PID_stamp.Ki = 0.2; //2
	PID_stamp.Kd =  0.162; //3
	PID_stamp.Ref = 0;
	PID_stamp.u = 0;
	PID_stamp.ui_1 = 0;
	PID_stamp.e = 0;
	PID_stamp.e_1 = 0;
	PID_stamp.Y = 0;
	PID_stamp.YF = 0;
	PID_stamp.YF_1 = 0;
	// oppdater
	PID_stamp.uimaks = 70;
	PID_stamp.uimin = -70;
	PID_stamp.umaks = 100;
	PID_stamp.umin = -100;
}


void hiv_regulering(void){

    PID_hiv.Y = sensordata.dybde;
    beregn_generell_PID(&PID_hiv, PID_felles);

    // fordel på 4 vertikale thrustere, og konverter til paadrag
    // fordeles likt

    reguleringbidrag_hiv.vhf = konverter_kg_paadrag(-PID_hiv.u);
    reguleringbidrag_hiv.vhb = konverter_kg_paadrag(-PID_hiv.u);
    reguleringbidrag_hiv.vvb = konverter_kg_paadrag(-PID_hiv.u);
    reguleringbidrag_hiv.vvf = konverter_kg_paadrag(-PID_hiv.u);
}

void stamp_regulering(void){

    PID_stamp.Ref = REF.stamp;
    PID_stamp.Y = sensordata.stamp;

    beregn_generell_PID(&PID_stamp, PID_felles);

    // fordel på 4 vertikale thrustere, fordeles likt på to grupper
    // hhb og hvb , hhf og hvf -> har motsatt fortegn av hverandre

    reguleringbidrag_stamp.vhf = konverter_kg_paadrag(-PID_stamp.u);
    reguleringbidrag_stamp.vhb = konverter_kg_paadrag(PID_stamp.u);
    reguleringbidrag_stamp.vvb = konverter_kg_paadrag(PID_stamp.u);
    reguleringbidrag_stamp.vvf = konverter_kg_paadrag(-PID_stamp.u);
}

void rull_regulering(void){

    PID_rull.Ref = REF.rull;
    PID_rull.Y = sensordata.rull;

    beregn_generell_PID(&PID_rull, PID_felles);

    // fordel på 4 vertikale thrustere, fordeles likt,
    // vhb og vhf , vvb og vvf

    reguleringbidrag_rull.vhf = konverter_kg_paadrag(-PID_rull.u);
    reguleringbidrag_rull.vhb = konverter_kg_paadrag(-PID_rull.u);
    reguleringbidrag_rull.vvb = konverter_kg_paadrag(PID_rull.u);
    reguleringbidrag_rull.vvf = konverter_kg_paadrag(PID_rull.u);

}


void beregn_reguleringbidrag(void){
	// konverter int til float for videre beregninger
	// "casting" (ny variabeltype)
	sensordata.rull = (float) sensordata_int.rull/100;
	sensordata.stamp = (float) sensordata_int.stamp/100;
	// egentlig være delt på 100, skal sende i cm men sender i mm
	sensordata.dybde = (float) sensordata_int.dybde/1000;
	// sjekke om reguleringsvar skal være aktiv eller ikke.
	// dersom 0 er ikke aktiv

	//////
	if(!(REG_AKTIVER & 0x01)){
		null_PID_var(&PID_rull);
		null_bidrag_horisontal(&reguleringbidrag_rull);
		null_bidrag_vertikal(&reguleringbidrag_rull);
	}
	else{
		rull_regulering();
	}
	//////
	if(!(REG_AKTIVER & 0x04)){
		null_PID_var(&PID_stamp);
		null_bidrag_horisontal(&reguleringbidrag_stamp);
		null_bidrag_vertikal(&reguleringbidrag_stamp);
	}
	else{
		stamp_regulering();
	}
	//////
	if(!(REG_AKTIVER_DYBDE & 0x08)){
		null_PID_var(&PID_hiv);
		null_bidrag_horisontal(&reguleringbidrag_hiv);
		null_bidrag_vertikal(&reguleringbidrag_hiv);
	}
	else{
		hiv_regulering();
	}

}



///// OPPDATERING AV PARAMETRE

void oppdater_parameter(uint32_t param_id, float parameter){
	switch(param_id){

///// fra id    til     : regulering av hiv parametre
	case 1:
		PID_rull.Ki = parameter;			break;
	case 2:
		PID_rull.Kd = parameter;			break;
	case 3:
		PID_rull.Kp = parameter;			break;
	case 4:
		PID_stamp.Ki = parameter;			break;
	case 5:
		PID_stamp.Kd = parameter;			break;
	case 6:
		PID_stamp.Kp = parameter;			break;
	case 7:
		PID_hiv.Ki = parameter;				break;
	case 8:
		PID_hiv.Kd = parameter;				break;
	case 9:
		PID_hiv.Kp = parameter;				break;
	case 10:
		PID_felles.Ts = parameter;			break;
	case 11:
		PID_felles.a = parameter;		   break;


	/// fra id    til     : lowpass filter parametre
	case 15:
		/// b blir oppdatert automatisk i kode.
		lowpass_thruster.a =  parameter;        	break;
	case 16:

		lowpass_manpu_steg.a =  parameter;        break;
	case 17:

		lowpass_manpu_borste.a = parameter;       break;


	case 18:
		BEGRENSING = parameter; break;

	case 19:
		d_effekt.horisontal = parameter; break;
	case 20:
		d_effekt.horisontal = parameter; break;


	/// fra id    til     : sensitivitet parametre
	case 30:
		thrust_sensitivitet.gir = (float) parameter;			break;
	case 31:
		thrust_sensitivitet.hiv = (float) parameter;			break;
	case 32:
		thrust_sensitivitet.jag = (float) parameter;			break;
	case 33:
		thrust_sensitivitet.svai = (float) parameter;			break;

	case 34:
		manpu_sensitivitet.gripe = (float) parameter;		break;
	case 35:
		manpu_sensitivitet.snurr = (float) parameter;		break;
	case 36:
		manpu_sensitivitet.sving = (float) parameter;		break;
//	case (..):
//			manpu_sensitivitet.skyv = (float) parameter;		break;


	case 50:
		PID_rull.uimaks = parameter;			break;
	case 51:
		PID_rull.uimin = parameter;			   break;
	case 52:
		PID_stamp.uimaks = parameter;			break;
	case 53:
		PID_stamp.uimin = parameter;			break;
	case 54:
		PID_hiv.uimaks = parameter;			break;
	case 55:
		PID_hiv.uimin = parameter;			break;

	case 100:
		ENDRE_FREKVENS = (uint8_t) parameter; break;


	case 200:
		pos_paa_PWM.a = parameter; break;
	case 201:
		pos_paa_PWM.b = parameter; break;
	case 202:
		neg_paa_PWM.a = parameter; break;
	case 203:
		neg_paa_PWM.b = parameter; break;
	case 204:
		pos_kr_paa.a = parameter; break;
	case 205:
		pos_kr_paa.b = parameter; break;
	case 206:
		neg_kr_paa.a = parameter; break;
	case 207:
		neg_kr_paa.b = parameter; break;


	case 300:
		REF.dybde = parameter; break;
	case 301:
		REF.rull = parameter; break;
	case 302:
		REF.stamp = parameter; break;

	case 303:
		 manu_ARR_default.gripe = parameter; break;
	case 304:
		 manu_ARR_default.skyv = parameter; break;
	case 305:
		 manu_ARR_default.snurr= parameter; break;
	case 306:
		 manu_ARR_default.sving= parameter; break;

	case 308:
		 motorkar.max_neg_retn = parameter; break;


	case 330:
		kraftdata.EFFEKT_begrensing = parameter; break;
	}
}



