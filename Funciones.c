/*
 * Funciones.c
 *
 *  Created on: Oct 30, 2017
 *      Author: Esteban
 */
#include "Funciones.h"

/*
 * ######################## FUNCIONES ###############################
 *  * ######################## FUNCIONES ###############################
 *   * ######################## FUNCIONES ###############################
 *    * ######################## FUNCIONES ###############################
 *       
 */

long Mapeo(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Get_Remoto(REMOTO *remoto_x){
	if (remoto_x->FLAG_TIEMPO){
		remoto_x->FLAG_TIEMPO = false;
		remoto_x->ms = GET_RECEP(remoto_x->Input.periodo);
		remoto_x->perdida_senal_remoto = 0;
	}
}
void GetVelocidad (MOTOR *motor_x){
	if (motor_x->FLAG_TIEMPO){							//FLAG DE TIEMPO?
		motor_x->FLAG_TIEMPO = false;					//RESET FLAG DE TIEMPO
		motor_x->ms = GET_VEL(motor_x->Input.periodo);	//PASAJE DEL ANCHO DEL PULSO EN MUESTRAS A TIEMPO
		/*
		 * Pasaje de MILISEGUNDOS a RPM
		 */
		if (motor_x->ms == 0){
			motor_x->rpm = 0;			
		} else {
			motor_x->rpm = 60000/(motor_x->ms/10*24);	
		}
	}
	/*
	 * Si no se obtuvo una señal del Sensor HALL por mas de "RESET_VELOCIDAD_MS" de milisegundos,
	 * detecta que la rueda esta detenida y setea los valores de velocidad a cero junto al indice
	 * utilizado para el calculo del periodo de la señal de velocidad.
	 */
	if (motor_x->cuenta_vel_cero >= RESET_VELOCIDAD_MS){									
		motor_x->FLAG_TIEMPO = false;
		motor_x->ms = 0;
		motor_x->rpm = 0;
		motor_x->cuenta_vel_cero = 0;
		motor_x->Input.indices = 0;
	}
}
void Error_PID(MOTOR *motor_x){
	motor_x->error_RPM = (motor_x->RPM_set - motor_x->rpm);		//CALCULO DEL ERROR DE VELOCIDAD
	/*
	 * Existen errores esporadicos en las lecturas de velocidad las cuales generan valores de error de velocidad altos.
	 * Estos errores generan en el control PID un cambio de signo de la variable controlada, lo que produce que el sistema se inestabilice.
	 * Para evitarlo, se implemento un maximo de error admitido, es decir, una saturacion en la entrada del control PID de 50 RPM.
	 */
	motor_x->error_RPM = (motor_x->error_RPM >= 50) ? 50 : motor_x->error_RPM;
	motor_x->error_RPM = (motor_x->error_RPM <= -50) ? -50 : motor_x->error_RPM;
}
void CtrlPID_SetK(MOTOR motor_x){
	switch (motor_x.nro){
	case MOTOR_DI:
		CtrlPID_DI_Set_K((float)(motor_x.k));		
		break;
	case MOTOR_DD:
		CtrlPID_DD_Set_K((float)(motor_x.k));
		break;
	case MOTOR_TD:
		CtrlPID_TD_Set_K((float)(motor_x.k));
		break;
	case MOTOR_TI:
		CtrlPID_TI_Set_K((float)(motor_x.k));
		break;
	default:
		break;
	}
}
void CtrlPID_Control(MOTOR *motor_x){
	switch (motor_x->nro){
	case MOTOR_DI:
		CtrlPID_DI_Control(motor_x->error_RPM,&motor_x->control);		
		break;
	case MOTOR_DD:
		CtrlPID_DD_Control(motor_x->error_RPM,&motor_x->control);	
		break;
	case MOTOR_TD:
		CtrlPID_TD_Control(motor_x->error_RPM,&motor_x->control);	
		break;
	case MOTOR_TI:
		CtrlPID_TI_Control(motor_x->error_RPM,&motor_x->control);	
		break;
	default:
		break;
	}
}
unsigned char Vel_Cero(MOTOR motor_1, MOTOR motor_2, MOTOR motor_3, MOTOR motor_4){
	if ((motor_1.ms + motor_2.ms +
		  motor_3.ms + motor_4.ms)>=1){
		return 0;
	}
	return 1;
}
unsigned char Tension_Cero(MOTOR motor_1, MOTOR motor_2, MOTOR motor_3, MOTOR motor_4){
	if ((motor_1.tension + motor_2.tension +
		  motor_3.tension + motor_4.tension)>=1){
		return 0;
	}
	return 1;
}
void Tension2Duty(MOTOR *motor_x){
	motor_x->tension = (motor_x->tension >= U_MAX) ? U_MAX : motor_x->tension;	
	motor_x->duty = (motor_x->tension != 0) ? Mapeo(motor_x->tension,U_MIN,U_MAX,DUTY_MIN,DUTY_MAX) : 65535;
}

void Reset_PIDs(MOTOR motor_x){
	switch (motor_x.nro){
	case MOTOR_DI:
		CtrlPID_DI_Reset();		
		break;
	case MOTOR_DD:
		CtrlPID_DD_Reset();	
		break;
	case MOTOR_TD:
		CtrlPID_TD_Reset();	
		break;
	case MOTOR_TI:
		CtrlPID_TI_Reset();
		break;
	default:
		break;
	}
}

unsigned short GrayToBin(unsigned short N){
	unsigned short D = N;
	unsigned short X;
	int i;
	for(i=0; D>1;i++) D>>=1;
	D<<=i;
	X = D;	
	while(X>1){
		N = N^((N&X)>>1);
		X>>=1;
	}
	return N;
}
void NumeroFin(SERIE *serie_x){
	unsigned char i;
	i = 0;
	serie_x->tx_next = 0;
	while(serie_x->tx_buf[i]!='\n'){
		inc(serie_x->tx_next);	
		i++;
	}
	inc(serie_x->tx_next);	
}
/*
 * Realiza la validación de las muestras adquiridas por las
 * 	interrupciones que capturan los flancos ascendentes y
 * 	descendentes de los sensores Hall.
 * De validar dos muestras, obtiene la diferencia que existe
 * 	entre los dos flancos y da aviso de un valor nuevo
 */
void GetHall(MOTOR *motor_x){
	bool val;
	/*
	 * Segun sea el motor ingresado, capturara el valor de la entrada HALL que corresponda.
	 */
	switch (motor_x->nro){	//TOMAR VALOR DE LA ENTRADA HALL Y RETENERLO(ALTO O BAJO)
	case MOTOR_DI:
		val = Hall_DI_GetVal();
		break;
	case MOTOR_DD:
		val = Hall_DD_GetVal();	
		break;
	case MOTOR_TD:
		val = Hall_TD_GetVal();
		break;
	case MOTOR_TI:
		val = Hall_TI_GetVal();
		break;
	default:
		break;
	}	
	if(motor_x->Input.tiempo >= RETENCION_MS){			//ESPERO UN TIEMPO UNA VEZ RECIBIDO UN PULSO
		  if (motor_x->Input.FLAG_E){					//BANDERA DE UN PULSO NUEVO?
			  if( motor_x->Input.edge == RISING){		//FLANCO DE SUBIDA?
				  if (val == true){						//ESTADO DE LA ENTRADA QUE SE RETUVO ALTO?
					  motor_x->Input.datos[motor_x->Input.indices] = motor_x->Input.aux;	//MUESTRAS CAPTURADAS. VALIDACION OK
					  motor_x->Input.indices++;												//INCREMENTO DEL INDICE
					  if (motor_x->Input.indices == 2){										//DOS MUESTRAS CAPTURADAS?										
						  motor_x->Input.periodo = motor_x->Input.datos[1] - motor_x->Input.datos[0];	//DIFERENCIA DE MUESTRAS CAPTURADAS
						  motor_x->Input.datos[0] = motor_x->Input.datos[1];							//RELEVO DE MUESTRAS
						  motor_x->Input.indices = 1;					//INDICE EN UNO. MUESTRA ANTERIOR
						  motor_x->FLAG_TIEMPO = 1;						//FLAG TIEMPO EN ALTO
						  motor_x->posicion_pulsos++;					//INCREMENTO DE VARIABLE DE PULSOS
						  motor_x->cuenta_vel_cero = 0;					//RESET SEGURIDAD VELOCIDAD CERO
					  }
					  motor_x->Input.edge = FALLING;					//SETEAR FLANCO ESPERADO: BAJADA
				  }
			  } else {									//FLANCO DE BAJADA?
				  if (val == false){
					  motor_x->Input.datos[motor_x->Input.indices] = motor_x->Input.aux;
					  motor_x->Input.indices++;
					  if (motor_x->Input.indices == 2){
						  motor_x->Input.periodo = motor_x->Input.datos[1] - motor_x->Input.datos[0];	//DIFERENCIA DE MUESTRAS CAPTURADAS
						  motor_x->Input.datos[0] = motor_x->Input.datos[1];							//RELEVO DE MUESTRAS
						  motor_x->Input.indices = 1;					//INDICE EN UNO. MUESTRA ANTERIOR
						  motor_x->FLAG_TIEMPO = 1;						//FLAG TIEMPO EN ALTO
						  motor_x->posicion_pulsos++;					//INCREMENTO DE VARIABLE DE PULSOS
						  motor_x->cuenta_vel_cero = 0;					//RESET SEGURIDAD VELOCIDAD CERO
					  }
					  motor_x->Input.edge = RISING;						//SETEAR FLANCO ESPERADO: SUBIDA
				  }
			  }
			  motor_x->Input.FLAG_E = false;		//BAJO BANDERA DE PULSO
		  }
	}
	
}
void Duty2Motor(PC *pc, MOTOR *motor){
	pc->duty[motor->nro] = (pc->duty[motor->nro] >= PC_LDUTYMAX) ? PC_LDUTYMAX : pc->duty[motor->nro];
	pc->duty[motor->nro] = (pc->duty[motor->nro] <= PC_LDUTYMIN) ? PC_LDUTYMIN : pc->duty[motor->nro];
	if (pc->duty[motor->nro] == PC_LDUTYMIN) {
		motor->duty = DUTY_CERO;
	} else {
		motor->duty = Mapeo(pc->duty[motor->nro], PC_LDUTYMIN, PC_LDUTYMAX,
				DUTY_MIN, DUTY_MAX);
	}
	
}
