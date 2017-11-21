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
		remoto_x->FLAG_TIEMPO = 0;
		remoto_x->ms = GET_RECEP(remoto_x->Input.periodo);
		remoto_x->perdida_senal_remoto = 0;
	}
}
void GetVelocidad (MOTOR *motor_x){
	if (motor_x->FLAG_TIEMPO){
		motor_x->FLAG_TIEMPO = false;
		motor_x->ms = GET_VEL(motor_x->Input.periodo);
		motor_x->rpm=60000/(motor_x->ms/10*24);
	}
	if (motor_x->cuenta_vel_cero >= RESET_VELOCIDAD_MS){	//un pulso, se pone vel en cero										
		motor_x->FLAG_TIEMPO = false;
		motor_x->ms = 0;
		motor_x->ms_ant = 0;
		motor_x->rpm = 0;
		motor_x->cuenta_vel_cero = 0;
		motor_x->Input.indices = 0;
	}
}
void RPM_Cero(MOTOR *motor_x){
	if (motor_x->rpm == 0 && motor_x->RPM_set == 0){ //1 segundo
		motor_x->control = 0;
	}
}
void Error_PID(MOTOR *motor_x){
	motor_x->error_RPM = (motor_x->RPM_set - motor_x->rpm);	
	if (motor_x->RPM_set < 10){
		motor_x->k = K_PID/2;
	} else {
		motor_x->k = K_PID;
	}
	//TI = 0.5352609
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
void Tension2Duty(MOTOR *motor_x){
	//%%%%%%%%%%%%%%%%%%%% VER QUE LA VARIABLE TENSION NO SEA NEGATIVA! %%%%%%%%%%%%%%%%%%%%%%%%%%%%!!!!!!!
	motor_x->tension = (motor_x->tension >= U_MAX) ? U_MAX : motor_x->tension;	
	motor_x->duty = (motor_x->tension != 0) ? Mapeo(motor_x->tension,U_MIN,U_MAX,DUTY_MIN,DUTY_MAX) : 65535;
}

void Reset_PIDs(MOTOR motor_x){
	switch (motor_x.nro){
	case MOTOR_DI:
		CtrlPID_DD_Reset();		
		break;
	case MOTOR_DD:
		CtrlPID_DI_Reset();	
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
void ResetVar (void){

}
void RX (SERIE seriex, MOTOR *motordi, MOTOR *motordd, MOTOR *motortd, MOTOR *motorti, uint8 *ESTADO){
	
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
MOTOR_TX TX_Motor(MOTOR motor_x){
	MOTOR_TX tx;
	tx.i = motor_x.adc;
	tx.posicion = motor_x.posicion_pulsos;
	tx.rpm = motor_x.rpm;
	return tx;
}
void Motor2Send(SERIE *serie, MOTOR_TX *motor){
	uint8 *p;
	uint8 i;
	i=0;
	p = (uint8 *)motor;
	while(i<=STRLEN_TXMOTOR-1){
		serie->tx_buf[serie->tx_next] = *p;
		inc(serie->tx_next);
		p++;
		i++;
	}
}
void GetEncoder(MOTOR *motor_x){
	if(motor_x->Input.tiempo >= RETENCION_MS){						//ESPERO UN TIEMPO UNA VEZ RECIBIDO UN PULSO
		  if (motor_x->Input.FLAG_E){						//BANDERA DE UN PULSO
			  if( motor_x->Input.edge == RISING){
				  if (Encoder_DD_GetVal()){
					  motor_x->Input.datos[motor_x->Input.indices] = motor_x->Input.aux;
					  motor_x->Input.indices++;
					  if (motor_x->Input.indices == 2){
						  motor_x->Input.periodo = motor_x->Input.datos[1] - motor_x->Input.datos[0];
						  motor_x->Input.datos[0] = motor_x->Input.datos[1];
						  motor_x->Input.indices = 1;
						  motor_x->FLAG_TIEMPO = 1;
						  motor_x->posicion_pulsos++;
						  motor_x->cuenta_vel_cero = 0;
					  }
					  motor_x->Input.edge = FALLING;
					  //TPulsos_SelectCaptureEdge(DeviceDataPtr, motor_x->Input.nro, EDGE_FALLING);
				  }
			  } else { //CUANDO ES FALLING
				  if (!Encoder_DD_GetVal()){
					  motor_x->Input.datos[motor_x->Input.indices] = motor_x->Input.aux;
					  motor_x->Input.indices++;
					  if (motor_x->Input.indices == 2){
						  motor_x->Input.periodo = motor_x->Input.datos[1] - motor_x->Input.datos[0];
						  motor_x->Input.datos[0] = motor_x->Input.datos[1];
						  motor_x->Input.indices = 1;
						  motor_x->FLAG_TIEMPO = 1;
						  motor_x->posicion_pulsos++;
						  motor_x->cuenta_vel_cero = 0;
					  }
					  motor_x->Input.edge = RISING;
					  //TPulsos_SelectCaptureEdge(DeviceDataPtr, motor_x->Input.nro, EDGE_RISING);
				  }
			  }
			  motor_x->Input.FLAG_E = false;
		  }
	}
	
}
