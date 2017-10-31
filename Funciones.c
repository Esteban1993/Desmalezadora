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
void Control_LC(MOTOR *motor_x){
	motor_x->error_RPM = (motor_x->RPM_set - motor_x->rpm);	
	if (RPM_SET < 10){
		motor_x->k = K_PID/2;
		CtrlPID_SetK(&motor_x);
		CtrlPID_Control(&motor_x);
	} else {
		motor_x->k = K_PID;
		CtrlPID_SetK(motor_x);
		CtrlPID_Control(&motor_x);
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
bool Vel_Cero(MOTOR motor_1, MOTOR motor_2, MOTOR motor_3, MOTOR motor_4){
	if ((motor_1.ms + motor_2.ms +
		  motor_3.ms + motor_4.ms)>=1){
		return FALSE;
	}
	return TRUE;
}
void Tension2Duty(MOTOR *motor_x){
	//%%%%%%%%%%%%%%%%%%%% VER QUE LA VARIABLE TENSION NO SEA NEGATIVA! %%%%%%%%%%%%%%%%%%%%%%%%%%%%!!!!!!!
	motor_x->tension = (motor_x->tension >= U_MAX) ? U_MAX : motor_x->tension;	
	motor_x->duty = (motor_x->tension != 0) ? Mapeo(motor_x->tension,U_MIN,U_MAX,DUTY_MIN,DUTY_MAX) : 65535;
}
void SetDuty(MOTOR motor_x){
	switch (motor_x.nro){
	case MOTOR_DI:
		Out_PWM_DI_SetRatio16(motor_x.duty);		
		break;
	case MOTOR_DD:
		Out_PWM_DD_SetRatio16(motor_x.duty);	
		break;
	case MOTOR_TD:
		Out_PWM_TD_SetRatio16(motor_x.duty);	
		break;
	case MOTOR_TI:
		Out_PWM_TI_SetRatio16(motor_x.duty);	
		break;
	default:
		break;
	}
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
void RX(void){
	/*
	//####### DECODIFICAR SEÑAL
	word setpoint;
	
	if (FLAG_RX){
		//DECODIFICADO
		DECODIFICADO(rx_buf,rx_read);
		rx_read = rx_next;
		
		//########PASA A TX PARA ENVIAR
		/*
		while (rx_read != rx_next){
			tx_buf[tx_next] = rx_buf[rx_read];
			inc(rx_read);
			inc(tx_next);
		}
		FLAG_TX = true;
		*/
		//FLAG_RX = false;
		
	//}
}
void DECODIFICADO(byte* codigo, byte inicio){
	/*
	 * byte*	codigo	=	buffer de informacion
	 * byte		inicio	=	posicion del primer byte del mensaje dentro del buffer "codigo"
	 */
	/*
	long var;
	const unsigned char *pcodigo;
	bool VALIDAR;
	byte *pvariables;
	word la_vel_duty[4];
	word lc_pc_rpm;
	byte err;
	byte ESTADO_ANTERIOR;
	
	VALIDAR = FALSE;
	ESTADO_ANTERIOR = ESTADO;
	
	pvariables = (byte *) &la_vel_duty;
	pcodigo = (const unsigned char *) &codigo[inicio];	
	if (*pcodigo++ == ':'){
		switch (*pcodigo++){
		case (LA_VELOCIDAD):
				pvariables = (byte *) &la_vel_duty;
				*pvariables++ = *pcodigo++;
				*pvariables++ = *pcodigo++;
				*pvariables++ = *pcodigo++;
				*pvariables++ = *pcodigo++;
				*pvariables++ = *pcodigo++;
				*pvariables++ = *pcodigo++;
				*pvariables++ = *pcodigo++;
				*pvariables++ = *pcodigo++;
				VALIDAR = (*pcodigo++ == ':') ? TRUE : FALSE;
				if (VALIDAR){
					ESTADO = LA_VELOCIDAD;				
				}
				break;
		case (LC_PC):
				lc_pc_rpm = *pcodigo++;
				VALIDAR = (*pcodigo++ == ':') ? TRUE : FALSE;
				if (VALIDAR){
					pc.rpm_global = lc_pc_rpm;
					ESTADO = LC_PC;				
				}
				break;
		default:
			return;
		}
		if (ESTADO != ESTADO_ANTERIOR){
			lectura_nueva = TRUE;
		}
		
	}	
	*/
}
void TX(void){
	/*
	word *SND;
	word snd;
	SND = &snd;
	snd = 0;
	if (FLAG_TX){
		FLAG_TX = false;
		UART_MODBUS_SendBlock(tx_buf,(word)(tx_next - tx_sent),SND);
		tx_next = 0;
		tx_sent = 0;
	}
	*/
}
word GrayToBin(word N){
	word D = N;
	word X;
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
