/*
 * Defines2.h
 *
 *  Created on: Oct 31, 2017
 *      Author: Esteban
 */

#ifndef DEFINES2_H_
#define DEFINES2_H_

/*
 * ######################## DEFINES ###############################
 */
#define LA_VELOCIDAD			1
#define LA_REMOTO				4
#define CALIBRACION				2
#define LC_PC					3
#define LC_REMOTO				6
#define PASOS					7
#define PERDIDA_SENAL			30

#define	MOTOR_DI				0
#define	MOTOR_DD				1
#define	MOTOR_TI				2
#define	MOTOR_TD				3

#define DERECHA					1
#define IZQUIERDA				0
#define REVERSA					1
#define	ADELANTE				0

#define PERDIDA_REMOTO			1000	//ms que deben pasar para definir la perdida de la señal REMOTO
#define CUENTAS_REMOTO			2000		//Limite para definir el ancho del pulso CALIBRADO
#define RESET_VELOCIDAD_MS		300
#define	VELOCIDAD				0
#define	DIRECCION				1
#define REMOTO_PERDIDA_SENAL	500		//ms
#define REMOTO_ANCHO_PULSO		28		//Desplazamiento MINIMO del ancho del pulso REMOTO x10us
#define REMOTO_VENTANA			0		//definir
#define REMOTO_VEL_CERO			150
#define REMOTO_VEL_MAX			204
#define REMOTO_VEL_MIN			116
#define REMOTO_DIR_CERO			150		
#define REMOTO_DIR_MAX			205
#define REMOTO_DIR_MIN			97
#define VENTANA_DIRECCION		2

#define K_PID					15	//33
#define TI_PID					4	//NO ANDA, HAY QUE PONER 4 y DIVIDIR POR 10 CASTEANDO A FLOAT
#define MUESTREO_PID			100

#define BUTTON_PRESSED      	0
#define BUTTON_NOT_PRESSED  	1
#define true					1
#define false					0

#define DUTY_MAX				0
#define DUTY_MIN				40000//37000
#define DUTY_CERO				65535


#define SET_RPM_MAX				100		//definir
#define SET_RPM_MIN				0

#define U_MAX					19350
#define U_MIN					0

#define RPM_STEP				5000//5
#define FREQ_INPUT				374912
#define FREQ_RECEPTOR			2999296
#define ADC_MIN					0
#define ADC_MAX					3300
#define BIT_16					65400
#define GET_VEL(x)				(((x<<1)*1000*10)/FREQ_INPUT) //BOTH EDGE
//#define GET_VEL(x)				(((x)*1000*10)/FREQ_INPUT) //rising EDGE
#define GET_RECEP(x)			(((x)*10000*10)/FREQ_RECEPTOR)
#define RISING					0
#define FALLING					1
#define RETENCION_MS			2

#define MAX16BIT				65535
#define MIN16BIT				0

#define BUF_SIZE 				256
#define inc(x) 					{x++; x&=(BUF_SIZE-1);}

#define FREQ_PWM_DUTY			2
#define STRLEN_TXMOTOR			8

#define TIMEOUT_RX				5000
#define DIRECCION_MODBUS		9

#define DIRECCION_HORARIA		BitOut_DIR_SENT_SetVal()
#define	DIRECCION_ANTI			BitOut_DIR_SENT_ClrVal()
#define DIRECCION_ON			BitOut_DIR_EN_ClrVal()
#define DIRECCION_OFF			BitOut_DIR_EN_SetVal()
#define LIMITE_DIRECCION_DERECHO	238
#define DIRECCION_CENTRO			200
#define LIMITE_DIRECCION_IZQUIERDO	160

#define PC_LI						-100
#define PC_LD						100

#endif /* DEFINES2_H_ */
