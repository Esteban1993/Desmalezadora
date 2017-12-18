/*
 * Struct2.h
 *
 *  Created on: Oct 31, 2017
 *      Author: Esteban
 */

#ifndef STRUCT2_H_
#define STRUCT2_H_

#include "Defines2.h"
#include "PE_Types.h"
/*
 * #ifndef __cplusplus
  #ifndef bool 
typedef unsigned char           bool;
  #endif
#endif
typedef unsigned char           byte;
typedef unsigned short          word;
typedef unsigned long           dword;
typedef unsigned long long      dlong;
typedef unsigned char           TPE_ErrCode;
#ifndef TPE_Float
typedef float                   TPE_Float;
#endif
#ifndef char_t
typedef char                    char_t;
#endif

typedef signed char             int8;
typedef signed short int        int16;
typedef signed long int         int32;

typedef unsigned char           uint8;
typedef unsigned short int      uint16;
typedef unsigned long int       uint32;
 * 
 *
 *
 *
 */


/*
 * ######################## ESTRUCTURAS ###############################
 */
typedef struct input_capture {
	uint8 nro;						//nro de canal del Input
	uint16 datos[2];				//Contador de pulsos de cada flanco [1xMOTOR 2xVariableAnteriorYActual] 
	uint16 periodo;					//Diferencia entre los pulsos 2 y 1 de cada MOTOR
	uint8 indices;					//Indice de pulsos
	uint8 err;						//Por si ocurre un error al momento de leer pulsos
	uint8 edge;						//Flanco en el que esta trabajando
	uint16 aux;
	uint8 FLAG_E;
	uint8 tiempo;
} INPUT_CAPTURE;

typedef struct motor {
	uint8 nro;
	INPUT_CAPTURE Input;
	uint8 FLAG_TIEMPO;				//FLAG de VELOCIDAD LEIDA
	uint32 posicion_pulsos;			//Contador de Flancos del Motor
	uint16 cuenta_vel_cero;			//Contador para determinar Vel CERO si no se registran nuevos flancos
	uint16 adc;						//Valores de los ADC de cada MOTOR
	uint16 i;						//Valores de Corriente
	uint16 ms;						//Velocidad en ms del encoder
	uint16 rpm;						//Lectura RPM
	uint16 RPM_set;					//SETPOINT DE RPM GLOBAL
	int32 error_RPM;				//Entrada al PID [SETPOINT - LECTURA]
	
	uint16 control;					//Salida del PID
	uint8 k;
	uint8 td;
	uint8 ti;
	uint16 tension;					//Salida del PID para la FUNCION Tension_PWM
	uint16 duty;					//Duty aplicado a los PWM de cada MOTOR
	uint16 duty_entrada;			//Lectura DUTY en ESTADO LA_VELOCIDAD
	
} MOTOR;

typedef struct pap {
	uint8 pwm_direccion;			//Contador para el PWM Manual
	uint16 pasos_dados;				//Cantidad de PASOS que se ha dado	//pwm_pasos
	uint16 pasos_adar;				//Pasos que se desean - PASO A PASO //pasos_direccion
	uint16 direccion_lectura;		//Lee la posicion del PASO a PASO
	uint8 direccion_set;			//SETPOINT de direcion
	uint8 FLAG_EN;					//PASO A PASO ENEABLE
	uint8 FLAG_SENTIDO;				//PASO A PASO SENTIDO DE GIRO
	uint8 FLAG_DIRECCION;			//FLAG usado para definir cuando hay una señal de direccion
	uint8 FLAG_HABILITADO;			//Habilitacion Direccion
} PAP;

typedef struct remoto {
	INPUT_CAPTURE Input;
	uint8 FLAG_TIEMPO;				//FLAG de VELOCIDAD LEIDA
	uint16 remoto_cero;				//Ancho del pulso en CERO
	uint16 cuenta_remoto;			//Cuenta las veces para definir ancho de CALIBRACION
	uint16 ms;						//Ancho del pulso del receptor REMOTO
	uint16 perdida_senal_remoto;	//Contador para detectar perdida de señal en modo REMOTO o CALIBRACION
} REMOTO;

typedef struct serie {
	uint8 send[BUF_SIZE];
	uint8 tx_buf[BUF_SIZE];            	// TX buffer
	uint8 tx_next;						// TX indice siguiente a GUARDAR						
	uint8 tx_sent;					  	// TX indice siguiente a MANDAR
	uint8 rx_buf[BUF_SIZE];            	// RX buffer
	uint8 rx_next;						// RX indice siguiente a GUARDAR
	uint8 rx_read;  					// RX indice siguiente a LEERx
	uint8 FLAG_RX;						//Hay datos para procesar RECIBIDOS
	uint8 FLAG_TX;						//Hay datos para procesar ENVIAR
	uint8 HAB_RX;						//Habilitar RX
	uint8 HAB_TX;						//Habilitar TX
} SERIE;

typedef struct pc {
	uint8 duty_global;          //Duty de Entrada PC 0 a 100
	uint8 duty[4];				//Duty de Entrada PC 0 a 100 INDIVIDUAL
	uint16 rpm_global;          //RPM de Entrada PC
	uint16 rpm[4];				//RPM de Entrada PC INDIVIDUAL
	int8 direccion;				//Posicion Encoder
	uint8 pap_ha;				//Habilita PAP
	uint8 pap_en;				//Enable Driver
	uint8 reversa;				//Posicion Encoder
	uint8 ha_TX;				//Habilito Trasmicion
} PC;


#endif /* STRUCT2_H_ */
