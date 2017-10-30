#include "Defines.h"

/*
 * ######################## ESTRUCTURAS ###############################
 */
typedef struct input_capture {
	uint16 datos[2];				//Contador de pulsos de cada flanco [1xMOTOR 2xVariableAnteriorYActual] 
	int16 periodo;					//Diferencia entre los pulsos 2 y 1 de cada MOTOR
	uint8 indices;					//Indice de pulsos
	int8 err;						//Por si ocurre un error al momento de leer pulsos
} INPUT_CAPTURE;

typedef struct motor {
	INPUT_CAPTURE Input;
	byte FLAG_TIEMPO;				//FLAG de VELOCIDAD LEIDA
	uint32 posicion_pulsos;			//Contador de Flancos del Motor
	uint16 cuenta_vel_cero;			//Contador para determinar Vel CERO si no se registran nuevos flancos
	uint16 adc;						//Valores de los ADC de cada MOTOR
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
	uint8 direccion_lectura;		//Lee la posicion del PASO a PASO
	uint8 direccion_set;			//SETPOINT de direcion
	uint8 FLAG_EN;					//PASO A PASO ENEABLE
	uint8 FLAG_SENTIDO;				//PASO A PASO SENTIDO DE GIRO
	uint8 FLAG_DIRECCION;			//FLAG usado para definir cuando hay una señal de direccion
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
	uint8 tx_buf[BUF_SIZE];            	// TX buffer
	uint8 tx_next;						// TX indice siguiente a GUARDAR						
	uint8 tx_sent;					  	// TX indice siguiente a MANDAR
	uint8 rx_buf[BUF_SIZE];            	// RX buffer
	uint8 rx_next;						// RX indice siguiente a GUARDAR
	uint8 rx_read;  					// RX indice siguiente a LEERx
	uint8 FLAG_RX;						//Hay datos para procesar RECIBIDOS
	uint8 FLAG_TX;						//Hay datos para procesar ENVIAR
} SERIE;

typedef struct pc {
	uint16 rpm_global;            	// RPM de Entrada PC
} PC;
