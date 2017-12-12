/* ###################################################################
 **     Filename    : main.c
 **     Project     : BOBOT
 **     Processor   : MK64FN1M0VLL12
 **     Version     : Driver 01.01
 **     Compiler    : CodeWarrior ARM C Compiler
 **     Date/Time   : 2017-06-29, 16:06, # CodeGen: 0
 **     Abstract    :
 **         Main module.
 **         This module contains user's application code.
 **     Settings    :
 **     Contents    :
 **         No public methods
 **
 ** ###################################################################*/
/*!
 ** @file main.c
 ** @version 01.01
 ** @brief
 **         Main module.
 **         This module contains user's application code.
 */
/*!
 **  @addtogroup main_module main module documentation
 **  @{
 */
/* MODULE main */

/* Including needed modules to compile this module/procedure */
#include "Cpu.h"
#include "Events.h"
#include "IntTiempo.h"
#include "TimerIntLdd1.h"
#include "CtrlPID_DD.h"
#include "BitLed_Verde.h"
#include "BitIoLdd1.h"
#include "ReceptorVelocidad.h"
#include "CaptureLdd5.h"
#include "TReceptor.h"
#include "ReceptorDireccion.h"
#include "CaptureLdd6.h"
#include "BitOut_DIR_EN.h"
#include "BitIoLdd3.h"
#include "BitOut_DIR_PWM.h"
#include "BitIoLdd4.h"
#include "CtrlPID_TI.h"
#include "CtrlPID_TD.h"
#include "Out_Reversa.h"
#include "BitIoLdd5.h"
#include "UART_MODBUS.h"
#include "ASerialLdd2.h"
#include "Btn_SW1.h"
#include "ExtIntLdd3.h"
#include "Btn_SW2.h"
#include "ExtIntLdd1.h"
#include "TTemp.h"
#include "CtrlPID_DI.h"
#include "MCUC1.h"
#include "Out_PWM_DD.h"
#include "PwmLdd1.h"
#include "TTemp.h"
#include "TPWM.h"
#include "Out_PWM_TD.h"
#include "PwmLdd3.h"
#include "Out_PWM_DI.h"
#include "PwmLdd2.h"
#include "TPulsos.h"
#include "Out_PWM_TI.h"
#include "PwmLdd4.h"
#include "Input_Encoder_DD.h"
#include "CaptureLdd1.h"
#include "Input_Encoder_TD.h"
#include "CaptureLdd2.h"
#include "Input_Encoder_TI.h"
#include "CaptureLdd3.h"
#include "Input_Encoder_DI.h"
#include "CaptureLdd4.h"
#include "BitOut_DIR_SENT.h"
#include "BitIoLdd2.h"
#include "ADC_I.h"
#include "AdcLdd2.h"
#include "IntDireccion.h"
#include "TimerIntLdd2.h"
#include "BitLed_Azul.h"
#include "BitIoLdd6.h"
#include "TEXT.h"
#include "Status_LED.h"
#include "BitIoLdd7.h"
#include "BIT0.h"
#include "BitIoLdd8.h"
#include "BIT1.h"
#include "BitIoLdd9.h"
#include "BIT2.h"
#include "BitIoLdd10.h"
#include "BIT3.h"
#include "BitIoLdd11.h"
#include "BIT4.h"
#include "BitIoLdd12.h"
#include "BIT5.h"
#include "BitIoLdd13.h"
#include "BIT6.h"
#include "BitIoLdd14.h"
#include "BIT7.h"
#include "BitIoLdd15.h"
#include "Btn_Emergencia.h"
#include "ExtIntLdd2.h"
#include "Encoder_TD.h"
#include "BitIoLdd16.h"
#include "Encoder_DD.h"
#include "BitIoLdd17.h"
#include "Encoder_DI.h"
#include "BitIoLdd18.h"
#include "Encoder_TI.h"
#include "BitIoLdd19.h"
#include "WDog1.h"
#include "WatchDogLdd1.h"
/* Including shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "PDD_Includes.h"
#include "Init_Config.h"
#include "Struct2.h"
#include "Defines2.h"
#include "Funciones.h"

void Get_Corriente(void);
void Get_Direccion(PAP *pap_x);
void SetDuty(MOTOR motor_x);
void TXS(SERIE *serie_x);
void StrcatMotor(MOTOR motor_x);
void RX(void);

/* User includes (#include below this line is not maintained by Processor Expert) */
/*
 * ######################## VARIABLES GLOBALES ###############################
 */

// FLAGS
byte ESTADO;						//Indica el ESTADO del PROGRAMA
byte ESTADOANTERIOR;				//Indica el ESTADO del PROGRAMA ANTERIOR
uint16 WATCHDOG = 0;				//FLAG DEL WATCHDOG
int8 TEMPERATURA;
byte FLAG_SW1;						//FLAG DEL PULSADOR 1
byte FLAG_SW2;						//FLAG DEL PULSADOR 2
word cuenta_EMERGENCIA;
word cnt_aux;						//Cuenta AUXILIAR en interrupcion INTTIEMPO
uint16 cuenta_x2 = 0;				//Cuenta x2
word cuenta_PID;					//Cuenta de tiempo PID
word cuenta_RX;				//Cuenta de tiempo RECIBIR
word cuenta_TX;						//Cuenta de tiempo RECIBIR
word cuenta_DIRECCION = 0;			//Cuenta para leer direccion
byte FLAG_TIEMPO[4];				//FLAG de VELOCIDAD LEIDA
byte FLAG_RECEPTOR[2];				//FLAG de ANCHO DE PULSO LEIDO REMOTO
byte FLAG_ADC;						//FLAG de ADC TERMINO
byte FLAG_RX;						//Hay datos para procesar RECIBIDOS
byte FLAG_TX;						//Hay datos para procesar ENVIAR
byte FLAG_RETROCESO;

// DIRECCION PWM
byte pwm_direccion;					//Contador para el PWM Manual
word pwm_pasos;						//Cantidad de PASOS que se ha dado

// ######################## VARIABLES INTERNAS ###############################

bool lectura_nueva;			//Cuando se cambia de estado, sirve para ejecutar 
//	una porcion de codigo por unica vez

uint16 emergencias = 0;				//Cuenta las entradas a emergencia

word RPM_SET = 0;					//Setpoint RPM Global
word DUTY_SET = 0;				//Setpoint TENSION Global
bool sentido_ant;		//Sentido anterior de GIRO MOTOR 			1-REVERSA
bool sentido_act;		//Sentido actual que se desea GIRAR MOTOR 	1-REVERSA
byte sentido_entrada;	//Sentido de GIRO MOTOR desde la PC 		1-REVERSA

uint8 x2 = false;

uint16 rpm_max_control;				//Setea el MAXIMO de RPM que se puede aplicar en MODO REMOTO LC
uint16 duty_max_control = DUTY_MAX;	//Setea el MAXIMO de DUTY que se puede aplicar en MODO REMOTO LA

byte FLAG_PASOS;				//FLAG usada en el ESTADO PASOS
byte FLAG_DIRECCION = false;//FLAG usado para definir cuando hay una señal de direccion

word tension_global = 0;
word duty_global = DUTY_CERO;

//######################NUEVO!

MOTOR motor_di = { MOTOR_DI, { MOTOR_DI, { 0, 0 }, 0, 0, 0, RISING, 0, 0, 0 },
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, K_PID, 0, TI_PID, 0, DUTY_CERO, 0 };
MOTOR motor_dd = { MOTOR_DD, { MOTOR_DD, { 0, 0 }, 0, 0, 0, RISING, 0, 0, 0 },
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, K_PID, 0, TI_PID, 0, DUTY_CERO, 0 };
MOTOR motor_ti = { MOTOR_TI, { MOTOR_TI, { 0, 0 }, 0, 0, 0, RISING, 0, 0, 0 },
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, K_PID, 0, TI_PID, 0, DUTY_CERO, 0 };
MOTOR motor_td = { MOTOR_TD, { MOTOR_TD, { 0, 0 }, 0, 0, 0, RISING, 0, 0, 0 },
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, K_PID, 0, TI_PID, 0, DUTY_CERO, 0 };

REMOTO direccion;
REMOTO velocidad;

PAP pap = { 0, 0, 0, 0, DIRECCION_CENTRO, false, false, false, false };

PC pc = { DUTY_CERO, {DUTY_CERO, DUTY_CERO, DUTY_CERO, DUTY_CERO}, SET_RPM_MIN, {SET_RPM_MIN, SET_RPM_MIN, SET_RPM_MIN, SET_RPM_MIN}, 0, 1, 1, ADELANTE, true};

SERIE serie;

/*lint -save  -e970 Disable MISRA rule (6.3) checking. */

int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */

{
	byte i;
	byte x;
	//LDD_TDeviceData *DeviceDataPtr;

	/* Write your local variable definition here */

	/*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
	PE_low_level_init();
	/*** End of Processor Expert internal initialization.                    ***/

	/* Write your code here */

	//################ INICIALIZAR VARIABLES
	ResetVar();
	//PID

	CtrlPID_DD_Set_K((float) motor_dd.k);
	CtrlPID_DD_Set_Ti((float) (motor_dd.ti / 10));
	CtrlPID_DI_Set_K((float) motor_di.k);
	CtrlPID_DI_Set_Ti((float) (motor_di.ti / 10));
	CtrlPID_TD_Set_K((float) motor_td.k);
	CtrlPID_TD_Set_Ti((float) (motor_td.ti / 10));
	CtrlPID_TI_Set_K((float) motor_ti.k);
	CtrlPID_TI_Set_Ti((float) (motor_ti.ti / 10));

	//
	//#######################################
	/*
	while (ADC_I_Calibrate(TRUE) == ERR_BUSY) {
	}
	*/
	ADC_I_Measure(FALSE);
	DIRECCION_ON;
	ESTADO = CALIBRACION;
	//ESTADO = LC_PC;
	ESTADOANTERIOR = ESTADO;
	rpm_max_control = 40;
	lectura_nueva = true;
	pap.FLAG_HABILITADO = false;
	pap.FLAG_EN = true;
	serie.HAB_TX = true;
	serie.HAB_RX = true;
	for (;;) {
		WATCHDOG = WATCHDOG | WD_PROGRAMA;
		if (ESTADO != ESTADOANTERIOR) {			
			lectura_nueva = true;
		}
		//INICIO
		Get_Direccion(&pap);					//LEER DIRECCION
		GetEncoder(&motor_dd);
		GetEncoder(&motor_di);
		GetEncoder(&motor_td);
		GetEncoder(&motor_ti);
		GetVelocidad(&motor_dd);
		GetVelocidad(&motor_di);
		GetVelocidad(&motor_td);
		GetVelocidad(&motor_ti);
		Get_Corriente();

		switch (ESTADO) {
		case LA_REMOTO:
			WATCHDOG = WATCHDOG | WD_ESTADO;
			ESTADOANTERIOR = ESTADO;
			if (lectura_nueva) {
				motor_di.duty = DUTY_CERO;
				motor_dd.duty = DUTY_CERO;
				motor_ti.duty = DUTY_CERO;
				motor_td.duty = DUTY_CERO;
				lectura_nueva = false;
			}
			Get_Remoto(&velocidad);
			Get_Remoto(&direccion);
			//LEER DIRECCION
			direccion.ms =
					(direccion.ms >= direccion.remoto_cero + REMOTO_ANCHO_PULSO) ?
							direccion.remoto_cero + REMOTO_ANCHO_PULSO :
							direccion.ms;
			direccion.ms =
					(direccion.ms <= direccion.remoto_cero - REMOTO_ANCHO_PULSO) ?
							direccion.remoto_cero - REMOTO_ANCHO_PULSO :
							direccion.ms;
			pap.direccion_set = Mapeo(direccion.ms,
					direccion.remoto_cero - REMOTO_ANCHO_PULSO,
					direccion.remoto_cero + REMOTO_ANCHO_PULSO,
					LIMITE_DIRECCION_IZQUIERDO, LIMITE_DIRECCION_DERECHO);
			//END LEER DIRECCION
			//LEER VELOCIDAD
			velocidad.ms =
					(velocidad.ms >= velocidad.remoto_cero + REMOTO_ANCHO_PULSO) ?
							velocidad.remoto_cero + REMOTO_ANCHO_PULSO :
							velocidad.ms;
			velocidad.ms =
					(velocidad.ms <= velocidad.remoto_cero - REMOTO_ANCHO_PULSO) ?
							velocidad.remoto_cero - REMOTO_ANCHO_PULSO :
							velocidad.ms;
			//CAMBIO DE SENTIDO
			if (velocidad.ms < (velocidad.remoto_cero - REMOTO_VENTANA)) {//ES REVERSA?
				sentido_act = REVERSA;
			} else {
				sentido_act = ADELANTE;
			}
			if (sentido_act != sentido_ant) {
				if (Vel_Cero(motor_di, motor_dd, motor_td, motor_ti)) {
					sentido_ant = sentido_act;
					Out_Reversa_PutVal(sentido_act);
				} else {
					velocidad.ms = velocidad.remoto_cero;
				}
			}
			if (sentido_act == ADELANTE) {
				DUTY_SET = Mapeo(velocidad.ms, velocidad.remoto_cero,
						velocidad.remoto_cero + REMOTO_ANCHO_PULSO, DUTY_MIN,
						duty_max_control);
			} else {
				DUTY_SET = Mapeo(velocidad.ms, velocidad.remoto_cero,
						velocidad.remoto_cero - REMOTO_ANCHO_PULSO, DUTY_MIN,
						duty_max_control);
			}
			DUTY_SET = (DUTY_SET <= duty_max_control) ? duty_max_control : DUTY_SET;
			
			//END CAMBIO DE SENTIDO
			motor_di.duty = DUTY_SET;
			motor_dd.duty = DUTY_SET;
			motor_ti.duty = DUTY_SET;
			motor_td.duty = DUTY_SET;
			//PERDIDA DE SENAL - SALE DEL ESTADO
			if (velocidad.perdida_senal_remoto >= REMOTO_PERDIDA_SENAL
					|| direccion.perdida_senal_remoto >= REMOTO_PERDIDA_SENAL || velocidad.remoto_cero == 0 || direccion.remoto_cero == 0) {
				velocidad.perdida_senal_remoto = 0;
				direccion.perdida_senal_remoto = 0;
				ESTADO = PERDIDA_SENAL;
			}
			//END PERDIDA DE SENAL - SALE DEL ESTADO
			break;
		case LC_REMOTO:
			ESTADOANTERIOR = ESTADO;
			if (lectura_nueva) {
				//RESET AL PASAR AL ESTADO LC_REMOTO
				cuenta_PID = 0;
				Reset_PIDs(motor_di);
				Reset_PIDs(motor_dd);
				Reset_PIDs(motor_ti);
				Reset_PIDs(motor_td);
				motor_dd.error_RPM = 0;
				motor_di.error_RPM = 0;
				motor_ti.error_RPM = 0;
				motor_td.error_RPM = 0;
				motor_dd.control = 0;
				motor_di.control = 0;
				motor_td.control = 0;
				motor_ti.control = 0;
				motor_dd.tension = motor_dd.control;
				motor_di.tension = motor_di.control;
				motor_td.tension = motor_td.control;
				motor_ti.tension = motor_ti.control;
				lectura_nueva = false;
				//END RESET
			}
			//LEER REMOTO
			Get_Remoto(&velocidad);
			Get_Remoto(&direccion);
			//END LEER REMOTO
			//LEER DIRECCION
			direccion.ms =
					(direccion.ms >= direccion.remoto_cero + REMOTO_ANCHO_PULSO) ?
							direccion.remoto_cero + REMOTO_ANCHO_PULSO :
							direccion.ms;
			direccion.ms =
					(direccion.ms <= direccion.remoto_cero - REMOTO_ANCHO_PULSO) ?
							direccion.remoto_cero - REMOTO_ANCHO_PULSO :
							direccion.ms;
			pap.direccion_set = Mapeo(direccion.ms,
					direccion.remoto_cero - REMOTO_ANCHO_PULSO,
					direccion.remoto_cero + REMOTO_ANCHO_PULSO,
					LIMITE_DIRECCION_IZQUIERDO, LIMITE_DIRECCION_DERECHO);
			//END LEER DIRECCION
			//LEER VELOCIDAD
			velocidad.ms =
					(velocidad.ms >= velocidad.remoto_cero + REMOTO_ANCHO_PULSO) ?
							velocidad.remoto_cero + REMOTO_ANCHO_PULSO :
							velocidad.ms;
			velocidad.ms =
					(velocidad.ms <= velocidad.remoto_cero - REMOTO_ANCHO_PULSO) ?
							velocidad.remoto_cero - REMOTO_ANCHO_PULSO :
							velocidad.ms;
			//END LEER VELOCIDAD
			//CAMBIO DE SENTIDO
			if (velocidad.ms < (velocidad.remoto_cero - REMOTO_VENTANA)) {//ES REVERSA?
				sentido_act = REVERSA;
			} else {
				sentido_act = ADELANTE;
			}
			if (sentido_act != sentido_ant) {
				if (Vel_Cero(motor_di, motor_dd, motor_td, motor_ti)) {
					sentido_ant = sentido_act;
					Out_Reversa_PutVal(sentido_act);
					cuenta_PID = 0;
					Reset_PIDs(motor_di);
					Reset_PIDs(motor_dd);
					Reset_PIDs(motor_ti);
					Reset_PIDs(motor_td);
					motor_dd.error_RPM = 0;
					motor_di.error_RPM = 0;
					motor_ti.error_RPM = 0;
					motor_td.error_RPM = 0;
					motor_dd.control = 0;
					motor_di.control = 0;
					motor_td.control = 0;
					motor_ti.control = 0;
					motor_dd.tension = motor_dd.control;
					motor_di.tension = motor_di.control;
					motor_td.tension = motor_td.control;
					motor_ti.tension = motor_ti.control;
					//-
				} else {
					velocidad.ms = velocidad.remoto_cero; //PONE RPM_SET EN CERO
				}
			}
			if (sentido_act == ADELANTE) {
				RPM_SET = Mapeo(velocidad.ms, velocidad.remoto_cero,
						velocidad.remoto_cero + REMOTO_ANCHO_PULSO, SET_RPM_MIN,
						rpm_max_control);
			} else {
				RPM_SET = Mapeo(velocidad.ms, velocidad.remoto_cero,
						velocidad.remoto_cero - REMOTO_ANCHO_PULSO, SET_RPM_MIN,
						rpm_max_control);
			}
			RPM_SET = (RPM_SET > rpm_max_control) ? rpm_max_control : RPM_SET;
			//END CAMBIO DE SENTIDO
			//CONTROL PID
			if (cuenta_PID >= MUESTREO_PID) { //100 milisegundos periodo de muestreo
				WATCHDOG = WATCHDOG | WD_ESTADO;
				cuenta_PID -= MUESTREO_PID;
				motor_dd.pulsos = 0;
				motor_di.pulsos = 0;
				motor_td.pulsos = 0;
				motor_ti.pulsos = 0;
				motor_di.RPM_set = RPM_SET;
				motor_dd.RPM_set = RPM_SET;
				motor_ti.RPM_set = RPM_SET;
				motor_td.RPM_set = RPM_SET;
				if (RPM_SET != 0) {
					Error_PID(&motor_dd);
					Error_PID(&motor_di);
					Error_PID(&motor_td);
					Error_PID(&motor_ti);
					CtrlPID_DD_Control(motor_dd.error_RPM, &motor_dd.control);
					CtrlPID_DI_Control(motor_di.error_RPM, &motor_di.control);
					CtrlPID_TD_Control(motor_td.error_RPM, &motor_td.control);
					CtrlPID_TI_Control(motor_ti.error_RPM, &motor_ti.control);
					RPM_Cero(&motor_dd);	//RPM_SET = 0?
					RPM_Cero(&motor_di);
					RPM_Cero(&motor_td);
					RPM_Cero(&motor_ti);
					motor_dd.tension = motor_dd.control;
					motor_di.tension = motor_di.control;
					motor_td.tension = motor_td.control;
					motor_ti.tension = motor_ti.control;
				} else {
					Reset_PIDs(motor_di);
					Reset_PIDs(motor_dd);
					Reset_PIDs(motor_ti);
					Reset_PIDs(motor_td);
					motor_dd.error_RPM = 0;
					motor_di.error_RPM = 0;
					motor_ti.error_RPM = 0;
					motor_td.error_RPM = 0;
					motor_dd.control = 0;
					motor_di.control = 0;
					motor_td.control = 0;
					motor_ti.control = 0;
					motor_dd.tension = motor_dd.control;
					motor_di.tension = motor_di.control;
					motor_td.tension = motor_td.control;
					motor_ti.tension = motor_ti.control;
				}
			}
			//END CONTROL PID
			//TENSION A PONER EN MOTORES
			Tension2Duty(&motor_di);
			Tension2Duty(&motor_dd);
			Tension2Duty(&motor_td);
			Tension2Duty(&motor_ti);
			//END TENSION A PONER EN MOTORES
			//PERDIDA DE SENAL - SALE DEL ESTADO
			if (velocidad.perdida_senal_remoto >= REMOTO_PERDIDA_SENAL
					|| direccion.perdida_senal_remoto >= REMOTO_PERDIDA_SENAL || velocidad.remoto_cero == 0 || direccion.remoto_cero == 0) {
				velocidad.perdida_senal_remoto = 0;
				velocidad.ms = 0;
				velocidad.cuenta_remoto = 0;
				velocidad.remoto_cero = 300;
				direccion.perdida_senal_remoto = 0;
				direccion.ms = 0;
				direccion.cuenta_remoto = 0;
				direccion.remoto_cero = 300;
				ESTADO = PERDIDA_SENAL;
			}
			//END PERDIDA DE SENAL - SALE DEL ESTADO		  
			break;
		case LA_PC:
			WATCHDOG = WATCHDOG | WD_ESTADO;
			ESTADOANTERIOR = ESTADO;
			if (lectura_nueva) {
				motor_di.duty = DUTY_CERO;
				motor_dd.duty = DUTY_CERO;
				motor_ti.duty = DUTY_CERO;
				motor_td.duty = DUTY_CERO;
				lectura_nueva = false;
			}
			//LECTURA PC
			pc.direccion = (pc.direccion >= PC_LD) ? PC_LD : pc.direccion;
			pc.direccion = (pc.direccion <= PC_LI) ? PC_LI : pc.direccion;
			pap.direccion_set = Mapeo(pc.direccion, PC_LI, PC_LD,
					LIMITE_DIRECCION_IZQUIERDO, LIMITE_DIRECCION_DERECHO);
			pc.reversa = (pc.reversa >= REVERSA) ? REVERSA : pc.reversa;
			sentido_act = pc.reversa;
			
			Duty2Motor(&pc,&motor_di);
			Duty2Motor(&pc,&motor_dd);
			Duty2Motor(&pc,&motor_ti);
			Duty2Motor(&pc,&motor_td);
			//END LECTURA PC
			//CAMBIO DE SENTIDO
			if (sentido_act != sentido_ant) {
				if (Vel_Cero(motor_di, motor_dd, motor_td, motor_ti)) {
					sentido_ant = sentido_act;
					Out_Reversa_PutVal(sentido_act);
					motor_dd.duty = DUTY_CERO;
					motor_di.duty = DUTY_CERO;
					motor_td.duty = DUTY_CERO;
					motor_ti.duty = DUTY_CERO;
					//-
				} else {
					motor_dd.duty = DUTY_CERO;
					motor_di.duty = DUTY_CERO;
					motor_td.duty = DUTY_CERO;
					motor_ti.duty = DUTY_CERO;
				}
			}
			//END CAMBIO DE SENTIDO
			break;
		case LC_PC:
			ESTADOANTERIOR = ESTADO;
			if (lectura_nueva) {
				//RESET AL PASAR AL ESTADO LC_REMOTO
				cuenta_PID = 0;
				Reset_PIDs(motor_di);
				Reset_PIDs(motor_dd);
				Reset_PIDs(motor_ti);
				Reset_PIDs(motor_td);
				motor_dd.error_RPM = 0;
				motor_di.error_RPM = 0;
				motor_ti.error_RPM = 0;
				motor_td.error_RPM = 0;
				motor_dd.control = 0;
				motor_di.control = 0;
				motor_td.control = 0;
				motor_ti.control = 0;
				motor_dd.tension = motor_dd.control;
				motor_di.tension = motor_di.control;
				motor_td.tension = motor_td.control;
				motor_ti.tension = motor_ti.control;
				lectura_nueva = false;
				//END RESET
			}
			//LECTURA PC
			pc.direccion = (pc.direccion >= PC_LD) ? PC_LD : pc.direccion;
			pc.direccion = (pc.direccion <= PC_LI) ? PC_LI : pc.direccion;
			pap.direccion_set = Mapeo(pc.direccion, PC_LI, PC_LD,
					LIMITE_DIRECCION_IZQUIERDO, LIMITE_DIRECCION_DERECHO);
			pc.reversa = (pc.reversa >= REVERSA) ? REVERSA : pc.reversa;
			sentido_act = pc.reversa;					
			//END LECTURA PC
			//CAMBIO DE SENTIDO
			if (sentido_act != sentido_ant) {
				if (Vel_Cero(motor_di, motor_dd, motor_td, motor_ti)) {
					sentido_ant = sentido_act;
					Out_Reversa_PutVal(sentido_act);
					cuenta_PID = 0;
					Reset_PIDs(motor_di);
					Reset_PIDs(motor_dd);
					Reset_PIDs(motor_ti);
					Reset_PIDs(motor_td);
					motor_dd.error_RPM = 0;
					motor_di.error_RPM = 0;
					motor_ti.error_RPM = 0;
					motor_td.error_RPM = 0;
					motor_dd.control = 0;
					motor_di.control = 0;
					motor_td.control = 0;
					motor_ti.control = 0;
					motor_dd.tension = motor_dd.control;
					motor_di.tension = motor_di.control;
					motor_td.tension = motor_td.control;
					motor_ti.tension = motor_ti.control;
					//-
				} else {
					motor_dd.control = 0;
					motor_di.control = 0;
					motor_td.control = 0;
					motor_ti.control = 0;
					motor_dd.tension = motor_dd.control;
					motor_di.tension = motor_di.control;
					motor_td.tension = motor_td.control;
					motor_ti.tension = motor_ti.control;
				}
			}
			//END CAMBIO DE SENTIDO
			//CONTROL PID
			if (cuenta_PID >= MUESTREO_PID) { //100 milisegundos periodo de muestreo
				WATCHDOG = WATCHDOG | WD_ESTADO;
				cuenta_PID -= MUESTREO_PID;
				motor_di.RPM_set = (pc.rpm[MOTOR_DI] > SET_RPM_MAX) ? SET_RPM_MAX : pc.rpm[MOTOR_DI];
				motor_dd.RPM_set = (pc.rpm[MOTOR_DD] > SET_RPM_MAX) ? SET_RPM_MAX : pc.rpm[MOTOR_DD];
				motor_ti.RPM_set = (pc.rpm[MOTOR_TI] > SET_RPM_MAX) ? SET_RPM_MAX : pc.rpm[MOTOR_TI];
				motor_td.RPM_set = (pc.rpm[MOTOR_TD] > SET_RPM_MAX) ? SET_RPM_MAX : pc.rpm[MOTOR_TD];
				if (motor_di.RPM_set != 0) {
					Error_PID(&motor_di);
					CtrlPID_DI_Control(motor_di.error_RPM, &motor_di.control);
					RPM_Cero(&motor_di);
					motor_di.tension = motor_di.control;
				} else {
					Reset_PIDs(motor_di);
					motor_di.error_RPM = 0;
					motor_di.control = 0;
					motor_di.tension = motor_di.control;
				}
				if (motor_dd.RPM_set != 0) {
					Error_PID(&motor_dd);
					CtrlPID_DD_Control(motor_dd.error_RPM, &motor_dd.control);
					RPM_Cero(&motor_dd);
					motor_dd.tension = motor_dd.control;
				} else {
					Reset_PIDs(motor_dd);
					motor_dd.error_RPM = 0;
					motor_dd.control = 0;
					motor_dd.tension = motor_dd.control;
				}
				if (motor_ti.RPM_set != 0) {
					Error_PID(&motor_ti);
					CtrlPID_TI_Control(motor_ti.error_RPM, &motor_ti.control);
					RPM_Cero(&motor_ti);
					motor_ti.tension = motor_ti.control;
				} else {
					Reset_PIDs(motor_ti);
					motor_ti.error_RPM = 0;
					motor_ti.control = 0;
					motor_ti.tension = motor_ti.control;
				}
				if (motor_td.RPM_set != 0) {
					Error_PID(&motor_td);
					CtrlPID_TD_Control(motor_td.error_RPM, &motor_td.control);
					RPM_Cero(&motor_td);
					motor_td.tension = motor_td.control;
				} else {
					Reset_PIDs(motor_td);
					motor_td.error_RPM = 0;
					motor_td.control = 0;
					motor_td.tension = motor_td.control;
				}
			}
			//END CONTROL PID
			//TENSION A PONER EN MOTORES
			Tension2Duty(&motor_di);
			Tension2Duty(&motor_dd);
			Tension2Duty(&motor_td);
			Tension2Duty(&motor_ti);
			//END TENSION A PONER EN MOTORES
			break;
			
		case CALIBRACION:
			WATCHDOG = WATCHDOG | WD_ESTADO;
			Get_Remoto(&velocidad);
			Get_Remoto(&direccion);
			if (velocidad.ms != 0 || direccion.ms != 0 || !lectura_nueva) {
				if (velocidad.ms == velocidad.remoto_cero) {
					velocidad.cuenta_remoto++;
				} else {
					velocidad.remoto_cero = velocidad.ms;
					velocidad.cuenta_remoto = 0;
				}
				if (direccion.ms == direccion.remoto_cero) {
					direccion.cuenta_remoto++;
				} else {
					direccion.remoto_cero = direccion.ms;
					direccion.cuenta_remoto = 0;
				}
				if (velocidad.cuenta_remoto >= CUENTAS_REMOTO
						&& direccion.cuenta_remoto >= CUENTAS_REMOTO) {
					velocidad.cuenta_remoto = 0;
					direccion.cuenta_remoto = 0;
					ESTADO = (ESTADOANTERIOR == CALIBRACION) ? LC_REMOTO : ESTADOANTERIOR;
					lectura_nueva = true;
				}
				if (velocidad.perdida_senal_remoto >= REMOTO_PERDIDA_SENAL
						|| direccion.perdida_senal_remoto
								>= REMOTO_PERDIDA_SENAL || velocidad.remoto_cero == 0 || direccion.remoto_cero == 0) {
					velocidad.perdida_senal_remoto = 0;
					velocidad.cuenta_remoto = 0;
					direccion.perdida_senal_remoto = 0;
					direccion.cuenta_remoto = 0;
				}
			}
			break;
		case PERDIDA_SENAL:
			WATCHDOG = WATCHDOG | WD_ESTADO;
			ESTADO = CALIBRACION;
			lectura_nueva = true;
			velocidad.remoto_cero = 0;
			direccion.remoto_cero = 0;
			velocidad.cuenta_remoto = 0;
			direccion.cuenta_remoto = 0;
			Reset_PIDs(motor_di);
			Reset_PIDs(motor_dd);
			Reset_PIDs(motor_ti);
			Reset_PIDs(motor_td);
			motor_dd.error_RPM = 0;
			motor_di.error_RPM = 0;
			motor_ti.error_RPM = 0;
			motor_td.error_RPM = 0;
			motor_dd.control = 0;
			motor_di.control = 0;
			motor_td.control = 0;
			motor_ti.control = 0;
			motor_dd.tension = motor_dd.control;
			motor_di.tension = motor_di.control;
			motor_td.tension = motor_td.control;
			motor_ti.tension = motor_ti.control;
			motor_dd.duty = DUTY_CERO;
			motor_di.duty = DUTY_CERO;
			motor_td.duty = DUTY_CERO;
			motor_ti.duty = DUTY_CERO;			
			RPM_SET = 0;
			break;
		default:
			break;
		}
		//DUTY MOTORES
		if (x2 != false){
			motor_dd.error_RPM = 0;
			motor_di.error_RPM = 0;
			motor_dd.control = 0;
			motor_di.control = 0;
			motor_dd.tension = motor_dd.control;
			motor_di.tension = motor_di.control;
			motor_dd.duty = DUTY_CERO;
			motor_di.duty = DUTY_CERO;
		}
		SetDuty(motor_dd);
		SetDuty(motor_di);
		SetDuty(motor_td);
		SetDuty(motor_ti);
		//END DUTY MOTORES
		
		//####### DIRECCION
		//HABILITO DIRECCION
		pc.pap_ha = (pc.pap_ha >= 1) ? 1 : pc.pap_ha;
		pc.pap_en = (pc.pap_en >= 1) ? 1 : pc.pap_en;
		if (Vel_Cero(motor_di, motor_dd, motor_td, motor_ti)
				&& (pap.direccion_set == pap.direccion_lectura)) {
			pap.FLAG_EN = false;
			pap.FLAG_HABILITADO = false;
		} else {
			pap.FLAG_EN = true;
			pap.FLAG_HABILITADO = true;
		}
		//END HABILITO DIRECCION		
		if (pap.FLAG_EN && pc.pap_en) {
			DIRECCION_ON;
		} else {
			DIRECCION_OFF;
		}
		if (pap.FLAG_HABILITADO && pc.pap_ha) {
			pap.FLAG_HABILITADO = false;
			pap.direccion_set =
					(pap.direccion_set >= LIMITE_DIRECCION_DERECHO) ?
							LIMITE_DIRECCION_DERECHO : pap.direccion_set;
			pap.direccion_set =
					(pap.direccion_set <= LIMITE_DIRECCION_IZQUIERDO) ?
							LIMITE_DIRECCION_IZQUIERDO : pap.direccion_set;
			if (pap.direccion_set
					> (pap.direccion_lectura + VENTANA_DIRECCION)) {
				pap.FLAG_SENTIDO = DERECHA;
				pap.FLAG_DIRECCION = true;
			}
			if (pap.direccion_set
					< (pap.direccion_lectura - VENTANA_DIRECCION)) {
				pap.FLAG_SENTIDO = IZQUIERDA;
				pap.FLAG_DIRECCION = true;
			}
			if (pap.FLAG_SENTIDO) {
				DIRECCION_HORARIA;
			} else {
				DIRECCION_ANTI;
			}
		}
		if ((pap.direccion_set <= (pap.direccion_lectura + VENTANA_DIRECCION))
				&& (pap.direccion_set
						>= (pap.direccion_lectura - VENTANA_DIRECCION))) {
			pap.FLAG_DIRECCION = false;
			pap.pwm_direccion = 0;
		}
		//######## END DIRECCION
		//RX
		RX();
		//END RX
		//TX
		serie.HAB_TX = (pc.ha_TX >= 1) ? 1 : pc.ha_TX;
		if (cuenta_TX >= 100 && serie.HAB_TX) {
			TEXT_strcpy(serie.tx_buf, sizeof(serie.tx_buf),(unsigned char*) "(");
			TEXT_strcatNum8u(serie.tx_buf, sizeof(serie.tx_buf),ESTADO);
			TEXT_chcat(serie.tx_buf, sizeof(serie.tx_buf), TXDOSPUNTOS);
			StrcatMotor(motor_di);
			StrcatMotor(motor_dd);
			StrcatMotor(motor_ti);
			StrcatMotor(motor_td);
			TEXT_strcatNum8s(serie.tx_buf, sizeof(serie.tx_buf),(byte)Mapeo(pap.direccion_lectura,LIMITE_DIRECCION_IZQUIERDO,LIMITE_DIRECCION_DERECHO,PC_LI,PC_LD));
			TEXT_chcat(serie.tx_buf, sizeof(serie.tx_buf), TXSEPARADOR);
			TEXT_strcatNum8u(serie.tx_buf, sizeof(serie.tx_buf),(word)sentido_act);
			TEXT_chcat(serie.tx_buf, sizeof(serie.tx_buf), TXCERRADO);
			TEXT_chcat(serie.tx_buf, sizeof(serie.tx_buf), '\r');
			TEXT_chcat(serie.tx_buf, sizeof(serie.tx_buf), '\n');
			NumeroFin(&serie);
			cuenta_TX -= 100;
			serie.FLAG_TX = true;
		}
		TXS(&serie);
		//END TX

		//PULSADOR
		if (FLAG_SW1 && cuenta_x2 >= 500) {
			x2 =! x2;
			FLAG_SW1 = false;
			cuenta_x2 = 0;
		}
		//END PULSADOR
		if (WATCHDOG = WD_CANTIDADBYTES){
			WDog1_Clear();
			WATCHDOG = 0;
		}
		//LUCES Y MAS
		if (cuenta_RX >= 500) {
			cuenta_RX -= 500;
			//FLAG_TX = true;		  
		}
		if (cnt_aux >= 100 && ESTADO == CALIBRACION) {
			cnt_aux -= 100;
			BitLed_Azul_NegVal();
			Status_LED_NegVal();
		}
		if (cnt_aux >= 500 && ESTADO != CALIBRACION) {
			cnt_aux -= 500;
			BitLed_Azul_NegVal();
			Status_LED_NegVal();
		}
		//END LUCES Y MAS
	}

	/*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;){}
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/

/* END main */
/*!
 ** @}
 */
/*
 ** ###################################################################
 **
 **     This file was created by Processor Expert 10.3 [05.09]
 **     for the Freescale Kinetis series of microcontrollers.
 **
 ** ###################################################################
 */
void Get_Corriente(void) {
	uint16 value[4];
	if (FLAG_ADC) {
		FLAG_ADC = false;
		ADC_I_GetChanValue16(MOTOR_DI,&motor_di.adc);
		ADC_I_GetChanValue16(MOTOR_DD,&motor_dd.adc);
		ADC_I_GetChanValue16(MOTOR_TD,&motor_td.adc);
		ADC_I_GetChanValue16(MOTOR_TI,&motor_ti.adc);
		ADC_I_Measure(FALSE);
		motor_di.i = Mapeo(motor_di.adc,MIN16BIT,MAX16BIT,V0,V33);
		motor_dd.i = Mapeo(motor_dd.adc,MIN16BIT,MAX16BIT,V0,V33);
		motor_td.i = Mapeo(motor_td.adc,MIN16BIT,MAX16BIT,V0,V33);
		motor_ti.i = Mapeo(motor_ti.adc,MIN16BIT,MAX16BIT,V0,V33);
	}
	WATCHDOG = WATCHDOG | WD_CORRIENTE;
}
void Get_Direccion(PAP *pap_x) {
	byte lectura;
	byte z;
	lectura = 0;
	z = (!BIT0_GetVal()) ? 1 : 0;
	lectura = lectura + z;
	z = (!BIT1_GetVal()) ? 2 : 0;
	lectura = lectura + z;
	z = (!BIT2_GetVal()) ? 4 : 0;
	lectura = lectura + z;
	z = (!BIT3_GetVal()) ? 8 : 0;
	lectura = lectura + z;
	z = (!BIT4_GetVal()) ? 16 : 0;
	lectura = lectura + z;
	z = (!BIT5_GetVal()) ? 32 : 0;
	lectura = lectura + z;
	z = (!BIT6_GetVal()) ? 64 : 0;
	lectura = lectura + z;
	z = (!BIT7_GetVal()) ? 128 : 0;
	lectura = lectura + z;
	lectura = GrayToBin(lectura);
	lectura =
			(lectura >= LIMITE_DIRECCION_DERECHO) ?
					LIMITE_DIRECCION_DERECHO : lectura;
	lectura =
			(lectura <= LIMITE_DIRECCION_IZQUIERDO) ?
					LIMITE_DIRECCION_IZQUIERDO : lectura;
	pap_x->direccion_lectura = lectura;
	WATCHDOG = WATCHDOG | WD_DIRECCION;
}
void SetDuty(MOTOR motor_x) {
	switch (motor_x.nro) {
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

void TXS(SERIE *serie_x) {
	word *SND;
	word snd;
	SND = &snd;
	snd = 0;
	if (serie_x->FLAG_TX) {
		serie_x->FLAG_TX = false;
		UART_MODBUS_SendBlock(serie_x->tx_buf,
				(word) (serie_x->tx_next - serie_x->tx_sent), SND);
		serie_x->tx_next = 0;
		serie_x->tx_sent = 0;
	}
}
void StrcatMotor(MOTOR motor_x){
	TEXT_strcatNum8u(serie.tx_buf, sizeof(serie.tx_buf),
			(word) motor_x.nro);
	TEXT_chcat(serie.tx_buf, sizeof(serie.tx_buf), TXSEPARADOR);
	TEXT_strcatNum16u(serie.tx_buf, sizeof(serie.tx_buf),
			(word) motor_x.rpm);
	TEXT_chcat(serie.tx_buf, sizeof(serie.tx_buf), TXSEPARADOR);
	TEXT_strcatNum32u(serie.tx_buf, sizeof(serie.tx_buf),
			(word) motor_x.pulsos);
	TEXT_chcat(serie.tx_buf, sizeof(serie.tx_buf), TXSEPARADOR);
	TEXT_strcatNum16u(serie.tx_buf, sizeof(serie.tx_buf),
			(word) motor_x.i);
	TEXT_chcat(serie.tx_buf, sizeof(serie.tx_buf), TXSEPARADOR);	
}
void RX(void){
	bool VALIDAR = true;
	uint8 aux8 = 0;
	uint16 aux16 = 0;
	uint8 var8 = 0;
	uint8 *pvar8;
	int8 DIR = 0;
	uint8 REVER = 0;
	uint8 DUTYMAX = 0;
	uint16 RPMMAX = 0;
	uint8 DUTY[4] = {0,0,0,0};
	uint16 RPM[4] = {0,0,0,0};
	pvar8 = &var8;
	pvar8 = &serie.rx_buf[serie.rx_read];
	if (serie.FLAG_RX && serie.HAB_RX) {	
		if (*pvar8 == TXABIERTO){
			pvar8++;
			VALIDAR = (TEXT_ScanDecimal8uNumber((const unsigned char**)&pvar8,&aux8) != ERR_OK) ? false : VALIDAR;
			VALIDAR = (*pvar8 != TXDOSPUNTOS) ? false : VALIDAR;
			pvar8++;
			switch (aux8){
			case (LA_REMOTO):
					//DUTY MAXIMO 0 a 100
					VALIDAR = (TEXT_ScanDecimal8uNumber((const unsigned char**)&pvar8,&aux8) != ERR_OK) ? false : VALIDAR;
					VALIDAR = (*pvar8 != TXCERRADO) ? VALIDAR = false : VALIDAR;
					if (VALIDAR){
						ESTADO = LA_REMOTO;
						duty_max_control = (aux8 == 0) ? DUTY_CERO : (uint16)Mapeo(aux8, PC_LDUTYMIN, PC_LDUTYMAX,DUTY_MIN, DUTY_MAX);						
					}
					break;
			case (LC_REMOTO):
					//RPM MMAXIMO 0 a 300
					VALIDAR = (TEXT_ScanDecimal16uNumber((const unsigned char**)&pvar8,&aux16) != ERR_OK) ? false : VALIDAR;
					VALIDAR = (*pvar8 != TXCERRADO) ? false : VALIDAR;
					if (VALIDAR){
						ESTADO = LC_REMOTO;
						rpm_max_control = (aux16 >= SET_RPM_MAX) ? SET_RPM_MAX : aux16;						
					}			
					break;
			case (LA_PC):
					//DUTY EN CADA RUEDA + RETROCESO + DIRECCION GIRO
					VALIDAR = (TEXT_ScanDecimal8uNumber((const unsigned char**)&pvar8,&aux8) != ERR_OK) ? false : VALIDAR;					
					DUTY[0] = aux8;
					VALIDAR = (*pvar8 != TXSEPARADOR) ? false : VALIDAR;
					pvar8++;
					VALIDAR = (TEXT_ScanDecimal8uNumber((const unsigned char**)&pvar8,&aux8) != ERR_OK) ? false : VALIDAR;					
					DUTY[1] = aux8;
					VALIDAR = (*pvar8 != TXSEPARADOR) ? false : VALIDAR;
					pvar8++;
					VALIDAR = (TEXT_ScanDecimal8uNumber((const unsigned char**)&pvar8,&aux8) != ERR_OK) ? false : VALIDAR;					
					DUTY[2] = aux8;
					VALIDAR = (*pvar8 != TXSEPARADOR) ? false : VALIDAR;
					pvar8++;
					VALIDAR = (TEXT_ScanDecimal8uNumber((const unsigned char**)&pvar8,&aux8) != ERR_OK) ? false : VALIDAR;					
					DUTY[3] = aux8;
					VALIDAR = (*pvar8 != TXSEPARADOR) ? false : VALIDAR;
					pvar8++;
					VALIDAR = (TEXT_ScanDecimal8uNumber((const unsigned char**)&pvar8,&aux8) != ERR_OK) ? false : VALIDAR;					
					REVER = aux8;
					VALIDAR = (*pvar8 != TXSEPARADOR) ? false : VALIDAR;
					pvar8++;
					if (*pvar8 == TXMENOS){
						pvar8++;					
						VALIDAR = (TEXT_ScanDecimal8uNumber((const unsigned char**)&pvar8,&aux8) != ERR_OK) ? false : VALIDAR;					
						DIR = (-aux8 <= PC_LI) ? PC_LI : -aux8;						
					} else {					
						VALIDAR = (TEXT_ScanDecimal8uNumber((const unsigned char**)&pvar8,&aux8) != ERR_OK) ? false : VALIDAR;
						DIR = (aux8 >= PC_LD) ? PC_LD : aux8;				
					}
					VALIDAR = (*pvar8 != TXCERRADO) ? false : VALIDAR;
					pvar8++;
					if (VALIDAR){
						ESTADO = LA_PC;
						pc.duty[MOTOR_DI] = DUTY[0];
						pc.duty[MOTOR_DD] = DUTY[1];
						pc.duty[MOTOR_TI] = DUTY[2];
						pc.duty[MOTOR_TD] = DUTY[3];
						pc.reversa = REVER;
						pc.direccion = DIR;
					}
					break;
			case (LC_PC):
					//RPM EN CADA RUEDA + RETROCESO + DIRECCION GIRO
					VALIDAR = (TEXT_ScanDecimal8uNumber((const unsigned char**)&pvar8,&aux8) != ERR_OK) ? false : VALIDAR;					
					RPM[0] = aux8;
					VALIDAR = (*pvar8 != TXSEPARADOR) ? false : VALIDAR;
					pvar8++;
					VALIDAR = (TEXT_ScanDecimal8uNumber((const unsigned char**)&pvar8,&aux8) != ERR_OK) ? false : VALIDAR;					
					RPM[1] = aux8;
					VALIDAR = (*pvar8 != TXSEPARADOR) ? false : VALIDAR;
					pvar8++;
					VALIDAR = (TEXT_ScanDecimal8uNumber((const unsigned char**)&pvar8,&aux8) != ERR_OK) ? false : VALIDAR;					
					RPM[2] = aux8;
					VALIDAR = (*pvar8 != TXSEPARADOR) ? false : VALIDAR;
					pvar8++;
					VALIDAR = (TEXT_ScanDecimal8uNumber((const unsigned char**)&pvar8,&aux8) != ERR_OK) ? false : VALIDAR;					
					RPM[3] = aux8;
					VALIDAR = (*pvar8 != TXSEPARADOR) ? false : VALIDAR;
					pvar8++;
					VALIDAR = (TEXT_ScanDecimal8uNumber((const unsigned char**)&pvar8,&aux8) != ERR_OK) ? false : VALIDAR;					
					REVER = aux8;
					VALIDAR = (*pvar8 != TXSEPARADOR) ? false : VALIDAR;
					pvar8++;
					if (*pvar8 == TXMENOS){
						pvar8++;					
						VALIDAR = (TEXT_ScanDecimal8uNumber((const unsigned char**)&pvar8,&aux8) != ERR_OK) ? false : VALIDAR;					
						DIR = (-aux8 <= PC_LI) ? PC_LI : -aux8;						
					} else {					
						VALIDAR = (TEXT_ScanDecimal8uNumber((const unsigned char**)&pvar8,&aux8) != ERR_OK) ? false : VALIDAR;
						DIR = (aux8 >= PC_LD) ? PC_LD : aux8;				
					}
					VALIDAR = (*pvar8 != TXCERRADO) ? false : VALIDAR;
					pvar8++;
					if (VALIDAR){
						ESTADO = LC_PC;
						pc.rpm[MOTOR_DI] = RPM[0];
						pc.rpm[MOTOR_DD] = RPM[1];
						pc.rpm[MOTOR_TI] = RPM[2];
						pc.rpm[MOTOR_TD] = RPM[3];
						pc.reversa = REVER;
						pc.direccion = DIR;
					}					
					break;
			default:
				VALIDAR = false;
				break;
			}
			
		}
		serie.FLAG_RX = false;
		serie.rx_next = 0;
		serie.rx_read = 0;
	} 
}
