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
#include "UART.h"
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
#include "Input_Hall_DD.h"
#include "CaptureLdd1.h"
#include "Input_Hall_TD.h"
#include "CaptureLdd2.h"
#include "Input_Hall_TI.h"
#include "CaptureLdd3.h"
#include "Input_Hall_DI.h"
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
#include "Hall_TD.h"
#include "BitIoLdd16.h"
#include "Hall_DD.h"
#include "BitIoLdd17.h"
#include "Hall_DI.h"
#include "BitIoLdd18.h"
#include "Hall_TI.h"
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

void duty_remoto(void);
void rpm_remoto(void);
void duty_pc(void);
void rpm_pc(void);
void perdida_senal(void);
void calibracion(void);
void set_direccion(void);
void set_x2(void);
void pulsador(void);
void led_aux(void);
void Init(void);
void GetCorriente(void);
void GetDireccion(PAP *pap_x);
void SetDuty(MOTOR motor_x);
void TX(SERIE *serie_x);
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
byte FLAG_SW1;						//FLAG DEL PULSADOR 1
word cnt_aux;						//Cuenta AUXILIAR en interrupcion INTTIEMPO
uint16 cuenta_x2 = 0;				//Cuenta x2
word cuenta_PID;					//Cuenta de tiempo PID
word cuenta_RX;						//Cuenta de tiempo RECIBIR
word cuenta_TX;						//Cuenta de tiempo RECIBIR
word cuenta_DIRECCION = 0;			//Cuenta para leer direccion
byte FLAG_ADC;						//FLAG de ADC TERMINO

// ######################## VARIABLES INTERNAS ###############################

bool cambio_estado;					//Cuando se cambia de estado, sirve para ejecutar 
									//una porcion de codigo por unica vez

uint16 emergencias = 0;				//Cuenta las entradas a emergencia

word RPM_SET = 0;					//Setpoint RPM Global
word DUTY_SET = 0;					//Setpoint TENSION Global
bool sentido_ant;					//Sentido anterior de GIRO MOTOR 			1-REVERSA
bool sentido_act;					//Sentido actual que se desea GIRAR MOTOR 	1-REVERSA

uint8 x2 = false;					//Determina si esta activa la traccion SOLO con DOS RUEDAS

uint16 rpm_max_control = 40;				//Setea el MAXIMO de RPM que se puede aplicar en MODO REMOTO RPM
uint16 duty_max_control = DUTY_MAX;	//Setea el MAXIMO de DUTY que se puede aplicar en MODO REMOTO DUTY

byte FLAG_PASOS;					//FLAG usada en el ESTADO PASOS
byte FLAG_DIRECCION = false;		//FLAG usado para definir cuando hay una seÃ±al de direccion

word tension_global = 0;			//Tension aplicable a todas las ruedas
word duty_global = DUTY_CERO;		//Duty aplicable a todas las ruedas

//###################### INICIALIZACIÓN DE LOS STRUCT ###############################

MOTOR motor_di = { MOTOR_DI, { MOTOR_DI, { 0, 0 }, 0, 0, 0, RISING, 0, 0, 0 }, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, K_PID, 0, TI_PID, 0, DUTY_CERO, 0 };
MOTOR motor_dd = { MOTOR_DD, { MOTOR_DD, { 0, 0 }, 0, 0, 0, RISING, 0, 0, 0 }, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, K_PID, 0, TI_PID, 0, DUTY_CERO, 0 };
MOTOR motor_ti = { MOTOR_TI, { MOTOR_TI, { 0, 0 }, 0, 0, 0, RISING, 0, 0, 0 }, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, K_PID, 0, TI_PID, 0, DUTY_CERO, 0 };
MOTOR motor_td = { MOTOR_TD, { MOTOR_TD, { 0, 0 }, 0, 0, 0, RISING, 0, 0, 0 }, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, K_PID, 0, TI_PID, 0, DUTY_CERO, 0 };

REMOTO direccion;
REMOTO velocidad;

PAP pap = { 0, 0, 0, 0, DIRECCION_CENTRO, false, false, false, false };

PC pc = { DUTY_CERO, {DUTY_CERO, DUTY_CERO, DUTY_CERO, DUTY_CERO}, SET_RPM_MIN, {SET_RPM_MIN, SET_RPM_MIN, SET_RPM_MIN, SET_RPM_MIN}, 0, 1, 1, ADELANTE, true};

SERIE serie;

/*lint -save  -e970 Disable MISRA rule (6.3) checking. */

int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */

{
	//LDD_TDeviceData *DeviceDataPtr;

	/* Write your local variable definition here */

	/*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
	PE_low_level_init();
	/*** End of Processor Expert internal initialization.                    ***/

	/* Write your code here */

	//################ INICIALIZAR VARIABLES
	//PID
	CtrlPID_DD_Set_K((float) motor_dd.k);
	CtrlPID_DD_Set_Ti((float) (motor_dd.ti / 10));
	CtrlPID_DI_Set_K((float) motor_di.k);
	CtrlPID_DI_Set_Ti((float) (motor_di.ti / 10));
	CtrlPID_TD_Set_K((float) motor_td.k);
	CtrlPID_TD_Set_Ti((float) (motor_td.ti / 10));
	CtrlPID_TI_Set_K((float) motor_ti.k);
	CtrlPID_TI_Set_Ti((float) (motor_ti.ti / 10));
	//#######################################
	/*
	*/
	Init();
	for (;;) {
		WATCHDOG = WATCHDOG | WD_PROGRAMA;			//WATCHDOG DEL PROGRAMA
		if (ESTADO != ESTADOANTERIOR) {				//HUBO CAMBIO DE ESTADO?
			cambio_estado = true;
		}
		//INICIO
		GetDireccion(&pap);							//LEER DIRECCION
		GetHall(&motor_dd);							//LEER HALL MOTOR DD
		GetHall(&motor_di);							//LEER HALL MOTOR DI
		GetHall(&motor_td);							//LEER HALL MOTOR TD
		GetHall(&motor_ti);							//LEER HALL MOTOR TI
		GetVelocidad(&motor_dd);					//LEER VELOCIDAD MOTOR DD
		GetVelocidad(&motor_di);					//LEER VELOCIDAD MOTOR DI
		GetVelocidad(&motor_td);					//LEER VELOCIDAD MOTOR TD
		GetVelocidad(&motor_ti);					//LEER VELOCIDAD MOTOR TI
		GetCorriente();								//LEER CORRIENTE

		switch (ESTADO) {
		case DUTY_REMOTO:
			duty_remoto();
			break;
		case RPM_REMOTO:
			rpm_remoto();
			break;
		case DUTY_PC:
			duty_pc();
			break;
		case RPM_PC:
			rpm_pc();
			break;			
		case CALIBRACION:
			calibracion();
			break;
		case PERDIDA_SENAL:
			perdida_senal();
			break;
		default:
			ESTADO = PERDIDA_SENAL;
			break;
		}
		//DUTY MOTORES
		set_x2();									//Cambio a traccion con las 2 ruedas traseras
		SetDuty(motor_dd);
		SetDuty(motor_di);
		SetDuty(motor_td);
		SetDuty(motor_ti);
		//END DUTY MOTORES
		
		//####### DIRECCION
		set_direccion();
		//######## END DIRECCION
		//RX
		RX();
		//END RX
		//TX
		TX(&serie);
		//END TX

		//PULSADOR
		pulsador();
		//END PULSADOR
		//LUCES Y MAS
		led_aux();
		//END LUCES Y MAS
		//WATCHDOG
		if (WATCHDOG = WD_CANTIDADBYTES){
			WDog1_Clear();
			WATCHDOG = 0;
		}
		//END WATCHDOG
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
void Init(void){
	/*
	 * Inicializar variblaes del programa
	 */
	ADC_I_Measure(FALSE);
	DIRECCION_ON;
	ESTADO = CALIBRACION;
	ESTADOANTERIOR = ESTADO;
	rpm_max_control = 40;
	pap.FLAG_HABILITADO = false;
	pap.FLAG_EN = true;
	serie.HAB_TX = true;
	serie.HAB_RX = true;
	cambio_estado = true;
}
void GetCorriente(void) {
	uint16 value[4];
	if (FLAG_ADC) {			//Conversión Completa del ADC
		FLAG_ADC = false;	//Reset Flag ADC
		/*
		 * Lectura de cada uno de los canales del ADC.
		 * Conversión a Corriente. Pasaje de valores de 16 bits
		 * a valores de Corriente
		 */
		ADC_I_GetChanValue16(MOTOR_DI,&motor_di.adc);
		ADC_I_GetChanValue16(MOTOR_DD,&motor_dd.adc);
		ADC_I_GetChanValue16(MOTOR_TD,&motor_td.adc);
		ADC_I_GetChanValue16(MOTOR_TI,&motor_ti.adc);
		ADC_I_Measure(FALSE);
		/*
		 * Con la finalidad de no perder precision en los calculos,
		 * 	se envian los valores de corriente en formato binario
		 * 	de 16 bit, con los datos provenientes del ADC
		 */
		/*
		 * Mapeo realizado anteriormente a valores de tensión
		motor_di.i = Mapeo(motor_di.adc,MIN16BIT,MAX16BIT,V0,V33);
		motor_dd.i = Mapeo(motor_dd.adc,MIN16BIT,MAX16BIT,V0,V33);
		motor_td.i = Mapeo(motor_td.adc,MIN16BIT,MAX16BIT,V0,V33);
		motor_ti.i = Mapeo(motor_ti.adc,MIN16BIT,MAX16BIT,V0,V33);
		*/
		motor_di.i = motor_di.adc;
		motor_dd.i = motor_dd.adc;
		motor_td.i = motor_td.adc;
		motor_ti.i = motor_ti.adc;
		
	}
	WATCHDOG = WATCHDOG | WD_CORRIENTE;
}
void GetDireccion(PAP *pap_x) {
	/*
	 * Lectura de la direccion
	 * Leer las 8 entradas, concatenarlas y decodificiarlas
	 * 	a binario y tomar lectura de la posición del Encoder
	 */
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
	/*
	 * Setear DUTY CYCLE del PWM correspondiente
	 * 	al driver de cada motor
	 */
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

void TX(SERIE *serie_x) {
	/*
	 * Inicializar variables utilizadas en la trasmición
	 */
	word *SND;
	word snd;
	SND = &snd;
	snd = 0;
	/*
	 * Verificar que esta habilitada la trasmición
	 * Realizar la codificacion de los datos del Motor
	 * 	,concatenarlos en un String y enviarlos.
	 */
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
		//BORRAR
		TEXT_chcat(serie.tx_buf, sizeof(serie.tx_buf), TXSEPARADOR);
		TEXT_strcatNum16u(serie.tx_buf, sizeof(serie.tx_buf),(word)motor_dd.tension);
		//BORRAR
		TEXT_chcat(serie.tx_buf, sizeof(serie.tx_buf), TXCERRADO);
		TEXT_chcat(serie.tx_buf, sizeof(serie.tx_buf), '\r');
		TEXT_chcat(serie.tx_buf, sizeof(serie.tx_buf), '\n');
		NumeroFin(&serie);
		cuenta_TX -= 100;
		serie.FLAG_TX = true;
	}
	if (serie_x->FLAG_TX) {
		serie_x->FLAG_TX = false;
		UART_SendBlock(serie_x->tx_buf,(word) (serie_x->tx_next - serie_x->tx_sent), SND);
		serie_x->tx_next = 0;
		serie_x->tx_sent = 0;
	}
}
void StrcatMotor(MOTOR motor_x){
	/*
	 * Pasa los datos del motor a un String
	 */
	TEXT_strcatNum8u(serie.tx_buf, sizeof(serie.tx_buf),
			(word) motor_x.nro);
	TEXT_chcat(serie.tx_buf, sizeof(serie.tx_buf), TXSEPARADOR);
	TEXT_strcatNum16u(serie.tx_buf, sizeof(serie.tx_buf),
			(word) motor_x.rpm);
	TEXT_chcat(serie.tx_buf, sizeof(serie.tx_buf), TXSEPARADOR);
	TEXT_strcatNum32u(serie.tx_buf, sizeof(serie.tx_buf),
			(word) motor_x.posicion_pulsos);
	TEXT_chcat(serie.tx_buf, sizeof(serie.tx_buf), TXSEPARADOR);
	TEXT_strcatNum16u(serie.tx_buf, sizeof(serie.tx_buf),
			(word) motor_x.i);
	TEXT_chcat(serie.tx_buf, sizeof(serie.tx_buf), TXSEPARADOR);	
}
void RX(void){
	/*
	 * Inicializar variables utilizadas en la validacion
	 * 	del String recibido.
	 */
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
	/*
	 * Verificar si se recibio un String y si esta habilitada la recepción
	 */
	if (serie.FLAG_RX && serie.HAB_RX) {
		pvar8 = &var8;
		pvar8 = &serie.rx_buf[serie.rx_read];	
		/*
		 * Realiza la DECODIFICACIÓN y VALIDACION de los
		 * 	datos recibidos.
		 */
		if (*pvar8 == TXABIERTO){
			pvar8++;
			VALIDAR = (TEXT_ScanDecimal8uNumber((const unsigned char**)&pvar8,&aux8) != ERR_OK) ? false : VALIDAR;
			VALIDAR = (*pvar8 != TXDOSPUNTOS) ? false : VALIDAR;
			pvar8++;
			switch (aux8){
			case (DUTY_REMOTO):
					//DUTY MAXIMO 0 a 100
					VALIDAR = (TEXT_ScanDecimal8uNumber((const unsigned char**)&pvar8,&aux8) != ERR_OK) ? false : VALIDAR;
					VALIDAR = (*pvar8 != TXCERRADO) ? VALIDAR = false : VALIDAR;
					if (VALIDAR){
						ESTADO = DUTY_REMOTO;
						duty_max_control = (aux8 == 0) ? DUTY_CERO : (uint16)Mapeo(aux8, PC_LDUTYMIN, PC_LDUTYMAX,DUTY_MIN, DUTY_MAX);						
					}
					break;
			case (RPM_REMOTO):
					//RPM MMAXIMO 0 a 300
					VALIDAR = (TEXT_ScanDecimal16uNumber((const unsigned char**)&pvar8,&aux16) != ERR_OK) ? false : VALIDAR;
					VALIDAR = (*pvar8 != TXCERRADO) ? false : VALIDAR;
					if (VALIDAR){
						ESTADO = RPM_REMOTO;
						rpm_max_control = (aux16 >= SET_RPM_MAX) ? SET_RPM_MAX : aux16;						
					}			
					break;
			case (DUTY_PC):
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
						ESTADO = DUTY_PC;
						pc.duty[MOTOR_DI] = DUTY[0];
						pc.duty[MOTOR_DD] = DUTY[1];
						pc.duty[MOTOR_TI] = DUTY[2];
						pc.duty[MOTOR_TD] = DUTY[3];
						pc.reversa = REVER;
						pc.direccion = DIR;
					}
					break;
			case (RPM_PC):
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
						ESTADO = RPM_PC;
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
void duty_remoto(void){
	WATCHDOG = WATCHDOG | WD_ESTADO;
	ESTADOANTERIOR = ESTADO;
	if (cambio_estado) {
		/*
		 * Si hubo un cambio de estado reciente, se detecta
		 * 	y setea los valores de DUTY a CERO
		 */
		motor_di.duty = DUTY_CERO;
		motor_dd.duty = DUTY_CERO;
		motor_ti.duty = DUTY_CERO;
		motor_td.duty = DUTY_CERO;
		cambio_estado = false;
	}
	/*
	 * Toma lectura de las señales del control remoto en milisegundos
	 */
	Get_Remoto(&velocidad);
	Get_Remoto(&direccion);
	/*
	 * Se limita el ancho máximo y mínimo de la señal leida,
	 * 	tanto para velocidad como direccion.
	 * Setea direccion a PaP.
	 */
	//LEER DIRECCION
	direccion.ms = (direccion.ms >= direccion.remoto_cero + REMOTO_ANCHO_PULSO) ? direccion.remoto_cero + REMOTO_ANCHO_PULSO : direccion.ms;
	direccion.ms = (direccion.ms <= direccion.remoto_cero - REMOTO_ANCHO_PULSO) ? direccion.remoto_cero - REMOTO_ANCHO_PULSO : direccion.ms;
	pap.direccion_set = Mapeo(direccion.ms, direccion.remoto_cero - REMOTO_ANCHO_PULSO, direccion.remoto_cero + REMOTO_ANCHO_PULSO,	LIMITE_DIRECCION_IZQUIERDO, LIMITE_DIRECCION_DERECHO);
	//END LEER DIRECCION
	//LEER VELOCIDAD
	velocidad.ms = (velocidad.ms >= velocidad.remoto_cero + REMOTO_ANCHO_PULSO) ? velocidad.remoto_cero + REMOTO_ANCHO_PULSO : velocidad.ms;
	velocidad.ms = (velocidad.ms <= velocidad.remoto_cero - REMOTO_ANCHO_PULSO) ? velocidad.remoto_cero - REMOTO_ANCHO_PULSO : velocidad.ms;
	//END LEER VELOCIDAD
	/*
	 * Se detecta el sentido deseado de movimiento.
	 * Si el estado actual de movimiento es distinto al deseado,
	 * 	se setea la velocidad en cero hasta que sea posible cambiar
	 * 	el sentido de avance.
	 * La velocidad leida por el control remoto en milisegundos
	 * 	es mapeada a DUTY
	 */
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
	/*
	 * Limita el DUTY maximo.
	 * El maximo lo establece el usuario
	 */
	DUTY_SET = (DUTY_SET <= duty_max_control) ? duty_max_control : DUTY_SET;
	//END CAMBIO DE SENTIDO
	motor_di.duty = DUTY_SET;
	motor_dd.duty = DUTY_SET;
	motor_ti.duty = DUTY_SET;
	motor_td.duty = DUTY_SET;
	//PERDIDA DE SENAL - SALE DEL ESTADO
	/*
	 * Si no se recibe señal por parte del control remoto,
	 * 	sea de velocidad o de direcion, o hubo un error en la calibracion
	 * 	de la posicion CERO del control remoto
	 * 	el ESTADO del programa detecta PERDIDA DE SEÑAL
	 */
	if (velocidad.perdida_senal_remoto >= REMOTO_PERDIDA_SENAL
			|| direccion.perdida_senal_remoto >= REMOTO_PERDIDA_SENAL || velocidad.remoto_cero == 0 || direccion.remoto_cero == 0) {
		velocidad.perdida_senal_remoto = 0;
		direccion.perdida_senal_remoto = 0;
		ESTADO = PERDIDA_SENAL;
	}
	//END PERDIDA DE SENAL - SALE DEL ESTADO	
}
void rpm_remoto(void){
	WATCHDOG = WATCHDOG | WD_ESTADO;
	ESTADOANTERIOR = ESTADO;
	if (cambio_estado) {
		//RESET AL PASAR AL ESTADO RPM_REMOTO
		/*
		 * Si hubo un cambio de estado reciente, se detecta
		 * 	y setea los valores de los PID, errores
		 * 	y tensiones de control.
		 */
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
		cambio_estado = false;
		//END RESET
	}
	//LEER REMOTO
	/*
	 * Toma lectura de las señales del control remoto en milisegundos
	 */
	Get_Remoto(&velocidad);
	Get_Remoto(&direccion);
	/*
	 * Se limita el ancho máximo y mínimo de la señal leida,
	 * 	tanto para velocidad como direccion.
	 * Setea direccion a PaP.
	 */
	//END LEER REMOTO
	//LEER DIRECCION
	direccion.ms = (direccion.ms >= direccion.remoto_cero + REMOTO_ANCHO_PULSO) ? direccion.remoto_cero + REMOTO_ANCHO_PULSO : direccion.ms;
	direccion.ms = (direccion.ms <= direccion.remoto_cero - REMOTO_ANCHO_PULSO) ? direccion.remoto_cero - REMOTO_ANCHO_PULSO : direccion.ms;
	pap.direccion_set = Mapeo(direccion.ms, direccion.remoto_cero - REMOTO_ANCHO_PULSO, direccion.remoto_cero + REMOTO_ANCHO_PULSO,	LIMITE_DIRECCION_IZQUIERDO, LIMITE_DIRECCION_DERECHO);
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
	/*
	 * Se detecta el sentido deseado de movimiento.
	 * Si el estado actual de movimiento es distinto al deseado,
	 * 	se setea la velocidad en cero hasta que sea posible cambiar
	 * 	el sentido de avance.
	 * La velocidad leida por el control remoto en milisegundos
	 * 	es mapeada a RPM
	 */
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
		RPM_SET = Mapeo(velocidad.ms, velocidad.remoto_cero, velocidad.remoto_cero + REMOTO_ANCHO_PULSO, SET_RPM_MIN, rpm_max_control);
	} else {
		RPM_SET = Mapeo(velocidad.ms, velocidad.remoto_cero, velocidad.remoto_cero - REMOTO_ANCHO_PULSO, SET_RPM_MIN, rpm_max_control);
	}
	RPM_SET = (RPM_SET > rpm_max_control) ? rpm_max_control : RPM_SET;
	//END CAMBIO DE SENTIDO
	//CONTROL PID
	/*
	 * Actua el control PI de cada uno de los motores
	 * 	cada un periodo de "MUESTRO_PID" milisegundo.
	 * Si el setpoint de RPM es CERO, el sistema elude
	 * 	el control y setea las variables controladas y las tensiones
	 * 	aplicadas de los drivers a cero.
	 * 	
	 */
	if (cuenta_PID >= MUESTREO_PID) { //100 milisegundos periodo de muestreo
		WATCHDOG = WATCHDOG | WD_ESTADO;
		cuenta_PID -= MUESTREO_PID;
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
	/*
	 * Adaptación de DUTY a Tension de Control equivalente
	 */
	Tension2Duty(&motor_di);
	Tension2Duty(&motor_dd);
	Tension2Duty(&motor_td);
	Tension2Duty(&motor_ti);
	//END TENSION A PONER EN MOTORES
	//PERDIDA DE SENAL - SALE DEL ESTADO
	if (velocidad.perdida_senal_remoto >= REMOTO_PERDIDA_SENAL || direccion.perdida_senal_remoto >= REMOTO_PERDIDA_SENAL || velocidad.remoto_cero == 0 || direccion.remoto_cero == 0) {
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
}
void duty_pc(void){
	WATCHDOG = WATCHDOG | WD_ESTADO;
	ESTADOANTERIOR = ESTADO;
	if (cambio_estado) {
		motor_di.duty = DUTY_CERO;
		motor_dd.duty = DUTY_CERO;
		motor_ti.duty = DUTY_CERO;
		motor_td.duty = DUTY_CERO;
		cambio_estado = false;
	}
	//LECTURA PC
	/*
	 * Se comprueban los limites de los valores ingresados por PC.
	 * Si el estado actual de movimiento es distinto al deseado,
	 * 	se setea la velocidad en cero hasta que sea posible cambiar
	 * 	el sentido de avance.
	 * Se setea el DUTY de los PWM correspondientes a cada driver
	 */
	pc.direccion = (pc.direccion >= PC_LD) ? PC_LD : pc.direccion;
	pc.direccion = (pc.direccion <= PC_LI) ? PC_LI : pc.direccion;
	pap.direccion_set = Mapeo(pc.direccion, PC_LI, PC_LD, LIMITE_DIRECCION_IZQUIERDO, LIMITE_DIRECCION_DERECHO);
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
		}
		motor_dd.duty = DUTY_CERO;
		motor_di.duty = DUTY_CERO;
		motor_td.duty = DUTY_CERO;
		motor_ti.duty = DUTY_CERO;
	}
	//END CAMBIO DE SENTIDO	
}
void rpm_pc(void){
	ESTADOANTERIOR = ESTADO;
	if (cambio_estado) {
		//RESET AL PASAR AL ESTADO RPM_REMOTO
		/*
		 * Si hubo un cambio de estado reciente, se detecta
		 * 	y setea los valores de los PID, errores
		 * 	y tensiones de control.
		 */
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
		cambio_estado = false;
		//END RESET
	}
	//LECTURA PC
	/*
	 * Se comprueban los limites de los valores ingresados por PC.
	 * Si el estado actual de movimiento es distinto al deseado,
	 * 	se setea el setpoint de RPM en cero hasta que sea posible cambiar
	 * 	el sentido de avance.
	 */
	pc.direccion = (pc.direccion >= PC_LD) ? PC_LD : pc.direccion;
	pc.direccion = (pc.direccion <= PC_LI) ? PC_LI : pc.direccion;
	pap.direccion_set = Mapeo(pc.direccion, PC_LI, PC_LD,LIMITE_DIRECCION_IZQUIERDO, LIMITE_DIRECCION_DERECHO);
	pc.reversa = (pc.reversa >= REVERSA) ? REVERSA : pc.reversa;
	sentido_act = pc.reversa;					
	//END LECTURA PC
	//CAMBIO DE SENTIDO
	if (sentido_act != sentido_ant) {
		if (Vel_Cero(motor_di, motor_dd, motor_td, motor_ti)) {
			sentido_ant = sentido_act;
			Out_Reversa_PutVal(sentido_act);
		}
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
	}
	//END CAMBIO DE SENTIDO
	//CONTROL PID
	/*
	 * Actua el control PI de cada uno de los motores por separado
	 * 	cada un periodo de "MUESTRO_PID" milisegundo.
	 * Si el setpoint de RPM es CERO, el sistema elude
	 * 	el control y setea las variables controladas y las tensiones
	 * 	aplicadas de los drivers a cero.
	 * 	
	 */
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
	
}
void perdida_senal(void){
	/*
	 * En caso de perder la señal del control remoto
	 * 	se pasa automaticamente a este estado y se
	 * 	resetean todos los valores de CALIBRACION,
	 * 	datos del PI, setpoint de velocidad y
	 * 	Tensiones en los drivers.
	 */
	WATCHDOG = WATCHDOG | WD_ESTADO;
	ESTADO = CALIBRACION;
	cambio_estado = true;
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
}
void calibracion(void){
	WATCHDOG = WATCHDOG | WD_ESTADO;
	Get_Remoto(&velocidad);
	Get_Remoto(&direccion);
	/*
	 * Toma lectura del control remoto y verifica que se este leyendo un valor
	 * 	distinto de CERO o bien si se llego al estado CALIBRACION por un
	 * 	cambio de estado.
	 * 
	 * Reliza varias lecturas de velocidad y direcion, si las mismas son iguales
	 * 	una cantidad de veces preestablecida, define ese valor como el de calibracion
	 * 	del CERO, tanto de velocidad como de direccion y vuelve al ESTADO ANTERIOR.
	 */
	if (velocidad.ms != 0 || direccion.ms != 0 || !cambio_estado) {
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
		if (velocidad.cuenta_remoto >= CUENTAS_REMOTO && direccion.cuenta_remoto >= CUENTAS_REMOTO) {
			velocidad.cuenta_remoto = 0;
			direccion.cuenta_remoto = 0;
			ESTADO = (ESTADOANTERIOR == CALIBRACION) ? RPM_REMOTO : ESTADOANTERIOR;
			cambio_estado = true;
		}
		if (velocidad.perdida_senal_remoto >= REMOTO_PERDIDA_SENAL || direccion.perdida_senal_remoto >= REMOTO_PERDIDA_SENAL || velocidad.remoto_cero == 0 || direccion.remoto_cero == 0) {
			velocidad.perdida_senal_remoto = 0;
			velocidad.cuenta_remoto = 0;
			direccion.perdida_senal_remoto = 0;
			direccion.cuenta_remoto = 0;
		}
	}	
}
void set_direccion(void){
	//HABILITO DIRECCION
	/*
	 * Verificar una correcta lectura de 
	 * 	habilitación
	 */
	pc.pap_ha = (pc.pap_ha >= 1) ? 1 : pc.pap_ha;
	pc.pap_en = (pc.pap_en >= 1) ? 1 : pc.pap_en;
	//END HABILITO DIRECCION
	//DESHABILITAR ENABLE
	/*
	 * Si la velocidad de lectura es cero y 
	 * 	el setpoint de direccion es igual a la
	 * 	lectura de direccion se deshabilita el
	 * 	driver del PaP
	 */
	if (Vel_Cero(motor_di, motor_dd, motor_td, motor_ti)
			&& (pap.direccion_set == pap.direccion_lectura)) {
		pap.FLAG_EN = false;
		pap.FLAG_HABILITADO = false;
	} else {
		pap.FLAG_EN = true;
		pap.FLAG_HABILITADO = true;
	}
	//END DESHABILITAR ENABLE
	//END HABILITO DIRECCION
	/*
	 * Habilita o Deshabilita el DRIVER del PaP
	 * 	permitiendo a este estar habilitado, manteniendo
	 * 	de esta manera el motor bloqueado o directamente
	 * 	deshabilitarlo por completo 
	 */
	if (pap.FLAG_EN && pc.pap_en) {
		DIRECCION_ON;
	} else {
		DIRECCION_OFF;
	}
	/*
	 * Habilita o Desahibilita el CONTROL de PaP
	 * 	Si se encuentra habilitado, realiza el control
	 * 	de la direccion.
	 * 	De no estarlo, queda deshabilitado el control
	 * 	permitiendo solamente controlar el bloqueo (retencion)
	 * 	o desbloqueo del driver
	 */
	if (pap.FLAG_HABILITADO && pc.pap_ha) {
		pap.FLAG_HABILITADO = false;
		pap.direccion_set = (pap.direccion_set >= LIMITE_DIRECCION_DERECHO) ? LIMITE_DIRECCION_DERECHO : pap.direccion_set;
		pap.direccion_set =	(pap.direccion_set <= LIMITE_DIRECCION_IZQUIERDO) ?	LIMITE_DIRECCION_IZQUIERDO : pap.direccion_set;
		if (pap.direccion_set > (pap.direccion_lectura + VENTANA_DIRECCION)) {
			pap.FLAG_SENTIDO = DERECHA;
			pap.FLAG_DIRECCION = true;
		}
		if (pap.direccion_set < (pap.direccion_lectura - VENTANA_DIRECCION)) {
			pap.FLAG_SENTIDO = IZQUIERDA;
			pap.FLAG_DIRECCION = true;
		}
		if (pap.FLAG_SENTIDO) {
			DIRECCION_HORARIA;
		} else {
			DIRECCION_ANTI;
		}
	}
	if ((pap.direccion_set <= (pap.direccion_lectura + VENTANA_DIRECCION)) && (pap.direccion_set >= (pap.direccion_lectura - VENTANA_DIRECCION))) {
		pap.FLAG_DIRECCION = false;
		pap.pwm_direccion = 0;
	}	
}
void set_x2(void){
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
}
void pulsador(void){
	if (FLAG_SW1 && cuenta_x2 >= 500) {
		x2 =! x2;
		FLAG_SW1 = false;
		cuenta_x2 = 0;
	}	
}
void led_aux(void){
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
}
