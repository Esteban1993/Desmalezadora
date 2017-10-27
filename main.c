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
//JUUAN
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
#include "Out_PWM_DI.h"
#include "PwmLdd1.h"
#include "TTemp.h"
#include "TPWM.h"
#include "Out_PWM_DD.h"
#include "PwmLdd3.h"
#include "Out_PWM_TD.h"
#include "PwmLdd2.h"
#include "TPulsos.h"
#include "Out_PWM_TI.h"
#include "PwmLdd4.h"
#include "Input_Encoder_DI.h"
#include "CaptureLdd1.h"
#include "Input_Encoder_DD.h"
#include "CaptureLdd2.h"
#include "Input_Encoder_TI.h"
#include "CaptureLdd3.h"
#include "Input_Encoder_TD.h"
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
/* Including shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "PDD_Includes.h"
#include "Init_Config.h"
#include "Struct.h"
#include "Defines.h"


/*
 * ######################## DECLARAR FUNCIONES ###############################
 */
long Mapeo(long x, long in_min, long in_max, long out_min, long out_max);	//Mapear valores
void ResetVar (void);				//Reset de las variables globales
void Get_Remoto (void);				//Recibi ancho de los pulsos de REMOTO
void Get_Velocidad (void);			//Recibe ancho de los pulsos del SENSOR HALL
void Get_Corriente (void);			//Lee los ADC de CORRIENTE
void Send_Velocidad(void); 			//Envia Velocidad
void Send_Corriente(void);			//Envia Corriente
void Send_RPM(void);				//Envia RPM
void Send_Calibrado(void);			//Envia Calibracion
void Control_LC(void);				//PID LC
void Send_Calibrado(void);			//Envia Calibracion
void Tension_PWM(void);				//Aplica la tension a los PWM
void RPM_Cero(void);				//Verifica que el SETPOINT de RPM sea CERO
bool Vel_Cero(void);				//Verifica que el motor este detenido
void Reset_PIDs(void);				//Resetea los valores de TODOS los PID
void RX(void);						//RECIBIR
void TX(void);						//ENVIAR
void Get_Direccion(void);			//Recibe CODIFICACION de POSICION DIRECCION
word GrayToBin(word N);				//Pasa de Cogigo GRAY a BINARIO
void DECODIFICADO(byte* codigo, byte inicio);	//Decodifica el mensaje recibido y lo ubica en las variables

/* User includes (#include below this line is not maintained by Processor Expert) */
/*
 * ######################## VARIABLES GLOBALES ###############################
 */
// INPUT CAPTURE
uint16_t Data[4][2];				//Contador de pulsos de cada flanco [1xMOTOR 2xVariableAnteriorYActual] 
word Period[4];						//Diferencia entre los pulsos 2 y 1 de cada MOTOR
byte index[4];						//Indice de pulsos
byte err[4];						//Por si ocurre un error al momento de leer pulsos

word cuenta_vel_cero[4];			//Contador para determinar Vel CERO si no se registran nuevos flancos
dlong posicion_pulsos[4];			//Contador de Flancos del Motor

uint16_t Data_Receptor[2][2];		//IDEM A ANTERIOR SOLO QUE PARA MEDIR ANCHO DE PULSO DE REMOTO
dword Period_Receptor[2];			//IDEM A ANTERIOR SOLO QUE PARA MEDIR ANCHO DE PULSO DE REMOTO
byte index_Receptor[2];				//IDEM A ANTERIOR SOLO QUE PARA MEDIR ANCHO DE PULSO DE REMOTO
byte err_Receptor[2];				//IDEM A ANTERIOR SOLO QUE PARA MEDIR ANCHO DE PULSO DE REMOTO

//ENVIAR Y RECIBIR
byte tx_buf[BUF_SIZE];            	// TX buffer
byte tx_next;						// TX indice siguiente a GUARDAR						
byte tx_sent;					  	// TX indice siguiente a MANDAR
byte rx_buf[BUF_SIZE];            	// RX buffer
byte rx_next;						// RX indice siguiente a GUARDAR
byte rx_read;  						// RX indice siguiente a LEER

// FLAGS
byte ESTADO;						//Indica el ESTADO del PROGRAMA
byte FLAG_DIRECCION_PWM_EN;			//PASO A PASO ENEABLE
byte FLAG_DIRECCION_SENTIDO;		//PASO A PASO SENTIDO DE GIRO
byte FLAG_SW1;						//FLAG DEL PULSADOR 1
byte FLAG_SW2;						//FLAG DEL PULSADOR 2
word cnt_aux;						//Cuenta AUXILIAR en interrupcion INTTIEMPO
word cuenta_PID;					//Cuenta de tiempo PID
word cuenta_RECIBIR;				//Cuenta de tiempo RECIBIR
word cuenta_ENVIAR;					//Cuenta de tiempo ENVIAR
word cuenta_RX;						//Cuenta de tiempo RECIBIR
word cuenta_DIRECCION = 0;			//Cuenta para leer direccion
byte FLAG_TIEMPO[4];				//FLAG de VELOCIDAD LEIDA
byte FLAG_RECEPTOR[2];				//FLAG de ANCHO DE PULSO LEIDO REMOTO
byte FLAG_ADC;						//FLAG de ADC TERMINO
byte FLAG_RX;						//Hay datos para procesar RECIBIDOS
byte FLAG_TX;						//Hay datos para procesar ENVIAR

// DIRECCION PWM
byte pwm_direccion;					//Contador para el PWM Manual
word pwm_pasos;						//Cantidad de PASOS que se ha dado

// REMOTO
word perdida_senal_remoto[2];		//Contador para detectar perdida de señal en modo REMOTO o CALIBRACION
word remoto_cero[2];				//Ancho del pulso en CALIBRACION



// ######################## VARIABLES INTERNAS ###############################

bool lectura_nueva;				//Cuando se cambia de estado, sirve para ejecutar 
								//	una porcion de codigo por unica vez

word remoto_vel_cero;			//Ancho del pulso en CERO
word remoto_dir_cero;			//Ancho del pulso en CERO
word cuenta_remoto_vel;			//Cuenta las veces para definir ancho de CALIBRACION
word cuenta_remoto_dir;			//Cuenta las veces para definir ancho de CALIBRACION
word receptor_ms[2];			//Ancho del pulso del receptor REMOTO 		[VELOCIDAD POSICION]
bool sentido_ant;				//Sentido anterior de GIRO MOTOR 			1-REVERSA
bool sentido_act;				//Sentido actual que se desea GIRAR MOTOR 	1-REVERSA
byte sentido_entrada;			//Sentido de GIRO MOTOR desde la PC 		1-REVERSA
word rpm_entrada;				//Lectura SETPOINT RPM desde la PC
word duty_entrada[4];			//Lectura DUTY en ESTADO LA_VELOCIDAD

word adc[4];					//Valores de los ADC de cada MOTOR
word *adc_value[4];				//Puntero de los valores de los ADC

word encoder_ms[4];				//Velocidad en ms del encoder 				[4 MOTORES]
word RPM_SET;					//SETPOINT DE RPM GLOBAL					VER SI HACER 4 SETPOINT?
signed long error_RPM[4];		//Entrada al PID [SETPOINT - LECTURA]
word rpm[4];					//Lectura RPM

word Var_Control[4];			//Salida del PID
word *Control[4];				//Puntero de las salidas del PID
word Var_Tension[4];			//Salida del PID para la FUNCION Tension_PWM
word duty[4];					//Duty aplicado a los PWM de cada MOTOR

word pasos_direccion;			//Pasos que se desean - PASO A PASO
word rpm_max_control;			//Setea el MAXIMO de RPM que se puede aplicar en MODO REMOTO LC
word tension_max_control = U_MAX;			//Setea el MAXIMO de DUTY que se puede aplicar en MODO REMOTO LA

byte lectura_direccion;			//Lee la posicion del PASO a PASO
word direccion_set = 196;				//SETPOINT de direcion

byte FLAG_PASOS;				//FLAG usada en el ESTADO PASOS
byte FLAG_DIRECCION = false;	//FLAG usado para definir cuando hay una señal de direccion

//######################NUEVO!

MOTOR motor_di;
MOTOR motor_dd;
MOTOR motor_ti;
MOTOR motor_td;

REMOTO direccion;
REMOTO velocidad;

PAP pap;

/*lint -save  -e970 Disable MISRA rule (6.3) checking. */

int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */

{
	byte i;
	byte x;
	byte aux;

  /* Write your local variable definition here */

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  PE_low_level_init();
  /*** End of Processor Expert internal initialization.                    ***/

  /* Write your code here */
  
  //################ INICIALIZAR VARIABLES
  ResetVar();
  //PID
  CtrlPID_DD_Set_K((float)K_PID);
  CtrlPID_DD_Set_Ti((float)TI_PID);
  CtrlPID_DI_Set_K((float)K_PID);
  CtrlPID_DI_Set_Ti((float)TI_PID);
  CtrlPID_TD_Set_K((float)K_PID);
  CtrlPID_TD_Set_Ti((float)TI_PID);
  CtrlPID_TI_Set_K((float)K_PID);
  CtrlPID_TI_Set_Ti((float)TI_PID);
  //
  //#######################################
  ADC_I_Calibrate(TRUE);
  ADC_I_Measure(FALSE);
  DIRECCION_ON;
  ESTADO = CALIBRACION;
  rpm_max_control = 40;
  lectura_nueva = true;
  pap.FLAG_DIRECCION = true;
  for(;;){
	  Get_Velocidad();					//LEER DATOS VELOCIDAD
	  Get_Corriente();					//LEER DATOS CORRIENTE
	  Get_Direccion();					//LEER DIRECCION
	  switch (ESTADO){
	  
	  case LA_VELOCIDAD:
		  if (lectura_nueva){
			  lectura_nueva = false;
			  motor_di.duty_entrada = (motor_di.duty_entrada >= MAX16BIT) ? MAX16BIT : motor_di.duty_entrada;
			  motor_di.duty_entrada = (motor_di.duty_entrada <= MIN16BIT) ? MIN16BIT : motor_di.duty_entrada;
		  }
		  break;
		  
	  case LC_REMOTO:
		  if (lectura_nueva){
			  Reset_PIDs();
			  lectura_nueva = false;
		  }
		  Get_Remoto();
		  
		  //LEER DIRECCION
		  direccion.ms = (direccion.ms >= direccion.remoto_cero + REMOTO_ANCHO_PULSO) ? direccion.remoto_cero + REMOTO_ANCHO_PULSO : direccion.ms;
		  direccion.ms = (direccion.ms <= direccion.remoto_cero - REMOTO_ANCHO_PULSO) ? direccion.remoto_cero - REMOTO_ANCHO_PULSO : direccion.ms;
		  pap.direccion_set = Mapeo(direccion.ms,direccion.remoto_cero - REMOTO_ANCHO_PULSO,direccion.remoto_cero + REMOTO_ANCHO_PULSO,LIMITE_DIRECCION_IZQUIERDO,LIMITE_DIRECCION_DERECHO);
		  //END LEER DIRECCION
		  
		  //LEER VELOCIDAD
		  velocidad.ms = (velocidad.ms >= velocidad.remoto_cero + REMOTO_ANCHO_PULSO) ? velocidad.remoto_cero + REMOTO_ANCHO_PULSO : velocidad.ms;
		  velocidad.ms = (velocidad.ms <= velocidad.remoto_cero - REMOTO_ANCHO_PULSO) ? velocidad.remoto_cero - REMOTO_ANCHO_PULSO : velocidad.ms;
			
		  //CAMBIO DE SENTIDO
		  if (velocidad.ms < (velocidad.remoto_cero - REMOTO_VENTANA)){//ES REVERSA?
			  sentido_act = REVERSA;
		  } else {
			  sentido_act = ADELANTE;
		  }
		  if (sentido_act != sentido_ant){
			  if (Vel_Cero()){
				  sentido_ant = sentido_act;
				  Out_Reversa_PutVal(sentido_act);
				  cuenta_PID = 0;
				  Reset_PIDs();
				  //AGREGADO JUEVES 26/10
				  motor_dd.tension = 0;
				  motor_di.tension = 0;
				  motor_td.tension = 0;
				  motor_ti.tension = 0;
				  motor_dd.control = 0;
				  motor_di.control = 0;
				  motor_td.control = 0;
				  motor_ti.control = 0;
				  //-
			  } else {
				  velocidad.ms = velocidad.remoto_cero; //PONE RPM_SET EN CERO
			  }
		  }
		  if (sentido_act == ADELANTE){
			  RPM_SET=Mapeo(velocidad.ms,velocidad.remoto_cero,velocidad.remoto_cero + REMOTO_ANCHO_PULSO,SET_RPM_MIN,rpm_max_control);
		  } else {
			  RPM_SET=Mapeo(velocidad.ms,velocidad.remoto_cero,velocidad.remoto_cero - REMOTO_ANCHO_PULSO,SET_RPM_MIN,rpm_max_control);
		  }
		  if (RPM_SET > rpm_max_control){
			  RPM_SET = rpm_max_control;
		  }
		  //END CAMBIO DE SENTIDO
		  
		  if (cuenta_PID >= 100){ //10 milisegundos periodo de muestreo
			  cuenta_PID = 0;
			  motor_di.RPM_set = RPM_SET;
			  motor_dd.RPM_set = RPM_SET;
			  motor_ti.RPM_set = RPM_SET;
			  motor_td.RPM_set = RPM_SET;
			  if (RPM_SET != 0){
				  Control_LC();
				  RPM_Cero();	//RPM_SET = 0?
				  motor_dd.tension = motor_dd.control;
				  motor_di.tension = motor_di.control;
				  motor_td.tension = motor_td.control;
				  motor_ti.tension = motor_ti.control;
			  } else {
				  motor_dd.tension = 0;
				  motor_di.tension = 0;
				  motor_td.tension = 0;
				  motor_ti.tension = 0;
				  motor_dd.control = 0;
				  motor_di.control = 0;
				  motor_td.control = 0;
				  motor_ti.control = 0;
				  Reset_PIDs();
			  }
		  }
		  //PERDIDA DE SEÑAL - SALE DEL ESTADO
		  if (velocidad.perdida_senal_remoto >= 500 || direccion.perdida_senal_remoto >= 500){
			  velocidad.perdida_senal_remoto = 0;
			  direccion.perdida_senal_remoto = 0;
			  ESTADO = PERDIDA_SENAL;
		  }
		  if (cuenta_ENVIAR >= 500){
			  cuenta_ENVIAR = 0;
		  }
		  break;
		  
	  case LA_REMOTO:
		  Get_Remoto();
		  
		  //LEER DIRECCION
		  direccion.ms = (direccion.ms >= direccion.remoto_cero + REMOTO_ANCHO_PULSO) ? direccion.remoto_cero + REMOTO_ANCHO_PULSO : direccion.ms;
		  direccion.ms = (direccion.ms <= direccion.remoto_cero - REMOTO_ANCHO_PULSO) ? direccion.remoto_cero - REMOTO_ANCHO_PULSO : direccion.ms;
		  pap.direccion_set = Mapeo(direccion.ms,direccion.remoto_cero - REMOTO_ANCHO_PULSO,direccion.remoto_cero + REMOTO_ANCHO_PULSO,LIMITE_DIRECCION_IZQUIERDO,LIMITE_DIRECCION_DERECHO);
		  //END LEER DIRECCION
		  
		  //LEER VELOCIDAD
		  velocidad.ms = (velocidad.ms >= velocidad.remoto_cero + REMOTO_ANCHO_PULSO) ? velocidad.remoto_cero + REMOTO_ANCHO_PULSO : velocidad.ms;
		  velocidad.ms = (velocidad.ms <= velocidad.remoto_cero - REMOTO_ANCHO_PULSO) ? velocidad.remoto_cero - REMOTO_ANCHO_PULSO : velocidad.ms;
		  		  
		  //CAMBIO DE SENTIDO
		  if (velocidad.ms < (velocidad.remoto_cero - REMOTO_VENTANA)){//ES REVERSA?
			  sentido_act = REVERSA;
		  } else {
			  sentido_act = ADELANTE;
		  }
		  if (sentido_act != sentido_ant){
			  if (Vel_Cero()){
				  sentido_ant = sentido_act;
				  Out_Reversa_PutVal(sentido_act);
			  } else {
				  velocidad.ms = velocidad.remoto_cero;
			  }
		  }
		  if (sentido_act == ADELANTE){
			  RPM_SET=Mapeo(velocidad.ms,velocidad.remoto_cero,velocidad.remoto_cero + REMOTO_ANCHO_PULSO,U_MIN,tension_max_control);
		  } else {
			  RPM_SET=Mapeo(velocidad.ms,velocidad.remoto_cero,velocidad.remoto_cero - REMOTO_ANCHO_PULSO,U_MIN,tension_max_control);
		  }
		  if (RPM_SET > tension_max_control){
			  RPM_SET = tension_max_control;
		  }
		  //END CAMBIO DE SENTIDO
		  motor_di.tension = RPM_SET;
		  motor_dd.tension = RPM_SET;
		  motor_ti.tension = RPM_SET;
		  motor_td.tension = RPM_SET;
		  //PERDIDA DE SEÑAL - SALE DEL ESTADO
		  if (velocidad.perdida_senal_remoto >= 500 || direccion.perdida_senal_remoto >= 500){
			  velocidad.perdida_senal_remoto = 0;
			  direccion.perdida_senal_remoto = 0;
			  ESTADO = PERDIDA_SENAL;
		  }
		  if (cuenta_ENVIAR >= 500){
			  cuenta_ENVIAR = 0;
		  }
		  break;  
		  
	  case LC_PC:
		  if (lectura_nueva){
			  Reset_PIDs();
			  lectura_nueva = false;
		  }
		  RPM_SET = rpm_entrada; //HACER
		  
		  if (RPM_SET <= SET_RPM_MIN){
			  RPM_SET = SET_RPM_MIN;
		  }
		  if (RPM_SET >= SET_RPM_MAX){
			  RPM_SET = SET_RPM_MAX;
		  }
		  if (sentido_entrada == REVERSA){//ES REVERSA PARA PC?
			  sentido_act = REVERSA;
		  } else {
			  sentido_act = ADELANTE;
		  }
		  if (sentido_act != sentido_ant){
			  if (Vel_Cero()){
				  sentido_ant = sentido_act;
				  Out_Reversa_PutVal(sentido_act);
			  } else {
				  RPM_SET = 0;
			  }
		  }
		  if (cuenta_PID >= 100){ //10 milisegundos periodo de muestreo
			  cuenta_PID = 0;
			  if (RPM_SET != 0){
				  Control_LC();
				  RPM_Cero();//RPM_SET = 0?
				  motor_dd.tension = motor_dd.control;
				  motor_di.tension = motor_di.control;
				  motor_td.tension = motor_td.control;
				  motor_ti.tension = motor_ti.control;
			  } else {
				  motor_dd.tension = 0;
				  motor_di.tension = 0;
				  motor_td.tension = 0;
				  motor_ti.tension = 0;				  
			  }
		  }		  
		  if (cuenta_ENVIAR >= 500){
			  cuenta_ENVIAR = 0;
		  }
		  break;
		  
	  case CALIBRACION:
		  Get_Remoto();
		  if (velocidad.ms != 0 || direccion.ms != 0 || !lectura_nueva){
			  lectura_nueva = false;
		  
			  if (velocidad.ms == velocidad.remoto_cero){
				  velocidad.cuenta_remoto++;			  
			  } else {
				  velocidad.remoto_cero = velocidad.ms;
				  velocidad.cuenta_remoto = 0;
			  }
			  if (direccion.ms == direccion.remoto_cero){
				  cuenta_remoto_dir++;
			  } else {
				  direccion.remoto_cero = direccion.ms;
				  cuenta_remoto_dir = 0;
			  }		  
			  if (cuenta_remoto_vel >= CUENTAS_REMOTO && cuenta_remoto_dir >= CUENTAS_REMOTO){
				  ESTADO = LC_REMOTO;
				  lectura_nueva = true;
			  }
			  if (velocidad.perdida_senal_remoto >= 500 || direccion.perdida_senal_remoto >= 500){
				  velocidad.perdida_senal_remoto = 0;
				  direccion.perdida_senal_remoto = 0;
				  ESTADO = PERDIDA_SENAL;
			  }
		  }
		  break;
		  
	  case PASOS:
		  if (FLAG_PASOS){
			  if (pasos_direccion != 0){
				  FLAG_DIRECCION_PWM_EN = true;
				  DIRECCION_ON;
				  if (pwm_pasos == pasos_direccion){
					  FLAG_DIRECCION_PWM_EN = false;
					  FLAG_PASOS = false;
					  pasos_direccion = 0;
					  pwm_direccion = 0;
					  pwm_pasos = 0;
					  DIRECCION_OFF;
				  }
			  }
		  }		  
		  break;
		  
	  case PERDIDA_SENAL:
		  ESTADO = CALIBRACION;
		  motor_di.tension = 0;
		  motor_dd.tension = 0;
		  motor_ti.tension = 0;
		  motor_td.tension = 0;
		  break;
		  
	  default:
		  break;
	  }
	  Tension_PWM();
	  
	  //PRUEBA CONTACTORES
	  if (FLAG_SW1){
		  FLAG_SW1 = 0;
	  }
	  
	  //####### DIRECCION
	  pap.FLAG_DIRECCION = true;
	  if (pap.FLAG_DIRECCION){
		  pap.FLAG_DIRECCION = false;
		  if (pap.FLAG_EN){
			  DIRECCION_ON;
		  } else {
			  DIRECCION_OFF;
		  }
		  pap.direccion_set = (pap.direccion_set >= LIMITE_DIRECCION_DERECHO) ? LIMITE_DIRECCION_DERECHO : pap.direccion_set;
		  pap.direccion_set = (pap.direccion_set <= LIMITE_DIRECCION_IZQUIERDO) ? LIMITE_DIRECCION_IZQUIERDO : pap.direccion_set;
		  if (pap.direccion_set > (pap.direccion_lectura + VENTANA_DIRECCION)){
			  pap.FLAG_DIRECCION = DERECHA;
		  }
		  if (pap.direccion_set < (pap.direccion_lectura - VENTANA_DIRECCION)){
			  pap.FLAG_DIRECCION = IZQUIERDA;
		  }
		  if (pap.FLAG_SENTIDO){
			  pap.FLAG_DIRECCION = true;
			  DIRECCION_HORARIA; 
		  } else {
			  pap.FLAG_DIRECCION = true;
			  DIRECCION_ANTI;
		  }
	  }
	  if ((pap.direccion_set <= (pap.direccion_lectura + VENTANA_DIRECCION)) && (pap.direccion_set >= (pap.direccion_lectura - VENTANA_DIRECCION))){
		  pap.FLAG_DIRECCION = false;
		  pap.pwm_direccion = 0;
		  //BitOut_DIR_PWM_ClrVal();
		  //DIRECCION_OFF;
	  }
	  //######## END DIRECCION
	  
	  RX();
	  TX();
	  if (cuenta_RECIBIR >= 500){
		  cuenta_RECIBIR = 0;
		  //FLAG_TX = true;		  
	  }
	  if (cnt_aux >= 1000){
		  cnt_aux = 0;
		  BitLed_Azul_NegVal();
		  Status_LED_NegVal();
	  }
	  /*		  
	  //####### ENVIAR CARACTERES
	  if (tx_sent!=tx_next && FLAG_TX){
		  UART_SDA_SendChar(tx_buf[tx_sent]);
		  inc(tx_sent);
		  if (tx_sent==tx_next){
			  UART_SDA_SendChar('\n');
			  FLAG_TX = false;
		  }
	  }
	  //####### END ENVIAR CARACTERES
	  */  
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

void Get_Remoto(void){
	if (velocidad.FLAG_TIEMPO){
		velocidad.FLAG_TIEMPO = 0;
		velocidad.ms = GET_RECEP(velocidad.Input.periodo);
		velocidad.perdida_senal_remoto = 0;
	}
	if (direccion.FLAG_TIEMPO){
		direccion.FLAG_TIEMPO = 0;
		direccion.ms = GET_RECEP(direccion.Input.periodo);
		direccion.perdida_senal_remoto = 0;
	}
}
void Get_Velocidad(void){
	if (motor_di.FLAG_TIEMPO){
		motor_di.FLAG_TIEMPO = false;
		motor_di.ms = GET_VEL(motor_di.Input.periodo);
		motor_di.rpm=60000/(motor_di.ms/10*24);
	}
	if (motor_dd.FLAG_TIEMPO){
		motor_dd.FLAG_TIEMPO = false;
		motor_dd.ms = GET_VEL(motor_dd.Input.periodo);
		motor_dd.rpm=60000/(motor_dd.ms/10*24);
	}
	if (motor_ti.FLAG_TIEMPO){
		motor_ti.FLAG_TIEMPO = false;
		motor_ti.ms = GET_VEL(motor_ti.Input.periodo);
		motor_ti.rpm=60000/(motor_ti.ms/10*24);
	}
	if (motor_td.FLAG_TIEMPO){
		motor_td.FLAG_TIEMPO = false;
		motor_td.ms = GET_VEL(motor_td.Input.periodo);
		motor_td.rpm=60000/(motor_td.ms/10*24);
	}	
	if (motor_di.cuenta_vel_cero >= RESET_VELOCIDAD_MS){	//un pulso, se pone vel en cero										
		motor_di.FLAG_TIEMPO = false;
		motor_di.ms = 0;
		motor_di.rpm = 0;
		motor_di.cuenta_vel_cero = 0;
		motor_di.Input.indices = 0;
	}
	if (motor_dd.cuenta_vel_cero >= RESET_VELOCIDAD_MS){	//un pulso, se pone vel en cero										
		motor_dd.FLAG_TIEMPO = false;
		motor_dd.ms = 0;
		motor_dd.rpm = 0;
		motor_dd.cuenta_vel_cero = 0;
		motor_dd.Input.indices = 0;
	}
	if (motor_ti.cuenta_vel_cero >= RESET_VELOCIDAD_MS){	//un pulso, se pone vel en cero										
		motor_ti.FLAG_TIEMPO = false;
		motor_ti.ms = 0;
		motor_ti.rpm = 0;
		motor_ti.cuenta_vel_cero = 0;
		motor_ti.Input.indices = 0;
	}
	if (motor_td.cuenta_vel_cero >= RESET_VELOCIDAD_MS){	//un pulso, se pone vel en cero										
		motor_td.FLAG_TIEMPO = false;
		motor_td.ms = 0;
		motor_td.rpm = 0;
		motor_td.cuenta_vel_cero = 0;
		motor_td.Input.indices = 0;
	}
}
void Get_Corriente(void){
	if (FLAG_ADC){
		FLAG_ADC = false;
		ADC_I_GetChanValue16(MOTOR_DI,&motor_di.adc);
		ADC_I_GetChanValue16(MOTOR_DD,&motor_dd.adc);
		ADC_I_GetChanValue16(MOTOR_TI,&motor_ti.adc);
		ADC_I_GetChanValue16(MOTOR_TD,&motor_td.adc);
		ADC_I_Measure(FALSE);
	}
}
void Get_Direccion(void){	
		byte lectura;
		byte z;
		if (cuenta_DIRECCION >= 10){
			cuenta_DIRECCION = cuenta_DIRECCION - 10;
		}
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
		pap.direccion_lectura = GrayToBin(lectura);		
		/*
		if ((direccion_set <= (lectura_direccion + 1)) && (direccion_set >= (lectura_direccion - 1))){
			lectura_direccion = lectura_direccion;
		}
		*/
		  
}
void RPM_Cero(void){
	byte i;
	if (motor_di.rpm == 0 && motor_di.RPM_set == 0){ //1 segundo
		motor_di.control = 0;
	}
	if (motor_dd.rpm == 0 && motor_dd.RPM_set == 0){ //1 segundo
		motor_dd.control = 0;
	}
	if (motor_ti.rpm == 0 && motor_ti.RPM_set == 0){ //1 segundo
		motor_ti.control = 0;
	}
	if (motor_td.rpm == 0 && motor_td.RPM_set == 0){ //1 segundo
		motor_td.control = 0;
	}
}
void Send_Velocidad(void){
	
}
void Send_RPM(void){
	
}
void Send_Corriente(void){
	
}
void Send_Calibrado(void){
	
}
void Control_LC(void){
	motor_di.error_RPM = (motor_di.RPM_set - motor_di.rpm);
	motor_dd.error_RPM = (motor_di.RPM_set - motor_dd.rpm);
	motor_ti.error_RPM = (motor_di.RPM_set - motor_ti.rpm);
	motor_td.error_RPM = (motor_di.RPM_set - motor_td.rpm);
	
	if (RPM_SET < 10){
	  CtrlPID_DI_Set_K((float)(K_PID/2));
	  CtrlPID_DD_Set_K((float)(K_PID/2));
	  CtrlPID_TI_Set_K((float)(K_PID/2));
	  CtrlPID_TD_Set_K((float)(K_PID/2));
	  CtrlPID_DI_Control(motor_di.error_RPM,&motor_di.control);
	  CtrlPID_DD_Control(motor_dd.error_RPM,&motor_dd.control);
	  CtrlPID_TI_Control(motor_ti.error_RPM,&motor_ti.control);
	  CtrlPID_TD_Control(motor_td.error_RPM,&motor_td.control);
	} else {
	  CtrlPID_DI_Set_K((float)(K_PID));
	  CtrlPID_DD_Set_K((float)(K_PID));
	  CtrlPID_TI_Set_K((float)(K_PID));
	  CtrlPID_TD_Set_K((float)(K_PID));
	  CtrlPID_DI_Control(motor_di.error_RPM,&motor_di.control);
	  CtrlPID_DD_Control(motor_dd.error_RPM,&motor_dd.control);
	  CtrlPID_TI_Control(motor_ti.error_RPM,&motor_ti.control);
	  CtrlPID_TD_Control(motor_td.error_RPM,&motor_td.control);
	}
	//TI = 0.5352609
}
bool Vel_Cero(void){
	if ((motor_dd.ms + motor_di.ms +
		  motor_td.ms + motor_ti.ms)>=1){
		return FALSE;
	}
	return TRUE;
}
void Tension_PWM(void){
	//%%%%%%%%%%%%%%%%%%%% VER QUE LA VARIABLE TENSION NO SEA NEGATIVA! %%%%%%%%%%%%%%%%%%%%%%%%%%%%!!!!!!!
	motor_di.tension = (motor_di.tension >= U_MAX) ? U_MAX : motor_di.tension;
	motor_dd.tension = (motor_dd.tension >= U_MAX) ? U_MAX : motor_dd.tension;
	motor_ti.tension = (motor_ti.tension >= U_MAX) ? U_MAX : motor_ti.tension;
	motor_td.tension = (motor_td.tension >= U_MAX) ? U_MAX : motor_td.tension;
	
	motor_di.duty = (motor_di.tension != 0) ? Mapeo(motor_di.tension,U_MIN,U_MAX,DUTY_MIN,DUTY_MAX) : 65535;
	motor_dd.duty = (motor_dd.tension != 0) ? Mapeo(motor_dd.tension,U_MIN,U_MAX,DUTY_MIN,DUTY_MAX) : 65535;
	motor_td.duty = (motor_td.tension != 0) ? Mapeo(motor_td.tension,U_MIN,U_MAX,DUTY_MIN,DUTY_MAX) : 65535;
	motor_ti.duty = (motor_ti.tension != 0) ? Mapeo(motor_ti.tension,U_MIN,U_MAX,DUTY_MIN,DUTY_MAX) : 65535;
	
	Out_PWM_DD_SetRatio16(motor_dd.duty);
	Out_PWM_DI_SetRatio16(motor_di.duty);
	Out_PWM_TI_SetRatio16(motor_ti.duty);
	Out_PWM_TD_SetRatio16(motor_td.duty);
}
void Reset_PIDs(void){
	  CtrlPID_DD_Reset();
	  CtrlPID_DI_Reset();
	  CtrlPID_TD_Reset();
	  CtrlPID_TI_Reset();
}
void RX(void){
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
		FLAG_RX = false;
		
	}	
}
void DECODIFICADO(byte* codigo, byte inicio){
	/*
	 * byte*	codigo	=	buffer de informacion
	 * byte		inicio	=	posicion del primer byte del mensaje dentro del buffer "codigo"
	 */
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
					rpm_entrada = lc_pc_rpm;
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
}
void TX(void){
	word *SND;
	word snd;
	SND = &snd;
	snd = 0;
	if (FLAG_TX){
		FLAG_TX = false;
		UART_MODBUS_SendBlock(tx_buf,(word)(tx_next - tx_sent),SND);
		tx_next = 0;
		tx_sent = 0;
		
		/* ENVIAR ENTERO MAPEADO
		TEXT_Num16uToStr(tx_buf,sizeof(tx_buf),(byte)Mapeo(lectura_direccion,0,255,0,359));
		TEXT_chcat(tx_buf,sizeof(tx_buf),(byte)'\n');
		UART_MODBUS_SendBlock(tx_buf,TEXT_strlen((const char *)tx_buf),SND);
		*/
	}
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
	byte i;
	byte x;
	//#### Variables
	for (i=0;i<=1;i++){
	  receptor_ms[i] = 0;
	}
	for (i=0;i<=3;i++){
	  encoder_ms[i] = 0;
	  adc[i]=0;
	  adc_value[i] = &adc[i];
	  error_RPM[i]=0;
	  rpm[i]=0;
	  Var_Control[i] = 0;
	  Control[i] = &Var_Control[i];
	  duty[i] = 65534;
	  Var_Tension[i] = 0;
	  duty_entrada[i] = 65535;
	}
	//REMOTO
	velocidad.ms = 0;
	direccion.ms = 0;
	//
	lectura_nueva = false;
	rpm_max_control = 0;
	sentido_entrada = ADELANTE;
	sentido_ant = ADELANTE;
	sentido_act = ADELANTE;
	rpm_entrada = 0;
	pasos_direccion = 0;
	Out_Reversa_PutVal(sentido_act);
	ESTADO = 25;
	
	FLAG_RX = 0;
	FLAG_TX = 0;
	FLAG_PASOS = 0;
	RPM_SET = 0;
	lectura_direccion = 0;
	cnt_aux = 0;
	cuenta_RX = 0;
	pwm_direccion = 0;
	pwm_pasos = 0;
	
	for (i=0;i<=1;i++){
		//##### InputCapture
		for (x=0;x<=1;x++){
			Data_Receptor[i][x] = 0;
		}
		Period_Receptor[i] = 0;
		index_Receptor[i] = 0;
		err_Receptor[i] = 0;
		FLAG_RECEPTOR[i]= 0;
	}
	for (i=0;i<=3;i++){
		//##### InputCapture
		for (x=0;x<=1;x++){
			Data[i][x] = 0;
		}
		Period[i] = 0;
		index[i] = 0;
		err[i] = 0;
		FLAG_TIEMPO[i] = 0;
		cuenta_vel_cero[i] = 0;
		posicion_pulsos[i] = 0;
	}
	//##### FLAGS
	FLAG_DIRECCION_PWM_EN = 0;
	FLAG_DIRECCION_SENTIDO = 0;
	FLAG_ADC = 0;
	FLAG_SW1 = false;
	FLAG_SW2 = false;
}
