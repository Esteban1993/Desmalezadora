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
#include "Btn_Emergencia.h"
#include "ExtIntLdd2.h"
#include "Encoder_DD.h"
#include "BitIoLdd16.h"
#include "Encoder_DI.h"
#include "BitIoLdd17.h"
#include "Encoder_TD.h"
#include "BitIoLdd18.h"
#include "Encoder_TI.h"
#include "BitIoLdd19.h"
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


/* User includes (#include below this line is not maintained by Processor Expert) */
/*
 * ######################## VARIABLES GLOBALES ###############################
 */

//ENVIAR Y RECIBIR
byte tx_buf[BUF_SIZE];            	// TX buffer
byte tx_next;						// TX indice siguiente a GUARDAR						
byte tx_sent;					  	// TX indice siguiente a MANDAR
byte rx_buf[BUF_SIZE];            	// RX buffer
byte rx_next;						// RX indice siguiente a GUARDAR
byte rx_read;  						// RX indice siguiente a LEER

// FLAGS
byte ESTADO;						//Indica el ESTADO del PROGRAMA
byte FLAG_SW1;						//FLAG DEL PULSADOR 1
byte FLAG_SW2;						//FLAG DEL PULSADOR 2
word cuenta_EMERGENCIA;
word cnt_aux;						//Cuenta AUXILIAR en interrupcion INTTIEMPO
word cuenta_PID;					//Cuenta de tiempo PID
word cuenta_RECIBIR;				//Cuenta de tiempo RECIBIR
word cuenta_ENVIAR;					//Cuenta de tiempo ENVIAR
word cuenta_TX;						//Cuenta de tiempo RECIBIR
word cuenta_DIRECCION = 0;			//Cuenta para leer direccion
byte FLAG_TIEMPO[4];				//FLAG de VELOCIDAD LEIDA
byte FLAG_RECEPTOR[2];				//FLAG de ANCHO DE PULSO LEIDO REMOTO
byte FLAG_ADC;						//FLAG de ADC TERMINO
byte FLAG_RX;						//Hay datos para procesar RECIBIDOS
byte FLAG_TX;						//Hay datos para procesar ENVIAR

// DIRECCION PWM
byte pwm_direccion;					//Contador para el PWM Manual
word pwm_pasos;						//Cantidad de PASOS que se ha dado


// ######################## VARIABLES INTERNAS ###############################

bool lectura_nueva;				//Cuando se cambia de estado, sirve para ejecutar 
								//	una porcion de codigo por unica vez

word RPM_SET;					//Setpoint RPM Global
bool sentido_ant;				//Sentido anterior de GIRO MOTOR 			1-REVERSA
bool sentido_act;				//Sentido actual que se desea GIRAR MOTOR 	1-REVERSA
byte sentido_entrada;			//Sentido de GIRO MOTOR desde la PC 		1-REVERSA

word rpm_max_control;			//Setea el MAXIMO de RPM que se puede aplicar en MODO REMOTO LC
word tension_max_control = U_MAX;	//Setea el MAXIMO de DUTY que se puede aplicar en MODO REMOTO LA

byte FLAG_PASOS;				//FLAG usada en el ESTADO PASOS
byte FLAG_DIRECCION = false;	//FLAG usado para definir cuando hay una señal de direccion

word tension_global = 0;

//######################NUEVO!

MOTOR motor_di = {MOTOR_DI, {MOTOR_DI, {0, 0}, 0, 0, 0, RISING, 0, 0, 0}, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, K_PID, 0, TI_PID, 0, DUTY_CERO, 0};
MOTOR motor_dd = {MOTOR_DD, {MOTOR_DD, {0, 0}, 0, 0, 0, RISING, 0, 0, 0}, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, K_PID, 0, TI_PID, 0, DUTY_CERO, 0};
MOTOR motor_ti = {MOTOR_TI, {MOTOR_TI, {0, 0}, 0, 0, 0, RISING, 0, 0, 0}, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, K_PID, 0, TI_PID, 0, DUTY_CERO, 0};
MOTOR motor_td = {MOTOR_TD, {MOTOR_TD, {0, 0}, 0, 0, 0, RISING, 0, 0, 0}, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, K_PID, 0, TI_PID, 0, DUTY_CERO, 0};

MOTOR_TX tx_dd;

REMOTO direccion;
REMOTO velocidad;

PAP pap = {0, 0, 0, 0, DIRECCION_CENTRO, false, false, false, false};

PC pc;

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
  
  CtrlPID_DD_Set_K((float)motor_dd.k);
  CtrlPID_DD_Set_Ti((float)motor_dd.ti/10);
  CtrlPID_DI_Set_K((float)motor_di.k);
  CtrlPID_DI_Set_Ti((float)motor_di.ti/10);
  CtrlPID_TD_Set_K((float)motor_td.k);
  CtrlPID_TD_Set_Ti((float)motor_td.ti/10);
  CtrlPID_TI_Set_K((float)motor_ti.k);
  CtrlPID_TI_Set_Ti((float)motor_ti.ti/10);
  //
  //#######################################
  ADC_I_Calibrate(TRUE);
  ADC_I_Measure(FALSE);
  DIRECCION_ON;
  ESTADO = CALIBRACION;
  rpm_max_control = 40;
  lectura_nueva = true;
  pap.FLAG_HABILITADO = false;
  pap.FLAG_EN = true;
  //TPulsos_SelectCaptureEdge(DeviceDataPtr, motor_dd.Input.nro, EDGE_RISING); //DSDASDASDASDASDASDASD
  for(;;){
	  while(true){
		  GetEncoder(&motor_dd);
		  GetVelocidad(&motor_dd);		  
		  Tension2Duty(&motor_dd);
		  SetDuty(motor_dd);
		  //####### DIRECCION
		  Get_Direccion(&pap);					//LEER DIRECCION
		  pap.FLAG_HABILITADO = true;
		  if (pap.FLAG_HABILITADO){
			  pap.FLAG_HABILITADO = false;
			  if (pap.FLAG_EN){
				  DIRECCION_ON;
			  } else {
				  DIRECCION_OFF;
			  }
			  pap.direccion_set = (pap.direccion_set >= LIMITE_DIRECCION_DERECHO) ? LIMITE_DIRECCION_DERECHO : pap.direccion_set;
			  pap.direccion_set = (pap.direccion_set <= LIMITE_DIRECCION_IZQUIERDO) ? LIMITE_DIRECCION_IZQUIERDO : pap.direccion_set;
			  if (pap.direccion_set > (pap.direccion_lectura + VENTANA_DIRECCION)){
				  pap.FLAG_SENTIDO = DERECHA;
				  pap.FLAG_DIRECCION = true;
			  }
			  if (pap.direccion_set < (pap.direccion_lectura - VENTANA_DIRECCION)){
				  pap.FLAG_SENTIDO = IZQUIERDA;
				  pap.FLAG_DIRECCION = true;
			  }
			  if (pap.FLAG_SENTIDO){
				  DIRECCION_HORARIA; 
			  } else {
				  DIRECCION_ANTI;
			  }
		  }
		  if ((pap.direccion_set <= (pap.direccion_lectura + VENTANA_DIRECCION)) && (pap.direccion_set >= (pap.direccion_lectura - VENTANA_DIRECCION))){
			  //pap.FLAG_EN = false;
			  pap.FLAG_DIRECCION = false;
			  pap.pwm_direccion = 0;
			  //BitOut_DIR_PWM_ClrVal();
			  //DIRECCION_OFF;
		  }
		  //######## END DIRECCION
		  
		  if(cuenta_TX >= 100){
			  TEXT_strcpy(serie.tx_buf,sizeof(serie.tx_buf),(unsigned char*)"Mdd-");
			  TEXT_strcatNum16u(serie.tx_buf,sizeof(serie.tx_buf),(word)motor_dd.ms);
			  TEXT_chcat(serie.tx_buf,sizeof(serie.tx_buf),'\r');
			  TEXT_chcat(serie.tx_buf,sizeof(serie.tx_buf),'\n');
			  NumeroFin(&serie);		  
			  cuenta_TX -= 100;
			  serie.FLAG_TX = true;
		  }
		  if (cnt_aux >= 500){
			  cnt_aux -= 100;
			  BitLed_Azul_NegVal();
			  Status_LED_NegVal();
		  }
		  TXS(&serie);
	  }
	  while(false){
	  motor_di.tension = tension_global;
	  motor_dd.tension = tension_global;
	  motor_td.tension = tension_global;
	  motor_ti.tension = tension_global;
	  Tension2Duty(&motor_di);
	  Tension2Duty(&motor_dd);
	  Tension2Duty(&motor_td);
	  Tension2Duty(&motor_ti);
	  SetDuty(motor_di);
	  SetDuty(motor_dd);
	  SetDuty(motor_td);
	  SetDuty(motor_ti);
	  Get_Direccion(&pap);					//LEER DIRECCION
	  
	  if(cuenta_TX >= 100){
		  TEXT_strcpy(serie.tx_buf,sizeof(serie.tx_buf),(unsigned char*)"Mdd-");
		  TEXT_strcatNum16u(serie.tx_buf,sizeof(serie.tx_buf),(word)motor_dd.ms);
		  /*	  
		  TEXT_strcat(serie.tx_buf,sizeof(serie.tx_buf),(unsigned char*)" Mdi-");
		  TEXT_strcatNum16u(serie.tx_buf,sizeof(serie.tx_buf),(word)motor_di.ms);
		  TEXT_strcat(serie.tx_buf,sizeof(serie.tx_buf),(unsigned char*)" Mtd-");
		  TEXT_strcatNum16u(serie.tx_buf,sizeof(serie.tx_buf),(word)motor_td.ms);
		  TEXT_strcat(serie.tx_buf,sizeof(serie.tx_buf),(unsigned char*)" Mti-");
		  TEXT_strcatNum16u(serie.tx_buf,sizeof(serie.tx_buf),(word)motor_ti.ms);
		  TEXT_strcat(serie.tx_buf,sizeof(serie.tx_buf),(unsigned char*)" ");
		  TEXT_strcatNum16u(serie.tx_buf,sizeof(serie.tx_buf),(word)cuenta_TX);
		  */
		  TEXT_chcat(serie.tx_buf,sizeof(serie.tx_buf),'\r');
		  TEXT_chcat(serie.tx_buf,sizeof(serie.tx_buf),'\n');
		  NumeroFin(&serie);
		  /*
		  tx_dd = TX_Motor(motor_dd);
		  Motor2Send(&serie,&tx_dd);
		  serie.tx_buf[serie.tx_next] = '\r';
		  inc(serie.tx_next);
		  serie.tx_buf[serie.tx_next] = '\n';
		  inc(serie.tx_next);
		  */			  
		  cuenta_TX -= 100;
		  serie.FLAG_TX = true;
	  }
	  GetVelocidad(&motor_di);
	  GetVelocidad(&motor_dd);
	  GetVelocidad(&motor_td);
	  GetVelocidad(&motor_ti);
	  Get_Corriente();
	  if (cnt_aux >= 100){
		  cnt_aux -= 100;
		  BitLed_Azul_NegVal();
		  Status_LED_NegVal();
	  }
	  TXS(&serie);
	  //####### DIRECCION
	  pap.FLAG_HABILITADO = true;
	  if (pap.FLAG_HABILITADO){
		  pap.FLAG_HABILITADO = false;
		  if (pap.FLAG_EN){
			  DIRECCION_ON;
		  } else {
			  DIRECCION_OFF;
		  }
		  pap.direccion_set = (pap.direccion_set >= LIMITE_DIRECCION_DERECHO) ? LIMITE_DIRECCION_DERECHO : pap.direccion_set;
		  pap.direccion_set = (pap.direccion_set <= LIMITE_DIRECCION_IZQUIERDO) ? LIMITE_DIRECCION_IZQUIERDO : pap.direccion_set;
		  if (pap.direccion_set > (pap.direccion_lectura + VENTANA_DIRECCION)){
			  pap.FLAG_SENTIDO = DERECHA;
			  pap.FLAG_DIRECCION = true;
		  }
		  if (pap.direccion_set < (pap.direccion_lectura - VENTANA_DIRECCION)){
			  pap.FLAG_SENTIDO = IZQUIERDA;
			  pap.FLAG_DIRECCION = true;
		  }
		  if (pap.FLAG_SENTIDO){
			  DIRECCION_HORARIA; 
		  } else {
			  DIRECCION_ANTI;
		  }
	  }
	  if ((pap.direccion_set <= (pap.direccion_lectura + VENTANA_DIRECCION)) && (pap.direccion_set >= (pap.direccion_lectura - VENTANA_DIRECCION))){
		  //pap.FLAG_EN = false;
		  pap.FLAG_DIRECCION = false;
		  pap.pwm_direccion = 0;
		  //BitOut_DIR_PWM_ClrVal();
		  //DIRECCION_OFF;
	  }
	  //######## END DIRECCION
	}
	//FIN
	  GetVelocidad(&motor_di);
	  GetVelocidad(&motor_dd);
	  GetVelocidad(&motor_td);
	  GetVelocidad(&motor_ti);
	  Get_Corriente();
	  Get_Direccion(&pap);
	  
	  switch (ESTADO){	  
	  case LA_VELOCIDAD:
		  if (lectura_nueva){
			  lectura_nueva = false;
		  }
		  break;		  
	  case LC_REMOTO:
		  if (lectura_nueva){
			  Reset_PIDs(motor_di);
			  Reset_PIDs(motor_dd);
			  Reset_PIDs(motor_ti);
			  Reset_PIDs(motor_td);
			  lectura_nueva = false;
		  }
		  Get_Remoto(&velocidad);
		  Get_Remoto(&direccion);
		  
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
			  if (Vel_Cero(motor_di,motor_dd,motor_td,motor_ti)){
				  sentido_ant = sentido_act;
				  Out_Reversa_PutVal(sentido_act);
				  cuenta_PID = 0;
				  Reset_PIDs(motor_di);
				  Reset_PIDs(motor_dd);
				  Reset_PIDs(motor_ti);
				  Reset_PIDs(motor_td);
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
		  if (cuenta_PID >= MUESTREO_PID){ //100 milisegundos periodo de muestreo
			  cuenta_PID = 0;
			  motor_di.RPM_set = RPM_SET;
			  motor_dd.RPM_set = RPM_SET;
			  motor_ti.RPM_set = RPM_SET;
			  motor_td.RPM_set = RPM_SET;
			  if (RPM_SET != 0){
				  Error_PID(&motor_dd);
				  Error_PID(&motor_di);
				  Error_PID(&motor_td);
				  Error_PID(&motor_ti);
				  CtrlPID_DD_Set_K((float)(motor_dd.k));
				  CtrlPID_DI_Set_K((float)(motor_di.k));
				  CtrlPID_TD_Set_K((float)(motor_td.k));
				  CtrlPID_TI_Set_K((float)(motor_ti.k));
				  CtrlPID_DD_Control(motor_dd.error_RPM,&motor_dd.control);
				  CtrlPID_DI_Control(motor_di.error_RPM,&motor_di.control);
				  CtrlPID_TD_Control(motor_td.error_RPM,&motor_td.control);
				  CtrlPID_TI_Control(motor_ti.error_RPM,&motor_ti.control);
				  /*
				  CtrlPID_Control(&motor_dd);
				  CtrlPID_Control(&motor_di);
				  CtrlPID_Control(&motor_td);
				  CtrlPID_Control(&motor_ti);
				  */
				  RPM_Cero(&motor_dd);	//RPM_SET = 0?
				  RPM_Cero(&motor_di);
				  RPM_Cero(&motor_td);
				  RPM_Cero(&motor_ti);
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
				  Reset_PIDs(motor_di);
				  Reset_PIDs(motor_dd);
				  Reset_PIDs(motor_ti);
				  Reset_PIDs(motor_td);
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
		  Get_Remoto(&velocidad);
		  Get_Remoto(&direccion);
		  
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
			  if (Vel_Cero(motor_di,motor_dd,motor_td,motor_ti)){
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
			  Reset_PIDs(motor_di);
			  Reset_PIDs(motor_dd);
			  Reset_PIDs(motor_ti);
			  Reset_PIDs(motor_td);
			  lectura_nueva = false;
		  }
		  RPM_SET = pc.rpm_global; //HACER
		  
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
			  if (Vel_Cero(motor_di,motor_dd,motor_td,motor_ti)){
				  sentido_ant = sentido_act;
				  Out_Reversa_PutVal(sentido_act);
			  } else {
				  RPM_SET = 0;
			  }
		  }
		  if (cuenta_PID >= MUESTREO_PID){ //10 milisegundos periodo de muestreo
			  cuenta_PID = 0;
			  if (RPM_SET != 0){
				  Error_PID(&motor_di);
				  Error_PID(&motor_dd);
				  Error_PID(&motor_td);
				  Error_PID(&motor_ti);
				  CtrlPID_Control(&motor_dd);
				  CtrlPID_Control(&motor_di);
				  CtrlPID_Control(&motor_td);
				  CtrlPID_Control(&motor_ti);
				  RPM_Cero(&motor_dd);	//RPM_SET = 0?
				  RPM_Cero(&motor_di);
				  RPM_Cero(&motor_td);
				  RPM_Cero(&motor_ti);
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
		  Get_Remoto(&velocidad);
		  Get_Remoto(&direccion);
		  if (velocidad.ms != 0 || direccion.ms != 0 || !lectura_nueva){
			  lectura_nueva = false;
		  
			  if (velocidad.ms == velocidad.remoto_cero){
				  velocidad.cuenta_remoto++;			  
			  } else {
				  velocidad.remoto_cero = velocidad.ms;
				  velocidad.cuenta_remoto = 0;
			  }
			  if (direccion.ms == direccion.remoto_cero){
				  direccion.cuenta_remoto++;
			  } else {
				  direccion.remoto_cero = direccion.ms;
				  direccion.cuenta_remoto = 0;
			  }		  
			  if (velocidad.cuenta_remoto >= CUENTAS_REMOTO && direccion.cuenta_remoto >= CUENTAS_REMOTO){
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
			  if (pap.pasos_adar != 0){
				  pap.FLAG_DIRECCION = true;
				  DIRECCION_ON;
				  if (pap.pasos_dados == pap.pasos_adar){
					  pap.FLAG_DIRECCION = false;
					  FLAG_PASOS = false;
					  pap.pasos_adar = 0;
					  pap.pwm_direccion = 0;
					  pap.pasos_dados = 0;
					  DIRECCION_OFF;
				  }
			  }
		  }		  
		  break;
		  
	  case PERDIDA_SENAL:
		  ESTADO = CALIBRACION;
		  Reset_PIDs(motor_di);
		  Reset_PIDs(motor_dd);
		  Reset_PIDs(motor_ti);
		  Reset_PIDs(motor_td);
		  motor_di.tension = 0;
		  motor_dd.tension = 0;
		  motor_ti.tension = 0;
		  motor_td.tension = 0;
		  break;
		  
	  default:
		  break;
	  }
	  Tension2Duty(&motor_di);
	  Tension2Duty(&motor_dd);
	  Tension2Duty(&motor_td);
	  Tension2Duty(&motor_ti);
	  SetDuty(motor_di);
	  SetDuty(motor_dd);
	  SetDuty(motor_td);
	  SetDuty(motor_ti);
	  
	  //PRUEBA CONTACTORES
	  if (FLAG_SW1){
		  FLAG_SW1 = 0;
	  }
	  
	  //####### DIRECCION
	  pap.FLAG_HABILITADO = true;
	  if (pap.FLAG_HABILITADO){
		  pap.FLAG_HABILITADO = false;
		  if (pap.FLAG_EN){
			  DIRECCION_ON;
		  } else {
			  DIRECCION_OFF;
		  }
		  pap.direccion_set = (pap.direccion_set >= LIMITE_DIRECCION_DERECHO) ? LIMITE_DIRECCION_DERECHO : pap.direccion_set;
		  pap.direccion_set = (pap.direccion_set <= LIMITE_DIRECCION_IZQUIERDO) ? LIMITE_DIRECCION_IZQUIERDO : pap.direccion_set;
		  if (pap.direccion_set > (pap.direccion_lectura + VENTANA_DIRECCION)){
			  pap.FLAG_DIRECCION = true;
			  pap.FLAG_SENTIDO = DERECHA;
		  }
		  if (pap.direccion_set < (pap.direccion_lectura - VENTANA_DIRECCION)){
			  pap.FLAG_DIRECCION = true;
			  pap.FLAG_SENTIDO = IZQUIERDA;
		  }
		  if (pap.FLAG_SENTIDO){			  
			  DIRECCION_HORARIA; 
		  } else {			  
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
	  
	  //RX();
	  //RX(serie, &motor_di, &motor_dd, &motor_td, &motor_ti, &ESTADO);
	  
	  //TX();
	  if(cuenta_TX >= 100){
		  TEXT_strcpy(serie.tx_buf,sizeof(serie.tx_buf),(unsigned char*)"Mdd:");
		  TEXT_strcatNum16u(serie.tx_buf,sizeof(serie.tx_buf),(word)motor_dd.ms);
		  TEXT_strcat(serie.tx_buf,sizeof(serie.tx_buf),(unsigned char*)" Edd:");
		  TEXT_strcatNum32s(serie.tx_buf,sizeof(serie.tx_buf),motor_dd.error_RPM);
		  TEXT_strcat(serie.tx_buf,sizeof(serie.tx_buf),(unsigned char*)" ");
		  TEXT_strcatNum16u(serie.tx_buf,sizeof(serie.tx_buf),(word)cuenta_TX);
		  /*
		  TEXT_strcpy(serie.tx_buf,sizeof(serie.tx_buf),(unsigned char*)"Mdd-");
		  TEXT_strcatNum16u(serie.tx_buf,sizeof(serie.tx_buf),(word)motor_dd.ms);
		  TEXT_strcat(serie.tx_buf,sizeof(serie.tx_buf),(unsigned char*)" Mdi-");
		  TEXT_strcatNum16u(serie.tx_buf,sizeof(serie.tx_buf),(word)motor_di.ms);
		  TEXT_strcat(serie.tx_buf,sizeof(serie.tx_buf),(unsigned char*)" Mtd-");
		  TEXT_strcatNum16u(serie.tx_buf,sizeof(serie.tx_buf),(word)motor_td.ms);
		  TEXT_strcat(serie.tx_buf,sizeof(serie.tx_buf),(unsigned char*)" Mti-");
		  TEXT_strcatNum16u(serie.tx_buf,sizeof(serie.tx_buf),(word)motor_ti.ms);
		  TEXT_strcat(serie.tx_buf,sizeof(serie.tx_buf),(unsigned char*)" ");
		  TEXT_strcatNum16u(serie.tx_buf,sizeof(serie.tx_buf),(word)cuenta_TX);
		  */
		  TEXT_chcat(serie.tx_buf,sizeof(serie.tx_buf),'\r');
		  TEXT_chcat(serie.tx_buf,sizeof(serie.tx_buf),'\n');
		  NumeroFin(&serie);	  
		  cuenta_TX -= 100;
		  serie.FLAG_TX = true;
	  }
	  TXS(&serie);
	  
	  if (cuenta_RECIBIR >= 500){
		  cuenta_RECIBIR = 0;
		  //FLAG_TX = true;		  
	  }
	  if (cnt_aux >= 100 && ESTADO == CALIBRACION){
		  cnt_aux = 0;
		  BitLed_Azul_NegVal();
		  Status_LED_NegVal();
	  }
	  if (cnt_aux >= 500 && ESTADO != CALIBRACION){
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
void Get_Direccion(PAP *pap_x){	
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
		pap_x->direccion_lectura = GrayToBin(lectura);
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

void TXS(SERIE *serie_x){
	word *SND;
	word snd;
	SND = &snd;
	snd = 0;
	if (serie_x->FLAG_TX){
		serie_x->FLAG_TX = false;
		//NumeroFin(&serie);
		UART_MODBUS_SendBlock(serie_x->tx_buf,(word)(serie_x->tx_next - serie_x->tx_sent),SND);
		//UART_MODBUS_SendBlock(serie_x->tx_buf,TEXT_strlen((const char *)serie_x->tx_buf),SND);
		serie_x->tx_next = 0;
		serie_x->tx_sent = 0;
	}
}
