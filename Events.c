/* ###################################################################
**     Filename    : Events.c
**     Project     : Pre1Motor1.0
**     Processor   : MK64FN1M0VLL12
**     Component   : Events
**     Version     : Driver 01.00
**     Compiler    : CodeWarrior ARM C Compiler
**     Date/Time   : 2017-06-29, 16:06, # CodeGen: 0
**     Abstract    :
**         This is user's event module.
**         Put your event handler code here.
**     Settings    :
**     Contents    :
**         ADC_DD_OnEnd                 - void ADC_DD_OnEnd(void);
**         ADC_DD_OnCalibrationEnd      - void ADC_DD_OnCalibrationEnd(void);
**         Input_Encoder_DI_OnCapture            - void Input_Encoder_DI_OnCapture(void);
**         IntEncoder_DD_OnInterrupt - void IntEncoder_DD_OnInterrupt(void);
**         IntEncoder_DI_OnInterrupt - void IntEncoder_DI_OnInterrupt(void);
**         Btn_SW2_OnInterrupt        - void Btn_SW2_OnInterrupt(void);
**         Btn_SW1_OnInterrupt        - void Btn_SW1_OnInterrupt(void);
**         IntTiempo_OnInterrupt     - void IntTiempo_OnInterrupt(void);
**         Cpu_OnNMI                 - void Cpu_OnNMI(void);
**
** ###################################################################*/
/*!
** @file Events.c
** @version 01.00
** @brief
**         This is user's event module.
**         Put your event handler code here.
*/         
/*!
**  @addtogroup Events_module Events module documentation
**  @{
*/         
/* MODULE Events */

#include "Cpu.h"
#include "Events.h"
#include "Init_Config.h"
#include "PDD_Includes.h"
#include "Struct.h"
#include "Defines.h"

#ifdef __cplusplus
extern "C" {
#endif 

/*
 * ######################## VARIABLES GLOBALES ###############################
 */
// INPUT CAPTURE
extern uint16_t Data[4][2];					//Contador de pulsos de cada flanco [1xMOTOR 2xVariableAnteriorYActual] 
extern word Period[4];						//Diferencia entre los pulsos 2 y 1 de cada MOTOR
extern byte index[4];						//Indice de pulsos
extern byte err[4];							//Por si ocurre un error al momento de leer pulsos

extern word cuenta_vel_cero[4];				//Contador para determinar Vel CERO si no se registran nuevos flancos
extern dlong posicion_pulsos[4];			//Contador de Flancos del Motor

extern uint16_t Data_Receptor[2][2];		//IDEM A ANTERIOR SOLO QUE PARA MEDIR ANCHO DE PULSO DE REMOTO
extern dword Period_Receptor[2];				//IDEM A ANTERIOR SOLO QUE PARA MEDIR ANCHO DE PULSO DE REMOTO
extern byte index_Receptor[2];				//IDEM A ANTERIOR SOLO QUE PARA MEDIR ANCHO DE PULSO DE REMOTO
extern byte err_Receptor[2];				//IDEM A ANTERIOR SOLO QUE PARA MEDIR ANCHO DE PULSO DE REMOTO

//ENVIAR Y RECIBIR
extern byte tx_buf[BUF_SIZE];            	// TX buffer
extern byte tx_next;						// TX indice siguiente a GUARDAR						
extern byte tx_sent;					  	// TX indice siguiente a MANDAR
extern byte rx_buf[BUF_SIZE];            	// RX buffer
extern byte rx_next;						// RX indice siguiente a GUARDAR
extern byte rx_read;  						// RX indice siguiente a LEER

// FLAGS
extern byte ESTADO;							//Indica el ESTADO del PROGRAMA
extern byte FLAG_DIRECCION_PWM_EN;			//PASO A PASO ENEABLE
extern byte FLAG_DIRECCION_SENTIDO;			//PASO A PASO SENTIDO DE GIRO
extern byte FLAG_SW1;						//FLAG DEL PULSADOR 1
extern byte FLAG_SW2;						//FLAG DEL PULSADOR 2
extern word perdida_senal_remoto[2];		//Contador para detectar perdida de se�al en modo REMOTO o CALIBRACION
extern word cnt_aux;						//Cuenta AUXILIAR en interrupcion INTTIEMPO
extern word cuenta_PID;						//Cuenta de tiempo PID
extern word cuenta_RECIBIR;					//Cuenta de tiempo RECIBIR
extern word cuenta_ENVIAR;					//Cuenta de tiempo ENVIAR
extern word cuenta_RX;						//Cuenta de tiempo RECIBIR
extern word cuenta_DIRECCION;				//Cuenta para leer direccion
extern byte FLAG_TIEMPO[4];					//FLAG de VELOCIDAD LEIDA
extern byte FLAG_RECEPTOR[2];				//FLAG de ANCHO DE PULSO LEIDO REMOTO
extern byte FLAG_ADC;						//FLAG de ADC TERMINO
extern byte FLAG_RX;						//Hay datos para procesar RECIBIDOS
extern byte FLAG_TX;						//Hay datos para procesar ENVIAR

// DIRECCION PWM
extern byte pwm_direccion;					//Define el DUTY CICLE del PWM de la direccion
extern word pwm_pasos;						//Cantidad de PASOS que se ha dado

// NUEVO
extern MOTOR motor_di;
extern MOTOR motor_dd;
extern MOTOR motor_ti;
extern MOTOR motor_td;

extern REMOTO direccion;
extern REMOTO velocidad;

extern PAP pap;

/* User includes (#include below this line is not maintained by Processor Expert) */



/*
** ===================================================================
**     Event       :  Input_Encoder_DI_OnCapture (module Events)
**
**     Component   :  Input_Encoder_DI [Capture]
**     Description :
**         This event is called on capturing of Timer/Counter actual
**         value (only when the component is enabled - <Enable> and the
**         events are enabled - <EnableEvent>.This event is available
**         only if a <interrupt service/event> is enabled.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void Input_Encoder_DI_OnCapture(void)
{
	if (motor_di.Input.indices == 2){
		motor_di.Input.periodo = motor_di.Input.datos[1] - motor_di.Input.datos[0];
		motor_di.Input.indices = 0;
		motor_di.FLAG_TIEMPO = 1;
		motor_di.posicion_pulsos++;
		motor_di.cuenta_vel_cero = 0;
	}
	motor_di.Input.err = Input_Encoder_DI_GetCaptureValue(&motor_di.Input.datos[motor_di.Input.indices]);
	motor_di.Input.indices++;
  /* Write your code here ... */
}

/*
** ===================================================================
**     Event       :  Btn_SW2_OnInterrupt (module Events)
**
**     Component   :  Btn_SW2 [ExtInt]
**     Description :
**         This event is called when an active signal edge/level has
**         occurred.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void Btn_SW2_OnInterrupt(void)
{
  /* Write your code here ... */
}

/*
** ===================================================================
**     Event       :  Btn_SW1_OnInterrupt (module Events)
**
**     Component   :  Btn_SW1 [ExtInt]
**     Description :
**         This event is called when an active signal edge/level has
**         occurred.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void Btn_SW1_OnInterrupt(void)
{
	FLAG_SW1 = 1;
  /* Write your code here ... */
}

/*
** ===================================================================
**     Event       :  IntTiempo_OnInterrupt (module Events)
**
**     Component   :  IntTiempo [TimerInt]
**     Description :
**         When a timer interrupt occurs this event is called (only
**         when the component is enabled - <Enable> and the events are
**         enabled - <EnableEvent>). This event is enabled only if a
**         <interrupt service/event> is enabled.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void IntTiempo_OnInterrupt(void)
{
	byte i;
	
	cnt_aux++;
	cuenta_RECIBIR++;
	cuenta_ENVIAR++;
	cuenta_PID++;
	cuenta_RX++;
	cuenta_DIRECCION++;
	motor_di.cuenta_vel_cero += 1;
	motor_dd.cuenta_vel_cero += 1;
	motor_ti.cuenta_vel_cero += 1;
	motor_td.cuenta_vel_cero += 1;
	if (ESTADO == CALIBRACION || ESTADO == LC_REMOTO || ESTADO == LA_REMOTO){
		velocidad.perdida_senal_remoto++;
		direccion.perdida_senal_remoto++;
	}
  /* Write your code here ... */
}

/*
** ===================================================================
**     Event       :  Cpu_OnNMI (module Events)
**
**     Component   :  Cpu [MK64FN1M0LL12]
*/
/*!
**     @brief
**         This event is called when the Non maskable interrupt had
**         occurred. This event is automatically enabled when the [NMI
**         interrupt] property is set to 'Enabled'.
*/
/* ===================================================================*/
void Cpu_OnNMI(void)
{
  /* Write your code here ... */
}

/*
** ===================================================================
**     Event       :  Input_Encoder_DD_OnCapture (module Events)
**
**     Component   :  Input_Encoder_DD [Capture]
**     Description :
**         This event is called on capturing of Timer/Counter actual
**         value (only when the component is enabled - <Enable> and the
**         events are enabled - <EnableEvent>.This event is available
**         only if a <interrupt service/event> is enabled.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void Input_Encoder_DD_OnCapture(void)
{
  /* Write your code here ... */
	if (motor_dd.Input.indices == 2){
		motor_dd.Input.periodo = motor_dd.Input.datos[1] - motor_dd.Input.datos[0];
		motor_dd.Input.indices = 0;
		motor_dd.FLAG_TIEMPO = 1;
		motor_dd.posicion_pulsos++;
		motor_dd.cuenta_vel_cero = 0;
	}
	motor_dd.Input.err = Input_Encoder_DI_GetCaptureValue(&motor_dd.Input.datos[motor_dd.Input.indices]);
	motor_dd.Input.indices++;
}

/*
** ===================================================================
**     Event       :  Input_Encoder_TD_OnCapture (module Events)
**
**     Component   :  Input_Encoder_TD [Capture]
**     Description :
**         This event is called on capturing of Timer/Counter actual
**         value (only when the component is enabled - <Enable> and the
**         events are enabled - <EnableEvent>.This event is available
**         only if a <interrupt service/event> is enabled.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void Input_Encoder_TD_OnCapture(void)
{
  /* Write your code here ... */
	if (motor_td.Input.indices == 2){
		motor_td.Input.periodo = motor_td.Input.datos[1] - motor_td.Input.datos[0];
		motor_td.Input.indices = 0;
		motor_td.FLAG_TIEMPO = 1;
		motor_td.posicion_pulsos++;
		motor_td.cuenta_vel_cero = 0;
	}
	motor_td.Input.err = Input_Encoder_DI_GetCaptureValue(&motor_td.Input.datos[motor_td.Input.indices]);
	motor_td.Input.indices++;
}

/*
** ===================================================================
**     Event       :  Input_Encoder_TI_OnCapture (module Events)
**
**     Component   :  Input_Encoder_TI [Capture]
**     Description :
**         This event is called on capturing of Timer/Counter actual
**         value (only when the component is enabled - <Enable> and the
**         events are enabled - <EnableEvent>.This event is available
**         only if a <interrupt service/event> is enabled.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void Input_Encoder_TI_OnCapture(void)
{
  /* Write your code here ... */
	if (motor_ti.Input.indices == 2){
		motor_ti.Input.periodo = motor_ti.Input.datos[1] - motor_ti.Input.datos[0];
		motor_ti.Input.indices = 0;
		motor_ti.FLAG_TIEMPO = 1;
		motor_ti.posicion_pulsos++;
		motor_ti.cuenta_vel_cero = 0;
	}
	motor_ti.Input.err = Input_Encoder_DI_GetCaptureValue(&motor_ti.Input.datos[motor_ti.Input.indices]);
	motor_ti.Input.indices++;
}

/*
** ===================================================================
**     Event       :  ADC_I_OnEnd (module Events)
**
**     Component   :  ADC_I [ADC]
**     Description :
**         This event is called after the measurement (which consists
**         of <1 or more conversions>) is/are finished.
**         The event is available only when the <Interrupt
**         service/event> property is enabled.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void ADC_I_OnEnd(void)
{
	FLAG_ADC = 1;
  /* Write your code here ... */
}

/*
** ===================================================================
**     Event       :  ADC_I_OnCalibrationEnd (module Events)
**
**     Component   :  ADC_I [ADC]
**     Description :
**         This event is called when the calibration has been finished.
**         User should check if the calibration pass or fail by
**         Calibration status method./nThis event is enabled only if
**         the <Interrupt service/event> property is enabled.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void ADC_I_OnCalibrationEnd(void)
{
  /* Write your code here ... */
}

/*
** ===================================================================
**     Event       :  ReceptorVelocidad_OnCapture (module Events)
**
**     Component   :  ReceptorVelocidad [Capture]
**     Description :
**         This event is called on capturing of Timer/Counter actual
**         value (only when the component is enabled - <Enable> and the
**         events are enabled - <EnableEvent>.This event is available
**         only if a <interrupt service/event> is enabled.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void ReceptorVelocidad_OnCapture(void)
{
  /* Write your code here ... */	
	if (velocidad.Input.indices == 2){
		velocidad.Input.periodo = velocidad.Input.datos[1] - velocidad.Input.datos[0];
		velocidad.Input.indices = 0;
		velocidad.FLAG_TIEMPO = 1;
		if (velocidad.Input.periodo >= 7000){
			velocidad.Input.datos[0] = velocidad.Input.datos[1];
			velocidad.Input.indices = 1;
			velocidad.FLAG_TIEMPO = 0;
		}
	}
	velocidad.Input.err = ReceptorVelocidad_GetCaptureValue(&velocidad.Input.datos[velocidad.Input.indices]);
	velocidad.Input.indices++;
}

/*
** ===================================================================
**     Event       :  ReceptorDireccion_OnCapture (module Events)
**
**     Component   :  ReceptorDireccion [Capture]
**     Description :
**         This event is called on capturing of Timer/Counter actual
**         value (only when the component is enabled - <Enable> and the
**         events are enabled - <EnableEvent>.This event is available
**         only if a <interrupt service/event> is enabled.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void ReceptorDireccion_OnCapture(void)
{
  /* Write your code here ... */
	if (direccion.Input.indices == 2){
		direccion.Input.periodo = direccion.Input.datos[1] - direccion.Input.datos[0];
		direccion.Input.indices = 0;
		direccion.FLAG_TIEMPO = 1;
		if (direccion.Input.periodo >= 7000){
			direccion.Input.datos[0] = direccion.Input.datos[1];
			direccion.Input.indices = 1;
			direccion.FLAG_TIEMPO = 0;
		}
	}
	direccion.Input.err = ReceptorVelocidad_GetCaptureValue(&direccion.Input.datos[direccion.Input.indices]);
	direccion.Input.indices++;
}


/*
** ===================================================================
**     Event       :  IntDireccion_OnInterrupt (module Events)
**
**     Component   :  IntDireccion [TimerInt]
**     Description :
**         When a timer interrupt occurs this event is called (only
**         when the component is enabled - <Enable> and the events are
**         enabled - <EnableEvent>). This event is enabled only if a
**         <interrupt service/event> is enabled.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void IntDireccion_OnInterrupt(void)
{
	static byte step_direccion = FREQ_PWM_DUTY/2; //Define el tiempo en ALTO
	
	if (pap.FLAG_DIRECCION){						//Esta habilitado el PWM de direccion?	
		if (pap.pwm_direccion == 0){				
			BitOut_DIR_PWM_SetVal();				//Salida en ALTO
		}
		if (pap.pwm_direccion == step_direccion){	//Ya paso el tiempo en alto?		
			BitOut_DIR_PWM_ClrVal();				//Salida en BAJO
		}
		pap.pwm_direccion++;						//Incremento un step del PWM
		if (pap.pwm_direccion == FREQ_PWM_DUTY){	//Cuando se alcanza el STEP de la frecuencia, reseteo
			pap.pasos_dados++;						//Se realizo UN PASO
			pap.pwm_direccion = 0;					//Reseteo el step
		}
	} else {
		pap.pasos_dados = 0;
		BitOut_DIR_PWM_ClrVal();
	}
  /* Write your code here ... */
}

/*
** ===================================================================
**     Event       :  UART_MODBUS_OnError (module Events)
**
**     Component   :  UART_MODBUS [AsynchroSerial]
**     Description :
**         This event is called when a channel error (not the error
**         returned by a given method) occurs. The errors can be read
**         using <GetError> method.
**         The event is available only when the <Interrupt
**         service/event> property is enabled.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void UART_MODBUS_OnError(void)
{
  /* Write your code here ... */
}

/*
** ===================================================================
**     Event       :  UART_MODBUS_OnRxChar (module Events)
**
**     Component   :  UART_MODBUS [AsynchroSerial]
**     Description :
**         This event is called after a correct character is received.
**         The event is available only when the <Interrupt
**         service/event> property is enabled and either the <Receiver>
**         property is enabled or the <SCI output mode> property (if
**         supported) is set to Single-wire mode.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void UART_MODBUS_OnRxChar(void)
{
	UART_MODBUS_RecvChar(&rx_buf[rx_next]);
	if (rx_buf[rx_next] == '\r'){
		FLAG_RX = 1;
	}
	inc(rx_next);
}

/*
** ===================================================================
**     Event       :  UART_MODBUS_OnTxChar (module Events)
**
**     Component   :  UART_MODBUS [AsynchroSerial]
**     Description :
**         This event is called after a character is transmitted.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void UART_MODBUS_OnTxChar(void)
{
  /* Write your code here ... */
}

/*
** ===================================================================
**     Event       :  UART_MODBUS_OnFullRxBuf (module Events)
**
**     Component   :  UART_MODBUS [AsynchroSerial]
**     Description :
**         This event is called when the input buffer is full;
**         i.e. after reception of the last character 
**         that was successfully placed into input buffer.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void UART_MODBUS_OnFullRxBuf(void)
{
  /* Write your code here ... */
}

/*
** ===================================================================
**     Event       :  UART_MODBUS_OnFreeTxBuf (module Events)
**
**     Component   :  UART_MODBUS [AsynchroSerial]
**     Description :
**         This event is called after the last character in output
**         buffer is transmitted.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void UART_MODBUS_OnFreeTxBuf(void)
{
  /* Write your code here ... */
}

/*
** ===================================================================
**     Event       :  UART_MODBUS_OnTxComplete (module Events)
**
**     Component   :  UART_MODBUS [AsynchroSerial]
**     Description :
**         This event indicates that the transmitter is finished
**         transmitting all data, preamble, and break characters and is
**         idle. It can be used to determine when it is safe to switch
**         a line driver (e.g. in RS-485 applications).
**         The event is available only when both <Interrupt
**         service/event> and <Transmitter> properties are enabled.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void UART_MODBUS_OnTxComplete(void)
{
  /* Write your code here ... */
}

/* END Events */

#ifdef __cplusplus
}  /* extern "C" */
#endif 

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
