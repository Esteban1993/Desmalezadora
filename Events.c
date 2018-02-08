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
**         Input_Hall_DD_OnCapture            - void Input_Hall_DD_OnCapture(void);
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
#include "Struct2.h"
#include "Defines2.h"

#ifdef __cplusplus
extern "C" {
#endif 

/*
 * ######################## VARIABLES GLOBALES ###############################
 */

// FLAGS
extern byte ESTADO;							//Indica el ESTADO del PROGRAMA
extern byte FLAG_DIRECCION_PWM_EN;			//PASO A PASO ENEABLE
extern byte FLAG_DIRECCION_SENTIDO;			//PASO A PASO SENTIDO DE GIRO
extern byte FLAG_SW1;						//FLAG DEL PULSADOR 1
extern word perdida_senal_remoto[2];		//Contador para detectar perdida de señal en modo REMOTO o CALIBRACION
extern uint16 cuenta_x2;					//Cuenta x2
extern word cnt_aux;						//Cuenta AUXILIAR en interrupcion INTTIEMPO
extern word cuenta_PID;						//Cuenta de tiempo PID
extern word cuenta_RX;						//Cuenta de tiempo RECIBIR
extern word cuenta_TX;						//Cuenta de tiempo RECIBIR
extern word cuenta_DIRECCION;				//Cuenta para leer direccion
extern byte FLAG_ADC;						//FLAG de ADC TERMINO

extern uint16 emergencias;					//Cuenta cantidad de llamadas a Emergencia
extern uint16 RPM_SET;						//Setea RPM Global

extern uint16 WATCHDOG;						//FLAG DEL WATCHDOG


// NUEVO
extern MOTOR motor_di;
extern MOTOR motor_dd;
extern MOTOR motor_ti;
extern MOTOR motor_td;

extern REMOTO direccion;
extern REMOTO velocidad;

extern PAP pap;

extern SERIE serie;

/* User includes (#include below this line is not maintained by Processor Expert) */



/*
** ===================================================================
**     Event       :  Input_Hall_DD_OnCapture (module Events)
**
**     Component   :  Input_Hall_DD [Capture]
**     Description :
**         This event is called on capturing of Timer/Counter actual
**         value (only when the component is enabled - <Enable> and the
**         events are enabled - <EnableEvent>.This event is available
**         only if a <interrupt service/event> is enabled.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void Input_Hall_DD_OnCapture(void)
{
	if(!motor_dd.Input.FLAG_E){							//ESPERO PROCESAR VALOR
		motor_dd.Input.err = Input_Hall_DD_GetCaptureValue(&motor_dd.Input.aux);
		motor_dd.Input.FLAG_E = TRUE;
		motor_dd.Input.tiempo = 0;
	}
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
	WATCHDOG = WATCHDOG | WD_TIMER;
	cnt_aux++;
	cuenta_PID++;
	cuenta_TX++;
	cuenta_RX++;
	cuenta_DIRECCION++;
	cuenta_x2 = (cuenta_x2 <= 500) ? ++cuenta_x2 : 500;
	motor_di.cuenta_vel_cero += 1;
	motor_dd.cuenta_vel_cero += 1;
	motor_ti.cuenta_vel_cero += 1;
	motor_td.cuenta_vel_cero += 1;
	motor_dd.Input.tiempo = (motor_dd.Input.FLAG_E) ? ++motor_dd.Input.tiempo : 0;	//CUENTO SI BANDERA PULSO
	motor_di.Input.tiempo = (motor_di.Input.FLAG_E) ? ++motor_di.Input.tiempo : 0;	//CUENTO SI BANDERA PULSO
	motor_td.Input.tiempo = (motor_td.Input.FLAG_E) ? ++motor_td.Input.tiempo : 0;	//CUENTO SI BANDERA PULSO
	motor_ti.Input.tiempo = (motor_ti.Input.FLAG_E) ? ++motor_ti.Input.tiempo : 0;	//CUENTO SI BANDERA PULSO
	if (ESTADO == CALIBRACION || ESTADO == RPM_REMOTO || ESTADO == DUTY_REMOTO){
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
**     Event       :  Input_Hall_TD_OnCapture (module Events)
**
**     Component   :  Input_Hall_TD [Capture]
**     Description :
**         This event is called on capturing of Timer/Counter actual
**         value (only when the component is enabled - <Enable> and the
**         events are enabled - <EnableEvent>.This event is available
**         only if a <interrupt service/event> is enabled.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void Input_Hall_TD_OnCapture(void)
{
  /* Write your code here ... */
	if(!motor_td.Input.FLAG_E){							//ESPERO PROCESAR VALOR
		motor_td.Input.err = Input_Hall_TD_GetCaptureValue(&motor_td.Input.aux);
		motor_td.Input.FLAG_E = TRUE;
		motor_td.Input.tiempo = 0;
	}
}

/*
** ===================================================================
**     Event       :  Input_Hall_DI_OnCapture (module Events)
**
**     Component   :  Input_Hall_DI [Capture]
**     Description :
**         This event is called on capturing of Timer/Counter actual
**         value (only when the component is enabled - <Enable> and the
**         events are enabled - <EnableEvent>.This event is available
**         only if a <interrupt service/event> is enabled.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void Input_Hall_DI_OnCapture(void)
{
	if(!motor_di.Input.FLAG_E){							//ESPERO PROCESAR VALOR
		motor_di.Input.err = Input_Hall_DI_GetCaptureValue(&motor_di.Input.aux);
		motor_di.Input.FLAG_E = TRUE;
		motor_di.Input.tiempo = 0;
	}
}

/*
** ===================================================================
**     Event       :  Input_Hall_TI_OnCapture (module Events)
**
**     Component   :  Input_Hall_TI [Capture]
**     Description :
**         This event is called on capturing of Timer/Counter actual
**         value (only when the component is enabled - <Enable> and the
**         events are enabled - <EnableEvent>.This event is available
**         only if a <interrupt service/event> is enabled.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void Input_Hall_TI_OnCapture(void)
{
	if(!motor_ti.Input.FLAG_E){							//ESPERO PROCESAR VALOR
		motor_ti.Input.err = Input_Hall_TI_GetCaptureValue(&motor_ti.Input.aux);
		motor_ti.Input.FLAG_E = TRUE;
		motor_ti.Input.tiempo = 0;
	}
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
	direccion.Input.err = ReceptorDireccion_GetCaptureValue(&direccion.Input.datos[direccion.Input.indices]);
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
	static byte step_direccion = FREQ_PWM_DUTY>>1; //Define el tiempo en ALTO
	
	if (pap.FLAG_DIRECCION){						//Esta habilitado el PWM de direccion?	
		if (pap.direccion_lectura <= LIMITE_DIRECCION_DERECHO || pap.direccion_lectura >= LIMITE_DIRECCION_IZQUIERDO){
			if (pap.pwm_direccion == 0){				
				BitOut_DIR_PWM_SetVal();				//Salida en ALTO
			}
			if (pap.pwm_direccion == step_direccion){	//Ya paso el tiempo en alto?		
				BitOut_DIR_PWM_ClrVal();				//Salida en BAJO
			}
			pap.pwm_direccion++;						//Incremento un step del PWM
			if (pap.pwm_direccion == FREQ_PWM_DUTY){	//Cuando se alcanza el STEP de la frecuencia, reseteo
				//pap.pasos_dados++;						//Se realizo UN PASO
				pap.pwm_direccion = 0;					//Reseteo el step
			}
		} else {
			pap.FLAG_DIRECCION = false;
		}		
	} else {
		//pap.pasos_dados = 0;
		BitOut_DIR_PWM_ClrVal();
	}
  /* Write your code here ... */
}

/*
** ===================================================================
**     Event       :  UART_OnError (module Events)
**
**     Component   :  UART [AsynchroSerial]
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
void UART_OnError(void)
{
  /* Write your code here ... */
}

/*
** ===================================================================
**     Event       :  UART_OnRxChar (module Events)
**
**     Component   :  UART [AsynchroSerial]
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
void UART_OnRxChar(void)
{
	UART_RecvChar(&serie.rx_buf[serie.rx_next]);
	if (serie.rx_buf[serie.rx_next] == '\n' && serie.rx_buf[serie.rx_next - 1] == '\r'){
		serie.FLAG_RX = true;
	}
	inc(serie.rx_next);
}

/*
** ===================================================================
**     Event       :  UART_OnTxChar (module Events)
**
**     Component   :  UART [AsynchroSerial]
**     Description :
**         This event is called after a character is transmitted.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void UART_OnTxChar(void)
{
  /* Write your code here ... */
}

/*
** ===================================================================
**     Event       :  UART_OnFullRxBuf (module Events)
**
**     Component   :  UART [AsynchroSerial]
**     Description :
**         This event is called when the input buffer is full;
**         i.e. after reception of the last character 
**         that was successfully placed into input buffer.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void UART_OnFullRxBuf(void)
{
  /* Write your code here ... */
}

/*
** ===================================================================
**     Event       :  UART_OnFreeTxBuf (module Events)
**
**     Component   :  UART [AsynchroSerial]
**     Description :
**         This event is called after the last character in output
**         buffer is transmitted.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void UART_OnFreeTxBuf(void)
{
  /* Write your code here ... */
}

/*
** ===================================================================
**     Event       :  UART_OnTxComplete (module Events)
**
**     Component   :  UART [AsynchroSerial]
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
void UART_OnTxComplete(void)
{
  /* Write your code here ... */
}

/*
** ===================================================================
**     Event       :  Btn_Emergencia_OnInterrupt (module Events)
**
**     Component   :  Btn_Emergencia [ExtInt]
**     Description :
**         This event is called when an active signal edge/level has
**         occurred.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void Btn_Emergencia_OnInterrupt(void)
{
  /* Write your code here ... */
	emergencias++;
	BitLed_Verde_ClrVal();
	WDog1_Clear();
	while(Btn_Emergencia_GetVal()){
		WDog1_Clear();
		Reset_PIDs(motor_di);
		Reset_PIDs(motor_dd);
		Reset_PIDs(motor_ti);
		Reset_PIDs(motor_td);
		motor_di.tension = 0;
		motor_dd.tension = 0;
		motor_ti.tension = 0;
		motor_td.tension = 0;
		motor_dd.duty  = 0;
		motor_di.duty  = 0;
		motor_td.duty  = 0;
		motor_ti.duty  = 0;
		motor_dd.rpm  = 0;
		motor_di.rpm  = 0;
		motor_td.rpm  = 0;
		motor_ti.rpm  = 0;
		RPM_SET = 0;
		Out_PWM_TD_SetRatio16(DUTY_CERO);
		Out_PWM_DD_SetRatio16(DUTY_CERO);
		Out_PWM_DI_SetRatio16(DUTY_CERO);
		Out_PWM_TI_SetRatio16(DUTY_CERO);
	}
	while(Btn_Emergencia_GetVal()){
		WDog1_Clear();
		Reset_PIDs(motor_di);
		Reset_PIDs(motor_dd);
		Reset_PIDs(motor_ti);
		Reset_PIDs(motor_td);
		motor_di.tension = 0;
		motor_dd.tension = 0;
		motor_ti.tension = 0;
		motor_td.tension = 0;
		motor_dd.control = 0;
		motor_di.control = 0;
		motor_td.control = 0;
		motor_ti.control = 0;
		motor_dd.duty = 0;
		motor_di.duty  = 0;
		motor_td.duty  = 0;
		motor_ti.duty  = 0;
		motor_dd.rpm  = 0;
		motor_di.rpm  = 0;
		motor_td.rpm  = 0;
		motor_ti.rpm  = 0;
		RPM_SET = 0;
		Out_PWM_TD_SetRatio16(DUTY_CERO);
		Out_PWM_DD_SetRatio16(DUTY_CERO);
		Out_PWM_DI_SetRatio16(DUTY_CERO);
		Out_PWM_TI_SetRatio16(DUTY_CERO);
	}
	BitLed_Verde_SetVal();
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
