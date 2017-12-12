/* ###################################################################
**     Filename    : Events.h
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
**         Input_Encoder_DD_OnCapture            - void Input_Encoder_DD_OnCapture(void);
**         IntEncoder_DD_OnInterrupt - void IntEncoder_DD_OnInterrupt(void);
**         IntEncoder_DI_OnInterrupt - void IntEncoder_DI_OnInterrupt(void);
**         Btn_SW2_OnInterrupt        - void Btn_SW2_OnInterrupt(void);
**         Btn_SW1_OnInterrupt        - void Btn_SW1_OnInterrupt(void);
**         IntTiempo_OnInterrupt     - void IntTiempo_OnInterrupt(void);
**         Cpu_OnNMI                 - void Cpu_OnNMI(void);
**
** ###################################################################*/
/*!
** @file Events.h
** @version 01.00
** @brief
**         This is user's event module.
**         Put your event handler code here.
*/         
/*!
**  @addtogroup Events_module Events module documentation
**  @{
*/         

#ifndef __Events_H
#define __Events_H
/* MODULE Events */

#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
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

#ifdef __cplusplus
extern "C" {
#endif 

void ADC_DD_OnEnd(void);
/*
** ===================================================================
**     Event       :  ADC_DD_OnEnd (module Events)
**
**     Component   :  ADC_DD [ADC]
**     Description :
**         This event is called after the measurement (which consists
**         of <1 or more conversions>) is/are finished.
**         The event is available only when the <Interrupt
**         service/event> property is enabled.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

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
void Input_Encoder_DD_OnCapture(void);

void Btn_SW2_OnInterrupt(void);
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

void Btn_SW1_OnInterrupt(void);
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
void IntTiempo_OnInterrupt(void);

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
void Cpu_OnNMI(void);


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
void Input_Encoder_TD_OnCapture(void);

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
void Input_Encoder_DI_OnCapture(void);

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
void Input_Encoder_TI_OnCapture(void);

void ADC_I_OnEnd(void);
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

void ADC_I_OnCalibrationEnd(void);
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
void ReceptorVelocidad_OnCapture(void);

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
void ReceptorDireccion_OnCapture(void);

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
void IntDireccion_OnInterrupt(void);

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
void UART_MODBUS_OnError(void);

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
void UART_MODBUS_OnRxChar(void);

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
void UART_MODBUS_OnTxChar(void);

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
void UART_MODBUS_OnFullRxBuf(void);

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
void UART_MODBUS_OnFreeTxBuf(void);

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
void UART_MODBUS_OnTxComplete(void);

void Btn_Emergencia_OnInterrupt(void);
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

/* END Events */

#ifdef __cplusplus
}  /* extern "C" */
#endif 

#endif 
/* ifndef __Events_H*/
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
