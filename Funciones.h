/*
 * Funciones.h
 *
 *  Created on: Oct 30, 2017
 *      Author: Esteban
 */

#ifndef FUNCIONES_H_
#define FUNCIONES_H_

#include "Struct2.h"
#include "Defines2.h"
#include "PE_Types.h"
#include "Encoder_DD.h"
#include "BitIoLdd16.h"
#include "Encoder_DI.h"
#include "BitIoLdd17.h"
#include "Encoder_TD.h"
#include "BitIoLdd18.h"
#include "Encoder_TI.h"
#include "BitIoLdd19.h"
/*
 * ######################## FUNCIONES ###############################
 *  * ######################## FUNCIONES ###############################
 *   * ######################## FUNCIONES ###############################
 *    * ######################## FUNCIONES ###############################
 *       
 */

long Mapeo(long x, long in_min, long in_max, long out_min, long out_max);
void Get_Remoto(REMOTO *remoto_x);
void GetVelocidad (MOTOR *motor_x);
void RPM_Cero(MOTOR *motor_x);
void Error_PID(MOTOR *motor_x);
void CtrlPID_SetK(MOTOR motor_x);
void CtrlPID_Control(MOTOR *motor_x);
unsigned char Vel_Cero(MOTOR motor_1, MOTOR motor_2, MOTOR motor_3, MOTOR motor_4);
unsigned char Tension_Cero(MOTOR motor_1, MOTOR motor_2, MOTOR motor_3, MOTOR motor_4);
void Tension2Duty(MOTOR *motor_x);
void Reset_PIDs(MOTOR motor_x);
unsigned short GrayToBin(unsigned short N);
void ResetVar (void);
void NumeroFin(SERIE *serie_x);
void GetEncoder(MOTOR *motor_x);
void Duty2Motor(PC *pc, MOTOR *motor);



#endif /* FUNCIONES_H_ */

