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
#include "Hall_DD.h"
#include "BitIoLdd16.h"
#include "Hall_DI.h"
#include "BitIoLdd17.h"
#include "Hall_TD.h"
#include "BitIoLdd18.h"
#include "Hall_TI.h"
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
void Error_PID(MOTOR *motor_x);
void CtrlPID_SetK(MOTOR motor_x);
void CtrlPID_Control(MOTOR *motor_x);
unsigned char Vel_Cero(MOTOR motor_1, MOTOR motor_2, MOTOR motor_3, MOTOR motor_4);
unsigned char Tension_Cero(MOTOR motor_1, MOTOR motor_2, MOTOR motor_3, MOTOR motor_4);
void Tension2Duty(MOTOR *motor_x);
void Reset_PIDs(MOTOR motor_x);
unsigned short GrayToBin(unsigned short N);
void NumeroFin(SERIE *serie_x);
void GetHall(MOTOR *motor_x);
void Duty2Motor(PC *pc, MOTOR *motor);



#endif /* FUNCIONES_H_ */

