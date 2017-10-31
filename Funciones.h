/*
 * Funciones.h
 *
 *  Created on: Oct 30, 2017
 *      Author: Esteban
 */

#ifndef FUNCIONES_H_
#define FUNCIONES_H_



#endif /* FUNCIONES_H_ */

#include "Struct.h"
#include "Defines.h"
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
void Control_LC(MOTOR *motor_x);
void CtrlPID_SetK(MOTOR motor_x);
void CtrlPID_Control(MOTOR *motor_x);
bool Vel_Cero(MOTOR motor_1, MOTOR motor_2, MOTOR motor_3, MOTOR motor_4);
void Tension2Duty(MOTOR *motor_x);
void SetDuty(MOTOR motor_x);
void Reset_PIDs(MOTOR motor_x);
void RX(void);
void DECODIFICADO(byte* codigo, byte inicio);
void TX(void);
word GrayToBin(word N);
void ResetVar (void);
