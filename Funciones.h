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
unsigned char Vel_Cero(MOTOR motor_1, MOTOR motor_2, MOTOR motor_3, MOTOR motor_4);
void Tension2Duty(MOTOR *motor_x);
void Reset_PIDs(MOTOR motor_x);
unsigned short GrayToBin(unsigned short N);
void ResetVar (void);
void RX (SERIE seriex, MOTOR *motordi, MOTOR *motordd, MOTOR *motortd, MOTOR *motorti, uint8 *ESTADO);
void NumeroFin(SERIE *serie_x);
MOTOR_TX TX_Motor(MOTOR motor_x);
void Motor2Send(SERIE *serie, MOTOR_TX *motor);



#endif /* FUNCIONES_H_ */

