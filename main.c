/* ###################################################################
**     Filename    : main.c
**     Project     : Pre1Motor1.0
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
#include "PTE25.h"
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

/*
 * ######################## DEFINES ###############################
 */
#define LA_VELOCIDAD			1
#define LA_REMOTO				4
#define LC_REMOTO				2
#define LC_PC					3
#define CALIBRACION				6
#define PASOS					7

#define	MOTOR_DI				0
#define	MOTOR_DD				1
#define	MOTOR_TI				2
#define	MOTOR_TD				3

#define DERECHA					1
#define IZQUIERDA				0
#define REVERSA					1
#define	ADELANTE				0

#define PERDIDA_REMOTO			1000	//ms que deben pasar para definir la perdida de la se�al REMOTO
#define CUENTAS_REMOTO			100		//Limite para definir el ancho del pulso CALIBRADO
#define RESET_VELOCIDAD_MS		600
#define	VELOCIDAD				0
#define	DIRECCION				1
#define REMOTO_ANCHO_PULSO		28		//Desplazamiento MINIMO del ancho del pulso REMOTO x10us
#define REMOTO_VENTANA			0		//definir
#define REMOTO_VEL_CERO			150
#define REMOTO_VEL_MAX			204
#define REMOTO_VEL_MIN			116
#define REMOTO_DIR_CERO			150		
#define REMOTO_DIR_MAX			205
#define REMOTO_DIR_MIN			97
#define VENTANA_DIRECCION		10

#define K_PID					33
#define TI_PID					0.4

#define BUTTON_PRESSED      	0
#define BUTTON_NOT_PRESSED  	1
#define true					1
#define false					0

#define DUTY_MAX				0
#define DUTY_MIN				40000//37000

#define LIMITE_DIRECCION_DERECHO	246
#define LIMITE_DIRECCION_IZQUIERDO	150

#define SET_RPM_MAX				100		//definir
#define SET_RPM_MIN				0

#define U_MAX					19350
#define U_MIN					0

#define RPM_STEP				5000//5
#define FREQ_INPUT				374912
#define FREQ_RECEPTOR			2999296
#define ADC_MIN					0
#define ADC_MAX					3300
#define BIT_16					65400
#define GET_VEL(x)				(((Period[x]<<1)*1000*10)/FREQ_INPUT)
#define GET_RECEP(x)			(((Period_Receptor[x])*10000*10)/FREQ_RECEPTOR)

#define MAX16BIT				65535
#define MIN16BIT				0

#define BUF_SIZE 				64
#define inc(x) 					{x++; x&=(BUF_SIZE-1);}

#define TIMEOUT_RX				5000
#define DIRECCION_MODBUS		9

#define DIRECCION_HORARIA		BitOut_DIR_SENT_SetVal()
#define	DIRECCION_ANTI			BitOut_DIR_SENT_ClrVal()
#define DIRECCION_ON			BitOut_DIR_EN_ClrVal()
#define DIRECCION_OFF			BitOut_DIR_EN_SetVal()

/*
 * ######################## ESTRUCTURAS ###############################
 */
typedef struct input_capture {
	uint16 datos[2];				//Contador de pulsos de cada flanco [1xMOTOR 2xVariableAnteriorYActual] 
	int16 periodo;					//Diferencia entre los pulsos 2 y 1 de cada MOTOR
	uint8 indices;					//Indice de pulsos
	int8 err;						//Por si ocurre un error al momento de leer pulsos
} INPUT_CAPTURE;

typedef struct motor {
	INPUT_CAPTURE Input;
	uint32 posicion_pulsos;			//Contador de Flancos del Motor
	uint16 cuenta_vel_cero;			//Contador para determinar Vel CERO si no se registran nuevos flancos
	uint16 adc;						//Valores de los ADC de cada MOTOR
	uint16 encoder_ms;				//Velocidad en ms del encoder
	uint16 rpm;						//Lectura RPM
	uint16 RPM_set;					//SETPOINT DE RPM GLOBAL
	int32 error_RPM;				//Entrada al PID [SETPOINT - LECTURA]
	
	uint16 control;					//Salida del PID
	//word *Control;				//Puntero de las salidas del PID
	word tension;					//Salida del PID para la FUNCION Tension_PWM
	word duty;						//Duty aplicado a los PWM de cada MOTOR
	
} MOTOR;

typedef struct pap {
	uint8 pwm_direccion;			//Contador para el PWM Manual
	uint16 pasos_dados;				//Cantidad de PASOS que se ha dado	//pwm_pasos
	uint16 pasos_adar;				//Pasos que se desean - PASO A PASO //pasos_direccion
	uint8 lectura_direccion;		//Lee la posicion del PASO a PASO
	uint8 direccion_set;			//SETPOINT de direcion
	uint8 FLAG_EN;					//PASO A PASO ENEABLE
	uint8 FLAG_SENTIDO;				//PASO A PASO SENTIDO DE GIRO
	
} PAP;

typedef struct remoto {
	INPUT_CAPTURE Input;
	uint16 remoto_cero;				//Ancho del pulso en CERO
	uint16 cuenta_remoto;			//Cuenta las veces para definir ancho de CALIBRACION
	uint16 ms;						//Ancho del pulso del receptor REMOTO
	uint16 perdida_senal_remoto;	//Contador para detectar perdida de se�al en modo REMOTO o CALIBRACION
} ROMOTO;

typedef struct serie {
	uint8 tx_buf[BUF_SIZE];            	// TX buffer
	uint8 tx_next;						// TX indice siguiente a GUARDAR						
	uint8 tx_sent;					  	// TX indice siguiente a MANDAR
	uint8 rx_buf[BUF_SIZE];            	// RX buffer
	uint8 rx_next;						// RX indice siguiente a GUARDAR
	uint8 rx_read;  					// RX indice siguiente a LEERx
	uint8 FLAG_RX;						//Hay datos para procesar RECIBIDOS
	uint8 FLAG_TX;						//Hay datos para procesar ENVIAR
} SERIE;
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
word perdida_senal_remoto[2];		//Contador para detectar perdida de se�al en modo REMOTO o CALIBRACION
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
byte FLAG_DIRECCION = false;	//FLAG usado para definir cuando hay una se�al de direccion

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
  for(;;){
	  Get_Velocidad();					//LEER DATOS VELOCIDAD
	  Get_Corriente();					//LEER DATOS CORRIENTE
	  Get_Direccion();					//LEER DIRECCION
	  switch (ESTADO){
	  
	  case LA_VELOCIDAD:
		  if (lectura_nueva){
			  lectura_nueva = false;
			  duty_entrada[MOTOR_DD] = (duty_entrada[MOTOR_DD] >= MAX16BIT) ? MAX16BIT : duty_entrada[MOTOR_DD];
			  duty_entrada[MOTOR_DD] = (duty_entrada[MOTOR_DD] <= MIN16BIT) ? MIN16BIT : duty_entrada[MOTOR_DD];
			  duty[MOTOR_DD] = duty_entrada[MOTOR_DD];
			  Out_PWM_DD_SetRatio16(duty[MOTOR_DD]);
		  }
		  break;
		  
	  case LC_REMOTO:
		  if (lectura_nueva){
			  Reset_PIDs();
			  lectura_nueva = false;
		  }
		  Get_Remoto();
		  
		  //LEER DIRECCION
		  receptor_ms[DIRECCION] = (receptor_ms[DIRECCION] >= remoto_dir_cero + REMOTO_ANCHO_PULSO) ? remoto_dir_cero + REMOTO_ANCHO_PULSO : receptor_ms[DIRECCION];
		  receptor_ms[DIRECCION] = (receptor_ms[DIRECCION] <= remoto_dir_cero - REMOTO_ANCHO_PULSO) ? remoto_dir_cero - REMOTO_ANCHO_PULSO : receptor_ms[DIRECCION];
		  direccion_set = Mapeo(receptor_ms[DIRECCION],remoto_dir_cero - REMOTO_ANCHO_PULSO,remoto_dir_cero + REMOTO_ANCHO_PULSO,LIMITE_DIRECCION_IZQUIERDO,LIMITE_DIRECCION_DERECHO);
		  //END LEER DIRECCION
		  
		  //LEER VELOCIDAD
		  receptor_ms[VELOCIDAD] = (receptor_ms[VELOCIDAD] >= remoto_vel_cero + REMOTO_ANCHO_PULSO) ? remoto_vel_cero + REMOTO_ANCHO_PULSO : receptor_ms[VELOCIDAD];
		  receptor_ms[VELOCIDAD] = (receptor_ms[VELOCIDAD] <= remoto_vel_cero - REMOTO_ANCHO_PULSO) ? remoto_vel_cero - REMOTO_ANCHO_PULSO : receptor_ms[VELOCIDAD];
			
		  //CAMBIO DE SENTIDO
		  if (receptor_ms[VELOCIDAD] < (remoto_vel_cero - REMOTO_VENTANA)){//ES REVERSA?
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
			  } else {
				  receptor_ms[VELOCIDAD] = remoto_vel_cero;
			  }
		  }
		  if (sentido_act == ADELANTE){
			  RPM_SET=Mapeo(receptor_ms[VELOCIDAD],remoto_vel_cero,remoto_vel_cero + REMOTO_ANCHO_PULSO,SET_RPM_MIN,rpm_max_control);
		  } else {
			  RPM_SET=Mapeo(receptor_ms[VELOCIDAD],remoto_vel_cero,remoto_vel_cero - REMOTO_ANCHO_PULSO,SET_RPM_MIN,rpm_max_control);
		  }
		  if (RPM_SET > rpm_max_control){
			  RPM_SET = rpm_max_control;
		  }
		  //END CAMBIO DE SENTIDO
		  
		  if (cuenta_PID >= 100){ //10 milisegundos periodo de muestreo
			  cuenta_PID = 0;
			  
			  if (RPM_SET != 0){
				  Control_LC();
				  //RPM_SET = 0?
				  RPM_Cero();
				  Var_Tension[MOTOR_DD] = Var_Control[MOTOR_DD];
				  Var_Tension[MOTOR_DI] = Var_Control[MOTOR_DI];
				  Var_Tension[MOTOR_TD] = Var_Control[MOTOR_TD];
				  Var_Tension[MOTOR_TI] = Var_Control[MOTOR_TI];
			  } else {
				  Var_Tension[MOTOR_DD] = 0;
				  Var_Tension[MOTOR_DI] = 0;
				  Var_Tension[MOTOR_TD] = 0;
				  Var_Tension[MOTOR_TI] = 0;
				  Var_Control[MOTOR_DD] = 0;
				  Var_Control[MOTOR_DI] = 0;
				  Var_Control[MOTOR_TD] = 0;
				  Var_Control[MOTOR_TI] = 0;
				  Reset_PIDs();
			  }
		  }
		  //PERDIDA DE SE�AL - SALE DEL ESTADO
		  if (perdida_senal_remoto[VELOCIDAD] >= 500 || perdida_senal_remoto[DIRECCION] >= 500){
			  perdida_senal_remoto[VELOCIDAD] = 0;
			  perdida_senal_remoto[DIRECCION] = 0;
			  ESTADO = 30;
		  }
		  if (cuenta_ENVIAR >= 500){
			  cuenta_ENVIAR = 0;
		  }
		  break;
	  case LA_REMOTO:
		  Get_Remoto();
		  
		  //LEER DIRECCION
		  receptor_ms[DIRECCION] = (receptor_ms[DIRECCION] >= remoto_dir_cero + REMOTO_ANCHO_PULSO) ? remoto_dir_cero + REMOTO_ANCHO_PULSO : receptor_ms[DIRECCION];
		  receptor_ms[DIRECCION] = (receptor_ms[DIRECCION] <= remoto_dir_cero - REMOTO_ANCHO_PULSO) ? remoto_dir_cero - REMOTO_ANCHO_PULSO : receptor_ms[DIRECCION];
		  direccion_set = Mapeo(receptor_ms[DIRECCION],remoto_dir_cero - REMOTO_ANCHO_PULSO,remoto_dir_cero + REMOTO_ANCHO_PULSO,LIMITE_DIRECCION_IZQUIERDO,LIMITE_DIRECCION_DERECHO);
		  //END LEER DIRECCION
		  
		  //LEER VELOCIDAD
		  receptor_ms[VELOCIDAD] = (receptor_ms[VELOCIDAD] >= remoto_vel_cero + REMOTO_ANCHO_PULSO) ? remoto_vel_cero + REMOTO_ANCHO_PULSO : receptor_ms[VELOCIDAD];
		  receptor_ms[VELOCIDAD] = (receptor_ms[VELOCIDAD] <= remoto_vel_cero - REMOTO_ANCHO_PULSO) ? remoto_vel_cero - REMOTO_ANCHO_PULSO : receptor_ms[VELOCIDAD];
		  		  
		  //CAMBIO DE SENTIDO
		  if (receptor_ms[VELOCIDAD] < (remoto_vel_cero - REMOTO_VENTANA)){//ES REVERSA?
			  sentido_act = REVERSA;
		  } else {
			  sentido_act = ADELANTE;
		  }
		  if (sentido_act != sentido_ant){
			  if (Vel_Cero()){
				  sentido_ant = sentido_act;
				  Out_Reversa_PutVal(sentido_act);
			  } else {
				  receptor_ms[VELOCIDAD] = remoto_vel_cero;
			  }
		  }
		  if (sentido_act == ADELANTE){
			  RPM_SET=Mapeo(receptor_ms[VELOCIDAD],remoto_vel_cero,remoto_vel_cero + REMOTO_ANCHO_PULSO,U_MIN,tension_max_control);
		  } else {
			  RPM_SET=Mapeo(receptor_ms[VELOCIDAD],remoto_vel_cero,remoto_vel_cero - REMOTO_ANCHO_PULSO,U_MIN,tension_max_control);
		  }
		  if (RPM_SET > tension_max_control){
			  RPM_SET = tension_max_control;
		  }
		  //END CAMBIO DE SENTIDO
		  Var_Tension[MOTOR_DI] = RPM_SET;
		  Var_Tension[MOTOR_DD] = RPM_SET;
		  Var_Tension[MOTOR_TI] = RPM_SET;
		  Var_Tension[MOTOR_TD] = RPM_SET;
		  //PERDIDA DE SE�AL - SALE DEL ESTADO
		  if (perdida_senal_remoto[VELOCIDAD] >= 500 || perdida_senal_remoto[DIRECCION] >= 500){
			  perdida_senal_remoto[VELOCIDAD] = 0;
			  perdida_senal_remoto[DIRECCION] = 0;
			  ESTADO = 30;
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
				  Var_Tension[MOTOR_DD] = Var_Control[MOTOR_DD];
				  Var_Tension[MOTOR_DI] = Var_Control[MOTOR_DI];
				  Var_Tension[MOTOR_TD] = Var_Control[MOTOR_TD];
				  Var_Tension[MOTOR_TI] = Var_Control[MOTOR_TI];
			  } else {
				  Var_Tension[MOTOR_DD] = 0;
				  Var_Tension[MOTOR_DI] = 0;
				  Var_Tension[MOTOR_TD] = 0;
				  Var_Tension[MOTOR_TI] = 0;				  
			  }
		  }		  
		  if (cuenta_ENVIAR >= 500){
			  cuenta_ENVIAR = 0;
		  }
		  break;
		  
	  case CALIBRACION:
		  Get_Remoto();
		  if (receptor_ms[VELOCIDAD] != 0 || receptor_ms[DIRECCION] != 0 || !lectura_nueva){
			  lectura_nueva = false;
		  
			  if (receptor_ms[VELOCIDAD] == remoto_vel_cero){
				  cuenta_remoto_vel++;			  
			  } else {
				  remoto_vel_cero = receptor_ms[VELOCIDAD];
				  cuenta_remoto_vel = 0;
			  }
			  if (receptor_ms[DIRECCION] == remoto_dir_cero){
				  cuenta_remoto_dir++;
			  } else {
				  remoto_dir_cero = receptor_ms[DIRECCION];
				  cuenta_remoto_dir = 0;
			  }		  
			  if (cuenta_remoto_vel >= CUENTAS_REMOTO && cuenta_remoto_dir >= CUENTAS_REMOTO){
				  ESTADO = LC_REMOTO;
				  lectura_nueva = true;
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
		  
	  case 30:
		  ESTADO = 30;
		  Var_Tension[MOTOR_DI] = 0;
		  Var_Tension[MOTOR_DD] = 0;
		  Var_Tension[MOTOR_TI] = 0;
		  Var_Tension[MOTOR_TD] = 0;
		  break;
		  
	  default:
		  break;
	  }
	  Tension_PWM();
	  
	  //PRUEBA CONTACTORES
	  if (FLAG_SW1){
		  Out_Reversa_NegVal();
		  FLAG_SW1 = 0;
	  }
	  
	  //####### DIRECCION
	  FLAG_DIRECCION = 1;
	  if (FLAG_DIRECCION){
		  FLAG_DIRECCION = false;
		  DIRECCION_ON;
		  direccion_set = (direccion_set >= LIMITE_DIRECCION_DERECHO) ? LIMITE_DIRECCION_DERECHO : direccion_set;
		  direccion_set = (direccion_set <= LIMITE_DIRECCION_IZQUIERDO) ? LIMITE_DIRECCION_IZQUIERDO : direccion_set;
		  if (direccion_set > (lectura_direccion + VENTANA_DIRECCION)){
			  FLAG_DIRECCION_PWM_EN = true;
			  DIRECCION_HORARIA;
		  }
		  if (direccion_set < (lectura_direccion - VENTANA_DIRECCION)){
			  FLAG_DIRECCION_PWM_EN = true;
			  DIRECCION_ANTI;
		  }
	  }
	  if ((direccion_set <= (lectura_direccion + VENTANA_DIRECCION)) && (direccion_set >= (lectura_direccion - VENTANA_DIRECCION))){
		  FLAG_DIRECCION_PWM_EN = false;
		  pwm_direccion = 0;
		  BitOut_DIR_PWM_ClrVal();
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
		  PTE25_NegVal();
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
 *     * ######################## FUNCIONES ###############################
 *      * ######################## FUNCIONES ###############################
 *       * ######################## FUNCIONES ###############################
 *       
 */

long Mapeo(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Get_Remoto(void){
	if (FLAG_RECEPTOR[VELOCIDAD]){
		FLAG_RECEPTOR[VELOCIDAD] = 0;
		receptor_ms[VELOCIDAD] = GET_RECEP(VELOCIDAD);
		perdida_senal_remoto[VELOCIDAD] = 0;
	}
	if (FLAG_RECEPTOR[DIRECCION]){
		FLAG_RECEPTOR[DIRECCION] = 0;
		receptor_ms[DIRECCION] = GET_RECEP(DIRECCION);
		perdida_senal_remoto[DIRECCION] = 0;
	}
}
void Get_Velocidad(void){	
	byte i;
		
	if (FLAG_TIEMPO[MOTOR_DI]){
		FLAG_TIEMPO[MOTOR_DI] = false;
		encoder_ms[MOTOR_DI] = GET_VEL(MOTOR_DI);
		rpm[MOTOR_DI]=60000/(encoder_ms[MOTOR_DI]/10*24);
	}
	if (FLAG_TIEMPO[MOTOR_DD]){
		FLAG_TIEMPO[MOTOR_DD] = false;
		encoder_ms[MOTOR_DD] = GET_VEL(MOTOR_DD);
		rpm[MOTOR_DD]=60000/(encoder_ms[MOTOR_DD]/10*24);
	}
	if (FLAG_TIEMPO[MOTOR_TI]){
		FLAG_TIEMPO[MOTOR_TI] = false;
		encoder_ms[MOTOR_TI] = GET_VEL(MOTOR_TI);
		rpm[MOTOR_TI]=60000/(encoder_ms[MOTOR_TI]/10*24);
	}
	if (FLAG_TIEMPO[MOTOR_TD]){
		FLAG_TIEMPO[MOTOR_TD] = false;
		encoder_ms[MOTOR_TD] = GET_VEL(MOTOR_TD);
		rpm[MOTOR_TD]=60000/(encoder_ms[MOTOR_TD]/10*24);
	}
	for (i=0;i<=3;i++){									//Si en 600 ms no se recibio
		if (cuenta_vel_cero[i] >= RESET_VELOCIDAD_MS){	//un pulso, se pone vel en cero										
			FLAG_TIEMPO[i] = false;
			encoder_ms[i] = 0;
			rpm[i] = 0;
			cuenta_vel_cero[i] = 0;
			index[i] = 0;
		}
	}
}
void Get_Corriente(void){
	if (FLAG_ADC){
		FLAG_ADC = false;
		ADC_I_GetChanValue16(MOTOR_DI,adc_value[MOTOR_DI]);
		ADC_I_GetChanValue16(MOTOR_DD,adc_value[MOTOR_DD]);
		ADC_I_GetChanValue16(MOTOR_TI,adc_value[MOTOR_TI]);
		ADC_I_GetChanValue16(MOTOR_TD,adc_value[MOTOR_TD]);
		ADC_I_Measure(FALSE);
	}
}
void Get_Direccion(void){
	
		byte lectura;
		byte z;
		if (cuenta_DIRECCION >= 100){

			cuenta_DIRECCION = cuenta_DIRECCION - 100;
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
		lectura_direccion = GrayToBin(lectura);
		if ((direccion_set <= (lectura_direccion + 1)) && (direccion_set >= (lectura_direccion - 1))){
			lectura_direccion = lectura_direccion;
			  }
		  
}
void RPM_Cero(void){
	byte i;
	for (i=0;i<=3;i++){ 
		if (rpm[i] == 0 && RPM_SET==0){ //1 segundo
			Var_Control[i] = 0;
		}
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
	error_RPM[MOTOR_DI] = (RPM_SET - rpm[MOTOR_DI]);
	error_RPM[MOTOR_DD] = (RPM_SET - rpm[MOTOR_DD]);
	error_RPM[MOTOR_TI] = (RPM_SET - rpm[MOTOR_TI]);
	error_RPM[MOTOR_TD] = (RPM_SET - rpm[MOTOR_TD]);
	
	if (RPM_SET < 10){
	  CtrlPID_DI_Set_K((float)(K_PID/2));
	  CtrlPID_DD_Set_K((float)(K_PID/2));
	  CtrlPID_TI_Set_K((float)(K_PID/2));
	  CtrlPID_TD_Set_K((float)(K_PID/2));
	  CtrlPID_DI_Control(error_RPM[MOTOR_DI],Control[MOTOR_DI]);
	  CtrlPID_DD_Control(error_RPM[MOTOR_DD],Control[MOTOR_DD]);
	  CtrlPID_TI_Control(error_RPM[MOTOR_TI],Control[MOTOR_TI]);
	  CtrlPID_TD_Control(error_RPM[MOTOR_TD],Control[MOTOR_TD]);
	} else {
	  CtrlPID_DI_Set_K((float)(K_PID));
	  CtrlPID_DD_Set_K((float)(K_PID));
	  CtrlPID_TI_Set_K((float)(K_PID));
	  CtrlPID_TD_Set_K((float)(K_PID));
	  CtrlPID_DI_Control(error_RPM[MOTOR_DI],Control[MOTOR_DI]);
	  CtrlPID_DD_Control(error_RPM[MOTOR_DD],Control[MOTOR_DD]);
	  CtrlPID_TI_Control(error_RPM[MOTOR_TI],Control[MOTOR_TI]);
	  CtrlPID_TD_Control(error_RPM[MOTOR_TD],Control[MOTOR_TD]);
	}
	//TI = 0.5352609
}
bool Vel_Cero(void){
	if ((encoder_ms[MOTOR_DD] + encoder_ms[MOTOR_DI] +
		  encoder_ms[MOTOR_TD] + encoder_ms[MOTOR_TI])>=1){
		return FALSE;
	}
	return TRUE;
}
void Tension_PWM(void){
	Var_Tension[MOTOR_DI] = (Var_Tension[MOTOR_DI] >= U_MAX) ? U_MAX : Var_Tension[MOTOR_DI];
	Var_Tension[MOTOR_DD] = (Var_Tension[MOTOR_DD] >= U_MAX) ? U_MAX : Var_Tension[MOTOR_DD];
	Var_Tension[MOTOR_TI] = (Var_Tension[MOTOR_TI] >= U_MAX) ? U_MAX : Var_Tension[MOTOR_TI];
	Var_Tension[MOTOR_TD] = (Var_Tension[MOTOR_TD] >= U_MAX) ? U_MAX : Var_Tension[MOTOR_TD];
	
	duty[MOTOR_DI] = (Var_Tension[MOTOR_DI] != 0) ? Mapeo(Var_Tension[MOTOR_DI],U_MIN,U_MAX,DUTY_MIN,DUTY_MAX) : 65535;
	duty[MOTOR_DD] = (Var_Tension[MOTOR_DD] != 0) ? Mapeo(Var_Tension[MOTOR_DD],U_MIN,U_MAX,DUTY_MIN,DUTY_MAX) : 65535;
	duty[MOTOR_TD] = (Var_Tension[MOTOR_TD] != 0) ? Mapeo(Var_Tension[MOTOR_TD],U_MIN,U_MAX,DUTY_MIN,DUTY_MAX) : 65535;
	duty[MOTOR_TI] = (Var_Tension[MOTOR_TI] != 0) ? Mapeo(Var_Tension[MOTOR_TI],U_MIN,U_MAX,DUTY_MIN,DUTY_MAX) : 65535;
	
	Out_PWM_DD_SetRatio16(duty[MOTOR_DD]);
	Out_PWM_DI_SetRatio16(duty[MOTOR_DI]);
	Out_PWM_TI_SetRatio16(duty[MOTOR_TI]);
	Out_PWM_TD_SetRatio16(duty[MOTOR_TD]);
}
void Reset_PIDs(void){
	  CtrlPID_DD_Reset();
	  CtrlPID_DI_Reset();
	  CtrlPID_TD_Reset();
	  CtrlPID_TI_Reset();
}
void RX(void){
	//####### DECODIFICAR SE�AL
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
					duty_entrada[MOTOR_DI] = la_vel_duty[MOTOR_DI];
					duty_entrada[MOTOR_DD] = la_vel_duty[MOTOR_DD];
					duty_entrada[MOTOR_TI] = la_vel_duty[MOTOR_TI];
					duty_entrada[MOTOR_TD] = la_vel_duty[MOTOR_TD];
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
	receptor_ms[VELOCIDAD] = 0;
	receptor_ms[DIRECCION] = 0;
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