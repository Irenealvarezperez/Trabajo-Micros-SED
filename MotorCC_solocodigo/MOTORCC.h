#ifndef _MOTORCC_H
#define _MOTORCC_H
#include <MOTORCC.h>
#include <sys.h>


// Defina el registro del temporizador para facilitar la expansión y modificación del programa
#define LF1_CCR TIM2->CCR1
#define LF2_CCR TIM2->CCR2

#define RF1_CCR TIM2->CCR3
#define RF2_CCR TIM2->CCR4

#define LR1_CCR TIM3->CCR1
#define LR2_CCR TIM3->CCR2

#define RR1_CCR TIM3->CCR3
#define RR2_CCR TIM3->CCR4

 // Definir la función de pin multiplexado
#define LF_GPIO_AF GPIO_AF1_TIM2
#define RF_GPIO_AF GPIO_AF1_TIM2
#define LR_GPIO_AF GPIO_AF2_TIM3
#define RR_GPIO_AF GPIO_AF2_TIM3

 // define el temporizador
#define LF_TIM htim2
#define RF_TIM htim2
#define LR_TIM htim3
#define RR_TIM htim3

 // define el canal del temporizador
#define LF1_TIM_CHANNEL TIM_CHANNEL_1
#define LF2_TIM_CHANNEL TIM_CHANNEL_2
#define RF1_TIM_CHANNEL TIM_CHANNEL_3
#define RF2_TIM_CHANNEL TIM_CHANNEL_4

#define LR1_TIM_CHANNEL TIM_CHANNEL_1
#define LR2_TIM_CHANNEL TIM_CHANNEL_2
#define RR1_TIM_CHANNEL TIM_CHANNEL_3
#define RR2_TIM_CHANNEL TIM_CHANNEL_4
 // Definir la operación de inicialización de pin
#define MOTOR_LF_F initGPIO_AF(MOTOR_LF1_GPIO_Port,MOTOR_LF1_Pin,LF_GPIO_AF);initGPIO_OUTPUT(MOTOR_LF2_GPIO_Port,MOTOR_LF2_Pin);
#define MOTOR_LF_B initGPIO_OUTPUT(MOTOR_LF1_GPIO_Port,MOTOR_LF1_Pin);initGPIO_AF(MOTOR_LF2_GPIO_Port,MOTOR_LF2_Pin,LF_GPIO_AF);
#define MOTOR_LF_S initGPIO_OUTPUT(MOTOR_LF1_GPIO_Port,MOTOR_LF1_Pin);initGPIO_OUTPUT(MOTOR_LF2_GPIO_Port,MOTOR_LF2_Pin);

#define MOTOR_RF_F initGPIO_AF(MOTOR_RF1_GPIO_Port,MOTOR_RF1_Pin,RF_GPIO_AF);initGPIO_OUTPUT(MOTOR_RF2_GPIO_Port,MOTOR_RF2_Pin);
#define MOTOR_RF_B initGPIO_OUTPUT(MOTOR_RF1_GPIO_Port,MOTOR_RF1_Pin);initGPIO_AF(MOTOR_RF2_GPIO_Port,MOTOR_RF2_Pin,RF_GPIO_AF);
#define MOTOR_RF_S initGPIO_OUTPUT(MOTOR_RF1_GPIO_Port,MOTOR_RF1_Pin);initGPIO_OUTPUT(MOTOR_RF2_GPIO_Port,MOTOR_RF2_Pin);

#define MOTOR_LR_F initGPIO_AF(MOTOR_LR1_GPIO_Port,MOTOR_LR1_Pin,LR_GPIO_AF);initGPIO_OUTPUT(MOTOR_LR2_GPIO_Port,MOTOR_LR2_Pin);
#define MOTOR_LR_B initGPIO_OUTPUT(MOTOR_LR1_GPIO_Port,MOTOR_LR1_Pin);initGPIO_AF(MOTOR_LR2_GPIO_Port,MOTOR_LR2_Pin,LR_GPIO_AF);
#define MOTOR_LR_S initGPIO_OUTPUT(MOTOR_LR1_GPIO_Port,MOTOR_LR1_Pin);initGPIO_OUTPUT(MOTOR_LR2_GPIO_Port,MOTOR_LR2_Pin);

#define MOTOR_RR_F initGPIO_AF(MOTOR_RR1_GPIO_Port,MOTOR_RR1_Pin,RR_GPIO_AF);initGPIO_OUTPUT(MOTOR_RR2_GPIO_Port,MOTOR_RR2_Pin);
#define MOTOR_RR_B initGPIO_OUTPUT(MOTOR_RR1_GPIO_Port,MOTOR_RR1_Pin);initGPIO_AF(MOTOR_RR2_GPIO_Port,MOTOR_RR2_Pin,RR_GPIO_AF);
#define MOTOR_RR_S initGPIO_OUTPUT(MOTOR_RR1_GPIO_Port,MOTOR_RR1_Pin);initGPIO_OUTPUT(MOTOR_RR2_GPIO_Port,MOTOR_RR2_Pin);

 // Introduce una instancia de temporizador desde main.c
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

 // ********* Definición de la función **************** /

 // Detener la salida de todos los canales PWM
void Motor_TIM_PWM_Stop(void);
 // Establecer la dirección hacia adelante del motor para avanzar
void MotorTurnForward(void);
 // Establecer la dirección de avance del motor hacia atrás
void MotorTurnBackward(void);
 // Establecer la dirección del motor hacia la izquierda
void MotorTurnLeft(void);
 // Coloca el motor hacia adelante a la derecha
void MotorTurnRight(void);
 // Establecer la dirección del motor hacia la izquierda
void MotorTurnLF(void);
 // Coloca el motor hacia adelante a la derecha
void MotorTurnRF(void);
 // Ajuste la dirección del motor hacia la izquierda y hacia atrás
void MotorTurnLR(void);
 // Ajuste la dirección del motor hacia la derecha y hacia atrás
void MotorTurnRR(void);
 // Coche parado
void MotorStop(void);

 // Establezca la velocidad del motor estableciendo el valor de comparación
void MotorForward(u32 valLeftFront,u32 valRightFront,u32 valLeftRear,u32 valRightRear);
void MotorBack(u32 valLeftFront,u32 valRightFront,u32 valLeftRear,u32 valRightRear);
void MotorLeft(u32 valLeftFront,u32 valRightFront,u32 valLeftRear,u32 valRightRear);
void MotorRight(u32 valLeftFront,u32 valRightFront,u32 valLeftRear,u32 valRightRear);
void MotorLF(u32 valRF,u32 valLR);
void MotorRF(u32 valLF,u32 valRR);
void MotorLR(u32 valLF,u32 valRR);
void MotorRR(u32 valRF,u32 valLR);

 // inicialización de pin
void initGPIO_OUTPUT(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin);
void initGPIO_AF(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin,uint8_t GPIO_AF);

 // función de prueba
void MotorTest(void);

#endif
