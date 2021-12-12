#include "MOTORCC.h"

 // Estructura de inicialización de pin
GPIO_InitTypeDef GPIO_InitStruct_InitMotor;
 // Inicializa el pin como salida normal y ponlo bajo
void initGPIO_OUTPUT(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin)
{
     GPIO_InitStruct_InitMotor.Pin = GPIO_Pin;
           GPIO_InitStruct_InitMotor.Mode = GPIO_MODE_OUTPUT_PP; // Establecer en salida push-pull
           GPIO_InitStruct_InitMotor.Pull = GPIO_PULLDOWN; // Desplegable
           GPIO_InitStruct_InitMotor.Speed ​​= GPIO_SPEED_FREQ_HIGH; // Alta velocidad
     HAL_GPIO_Init(GPIOx, &GPIO_InitStruct_InitMotor);
     HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_RESET);
}
 // El pin de inicialización es una salida multiplexada por temporizador
void initGPIO_AF(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin,uint8_t GPIO_AF)
{
    GPIO_InitStruct_InitMotor.Pin = GPIO_Pin;
         GPIO_InitStruct_InitMotor.Mode = GPIO_MODE_AF_PP; // push-pull multiplexado
    GPIO_InitStruct_InitMotor.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct_InitMotor.Speed = GPIO_SPEED_FREQ_HIGH;
         GPIO_InitStruct_InitMotor.Alternate = GPIO_AF; // multiplexado como temporizador
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct_InitMotor);
}

 // Detener la salida de todos los canales PWM
void Motor_TIM_PWM_Stop(void)
{
	HAL_TIM_PWM_Stop(&LF_TIM,LF1_TIM_CHANNEL);
	HAL_TIM_PWM_Stop(&LF_TIM,LF2_TIM_CHANNEL);
	HAL_TIM_PWM_Stop(&RF_TIM,RF1_TIM_CHANNEL);
	HAL_TIM_PWM_Stop(&RF_TIM,RF2_TIM_CHANNEL);
	HAL_TIM_PWM_Stop(&LR_TIM,LR1_TIM_CHANNEL);
	HAL_TIM_PWM_Stop(&LR_TIM,LR2_TIM_CHANNEL);
	HAL_TIM_PWM_Stop(&RR_TIM,RR1_TIM_CHANNEL);
	HAL_TIM_PWM_Stop(&RR_TIM,RR2_TIM_CHANNEL);
}


 // Establecer el valor de comparación del temporizador para controlar la velocidad de rotación del motor
 // configuración del valor de comparación directa
void MotorForward(u32 valLeftFront,u32 valRightFront,u32 valLeftRear,u32 valRightRear)
{
	LF1_CCR=valLeftFront;

	RF1_CCR=valRightFront;

	LR1_CCR=valLeftRear;

	RR1_CCR=valRightRear;
}
 // Configuración del valor de comparación hacia atrás
void MotorBack(u32 valLeftFront,u32 valRightFront,u32 valLeftRear,u32 valRightRear)
{
	LF2_CCR=valLeftFront;

	RF2_CCR=valRightFront;

	LR2_CCR=valLeftRear;

	RR2_CCR=valRightRear;

}
 // configuración del valor de comparación izquierdo
void MotorLeft(u32 valLeftFront,u32 valRightFront,u32 valLeftRear,u32 valRightRear)
{
	//LF RR back,RF LR forward
	LF2_CCR=valLeftFront;

	RF1_CCR=valRightFront;

	LR1_CCR=valLeftRear;

	RR2_CCR=valRightRear;
}
 // Establecer el valor de comparación a la derecha
void MotorRight(u32 valLeftFront,u32 valRightFront,u32 valLeftRear,u32 valRightRear)
{
	LF1_CCR=valLeftFront;

	RF2_CCR=valRightFront;

	LR2_CCR=valLeftRear;

	RR1_CCR=valRightRear;
}
 // configuración del valor de comparación del frente izquierdo
void MotorLF(u32 valRF,u32 valLR)
{
	RF1_CCR=valRF;
	LR1_CCR=valLR;
}
 // Configuración del valor de comparación frontal derecho
void MotorRF(u32 valLF,u32 valRR)
{
	LF1_CCR=valLF;
	RR1_CCR=valRR;
}
 // Configuración del valor de comparación izquierda izquierda
void MotorLR(u32 valLF,u32 valRR)
{
	LF2_CCR=valLF;
	RR2_CCR=valRR;
}
 // Configuración de valor de comparación derecha derecha
void MotorRR(u32 valRF,u32 valLR)
{
	RF2_CCR=valRF;
	LR2_CCR=valLR;
}
 // Coche parado, establecer el nivel bajo del canal PWM
void MotorStop(void)
{
	Motor_TIM_PWM_Stop();
	MOTOR_LF_S
	MOTOR_RF_S
	MOTOR_LR_S
	MOTOR_RR_S
}
 // Configuración del pin de avance del motor
void MotorTurnForward()
{
      // Primero detenga la salida PWM, luego configure los pines y finalmente encienda la salida PWM correspondiente
	Motor_TIM_PWM_Stop();

	MOTOR_LF_F
	MOTOR_RF_F
	MOTOR_LR_F
	MOTOR_RR_F

	HAL_TIM_PWM_Start(&LF_TIM,LF1_TIM_CHANNEL);
	HAL_TIM_PWM_Start(&RF_TIM,RF1_TIM_CHANNEL);
	HAL_TIM_PWM_Start(&LR_TIM,LR1_TIM_CHANNEL);
	HAL_TIM_PWM_Start(&RR_TIM,RR1_TIM_CHANNEL);
}

void MotorTurnBackward()
{
	Motor_TIM_PWM_Stop();

	MOTOR_LF_B
	MOTOR_RF_B
	MOTOR_LR_B
	MOTOR_RR_B

	HAL_TIM_PWM_Start(&LF_TIM,LF2_TIM_CHANNEL);
	HAL_TIM_PWM_Start(&RF_TIM,RF2_TIM_CHANNEL);
	HAL_TIM_PWM_Start(&LR_TIM,LR2_TIM_CHANNEL);
	HAL_TIM_PWM_Start(&RR_TIM,RR2_TIM_CHANNEL);
}

void MotorTurnLeft()
{
	Motor_TIM_PWM_Stop();

	MOTOR_LF_B
	MOTOR_RF_F
	MOTOR_LR_F
	MOTOR_RR_B

	HAL_TIM_PWM_Start(&LF_TIM,LF2_TIM_CHANNEL);
	HAL_TIM_PWM_Start(&RF_TIM,RF1_TIM_CHANNEL);
	HAL_TIM_PWM_Start(&LR_TIM,LR1_TIM_CHANNEL);
	HAL_TIM_PWM_Start(&RR_TIM,RR2_TIM_CHANNEL);
}


//LF RR forward RF LR backward
void MotorTurnRight()
{
	Motor_TIM_PWM_Stop();

	MOTOR_LF_F
	MOTOR_RF_B
	MOTOR_LR_B
	MOTOR_RR_F

	HAL_TIM_PWM_Start(&LF_TIM,LF1_TIM_CHANNEL);
	HAL_TIM_PWM_Start(&RF_TIM,RF2_TIM_CHANNEL);
	HAL_TIM_PWM_Start(&LR_TIM,LR2_TIM_CHANNEL);
	HAL_TIM_PWM_Start(&RR_TIM,RR1_TIM_CHANNEL);
}

void MotorTurnLF(void)
{
	Motor_TIM_PWM_Stop();

	MOTOR_LF_S
	MOTOR_RF_F
	MOTOR_LR_F
	MOTOR_RR_S
	HAL_TIM_PWM_Start(&RF_TIM,RF1_TIM_CHANNEL);
	HAL_TIM_PWM_Start(&LR_TIM,LR1_TIM_CHANNEL);
}

void MotorTurnRF(void)
{
	Motor_TIM_PWM_Stop();

	MOTOR_LF_F
	MOTOR_RF_S
	MOTOR_LR_S
	MOTOR_RR_F
	HAL_TIM_PWM_Start(&LF_TIM,LF1_TIM_CHANNEL);
	HAL_TIM_PWM_Start(&RR_TIM,RR1_TIM_CHANNEL);
}

void MotorTurnLR(void)
{
	Motor_TIM_PWM_Stop();

	MOTOR_LF_B
	MOTOR_RF_S
	MOTOR_LR_S
	MOTOR_RR_B
	HAL_TIM_PWM_Start(&LF_TIM,LF2_TIM_CHANNEL);
	HAL_TIM_PWM_Start(&RR_TIM,RR2_TIM_CHANNEL);
}

void MotorTurnRR(void)
{
	Motor_TIM_PWM_Stop();

	MOTOR_LF_S
	MOTOR_RF_B
	MOTOR_LR_B
	MOTOR_RR_S
	HAL_TIM_PWM_Start(&RF_TIM,RF2_TIM_CHANNEL);
	HAL_TIM_PWM_Start(&LR_TIM,LR2_TIM_CHANNEL);
}


void MotorTest(void)
{
	MotorTurnForward();
	MotorForward(600,600,600,600);
	HAL_Delay(2000);

	MotorTurnLeft();
	MotorLeft(600,600,600,600);
	HAL_Delay(2000);

	MotorTurnBackward();
	MotorBack(600,600,600,600);
	HAL_Delay(2000);

	MotorTurnRight();
	MotorRight(600,600,600,600);
	HAL_Delay(2000);

	MotorStop();
	HAL_Delay(10000);
}
