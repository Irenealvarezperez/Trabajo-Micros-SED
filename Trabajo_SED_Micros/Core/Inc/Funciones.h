#include "main.h"

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

void delay (uint16_t time) //función para hacer el delay en el ultrasonidos. Cambiar por hilos????
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER (&htim1) < time);
}

//Variables alarma
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint8_t Distance  = 0;
uint32_t zumb=200;
uint32_t tiempo_alarma=0;
int sonando=0;

//Variables puerta
uint32_t espera_puerta;
int abierto=0, bloqueo=1, abriendo=0, cerrando=0;
uint32_t mov;
volatile int boton1=0,boton2=0,boton3=0,boton4=0;

//variables LDR
uint32_t LDR_val;

//variables Bluetooth
char readBuf[1];

//variables persiana
int subiendo=0, bajada=0, bajando=0;
uint32_t tiempo_motor, tiempo_persiana=3000;

//variables ventilador

int mov_calor=0, mov__frio=0, dando_calor=0, dando_frio=0;
//uint32_t tiempo_motor_ventilador, tiempo_ventilador=5000;

//variables sensor temperatura
uint32_t sensorTemp_val;

int debouncer2(volatile int* button_int, GPIO_TypeDef* GPIO_port, uint16_t GPIO_number){
	static uint8_t cuenta_boton=0;
	static int cuenta=0;

	if (*button_int==1){
		if (cuenta_boton==0) {
			cuenta=HAL_GetTick();
			cuenta_boton++;
		}
		if (HAL_GetTick()-cuenta>=20){
			cuenta=HAL_GetTick();
			if (HAL_GPIO_ReadPin(GPIO_port, GPIO_number)!=1){
				cuenta_boton=1;
			}
			else{
				cuenta_boton++;
			}
			if (cuenta_boton==3){ //Periodo antirebotes
				cuenta_boton=0;
				*button_int=0;
				return 1;
			}
		}

	}
	return 0;
}


void servo(TIM_HandleTypeDef* htim, int grados){
	 const int MAX=20; //comprobar qe pasa si cambio esto por 13.9 ms. Frecuencia a 72 Hz
	 float ms= grados/90.0f +0.5f;
	 float ciclo = ms/(float)MAX;
	 mov =htim->Instance->ARR*ciclo;
	 htim->Instance->CCR1 = mov;
}

void puerta(void)  //PUERTA
{
	if((debouncer2(&boton3,GPIOA,GPIO_PIN_0))==1||readBuf[0]==50) //si pulsamos el botón de desbloqueo o mandamos la orden desde la aplicación
	{
		  if(bloqueo==1 && abierto==0) //en caso de que la puerta este bloqueada y cerrada
		 {
			 bloqueo=0; //se desbloquea
		   }
		 else if (bloqueo==0 && abierto==0) //si esta cerrada y desbloqueada
		 {
			 bloqueo=1; //se bloquea
		 }
		  readBuf[0]=0; //pongo a cero la variable que recibe el valor del bluetooth
	}


	if ((debouncer2(&boton4,GPIOA,GPIO_PIN_1))==1||readBuf[0]==51) //si pulso el botón de apertura o mando la orden desde la app
	{
		 if(abierto==1) //si está abierta
		 {
			 //abierto=0;
			 espera_puerta=0;//pongo el tiempo de espera a 0
			 cerrando=1; //activo el flag que indica que voy a cerrar la puerta
		 }
		 else
		 {
			  //abierto=1;
			  abriendo=1; //activo el flag que indica que voy a abrir la puerta

		 }
		 readBuf[0]=0; //pongo a cero la variable que recibe el valor del bluetooth
	}

	 if(abierto==0 && bloqueo==0 && abriendo==1) //Si está cerrada, no bloqueada y el flag de apertura activado
	 {
			 servo(&htim2, 180); //pongo el servo a cero grados(posición de la puerta abierta)
			 abierto=1; //indico que ya está abierta la puerta
		 	 espera_puerta = HAL_GetTick(); //tomo el tiempo actual
		 	 abriendo=0; //pongo el flag de apertura a 0
	 }
	if(HAL_GetTick()-espera_puerta > 10000  &&  abierto==1 && cerrando==0) //si han pasado 10s y está abierta, la cierro y la bloqueo
		 {
	//		bloqueo=1;
			espera_puerta=0; //reseteo el tiempo de espera
			cerrando = 1; //indico que quiero cerrar la puerta
		 }

	if(abierto==1 && bloqueo==0 && cerrando==1) //Si está abierta, no bloqueada y quiero cerrarla
	{
		servo(&htim2, 0);//ordeno al servo la posición de la puerta cerrada
		 abierto=0; //indico que está cerrada
	 	 espera_puerta = 0; //reseteo el tiempo
	 	 cerrando=0; //pongo el flag de ciere a 0
	 	 bloqueo=1; //bloqueo la puerta
	}
	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, bloqueo); //control de la luz. ENCENDIDA->Bloqueada
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) //Callback de los botone
{
	if(GPIO_Pin == GPIO_PIN_0)//Botón de bloqueo
	{
		boton3=1;
	}
	if(GPIO_Pin == GPIO_PIN_1)//Botón de apertura de la puerta
	{
		boton4=1;
	}
	if(GPIO_Pin == GPIO_PIN_4)//Bloqueo de desactivación de la alarma
	{
		boton2=1;
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) //Callback para la medición del ultrasonidos
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
	{
		if (Is_First_Captured==0) // if the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured==1)   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			Distance = Difference * .034/2;
			//Distance = Difference;
			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
		}
	}
}

void HCSR04_Read (void) //Función de lectura del ultrasonidos
{
	//enviamos un pulso en el pin TRIG
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); // ponemos el pin TRIG on
	delay(10);  // esperamos 10 us
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);// ponemos el pin TRIG off

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1); //Habilitamos las interrupciones para esperar a la recepción
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	if (hadc->Instance==ADC1)
		LDR_val=HAL_ADC_GetValue(&hadc1); //guardamos el valor medido en LDR_val
	if (hadc->Instance==ADC2)
		sensorTemp_val=HAL_ADC_GetValue(&hadc2);
}

void LDR(void) //función de lectura del LDR
{
	HAL_ADC_Start_IT(&hadc1);
	if(LDR_val<60) //en caso de que el valor sea menor a 60 (luz ambiente)
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,1); //Encendemos la luz
	else
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,0); //Apagamos la luz
}

void temperatura(void) //Función para leer la temperatura
{
	HAL_ADC_Start_IT(&hadc2);
	if(sensorTemp_val>10) //si la temperatura es mayor de 10 grados
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,1);
	else
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,0);
}

void bajaPersiana(int s) //Función de bajada de la persiana
{
	__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, s);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,GPIO_PIN_SET); //Giro horario
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,GPIO_PIN_RESET);
}

void subePersiana(int s) //Función para bajar la persiana
{
	__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, s);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,GPIO_PIN_SET); //Giro antihorario
}

void pareMotor() //Función que para el motor
{
	__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,GPIO_PIN_RESET);
}

void persianas(){ //Función del control completo de la persiana

	if(bajada==0){ //Si está subida
		if(bajando==0){ //Y no se está subiendo
			if(readBuf[0]==52||LDR_val<60){ //Si detecta que pido desde la aplicación que baje
				bajaPersiana(5000); //bajo la persiana
				tiempo_motor=HAL_GetTick(); //cojo el tiempo
				bajando=1; //pongo el flag de subiendo a 1
			}
		}
		else if(bajando==1){
			if(HAL_GetTick()-tiempo_motor>tiempo_persiana){ //si ya ha llegado abajo la persiana
			pareMotor(); //paro el motor
			bajando=0; //pongo el flag de bajando a 0
			bajada=1;   //declaro que ya esta bajada
			readBuf[0]=0; //reseteo la variable de recepción del bluetooth
			}
		}
	}
	else if(bajada==1){ //si esta bajada
		if(subiendo==0){ //no se está subiendo aun
			if(readBuf[0]==52){  //las persianas se suben al pedirlo desde el movil o al bajar la
				subePersiana(5000);			// luminosidad (hacerse de noche)
				tiempo_motor=HAL_GetTick();
				subiendo=1; //activo el flag de subiendo
			}
		}
		else if(subiendo==1){ //si está subiendo
			if(HAL_GetTick()-tiempo_motor>tiempo_persiana){ //y ha acabado de subir
			pareMotor(); //paro el motor
			subiendo=0; //reseteo flags
			bajada=0;
			readBuf[0]=0;
			}
		}
	}
}

//Ventilador

void movimientoCalor(int s)
{
	//TIM10->CCR1=s;
	__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, s);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,GPIO_PIN_RESET);
}
void movimientoFrio(int s)
{
	//TIM10->CCR1=s;
	__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, s);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,GPIO_PIN_SET);
}
void pararMovimiento()
{
	//TIM10->CCR1=s;
	__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,GPIO_PIN_RESET);
}


void ventilador(){

	//if(readBuf[0]==53||sensorTemp_val<30){//el ventilador da calor si se pide desde el movil o al subir la temperatura
	if (readBuf[0]==55){ //parar motor desde aplicación
		pararMovimiento();
	}
	else if(readBuf[0]==53 || sensorTemp_val<30){
			movimientoCalor(5000);   // por debajo de 20 ºC
				//tiempo_motor_ventilador=HAL_GetTick();
				dando_calor=1;

		}

	else if(readBuf[0]==54 || sensorTemp_val>190){  //el ventilador da frio si se pide desde el movil o al subir la temperatura
				movimientoFrio(5000);			// por encima de 25 ºC
				//tiempo_motor_ventilador=HAL_GetTick();
				dando_frio=1;

		}
}


void alarma(void){ //Función completa de la alarma
	HCSR04_Read(); //Leemos el valor del ultrasonidos
	HAL_Delay(100);
	if((Distance<10) & (Distance>1)){ //Si la distancia es menor de 10 cm
		tiempo_alarma=HAL_GetTick(); //tomamos el tiempo en ese instante
 		htim4.Instance->CCR1=zumb; //encendemos el zumbador
		sonando=1;
	}
	if(sonando==1){ //si está sonando
		if(HAL_GetTick()-tiempo_alarma>5000||(debouncer2(&boton2,GPIOA,GPIO_PIN_4))==1||readBuf[0]==49){ //si pasan 5 s, pulso el botón, o lo pido desde la app la desactivo
			htim4.Instance->CCR1=0; //paro el zumbador
			tiempo_alarma=0; //reseteo tiempos y flags
			sonando=0;
			readBuf[0]=0;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) { //callback para el bluetooth
 /* Se recibe el caracter y se pide el siguiente*/
// CDC_Transmit_FS(readBuf, 1);
 if(huart->Instance==huart3.Instance)
 HAL_UART_Receive_IT(&huart3, (uint8_t*)readBuf, 1);
}
