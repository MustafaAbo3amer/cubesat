/*
 * dht.h
 *
 *  Created on: Oct 19, 2024
 *      Author: Magico
 */



#ifndef SRC_DHT_H_
#define SRC_DHT_H_

TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart1;

#define DHT22_PORT GPIOB
#define DHT22_PIN GPIO_PIN_9
uint8_t RH1, RH2, TC1, TC2, SUM, CHECK;
uint32_t pMillis, cMillis;
int tCelsius = 0;
int tFahrenheit = 0;
int RH = 0;


uint8_t DHT22_Start (void)
{
	uint8_t Response = 0;
	GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
	GPIO_InitStructPrivate.Pin = DHT22_PIN;
	GPIO_InitStructPrivate.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(DHT22_PORT, &GPIO_InitStructPrivate); // set the pin as output
	HAL_GPIO_WritePin (DHT22_PORT, DHT22_PIN, 0);   // pull the pin low
	microDelay (1300);   // wait for 1300us
	HAL_GPIO_WritePin (DHT22_PORT, DHT22_PIN, 1);   // pull the pin high
	microDelay (30);   // wait for 30us
	GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(DHT22_PORT, &GPIO_InitStructPrivate); // set the pin as input
	microDelay (40);
	if (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)))
	{
		microDelay (80);
		if ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN))) Response = 1;
	}
	pMillis = HAL_GetTick();
	cMillis = HAL_GetTick();
	while ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)) && pMillis + 2 > cMillis)
	{
		cMillis = HAL_GetTick();
	}
	return Response;
}

uint8_t DHT22_Read (void)
{
	uint8_t a,b;
	for (a=0;a<8;a++)
	{
		pMillis = HAL_GetTick();
		cMillis = HAL_GetTick();
		while (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)) && pMillis + 2 > cMillis)
		{  // wait for the pin to go high
			cMillis = HAL_GetTick();
		}
		microDelay (40);   // wait for 40 us
		if (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)))   // if the pin is low
			b&= ~(1<<(7-a));
		else
			b|= (1<<(7-a));
		pMillis = HAL_GetTick();
		cMillis = HAL_GetTick();
		while ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)) && pMillis + 2 > cMillis)
		{  // wait for the pin to go low
			cMillis = HAL_GetTick();
		}
	}
	return b;
}

void send_messages(char* string)
{
	HAL_UART_Transmit(&huart1, (uint8_t*) string, strlen(string), HAL_MAX_DELAY);
}
#endif /* SRC_DHT_H_ */
