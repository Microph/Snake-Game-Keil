/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include<string.h>
#include<stdlib.h>
#include<stdio.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

#define PDM_BUFFER_SIZE 20
#define LEAKY_KEEP_RATE 0.95
#define UART_DEBUG_TICK_RATE 100
#define PDM_BLOCK_SIZE_BITS 16
#define LEFT 0
#define RIGHT 1
#define UP 2
#define DOWN 3

//variables for sound
uint8_t initV[2];
uint16_t Istr[1] = {0};

void initSpeaker(){
	
	HAL_UART_Transmit(&huart2, "Loading...\r\n", 12, 500);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 0);
	HAL_Delay(2000);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 1);
	
	HAL_I2S_Transmit (&hi2s3, Istr , 0x10, 10);
	
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 0);

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 1);
	
	initV[0] = 0x00;
  initV[1] = 0x99;
  HAL_I2C_Master_Transmit(&hi2c1, 0x94, initV, 2, 50);
	
  initV[0] = 0x47;
  initV[1] = 0x80;
  HAL_I2C_Master_Transmit(&hi2c1, 0x94, initV, 2, 50);

  initV[0] = 0x32; 
  initV[1] = 0x80;
  HAL_I2C_Master_Transmit(&hi2c1, 0x94, initV, 2, 50);

  initV[0] = 0x32;
  initV[1] = 0x00;
  HAL_I2C_Master_Transmit(&hi2c1, 0x94, initV, 2, 50);

  initV[0] = 0x00;
  initV[1] = 0x00;
  HAL_I2C_Master_Transmit(&hi2c1, 0x94, initV, 2, 50);
	
  initV[0] = 0x1E;
  initV[1] = 0xC0;
  HAL_I2C_Master_Transmit(&hi2c1, 0x94, initV, 2, 50);
		
	initV[0] = 0x02; //You are powered up, get in there
  initV[1] = 0x9E;
  HAL_I2C_Master_Transmit(&hi2c1, 0x94, initV, 2, 50);
}

void playStart(){
		int i, j;
	
	for(i=0; i<2; i++)
	{
		initV[0] = 0x1E;
		initV[1] = 0x20;
		HAL_I2C_Master_Transmit(&hi2c1, 0x94, initV, 2, 50);

		initV[0] = 0x1C;
		if(i==0)
			initV[1] = 0x1A;
		else
			initV[1] = 0x8A;
		HAL_I2C_Master_Transmit(&hi2c1, 0x94, initV, 2, 50);
		
		initV[0] = 0x1E;
		initV[1] = 0xE0;
		HAL_I2C_Master_Transmit(&hi2c1, 0x94, initV, 2, 50);
	
		for (j=0 ; j<1000; j++)
		{
			HAL_I2S_Transmit (&hi2s3, Istr , 0x10, 50);
		}
		
		HAL_Delay(50);
	}
	
}

void playEat(){
	
		int i;
		initV[0] = 0x1E;
		initV[1] = 0x20;
		HAL_I2C_Master_Transmit(&hi2c1, 0x94, initV, 2, 50);

		initV[0] = 0x1C;
		initV[1] = 0xAA;
		HAL_I2C_Master_Transmit(&hi2c1, 0x94, initV, 2, 50);
		
		initV[0] = 0x1E;
		initV[1] = 0xE0;
		HAL_I2C_Master_Transmit(&hi2c1, 0x94, initV, 2, 50);
	
		for (i=0 ; i<500; i++)
		{
			HAL_I2S_Transmit (&hi2s3, Istr , 0x10, 50);
		}
}

void playDie(){
		int i, j;
	for(i=0; i<3; i++)
	{
		initV[0] = 0x1E;
		initV[1] = 0x20;
		HAL_I2C_Master_Transmit(&hi2c1, 0x94, initV, 2, 50);

		initV[0] = 0x1C;
		if(i==0)
			initV[1] = 0x6A;
		else if(i==1)
			initV[1] = 0x4A;
		else
			initV[1] = 0x0A;
		HAL_I2C_Master_Transmit(&hi2c1, 0x94, initV, 2, 50);
		
		initV[0] = 0x1E;
		initV[1] = 0xE0;
		HAL_I2C_Master_Transmit(&hi2c1, 0x94, initV, 2, 50);
	
		for (j=0 ; j<500; j++)
		{
			HAL_I2S_Transmit (&hi2s3, Istr , 0x10, 50);
		}
		
		HAL_Delay(25);
	}
}

	//variables for detecting sound
	uint8_t i;
  uint16_t pdm_buffer[PDM_BUFFER_SIZE];
  uint16_t pdm_value=0;
  uint8_t  pcm_value=0;
  uint16_t pcm_count = 0;

  char uart_temp_display_buffer[100];
  float leaky_pcm_buffer = 0.0;
  float leaky_amp_buffer = 0.0;
  float max_amp = 0;
	int get=0;
	
float float_abs(float in){
	return in < 0 ? -in : in;
}

void detectSound(){
	get=0;
	while(!get)
	{
		HAL_I2S_Receive(&hi2s2, pdm_buffer, PDM_BUFFER_SIZE, 1000);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    for(i=0; i<PDM_BUFFER_SIZE; i++){
      pcm_value = -PDM_BLOCK_SIZE_BITS/2;
      pdm_value = pdm_buffer[i];
      while ( pdm_value != 0 )
      {
        pcm_value ++;
        pdm_value ^= pdm_value & -pdm_value;
      }
      leaky_pcm_buffer += pcm_value;
      leaky_pcm_buffer *= LEAKY_KEEP_RATE;
      leaky_amp_buffer += float_abs(leaky_pcm_buffer);
      leaky_amp_buffer *= LEAKY_KEEP_RATE;
    }
    pcm_count++;
    if(max_amp < leaky_amp_buffer) max_amp = leaky_amp_buffer;
    if(pcm_count == 1000){
    	if((int)max_amp > 31000)
			{
				get = 1; 
				break;
			}
			pcm_count = 0;
    	max_amp = 0;
    }
	}
}
	
//update board vars
	int board[9][32];
	int headX = 3, headY = 4;
	int tailX = 1, tailY = 4;
	int snakes = 1;
	int direction = RIGHT;
	int eaten = 0;
	int score = 0;
	int size=0;

void updateNextTail(){
	if(eaten)
	{
		playEat();
		return;
	}
		
	int temX = tailX, temY = tailY;
	if(board[tailY][tailX+1] == board[tailY][tailX]+1)
		tailX++;
	else if(board[tailY][tailX-1] == board[tailY][tailX]+1)
		tailX--;
	else if(board[tailY+1][tailX] == board[tailY][tailX]+1)
		tailY++;
	else
		tailY--;
	board[temY][temX] = 0;
}

void updateNextHead(){
	if(direction == RIGHT)
		headX++;
	else if(direction == LEFT)
		headX--;
	else if(direction == DOWN)
		headY++;
	else
		headY--;
	
	if(board[headY][headX] == -1)
	{
		eaten = 1;
	}
}

//random number
int rX, rY;

void genFood(){
	while(1)
	{
		rX = rand() % (30 + 1 - 1) + 1;
		rY = rand() % (7 + 1 - 1) + 1;
		if(board[rY][rX] == 0)
		{
			board[rY][rX] = -1;
			break;
		}
	}
}

int updateBoard(){
	int i=0, j=0;
	//update board array and snake behaviors
	updateNextTail();
	if(eaten)
	{
		genFood();
		eaten = 0;
		score++;
	}
	updateNextHead();
	if(board[headY][headX] > 0)
		return 1;
	board[headY][headX] = snakes++;
	
	//clear screen
	HAL_UART_Transmit(&huart2, "\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n", 60, 500);
	
	//draw board
	for(i=0; i<9 ; i++)
	{
		for(j=0; j<32; j++)
		{
			if(i==0 || i==8)
				HAL_UART_Transmit(&huart2, "-", 1, 500);
			else if(j==0 || j==31)
				HAL_UART_Transmit(&huart2, "|", 1, 500);
			else if(board[i][j] > 0)
				HAL_UART_Transmit(&huart2, "o", 1, 500);
			else if(board[i][j] == -1)
				HAL_UART_Transmit(&huart2, "*", 1, 500);
			else
				HAL_UART_Transmit(&huart2, " ", 1, 500);
			
		}
			if(i < 8) HAL_UART_Transmit(&huart2, "\r\n", 2, 500);
	}
	
	sprintf(uart_temp_display_buffer, "\r\nSCORE: %d", score);
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_temp_display_buffer, strlen(uart_temp_display_buffer), 500);
	
	if(headX==0 || headX==31 || headY==0 || headY==8)
		return 1;
	return 0;
}

//accel vars
uint8_t address;
int8_t x,y, uartSize;

void accelInit(){
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
  	  address = 0x20;
  	  HAL_SPI_Transmit(&hspi1,&address,1,50);

  	  uint8_t data = 0x47;
  	  HAL_SPI_Transmit(&hspi1,&data,1,50);
  	  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
}

void accelDet(){
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);

	
	  		  address = 0x29+ 0x80;
	  		  HAL_SPI_Transmit(&hspi1,&address,1,50);
	  		  HAL_SPI_Receive(&hspi1,&x,1,50);

	  		  address = 0x2B+ 0x80;
	  		  HAL_SPI_Transmit(&hspi1,&address,1,50);
	  		  HAL_SPI_Receive(&hspi1,&y,1,50);

	  		  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
	
		if(y>=25) //right
	  {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
			if(direction != LEFT)
				direction = RIGHT;
		}
		else
	  {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
	  }
		if(y<-25) //left
	  {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
			if(direction != RIGHT)
				direction = LEFT;
		}
		else
	  {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
	  }
		if(x<-25) //up
		{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
			if(direction != DOWN)
				direction = UP;
		}
		else
	  {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
	  }
		if(x>=25) //down
		{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
			if(direction != UP)
				direction = DOWN;
		}
		else
	  {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
	  }
		
}

//game start vars
	int die=0;

void gameStart(){
	//initialise board and snake
	int i, j;
	for(i=0; i<9; i++)
	{
		for(j=0; j<32; j++)
		{
			if(i==4 && (j==1 || j==2 || j==3))
				board[i][j] = snakes++;
			else if(i==4 && j== 20)//first food!
				board[i][j] = -1;
			else
				board[i][j] = 0;
		}
	}
	tailY=4; tailX=1; headY=4; headX=3;
	
	//play sound
	playStart();
	
	//gameloop
	while(1)
	{
		die = updateBoard();
		if(die)
		{
			playDie();
			break;
		}
		
		//determine next direction
		accelDet();
		HAL_Delay(500);
	}
}

void gameOver(){
	HAL_UART_Transmit(&huart2, "\r\n\r\n", 4, 500);
	HAL_UART_Transmit(&huart2, "GAME OVER", 9, 500);
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
	
	//initialise speaker
	initSpeaker();
	
	//initialise accel
	accelInit();

	//wait for high volume sound to start game
	sprintf(uart_temp_display_buffer, "  #############  ####      ####      ######      ###     ###  ###########  #########\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)uart_temp_display_buffer, strlen(uart_temp_display_buffer), 100);
	sprintf(uart_temp_display_buffer, " ############    ####      ####    ###    ###    ###    ###   ###          #########\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)uart_temp_display_buffer, strlen(uart_temp_display_buffer), 100);
	sprintf(uart_temp_display_buffer, "####             ####     #####   ###      ###   ###   ###    ###           #######\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)uart_temp_display_buffer, strlen(uart_temp_display_buffer), 100);
	sprintf(uart_temp_display_buffer, "#####            ####   #######  ##############  #######      ###########    #####\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)uart_temp_display_buffer, strlen(uart_temp_display_buffer), 100);
	sprintf(uart_temp_display_buffer, "  ###########    ########   ###  ###        ###  ###  ###     ###             ###\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)uart_temp_display_buffer, strlen(uart_temp_display_buffer), 100);
	sprintf(uart_temp_display_buffer, "     ########    #######    ###  ###        ###  ###   ###    ###              #\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)uart_temp_display_buffer, strlen(uart_temp_display_buffer), 100);
	sprintf(uart_temp_display_buffer, "  #########      #####      ###  ###        ###  ###    ###   ###               \r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)uart_temp_display_buffer, strlen(uart_temp_display_buffer), 100);
	sprintf(uart_temp_display_buffer, "#######          ####       ###  ###        ###  ###     ###  ###########      O\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)uart_temp_display_buffer, strlen(uart_temp_display_buffer), 100);
	sprintf(uart_temp_display_buffer, "*SHOUT TO START*");
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_temp_display_buffer, strlen(uart_temp_display_buffer), 100);
	//wait for sound to start game
	detectSound();
	
	//tabs
	sprintf(uart_temp_display_buffer, "...");
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_temp_display_buffer, strlen(uart_temp_display_buffer), 100);
	
	//game loop
	gameStart();
	
	//show game over
	gameOver();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 88;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 50000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2S2 init function */
static void MX_I2S2_Init(void)
{

  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_44K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2S3 init function */
static void MX_I2S3_Init(void)
{

  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_MSB;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_44K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1680;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA9   ------> USB_OTG_FS_VBUS
     PA10   ------> USB_OTG_FS_ID
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT1_Pin */
  GPIO_InitStruct.Pin = MEMS_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
