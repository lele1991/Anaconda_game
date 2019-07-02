
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "defPrincipais.h"
#include "NOKIA5110_fb.h"
#include "PRNG_LFSR.h"
#include "sound.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;

//osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint32_t ADC_buffer[2];
uint32_t valor_ADC[2];

/* Definicoes da cobra */
#define velocidade 150
uint8_t dir_anaconda = 0; 					//dir = 0 baixo = 1 esq = 2 cima = 3
uint32_t anaconda[2][300]={{0}};			//[x/y][pos]
uint16_t tam_anaconda = 10;
uint32_t semente_jujuba = 1;
uint8_t jujuba[2], jujuba_on=0;							//verificação se há ou não jujuba - 0-> nao tem/ 1-> tem

//SOM
//extern const uint32_t Hedwigs_melody[], Hedwigs_tempo[];
extern const uint32_t Pulo_da_gaita_melody[], Pulo_da_gaita_tempo[];
//extern const uint32_t Tetris_melody[], Tetris_tempo[];
//extern const uint32_t Pantera_cor_de_Rosa_melody[], Pantera_cor_de_Rosa_tempo[];
extern const uint32_t Perdeu_melody[], Perdeu_tempo[];

//TASK

xTaskHandle xTaskSongHandle;


/*-----------------------------------------------------------------------------*/

/* Definicoes da arena*/
//game_state[y][x]
uint8_t game_state[24][42] = { //[coluna][linha] = [y][x]  0 space / 1 arena / 2 snake / 3 head / 4 jujuba/ 5 porta => representam 4 pixels
		{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
		{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
		{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
		{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
		{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
		{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
		{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
		{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
		{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
		{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
		{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
		{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
		{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
		{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
		{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
		{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
};

/*-----------------------------------------------------------------------------*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
//void StartDefaultTask(void const * argument);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	if(hadc->Instance == ADC1){
		valor_ADC[0]=ADC_buffer[0];
		valor_ADC[1]=ADC_buffer[1];
	}
}
//---------------------------------------------------------------------------------------------------
// Tarefa para atualizar periodicamente o LCD
void vTask_LCD_Print(void *pvParameters)
{
	while(1){
		imprime_LCD();
	}
}
//---------------------------------------------------------------------------------------------------

void vTask_Principal(void *pvParamters){
	uint16_t i=0, j=0, entrou=0;						//i - linha j - coluna
	struct pontos_t pixel; 								// desenhar

	while(1){											//primeiro faz a jogada e desenha
		//PARTE DE CONTROLE DA ANACONDA

		/*					CIMA  - 2000/0
		 * ESQ - 4000/2000		|||||		DIR - 0/2000
		 * 		  				  0
		 * 					BAIXO - 2000/4000
		 */

		//direcionando -> dir = 0 baixo = 1 esq = 2 cima = 3
		if(valor_ADC[0]>3500){				//controle for pra esquerda
			if(dir_anaconda == 0)			//se a cobra estiver na direita
				dir_anaconda = 0;			//continua na direita
			else
				dir_anaconda = 2;			//continua pra esquerda
		}
		if(valor_ADC[0]<1800){				//controle for pra direita
			if(dir_anaconda == 2)			//se a cobra estiver na esquerda
				dir_anaconda = 2;			//continua pra esquerda
			else
				dir_anaconda = 0;			//continua pra direita
		}

		if(valor_ADC[1]<1800   ){			//controle for pra cima
			if(dir_anaconda == 1)			//se a cobra estiver pra baixo
				dir_anaconda = 1;			//continua pra baixo
			else
				dir_anaconda = 3;			//continua pra cima
		}
		if(valor_ADC[1]>3500){				//controle for pra baixo
			if(dir_anaconda == 3)			//se a cobra estiver pra cima
				dir_anaconda = 3;			//continua pra cima
			else
				dir_anaconda = 1;			//cobra vai pra baixo
		}
//---------------------------------------------------------------------------------------------------

		//PARTE DE ATUALIZACAO DA ANACONDA (MOVIMENTO)
		//atualiza a cabeça - faz andar
		for(i=tam_anaconda; i>0; i--){ // da cauda até a cabeça
			//atualiza posicao, deslocando pra frente
			anaconda[0][i] = anaconda[0][i-1];
			anaconda[1][i] = anaconda[1][i-1];
		}
		//atualiza cabeça, deslocando pra frente
		if(dir_anaconda == 0){
			anaconda[0][0] = anaconda[0][0]+1;	//dir
		}
		if(dir_anaconda == 1){
			anaconda[1][0] = anaconda[1][0]+1;	//baixo

		}
		if(dir_anaconda == 2){
			anaconda[0][0] = anaconda[0][0]-1;	//esq
		}
		if(dir_anaconda == 3){
			anaconda[1][0] = anaconda[1][0]-1;	//cima
		}
		//---------------------------------------------------------------------------------------------------

		//PARTE DAS PORTAS

		if(anaconda[0][0] == 0 || anaconda[0][0] == 41){		//porta de cima    coluna 0 e coluna 41
			//PORTA DE CIMA
			if(anaconda[1][0] == 1){		//primeiro quadrado da porta pro ultimo
				anaconda[0][0] = 41;
				anaconda[1][0] = 22;
				entrou = 1;
			}
			if(anaconda[1][0] == 2){
				anaconda[0][0] = 41;
				anaconda[1][0] = 21;
				entrou = 1;
			}
			if(anaconda[1][0] == 3){
				anaconda[0][0] = 41;
				anaconda[1][0] = 20;
				entrou = 1;
			}
			if(anaconda[1][0] == 4){
				anaconda[0][0] = 41;
				anaconda[1][0] = 19;
				entrou = 1;
			}
			//PORTA DE BAIXO
			if(entrou == 0){
				if(anaconda[1][0] == 22){		//ultimo quadrado da porta pro primeiro
					anaconda[0][0] = 0;
					anaconda[1][0] = 1;
				}
				if(anaconda[1][0] == 21){
					anaconda[0][0] = 0;
					anaconda[1][0] = 2;
				}
				if(anaconda[1][0] == 20){
					anaconda[0][0] = 0;
					anaconda[1][0] = 3;
				}
				if(anaconda[1][0] == 19){
					anaconda[0][0] = 0;
					anaconda[1][0] = 4;
				}
			}
			entrou = 0;
		}

//---------------------------------------------------------------------------------------------------

		//PARTE DE CRIAÇAO A JUJUBA
		if(jujuba_on == 0){

			do{
				jujuba[0] = prng_LFSR()%40 + 1;								// prng() - um novo numero é gerado ->x de 1 à 40
				jujuba[1] = prng_LFSR()%22 + 1;								// prng() - um novo numero é gerado ->y de 1 à 22

			}while(game_state[jujuba[1]][jujuba[0]] != 0); 					//enquanto nao acha um espaço vago -> Game_S[Y][X]

			game_state[jujuba[1]][jujuba[0]] = 4;
			jujuba_on = 1;
		}
		if(game_state[anaconda[1][0]][anaconda[0][0]] == 4){				//cobra encontra 4
			tam_anaconda++;
			jujuba_on = 0;
		}

//---------------------------------------------------------------------------------------------------

		//PARTE DE CHECAGEM DA COLISÃO DA ANACONDA
		if(game_state[anaconda[1][0]][anaconda[0][0]] == 1 ||						//com a parede
		   game_state[anaconda[1][0]][anaconda[0][0]] == 2){ 						//com o corpo

			vTaskDelay(10/portTICK_RATE_MS);

			vTaskDelete(xTaskSongHandle);
			buzz(A4, 5);
			inverte_LCD();
			buzz(Ab4, 5);
			vTaskDelay(30/portTICK_RATE_MS);
			inverte_LCD();
			buzz(G4, 5);
			vTaskDelay(30/portTICK_RATE_MS);
			inverte_LCD();
			buzz(Gb4, 2);

			limpa_LCD();
			goto_XY(0, 0);
			string_LCD("                                  ");
			string_LCD("GAMEOVER");
			string_LCD("                                           ");
			imprime_LCD();
			vTaskDelay(3000/portTICK_RATE_MS);
			NVIC_SystemReset(); //reseta a porra toda
		}
//---------------------------------------------------------------------------------------------------

		//PARTE DE DESENHO DE TUDO
		//desenha cabeça
		game_state[anaconda[1][0]][anaconda[0][0]] = 3;							//head[x][y]
		//desenha corpo
		for(i = 1; i<tam_anaconda-1; i++){										//body i=1 - ja foi desenhada a cabeça
			game_state[anaconda[1][i]][anaconda[0][i]] = 2;						//anaconda[x/y][pos]
		}

		game_state[anaconda[1][tam_anaconda]][anaconda[0][tam_anaconda]] = 0;	//atualiza e limpa cauda
		limpa_LCD();
		game_state[0][0] = 1; // because reasons
		for(i=0; i<= 23; i++){
			for(j = 0; j<=41; j++){
				//[linha][coluna]  0 space / 1 arena / 2 snake / 3 head / 4 jujuba => representam 4 pixels
				if(game_state[i][j]==1 || game_state[i][j]==2 || game_state[i][j]==3 || game_state[i][j]== 4){
					/*	Funcao para desenho de um retângulo
					*		p1-----------
					*		|			|
					*		------------p2*/
					//esta dividido por dois na matriz geral
					pixel.x1 = j*2;
					pixel.y1 = i*2;
					pixel.x2 = j*2+1;
					pixel.y2 = i*2+1;
					desenha_retangulo(&pixel, 1);
				}
			}
		}
		vTaskDelay(velocidade/portTICK_RATE_MS);
	}
}

void Start(void){
	inic_LCD();
	limpa_LCD();
	goto_XY(0, 0);
	string_LCD("*******************************");
	string_LCD("ANACONDA");
	string_LCD("*********************************************");
	imprime_LCD();
	tam_anaconda = 10;
	jujuba_on  = 0;

/* INICIALIZACAO DA COBRA----------------------------------------------------------------------- */
	uint8_t i=0, j=0;
	for(i=0; i<tam_anaconda;i++){
		//posicao inicial da cabeça da cobra [][0]
		//posicao inicial do corpo [][i]
		anaconda[0][i] = 23-i;				//x
		anaconda[1][i] = 12;				//y
	}
/*-----------------------------------------------------------------------------------------------*/
	for(i = 1; i <= 40; i++){				//pega só o miolo da arena
		for(j = 1; j <= 22; j++){
			game_state[j][i] = 0;
		}
	}
}

static void vTask_Song(void *pvParameters){
	uint8_t i;
	vTaskDelay(500/portTICK_RATE_MS);
	while (1){
		for(i = 0; i < 80; i++){
			buzz(Pulo_da_gaita_melody[i], Pulo_da_gaita_tempo[i]);
			vTaskDelay(10/portTICK_RATE_MS);
		}
	}
}

//---------------------------------------------------------------------------------------------------
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t semente_PRNG=1;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

	//ADC
  	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC_buffer,2);
	HAL_ADC_Start_IT(&hadc1);

	// inicializa LCD 5110
	Start();

	// inicializa tela
	while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15)) 			// enquanto nao pressionar joystick fica travado
	{
		semente_jujuba++;
		semente_PRNG++;										// semente para o gerador de numeros pseudoaleatorios												// pode ser empregado o ADC lendo uma entrada flutuante para gerar a semente.
	}

	xTaskCreate(vTask_Song, "SONG", 100, NULL, 1, &xTaskSongHandle);
	init_LFSR(semente_jujuba);	// inicializacao para geracao de numeros pseudoaleatorios
	init_LFSR(semente_PRNG);	// inicializacao para geracao de numeros pseudoaleatorios
	//rand_prng = prng_LFSR();	// sempre q a funcao prng() for chamada um novo numero é gerado.

	//PWM
	//HAL_TIM_Base_Start(&htim3);
	//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2800);
	//HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);


  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  //osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  //defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */

	xTaskCreate(vTask_LCD_Print, "LCD_Print", 100, NULL, 1, NULL);
	xTaskCreate(vTask_Principal, "GAME_STATE", 100, NULL, 1, NULL);


  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  //osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	 vTaskStartScheduler();	// apos este comando o RTOS passa a executar as tarefas
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 5625;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 128;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 65;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 PA5 PA6 
                           PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* StartDefaultTask function */
//void StartDefaultTask(void const * argument)
//{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
//  for(;;)
//  {
//    osDelay(1);
//  }
  /* USER CODE END 5 */ 
//}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
