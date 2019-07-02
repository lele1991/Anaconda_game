/*
 * sound.c
 *
 *  Created on: 04 de junho de 2019
 *      Author: Letícia de Oliveira Nunes
 */
#include "../Inc/sound.h"
#include "atraso.h"
#include "stm32f1xx_hal.h"

extern TIM_HandleTypeDef htim3;

void buzz(uint32_t pitch, uint32_t tempo)
{
	__HAL_TIM_SET_PRESCALER(&htim3, pitch);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	vTaskDelay(250/portTICK_RATE_MS);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
}

const uint32_t Perdeu_melody[] = {
		C5, G4, AS4, A4,
};

const uint32_t Perdeu_tempo[] = {
		4, 8, 4, 8,
};

const uint32_t Tetris_melody[] = {

		E5, B4, C5, D5, C5, B4,
		A4, A4, C5, E5, D5, C5,
		B4, C5, D5, E5,
		C5, A4, A4, A4, B4, C5,

		D5, F5, A5, G5, F5,
		E5, C5, E5, D5, C5,
		B4, B4, C5, D5, E5,
		C5, A4, A4, 0,

		E5, B4, C5, D5, C5, B4,
		A4, A4, C5, E5, D5, C5,
		B4, C5, D5, E5,
		C5, A4, A4, A4, B4, C5,

		D5, F5, A5, G5, F5,
		E5, C5, E5, D5, C5,
		B4, B4, C5, D5, E5,
		C5, A4, A4, 0,

		E5, C5,
		D5, B4,
		C5, A4,
		GS4, B4, 0,
		E5, C5,
		D5, B4,
		C5,4, E5,4, A5,2,
		GS5,2,

};

const uint32_t Tetris_tempo[] = {
		4, 8, 8, 4, 8, 8,
		4, 8, 8, 4, 8, 8,
		-4, 8, 4, 4,
		4, 8, 8, 4, 8, 8,

		-4, 8, 4, 8, 8,
		-4, 8, 4, 8, 8,
		4, 8, 8, 4, 4,
		4, 4, 4, 4,

		4, 8, 8, 4, 8, 8,
		4, 8, 8, 4, 8, 8,
		-4, 8, 4, 4,
		4, 8, 8, 4, 8, 8,

		-4, 8, 4, 8, 8,
		-4, 8, 4, 8, 8,
		4, 8, 8, 4, 4,
		4, 4, 4, 4,

		2, 2,
		2, 2,
		2, 2,
		2, 4, 8,
		2, 2,
		2, 2,
		4, 4, 2,
		2,

};


const uint32_t Pantera_cor_de_Rosa_melody[] = {

		0, 0, 0, DS4,
		E4, 0, FS4, G4, 0, DS4,
		E4, FS4, G4, C5, B4, E4, G4, B4,
		AS4, A4, G4, E4, D4,
		E4, 0, 0, DS4,

		E4, 0, FS4, G4, 0, DS4,
		E4, FS4, G4, C5, B4, G4, B4, E5,
		DS5,
		D5, 0, 0, DS4,
		E4, 0, FS4, G4, 0, DS4,
		E4, FS4, G4, C5, B4, E4, G4, B4,

		AS4, A4, G4, E4, D4,
		E4, 0,
		0, E5, D5, B4, A4, G4, E4,
		AS4, A4, AS4, A4, AS4, A4, AS4, A4,
		G4, E4, D4, E4, E4, E4,
};

const uint32_t Pantera_cor_de_Rosa_tempo[] = {
		2, 4, 8, 8,
		-4, 8, 8, -4, 8, 8,
		-8, 8, -8, 8, -8, 8, -8, 8,
		2, -16, -16, -16, -16,
		2, 4, 8, 4,

		-4, 8, 8, -4, 8, 8,
		-8, 8, -8, 8, -8, 8, -8, 8,
		1,
		2, 4, 8, 8,
		-4, 8, 8, -4, 8, 8,
		-8, 8, -8, 8, -8, 8, -8, 8,

		2, -16, -16, -16, -16,
		-4, 4,
		4, -8, 8, -8, 8, -8, -8,
		16, -8, 16, -8, 16, -8, 16, -8
		-16, -16, -16, 16, 16, 2,
};

const uint32_t Hedwigs_melody[] = {
		0, D4,
		G4, AS4, A4,
		G4, D5, 4,
		C5,
		A4,
		G4, AS4, A4,
		F4, GS4,
		D4,
		D4,

		G4, AS4, A4,
		G4, D5,
		F5, E5,
		DS5, B4,
		DS5, D5, CS5,
		CS4, B4,
		G4,
		AS4,

		D5, AS4,
		D5, AS4,
		DS5, D5,
		CS5, A4,
		AS4, D5, CS5,
		CS4, D4,
		D5,
		0, AS4,

		D5, AS4,
		D5, AS4,
		F5, E5,
		DS5, B4,
		DS5, D5, CS5,
		CS4, AS4,
		G4,
};


const uint32_t Hedwigs_tempo[] = {
		2, 4,
		-4, 8, 4,
		2, 4,
		-2,
		-2,
		-4, 8, 4,
		2, 4,
		-1,
		4,

		-4, 8, 4,
		2, 4,
		2, 4,
		2, 4,
		-4, 8, 4,
		2, 4,
		-1,
		4,

		2, 4,
		2, 4,
		2, 4,
		2, 4,
		-4, 8, 4,
		2, 4,
		-1,
		4, 4,

		2, 4,
		2, 4,
		2, 4,
		2, 4,
		-4, 8, 4,
		2, 4,
		-1,
};

const uint32_t Pulo_da_gaita_melody[] = {
		C5, G4, AS4, A4,
		G4, C4, C4, G4, G4, G4,
		C5, G4, AS4, A4,
		G4,

		C5, G4, AS4, A4,
		G4, C4, C4, G4, G4, G4,
		F4, E4, D4, C4,
		C4,

		C5, G4, AS4, A4,
		G4, C4, C4, G4, G4, G4,
		C5, G4, AS4, A4,
		G4,

		C5, G4, AS4, A4,
		G4, C4, C4, G4, G4, G4,
		F4, E4, D4, C4,
		C4, D5, D5, D5, D5, D5,

		D5, D5, D5, C5, E5,
		C5, C5, E5, E5, C5,
		F5, D5, D5, E5,
		C5, D5, E5, D5, C5,

		F5, F5, A5, G5,//21
		G5, C5, C5, C5, C5,
		F5, E5, D5, C5,
		C5, C5, C5, C5,

		F5, F5, A5, G5,//21
		G5, C5, C5, C5, C5,
		F5, E5, D5, C5, E5,
		C5, D5, E5, D5, C5,

		F5, F5, A5, G5, //29
		G5, C5, C5, C5, C5,
		F5, E5, D5, C5,
		C5, G4, AS4, A4,

		G4, C4,C4, G4, G4, G4,
		C5, G4, AS4, A4,
		G4,
		C5, G4, AS4, A4,

		G4, C4, C4, G4, G4, G4,
		F4, E4, D4, C4,
		C5, G4, AS4, A4,

		G4, C4, C4, G4, G4, G4,
		C5, G4, AS4, A4,
		G4,
		C5, G4, AS4, A4,

		G4, C4, C4, G4, G4, G4,
		F4, E4, D4, C4,
		C4, C4, C4, E4, E4, E4,
		F4, F4, F4, FS4, FS4, FS4,

		G4, 0, AS4, C5,
};

const uint32_t Pulo_da_gaita_tempo[] = {
		4, 8, 4, 8,
		16, 8, 16, 16, 8, 16,
		4, 8, 4, 8,
		2,

		4, 8, 4, 8,
		16, 8, 16, 16, 8, 16,
		8, 8, 8, 8,
		2,

		4, 8, 4, 8,
		16, 8, 16, 16, 8, 16,
		4, 8, 4, 8,
		2,

		4, 8, 4, 8,
		16, 8, 16, 16, 8, 16,
		8, 8, 8, 8,
		16, 8, 16, 16, 8, 16,

		16, 8, 16, 8, -8,
		8, 16, 16, 8, 16,
		8, 8, 8, -8,
		8, 16, 16, 8, 16,

		8, 8, 8, -8,
		8, 16, 16, 8, 16,
		-8, 16, 8, 4,
		16, 16, 16, 16,

		8, 16, 8, -8,
		8, 16, 16, 8, 16,
		16, 8, 16, 8, -8,
		8, 16, 16, 8, 16,

		8, 16, 8, -8,
		8, 16, 16, 8, 16,
		8, 16, 8, 8,
		4, 8, 4, 8,

		16, 8, 16, 16, 8, 16,
		4, 8, 4, 8,
		2,
		4, 8, 4, 8,

		16, 8, 16, 16, 8, 16,
		8, 8, 8, -2,
		4, 8, 4, 8,

		16, 8, 16, 16, 8, 16,
		4, 8, 4, 8,
		2,
		4, 8, 4, 8,

		16, 8, 16, 16, 8, 16,
		8, 8, 8, -2,
		16, 8, 16, 16, 8, 16,
		16, 8, 16, 16, 8, 16,

		8, 8, 8, 1,
};

