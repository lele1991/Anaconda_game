/*
 * sound.h
 *
 *  Created on: 04 de junho de 2019
 *      Author: Let�cia de Oliveira Nunes
 */
#include "cmsis_os.h"
#include "main.h"

#ifndef SOUND_H_
#define SOUND_H_

#define	C0		34403
#define	CS0 	32476
#define	Db0 �	32476
#define	D0		30653
#define	DS0 �	28919
#define	Eb0 �	28919
#define	E0		27305
#define	F0		25766
#define	FS0 �	24329
#define	Gb0 �	24329
#define	G0		22958
#define	GS0 �	21667
#define	Ab0 �	21667
#define	A0		20454
#define	AS0 �	19302
#define	Bb0 �	19302
#define	B0		18221
#define	C1		17201
#define	CS1 �	16233
#define	Db1 �	16233
#define	D1		15322
#define	DS1 �	14463
#define	Eb1 �	14463
#define	E1		13652
#define	F1		12886
#define	FS1 �	12161
#define	Gb1 �	12161
#define	G1		11479
#define	GS1 �	10835
#define	Ab1 �	10835
#define	A1		10226
#define	AS1 �	9652
#define	Bb1 �	9652
#define	B1		9110
#define	C2		8599
#define	CS2 �	8116
#define	Db2 �	8116
#define	D2		7660
#define	DS2 �	7231
#define	Eb2 �	7231
#define	E2		6825
#define	F2		6442
#define	FS2 �	6080
#define	Gb2 �	6080
#define	G2		5739
#define	GS2 �	5417
#define	Ab2 �	5417
#define	A2		5113
#define	AS2 �	4826
#define	Bb2 �	4826
#define	B2		4555
#define	C3		4299
#define	CS3 �	4058
#define	Db3 �	4058
#define	D3		3830
#define	DS3 �	3615
#define	Eb3 �	3615
#define	E3		3412
#define	F3		3220
#define	FS3 �	3040
#define	Gb3 �	3040
#define	G3		2869
#define	GS3 �	2708
#define	Ab3 �	2708
#define	A3		2556
#define	AS3 �	2412
#define	Bb3 �	2412
#define	B3		2277
#define	C4		2149
#define CS4		2028
#define	Db4 �	2028
#define	D4		1914
#define DS4		1807
#define Eb4		1807
#define	E4		1705
#define	F4		1610
#define FS4		1519
#define	Gb4		1519
#define	G4		1434
#define GS4		1353
#define	Ab4		1353
#define	A4		1277
#define AS4 	1206
#define	Bb4 �	1206
#define	B4		1138
#define	C5		1074
#define CS5		1014
#define	Db5 �	1014
#define	D5		957
#define DS5		903
#define	Eb5 �	903
#define	E5		852
#define	F5		804
#define	FS5 �	759
#define	Gb5 �	759
#define	G5		716
#define GS5		676
#define	Ab5 �	676
#define	A5		638
#define	AS5 �	602
#define	Bb5 �	602
#define	B5		568
#define	C6		537
#define	CS6 �	506
#define	Db6 �	506
#define	D6		478
#define	DS6 �	451
#define	Eb6 �	451
#define	E6		426
#define	F6		402
#define	FS6 �	379
#define	Gb6 �	379
#define	G6		358
#define	GS6 �	338
#define	Ab6 �	338
#define	A6		319
#define AS6		301
#define	Bb6 �	301
#define	B6		284
#define	C7		268
#define	CS7 �	253
#define	Db7 �	253
#define	D7		238
#define	DS7 �	225
#define	Eb7 �	225
#define	E7		212
#define	F7		200
#define	FS7 �	189
#define	Gb7 �	189
#define	G7		178
#define	GS7 �	168
#define	Ab7 �	168
#define	A7		159
#define	AS7 �	150
#define	Bb7 �	150
#define	B7		141
#define	C8		133
#define	CS8 �	126
#define	Db8 �	126
#define	D8		119
#define	DS8 �	112
#define	Eb8 �	112
#define	E8		106
#define	F8		100
#define	FS8 �	94
#define	Gb8 �	94
#define	G8		89
#define	GS8 �	84
#define	Ab8 �	84
#define	A8		79
#define	AS8 �	74
#define	Bb8 �	74
#define	B8		70

void buzz(uint32_t pitch, uint32_t tempo);

#endif /* SOUND_H_ */
