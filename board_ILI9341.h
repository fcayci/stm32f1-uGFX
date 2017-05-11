/*
 * This file is subject to the terms of the GFX License. If a copy of
 * the license was not distributed with this file, you can obtain one at:
 *
 *              http://ugfx.org/license.html
 */

/*
 *  I80 - 8-bit parallel bus interface type I
 *  Display backlight control - PWM - PE9 - TIM1 CH1
 *  RST - PE8
 *  RS - PE12
 *  CS - PE15
 *  RD - PE10
 *  WR - PE11
 *  Data RE7:0
 */

#ifndef _GDISP_LLD_BOARD_H
#define _GDISP_LLD_BOARD_H

#include "stm32f107xx.h"

#define RD 10 // Read data
#define WR 11 // Write data
#define RS 12 // Command/data select
#define CS 15 // Chip select
#define RST 8 // Reset

#define SET_RST     SET_BIT(GPIOE->ODR, RST)
#define CLR_RST     CLEAR_BIT(GPIOE->ODR, RST)
#define SET_CS      SET_BIT(GPIOE->ODR, CS)
#define CLR_CS      CLEAR_BIT(GPIOE->ODR, CS)
#define SET_RS      SET_BIT(GPIOE->ODR, RS)
#define CLR_RS      CLEAR_BIT(GPIOE->ODR, RS)
#define SET_WR      SET_BIT(GPIOE->ODR, WR)
#define CLR_WR      CLEAR_BIT(GPIOE->ODR, WR)
#define SET_RD      SET_BIT(GPIOE->ODR, RD)
#define CLR_RD      CLEAR_BIT(GPIOE->ODR, RD)


static GFXINLINE void init_board(GDisplay *g) {
	//(void) g;
	g->board = 0;

	switch(g->controllerdisplay) {
	case 0:  // Set up for Display 0
		RCC->APB2ENR |=  (1 << 0); //  AFIO clock
		RCC->APB2ENR |=  (1 << 6); // GPIOE clock
		RCC->APB2ENR |= (1 << 11); //  TIM1 clock

		AFIO->MAPR |= 0x000000C0; // Remap PE9
		GPIOE->CRL  = 0x22222222;
		GPIOE->CRH  = 0x222222B2; // All output except PE9
		TIM1->CR1 = 0;
		TIM1->PSC = 71;
		TIM1->ARR = 5000;
		TIM1->CCR1 = 3000;
		TIM1->CCMR1 |= 0x0068;
		TIM1->CCER |= 0x0001;
		TIM1->BDTR |= (1 << 15);
		TIM1->CR1 |= (1 << 0);

		// Default states
		SET_RS;
		SET_RD;
		SET_WR;

		break;
	}
}

static GFXINLINE void post_init_board(GDisplay *g) {
	(void) g;
}

static GFXINLINE void setpin_reset(GDisplay *g, bool_t state) {
	(void) g;
	(state == TRUE) ? CLR_RST : SET_RST;
}

static GFXINLINE void set_backlight(GDisplay *g, uint8_t percent) {
	(void) g;
	TIM1->CCR1 = percent * 50; // 50 comes from 5000 / 100 which is the full period
}

static GFXINLINE void acquire_bus(GDisplay *g) {
	(void) g;
	CLR_CS;
}

static GFXINLINE void release_bus(GDisplay *g) {
	(void) g;
	SET_CS;
}

static GFXINLINE void busmode16(GDisplay *g) {
	(void) g;
}

static GFXINLINE void busmode8(GDisplay *g) {
	(void) g;
}

static GFXINLINE void write_index(GDisplay *g, uint8_t index) {
	(void) g;

	GPIOE->ODR &= ~0xFF;
	GPIOE->ODR |= index;

	CLR_RS;
	CLR_WR;
	SET_WR;
	SET_RS;

}

static GFXINLINE void write_data(GDisplay *g, uint8_t data) {
	(void) g;

	GPIOE->ODR &= ~0xFF;
	GPIOE->ODR |= data;

	CLR_WR;
	SET_WR;
}

static GFXINLINE void setreadmode(GDisplay *g) {
	(void) g;
}

static GFXINLINE void setwritemode(GDisplay *g) {
	(void) g;
}

static GFXINLINE uint16_t read_data(GDisplay *g) {
	(void) g;
	return 0;
}

#endif /* _GDISP_LLD_BOARD_H */
