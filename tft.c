/*
 * tft.c
 *
 * Description:
 *  An example project to show how to use the TFT display
 *
 * Setup steps:
 * Get uGFX library with git submodule init
 *  and git submodule update
 *
 * Author: Furkan Cayci
 *
 * Project Setup:
 *   EasyMX Pro V7 board
 *   ILI9341 TFT Display Driver
 *   Cortex-M3 arch
 *   STM32F107 chip
 */

/*************************************************
* Definitions
*************************************************/
#include <stdio.h>
#include "stm32f107xx.h"
#include <gfx.h>

// Function declarations. Add your functions here
void copy_data(void);
void enable_interrupt(IRQn_type IRQn);
void disable_interrupt(IRQn_type IRQn);
void systick_handler(void);
void init_systick(uint32_t s, uint8_t en);
void set_system_clock_to_72Mhz(void);
int main(void);

/*************************************************
* Vector Table
*************************************************/
// Attribute puts table in beginning of .vector section
//   which is the beginning of .text section in the linker script
// Add other vectors in order here
// Vector table can be found on page 197 in RM0008
uint32_t (* const vector_table[])
__attribute__ ((section(".vectors"))) = {
	(uint32_t *) STACKINIT,         /* 0x00 Stack Pointer */
	(uint32_t *) main,              /* 0x04 Reset         */
	0,                              /* 0x08 NMI           */
	0,                              /* 0x0C HardFaullt    */
	0,                              /* 0x10 MemManage     */
	0,                              /* 0x14 BusFault      */
	0,                              /* 0x18 UsageFault    */
	0,                              /* 0x1C Reserved      */
	0,                              /* 0x20 Reserved      */
	0,                              /* 0x24 Reserved      */
	0,                              /* 0x28 Reserved      */
	0,                              /* 0x2C SVCall        */
	0,                              /* 0x30 Debug Monitor */
	0,                              /* 0x34 Reserved      */
	0,                              /* 0x38 PendSV        */
	(uint32_t *) systick_handler,   /* 0x3C SysTick       */
};

static font_t  font;
volatile uint64_t tick_count = 0;
static gdispImage myImage;

/*************************************************
* Copy the data contents from LMA to VMA
*************************************************/
void copy_data(void)
{
	extern char _etext, _sdata, _edata, _sbss, _ebss;
	char *src = &_etext;
	char *dst = &_sdata;

	/* ROM has data at end of text; copy it.  */
	while (dst < &_edata)
		*dst++ = *src++;

	/* Zero bss.  */
	for (dst = &_sbss; dst< &_ebss; dst++)
		*dst = 0;
}

/*
 * Enable given interrupt
 *
 * Each ISER {0-7} holds 32 interrupts. Thus take mod32 of the given interrupt
 *   to choose the ISER number (ISER[0] for IRQn 0-31, and ISER[1] for IRQn 32-63 ..)
 *   Then, enable the given bit on that register based on the remainder of the mod.
 */
void enable_interrupt(IRQn_type IRQn)
{
	NVIC->ISER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));
}

/*
 * Disable given interrupt
 */
void disable_interrupt(IRQn_type IRQn)
{
	NVIC->ICER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));
}

/*
 * SysTick interrupt handler function
 */
void systick_handler(void)
{
	tick_count++;
}

/*
 * Initialize SysTick Timer
 *
 * Since it is set up to run at 1Mhz, an s value of
 * 1Khz needed to make 1 millisecond timer
 */
void init_systick(uint32_t s, uint8_t en)
{
	// SysTick clock source can be set with CTRL register (Bit 2) (pm0056 - page 151)
	// 0: AHB/8
	// 1: Processor clock (AHB)
	SYSTICK->CSR |= (1 << 2); // Processor clock
	// Enable callback
	SYSTICK->CSR |= (en << 1);
	// Load the reload value
	SYSTICK->RVR = s;
	// Set the current value to 0
	SYSTICK->CVR = 0;
	// Enable SysTick
	SYSTICK->CSR |= (1 << 0);
}

void set_system_clock_to_72Mhz(void)
{
	// Necessary wait states for Flash for high speeds
	FLASH->ACR = 0x12;
	// Enable HSE
	RCC->CR |= (1 << 16);
	// Wait untill HSE settles down
	while (!(RCC->CR & (1 << 17)));
	// Set PREDIV2 division factor to 5
	RCC->CFGR2 |= (0b0100 << 4);
	// Set PLL2 multiplication factor to 8
	RCC->CFGR2 |= (0b0110 << 8);
	// Enable PLL2
	RCC->CR |= (1 << 26);
	// Wait untill PLL2 settles down
	while (!(RCC->CR & (1 << 27)));
	// Set PLL2 as PREDIV1 clock source
	RCC->CFGR2 |= (1 << 16);
	// Set PREDIV1 division factor to 5
	RCC->CFGR2 |= (0b0100 << 0);
	// Select Prediv1 as PLL source
	RCC->CFGR |= (1 << 16);
	// Set PLL1 multiplication factor to 9
	RCC->CFGR |= (0b0111 << 18);
	// Set APB1 to 36MHz
	RCC->CFGR |= 1 << 10;
	// Enable PLL
	RCC->CR |= (1 << 24);
	// Wait untill PLL settles down
	while (!(RCC->CR & (1 << 25)));
	// Finally, choose PLL as the system clock
	RCC->CFGR |= (0b10 << 0);
}

systemticks_t gfxSystemTicks(void)
{
	return tick_count;
}

systemticks_t gfxMillisecondsToTicks(delaytime_t ms)
{
	return ms;
}

/*************************************************
* Main code starts from here
*************************************************/
int main(void)
{
	// Copy LMA to VMA for data section
	copy_data();
	set_system_clock_to_72Mhz();
	init_systick(72000, 1); // 1 ms ticks

	// Initialize the screen
	gfxInit();

	// Enable FONTS and TEXTs in gfxconf.h (GDISP_NEED_TEXT and GDISP_INCLUDE_FONT_UI2)
	font = gdispOpenFont("UI2");
	gdispDrawStringBox(0, 0, gdispGetWidth(),  gdispGetHeight(), "Woot!!!", font, White, justifyCenter);
	gdispDrawStringBox(gdispGetWidth()/2, gdispGetHeight()/2, gdispGetWidth()/2,  gdispGetHeight()/2, "*(&^%$#@!)+_            ", font, White, justifyCenter);

	// Enable Circles in gfxconf.h (GDISP_NEED_CIRCLE)
	gdispFillCircle(gdispGetWidth()/2, 100, 50, Green);
	gdispDrawCircle(gdispGetWidth()/2, 240, 10, Red);
	gdispDrawBox(10,10,50,50,Yellow);

	// Wait 10 secs
	uint32_t curTick = gfxSystemTicks();
	while( gfxSystemTicks() < curTick + 10000);

	// Enable IMAGE, BMP, GFILE and ROMFS in gfxconf.h (GDISP_NEED_IMAGE,
	//   GDISP_NEED_IMAGE_BMP, GFX_USE_GFILE and GFILE_NEED_ROMFS)
	// Got the image from demos/modules/gdisp/images directory
	gdispClear(Black);
	gdispImageOpenFile(&myImage, "test-pal8.bmp");
	gdispImageDraw(&myImage, 0, 0, gdispGetWidth(), gdispGetHeight(), 0, 0);
	gdispImageClose(&myImage);

	// Wait 10 secs
	curTick = gfxSystemTicks();
	while( gfxSystemTicks() < curTick + 10000);

	gdispClear(Black);
	gdispDrawStringBox(0, 0, gdispGetWidth(),  gdispGetHeight(), "Do. Or do not. There is no try!...", font, White, justifyCenter);

	while(1)
	{

	}

	// Should never reach here
	return 0;
}
