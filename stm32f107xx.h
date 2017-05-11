#ifndef _STM32F107XX_H
#define _STM32F107XX_H

#define HSE_Value       ((uint32_t) 25000000) /* Value of the External oscillator in Hz */
#define HSI_Value       ((uint32_t)  8000000) /* Value of the Internal oscillator in Hz*/

// Define the base addresses for peripherals
#define PERIPH_BASE     ((uint32_t) 0x40000000)
#define SRAM_BASE       ((uint32_t) 0x20000000)

#define APB1PERIPH_BASE PERIPH_BASE
#define APB2PERIPH_BASE (PERIPH_BASE + 0x10000)
#define AHBPERIPH_BASE  (PERIPH_BASE + 0x20000)

#define AFIO_BASE       (APB2PERIPH_BASE + 0x0000) //  AFIO base address is 0x40010000
#define GPIOE_BASE      (APB2PERIPH_BASE + 0x1800) // GPIOE base address is 0x40011800
#define GPIOD_BASE      (APB2PERIPH_BASE + 0x1400) // GPIOD base address is 0x40011400
#define TIM1_BASE       (APB2PERIPH_BASE + 0x2C00) //  TIM1 base address is 0x40012C00
#define RCC_BASE        ( AHBPERIPH_BASE + 0x1000) //   RCC base address is 0x40021000
#define FLASH_BASE      ( AHBPERIPH_BASE + 0x2000) // FLASH base address is 0x40022000

#define SYSTICK_BASE    ((uint32_t) 0xE000E010)
#define NVIC_BASE       ((uint32_t) 0xE000E100)

#define STACKINIT       0x2000FF00

/*
 * Register Addresses
 */
typedef struct
{
	uint32_t CRL;      /* GPIO port configuration register low,      Address offset: 0x00 */
	uint32_t CRH;      /* GPIO port configuration register high,     Address offset: 0x04 */
	uint32_t IDR;      /* GPIO port input data register,             Address offset: 0x08 */
	uint32_t ODR;      /* GPIO port output data register,            Address offset: 0x0C */
	uint32_t BSRR;     /* GPIO port bit set/reset register,          Address offset: 0x10 */
	uint32_t BRR;      /* GPIO port bit reset register,              Address offset: 0x14 */
	uint32_t LCKR;     /* GPIO port configuration lock register,     Address offset: 0x18 */
} GPIO_type;

typedef struct
{
	uint32_t CR;       /* RCC clock control register,                Address offset: 0x00 */
	uint32_t CFGR;     /* RCC clock configuration register,          Address offset: 0x04 */
	uint32_t CIR;      /* RCC clock interrupt register,              Address offset: 0x08 */
	uint32_t APB2RSTR; /* RCC APB2 peripheral reset register,        Address offset: 0x0C */
	uint32_t APB1RSTR; /* RCC APB1 peripheral reset register,        Address offset: 0x10 */
	uint32_t AHBENR;   /* RCC AHB peripheral clock enable register,  Address offset: 0x14 */
	uint32_t APB2ENR;  /* RCC APB2 peripheral clock enable register, Address offset: 0x18 */
	uint32_t APB1ENR;  /* RCC APB1 peripheral clock enable register, Address offset: 0x1C */
	uint32_t BDCR;     /* RCC backup domain control register,        Address offset: 0x20 */
	uint32_t CSR;      /* RCC control/status register,               Address offset: 0x24 */
	uint32_t AHBRSTR;  /* RCC AHB peripheral clock reset register,   Address offset: 0x28 */
	uint32_t CFGR2;    /* RCC clock configuration register 2,        Address offset: 0x2C */
} RCC_type;

typedef struct
{
	uint32_t CR1;       /* Address offset: 0x00 */
	uint32_t CR2;       /* Address offset: 0x04 */
	uint32_t SMCR;      /* Address offset: 0x08 */
	uint32_t DIER;      /* Address offset: 0x0C */
	uint32_t SR;        /* Address offset: 0x10 */
	uint32_t EGR;       /* Address offset: 0x14 */
	uint32_t CCMR1;     /* Address offset: 0x18 */
	uint32_t CCMR2;     /* Address offset: 0x1C */
	uint32_t CCER;      /* Address offset: 0x20 */
	uint32_t CNT;       /* Address offset: 0x24 */
	uint32_t PSC;       /* Address offset: 0x28 */
	uint32_t ARR;       /* Address offset: 0x2C */
	uint32_t RES1;      /* Address offset: 0x30 */
	uint32_t CCR1;      /* Address offset: 0x34 */
	uint32_t CCR2;      /* Address offset: 0x38 */
	uint32_t CCR3;      /* Address offset: 0x3C */
	uint32_t CCR4;      /* Address offset: 0x40 */
	uint32_t BDTR;      /* Address offset: 0x44 */
	uint32_t DCR;       /* Address offset: 0x48 */
	uint32_t DMAR;      /* Address offset: 0x4C */
} TIM_type;

typedef struct
{
	uint32_t ACR;
	uint32_t KEYR;
	uint32_t OPTKEYR;
	uint32_t SR;
	uint32_t CR;
	uint32_t AR;
	uint32_t RESERVED;
	uint32_t OBR;
	uint32_t WRPR;
} FLASH_type;

typedef struct
{
	uint32_t EVCR;      /* Address offset: 0x00 */
	uint32_t MAPR;      /* Address offset: 0x04 */
	uint32_t EXTICR1;   /* Address offset: 0x08 */
	uint32_t EXTICR2;   /* Address offset: 0x0C */
	uint32_t EXTICR3;   /* Address offset: 0x10 */
	uint32_t EXTICR4;   /* Address offset: 0x14 */
	uint32_t MAPR2;     /* Address offset: 0x18 */
} AFIO_type;

typedef struct
{
	uint32_t   ISER[8];     /* Address offset: 0x000 - 0x01C */
	uint32_t  RES0[24];     /* Address offset: 0x020 - 0x07C */
	uint32_t   ICER[8];     /* Address offset: 0x080 - 0x09C */
	uint32_t  RES1[24];     /* Address offset: 0x0A0 - 0x0FC */
	uint32_t   ISPR[8];     /* Address offset: 0x100 - 0x11C */
	uint32_t  RES2[24];     /* Address offset: 0x120 - 0x17C */
	uint32_t   ICPR[8];     /* Address offset: 0x180 - 0x19C */
	uint32_t  RES3[24];     /* Address offset: 0x1A0 - 0x1FC */
	uint32_t   IABR[8];     /* Address offset: 0x200 - 0x21C */
	uint32_t  RES4[56];     /* Address offset: 0x220 - 0x2FC */
	uint8_t   IPR[240];     /* Address offset: 0x300 - 0x3EC */
	uint32_t RES5[644];     /* Address offset: 0x3F0 - 0xEFC */
	uint32_t       STIR;    /* Address offset:         0xF00 */
} NVIC_type;

typedef struct
{
	uint32_t CSR;      /* SYSTICK control and status register,       Address offset: 0x00 */
	uint32_t RVR;      /* SYSTICK reload value register,             Address offset: 0x04 */
	uint32_t CVR;      /* SYSTICK current value register,            Address offset: 0x08 */
	uint32_t CALIB;    /* SYSTICK calibration value register,        Address offset: 0x0C */
} STK_type;

#define AFIO            ((AFIO_type  *)  AFIO_BASE)
#define GPIOE           ((GPIO_type  *) GPIOE_BASE)
#define RCC             ((RCC_type   *)   RCC_BASE)
#define FLASH           ((FLASH_type *) FLASH_BASE)
#define TIM1            ((TIM_type   *)  TIM1_BASE)
#define NVIC            ((NVIC_type  *)  NVIC_BASE)
#define SYSTICK ((STK_type *) SYSTICK_BASE)


#define SET_BIT(REG, BIT)     ((REG) |=  (1 << BIT))
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(1 << BIT))
#define READ_BIT(REG, BIT)    ((REG) &   (1 << BIT))

/*
 * STM32F107 Interrupt Number Definition
 */
typedef enum IRQn
{
	NonMaskableInt_IRQn         = -14,    /* 2 Non Maskable Interrupt                             */
	MemoryManagement_IRQn       = -12,    /* 4 Cortex-M3 Memory Management Interrupt              */
	BusFault_IRQn               = -11,    /* 5 Cortex-M3 Bus Fault Interrupt                      */
	UsageFault_IRQn             = -10,    /* 6 Cortex-M3 Usage Fault Interrupt                    */
	SVCall_IRQn                 = -5,     /* 11 Cortex-M3 SV Call Interrupt                       */
	DebugMonitor_IRQn           = -4,     /* 12 Cortex-M3 Debug Monitor Interrupt                 */
	PendSV_IRQn                 = -2,     /* 14 Cortex-M3 Pend SV Interrupt                       */
	SysTick_IRQn                = -1,     /* 15 Cortex-M3 System Tick Interrupt                   */
	WWDG_IRQn                   = 0,      /* Window WatchDog Interrupt                            */
	PVD_IRQn                    = 1,      /* PVD through EXTI Line detection Interrupt            */
	TAMPER_IRQn                 = 2,      /* Tamper Interrupt                                     */
	RTC_IRQn                    = 3,      /* RTC global Interrupt                                 */
	FLASH_IRQn                  = 4,      /* FLASH global Interrupt                               */
	RCC_IRQn                    = 5,      /* RCC global Interrupt                                 */
	EXTI0_IRQn                  = 6,      /* EXTI Line0 Interrupt                                 */
	EXTI1_IRQn                  = 7,      /* EXTI Line1 Interrupt                                 */
	EXTI2_IRQn                  = 8,      /* EXTI Line2 Interrupt                                 */
	EXTI3_IRQn                  = 9,      /* EXTI Line3 Interrupt                                 */
	EXTI4_IRQn                  = 10,     /* EXTI Line4 Interrupt                                 */
	DMA1_Channel1_IRQn          = 11,     /* DMA1 Channel 1 global Interrupt                      */
	DMA1_Channel2_IRQn          = 12,     /* DMA1 Channel 2 global Interrupt                      */
	DMA1_Channel3_IRQn          = 13,     /* DMA1 Channel 3 global Interrupt                      */
	DMA1_Channel4_IRQn          = 14,     /* DMA1 Channel 4 global Interrupt                      */
	DMA1_Channel5_IRQn          = 15,     /* DMA1 Channel 5 global Interrupt                      */
	DMA1_Channel6_IRQn          = 16,     /* DMA1 Channel 6 global Interrupt                      */
	DMA1_Channel7_IRQn          = 17,     /* DMA1 Channel 7 global Interrupt                      */
	ADC1_2_IRQn                 = 18,     /* ADC1 and ADC2 global Interrupt                       */
	CAN1_TX_IRQn                = 19,     /* USB Device High Priority or CAN1 TX Interrupts       */
	CAN1_RX0_IRQn               = 20,     /* USB Device Low Priority or CAN1 RX0 Interrupts       */
	CAN1_RX1_IRQn               = 21,     /* CAN1 RX1 Interrupt                                   */
	CAN1_SCE_IRQn               = 22,     /* CAN1 SCE Interrupt                                   */
	EXTI9_5_IRQn                = 23,     /* External Line[9:5] Interrupts                        */
	TIM1_BRK_IRQn               = 24,     /* TIM1 Break Interrupt                                 */
	TIM1_UP_IRQn                = 25,     /* TIM1 Update Interrupt                                */
	TIM1_TRG_COM_IRQn           = 26,     /* TIM1 Trigger and Commutation Interrupt               */
	TIM1_CC_IRQn                = 27,     /* TIM1 Capture Compare Interrupt                       */
	TIM2_IRQn                   = 28,     /* TIM2 global Interrupt                                */
	TIM3_IRQn                   = 29,     /* TIM3 global Interrupt                                */
	TIM4_IRQn                   = 30,     /* TIM4 global Interrupt                                */
	I2C1_EV_IRQn                = 31,     /* I2C1 Event Interrupt                                 */
	I2C1_ER_IRQn                = 32,     /* I2C1 Error Interrupt                                 */
	I2C2_EV_IRQn                = 33,     /* I2C2 Event Interrupt                                 */
	I2C2_ER_IRQn                = 34,     /* I2C2 Error Interrupt                                 */
	SPI1_IRQn                   = 35,     /* SPI1 global Interrupt                                */
	SPI2_IRQn                   = 36,     /* SPI2 global Interrupt                                */
	USART1_IRQn                 = 37,     /* USART1 global Interrupt                              */
	USART2_IRQn                 = 38,     /* USART2 global Interrupt                              */
	USART3_IRQn                 = 39,     /* USART3 global Interrupt                              */
	EXTI15_10_IRQn              = 40,     /* External Line[15:10] Interrupts                      */
	RTCAlarm_IRQn               = 41,     /* RTC Alarm through EXTI Line Interrupt                */
	OTG_FS_WKUP_IRQn            = 42,     /* USB OTG FS WakeUp from suspend through EXTI Line Int */
	TIM5_IRQn                   = 50,     /* TIM5 global Interrupt                                */
	SPI3_IRQn                   = 51,     /* SPI3 global Interrupt                                */
	UART4_IRQn                  = 52,     /* UART4 global Interrupt                               */
	UART5_IRQn                  = 53,     /* UART5 global Interrupt                               */
	TIM6_IRQn                   = 54,     /* TIM6 global Interrupt                                */
	TIM7_IRQn                   = 55,     /* TIM7 global Interrupt                                */
	DMA2_Channel1_IRQn          = 56,     /* DMA2 Channel 1 global Interrupt                      */
	DMA2_Channel2_IRQn          = 57,     /* DMA2 Channel 2 global Interrupt                      */
	DMA2_Channel3_IRQn          = 58,     /* DMA2 Channel 3 global Interrupt                      */
	DMA2_Channel4_IRQn          = 59,     /* DMA2 Channel 4 global Interrupt                      */
	DMA2_Channel5_IRQn          = 60,     /* DMA2 Channel 5 global Interrupt                      */
	ETH_IRQn                    = 61,     /* Ethernet global Interrupt                            */
	ETH_WKUP_IRQn               = 62,     /* Ethernet Wakeup through EXTI line Interrupt          */
	CAN2_TX_IRQn                = 63,     /* CAN2 TX Interrupt                                    */
	CAN2_RX0_IRQn               = 64,     /* CAN2 RX0 Interrupt                                   */
	CAN2_RX1_IRQn               = 65,     /* CAN2 RX1 Interrupt                                   */
	CAN2_SCE_IRQn               = 66,     /* CAN2 SCE Interrupt                                   */
	OTG_FS_IRQn                 = 67      /* USB OTG FS global Interrupt                          */
} IRQn_type;


#endif /* _STM32F107XX_H */
