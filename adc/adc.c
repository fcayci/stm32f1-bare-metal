/*
 * adc.c
 *
 * Description:
 *   An ADC input example.
 *   Displays the value of the analog pot on PortD LEDs.
 *   Connect analog pot to PC0 which is mapped to ADC12_IN10
 *
 * ADC setup steps:
 *   1. Enable ADC clock from RCC
 *   2. Enable Alternate function clock from RCC
 *   3. Enable the relevant GPIO clock from RCC
 *     (Each ADCx is connected to a GPIO pin.)
 *   4. Choose the channel to be read by writing the channel number
 *     to SQR3 SR1 bits.
 *   5. (Optional) Enable End of Conversion interrupt
 *   6. Enable interrupt from NVIC
 *   7. Enable ADC module from CR2
 *
 * Author: Furkan Cayci
 *
 *  Project Setup:
 *   EasyMX Pro V7 board
 *   Cortex-M3 arch
 *   STM32F107 chip
 */

/*************************************************
* Definitions
*************************************************/
// This section can go into a header file if wanted
// Define some types for readibility
#define int32_t         int
#define int16_t         short
#define int8_t          char
#define uint32_t        unsigned int
#define uint16_t        unsigned short
#define uint8_t         unsigned char

#define HSE_Value       ((uint32_t) 25000000) /* Value of the External oscillator in Hz */
#define HSI_Value       ((uint32_t)  8000000) /* Value of the Internal oscillator in Hz*/

// Define the base addresses for peripherals
#define PERIPH_BASE     ((uint32_t) 0x40000000)
#define SRAM_BASE       ((uint32_t) 0x20000000)

#define APB1PERIPH_BASE PERIPH_BASE
#define APB2PERIPH_BASE (PERIPH_BASE + 0x10000)
#define AHBPERIPH_BASE  (PERIPH_BASE + 0x20000)

#define AFIO_BASE       (APB2PERIPH_BASE + 0x0000) //  AFIO base address is 0x40010000
#define GPIOC_BASE      (APB2PERIPH_BASE + 0x1000) // GPIOC base address is 0x40011000
#define GPIOD_BASE      (APB2PERIPH_BASE + 0x1400) // GPIOD base address is 0x40011400
#define ADC1_BASE       (APB2PERIPH_BASE + 0x2400) //  ADC1 base address is 0x40012400

#define RCC_BASE        ( AHBPERIPH_BASE + 0x1000) //   RCC base address is 0x40021000
#define FLASH_BASE      ( AHBPERIPH_BASE + 0x2000) // FLASH base address is 0x40022000

#define NVIC_BASE       ((uint32_t) 0xE000E100)

#define STACKINIT       0x20008000
#define DELAY           7200000

#define AFIO            ((AFIO_type  *)  AFIO_BASE)
#define GPIOC           ((GPIO_type  *) GPIOC_BASE)
#define GPIOD           ((GPIO_type  *) GPIOD_BASE)
#define RCC             ((RCC_type   *)   RCC_BASE)
#define FLASH           ((FLASH_type *) FLASH_BASE)
#define ADC1            ((ADC_type   *)  ADC1_BASE)
#define NVIC            ((NVIC_type  *)  NVIC_BASE)

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
	uint32_t SR;        /* Address offset: 0x00 */
	uint32_t CR1;       /* Address offset: 0x04 */
	uint32_t CR2;       /* Address offset: 0x08 */
	uint32_t SMPR1;     /* Address offset: 0x0C */
	uint32_t SMPR2;     /* Address offset: 0x10 */
	uint32_t JOFR1;     /* Address offset: 0x14 */
	uint32_t JOFR2;     /* Address offset: 0x18 */
	uint32_t JOFR3;     /* Address offset: 0x1C */
	uint32_t JOFR4;     /* Address offset: 0x20 */
	uint32_t HTR;       /* Address offset: 0x24 */
	uint32_t LTR;       /* Address offset: 0x28 */
	uint32_t SQR1;      /* Address offset: 0x2C */
	uint32_t SQR2;      /* Address offset: 0x30 */
	uint32_t SQR3;      /* Address offset: 0x34 */
	uint32_t JSQR;      /* Address offset: 0x38 */
	uint32_t JDR1;      /* Address offset: 0x3C */
	uint32_t JDR2;      /* Address offset: 0x40 */
	uint32_t JDR3;      /* Address offset: 0x44 */
	uint32_t JDR4;      /* Address offset: 0x48 */
	uint32_t DR;        /* Address offset: 0x4C */
} ADC_type;

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


// Function declarations. Add your functions here
void enable_interrupt(IRQn_type IRQn);
void disable_interrupt(IRQn_type IRQn);
void set_system_clock_to_25Mhz(void);
void set_system_clock_to_72Mhz(void);
void adc1_handler(void);
int32_t main(void);

/*************************************************
* Vector Table
*************************************************/
// Attribute puts table in beginning of .vector section
//   which is the beginning of .text section in the linker script
// Add other vectors in order here
// Vector table can be found on page 197 in RM0008
uint32_t (* const vector_table[])
__attribute__ ((section(".vectors"))) = {
	(uint32_t *) STACKINIT,         /* 0x000 Stack Pointer                   */
	(uint32_t *) main,              /* 0x004 Reset                           */
	0,                              /* 0x008 Non maskable interrupt          */
	0,                              /* 0x00C HardFault                       */
	0,                              /* 0x010 Memory Management               */
	0,                              /* 0x014 BusFault                        */
	0,                              /* 0x018 UsageFault                      */
	0,                              /* 0x01C Reserved                        */
	0,                              /* 0x020 Reserved                        */
	0,                              /* 0x024 Reserved                        */
	0,                              /* 0x028 Reserved                        */
	0,                              /* 0x02C System service call             */
	0,                              /* 0x030 Debug Monitor                   */
	0,                              /* 0x034 Reserved                        */
	0,                              /* 0x038 PendSV                          */
	0,                              /* 0x03C System tick timer               */
	0,                              /* 0x040 Window watchdog                 */
	0,                              /* 0x044 PVD through EXTI Line detection */
	0,                              /* 0x048 Tamper                          */
	0,                              /* 0x04C RTC global                      */
	0,                              /* 0x050 FLASH global                    */
	0,                              /* 0x054 RCC global                      */
	0,                              /* 0x058 EXTI Line0                      */
	0,                              /* 0x05C EXTI Line1                      */
	0,                              /* 0x060 EXTI Line2                      */
	0,                              /* 0x064 EXTI Line3                      */
	0,                              /* 0x068 EXTI Line4                      */
	0,                              /* 0x06C DMA1_Ch1                        */
	0,                              /* 0x070 DMA1_Ch2                        */
	0,                              /* 0x074 DMA1_Ch3                        */
	0,                              /* 0x078 DMA1_Ch4                        */
	0,                              /* 0x07C DMA1_Ch5                        */
	0,                              /* 0x080 DMA1_Ch6                        */
	0,                              /* 0x084 DMA1_Ch7                        */
	(uint32_t *) adc1_handler,      /* 0x088 ADC1 and ADC2 global            */
	0,                              /* 0x08C CAN1_TX                         */
	0,                              /* 0x090 CAN1_RX0                        */
	0,                              /* 0x094 CAN1_RX1                        */
	0,                              /* 0x098 CAN1_SCE                        */
	0,                              /* 0x09C EXTI Lines 9:5                  */
	0,                              /* 0x0A0 TIM1 Break                      */
	0,                              /* 0x0A4 TIM1 Update                     */
	0,                              /* 0x0A8 TIM1 Trigger and Communication  */
	0,                              /* 0x0AC TIM1 Capture Compare            */
	0,                              /* 0x0B0 TIM2                            */
	0,                              /* 0x0B4 TIM3                            */
	0,                              /* 0x0B8 TIM4                            */
	0,                              /* 0x0BC I2C1 event                      */
	0,                              /* 0x0C0 I2C1 error                      */
	0,                              /* 0x0C4 I2C2 event                      */
	0,                              /* 0x0C8 I2C2 error                      */
	0,                              /* 0x0CC SPI1                            */
	0,                              /* 0x0D0 SPI2                            */
	0,                              /* 0x0D4 USART1                          */
	0,                              /* 0x0D8 USART2                          */
	0,                              /* 0x0DC USART3                          */
	0,                              /* 0x0E0 EXTI Lines 15:10                */
	0,                              /* 0x0E4 RTC alarm through EXTI line     */
	0,                              /* 49  USB OTG FS Wakeup through EXTI  */
	0,                              /* -   Reserved                        */
	0,                              /* -   Reserved                        */
	0,                              /* -   Reserved                        */
	0,                              /* -   Reserved                        */
	0,                              /* -   Reserved                        */
	0,                              /* -   Reserved                        */
	0,                              /* -   Reserved                        */
	0,                              /* 57  TIM5                            */
	0,                              /* 58  SPI3                            */
	0,                              /* 59  USART4                          */
	0,                              /* 60  USART5                          */
	0,                              /* 61  TIM6                            */
	0,                              /* 62  TIM7                            */
	0,                              /* 63  DMA2_Ch1                        */
	0,                              /* 64  DMA2_Ch2                        */
	0,                              /* 65  DMA2_Ch3                        */
	0,                              /* 66  DMA2_Ch4                        */
	0,                              /* 67  DMA2_Ch5                        */
	0,                              /* 68  Ethernet                        */
	0,                              /* 69  Ethernet wakeup                 */
	0,                              /* 70  CAN2_TX                         */
	0,                              /* 71  CAN2_RX0                        */
	0,                              /* 72  CAN2_RX1                        */
	0,                              /* 73  CAN2_SCE                        */
	0,                              /* 74  USB OTG FS                      */
};

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
 * By default 8 MHz internal clock is used (HSI)
 * Set up as 25 MHz
 *
 * 25 MHz External clock is selected as the source clock (HSE)
 */
void set_system_clock_to_25Mhz(void)
{
	// Enable HSE
	RCC->CR |= (1 << 16);
	// Wait untill HSE settles down
	while (!(RCC->CR & (1 << 17)));
	// Choose HSE as the system clock
	RCC->CFGR |= (1 << 0);
}

/*
 * By default 8 MHz internal clock is used (HSI)
 * Set up as 72 MHz (HSE-PLL)
 *
 * 25M ->   /5    ->   *8    ->   /5    ->   *9   -> 72 MHz
 * HSE -> PreDiv2 -> Pll2Mul -> PreDiv1 -> PllMul -> System Clock
 * Set Prediv1Src = PLL2, Set PllSrc as Prediv1
 *
 * 25 MHz External clock is selected as the source clock (HSE)
 * It is divided by 5 with PreDiv2, then muliplied by 8 with Pll2Mul
 * Then it is divided by 5 with PreDiv1, then multiplied by 9 with PllMul
 * Then choose Pll as the clock source
 */
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

/*
 * ADC 1 interrupt handler function
 */
void adc1_handler(void)
{
	// Read ADC value and pass it to GPIOD
	GPIOD->ODR = ADC1->DR;
}

/*************************************************
* Main code starts from here
*************************************************/
int32_t main(void)
{
	// Set clock to 72 MHz
	set_system_clock_to_72Mhz();

	// ADC clock can be 14 Mhz at max. So we need to divide the
	//   APB2 by 6 to make it 12 Mhz
	// Ensure ADCPRE is zero
	RCC-> CFGR &= ~(0b11 << 14);
	// Set prescaler to 6
	RCC-> CFGR |=  (0b10 << 14);

	// Enable alternate function clock. Bit 0 in RCC APB2ENR register
	RCC->APB2ENR |= (1 << 0);
	// Enable GPIOD clock. Bit 5 in RCC APB2ENR register
	RCC->APB2ENR |= (1 << 5);
	// Enable GPIOC clock. Bit 4 in RCC APB2ENR register
	RCC->APB2ENR |= (1 << 4);
	// Enable clock for ADC1 clock. Bit 9 in RCC APB2ENR register
	RCC->APB2ENR |= (1 << 9);

	// Make PC0 analog input (0b0000)
	GPIOC->CRL &= 0xFFFFFFF0;
	GPIOC->CRL |= 0x00000000;

	// Make PORTD output
	GPIOD->CRL = 0x22222222;
	GPIOD->CRH = 0x22222222;
	GPIOD->ODR = 0;

	// Enable End of Conversion (EOC) interrupt
	ADC1->CR1 |= (1 << 5);

	// One conversion
	ADC1->SQR1 = 0x00000000;

	// Choose the analog channel to read
	// Since we want channel 10 to be the first
	//   conversion we write 10 to SQ1 bits (3:0)
	//   which is in SQR3 register. For multiple conversions
	//   keep writing the channel numbers to SQx bits.
	ADC1->SQR3 = (10 << 0);

	// Set up software trigger to start conversion
	ADC1->CR2 |= (7 << 17);  // Select SWSTART as trigger
	ADC1->CR2 |= (1 << 20);  // Enable external trigger

	// Enable continuous conversion
	ADC1->CR2 |= (1 << 1);

	enable_interrupt(ADC1_2_IRQn);

	// Enable A/D conversion
	ADC1->CR2 |= (1 << 0);

	// Calibration reset and start
	//    Optional for better accuracy.
	ADC1->CR2 |= (1 << 3);
	while((ADC1->CR2 & (1 << 3)));
	ADC1->CR2 |= (1 << 2);
	while((ADC1->CR2 & (1 << 2)));

	// Start conversion with software trigger
	ADC1->CR2 |= (1<<22);

	while(1)
	{
		// Read ADC value and pass it to GPIOD
		// This is done in the interrupt handler
		// Optionally can be done here without an interrupt
		//GPIOD->ODR = ADC1->DR;
	}

	// Should never reach here
	return 0;
}
