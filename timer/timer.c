/*
 * timer.c
 *
 * Description:
 *   A simple timer example using LEDs.
 *   Toggels PortE Pin14 and Pin13 with 1 second interval.
 *
 * Timer setup steps:
 *   1. Enable TIMx clock from RCC
 *   2. Choose the prescaler value
 *   3. Set the count value to max
 *   4. Enable TIMx module
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

#define TIM3_BASE       (APB1PERIPH_BASE + 0x0400) //  TIM3 base address is 0x40000400

#define GPIOE_BASE      (APB2PERIPH_BASE + 0x1800) // GPIOE base address is 0x40011800

#define RCC_BASE        ( AHBPERIPH_BASE + 0x1000) //   RCC base address is 0x40021000
#define FLASH_BASE      ( AHBPERIPH_BASE + 0x2000) // FLASH base address is 0x40022000

#define STACKINIT       0x20008000
#define DELAY           7200000

#define GPIOE           ((GPIO_type  *) GPIOE_BASE)
#define RCC             ((RCC_type   *)   RCC_BASE)
#define FLASH           ((FLASH_type *) FLASH_BASE)
#define TIM3            ((TIM_type  *)   TIM3_BASE)

/*
 * Macros
 */
#define SET_BIT(REG, BIT)     ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)    ((REG) & (BIT))

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
	uint32_t RES2;      /* Address offset: 0x44 */
	uint32_t DCR;       /* Address offset: 0x48 */
	uint32_t DMAR;      /* Address offset: 0x4C */
} TIM_type;

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

// Function declarations. Add your functions here
void set_system_clock_to_25Mhz(void);
void set_system_clock_to_72Mhz(void);
int32_t main(void);
void delay_ms(volatile uint32_t s);

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
};

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

/*************************************************
* Main code starts from here
*************************************************/
int32_t main(void)
{
	// Set clock to 72 MHz
	set_system_clock_to_72Mhz();

	// Set Bit 6 to enable GPIOE clock
	RCC->APB2ENR |= (1 << 6);

	// Make GPIOE Pin14 output
	GPIOE->CRH &= 0xF00FFFFF;
	GPIOE->CRH |= 0x02200000;

	// Reset GPIOE Pin14
	GPIOE->ODR &= ~(1 << 14);
	// Set GPIOE Pin13
	GPIOE->ODR |= (1 << 13);

	// Enable clock for that module for TIM3. Bit1 in RCC APB1ENR register
	RCC->APB1ENR |= (1 << 1);

	// Reset CR1 just in case
	TIM3->CR1 = 0x0000;

	// fCK_PSC / (PSC[15:0] + 1)
	// 72 Mhz / 71 + 1 = 1 Mhz timer clock speed
	TIM3->PSC = 71;

	// This is set to max value (0xFFFF) since we manually check
	//   if the value reach to 1000 in the delay_ms function
	TIM3->ARR = 0xFFFF;

	// Finally enable TIM3 module
	TIM3->CR1 |= (1 << 0);

	while(1)
	{
		delay_ms(1000);
		GPIOE->ODR ^= (1 << 14);
		GPIOE->ODR ^= (1 << 13);
	}

	// Should never reach here
	return 0;
}

/*
 * milli-second delay function. Check if the 1 MHz timer reached to 1000.
 * Repeat it s times
 */
void delay_ms(volatile uint32_t s)
{
	for(s; s>0; s--){
		// Reset the timer
		TIM3->EGR |= 0x0001;
		// Wait until timer reaches to 1000
		// It is 1000 becuase timer is running at 1 MHz and 1000 will
		//   generate 1 milli-second
 		while(TIM3->CNT < 1000);
	}
}
