/*
 * systick.c
 *
 * Description:
 *   Systick example to turn on LEDs at 1 second intervals.
 *   GPIOD Pin1 and GPIOE Pin14
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
// Define some types for readibility
#define int32_t         int
#define int16_t         short
#define int8_t          char
#define uint32_t        unsigned int
#define uint16_t        unsigned short
#define uint8_t         unsigned char

// Define the base addresses for peripherals
#define PERIPH_BASE     ((uint32_t) 0x40000000)
#define SYSTICK_BASE    ((uint32_t) 0xE000E010)

#define GPIOC_BASE      (PERIPH_BASE + 0x11000) // GPIOC base address is 0x40011000
#define GPIOD_BASE      (PERIPH_BASE + 0x11400) // GPIOD base address is 0x40011400
#define GPIOE_BASE      (PERIPH_BASE + 0x11800) // GPIOE base address is 0x40011800
#define RCC_BASE        (PERIPH_BASE + 0x21000) //   RCC base address is 0x40021000

#define STACKINIT       0x20008000
#define DELAY           70000

#define GPIOD   ((GPIO_type *)  GPIOD_BASE)
#define GPIOE   ((GPIO_type *)  GPIOE_BASE)
#define RCC     ((RCC_type *)     RCC_BASE)
#define SYSTICK ((STK_type *) SYSTICK_BASE)

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
	uint32_t CFGR2;    /* RCC clock configuration register2,         Address offset: 0x2C */
} RCC_type;

typedef struct
{
	uint32_t CSR;      /* SYSTICK control and status register,       Address offset: 0x00 */
	uint32_t RVR;      /* SYSTICK reload value register,             Address offset: 0x04 */
	uint32_t CVR;      /* SYSTICK current value register,            Address offset: 0x08 */
	uint32_t CALIB;    /* SYSTICK calibration value register,        Address offset: 0x0C */
} STK_type;

/* Function declarations */
void systick_handler(void);
void init_systick(uint32_t s, uint8_t en);
int32_t main(void);
void delay_ms(volatile uint32_t s);

/*************************************************
* Vector Table
*************************************************/
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

/*
 * SysTick interrupt handler function
 *
 * If systick is enabled with interrupt, this function is used
 */
void systick_handler(void){
	GPIOE->ODR = GPIOE->ODR ^ 0x4000;
	GPIOD->ODR = GPIOD->ODR ^ 0x0002;
}

/*
 * Initialize SysTick Timer
 *
 * Since it is set up to run at 1Mhz, an s value of
 * 1Khz needed to make 1 millisecond timer
 */
void init_systick(uint32_t s, uint8_t en)
{
	// Main clock source is running with HSI by default which is at 8 Mhz.
	// SysTick clock source can be set with CTRL register (Bit 2) (pm0056 - page 151)
	// 0: AHB/8 -> (1 MHz)
	// 1: Processor clock (AHB) -> (8 MHz)
	SYSTICK->CSR |= 0x00000; // Currently set to run at 1 Mhz
	// Enable callback
	SYSTICK->CSR |= (en << 1);
	// Load the reload value
	SYSTICK->RVR = s;
	// Set the current value to 0
	SYSTICK->CVR = 0;
	// Enable SysTick
	SYSTICK->CSR |= (1 << 0);
}

int32_t main(void)
{
	RCC->APB2ENR |= (1 << 5); // Enable GPIOD
	RCC->APB2ENR |= (1 << 6); // Enable GPIOE

	// Make GPIOD Pin1 output
	GPIOD->CRL &= 0xFFFFFF0F;
	GPIOD->CRL |= 0x00000020;

	// Make GPIOE Pin14 output
	GPIOE->CRH &= 0xF0FFFFFF;
	GPIOE->CRH |= 0x02000000;

	// Initial values
	GPIOD->ODR |= 0x0000; // 0x0002
	GPIOE->ODR |= 0x4000;

	// Initialize systick with 2**10 ~ 1Kil
	// Disable interrupt
	init_systick(1 << 10, 0);

	while(1){
		delay_ms(1000);  // ~1 second
		GPIOE->ODR = GPIOE->ODR ^ 0x4000;
		GPIOD->ODR = GPIOD->ODR ^ 0x0002;
	}

	// Should never reach here
	return 0;
}

/*
 * Millisecond delay function.
 *   volatile keyword is used so that compiler does not optimize it away
 * Polling method (If interrupt is not enabled)
 */
void delay_ms(volatile uint32_t s)
{
	for(s; s>0; s--){
		while(!(SYSTICK->CSR & (1 << 16))); // Wait until COUNTFLAG is 1
	}
}
