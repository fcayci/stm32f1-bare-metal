/*
 * mathtest.c
 *
 * Description:
 *  An example project to show how to use functions from the math library
 *
 * Setup steps:
 *   1. Link against math library (-lm in makefile)
 *   2. Define the architecture when linking (-march=armv7-m in makefile)
 *     (note: -mcpu=cortex-m3 -mthumb should also work instead)
 *   3. When running the linker command, pass -lm after objects
 *
 * Author: Furkan Cayci
 *
 * Project Setup:
 *   EasyMX Pro V7 board
 *   Cortex-M3 arch
 *   STM32F107 chip
 */

/*************************************************
* Definitions
*************************************************/
// Include the math header from your toolchain installation.
// It should automatically be pulled from
// <arm-none-eabi-gcc toolchain installation directory>/arm-none-eabi/include
#include <math.h>
// Linker should patch the actual function from library directory located at
//  <arm-none-eabi-gcc toolchain installation directory>/arm-none-eabi/lib/thumb/v7-m

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

#define GPIOD_BASE      (APB2PERIPH_BASE + 0x1400) // GPIOD base address is 0x40011400
#define RCC_BASE        ( AHBPERIPH_BASE + 0x1000) //   RCC base address is 0x40021000

#define STACKINIT       0x20008000

#define GPIOD           ((GPIO_type  *) GPIOD_BASE)
#define RCC             ((RCC_type   *)   RCC_BASE)

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

// Function declarations. Add your functions here
void copy_data(void);
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
};

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

double alpha;

uint32_t glb_uint_array[80] = {
	1 ,2 ,3 ,4 ,5 ,6 ,7 ,8 ,9 ,10,
	11,12,13,14,15,16,17,18,19,20,
	21,22,23,24,25,26,27,28,29,30,
	31,32,33,34,35,36,37,38,39,40,
	41,42,43,44,45,46,47,48,49,50,
	51,52,53,54,55,56,57,58,59,60,
	61,62,63,64,65,66,67,68,69,70,
	71,72,73,74,75,76,77,78,79,80};

/*************************************************
* Main code starts from here
*************************************************/
int32_t main(void)
{
	// Copy LMA to VMA for data section
	copy_data();

	alpha = 0.5f;

	uint32_t i, j;
	// Enable GPIOD clock. Bit 5 in RCC APB2ENR register
	RCC->APB2ENR |= (1 << 5);

	// Make PORTD output
	GPIOD->CRL = 0x22222222;
	GPIOD->CRH = 0x22222222;
	GPIOD->ODR = 0xFF;

	while(1)
	{
		for (i=1; i<80; i++)
		{
			GPIOD->ODR = (uint16_t)sqrt((double)glb_uint_array[i] * alpha);
			for (j=10000; j>0; j--); // Delay
		}
	}

	// Should never reach here
	return 0;
}
