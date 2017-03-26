# stm32f1-bare-metal

Bare-metal programming for a STM32F1-based board. (Cortex-M3)

Tested on EasyMx Pro v7 board with STM32F107 chip.

# Install
* Toolchain - [GNU ARM Toolchain](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads)
* (Windows only) - [MinGW and MSYS ](http://www.mingw.org/)
* Programmer - [STLink](https://github.com/texane/stlink)

# Compile
* Browse into any of the project directory `cd systick` and run `make` to compile.
```
Cleaning...
Building systick.c
   text	   data	    bss	    dec	    hex	filename
    384	      0	      0	    384	    180	systick.elf
Successfully finished...
```

# Program
* Run `make burn` to program the chip.
```
...
...
Flash written and verified! jolly good!
```

## Projects
* systick - SysTick Timer example
* clock - An example to bump the clock speed to 72 Mhz
* ext_int - An external interrupt example to toggle LEDs using button presses. Uses EXTI0 and EXTI1 modules.
* timer - A timer example to toggle LEDs in 1 second intervals. Uses Timer3 module.
