# stm32f1-uGFX

Template for TFT display use on an STM32F107-based board. (Cortex-M3)
The display is ILI9341 driver based EasyTFT from mikroelektronika
Depends on the uGFX library

# Preinstall
* Toolchain - [GNU ARM Toolchain](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads)
* (Windows only) - [MinGW and MSYS ](http://www.mingw.org/)
* Programmer & Debugger - [STLink](https://github.com/texane/stlink)

# Initialize the uGFX library
* Run `git submodule init`
* Run `git submodule update`

or

* Clone the project recursively `git clone --recursive https://github.com/fcayci/stm32f1-uGFX`

# Compile
* Browse into the directory and run `make` to compile.
```
Cleaning...
Building tft.c
   text	   data	    bss	    dec	    hex	filename
    16964	   2112	    292	  19368	   4ba8	tft.elf
Successfully finished...
```

# Program
* Run `make burn` to program the chip.
```
...
Flash written and verified! jolly good!
```

# Debug
* Run `st-util` from one terminal
* Run `make debug` from a second terminal to debug the program.
* You can turn off tui layout with `tui disable` if needed
