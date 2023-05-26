## PY32F030 Project using IAR 8.4

### Project home on [github.com: markr1961/PY32F030_Start_Kit](https://github.com/markr1961/PY32F030_Start_Kit)

Based on [GitHub: markr1961 PY32F0xx_IAR_template](https://github.com/markr1961/PY32F0xx_IAR_template) 
Which is in turn based on the work from [Jay Carlson's py32 template](https://github.com/jaydcarlson/py32-template)  
The CMSIS core has been updated to a newer version, V5.3.2 dates 27. May 2021  
Unlike the template, this project compiles fine with HAL DMA enabled in py32f0xx_hal_conf.h.  

### External links
- [Jay's article with links to the Puya SDK files](https://jaycarlson.net/2023/02/04/the-cheapest-flash-microcontroller-you-can-buy-is-actually-an-arm-cortex-m0/)
- [PY32F0xx at PuyaSemi](https://www.puyasemi.com/cpzx3/info_271_aid_247_kid_246.html)
- link to Puya SDK, docs, and IDE drivers in a RAR file: https://www.puyasemi.com/uploadfiles/2022/11/PY-MCU%E8%B5%84%E6%96%99-20221117.rar  
- Another github repo with more recent drivers and docs https://github.com/IOsetting/py32f0-template Not sure who 'IOsetting' is. Does not appear to be an official Puya repo.

## Build Info
IAR 8.4.x  
This project can be built using the IAR Kickstarter license.  
The debug, linker and programming files are needed from the Puya file and copied to your IAR install.  
Both LL_Flash and HAL_Flash cannot be compiled at the same time. Needs investigation.  

#### preprocessor defines 
PY32F030x8  
USE_HAL_DRIVER  
USE_FULL_LL_DRIVER  

#### include paths
\$PROJ_DIR\$\CMSIS\Device\PY32F0xx\Include  
\$PROJ_DIR\$\CMSIS\Core\Include  
\$PROJ_DIR\$\PY32F0xx_HAL_Driver\Inc  
\$PROJ_DIR\$\Inc  

### Target:  PY32F030K18T6 using a "PY32F030_Start Kit V1.3"
64K flash, xxK RAM  
speed: 2x PLL -> 48 MHz from 24 MHz HSI

LQFP32  

### Debuger
ST-Link v2 seems to work. Returns x453 "unknown CPU"  

## -- commit notes --

05/25/2023 MAR  
existing project blinks LED @ 10Hz, with EXTI IRQ handling button(key). When the button is pressed the main loop halts while it turns LED on for 1 second then off for one second.  
fixed issues with compiling both HAL and LL in the same project.  
now uses the correct CPU (Puya PY32F030x8)  
