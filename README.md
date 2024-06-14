# f407ze

本项目使用的 CPU 为 STM32F407ZET6,   
Firmware version 是 STM32Cube FW_F4 V1.27.1  
使用 STM32CubeMX 生成目录结构 和 Makefile.  

artuguy@ryzen5:~/stm32/f407ze$ tree  
.  
├── Core  
│   ├── Inc  
│   │   ├── main.h  
│   │   ├── stm32_assert.h  
│   │   └── stm32f4xx_it.h  
│   └── Src  
│       ├── main.c  
│       ├── stm32f4xx_it.c  
│       └── system_stm32f4xx.c  
├── Drivers
│   ├── CMSIS
│   │   ├── Device
│   │   │   └── ST
│   │   │       └── STM32F4xx
│   │   │           ├── Include
│   │   │           │   ├── stm32f407xx.h
│   │   │           │   ├── stm32f4xx.h
│   │   │           │   └── system_stm32f4xx.h
│   │   │           ├── LICENSE.txt
│   │   │           └── Source
│   │   │               └── Templates
│   │   ├── Include
│   │   │   ├── cmsis_armcc.h
│   │   │   ├── cmsis_armclang.h
│   │   │   ├── cmsis_compiler.h
│   │   │   ├── cmsis_gcc.h
│   │   │   ├── cmsis_iccarm.h
│   │   │   ├── cmsis_version.h
│   │   │   ├── core_armv8mbl.h
│   │   │   ├── core_armv8mml.h
│   │   │   ├── core_cm0.h
│   │   │   ├── core_cm0plus.h
│   │   │   ├── core_cm1.h
│   │   │   ├── core_cm23.h
│   │   │   ├── core_cm3.h
│   │   │   ├── core_cm33.h
│   │   │   ├── core_cm4.h
│   │   │   ├── core_cm7.h
│   │   │   ├── core_sc000.h
│   │   │   ├── core_sc300.h
│   │   │   ├── mpu_armv7.h
│   │   │   ├── mpu_armv8.h
│   │   │   └── tz_context.h
│   │   └── LICENSE.txt
│   └── STM32F4xx_HAL_Driver
│       ├── Inc
│       │   ├── stm32f4xx_ll_bus.h
│       │   ├── stm32f4xx_ll_cortex.h
│       │   ├── stm32f4xx_ll_dma.h
│       │   ├── stm32f4xx_ll_exti.h
│       │   ├── stm32f4xx_ll_gpio.h
│       │   ├── stm32f4xx_ll_pwr.h
│       │   ├── stm32f4xx_ll_rcc.h
│       │   ├── stm32f4xx_ll_system.h
│       │   ├── stm32f4xx_ll_usart.h
│       │   └── stm32f4xx_ll_utils.h
│       ├── LICENSE.txt
│       └── Src
│           ├── stm32f4xx_ll_dma.c
│           ├── stm32f4xx_ll_exti.c
│           ├── stm32f4xx_ll_gpio.c
│           ├── stm32f4xx_ll_rcc.c
│           ├── stm32f4xx_ll_usart.c
│           └── stm32f4xx_ll_utils.c
├── Makefile
├── README.md
├── STM32F407ZETx_FLASH.ld
├── build
│   ├── f407ze.bin
│   ├── f407ze.elf
│   ├── f407ze.hex
│   ├── f407ze.map
│   ├── main.d
│   ├── main.lst
│   ├── main.o
│   ├── startup_stm32f407xx.d
│   ├── startup_stm32f407xx.o
│   ├── stm32f4xx_it.d
│   ├── stm32f4xx_it.lst
│   ├── stm32f4xx_it.o
│   ├── stm32f4xx_ll_dma.d
│   ├── stm32f4xx_ll_dma.lst
│   ├── stm32f4xx_ll_dma.o
│   ├── stm32f4xx_ll_exti.d
│   ├── stm32f4xx_ll_exti.lst
│   ├── stm32f4xx_ll_exti.o
│   ├── stm32f4xx_ll_gpio.d
│   ├── stm32f4xx_ll_gpio.lst
│   ├── stm32f4xx_ll_gpio.o
│   ├── stm32f4xx_ll_rcc.d
│   ├── stm32f4xx_ll_rcc.lst
│   ├── stm32f4xx_ll_rcc.o
│   ├── stm32f4xx_ll_usart.d
│   ├── stm32f4xx_ll_usart.lst
│   ├── stm32f4xx_ll_usart.o
│   ├── stm32f4xx_ll_utils.d
│   ├── stm32f4xx_ll_utils.lst
│   ├── stm32f4xx_ll_utils.o
│   ├── system_stm32f4xx.d
│   ├── system_stm32f4xx.lst
│   └── system_stm32f4xx.o
├── f407ze.ioc
└── startup_stm32f407xx.s

17 directories, 87 files
