# STM32F4 STDPERIPH files.
STM32F4LIB = libstm32f4
STM32SRC = 	$(STM32F4LIB)/src/misc.c \
			$(STM32F4LIB)/src/stm32f4xx_adc.c \
			$(STM32F4LIB)/src/stm32f4xx_dma.c \
			$(STM32F4LIB)/src/stm32f4xx_exti.c \
			$(STM32F4LIB)/src/stm32f4xx_flash.c \
			$(STM32F4LIB)/src/stm32f4xx_rcc.c \
			$(STM32F4LIB)/src/stm32f4xx_syscfg.c \
			$(STM32F4LIB)/src/stm32f4xx_tim.c \
			$(STM32F4LIB)/src/stm32f4xx_iwdg.c \
			$(STM32F4LIB)/src/stm32f4xx_wwdg.c

STM32INC = $(STM32F4LIB)/inc

