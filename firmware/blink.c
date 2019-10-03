/* #define STM32F1 */
/* #include <libopencm3/stm32/rcc.h> */
/* #include <libopencm3/stm32/gpio.h> */

int main() {
	/* Enable GPIOC clock. */
	/* rcc_periph_clock_enable(RCC_GPIOC); */

	/* #<{(| Set GPIO8 (in GPIO port C) to 'output push-pull'. |)}># */
	/* gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, */
	/* 	            GPIO_CNF_OUTPUT_PUSHPULL, GPIO13); */
  /*  */
	/* int i = 0; */
	/* while(1) { */
	/* 	gpio_clear(GPIOC, GPIO13);	#<{(| LED on |)}># */
	/* 	for (i = 0; i < 1500000; i++)	{ */
	/* 	  #<{(| Wait a bit. |)}># */
	/* 		__asm__("nop"); */
	/* 	} */
  /*  */
	/* 	gpio_set(GPIOC, GPIO13);		#<{(| LED off |)}># */
	/* 	for (i = 0; i < 500000; i++) { */
	/* 	  #<{(| Wait a bit. |)}># */
	/* 		__asm__("nop"); */
	/* 	} */
	/* } */

	return 0;
}
