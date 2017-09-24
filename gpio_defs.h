#ifndef GPIO_DEFS_H
#define GPIO_DEFS_H

#define SW1_POS (7)		// on port D
#define SW2_POS (6)		// on port D

#define MASK(x) (1UL << (x))

// Debug Signals on port B
#define DBG_1 1
#define DBG_2 2
#define DBG_3 3
#define DBG_4 8
#define DBG_5 9
#define DBG_6 10
#define DBG_7 11


#define SWITCH_PRESSED(x) (!(PTD->PDIR & (MASK(x))))

// Freedom KL25Z LEDs
#define RED_LED_POS (18)		// on port B
#define GREEN_LED_POS (19)		// on port B
#define BLUE_LED_POS (1)		// on port D

#define ON_TIME (4000)
#define FLASH_TIME (600)

/*
	Code phases:
			V1: Base with busy waits
			V2: Tasks
			V2.5: Interrupt to read switches
			V3: FSM LED tasks
			V4: Add HW Timer to FSM LED and switch tasks
			V5: Interrupt to read switches
*/

#define V1 0
#define V2 0
#define V2_5 0
#define V3 1
#define V4 0
#define V5 0

#if V1 | V2 | V2_5 | V3
#define W_DELAY_SLOW 400
#define W_DELAY_FAST 200
#define RGB_DELAY_SLOW 6000
#define RGB_DELAY_FAST 1000
#else
#define W_DELAY_SLOW 2000000
#define W_DELAY_FAST 1000000
#define RGB_DELAY_SLOW 36000000
#define RGB_DELAY_FAST 8000000
#endif

#endif
// *******************************ARM University Program Copyright © ARM Ltd 2013*************************************   
