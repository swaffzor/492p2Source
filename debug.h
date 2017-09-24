#ifndef DEBUG_H
#define DEBUG_H

#define MASK(x) (1UL << (x))

// Debug Signals on port B
#define DBG_0 8 	// SPI_RW wait
#define DBG_1 1 	// SPI_RW wait
#define DBG_2 2	  // SD_Read active or waiting
#define DBG_3 3		// __SD_Write_Block active or waiting
#define DBG_4 8		// __SD_Send_Cmd active or waiting
#define DBG_5 9		// SD_Init active
#define DBG_6 10  // SD_Send_Cmd 
#define DBG_7 11

void Init_Debug_Signals(void);

#endif // DEBUG_H
