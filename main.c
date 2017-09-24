#include "integer.h"
#include <MKL25Z4.h>
#include "spi_io.h"
#include "sd_io.h"
#include "LEDs.h"
#include "debug.h"

SDRESULTS Write2SD (SD_DEV *dev, void *dat, DWORD sector);
SDRESULTS InitSD(SD_DEV *dev);

SD_DEV dev[1];          // Create device descriptor
uint8_t buffer[512];    // Example of your buffer data
volatile uint32_t sum = 0;

void test_write(void) {
	// Write test data to given block (sector_num) in flash. 
	// Read it back, compute simple checksum to confirm it is correct.
	
	int i;
	DWORD sector_num = 0x32; // Manual wear leveling
  SDRESULTS res;
	
	PTB->PSOR = MASK(DBG_7);
	// Erase buffer
	for (i=0; i<SD_BLK_SIZE; i++)
		buffer[i] = 0;
	
	// Load sample data into buffer
	buffer[0] = 0xDE;
	buffer[1] = 0xAD;
	buffer[2] = 0xC0;
	buffer[3] = 0xDE;
	
	buffer[508] = 0xFE;
	buffer[509] = 0xED;
	buffer[510] = 0xCA;
	buffer[511] = 0xFE;
	
	res = InitSD(dev);
  if(res==SD_OK) {
		// Change the data in this sector
		res = Write2SD(dev, (void*)buffer, sector_num);
		if(res==SD_OK) {
			Control_RGB_LEDs(0,0,1);	// Blue - written ok
			// erase buffer
			for (i=0; i<SD_BLK_SIZE; i++)
				buffer[i] = 0;
			// read block again
			res = SD_Read(dev, (void*)buffer, sector_num, 0, 512);
			
			if(res==SD_OK) {
				for (i = 0, sum = 0; i < SD_BLK_SIZE; i++)
					sum += buffer[i];
			
				if (sum == 0x06DC)
					Control_RGB_LEDs(0,1,0); // Green - read was OK
				else
					Control_RGB_LEDs(1,0,0); // Red - checksum failed
			} else {
				Control_RGB_LEDs(1,0,0); // Red - read failed
			}
		} else {
				Control_RGB_LEDs(1,0,1); // Magenta - write failed
		}
	} else {
		while (1) {
			Control_RGB_LEDs(1,0,0); // Red - init failed
			SPI_Timer_On(250);
			while (SPI_Timer_Status() == TRUE)
				;
			SPI_Timer_Off();
			Control_RGB_LEDs(1,1,0); // Yellow - init failed
			SPI_Timer_On(250);
			while (SPI_Timer_Status() == TRUE)
				;
			SPI_Timer_Off();
		}
	}
	//while (1)
		//;
	
	PTB->PCOR = MASK(DBG_7);
}

SDRESULTS InitSD(SD_DEV *dev){
	int res;
	PTB->PSOR = MASK(DBG_0);
	do{
		res = SD_Init(dev);
	}while(res < 0);
	PTB->PCOR = MASK(DBG_0);
	return res;
}

SDRESULTS Write2SD (SD_DEV *dev, void *dat, DWORD sector){
	int res;
	do{
		res = SD_Write(dev, dat, sector);
	}while(res < 0);
	return res;
}

int main(void)
{
	Init_Debug_Signals();
	Init_RGB_LEDs();
	Control_RGB_LEDs(1,1,0);	// Yellow - starting up
	
 	// Test function to write a block and then read back, verify
	test_write();
  
	while (1)
		;
}
