
#include "sd_io.h"
#include <MKL25Z4.h>
#include "debug.h"
#include "customSD.h"

LONG __SD_SectorsFSM (SD_DEV *dev)
{
    static enum {SA,SB,SC,SD,SE,SERR} next_state = SA;
    static BYTE csd[16];
    static BYTE idx;
    static WORD C_SIZE = 0;
    static BYTE C_SIZE_MULT = 0;
    static BYTE READ_BL_LEN = 0;
    DWORD ss = 0;
		LONG res = -1;
	
		switch (next_state) {
			case SA:
				if(__SD_Send_Cmd(CMD9, 0)==0){
					next_state = SB;
				}
				else{
					next_state = SERR;
				}
			break;
			case SB:
				// Wait for response
        if(!(SPI_RW(0xFF) == 0xFF)){
					next_state = SC;
					idx = 0;
				}
			break;
			case SC:
				if(idx < 16){ 
					csd[idx++] = SPI_RW(0xFF);
				}
				else{
					next_state = SD;
				}
			break;
			case SD:
        // Dummy CRC
        SPI_RW(0xFF);
        SPI_RW(0xFF);
        SPI_Release();
				next_state = SE;
			break;
			case SE:
        if(dev->cardtype & SDCT_SD1)
        {
            ss = csd[0];
            // READ_BL_LEN[83:80]: max. read data block length
            READ_BL_LEN = (csd[5] & 0x0F);
            // C_SIZE [73:62]
            C_SIZE = (csd[6] & 0x03);
            C_SIZE <<= 8;
            C_SIZE |= (csd[7]);
            C_SIZE <<= 2;
            C_SIZE |= ((csd[8] >> 6) & 0x03);
            // C_SIZE_MULT [49:47]
            C_SIZE_MULT = (csd[9] & 0x03);
            C_SIZE_MULT <<= 1;
            C_SIZE_MULT |= ((csd[10] >> 7) & 0x01);
        }
        else if(dev->cardtype & SDCT_SD2)
        {
            // C_SIZE [69:48]
            C_SIZE = (csd[7] & 0x3F);
            C_SIZE <<= 8;
            C_SIZE |= (csd[8] & 0xFF);
            C_SIZE <<= 8;
            C_SIZE |= (csd[9] & 0xFF);
            // C_SIZE_MULT [--]. don't exits
            C_SIZE_MULT = 0;
        }
        ss = (C_SIZE + 1);
        ss *= __SD_Power_Of_Two(C_SIZE_MULT + 2);
        ss *= __SD_Power_Of_Two(READ_BL_LEN);
        ss /= SD_BLK_SIZE;
				res = ss;
				next_state = SA;
			break;
			case SERR:
				res = 0;
				next_state = SA;
			break;
			default: next_state = SA;
				break;
		}
		
		return res;
}