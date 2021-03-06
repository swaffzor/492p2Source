/*
 *  File: sd_io.c
 *  Author: Nelson Lombardo
 *  Year: 2015
 *  e-mail: nelson.lombardo@gmail.com
 *  License at the end of file.
 */
 
// Modified 2017 by Alex Dean (agdean@ncsu.edu) for teaching FSMs
// - Removed support for PC development (_M_IX86)
// - Split single-line loops & conditionals in source code for readability
// - Fused loops in SD_Read 
// _ Inlined __SD_Write_Block into SD_Write

#include "sd_io.h"
#include <MKL25Z4.h>
#include "debug.h"
#include "customSD.h"

/******************************************************************************
 Private Methods - Direct work with SD card
******************************************************************************/

DWORD __SD_Power_Of_Two(BYTE e)
{
    DWORD partial = 1;
    BYTE idx;
    for(idx=0; idx!=e; idx++) partial *= 2;
    return(partial);
}

inline void __SD_Assert(void){
    SPI_CS_Low();
}

inline void __SD_Deassert(void){
    SPI_CS_High();
}

void __SD_Speed_Transfer(BYTE throttle) {
    if(throttle == HIGH) SPI_Freq_High();
    else SPI_Freq_Low();
}

BYTE __SD_Send_Cmd(BYTE cmd, DWORD arg)
{
    BYTE crc, res;

	// ACMD«n» is the command sequense of CMD55-CMD«n»
    if(cmd & 0x80) {
        cmd &= 0x7F;
        res = __SD_Send_Cmd(CMD55, 0);
        if (res > 1) 
					return (res);
    }

    // Select the card
    __SD_Deassert();
    SPI_RW(0xFF);
    __SD_Assert();
    SPI_RW(0xFF);

    // Send complete command set
    SPI_RW(cmd);                        // Start and command index
    SPI_RW((BYTE)(arg >> 24));          // Arg[31-24]
    SPI_RW((BYTE)(arg >> 16));          // Arg[23-16]
    SPI_RW((BYTE)(arg >> 8 ));          // Arg[15-08]
    SPI_RW((BYTE)(arg >> 0 ));          // Arg[07-00]

    // CRC?
    crc = 0x01;                         // Dummy CRC and stop
    if(cmd == CMD0) 
			crc = 0x95;         // Valid CRC for CMD0(0)
    if(cmd == CMD8) 
			crc = 0x87;         // Valid CRC for CMD8(0x1AA)
    SPI_RW(crc);

    // Receive command response
    // Wait for a valid response in timeout of 5 milliseconds
    SPI_Timer_On(5);
    do {
        res = SPI_RW(0xFF);
    } while((res & 0x80)&&(SPI_Timer_Status()==TRUE));
    SPI_Timer_Off();
		
    // Return with the response value
    return(res);
}

DWORD __SD_Sectors (SD_DEV *dev)
{
    BYTE csd[16];
    BYTE idx;
    DWORD ss = 0;
    WORD C_SIZE = 0;
    BYTE C_SIZE_MULT = 0;
    BYTE READ_BL_LEN = 0;
    if(__SD_Send_Cmd(CMD9, 0)==0) 
    {
        // Wait for response
        while (SPI_RW(0xFF) == 0xFF);
        for (idx=0; idx!=16; idx++) 
					csd[idx] = SPI_RW(0xFF);
        // Dummy CRC
        SPI_RW(0xFF);
        SPI_RW(0xFF);
        SPI_Release();
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

        return (ss);
    } else return (0); // Error
}

/******************************************************************************
 Public Methods - Direct work with SD card
******************************************************************************/

int SD_Init(SD_DEV *dev){
    static enum {SA,SB,SC,SD,SE,SE2A,SE2B,SE2B2,SF,SF2,SG,SG2,SH,SZ1,SS1,SS2,SS3,SS4,SS5,SZ2} next_state = SA;
    static BYTE n, cmd, ct = 0, ocr[4];
    static BYTE idx;
    static BYTE init_trys = 0;
		static uint8_t firstTime;
		int res = -1;
		LONG temp;
		
		PTB->PSOR = MASK(DBG_5);
		if((init_trys!=SD_INIT_TRYS)&&(!ct)){
			switch (next_state) {
				case SA:
					// Initialize SPI for use with the memory card
					SPI_Init();
					SPI_CS_High();
					SPI_Freq_Low();
					next_state = SB;
					idx = 0;
				break;
				case SB:
					// 80 dummy clocks
					if(idx++ < 10){
						SPI_RW(0xFF);
					}
					else{
						next_state = SC;
						SPI_Timer_On(500);
						firstTime = 1;
					}
				break;
				case SC:
//					if(firstTime){
//						firstTime = 0;
//					}
//					else{
						if(SPI_Timer_Status()==TRUE){
							PTB->PTOR = MASK(DBG_5);
							PTB->PTOR = MASK(DBG_5);
						}
						else{
							SPI_Timer_Off();
							dev->mount = FALSE;
							SPI_Timer_On(500);
							next_state = SD;
						}
					//}
				break;
				case SD:
					if((__SD_Send_Cmd(CMD0, 0) != 1)&&(SPI_Timer_Status()==TRUE)){
						PTB->PTOR = MASK(DBG_5);
						PTB->PTOR = MASK(DBG_5);
					}
					else{
						SPI_Timer_Off();
						next_state = SE;
					}
				break;
				case SE:
					// Idle state
					if (__SD_Send_Cmd(CMD0, 0) == 1) {
							// SD version 2?
							if (__SD_Send_Cmd(CMD8, 0x1AA) == 1) {
								next_state = SE2A;
								n = 0;
							}
							else{
								next_state = SE2B; //js: doesn't seem to ever hit here
							}
					}
					else{
						//end
						next_state = SH;
					}
				break;
				case SE2A: //check here
					// Get trailing return value of R7 resp
					if(n < 4){
						ocr[n++] = SPI_RW(0xFF);
					}
					else{
						next_state = SF;
					}
				break;
				case SE2B:
					// SD version 1 or MMC?
					if (__SD_Send_Cmd(ACMD41, 0) <= 1){
							// SD version 1
							ct = SDCT_SD1; 
							cmd = ACMD41;
					} else {
							// MMC version 3
							ct = SDCT_MMC; 
							cmd = CMD1;
					}
					// Wait for leaving idle state
					SPI_Timer_On(250);
					next_state = SE2B2;
				break;
				case SE2B2:
					if((SPI_Timer_Status()!=TRUE) || !(__SD_Send_Cmd(cmd, 0))) {
						PTB->PTOR = MASK(DBG_5);
						PTB->PTOR = MASK(DBG_5);
					}
					else{
						SPI_Timer_Off();
						if(SPI_Timer_Status()==FALSE) 
							ct = 0;
						if(__SD_Send_Cmd(CMD59, 0))   
							ct = 0;   // Deactivate CRC check (default)
						if(__SD_Send_Cmd(CMD16, 512)) 
							ct = 0;   // Set R/W block length to 512 bytes
						//end
						next_state = SH;
					}
				break;
				case SF:
					// VDD range of 2.7-3.6V is OK?  
					if ((ocr[2] == 0x01)&&(ocr[3] == 0xAA)){ //js: good here
						// Wait for leaving idle state (ACMD41 with HCS bit)...
						SPI_Timer_On(1000);
						next_state = SF2;
					}
					else{
						//end
						next_state = SH;
					}
				break;
				case SF2:
					if((SPI_Timer_Status()==TRUE) && (__SD_Send_Cmd(ACMD41, 1UL << 30))) {
							PTB->PTOR = MASK(DBG_5);
							PTB->PTOR = MASK(DBG_5);
					}
					else{
						SPI_Timer_Off(); 
						next_state = SG;
					}
				break;
				case SG:
					// CCS in the OCR? 
					// AGD: Delete SPI_Timer_Status call?
					if ((SPI_Timer_Status()==TRUE)&&(__SD_Send_Cmd(CMD58, 0) == 0)){
						n = 0;
						next_state = SG2;
					}
					else{
						//done end
						next_state = SH; //js: got here, why???
					}
				break;
				case SG2:
					if(n < 4){
						ocr[n++] = SPI_RW(0xFF);
					}
					else{
						//done end
						next_state = SZ1;
					}
				break;
				case SH:
					next_state = SA;
					init_trys++;
				break;
				case SZ1:
						// SD version 2?
					dev->cardtype = (ocr[0] & 0x40) ? SDCT_SD2 | SDCT_BLOCK : SDCT_SD2;
					dev->mount = TRUE;
					dev->debug.read = 0;
					dev->debug.write = 0;
					next_state = SS1;
				break;
				case SS1:
					PTB->PSOR = MASK(DBG_0);
					temp = __SD_SectorsFSM(dev);
					if(temp > -1){
						dev->last_sector = temp - 1;
						next_state = SZ2;
					}
					PTB->PCOR = MASK(DBG_0);
				break;
				case SZ2:
					__SD_Speed_Transfer(HIGH); // High speed transfer
					res = (dev->cardtype ? SD_OK : SD_NOINIT);
					ct = 0;
					SPI_Release();
				break;
				default: next_state = SA;
					break;
			}
		}
		
    if(init_trys == SD_INIT_TRYS){
			SPI_Release();
			init_trys = 0;
			res = SD_NOINIT; 
		}
    //return (ct ? SD_OK : SD_NOINIT);
		PTB->PCOR = MASK(DBG_5);
		return res;
}

int SD_Read(SD_DEV *dev, void *dat, DWORD sector, WORD ofs, WORD cnt){
    static enum {SA,SB1,SB2A,SB2B,SB3,SC1,SC2,SD} next_state = SA;
    static BYTE tkn, data;
    static WORD byte_num;
    static int idx;
		int res;
		
		PTB->PSOR = MASK(DBG_2);
    res = -1;
		
		switch (next_state) {
			case SA:
				if ((sector > dev->last_sector)||(cnt == 0)){
					res = SD_PARERR;
				}
				else{
					next_state = SB1;
				}
			break;
			case SB1:
				// Convert sector number to byte address (sector * SD_BLK_SIZE)
				if (__SD_Send_Cmd(CMD17, sector * SD_BLK_SIZE) == 0) {
					SPI_Timer_On(100);  // Wait for data packet (timeout of 100ms)
					next_state = SB2A;
				}
				else{
					res = SD_ERROR;
					next_state = SA;
				}
			break;
			case SB2A:
				tkn = SPI_RW(0xFF);
				next_state = SB2B;
			break;
			case SB2B:
				if((tkn!=0xFF) || (SPI_Timer_Status()!=TRUE)){
					SPI_Timer_Off();
					// Token of single block?
					if(tkn==0xFE) { 
						// AGD: Loop fusion to simplify FSM formation
						byte_num = 0;
						idx = 0;
						next_state = SC1;
					}
					else{
						res = SD_ERROR;
						next_state = SA;
					}
				}
				else{
					next_state = SB2A;
				}
			break;
			case SC1:
				data = SPI_RW(0xff);
				if ((byte_num >= ofs) && (byte_num < ofs+cnt)) {
					 (*((BYTE*)dat + idx)) = data;
					 idx++;
				} // else discard bytes before and after data
				next_state = SC2;
			break;
			case SC2:
				if(++byte_num < SD_BLK_SIZE + 2 ){ // 512 byte block + 2 byte CRC
					next_state = SC1;
				}
				else{
					res = SD_OK;
					next_state = SA;
				}
			break;
			default: next_state = SA;
				break;
		}
		
		if(next_state == SA){
			SPI_Release();
			dev->debug.read++;
		}
			
		PTB->PCOR = MASK(DBG_2);
    return(res);
}

int SD_Write(SD_DEV *dev, void *dat, DWORD sector){
    static enum {SA,SB1,SB2A,SB2B,SB3,SC,SD} next_state = SA;
		static WORD idx = 0;
    static BYTE line;
		static uint8_t firstTime = 1;
		int res = -1;
		BOOL temp;
		
		PTB->PSOR = MASK(DBG_3);
	
		switch (next_state) {
			case SA:
				// Query ok?
				if(sector > dev->last_sector) {
					res = SD_PARERR;
				}
				// Convert sector number to bytes address (sector * SD_BLK_SIZE)
				if(__SD_Send_Cmd(CMD24, sector * SD_BLK_SIZE)==0) {
					next_state = SB1;
				}
				else{					
					res = SD_ERROR;
					next_state = SA;
				}
			break;
			case SB1:
				// Send token (single block write)
				SPI_RW(0xFE);		//split into own state???
				next_state = SB2A;
			break;
			case SB2A:
				// Send block data
				if(SPI1_S & SPI_S_SPTEF_MASK){
					SPI1_D = (*((BYTE*)dat + idx));
					next_state = SB2B;
					if(idx++ == SD_BLK_SIZE){
						next_state = SB3;
						idx = 0;
					}
				}
			break;
			case SB2B:
				if(SPI1_S & SPI_S_SPRF_MASK){
					next_state = SB2A;
				}
			break;
			case SB3:
				/* Dummy CRC */
				SPI_RW(0xFF);
				SPI_RW(0xFF);
				// If not accepted, returns the reject error
				if((SPI_RW(0xFF) & 0x1F) != 0x05) {
					res = SD_REJECT;
					next_state = SA;
				}
				else{
					next_state = SC;
					firstTime = 1;
				}
			break;
			case SC:
				if(firstTime){
					// Waits until finish of data programming with a timeout
					SPI_Timer_On(SD_IO_WRITE_TIMEOUT_WAIT);
					firstTime = 0;
				}
				line = SPI_RW(0xFF);
				temp = SPI_Timer_Status();
				if((line!=0)||(temp!=TRUE)){
					next_state = SD;
				}
				else{
					PTB->PTOR = MASK(DBG_3);
					PTB->PTOR = MASK(DBG_3);
				}
			break;
			case SD:
				dev->debug.write++;
				if(line==0){
					res = SD_BUSY;
				}
				else {
					res = SD_OK;	
				}
			break;
			
			default: next_state = SA;
				break;
		}
		
		PTB->PCOR = MASK(DBG_3);
		return res;
}

SDRESULTS SD_Status(SD_DEV *dev)
{
    return(__SD_Send_Cmd(CMD0, 0) ? SD_OK : SD_NORESPONSE);
}

// «sd_io.c» is part of:
/*----------------------------------------------------------------------------/
/  ulibSD - Library for SD cards semantics            (C)Nelson Lombardo, 2015
/-----------------------------------------------------------------------------/
/ ulibSD library is a free software that opened under license policy of
/ following conditions.
/
/ Copyright (C) 2015, ChaN, all right reserved.
/
/ 1. Redistributions of source code must retain the above copyright notice,
/    this condition and the following disclaimer.
/
/ This software is provided by the copyright holder and contributors "AS IS"
/ and any warranties related to this software are DISCLAIMED.
/ The copyright owner or contributors be NOT LIABLE for any damages caused
/ by use of this software.
/----------------------------------------------------------------------------*/

// Derived from Mister Chan works on FatFs code (http://elm-chan.org/fsw/ff/00index_e.html):
/*----------------------------------------------------------------------------/
/  FatFs - FAT file system module  R0.11                 (C)ChaN, 2015
/-----------------------------------------------------------------------------/
/ FatFs module is a free software that opened under license policy of
/ following conditions.
/
/ Copyright (C) 2015, ChaN, all right reserved.
/
/ 1. Redistributions of source code must retain the above copyright notice,
/    this condition and the following disclaimer.
/
/ This software is provided by the copyright holder and contributors "AS IS"
/ and any warranties related to this software are DISCLAIMED.
/ The copyright owner or contributors be NOT LIABLE for any damages caused
/ by use of this software.
/----------------------------------------------------------------------------*/
