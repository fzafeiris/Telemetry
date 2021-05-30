/*
 * functions.c
 *
 *  Created on: 6 Απρ 2021
 *      Author: Φώτης
 */

#include "functions.h"



extern I2C_HandleTypeDef hi2c2;
void byteparser(uint16_t twobytes, uint8_t MSB, uint8_t LSB ){

	MSB = (twobytes>>8)&0xff;
	LSB = twobytes&0xff;
}



void GPS_Transmit(const char *pstring){

	HAL_I2C_Master_Transmit(&hi2c2, 0x3A<<1, (uint8_t*)pstring, strlen(pstring), HAL_MAX_DELAY);

}

void GPS_Save(){
	GPS_Transmit(GPS_SAVE_MSG);
	GPS_Transmit(GPS_REBOOT_MSG);
}

void Rocket_Transmit(const void *buf, uint8_t numofbytes){

	if(NRF24_write(buf, numofbytes)){
		printf("Transmission completed");
		HAL_Delay(1000);}

}

void Clear_2Buf(void *buf1,void *buf2){
	memset(buf1, 0, 32);
	memset(buf2, 0, 32);
}
