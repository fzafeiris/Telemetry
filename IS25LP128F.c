/*
 * IS25LP128F.c
 *
 *  Created on: 15 Φεβ 2021
 *      Author: Φώτης
 */

#include "IS25LP128F.h"


#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

extern SPI_HandleTypeDef hspi3;

static GPIO_TypeDef *FLASH_EN_PORT;
static uint16_t FLASH_EN_PIN;

//Functions
//1.IS25 CS PIN CONFIGURATION
void IS25_CS(int state){
	if(state) HAL_GPIO_WritePin(FLASH_EN_PORT, FLASH_EN_PIN, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(FLASH_EN_PORT, FLASH_EN_PIN, GPIO_PIN_RESET);
}

//2.WAIT TILL WRITE IS COMPLETE
void IS25_available(){

	int available = 0;
	do{
	uint8_t buf = READSTATUS;
	IS25_CS(0);
	HAL_SPI_Transmit(&hspi3, &buf, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi3, &buf, 1, HAL_MAX_DELAY);
	IS25_CS(1);
	if((buf&0x01)==0x00) available=1;
	else HAL_Delay(1);

	}while(available == 0 );
}

//3.IS25 NORMAL READ OPERATION
void IS25_NORD(uint32_t addr, uint32_t numofbytes, uint8_t *buf){

	uint8_t address[3];
	// WAIT UNTIL PREVIOUS TASK IS DONE
	IS25_available();
	//SPLIT ADDRESS INTO BYTES
	address[0]=NORD;
	address[1]=(addr&0xFF0000)>>16;
	address[2]=(addr&0xFF00)>>8;
	address[3]=(addr&0xFF);
	//SEND COMMAND AND ADDRESS
	IS25_CS(0);
	HAL_SPI_Transmit(&hspi3, address, 4, HAL_MAX_DELAY);
	//RECEIVE DATA
	HAL_SPI_Receive(&hspi3, buf, numofbytes, HAL_MAX_DELAY);
	IS25_CS(1);
	printf("Read %d from flash memory\r\n", *buf);
}

//4.IS25 ENABLE WRITE
void IS25_WEN(){

	uint8_t buf=WEN;
	IS25_CS(0);
	HAL_SPI_Transmit(&hspi3, &buf, 1, HAL_MAX_DELAY);
	IS25_CS(1);
}

//5.IS25 WRITE OPERATION
void IS25_write(uint32_t addr, uint32_t numofbytes, uint8_t *data){

	IS25_WEN();
	uint8_t address[3];
	//SPLIT ADDRESS INTO BYTES
	address[0]=PP;
	address[1]=(addr&0xFF0000)>>16;
	address[2]=(addr&0xFF00)>>8;
	address[3]=(addr&0xFF);
	//SEND COMMAND AND ADDRESS
	IS25_CS(0);
	HAL_SPI_Transmit(&hspi3, address, 4, HAL_MAX_DELAY);
	//SEND DATA TO BE WRITTEN
	HAL_SPI_Transmit(&hspi3, data, numofbytes, HAL_MAX_DELAY);
	//WAIT UNTIL WRITING IS DONE
	IS25_CS(1);
	IS25_available();
}

//6.IS25 SECTOR ERASE
void IS25_SER(uint32_t addr){

	IS25_WEN();
	uint8_t address[3];
	//SPLIT ADDRESS INTO BYTES
	address[0]=SER;
	address[1]=(addr&0xFF0000)>>16;
	address[2]=(addr&0xFF00)>>8;
	address[3]=(addr&0xFF);
	//SEND COMMAND AND ADDRESS
	IS25_CS(0);
	HAL_SPI_Transmit(&hspi3, address, 4, HAL_MAX_DELAY);
	//WAIT UNTIL ERASE IS DONE
	IS25_CS(1);
	IS25_available();
}

//7.IS25 BLOCK ERASE (32KB or 64KB)
void IS25_BER(uint32_t addr, Block_size bsize){

	IS25_WEN();
	uint8_t address[3];
	if(bsize == BSIZE64KB) address[0]=BER64;
	else address[0]=BER32;
	//SPLIT ADDRESS INTO BYTES
	address[1]=(addr&0xFF0000)>>16;
	address[2]=(addr&0xFF00)>>8;
	address[3]=(addr&0xFF);
	//SEND COMMAND AND ADDRESS
	IS25_CS(0);
	HAL_SPI_Transmit(&hspi3, address, 4, HAL_MAX_DELAY);
	IS25_CS(1);
}

//8.IS25 CHIP ERASE
void IS25_CHIP_ERASE(){

	uint8_t buf=CER;
	IS25_WEN();
	IS25_CS(0);
	HAL_SPI_Transmit(&hspi3, &buf, 1, HAL_MAX_DELAY);
	IS25_CS(1);
}

//9.READ FUNCTION REGISTER
uint8_t IS25_getfunctionreg(){

	uint8_t com = 0x48, ret;
	IS25_CS(0);
	HAL_SPI_Transmit(&hspi3, &com, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi3, &ret, 1, HAL_MAX_DELAY);
	IS25_CS(1);
	return ret;
}

//10.GET THE CREDENTIALS OF THE FLASH
void IS25_ID(){

	uint8_t buf[3];
	uint8_t dummy = 0xA5;
	uint8_t DevID,ManID,MemType,Capacity;
	buf[0] = 0xAB;
	buf[1]=dummy; buf[2] = dummy; buf[3] = dummy;
	IS25_CS(0);
	HAL_SPI_Transmit(&hspi3, buf, 4, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi3, buf, 1, HAL_MAX_DELAY);
	IS25_CS(1);
	DevID=buf[0];

	buf[0] = 0x9F;
	IS25_CS(0);
	HAL_SPI_Transmit(&hspi3, buf, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi3, buf, 3, HAL_MAX_DELAY);
	IS25_CS(1);
	ManID=buf[0];
	MemType=buf[1];
	Capacity=buf[2];
	printf("DevID: %X , ManID: %X , MemType %X , Capacity: %X \r\n",DevID, ManID, MemType, Capacity );
}

//11.PERFORM SOFTWARE RESET
void IS25_Reset(){

	uint8_t RSTEN=0x66, RST = 0x99;
	IS25_CS(0);
	HAL_SPI_Transmit(&hspi3, &RSTEN, 1, HAL_MAX_DELAY);
	IS25_CS(1);
	IS25_CS(0);
	HAL_SPI_Transmit(&hspi3, &RST , 1, HAL_MAX_DELAY);
	IS25_CS(1);
}

//12.INITIALIZE THE CHIP BEFORE ANY OTHER OPERATION
void IS25_Init(GPIO_TypeDef *GPIO, uint16_t EN_PIN){

	FLASH_EN_PORT = GPIO;
	FLASH_EN_PIN = EN_PIN;
	IS25_CS(1);
}
