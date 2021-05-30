/*
 * LSM9DS1.c
 *
 *  Created on: 12 Φεβ 2021
 *      Author: Kostas Papafotis, modified by Fotis Zafeiris
 */

//Includes
#include "LSM9DS1.h"

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define _BOOL(x) (((x)>0) ? 1:0)

static GPIO_TypeDef			*LSM9DS1_CS_AG_PORT;
static GPIO_TypeDef			*LSM9DS1_CS_M_PORT;
static uint16_t				LSM9DS1_CS_AG_PIN;
static uint16_t				LSM9DS1_CS_M_PIN;

//SPI Handle
static SPI_HandleTypeDef	LSM9DS1_hspi;

uint8_t command[10];

/*****************************************************************************************************/
/***************************************ACCELEROMETER - GYROCOPE**************************************/
/*****************************************************************************************************/

void LSM_CS_AG(int state){
	if(state) HAL_GPIO_WritePin(LSM9DS1_CS_AG_PORT, LSM9DS1_CS_AG_PIN, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(LSM9DS1_CS_AG_PORT, LSM9DS1_CS_AG_PIN, GPIO_PIN_RESET);
}

void LSM_Read_AG_Reg(uint8_t reg, uint8_t* data, uint16_t size){

	//CS Low
	LSM_CS_AG(0);
	//Mask Command
	command[0] = (0b10000000)|reg;
	//Send the register about to be read
	HAL_SPI_Transmit(&LSM9DS1_hspi, command, 1, HAL_MAX_DELAY);
	//Receive the data
	HAL_SPI_Receive(&LSM9DS1_hspi, data, size, HAL_MAX_DELAY);
	//CS HIGH
	LSM_CS_AG(1);

}

void LSM_Write_AG_Reg(uint8_t reg, uint8_t* data, uint16_t size){

	//CS Low
	LSM_CS_AG(0);
	//Mask Command
	command[0] = (0b01111111)&reg;
	//Write the data
	HAL_SPI_Transmit(&LSM9DS1_hspi, command, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&LSM9DS1_hspi, data, size, HAL_MAX_DELAY);
	//CS HIGH
	LSM_CS_AG(1);

}

void LSM_Modify_AG_Reg(uint8_t reg, uint8_t pBuffer, uint8_t mask){

	uint8_t regValue;
	//Read the value
	LSM_Read_AG_Reg(reg, &regValue, 1);
	//Mask the value
	regValue &= ~mask;
	//safety and to clear bits in pBuffer not in the mask
	pBuffer &= mask;
	regValue |= pBuffer;
	//Write the value
	LSM_Write_AG_Reg(reg, &regValue, 1);

}

//OUTPUT DATA RATE
void LSM_XL_SetOdr(uint8_t ODR){
	/*
	LSM9DS1_CTRL_REG6_XL_ODR_XL_POWERDOWN
	LSM9DS1_CTRL_REG6_XL_ODR_XL_10HZ
	LSM9DS1_CTRL_REG6_XL_ODR_XL_50HZ
	LSM9DS1_CTRL_REG6_XL_ODR_XL_119HZ
	LSM9DS1_CTRL_REG6_XL_ODR_XL_238HZ
	LSM9DS1_CTRL_REG6_XL_ODR_XL_476HZ
	LSM9DS1_CTRL_REG6_XL_ODR_XL_952HZ
	*/
	LSM_Modify_AG_Reg(LSM9DS1_CTRL_REG6_XL, ODR, LSM9DS1_CTRL_REG6_XL_ODR_XL_MASK);

}

void LSM_G_SetOdr(uint8_t ODR){
	/*
	LSM9DS1_CTRL_REG1_G_ODR_G_POWERDOWN
	LSM9DS1_CTRL_REG1_G_ODR_G_14p9HZ
	LSM9DS1_CTRL_REG1_G_ODR_G_59p5HZ
	LSM9DS1_CTRL_REG1_G_ODR_G_119HZ
	LSM9DS1_CTRL_REG1_G_ODR_G_238HZ
	LSM9DS1_CTRL_REG1_G_ODR_G_476HZ
	LSM9DS1_CTRL_REG1_G_ODR_G_952HZ
	*/
	LSM_Modify_AG_Reg(LSM9DS1_CTRL_REG1_G, ODR, LSM9DS1_CTRL_REG1_G_ODR_G_MASK);

}

//FULL SCALE
void LSM_XL_SetFs(uint8_t fullscale){
	/*
	LSM9DS1_CTRL_REG6_XL_FS_XL_2G
	LSM9DS1_CTRL_REG6_XL_FS_XL_4G
	LSM9DS1_CTRL_REG6_XL_FS_XL_8G
	LSM9DS1_CTRL_REG6_XL_FS_XL_16G
	 */
	LSM_Modify_AG_Reg(LSM9DS1_CTRL_REG6_XL, fullscale, LSM9DS1_CTRL_REG6_XL_FS_XL_MASK);

}

void LSM_G_SetFs(uint8_t fullscale){
	/*
	LSM9DS1_CTRL_REG1_G_FS_G_245DPS
	LSM9DS1_CTRL_REG1_G_FS_G_500DPS
	LSM9DS1_CTRL_REG1_G_FS_G_2000DPS
	 */
	LSM_Modify_AG_Reg(LSM9DS1_CTRL_REG1_G, fullscale, LSM9DS1_CTRL_REG1_G_FS_G_MASK);

}

//FIFO
void LSM_SetFIFO(uint8_t fifo){
	/*
	LSM9DS1_FIFO_CTRL_FMODE_BYPASS
	LSM9DS1_FIFO_CTRL_FMODE_FIFO
	LSM9DS1_FIFO_CTRL_FMODE_CONT_FIFO
	LSM9DS1_FIFO_CTRL_FMODE_BYPASS_CONT
	LSM9DS1_FIFO_CTRL_FMODE_CONT
	*/
	LSM_Modify_AG_Reg(LSM9DS1_FIFO_CTRL, fifo, LSM9DS1_FIFO_CTRL_FMODE_MASK);

}

uint16_t LSM_XL_GetOdr(){

	uint8_t odr;

	LSM_Read_AG_Reg(LSM9DS1_CTRL_REG6_XL, &odr, 1);

	odr= odr & LSM9DS1_CTRL_REG6_XL_ODR_XL_MASK;
	odr= odr >> LSM9DS1_CTRL_REG6_XL_ODR_XL_SHIFT;

	if (odr == 1)
	 {
		return 10;
	 }
	else if (odr == 2)
	{
		return 50;
	}
	else if (odr == 3)
	{
	return 119;
	}
	else if (odr == 4)
	{
		return 238;
	}
	else if (odr == 5)
	{
		return 476;
	}
	else if (odr == 6)
	{
	  return 952;
	}
	else
	{
		return 0;
	}

}

uint16_t LSM_G_GetOdr(){

	uint8_t odr;

	LSM_Read_AG_Reg(LSM9DS1_CTRL_REG1_G, &odr, 1);

	odr= odr & LSM9DS1_CTRL_REG1_G_ODR_G_MASK;
	odr= odr >> LSM9DS1_CTRL_REG1_G_ODR_G_SHIFT;

	if (odr == 1)
	 {
		return 14;
	 }
	else if (odr == 2)
	{
		return 59;
	}
	else if (odr == 3)
	{
	return 119;
	}
	else if (odr == 4)
	{
		return 238;
	}
	else if (odr == 5)
	{
		return 476;
	}
	else if (odr == 6)
	{
	  return 952;
	}
	else
	{
		return 0;
	}
}

uint16_t LSM_XL_GetFs(){

	uint8_t fs;

	LSM_Read_AG_Reg(LSM9DS1_CTRL_REG6_XL, &fs, 1);

	fs= fs & LSM9DS1_CTRL_REG6_XL_FS_XL_MASK;
	fs= fs >> LSM9DS1_CTRL_REG6_XL_FS_XL_SHIFT;


		if (fs == 1)
		{
			return 16;
		}
		else if (fs == 2)
		{
		return 4;
		}
		else if (fs == 3)
		{
			return 8;
		}
		else
		{
			return 2;
		}

}

uint16_t LSM_G_GetFs(){

	uint8_t fs;

	LSM_Read_AG_Reg(LSM9DS1_CTRL_REG1_G, &fs, 1);

	fs= fs & LSM9DS1_CTRL_REG1_G_FS_G_MASK;
	fs= fs >> LSM9DS1_CTRL_REG1_G_FS_G_SHIFT;


		if (fs == 1)
		{
			return 500;
		}
		else if (fs == 3)
		{
			return 2000;
		}
		else
		{
			return 245;
		}

}

void LSM_GetAcc(uint16_t* data){

	uint8_t temp[6];

	LSM_Read_AG_Reg(LSM9DS1_OUT_X_L_XL, temp, 6);
	data[0] = (temp[1]<<8)|temp[0];
	data[1] = (temp[3]<<8)|temp[2];
	data[2] = (temp[5]<<8)|temp[4];

}

void LSM_GetRawAcc(uint8_t* data){

	LSM_Read_AG_Reg(LSM9DS1_OUT_X_L_XL, data, 6);
}

void LSM_GetRawAG(uint8_t* data){

	LSM_Read_AG_Reg(LSM9DS1_OUT_X_L_G, data, 22);

}

void LSM_GetGyro(uint16_t* data){

	uint8_t temp[6];

	LSM_Read_AG_Reg(LSM9DS1_OUT_X_L_G, temp, 6);
	data[0] = (temp[1]<<8)|temp[0];
	data[1] = (temp[3]<<8)|temp[2];
	data[2] = (temp[5]<<8)|temp[4];
	//printf("X-axis: %d  Y-axis: %d  Z-axis: %d  \r\n", data[0],data[1],data[2]);

}

void LSM_GetRawGyro(uint8_t* data){

	LSM_Read_AG_Reg(LSM9DS1_OUT_X_L_G, data, 6);
}

void LSM_SetAG_Int1(){

	uint8_t temp = LSM9DS1_INT1_CTRL_INT1_DRDY_XL;
	LSM_Write_AG_Reg(LSM9DS1_INT1_CTRL, &temp, 1);

}

void LSM_SetAG_Int2(){

	uint8_t temp = LSM9DS1_INT2_CTRL_INT2_DRDY_G;
	LSM_Write_AG_Reg(LSM9DS1_INT2_CTRL, &temp, 1);

}

void LSM_SetAG(uint8_t ODRA, uint8_t ODRG, uint8_t fsa, uint8_t fsg, uint8_t fifo){

	LSM_XL_SetOdr(ODRA);
	LSM_G_SetOdr(ODRG);

	LSM_XL_SetFs(fsa);
	LSM_G_SetFs(fsg);

	LSM_SetFIFO(fifo);

	LSM_SetAG_Int1();
	LSM_SetAG_Int2();
}

/*****************************************************************************************************/
/*********************************************MAGNETOMETER********************************************/
/*****************************************************************************************************/

void LSM_CS_M(int state){
	if(state) HAL_GPIO_WritePin(LSM9DS1_CS_M_PORT, LSM9DS1_CS_M_PIN, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(LSM9DS1_CS_M_PORT, LSM9DS1_CS_M_PIN, GPIO_PIN_RESET);
}

void LSM_Read_M_Reg(uint8_t reg, uint8_t* data, uint16_t size){

	//CS Low
	LSM_CS_M(0);
	//Mask Command
	command[0] = (0b11000000)|reg;
	//Send the register about to be read
	HAL_SPI_Transmit(&LSM9DS1_hspi, command, 1, HAL_MAX_DELAY);
	//Receive the data
	HAL_SPI_Receive(&LSM9DS1_hspi, data, size, HAL_MAX_DELAY);
	//CS HIGH
	LSM_CS_M(1);

}

void LSM_Write_M_Reg(uint8_t reg, uint8_t* data, uint16_t size){

	//CS Low
	LSM_CS_M(0);
	//Mask Command
	command[0] = (0b01111111)&(reg|(0b01000000));
	//Write the data
	HAL_SPI_Transmit(&LSM9DS1_hspi, command, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&LSM9DS1_hspi, data, size, HAL_MAX_DELAY);
	//CS HIGH
	LSM_CS_M(1);

}

void LSM_Modify_M_Reg(uint8_t reg, uint8_t pBuffer, uint8_t mask){

	uint8_t regValue;
	//Read the value
	LSM_Read_M_Reg(reg, &regValue, 1);
	//Mask the value
	regValue &= ~mask;
	//safety and to clear bits in pBuffer not in the mask
	pBuffer &= mask;
	regValue |= pBuffer;
	//Write the value
	LSM_Write_M_Reg(reg, &regValue, 1);

}

void LSM_M_SetOdr(uint8_t ODR){
	/*
	 LSM9DS1_CTRL_REG1_M_DO_0p625HZ
	 LSM9DS1_CTRL_REG1_M_DO_1p25HZ
	 LSM9DS1_CTRL_REG1_M_DO_2p5HZ
	 LSM9DS1_CTRL_REG1_M_DO_5HZ
	 LSM9DS1_CTRL_REG1_M_DO_10HZ
	 LSM9DS1_CTRL_REG1_M_DO_20HZ
	 LSM9DS1_CTRL_REG1_M_DO_40HZ
	 LSM9DS1_CTRL_REG1_M_DO_80HZ
	*/
	LSM_Modify_M_Reg(LSM9DS1_CTRL_REG3_M, 0x00, LSM9DS1_CTRL_REG3_M_MD_MASK);
	LSM_Modify_M_Reg(LSM9DS1_CTRL_REG1_M, ODR, LSM9DS1_CTRL_REG1_M_DO_MASK);
}

void LSM_M_SetPerformance(uint8_t perf){
	/*
	LSM9DS1_CTRL_REG1_M_OM_LOW
	LSM9DS1_CTRL_REG1_M_OM_MEDIUM
	LSM9DS1_CTRL_REG1_M_OM_HIGH
	LSM9DS1_CTRL_REG1_M_OM_ULTRAHIGH

	LSM9DS1_CTRL_REG4_M_OMZ_LOW
	LSM9DS1_CTRL_REG4_M_OMZ_MEDIUM
	LSM9DS1_CTRL_REG4_M_OMZ_HIGH
	LSM9DS1_CTRL_REG4_M_OMZ_ULTRAHIGH
	*/

	LSM_Modify_M_Reg(LSM9DS1_CTRL_REG1_M, LSM9DS1_CTRL_REG1_M_OM_ULTRAHIGH, LSM9DS1_CTRL_REG1_M_OM_MASK);
	LSM_Modify_M_Reg(LSM9DS1_CTRL_REG4_M, LSM9DS1_CTRL_REG4_M_OMZ_ULTRAHIGH, LSM9DS1_CTRL_REG4_M_OMZ_MASK);

}

uint8_t LSM_M_GetOdr(){

	uint8_t odr;

	LSM_Read_M_Reg(LSM9DS1_CTRL_REG1_M, &odr, 1);

	odr= odr & 	LSM9DS1_CTRL_REG1_M_DO_MASK;
	odr= odr >> LSM9DS1_CTRL_REG1_M_DO_SHIFT;

		if (odr == 0)
		 {
			return 1;
		 }
		else if (odr == 1)
		{
			return 2;
		}
		else if (odr == 2)
		{
		return 3;
		}
		else if (odr == 3)
		{
			return 5;
		}
		else if (odr == 4)
		{
			return 10;
		}
		else if (odr == 5)
		{
		  return 20;
		}
		else if (odr == 6)
		{
		  return 40;
		}
		else if (odr == 7)
		{
		  return 80;
		}
		else
		{
			return 0;
		}
}

void LSM_M_SetFs(uint8_t fullscale){
	/*
	 LSM9DS1_CTRL_REG2_M_FS_4GAUSS
	 LSM9DS1_CTRL_REG2_M_FS_8GAUSS
	 LSM9DS1_CTRL_REG2_M_FS_12GAUSS
	 LSM9DS1_CTRL_REG2_M_FS_16GAUSS
	 */

	LSM_Modify_M_Reg(LSM9DS1_CTRL_REG2_M, fullscale, LSM9DS1_CTRL_REG2_M_FS_MASK);
}

uint8_t LSM_M_GetFs(){

	uint8_t fs;

	LSM_Read_M_Reg(LSM9DS1_CTRL_REG2_M, &fs, 1);
	fs= fs & LSM9DS1_CTRL_REG2_M_FS_MASK;
	fs= fs >> LSM9DS1_CTRL_REG2_M_FS_SHIFT;


		if (fs == 0)
		{
			return 4;
		}
		else if (fs == 1)
		{
			return 8;
		}
		else if (fs==2)
		{
			return 12;
		}
		else
		{
			return 16;
		}
}

void LSM_M_Set(uint8_t odr, uint8_t fullscale, uint8_t perf){

	LSM_M_SetOdr(odr);
	LSM_M_SetFs(fullscale);
	LSM_M_SetPerformance(perf);

	uint8_t temp=0x00;
	LSM_Write_M_Reg(LSM9DS1_CTRL_REG3_M, &temp, 1);
}

void LSM_GetM(uint16_t* data){

	uint8_t temp[6];

	LSM_Read_M_Reg(LSM9DS1_OUT_X_L_M, temp, 6);

	data[0] = (temp[1]<<8)|temp[0];
	data[1] = (temp[3]<<8)|temp[2];
	data[2] = (temp[5]<<8)|temp[4];

}

void LSM_GetRawM(uint8_t* data){

	LSM_Read_M_Reg(LSM9DS1_OUT_X_L_M, data, 6);
}

//General Functions

void LSM_begin(SPI_HandleTypeDef hspi, GPIO_TypeDef *AGPORT, GPIO_TypeDef *MPORT, uint16_t CSAG, uint16_t CSM ){

	memcpy(&LSM9DS1_hspi,&hspi,sizeof(hspi));
	LSM9DS1_CS_AG_PORT=AGPORT;
	LSM9DS1_CS_M_PORT=MPORT;
	LSM9DS1_CS_AG_PIN=CSAG;
	LSM9DS1_CS_M_PIN=CSM;


}
