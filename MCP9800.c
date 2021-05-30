/*
 * MCP9800.c
 *
 *  Created on: 3 Φεβ 2021
 *      Author: Fotis Zafeiris
 */

#include "MCP9800.h"
//#include "stm32f4xx_hal.h"
#include <stdio.h>

// General Functions

void MCP_Write_16bit(I2C_HandleTypeDef hi2c,uint8_t reg, uint8_t MSB, uint8_t LSB ){
	uint8_t buf[2];
	buf[0]=reg;
	buf[1]=MSB;
	buf[2]=LSB;
	HAL_I2C_Master_Transmit(&hi2c, MCP_ADDR, buf, 3, HAL_MAX_DELAY);
	return;
}

void MCP_Write_8bit(I2C_HandleTypeDef hi2c,uint8_t reg, uint8_t data){
	uint8_t buf[2];
	buf[0]=reg;
	buf[1]=data;
	HAL_I2C_Master_Transmit(&hi2c, MCP_ADDR, buf, 2, HAL_MAX_DELAY);
	return;
}

uint16_t MCP_Read_16bit(I2C_HandleTypeDef hi2c,uint8_t reg){
	uint8_t buf[2];
	uint16_t ret;
	HAL_I2C_Master_Transmit(&hi2c, MCP_ADDR, &reg, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c, MCP_ADDR, buf, 2, HAL_MAX_DELAY);
	ret = ((buf[0]<<8)&0xFF00) | (buf[1]&0xFF);
	return ret;
}

uint8_t MCP_Read_8bit(I2C_HandleTypeDef hi2c,uint8_t reg){
	uint8_t ret;
	HAL_I2C_Master_Transmit(&hi2c, MCP_ADDR, &reg, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c, MCP_ADDR, &ret, 1, HAL_MAX_DELAY);
	return ret;
}

void bytes(uint16_t two_bytes,uint8_t MSB, uint8_t LSB){
	MSB= (two_bytes>>8)&0xFF;
	LSB = two_bytes&0xFF;
	return;
}


uint16_t MCP_get_temp(I2C_HandleTypeDef hi2c){

	return MCP_Read_16bit(hi2c, TEMP_REG);

}

int8_t MCP_get_Temp_Lim(I2C_HandleTypeDef hi2c){
	int8_t ret;
	int neg=1;
	uint16_t T_Lim=MCP_Read_16bit(hi2c, TLIM_REG);
	if(T_Lim>0x7FFF){
			neg=-1;
	}
	ret = neg*((T_Lim>>8)&0xFF);
	return ret;
}

void MCP_set_Temp_Lim(I2C_HandleTypeDef hi2c,int8_t T){
	//Firstly we check if T meets the requirements
	if(T>125) T=125;
	if(T<0) T=0;
	MCP_Write_16bit(hi2c, TLIM_REG, T, 0x00);
	return;
}

int8_t MCP_get_Temp_Hys(I2C_HandleTypeDef hi2c){
	int8_t ret;
	int neg=1;
	uint16_t T_Hys=MCP_Read_16bit(hi2c, TH_REG);
	if(T_Hys>0x7FFF){
			neg=-1;
	}
	ret = neg*((T_Hys>>8)&0xFF);
	return ret;
}

void MCP_set_Temp_Hys(I2C_HandleTypeDef hi2c,int8_t T){
	//Firstly we check if T meets the requirements
		if(T>125) T=125;
		if(T<0) T=0;
		MCP_Write_16bit(hi2c, TH_REG, T, 0x00);
		return;
}
/*
 Function to set the configuration register:
 @param one_shot -> whether we want one shot mode ONE_SHOT_EN if yes 0x00 if not. To enable one shot we need to be in shutdown mode first
 @param bit_res  -> the bit resolution we want (BIT_RES_9,BIT_RES_10,BIT_RES_11,BIT_RES_12)
 @param fault_q  -> the amount of measurements needed before alert(FAULT_Q_1,FAULT_Q_2,FAULT_Q_4,FAULT_Q_6)
 @param alert_pol-> the polarity of ALERT pin, ALERT_HIGH for active high or ALERT_LOW for active low
 @param mode	 -> INT_MODE for interrupt mode, COMP_MODE for comparator mode
 @param shutdown -> SHUTDOWN to enable shutdown mode or 0x00 to disable
  */
void MCP_set_Conf(I2C_HandleTypeDef hi2c, uint8_t one_shot, uint8_t bit_res, uint8_t fault_q, uint8_t alert_pol, uint8_t mode, uint8_t shutdown){
	uint8_t buf;
	buf= one_shot|bit_res|fault_q|alert_pol|mode|shutdown;
	MCP_Write_8bit(hi2c, CONF_REG, buf);
	return;
}

// Functions for my system and use

void my_MCP_begin(I2C_HandleTypeDef hi2c, int8_t T_Lim, int8_t T_Hys, uint8_t bit_res, uint8_t fault_q ){
	//1.Set T_Lim
	MCP_set_Temp_Lim(hi2c, T_Lim);
	//2.Set T_Hys
	MCP_set_Temp_Hys(hi2c, T_Hys);
	//3.Set Configuration Register and put MCP in shutdown(ready for one shot)
	MCP_set_Conf(hi2c, 0, bit_res, fault_q, ALERT_HIGH, INT_MODE, 0);
}

//float my_MCP_get_Temp(I2C_HandleTypeDef hi2c){
	//uint8_t conf=MCP_Read_8bit(hi2c, CONF_REG);
	//conf = conf|ONE_SHOT_EN;
	//MCP_Write_8bit(hi2c, CONF_REG, conf);
	//float ret = MCP_get_temp(hi2c);
	//return ret;
//}
