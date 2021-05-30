/*
 * H3LIS100DLTR.c
 *
 *  Created on: 5 Φεβ 2021
 *      Author: Fotis Zafeiris
 */
#include "H3LIS100DLTR.h"


#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define _BOOL(x) (((x)>0) ? 1:0)

static GPIO_TypeDef			*H3LIS_PORT;
static uint16_t				H3LIS_CS_PIN;
//SPI Handle
static SPI_HandleTypeDef	H3LIS_hspi;

//1.Chip Select function
void H3LIS_CS(int state){

	if(state) HAL_GPIO_WritePin(H3LIS_PORT, H3LIS_CS_PIN, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(H3LIS_PORT, H3LIS_CS_PIN, GPIO_PIN_RESET);
}

//2.Read one byte from register
uint8_t H3LIS_read_register(uint8_t reg){

	uint8_t buf[2];
	uint8_t	ret;
	//Put CS low
	H3LIS_CS(0);
	//Transmit register address with the required masking
	buf[0]=reg|H3LIS_SINGLE_READ;
	HAL_SPI_Transmit(&H3LIS_hspi, buf, 1, HAL_MAX_DELAY);
	//Receive the byte
	HAL_SPI_Receive(&H3LIS_hspi, &buf[1], 1, HAL_MAX_DELAY);
	ret=buf[1];
	//Bring CS High
	H3LIS_CS(1);
	return ret;
}

//3.Read multiple bytes from register
void H3LIS_read_registerN(uint8_t reg, uint8_t *buf, uint8_t len){

	uint8_t spibuf;
	spibuf = reg|H3LIS_MULT_READ;
	//Put CS low
	H3LIS_CS(0);
	//Transmit register address with the required masking
	HAL_SPI_Transmit(&H3LIS_hspi, &spibuf, 1, HAL_MAX_DELAY);
	//Receive the bytes
	HAL_SPI_Receive(&H3LIS_hspi, buf, len, HAL_MAX_DELAY);
	//Bring CS High
	H3LIS_CS(1);
	return;
}

//4.Write one byte to register
void H3LIS_write_register(uint8_t reg, uint8_t data){

	uint8_t buf[2];
	//Put CS low
	H3LIS_CS(0);
	//Transmit register address with the required masking
	buf[0]=reg&0x3F;
	buf[1]=data;
	HAL_SPI_Transmit(&H3LIS_hspi, buf, 2, HAL_MAX_DELAY);
	//Bring CS High
	H3LIS_CS(1);
	return;
}

//5.Write multiple bytes to register
void H3LIS_write_registerN(uint8_t reg, uint8_t *buf, uint8_t len){
	uint8_t spibuf[len+1];
	int i;
	//Move buf to spibuf to send all the data in one go
	spibuf[0]=reg&H3LIS_MULT_WRITE;
	for (i=0; i<len; i++){
		spibuf[i+1]= buf[i];
	}
	//Put CS low
	H3LIS_CS(0);
	//Transmit the data
	HAL_SPI_Transmit(&H3LIS_hspi, spibuf, len+1, HAL_MAX_DELAY);
	//Bring CS High
	H3LIS_CS(1);
	return;
}

//6.Check ID
bool H3LIS_check(){
	bool ret=false;
	if(H3LIS_read_register(WHO_AM_I)==0x32) ret=true;
	return ret;
}

//7.Set PowerMode and DataRate
void H3LIS_set_datarate(h3lis_power_mode_e power_mode, h3lis_data_rate_e data_rate){
	uint8_t buf;
	buf =(power_mode<<5) | (data_rate<<3) | (uint8_t)0x07;
	H3LIS_write_register(CTRL_REG_1, buf);
	return;
}

//8.Read 3-axis Acceleration
void H3LIS_get_acceleration(uint8_t *val){
	uint8_t buff[5];
	H3LIS_read_registerN(OUT_X, buff, 5);
	val[0] = (uint8_t)buff[0];
	val[1] = (uint8_t)buff[2];
	val[2] = (uint8_t)buff[4];


}

//9.Sensor Initialization
bool H3LIS_begin(SPI_HandleTypeDef hspi, GPIO_TypeDef *GPIOPORT, uint16_t CSPIN,h3lis_power_mode_e power_mode,h3lis_data_rate_e data_rate){

	//Initialize Ports
	memcpy(&H3LIS_hspi, &hspi, sizeof(hspi));
	H3LIS_PORT=GPIOPORT;
	H3LIS_CS_PIN=CSPIN;
	//Check if the device is present
	bool ret=false;
	if(H3LIS_check()){
		ret=true;
		//Power up Device and pick Mode
		H3LIS_set_datarate(power_mode, data_rate);
		//Set INT1 PIN as data ready signal for the MCU
		H3LIS_write_register(CTRL_REG_3, 0x18);
	}
	return ret;
}

//10.Reset-Reboot function
void H3LIS_reset(){
	H3LIS_write_register(CTRL_REG_2, 0x80);
	return;
}

//11.Read raw acceleration
void H3LIS_get_raw_accel(uint8_t *buff){
	H3LIS_read_registerN(OUT_X-1U, buff, 6);
}
