/*
 * i2cslave.cpp
 *
 *  Created on: 8 Sep 2023
 *      Author: matth
 */

#include "i2cslave.h"
#include "main.h"
#include "string.h"

uint8_t I2C_REGISTERS[15] = {0,0,0,0,0,0,0,0,0,0};

uint8_t datatosend[6] = {2,5,8,9,12,34};

extern I2C_HandleTypeDef hi2c1;

#define RxSIZE  2
uint8_t RxData[RxSIZE];
uint8_t rxcount = 0;

uint8_t bytesTransd = 0;
uint8_t txcount = 0;
uint8_t startPosition = 0;

int countAddr = 0;
int countrxcplt = 0;
int counterror = 0;

void process_data (void)
{
	//do something here
	I2C_REGISTERS[0] = RxData[0];
	if (I2C_REGISTERS[0] == 0x01) {
		I2C_REGISTERS[1] = RxData[1];
		I2C_REGISTERS[0] = 0x00;
	}
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
	if (TransferDirection == I2C_DIRECTION_TRANSMIT)  // if the master wants to transmit the data
	{
		rxcount = 0;
		countAddr++;
		// receive using sequential function.
		HAL_I2C_Slave_Sequential_Receive_IT(hi2c, RxData+rxcount, 1, I2C_FIRST_FRAME);
	}
	else  // if the master requests the data from the slave
	{
		txcount = 0;
		startPosition = RxData[0];
		RxData[0] = 0;  // Reset the start register as we have already copied it
		HAL_I2C_Slave_Seq_Transmit_IT(hi2c, I2C_REGISTERS+startPosition+txcount, 1, I2C_FIRST_FRAME);
	}
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	rxcount++;
	if (rxcount < RxSIZE)
	{
		if (rxcount == RxSIZE-1)
		{
			HAL_I2C_Slave_Sequential_Receive_IT(hi2c, RxData+rxcount, 1, I2C_LAST_FRAME);
		}
		else
		{
			HAL_I2C_Slave_Sequential_Receive_IT(hi2c, RxData+rxcount, 1, I2C_NEXT_FRAME);
		}
	}

	if (rxcount == RxSIZE)
	{
		process_data();
	}

}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	txcount++;
	HAL_I2C_Slave_Seq_Transmit_IT(hi2c, I2C_REGISTERS+startPosition+txcount, 1, I2C_NEXT_FRAME);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	counterror++;
	uint32_t errorcode = HAL_I2C_GetError(hi2c);
	if (errorcode == 4)  // AF error
	{
		if (txcount == 0)  // error is while slave is receiving
			{
				rxcount = 0;  // Reset the rxcount for the next operation
				process_data();
			}
			else // error while slave is transmitting
			{
				txcount = 0;  // Reset the txcount for the next operation
			}
	} else if (errorcode == 2)  // ARLO Error
	{
		HAL_I2C_DeInit(hi2c);
		HAL_I2C_Init(hi2c);
		memset(RxData,'\0',RxSIZE);  // reset the Rx buffer
		rxcount =0;  // reset the count
	}
	else if (errorcode == 1)  // BERR Error
	{
		HAL_I2C_DeInit(hi2c);
		HAL_I2C_Init(hi2c);
		memset(RxData,'\0',RxSIZE);  // reset the Rx buffer
		rxcount =0;  // reset the count
	}
	HAL_I2C_EnableListen_IT(hi2c);
}
