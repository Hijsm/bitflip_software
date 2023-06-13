/*
 * memory_control.h
 *
 *  Created on: Jun 17, 2022
 *      Author: Nic Barker
 */

#ifndef INC_MEMORY_CONTROL_H_
#define INC_MEMORY_CONTROL_H_

#include "stm32l1xx_hal.h"
#include "main.h"

extern UART_HandleTypeDef huart1;

void delay_us(uint16_t n);
void Set_IO_Outputs();
void Set_IO_Inputs();

typedef enum {
  READING = 0,
  WRITING
} GPIO_Direction_State;

typedef enum {
  ENABLED = 0,
  DISABLED
} Memory_Enabled_State;

class MemoryController {
	private:
		GPIO_TypeDef *_CE_GPIO_Port;
		uint16_t _CE_Pin;
		GPIO_Direction_State prev_state;
		Memory_Enabled_State en_state;

	public:
		MemoryController(GPIO_TypeDef *CE_GPIO_Port, uint16_t CE_Pin);

		void set_CE(GPIO_TypeDef *CE_GPIO_Port, uint16_t CE_Pin);
		void write(uint32_t addr, uint8_t data);
		uint8_t read(uint32_t addr);
		inline void set_addr(uint32_t addr);
		void clear();
		void flood(uint8_t data);
		uint32_t check_for(uint8_t data);
		void enable();
		void disable();
};


#endif /* INC_MEMORY_CONTROL_H_ */
