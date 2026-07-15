#ifndef INC_DELAY_H_
#define INC_DELAY_H_

#include <stdint.h>

void delay_init(uint32_t core_clock_hz);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

#endif /* INC_DELAY_H_ */
