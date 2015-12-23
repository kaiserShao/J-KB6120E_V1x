
#include "stm32f10x.h"
#define PBOOT	1
#define PSHUT	0

// void BKP_Write (uint16_t BKP_DR, uint16_t Data);

// uint16_t BKP_Read (uint16_t BKP_DR);
void Powertime_Write( uint32_t Data,uint8_t state);

uint32_t Powertime_Read(uint8_t state);

void BKPConfig(void);
/**/
