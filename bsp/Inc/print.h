#ifndef PRINT_H
#define PRINT_H
#include "main.h"
extern void usart2_tx_dma_init(void);
extern void usart2_tx_dma_enable(uint8_t *data, uint16_t len);
extern void usart_printf(const char *fmt,...);
#endif
