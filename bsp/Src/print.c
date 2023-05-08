#include "print.h"
#include "main.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_tx;
void usart2_tx_dma_init(void)
{
    //enable the DMA transfer for the receiver request
    //ʹ��DMA���ڽ���
    SET_BIT(huart2.Instance->CR3, USART_CR3_DMAT);
}
void usart2_tx_dma_enable(uint8_t *data, uint16_t len)
{

    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart2_tx);
    while(hdma_usart2_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart2_tx);
    }

    //clear flag
    //�����־λ
    __HAL_DMA_CLEAR_FLAG(&hdma_usart2_tx, DMA_HISR_TCIF7);
    __HAL_DMA_CLEAR_FLAG(&hdma_usart2_tx, DMA_HISR_HTIF7);

    //set data address
    //�������ݵ�ַ
    hdma_usart2_tx.Instance->M0AR = (uint32_t)(data);
    //set data length
    //�������ݳ���
    hdma_usart2_tx.Instance->NDTR = len;

    //enable DMA
    //ʹ��DMA
    __HAL_DMA_ENABLE(&hdma_usart2_tx);
}
void usart_printf(const char *fmt,...)
{
    static uint8_t tx_buf[256] = {0};
    static va_list ap;
    static uint16_t len;
    va_start(ap, fmt);

    //return length of string 
    //�����ַ�������
    len = vsprintf((char *)tx_buf, fmt, ap);

    va_end(ap);

    usart2_tx_dma_enable(tx_buf, len);

}
