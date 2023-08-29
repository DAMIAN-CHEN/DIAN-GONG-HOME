//
// Created by 14685 on 2022/7/15.
//

#include "bsp_uart.h"
#include <string.h>
#include "usart.h"
#include "controller.h"

uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
RCTypeDef rc;

void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    //enable the DMA transfer for the receiver request
    //ʹ��DMA���ڽ���
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

    //enalbe idle interrupt
    //ʹ�ܿ����ж�
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
    //memory buffer 1
    //�ڴ滺����1
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //�ڴ滺����2
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //���ݳ���
    hdma_usart3_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer
    //ʹ��˫������
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //ʹ��DMA
    __HAL_DMA_ENABLE(&hdma_usart3_rx);
}

void Remote_Control_init(void)
{
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

void Remote_Data_handle(RCTypeDef *rc, uint8_t *buff)
{
    if (buff[0] == 0x0F){
    /* ����������ң�������ݵĴ��� */
    rc->ch1 = (buff[1] | buff[2] << 8) & 0x07FF;
    rc->ch1 -= 1000;
    rc->ch2 = (buff[2] >> 3 | buff[3] << 5) & 0x07FF;
    rc->ch2 -= 1000;
    rc->ch3 = (buff[3] >> 6 | buff[4] << 2 | buff[5] << 10) & 0x07FF;
    rc->ch3 -= 1000;
    rc->ch4 = (buff[5] >> 1 | buff[6] << 7) & 0x07FF;
    rc->ch4 -= 1000;

    /* ��ֹң���������ƫ�� */
    if(rc->ch1 <= 10 && rc->ch1 >= -10)
        rc->ch1 = 0;
    if(rc->ch2 <= 10 && rc->ch2 >= -10)
        rc->ch2 = 0;
    if(rc->ch3 <= 10 && rc->ch3 >= -10)
        rc->ch3 = 0;
    if(rc->ch4 <= 10 && rc->ch4 >= -10)
        rc->ch4 = 0;

    /* ����ֵ��ȡ */
    rc->sw1 = ((buff[6] >> 4|buff[7]<<4) & 0x07FF) ;
    rc->sw2 = ((buff[7] >> 7|buff[8]<<1) & 0x07FF) ;

    /* ң�����쳣ֵ��������ֱ�ӷ��� */
    if ((abs(rc->ch1) > 700) || \
      (abs(rc->ch2) > 700) || \
      (abs(rc->ch3) > 700) || \
      (abs(rc->ch4) > 700))
    {
        memset(rc, 0, sizeof(RCTypeDef));
        return ;
    }
    }

   /* *//* ����ƶ��ٶȻ�ȡ *//*
    rc->mouse.x = buff[6] | (buff[7] << 8);
    rc->mouse.y = buff[8] | (buff[9] << 8);

    *//* ������Ұ�����ֵ��ȡ *//*
    rc->mouse.l = buff[12];
    rc->mouse.r = buff[13];

    *//* ���̰�����ֵ��ȡ *//*
    rc->kb.key_code = buff[14] | buff[15] << 8;
*/
    /* ң��������Ϸ��������ݻ�ȡ����ң�����汾�йأ��е��޷��ش��������� */
    /* ��δ�����°� */
//    rc->wheel = buff[16] | buff[17] << 8;
//    rc->wheel -= 1024;
}

void Uart_Send(uint8_t uart_id, uint8_t *send_data, uint16_t size){
    switch(uart_id){
        case 1:
        {
            HAL_UART_Transmit(&huart1,send_data,size,HAL_MAX_DELAY);
            break;
        }
        default:
            break;
    }
}