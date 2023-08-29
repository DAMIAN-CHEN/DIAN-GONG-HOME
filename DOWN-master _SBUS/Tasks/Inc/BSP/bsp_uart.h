//
// Created by 14685 on 2022/7/15.
//

#ifndef HNU_RM_DOWN_BSP_UART_H
#define HNU_RM_DOWN_BSP_UART_H
#include "stdint.h"
#include "usart.h"

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)


#define SBUS_RX_BUF_NUM 36u
#define RC_FRAME_LENGTH 25u
#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/**
  * @brief     �������ң�������ݽṹ��
  */
typedef struct
{
    /* ң������ͨ�����ݣ���ֵ��Χ��-660 ~ 660 */
    int16_t ch1;   //�Ҳ�����
    int16_t ch2;   //�Ҳ�����
    int16_t ch3;   //�������
    int16_t ch4;   //�������

    /* ң�����Ĳ������ݣ������·ֱ�Ϊ��1��3��2 */
    uint8_t sw1;   //��ದ��
    uint8_t sw2;   //�Ҳದ��

    /* PC ������� */
    struct
    {
        /* ����ƶ���� */
        int16_t x;   //���ƽ��
        int16_t y;   //�������
        /* ��갴����أ�1Ϊ���£�0Ϊ�ɿ� */
        uint8_t l;   //��ఴ��
        uint8_t r;   //�Ҳఴ��
    }mouse;

    /* PC ���̰������� */
    union
    {
        uint16_t key_code;
        struct
        {
            uint16_t W:1;
            uint16_t S:1;
            uint16_t A:1;
            uint16_t D:1;
            uint16_t SHIFT:1;
            uint16_t CTRL:1;
            uint16_t Q:1;
            uint16_t E:1;
            uint16_t R:1;
            uint16_t F:1;
            uint16_t G:1;
            uint16_t Z:1;
            uint16_t X:1;
            uint16_t C:1;
            uint16_t V:1;
            uint16_t B:1;
            uint16_t F1:1;
            uint16_t F2:1;
            uint16_t F3:1;

        }bit;
    }kb;

    /* ң������ದ������ */
    int16_t wheel;
} RCTypeDef;

/**
  * @brief     ң������������ö��
  */
enum
{
    RC_UP = 1,
    RC_MI = 3,
    RC_DN = 2,
};



/**
 * @brief               ң�س�ʼ��
 * @param rx1_buf       �ڴ滺����1
 * @param rx2_buf       �ڴ滺����2
 * @param dma_buf_num   ���ݳ���
 */
void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

/**
 * @brief   ң�س�ʼ�����������
 */
void Remote_Control_init(void);

/**
 * @brief         ң�����ݴ���
 * @param rc      ����ң�����ݵĽṹ��
 * @param buff    ң�����ݻ�����
 */
void Remote_Data_handle(RCTypeDef *rc, uint8_t *buff);

/**
 * @brief              ���ڷ�������
 * @param uart_id      ����ID
 * @param send_data    ���͵�����
 * @param size         ���ݳ���
 */
void Uart_Send(uint8_t uart_id, uint8_t *send_data, uint16_t size);


extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern RCTypeDef rc;
extern uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

#endif //HNU_RM_DOWN_BSP_UART_H
