#include "CAN_task.h"
#include "can.h"
#include "Motor.h"
#include "main.h" 
uint8_t CAN1RXmessage[8];
extern int16_t ch3;
CAN_RxHeaderTypeDef rx1;

Motor_typedef GM6020_demo;
pid_struct_t gimbal_yaw_angle_pid;
pid_struct_t gimbal_yaw_speed_pid;
void CAN_Calc(CAN_HandleTypeDef *hcan, uint32_t ID);
void gimbal_PID_init();// jiao du huan he su du huan de chu shi hua 
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx1, CAN1RXmessage);
    CAN_Calc(hcan, rx1.StdId);
    CAN_send(hcan, 0x1FF, 0, 0, 0, pid_calc(&gimbal_yaw_speed_pid,pid_calc(&gimbal_yaw_angle_pid,3000,GM6020_demo.data.rawEncode),GM6020_demo.data.rawSpeed));
}

void CAN_Calc(CAN_HandleTypeDef *hcan, uint32_t ID)
{
    if (hcan->Instance == CAN1) {
        switch (ID) {
            case GM6020_D: {
                MotorResolve(&GM6020_demo, CAN1RXmessage);
            } break;
        }
    }
}
void CAN_send(CAN_HandleTypeDef *_hcan, int16_t stdid, int16_t num1, int16_t num2, int16_t num3, int16_t num4)
{
    CAN_TxHeaderTypeDef tx;
    uint8_t Data[8];
    uint32_t mailbox = 0;
    tx.DLC           = 0x08;
    tx.IDE           = CAN_ID_STD;
    tx.RTR           = CAN_RTR_DATA;
    tx.StdId         = stdid;
    tx.ExtId         = 0x000;
    Data[0]          = ((num1) >> 8);
    Data[1]          = (num1);
    Data[2]          = ((num2) >> 8);
    Data[3]          = (num2);
    Data[4]          = ((num3) >> 8);
    Data[5]          = (num3);
    Data[6]          = ((num4) >> 8);
    Data[7]          = (num4);
    HAL_CAN_AddTxMessage(&hcan, &tx, Data, &mailbox);
}

void CAN_Filter_Init(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank           = 0;
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh         = 0x0000;
    sFilterConfig.FilterIdLow          = 0x0000;
    sFilterConfig.FilterMaskIdHigh     = 0x0000;
    sFilterConfig.FilterMaskIdLow      = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation     = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) {
        /* Filter configuration Error */
        Error_Handler();
    }
    /* Start the CAN peripheral */
    if (HAL_CAN_Start(hcan) != HAL_OK) {
        /* Start Error */
        Error_Handler();
    }
    /* Activate CAN RX notification */
    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        /* Notification Error */
        Error_Handler();
    }
    /* Activate CAN TX notification */
    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK) {
        /* Notification Error */
        Error_Handler();
    }
}
