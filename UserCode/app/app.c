/**
 * @file    app.c
 * @author  syhanjin
 * @date    2025-09-03
 * @brief   Brief description of the file
 *
 * Detailed description (optional).
 *
 */
#include "app.h"

#include "bsp/can_driver.h"
#include "can.h"
#include "cmsis_os2.h"
#include "controllers/dji_driver.h"
#include "drivers/DJI.h"

DJI_PosCtrl_t pos_dji;
DJI_VelCtrl_t vel_dji;

void TIM6_Callback(TIM_HandleTypeDef* htim)
{
    /* 进行 PID 计算 */
    DJI_PosCtrlCalculate(&pos_dji);
    DJI_VelCtrlCalculate(&vel_dji);

    /* 发送控制信号 */
    DJI_SendSetIqCommand(&hcan1, IQ_CMD_GROUP_1_4);
    // DJI_SendSetIqCommand(&hcan1, IQ_CMD_GROUP_5_8);
}

/**
 * @brief Function implementing the initTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Init */
void Init(void* argument)
{
    /* 执行初始化 */
    // 初始化 CAN
    DJI_CAN_FilterInit(&hcan1, 0);
    // CAN_Init(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);
    CAN_Init(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    // 注册 DJI 处理回调
    HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, DJI_CAN_Fifo0ReceiveCallback);
    // HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO1_MSG_PENDING_CB_ID, DJI_CAN_Fifo1ReceiveCallback);

    DJI_PosCtrl_Init(&pos_dji, //
                     {
                         .dji = (DJI_Config_t){
                             .auto_zero  = true, // 是否在启动时自动清零角度
                             .hcan       = &hcan1,
                             .motor_type = M3508_C620,
                             .id1        = 1, // 电调 ID (1~8)
                         },
                         .velocity_pid = (MotorPID_Config_t){
                             .Kp             = 4.7f,                 //
                             .Ki             = 0.15f,                //
                             .Kd             = 0.15f,                //
                             .abs_output_max = DJI_M3508_C620_IQ_MAX //< 限幅为电流控制最大值
                         },
                         .position_pid = (MotorPID_Config_t){
                             .Kp             = 13.0f,  //
                             .Ki             = 0.015f, //
                             .Kd             = 0.01f,  //
                             .abs_output_max = 2000.0f // 限速
                         }});

    DJI_VelCtrl_Init(&vel_dji, //
                     {
                         .dji = (DJI_Config_t){
                             .auto_zero  = true, // 是否在启动时自动清零角度
                             .hcan       = &hcan1,
                             .motor_type = M3508_C620,
                             .id1        = 2, // 电调 ID (1~8)
                         },
                         .pid = (MotorPID_Config_t){
                             .Kp             = 4.7f,                 //
                             .Ki             = 0.15f,                //
                             .Kd             = 0.15f,                //
                             .abs_output_max = DJI_M3508_C620_IQ_MAX //< 限幅为电流控制最大值
                         },
                     });

    /* 初始化完成后退出线程 */
    osThreadExit();
}
