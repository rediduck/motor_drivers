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
#include "drivers/DJI.h"
#include "interfaces/motor_if.h"
#include "tim.h"

DJI_t dji[2];
Motor_PosCtrl_t pos_dji;
Motor_VelCtrl_t vel_dji;

void TIM6_Callback(TIM_HandleTypeDef* htim)
{
    /* 进行 PID 计算 */
    Motor_PosCtrlCalculate(&pos_dji);
    Motor_VelCtrlCalculate(&vel_dji);

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

    // 注册 DJI 处理回调
    HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, DJI_CAN_Fifo0ReceiveCallback);
    // HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO1_MSG_PENDING_CB_ID, DJI_CAN_Fifo1ReceiveCallback);

    // 启动 CAN 接收
    // CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);
    CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    /* 初始化电机 */
    DJI_Init(&dji[0], (DJI_Config_t){
                          .auto_zero  = false, // 是否在启动时自动清零角度
                          .hcan       = &hcan1,
                          .motor_type = M3508_C620,
                          .id1        = 1, // 电调 ID (1~8)
                      });
    DJI_Init(&dji[1], (DJI_Config_t){
                          .auto_zero  = false, // 是否在启动时自动清零角度
                          .hcan       = &hcan1,
                          .motor_type = M3508_C620,
                          .id1        = 2, // 电调 ID (1~8)
                      });

    /* 初始化电机控制 */
    Motor_PosCtrl_Init(&pos_dji, //
                       (Motor_PosCtrlConfig_t){
                           .motor_type   = MOTOR_TYPE_DJI, //< 电机类型
                           .motor        = &dji[0],        //< 控制的电机
                           .velocity_pid = (MotorPID_Config_t){
                               .Kp             = 12.0f,  //
                               .Ki             = 0.20f,  //
                               .Kd             = 5.00f,  //
                               .abs_output_max = 8000.0f // DJI_M3508_C620_IQ_MAX //< 限幅为电流控制最大值
                           },
                           .position_pid = (MotorPID_Config_t){
                               .Kp             = 80.0f,  //
                               .Ki             = 1.00f,  //
                               .Kd             = 0.00f,  //
                               .abs_output_max = 2000.0f // 限速
                           },
                           .pos_vel_freq_ratio = 1});

    Motor_VelCtrl_Init(&vel_dji, //
                       (Motor_VelCtrlConfig_t){
                           .motor_type = MOTOR_TYPE_DJI, //< 电机类型
                           .motor      = &dji[1],        //< 控制的电机
                           .pid        = (MotorPID_Config_t){
                                      .Kp             = 4.7f,                 //
                                      .Ki             = 0.15f,                //
                                      .Kd             = 0.15f,                //
                                      .abs_output_max = DJI_M3508_C620_IQ_MAX //< 限幅为电流控制最大值
                           },
                       });

    // 注册定时器回调
    HAL_TIM_RegisterCallback(&htim6, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM6_Callback);
    HAL_TIM_Base_Start_IT(&htim6);

    /* 初始化完成后退出线程 */
    osThreadExit();
}
