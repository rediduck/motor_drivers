/**
 * @file    DJI.h
 * @author  syhanjin
 * @date    2025-09-03
 * @version 0.0.1
 * @brief   DJI Motor driver
 */
#ifndef DJI_H
#define DJI_H
#define __DJI_VERSION__     "0.0.1"

#define DJI_ERROR_HANDLER() Error_Handler()

#define CAN_NUM             (2)

#include <stdbool.h>
#include "main.h"

typedef enum
{
    M3508_C620 = 0U,
    M2006_C610,

    DJI_MOTOR_TYPE_COUNT
} DJI_MotorType_t;

typedef enum
{
    IQ_CMD_GROUP_1_4 = 0U,
    IQ_CMD_GROUP_5_8 = 4U,
} DJI_IqSetCmdGroup_t;

typedef struct
{
    bool enable;    // 是否启用
    bool auto_zero; // 是否自动判断零点

    DJI_MotorType_t motor_type; //< 电机类型
    CAN_TypeDef* can;           //< CAN 实例
    uint8_t id1;                //< 电调 ID (1 ~ 8)
    float angle_zero;           //< 零点角度 (unit: degree)

    /* Feedback */
    uint32_t feedback_count; //< 接收到的反馈数据数量
    struct
    {
        float mech_angle; //< 单圈机械角度 (unit: degree)
        float rpm;        //< 转速
        // float current; //< 电流大小
        // float temperature; //< 温度

        int32_t round_cnt; //< 圈数
    } feedback;

    /* Data */
    float abs_angle; //< 电机轴输出角度 (unit: degree)
    float velocity;  //< 电机轴输出速度 (unit: rpm)

    /* Output */
    uint16_t iq_cmd; //< 电流指令值
} DJI_t;

typedef struct
{
    CAN_TypeDef* can; //< CAN 实例
    DJI_t* motors[8]; //< 电机指针数组
} DJI_FeedbackMap;

typedef struct
{
    bool auto_zero;
    DJI_MotorType_t motor_type;
    CAN_HandleTypeDef* hcan;
    uint8_t id1;
} DJI_Config_t;

#define __DJI_SET_IQ_CMD__(__DJI_HANDLE__, __IQ_CMD__) ((__DJI_HANDLE__)->iq_cmd = (__IQ_CMD__))

void DJI_ResetAngle(DJI_t* hdji);

void DJI_CAN_Fifo0ReceiveCallback(CAN_HandleTypeDef* hcan);
void DJI_CAN_Fifo1ReceiveCallback(CAN_HandleTypeDef* hcan);

#endif // DJI_H
