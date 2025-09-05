/**
 * @file    dji_driver.h
 * @author  syhanjin
 * @date    2025-09-04
 * @brief   DJI Motor driver with pid control
 */
#ifndef DJI_DRIVER_H
#define DJI_DRIVER_H

#include <stdbool.h>
#include "drivers/DJI.h"
#include "libs/pid_motor.h"

#define __DJI_DRIVER_VERSION__ "0.1.0"

typedef struct
{
    bool enable;
    DJI_t dji;
    MotorPID_t velocity_pid;     //< 内环，速度环
    MotorPID_t position_pid;     //< 外环，位置环
    uint32_t pos_vel_freq_ratio; //< 内外环频率比
    uint32_t count;              //< 计数
} DJI_PosCtrl_t;

typedef struct
{
    DJI_Config_t dji;
    MotorPID_Config_t velocity_pid;
    MotorPID_Config_t position_pid;
    uint32_t pos_vel_freq_ratio; //< 内外环频率比
} DJI_PosCtrlConfig_t;

typedef struct
{
    bool enable;
    DJI_t dji;
    MotorPID_t pid;
} DJI_VelCtrl_t;

typedef struct
{
    bool enable;
    DJI_Config_t dji;
    MotorPID_Config_t pid;
} DJI_VelCtrlConfig_t;

void DJI_PosCtrl_Init(DJI_PosCtrl_t* hctrl, DJI_PosCtrlConfig_t config);
void DJI_VelCtrl_Init(DJI_VelCtrl_t* hctrl, DJI_VelCtrlConfig_t config);
void DJI_PosCtrlCalculate(DJI_PosCtrl_t* hctrl);
void DJI_VelCtrlCalculate(DJI_VelCtrl_t* hctrl);

/**
 * 启用 DJI 控制
 * @param __CTRL_HANDLE__ 受控对象 (DJI_PosCtrl_t* 或 DJI_VelCtrl_t*)
 */
#define __DJI_CTRL_ENABLE(__CTRL_HANDLE__) ((__CTRL_HANDLE__)->enable = true)

/**
 * 禁用 DJI 控制
 * @param __CTRL_HANDLE__ 受控对象 (DJI_PosCtrl_t* 或 DJI_VelCtrl_t*)
 */
#define __DJI_CTRL_DISABLE(__CTRL_HANDLE__) ((__CTRL_HANDLE__)->enable = false)

/**
 * 设置位置环目标值
 * @param hctrl 受控对象
 * @param ref 目标值 (unit: deg)
 */
static inline void DJI_PosCtrl_SetRef(DJI_PosCtrl_t* hctrl, const float ref)
{
    hctrl->position_pid.ref = ref;
}

/**
 * 设置速度环目标值
 * @param hctrl 受控对象
 * @param ref 目标值 (unit: rpm)
 */
static inline void DJI_VelCtrl_SetRef(DJI_VelCtrl_t* hctrl, const float ref)
{
    hctrl->pid.ref = ref;
}

#endif // DJI_DRIVER_H
