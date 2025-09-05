/**
 * @file    dji_driver.c
 * @author  syhanjin
 * @date    2025-09-04
 * @brief
 */
#include "dji_driver.h"

/**
 * 初始化位置环控制参数
 * @param hctrl 受控对象
 * @param config 配置
 * @attention 计算函数不会对输出限幅，务必将内环输出限幅设为最大电流值
 */
void DJI_PosCtrl_Init(DJI_PosCtrl_t* hctrl, const DJI_PosCtrlConfig_t config)
{
    DJI_Init(&hctrl->dji, config.dji);
    MOTOR_PID_Init(&hctrl->velocity_pid, config.velocity_pid);
    MOTOR_PID_Init(&hctrl->position_pid, config.position_pid);
    hctrl->pos_vel_freq_ratio = config.pos_vel_freq_ratio ? config.pos_vel_freq_ratio : 1;
    hctrl->enable             = true;
}

/**
 * 初始化速度环受控对象
 * @param hctrl 受控对象
 * @param config 配置
 * @attention 计算函数不会对输出限幅，务必将输出限幅设为最大电流值
 */
void DJI_VelCtrl_Init(DJI_VelCtrl_t* hctrl, const DJI_VelCtrlConfig_t config)
{
    DJI_Init(&hctrl->dji, config.dji);
    MOTOR_PID_Init(&hctrl->pid, config.pid);
    hctrl->enable = true;
}

/**
 * 位置环控制计算
 * @param hctrl 受控对象
 */
void DJI_PosCtrlCalculate(DJI_PosCtrl_t* hctrl)
{
    if (!hctrl->enable)
        return;

    ++hctrl->count;

    if (hctrl->count == hctrl->pos_vel_freq_ratio)
    {
        // 反馈为当前电机输出角度
        hctrl->position_pid.fdb = hctrl->dji.abs_angle;
        MOTOR_PID_Calculate(&hctrl->position_pid);
        hctrl->count = 0;
    }

    hctrl->velocity_pid.ref = hctrl->position_pid.output;
    hctrl->velocity_pid.fdb = hctrl->dji.velocity;
    MOTOR_PID_Calculate(&hctrl->velocity_pid);

    // ATTENTION: 此处不做输出限幅校验，输出限幅应当放在 PID 参数中
    hctrl->dji.iq_cmd = (int16_t)hctrl->velocity_pid.output;
}

/**
 * 速度环控制计算
 * @param hctrl 受控对象
 */
void DJI_VelCtrlCalculate(DJI_VelCtrl_t* hctrl)
{
    if (!hctrl->enable)
        return;

    hctrl->pid.fdb = hctrl->dji.velocity;
    MOTOR_PID_Calculate(&hctrl->pid);

    // ATTENTION: 此处不做输出限幅校验，输出限幅应当放在 PID 参数中
    hctrl->dji.iq_cmd = (int16_t)hctrl->pid.output;
}
