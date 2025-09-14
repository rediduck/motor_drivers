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
#include "cmsis_os2.h"


/**
 * @brief Function implementing the initTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Init */
void Init(void* argument)
{
    /* 执行初始化 */

    // 运行实例将不在此处，请见 app/*_example.c

    /* 初始化完成后退出线程 */
    osThreadExit();
}
