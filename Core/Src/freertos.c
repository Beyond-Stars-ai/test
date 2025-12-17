/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#include "remote_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t receiveData[18];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for DebugTask */
osThreadId_t DebugTaskHandle;
const osThreadAttr_t DebugTask_attributes = {
    .name = "DebugTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for RemoteTask */
osThreadId_t RemoteTaskHandle;
const osThreadAttr_t RemoteTask_attributes = {
    .name = "RemoteTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for RemoteQueue */
osMessageQueueId_t RemoteQueueHandle;
const osMessageQueueAttr_t RemoteQueue_attributes = {
    .name = "RemoteQueue"};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDebugTask(void *argument);
void StartRemoteTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* Create the queue(s) */
    /* creation of RemoteQueue */
    RemoteQueueHandle = osMessageQueueNew(16, sizeof(receiveData), &RemoteQueue_attributes);

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of DebugTask */
    DebugTaskHandle = osThreadNew(StartDebugTask, NULL, &DebugTask_attributes);

    /* creation of RemoteTask */
    RemoteTaskHandle = osThreadNew(StartRemoteTask, NULL, &RemoteTask_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDebugTask */
/**
 * @brief  Function implementing the DebugTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDebugTask */
void StartDebugTask(void *argument)
{
    /* USER CODE BEGIN StartDebugTask */
    /* Infinite loop */
    for (;;)
    {
        HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
        // printf("Debug Task Running\n");
        osDelay(1000);
    }
    /* USER CODE END StartDebugTask */
}

/* USER CODE BEGIN Header_StartRemoteTask */
/**
 * @brief Function implementing the RemoteTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartRemoteTask */
void StartRemoteTask(void *argument)
{
    /* USER CODE BEGIN StartRemoteTask */
    osDelay(5);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, receiveData, sizeof(receiveData));

    RC_ctrl_t rc_control = {0}; // 初始化控制结构体

    /* Infinite loop */
    for (;;)
    {
        uint8_t Message_Remote[18] = {0};
        if (osMessageQueueGet(RemoteQueueHandle, &Message_Remote, NULL, osWaitForever) == osOK)
        {
            Message_Remote_to_rc(Message_Remote, &rc_control);
            // 打印解码后的数据
            printf("RC Channels: %d,%d,%d,%d,%d\n",
                   rc_control.rc.ch[0], rc_control.rc.ch[1],
                   rc_control.rc.ch[2], rc_control.rc.ch[3], rc_control.rc.ch[4]);
            printf("Switch: %d,%d\n",
                   rc_control.rc.s[0], rc_control.rc.s[1]);
            // const char *switch_states[] = {"DOWN", "UP", "MID"};
            // printf("Switch: %s,%s\n",
            //        switch_states[rc_control.rc.s[0] - 1],
            //        switch_states[rc_control.rc.s[1] - 1]);        
            printf("Mouse: x=%d,y=%d,z=%d,left=%d,right=%d\n",
                   rc_control.mouse.x, rc_control.mouse.y,
                   rc_control.mouse.z, rc_control.mouse.press_l,
                   rc_control.mouse.press_r);
            printf("Keys: 0x%04X\n", rc_control.key.v);
        }
    }
    /* USER CODE END StartRemoteTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

int _write(int fd, char *ptr, int len)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == KEY1_Pin)
    {
        osDelay(10);
        if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
        {
            HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
            printf("KEY Pressed\n");
        }
    }
}

// 不定长定时DMA中断接收
// void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
// {
//   HAL_UART_Transmit_DMA(&huart2, receiveData, sizeof(receiveData));
//   osDelay(1);
//   HAL_UARTEx_ReceiveToIdle_DMA(&huart2, receiveData, sizeof(receiveData));
// }

// 定长定时DMA中断接收
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//   HAL_UART_Transmit_DMA(&huart2, receiveData, sizeof(receiveData));
//   osDelay(10);
//   HAL_UART_Receive_DMA(&huart2, receiveData, sizeof(receiveData));
// }

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    uint8_t Message_Remote[18] = {0};
    if (huart->Instance == USART2 && Size > 0 && Size <= sizeof(receiveData))
    {
        // 复制接收到的数据
        memcpy(Message_Remote, receiveData, Size);
        // printf("第一次处理 %c\n", Message_Remote[0]);

        // 将消息放入队列
        osMessageQueuePut(RemoteQueueHandle, &Message_Remote, 0, 0);
        // printf("第二次处理 %c\n", Message_Remote[0]);

        // 清除 IDLE 中断标志
        __HAL_UART_CLEAR_IDLEFLAG(huart);

        // 重新启动IT接收
        // HAL_UART_Transmit_DMA(&huart2, receiveData, sizeof(receiveData));
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, receiveData, sizeof(receiveData));
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT); // 禁止半传送中断
    }
}
/* USER CODE END Application */
