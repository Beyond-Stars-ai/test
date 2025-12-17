/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#include "remote_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t receiveData[18];

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

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
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
void StartDebugTask(void *argument);
void StartRemoteTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    /* USER CODE BEGIN 2 */
    // HAL_UARTEx_ReceiveToIdle_DMA(&huart2, receiveData, sizeof(receiveData));
    // HAL_UART_Receive_DMA(&huart2, receiveData, sizeof(receiveData));
    /* USER CODE END 2 */

    /* Init scheduler */
    osKernelInitialize();

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

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

    /* USER CODE BEGIN USART3_Init 0 */

    /* USER CODE END USART3_Init 0 */

    /* USER CODE BEGIN USART3_Init 1 */

    /* USER CODE END USART3_Init 1 */
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART3_Init 2 */

    /* USER CODE END USART3_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Channel6_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
    /* DMA1_Channel7_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */

    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, LED_BLUE_Pin | LED_GREEN_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : LED_BLUE_Pin LED_GREEN_Pin */
    GPIO_InitStruct.Pin = LED_BLUE_Pin | LED_GREEN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : LED_RED_Pin */
    GPIO_InitStruct.Pin = LED_RED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : KEY1_Pin */
    GPIO_InitStruct.Pin = KEY1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(KEY1_GPIO_Port, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    /* USER CODE BEGIN MX_GPIO_Init_2 */

    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int fd, char *ptr, int len)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

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
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDebugTask */
/**
 * @brief Function implementing the DebugTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDebugTask */
void StartDebugTask(void *argument)
{
    /* USER CODE BEGIN 5 */
    /* Infinite loop */
    for (;;)
    {
        HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
        // printf("Debug Task Running\n");
        osDelay(1000);
    }
    /* USER CODE END 5 */
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

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
