/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 平衡杆小球控制主程序。
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "bsp_motor.h"
#include "task.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/** @brief QD4310 电机地址。 */
#define MOTOR_ID              0x00U
/** @brief QD4310 DMA 事务总超时时间，单位为 ms。 */
#define MOTOR_TIMEOUT_MS      100U
/** @brief 任务选择与启动按键端口。 */
#define MOTOR_KEY_PORT        GPIOA
/** @brief 任务选择与启动按键引脚，低电平表示按下。 */
#define MOTOR_KEY_PIN         GPIO_PIN_0
/** @brief 视觉任务选择命令的 USART1 发送超时，单位为 ms。 */
#define VISION_TX_TIMEOUT_MS  100U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/** @brief USART6 电机 DMA 上下文。 */
static bsp_motor_t motor;
/** @brief 单按键任务选择器和调度器上下文。 */
static task_mgr_t task_mgr;
/** @brief TASK2 视觉控制上下文存储。 */
static app_control_t control;
/** @brief USART1 单字节中断接收缓存。 */
static uint8_t vision_rx_byte;
/** @brief USART1 视觉接收当前是否已启动。 */
static volatile uint8_t vision_rx_on;
/** @brief USART1 视觉接收是否需要由主循环重新启动。 */
static volatile uint8_t vision_rx_restart;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Send an application command to the vision module through USART1.
 * @param ctx USART1 handle passed by the board layer.
 * @param data Bytes to transmit.
 * @param len Number of bytes to transmit.
 */
static void Vision_Tx(void *ctx, const uint8_t *data, uint16_t len)
{
  UART_HandleTypeDef *uart = (UART_HandleTypeDef *)ctx;

  if ((uart == NULL) || (data == NULL) || (len == 0U))
  {
    return;
  }

  (void)HAL_UART_Transmit(uart, (uint8_t *)data, len, VISION_TX_TIMEOUT_MS);
}

/**
 * @brief 根据当前任务启停 USART1 视觉中断接收。
 * @param enable 非零表示启动视觉接收，零表示完全挂起接收。
 */
static void VisionRx_Service(uint8_t enable)
{
  if (enable == 0U)
  {
    if ((vision_rx_on != 0U) || (vision_rx_restart != 0U))
    {
      /* 先清运行标志，避免完成回调在关闭期间重新挂接接收。 */
      vision_rx_on = 0U;
      vision_rx_restart = 0U;
      (void)HAL_UART_AbortReceive(&huart1);
    }
    return;
  }

  if ((vision_rx_on != 0U) && (vision_rx_restart == 0U))
  {
    return;
  }

  if (vision_rx_restart != 0U)
  {
    /* 错误后的 HAL 接收状态不确定，先同步复位再重新挂接。 */
    vision_rx_on = 0U;
    (void)HAL_UART_AbortReceive(&huart1);
  }

  /* 丢弃停收期间遗留的字节和溢出标志，只接收当前任务的新帧。 */
  __HAL_UART_CLEAR_OREFLAG(&huart1);
  vision_rx_restart = 0U;
  if (HAL_UART_Receive_IT(&huart1, &vision_rx_byte, 1U) == HAL_OK)
  {
    vision_rx_on = 1U;
  }
  else
  {
    vision_rx_on = 0U;
    vision_rx_restart = 1U;
  }
}

/**
 * @brief UART DMA 发送完成回调。
 * @param huart 触发回调的 UART 句柄。
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART6))
  {
    BSP_MOTOR_OnTxComplete(&motor);
  }
}

/**
 * @brief UART 接收完成回调。
 * @param huart 触发回调的 UART 句柄。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART1))
  {
    if (vision_rx_on != 0U)
    {
      TASK_VisionRxByte(&task_mgr, vision_rx_byte);
      if (HAL_UART_Receive_IT(&huart1, &vision_rx_byte, 1U) != HAL_OK)
      {
        vision_rx_restart = 1U;
      }
    }
  }
  else if ((huart != NULL) && (huart->Instance == USART6))
  {
    BSP_MOTOR_OnRxComplete(&motor);
  }
}

/**
 * @brief UART 接收或 DMA 错误回调。
 * @param huart 发生错误的 UART 句柄。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART1))
  {
    if (vision_rx_on != 0U)
    {
      vision_rx_restart = 1U;
    }
  }
  else if ((huart != NULL) && (huart->Instance == USART6))
  {
    BSP_MOTOR_OnError(&motor);
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  uint32_t now_ms;
  bsp_motor_err_t motor_err;
  task_err_t task_err;

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
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_SPI1_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */

  motor_err = BSP_MOTOR_Init(&motor, &huart6, MOTOR_ID,
                             MOTOR_TIMEOUT_MS);
  task_err = TASK_Init(&task_mgr, &motor, &control, Vision_Tx, &huart1);
  vision_rx_on = 0U;
  vision_rx_restart = 0U;

  if ((motor_err != BSP_MOTOR_OK) || (task_err != TASK_OK))
  {
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    now_ms = HAL_GetTick();

    (void)TASK_Run(
        &task_mgr, now_ms,
        (uint8_t)(HAL_GPIO_ReadPin(MOTOR_KEY_PORT, MOTOR_KEY_PIN) ==
                  GPIO_PIN_RESET));
    VisionRx_Service(TASK_NeedsVisionRx(&task_mgr));
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
