/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "MPU-9250.h"
#include "QMC5883L.h"
#include "stdio.h"
#include "stdlib.h"
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include <stdarg.h>
#include"semphr.h"
#include"queue.h"
#include "gyro_mag_fusion.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BIT_MPU (1u << 0)
#define BIT_QMC (1u << 1)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
TaskHandle_t qmcTaskHandle = NULL;
TaskHandle_t mpuTaskHandle = NULL;
TaskHandle_t printTaskHandle = NULL;

static char  uart_buf[192];
static MPU9250_PhysicalData  g_mpu;
static QMC5883L_PhysicalData g_qmc;
static volatile float        g_heading_deg = 0.0f;
static volatile uint8_t      g_still = 0;
static volatile uint8_t g_ready = 0;
static SemaphoreHandle_t tx_done;

static QueueHandle_t magQ;
static HdgKF         g_kf;
static volatile float g_heading_gyro = 0.0f;   // gyro-only, for comparison
static volatile float g_heading_mag  = 0.0f;
static volatile float g_bias         = 0.0f;

#define Q_TH   0.05f
#define Q_B    0.001f
#define R_MAG  25.0f
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void QMC5883L(void  *argument);
void MPU_6050(void  *argument);
void PrintTask(void *argument);
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  tx_done = xSemaphoreCreateBinary();
  xSemaphoreGive(tx_done);     // starts as "free"
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  magQ = xQueueCreate(1, sizeof(float));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  xTaskCreate(QMC5883L, "qmcTask", 384, NULL, 2, &qmcTaskHandle);
  xTaskCreate(MPU_6050, "mpuTask", 384, NULL, 2, &mpuTaskHandle);
  xTaskCreate(PrintTask, "printTask", 512, NULL, 1, &printTaskHandle);
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
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
static void DMA_print(const char *fmt, ...)
{
    /* wait until the previous DMA transfer has finished */
    xSemaphoreTake(tx_done, portMAX_DELAY);

    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(uart_buf, sizeof(uart_buf), fmt, ap);
    va_end(ap);

    if (n <= 0) {
        xSemaphoreGive(tx_done);
        return;
    }
    if (n >= (int)sizeof(uart_buf)) n = sizeof(uart_buf) - 1;

    if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)uart_buf, n) != HAL_OK) {
        xSemaphoreGive(tx_done);   // transfer didn't start, release
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	//do this work when HAL_UART_Transmit_DMA finishes the transfer ( give the semaphore)
    if (huart->Instance == USART1) {
        BaseType_t hpw = pdFALSE;
        xSemaphoreGiveFromISR(tx_done, &hpw);
        portYIELD_FROM_ISR(hpw);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        BaseType_t hpw = pdFALSE;
        xSemaphoreGiveFromISR(tx_done, &hpw);
        portYIELD_FROM_ISR(hpw);
    }
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    BaseType_t hpw = pdFALSE;
    if (hi2c->Instance == I2C1) {
        vTaskNotifyGiveFromISR(mpuTaskHandle, &hpw);
    } else if (hi2c->Instance == I2C2) {
        vTaskNotifyGiveFromISR(qmcTaskHandle, &hpw);
    }
    portYIELD_FROM_ISR(hpw);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    BaseType_t hpw = pdFALSE;
    if (hi2c->Instance == I2C1) {
            vTaskNotifyGiveFromISR(mpuTaskHandle, &hpw);
        } else if (hi2c->Instance == I2C2) {
            vTaskNotifyGiveFromISR(qmcTaskHandle, &hpw);
        }
    portYIELD_FROM_ISR(hpw);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    BaseType_t hpw = pdFALSE;
    if (hi2c->Instance == I2C1) {
            vTaskNotifyGiveFromISR(mpuTaskHandle, &hpw);
        } else if (hi2c->Instance == I2C2) {
            vTaskNotifyGiveFromISR(qmcTaskHandle, &hpw);
        }
    portYIELD_FROM_ISR(hpw);
}

void QMC5883L(void *argument)
{
    DMA_print("\r\n=== QMC5883L Calibration (I2C2) ===\r\n");
    uint8_t chip_id = 0;
    HAL_StatusTypeDef st = HAL_ERROR;
    for (uint8_t attempt = 0; attempt < 10; attempt++) {
        vTaskDelay(pdMS_TO_TICKS(20));
        st = QMC5883L_ReadChipID(&hi2c2, &chip_id);
        if (st == HAL_OK && chip_id == 0xFF) break;
    }
    DMA_print("QMC id=0x%02X (expect 0xFF)\r\n", chip_id);
    st = QMC5883L_Init(&hi2c2, QMC5883L_ODR_50HZ, QMC5883L_RNG_2G, QMC5883L_OSR_512);
    if (st != HAL_OK) {
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    QMC5883L_RawData mraw;
    DMA_print("Rotate FLAT through several full circles for 20 s. Starting in 3 s...\r\n");
    vTaskDelay(pdMS_TO_TICKS(3000));

    QMC5883L_Calibration mag_cal;
    st = QMC5883L_Calibrate(&hi2c2, QMC5883L_RNG_2G, 20000, &mag_cal);
    if (st != HAL_OK) {
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    DMA_print("Mag cal done. Put the board still and flat.\r\n");

    xTaskNotifyGive(mpuTaskHandle);          // let the MPU task start

    TickType_t last = xTaskGetTickCount();
    for (;;) {
        if (QMC5883L_DataReady(&hi2c2) &&
            QMC5883L_ReadRaw(&hi2c2, &mraw) == HAL_OK) {
            QMC5883L_PhysicalData m;
            QMC5883L_ConvertToPhysical(&mraw, &m, QMC5883L_RNG_2G);
            QMC5883L_ApplyCalibration(&m, &mag_cal);
            g_qmc = m;
            float h = QMC5883L_ComputeHeading(&m);
            g_heading_mag = h;
            xQueueOverwrite(magQ, &h);       // newest heading, overwrites the old one
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(20));   // 50 Hz
    }
}


void MPU_6050(void *argument)
{
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);   // wait for mag calibration to finish
    DMA_print("\r\n=== MPU Calibration (I2C1) ===\r\n");

    uint8_t IDMPU = 0;
    for (uint8_t attempt = 0; attempt < 10; attempt++) {
        MPU9250_ReadWhoAmI(&hi2c1, &IDMPU);
        if (IDMPU != 0) break;
    }
    DMA_print("MPU ID:0x%02X (expect 0x70)\r\n", IDMPU);
    if (IDMPU != 0x70) {
        DMA_print("MPU WHO_AM_I wrong\r\n");
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    uint8_t init_ok = 0;
    for (uint8_t attempt = 0; attempt < 10; attempt++) {
        if (MPU9250_Init(&hi2c1, ACCEL_FS_4G, GYRO_FS_500DPS) == HAL_OK) {
            init_ok = 1;
            DMA_print("MPU init OK\r\n");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (!init_ok) {
        DMA_print("MPU init FAILED\r\n");
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    DMA_print("Calibrating... keep tag still and flat for ~2s\r\n");
    MPU9250_Bias bias;
    if (MPU9250_CalibrateGyroAccel(&hi2c1, ACCEL_FS_4G, GYRO_FS_500DPS, 1000, &bias) != HAL_OK) {
        DMA_print("Calibration FAILED\r\n");
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    DMA_print("you have 12 seconds to connect the QMC to the body\r\n");
    HAL_Delay(12000);
    vTaskDelay(pdMS_TO_TICKS(200));

    MPU9250_RawData      raw;
    MPU9250_PhysicalData phys;

    /* start the filter from the first mag heading */
    float h0 = 0.0f;
    xQueueReceive(magQ, &h0, pdMS_TO_TICKS(500));
    HdgKF_Init(&g_kf, h0);
    g_ready = 1;
    float gyro_only = h0;                       // gyro-only heading, for comparison

    TickType_t last = xTaskGetTickCount();
    TickType_t prev = last;

    for (;;) {
        TickType_t now = xTaskGetTickCount();
        float dt = (now - prev) * portTICK_PERIOD_MS / 1000.0f;
        prev = now;

        if (MPU9250_ReadRaw(&hi2c1, &raw) == HAL_OK) {
            MPU9250_ConvertToPhysical(&raw, &phys, ACCEL_FS_4G, GYRO_FS_500DPS);
            MPU9250_ApplyBias(&phys, &bias);
            g_mpu   = phys;
            g_still = MPU9250_IsStationary(&phys, 1.5f, 0.05f);

            /* every cycle: predict with the gyro */
            HdgKF_Predict(&g_kf, phys.gyro_z_dps, dt, Q_TH, Q_B);

            /* only when a new mag heading arrived: correct with it */
            float mh;
            if (xQueueReceive(magQ, &mh, 0) == pdTRUE) {
                HdgKF_Update(&g_kf, mh, R_MAG);
            }

            gyro_only += GYRO_SIGN * phys.gyro_z_dps * dt;
            g_heading_gyro = gyro_only;
            g_heading_deg  = g_kf.O;
            g_bias         = g_kf.b;
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(5));   // 200 Hz
    }
}

/* //to print heading in putty
void PrintTask(void *argument)
{
    vTaskDelay(pdMS_TO_TICKS(35000));
    for (;;) {
        DMA_print("fused=%6.1f gyro=%7.1f mag=%6.1f bias=%+.3f still=%d\r\n",
                  g_heading_deg, g_heading_gyro, g_heading_mag, g_bias, g_still);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}*/


//to send headings to the esp32 (tag)
void PrintTask(void *argument)
{
    while (!g_ready) vTaskDelay(pdMS_TO_TICKS(100));   // wait until calibration is done
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        DMA_print("H,%.1f,0,%d\r\n", g_heading_deg, g_still);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(50));     // 20 Hz
    }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
