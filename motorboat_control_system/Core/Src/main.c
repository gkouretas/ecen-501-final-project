/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "app_bluenrg_ms.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "vl53l0x_driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticTimer_t osStaticTimerDef_t;
typedef StaticSemaphore_t osStaticMutexDef_t;
/* USER CODE BEGIN PTD */
typedef enum {
	kBoatIdle = 0,
	kBoatDriving = 1,
	kBoatAnchored = 2,
  kBoatError = 3
} BoatState_t;

typedef struct {
  uint16_t direction; // 0 -> 65536 : Left -> Right
  uint16_t speed;     // 0 -> 65536 : 0-100% speed
} BoatCommand_t;

typedef enum {
  kDirectionCCW = -1,
  kDirectionNull = 0,
  kDirectionCW = +1,
} Direction_t; // CCW (+), CW (-)

typedef struct {
  uint16_t duty_cycle; // 0 -> 65536
  Direction_t direction;
  bool is_alive;
  bool is_idle;
} MotorState_t;

typedef struct {
  Direction_t direction;
  uint16_t duty_cycle;
  bool timeout;
} MotorStatus_t;

#define NUM_MOTORS 4
#define NUM_SPARE_MOTORS 1

typedef struct {
  BoatState_t boat_state;
  bool control_active;
  bool collision_detected;
  bool depth_exceeded;
  bool anchor_lifted;
  MotorStatus_t motor_statuses[NUM_MOTORS + NUM_SPARE_MOTORS];
} SystemInformation_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define THREAD_FLAG_DRIVING 0x1

#define PWM_TIMER_PRIMARY_HANDLE   &htim1
#define PWM_TIMER_SECONDARY_HANDLE &htim2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// Convert convert 16-bit duty cycle value to correct timer CRR using PWM frequency
#define DUTY_TO_CCR(duty_cycle_16bit) \
    (((uint32_t)(duty_cycle_16bit) * TIM_ARR) / 65535)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;

I2C_HandleTypeDef hi2c2;

QSPI_HandleTypeDef hqspi;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim1_ch1;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for motorTmoutTask */
osThreadId_t motorTmoutTaskHandle;
uint32_t motorTmoutTaskBuffer[ 128 ];
osStaticThreadDef_t motorTmoutTaskControlBlock;
const osThreadAttr_t motorTmoutTask_attributes = {
  .name = "motorTmoutTask",
  .cb_mem = &motorTmoutTaskControlBlock,
  .cb_size = sizeof(motorTmoutTaskControlBlock),
  .stack_mem = &motorTmoutTaskBuffer[0],
  .stack_size = sizeof(motorTmoutTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for boatSMTask */
osThreadId_t boatSMTaskHandle;
uint32_t boatSMTaskBuffer[ 128 ];
osStaticThreadDef_t boatSMTaskControlBlock;
const osThreadAttr_t boatSMTask_attributes = {
  .name = "boatSMTask",
  .cb_mem = &boatSMTaskControlBlock,
  .cb_size = sizeof(boatSMTaskControlBlock),
  .stack_mem = &boatSMTaskBuffer[0],
  .stack_size = sizeof(boatSMTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for depthDetectTask */
osThreadId_t depthDetectTaskHandle;
uint32_t depthDetectTaskBuffer[ 128 ];
osStaticThreadDef_t depthDetectTaskControlBlock;
const osThreadAttr_t depthDetectTask_attributes = {
  .name = "depthDetectTask",
  .cb_mem = &depthDetectTaskControlBlock,
  .cb_size = sizeof(depthDetectTaskControlBlock),
  .stack_mem = &depthDetectTaskBuffer[0],
  .stack_size = sizeof(depthDetectTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for readMotorTask */
osThreadId_t readMotorTaskHandle;
uint32_t readMotorTaskBuffer[ 128 ];
osStaticThreadDef_t readMotorTaskControlBlock;
const osThreadAttr_t readMotorTask_attributes = {
  .name = "readMotorTask",
  .cb_mem = &readMotorTaskControlBlock,
  .cb_size = sizeof(readMotorTaskControlBlock),
  .stack_mem = &readMotorTaskBuffer[0],
  .stack_size = sizeof(readMotorTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for sendDataTask */
osThreadId_t sendDataTaskHandle;
uint32_t sendDataTaskBuffer[ 128 ];
osStaticThreadDef_t sendDataTaskControlBlock;
const osThreadAttr_t sendDataTask_attributes = {
  .name = "sendDataTask",
  .cb_mem = &sendDataTaskControlBlock,
  .cb_size = sizeof(sendDataTaskControlBlock),
  .stack_mem = &sendDataTaskBuffer[0],
  .stack_size = sizeof(sendDataTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for queueBoatCommand */
osMessageQueueId_t queueBoatCommandHandle;
const osMessageQueueAttr_t queueBoatCommand_attributes = {
  .name = "queueBoatCommand"
};
/* Definitions for timerMotorTimeout */
osTimerId_t timerMotorTimeoutHandle;
osStaticTimerDef_t motorTimerControlBlock;
const osTimerAttr_t timerMotorTimeout_attributes = {
  .name = "timerMotorTimeout",
  .cb_mem = &motorTimerControlBlock,
  .cb_size = sizeof(motorTimerControlBlock),
};
/* Definitions for timerSensorRead */
osTimerId_t timerSensorReadHandle;
osStaticTimerDef_t timerSensorReadControlBlock;
const osTimerAttr_t timerSensorRead_attributes = {
  .name = "timerSensorRead",
  .cb_mem = &timerSensorReadControlBlock,
  .cb_size = sizeof(timerSensorReadControlBlock),
};
/* Definitions for mutexSystemInfo */
osMutexId_t mutexSystemInfoHandle;
osStaticMutexDef_t mutexSystemInfoControlBlock;
const osMutexAttr_t mutexSystemInfo_attributes = {
  .name = "mutexSystemInfo",
  .cb_mem = &mutexSystemInfoControlBlock,
  .cb_size = sizeof(mutexSystemInfoControlBlock),
};
/* Definitions for mutexMotorState */
osMutexId_t mutexMotorStateHandle;
osStaticMutexDef_t mutexMotorStateControlBlock;
const osMutexAttr_t mutexMotorState_attributes = {
  .name = "mutexMotorState",
  .cb_mem = &mutexMotorStateControlBlock,
  .cb_size = sizeof(mutexMotorStateControlBlock),
};
/* Definitions for semaphoreToFISR */
osSemaphoreId_t semaphoreToFISRHandle;
const osSemaphoreAttr_t semaphoreToFISR_attributes = {
  .name = "semaphoreToFISR"
};
/* USER CODE BEGIN PV */
uint32_t motor_timeout = pdMS_TO_TICKS(2000);
VL53l0X_Interface_t *tof_interface = NULL;

volatile MotorState_t motor_states[NUM_MOTORS + NUM_SPARE_MOTORS];

volatile static SystemInformation_t system_information = {
  .anchor_lifted = false,
  .boat_state = kBoatIdle,
  .collision_detected = false,
  .control_active = false,
  .depth_exceeded = false,
  .motor_statuses = {
	  {.direction = kDirectionNull, .duty_cycle = 0, .timeout = false},
	  {.direction = kDirectionNull, .duty_cycle = 0, .timeout = false},
	  {.direction = kDirectionNull, .duty_cycle = 0, .timeout = false},
	  {.direction = kDirectionNull, .duty_cycle = 0, .timeout = false},
	  {.direction = kDirectionNull, .duty_cycle = 0, .timeout = false}
  }
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_I2C2_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
void StartTaskMotorTmout(void *argument);
void StartTaskBoatSM(void *argument);
void StartTaskDepthDetect(void *argument);
void StartTaskReadMotorCommands(void *argument);
void StartTaskSendData(void *argument);
void callbackMotorTimeout(void *argument);
void callbackSensorRead(void *argument);

/* USER CODE BEGIN PFP */
void initMotorStates(void);
void initMotorPWM(void);
void updateMotorPWM(uint16_t *pwm_array, size_t size);
void append_to_buffer(char* buffer, size_t* remaining_size, const char* string);
char* system_info_to_json(const SystemInformation_t* info, char* json_buffer, size_t buffer_size);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
  for (int i = 0; i < len; i++) {
    ITM_SendChar((*ptr++));
  }
  return len;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == VL53L0X_GPIO1_EXTI7_Pin)
    {
    	// Set ISR as "ready"
    	vl53l0x_set_isr_flag();

    	// Unblock depth task
      if (semaphoreToFISRHandle != NULL)
      {
        osSemaphoreRelease(semaphoreToFISRHandle);
      }
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
  MX_DFSDM1_Init();
  MX_I2C2_Init();
  MX_QUADSPI_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_BlueNRG_MS_Init();
  /* USER CODE BEGIN 2 */
  // Initialize ToF sensor
  tof_interface = vl53l0x_init(&hi2c2, 0x52, VL53L0X_XSHUT_GPIO_Port, VL53L0X_XSHUT_Pin);
  if (tof_interface == NULL)
  {
	  printf("Failed to initialize ToF sensor\n");
	  Error_Handler();
  }
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of mutexSystemInfo */
  mutexSystemInfoHandle = osMutexNew(&mutexSystemInfo_attributes);

  /* creation of mutexMotorState */
  mutexMotorStateHandle = osMutexNew(&mutexMotorState_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of semaphoreToFISR */
  semaphoreToFISRHandle = osSemaphoreNew(1, 1, &semaphoreToFISR_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of timerMotorTimeout */
  timerMotorTimeoutHandle = osTimerNew(callbackMotorTimeout, osTimerPeriodic, NULL, &timerMotorTimeout_attributes);

  /* creation of timerSensorRead */
  timerSensorReadHandle = osTimerNew(callbackSensorRead, osTimerPeriodic, NULL, &timerSensorRead_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of queueBoatCommand */
  queueBoatCommandHandle = osMessageQueueNew (1, sizeof(BoatCommand_t), &queueBoatCommand_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of motorTmoutTask */
  motorTmoutTaskHandle = osThreadNew(StartTaskMotorTmout, NULL, &motorTmoutTask_attributes);

  /* creation of boatSMTask */
  boatSMTaskHandle = osThreadNew(StartTaskBoatSM, NULL, &boatSMTask_attributes);

  /* creation of depthDetectTask */
  depthDetectTaskHandle = osThreadNew(StartTaskDepthDetect, NULL, &depthDetectTask_attributes);

  /* creation of readMotorTask */
  readMotorTaskHandle = osThreadNew(StartTaskReadMotorCommands, NULL, &readMotorTask_attributes);

  /* creation of sendDataTask */
  sendDataTaskHandle = osThreadNew(StartTaskSendData, NULL, &sendDataTask_attributes);

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_channel1.Instance = DFSDM1_Channel1;
  hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel1.Init.OutputClock.Divider = 2;
  hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
  hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel1.Init.Awd.Oversampling = 1;
  hdfsdm1_channel1.Init.Offset = 0;
  hdfsdm1_channel1.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

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
  hi2c2.Init.Timing = 0x10D19CE4;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 2;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 23;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = TIM1_PSC;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = TIM1_ARR;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_OTG_FS_PWR_EN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPBTLE_RF_SPI3_CSN_GPIO_Port, SPBTLE_RF_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPSGRF_915_SPI3_CSN_GPIO_Port, SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : M24SR64_Y_RF_DISABLE_Pin M24SR64_Y_GPO_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_OVRCR_EXTI3_Pin SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin ISM43362_DRDY_EXTI1_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_EXTI3_Pin|SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin|ISM43362_DRDY_EXTI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_EXTI13_Pin */
  GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_A5_Pin ARD_A4_Pin ARD_A3_Pin ARD_A2_Pin
                           ARD_A1_Pin ARD_A0_Pin */
  GPIO_InitStruct.Pin = ARD_A5_Pin|ARD_A4_Pin|ARD_A3_Pin|ARD_A2_Pin
                          |ARD_A1_Pin|ARD_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D1_Pin ARD_D0_Pin */
  GPIO_InitStruct.Pin = ARD_D1_Pin|ARD_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D10_Pin SPBTLE_RF_RST_Pin ARD_D9_Pin */
  GPIO_InitStruct.Pin = ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D7_Pin */
  GPIO_InitStruct.Pin = ARD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D13_Pin ARD_D12_Pin ARD_D11_Pin */
  GPIO_InitStruct.Pin = ARD_D13_Pin|ARD_D12_Pin|ARD_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D3_Pin */
  GPIO_InitStruct.Pin = ARD_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D6_Pin */
  GPIO_InitStruct.Pin = ARD_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D8_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin LED2_Pin
                           SPSGRF_915_SDN_Pin ARD_D5_Pin SPSGRF_915_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin|SPSGRF_915_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : INTERNAL_UART3_TX_Pin INTERNAL_UART3_RX_Pin */
  GPIO_InitStruct.Pin = INTERNAL_UART3_TX_Pin|INTERNAL_UART3_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI0_Pin LSM6DSL_INT1_EXTI11_Pin ARD_D2_Pin HTS221_DRDY_EXTI15_Pin
                           PMOD_IRQ_EXTI12_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin|LSM6DSL_INT1_EXTI11_Pin|ARD_D2_Pin|HTS221_DRDY_EXTI15_Pin
                          |PMOD_IRQ_EXTI12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin SPBTLE_RF_SPI3_CSN_Pin PMOD_RESET_Pin STSAFE_A100_RESET_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin|SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
  GPIO_InitStruct.Pin = VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VL53L0X_GPIO1_EXTI7_Pin */
  GPIO_InitStruct.Pin = VL53L0X_GPIO1_EXTI7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(VL53L0X_GPIO1_EXTI7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LSM3MDL_DRDY_EXTI8_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PMOD_SPI2_SCK_Pin */
  GPIO_InitStruct.Pin = PMOD_SPI2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PMOD_SPI2_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PMOD_UART2_CTS_Pin PMOD_UART2_RTS_Pin PMOD_UART2_TX_Pin PMOD_UART2_RX_Pin */
  GPIO_InitStruct.Pin = PMOD_UART2_CTS_Pin|PMOD_UART2_RTS_Pin|PMOD_UART2_TX_Pin|PMOD_UART2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D15_Pin ARD_D14_Pin */
  GPIO_InitStruct.Pin = ARD_D15_Pin|ARD_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void initMotorStates(void)
{
	for (int i = 0; i<NUM_MOTORS + NUM_SPARE_MOTORS; i++)
	{
		motor_states[i].direction = system_information.motor_statuses[i].direction;
		motor_states[i].duty_cycle = system_information.motor_statuses[i].duty_cycle;
		motor_states[i].is_alive = true;
		if(i >= NUM_MOTORS){
			motor_states[i].is_idle = true;
		}else{
			motor_states[i].is_idle = false;
		}
	}
}

// Use motor state to initialize PWM
void initMotorPWM(void)
{
	// Initialize PWM duty cycles
	__HAL_TIM_SET_COMPARE(PWM_TIMER_PRIMARY_HANDLE, TIM_CHANNEL_1, motor_states[0].duty_cycle);
	__HAL_TIM_SET_COMPARE(PWM_TIMER_PRIMARY_HANDLE, TIM_CHANNEL_2, motor_states[1].duty_cycle);
	__HAL_TIM_SET_COMPARE(PWM_TIMER_PRIMARY_HANDLE, TIM_CHANNEL_3, motor_states[2].duty_cycle);
	__HAL_TIM_SET_COMPARE(PWM_TIMER_PRIMARY_HANDLE, TIM_CHANNEL_4, motor_states[3].duty_cycle);
	__HAL_TIM_SET_COMPARE(PWM_TIMER_SECONDARY_HANDLE, TIM_CHANNEL_1, motor_states[4].duty_cycle); // Spare motor

	// Start PWM on all channels
	HAL_TIM_PWM_Start(PWM_TIMER_PRIMARY_HANDLE, TIM_CHANNEL_1);
 	HAL_TIM_PWM_Start(PWM_TIMER_PRIMARY_HANDLE, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(PWM_TIMER_PRIMARY_HANDLE, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(PWM_TIMER_SECONDARY_HANDLE, TIM_CHANNEL_1); // Spare motor
}

// Update motor states with new PWM values and update timers
void updateMotorPWM(uint16_t *pwm_array, size_t size)
{
	// Check if array has new PWM values for each motor
	if((size / sizeof(pwm_array[0])) != (NUM_MOTORS + NUM_SPARE_MOTORS))
	{
		Error_Handler();
	}

	osMutexAcquire(mutexMotorStateHandle, osWaitForever);
	// Update motor states with new PWM values
	for(int i=0; i<(NUM_MOTORS + NUM_SPARE_MOTORS);i++)
	{
		motor_states[i].duty_cycle = pwm_array[i];
	}
	osMutexRelease(mutexMotorStateHandle);

	__HAL_TIM_SET_COMPARE(PWM_TIMER_PRIMARY_HANDLE, TIM_CHANNEL_1, pwm_array[0]);
	__HAL_TIM_SET_COMPARE(PWM_TIMER_PRIMARY_HANDLE, TIM_CHANNEL_2, pwm_array[1]);
	__HAL_TIM_SET_COMPARE(PWM_TIMER_PRIMARY_HANDLE, TIM_CHANNEL_3, pwm_array[2]);
	__HAL_TIM_SET_COMPARE(PWM_TIMER_PRIMARY_HANDLE, TIM_CHANNEL_4, pwm_array[3]);
	__HAL_TIM_SET_COMPARE(PWM_TIMER_SECONDARY_HANDLE, TIM_CHANNEL_1, pwm_array[4]);

}

// Append to JSON buffer safely
void append_to_buffer(char* buffer, size_t* remaining_size, const char* string) {
    strncat(buffer, string, *remaining_size - 1);
    *remaining_size -= strlen(string);
}

// Convert the struct to a JSON string
char* system_info_to_json(const SystemInformation_t* info, char* json_buffer, size_t buffer_size) {
    if (!info || !json_buffer || buffer_size == 0) return NULL;

    size_t remaining_size = buffer_size;
    char temp_json[128];

    // Start the JSON string
    snprintf(temp_json, sizeof(temp_json),
        "{"
        "\"boat_state\":%d,"
        "\"control_active\":%s,"
        "\"collision_detected\":%s,"
        "\"depth_exceeded\":%s,"
        "\"anchor_lifted\":%s,"
        "\"motor_statuses\":[",
        info->boat_state,
        info->control_active ? "true" : "false",
        info->collision_detected ? "true" : "false",
        info->depth_exceeded ? "true" : "false",
        info->anchor_lifted ? "true" : "false"
    );
    append_to_buffer(json_buffer, &remaining_size, temp_json);

    // Append motor statuses
    for (int i = 0; i < NUM_MOTORS + NUM_SPARE_MOTORS; i++) {
        char temp_json[128]; // Temporary buffer for each motor
        snprintf(temp_json, sizeof(temp_json),
            "{"
            "\"direction\":%d,"
            "\"duty_cycle\":%u,"
            "\"timeout\":%s"
            "}%s",
            info->motor_statuses[i].direction,
            info->motor_statuses[i].duty_cycle,
            info->motor_statuses[i].timeout ? "true" : "false",
            (i < NUM_MOTORS + NUM_SPARE_MOTORS - 1) ? "," : "" // Add comma except for the last item
        );
        append_to_buffer(json_buffer, &remaining_size, temp_json);
    }

    // Close the JSON array and object
    append_to_buffer(json_buffer, &remaining_size, "]}");

    return json_buffer;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTaskMotorTmout */
/**
  * @brief  Function implementing the motorTmoutTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTaskMotorTmout */
void StartTaskMotorTmout(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTaskBoatSM */
/**
* @brief Function implementing the boatSMTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskBoatSM */
void StartTaskBoatSM(void *argument)
{
  /* USER CODE BEGIN StartTaskBoatSM */

  /* Infinite loop */
  for(;;)
  {
    osMutexAcquire(mutexSystemInfoHandle, osWaitForever);
    switch (system_information.boat_state)
    {
		case kBoatIdle:
			// Stopped exit: User input applied (i.e. gas pedal)
			if (system_information.control_active)
			{
				system_information.boat_state = kBoatDriving;
				osThreadFlagsSet(readMotorTaskHandle, THREAD_FLAG_DRIVING);
			}
			break;
    case kBoatDriving:
      // Driving exit: User input removed
      //               Collision detected or depth exceeded -> anchored
      if (!system_information.control_active)
      {
        // Clear flags
        osThreadFlagsSet(readMotorTaskHandle, 0x0);

        // If no control is active, return to "idle" state
        system_information.boat_state = kBoatIdle;
      }
      else if (system_information.collision_detected || system_information.depth_exceeded)
      {
        // Clear flags
        osThreadFlagsSet(readMotorTaskHandle, 0x0);

        // If an error occurs (i.e. collision detection or depth exceeded), 
        system_information.boat_state = kBoatError;
      }
      break;
    case kBoatAnchored:
      if (system_information.anchor_lifted)
      {
        // Once the anchor is listed, we now move back to "idle" state
        system_information.boat_state = kBoatIdle;
      }
      break;
    case kBoatError:
      if (!system_information.collision_detected && !system_information.depth_exceeded)
      {
        // If error conditions have cleared, return to "idle" state
        system_information.boat_state = kBoatIdle;
      }
      break;
    }
    osMutexRelease(mutexSystemInfoHandle);
  }
  /* USER CODE END StartTaskBoatSM */
}

/* USER CODE BEGIN Header_StartTaskDepthDetect */
/**
* @brief Function implementing the depthDetectTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskDepthDetect */
void StartTaskDepthDetect(void *argument)
{
  /* USER CODE BEGIN StartTaskDepthDetect */
  /* Infinite loop */
  uint16_t detected_depth_raw;
  vl53l0x_prepare_sample(tof_interface);
  printf("Kicked off depth sampling\n");

  for(;;)
  {
    osSemaphoreAcquire(semaphoreToFISRHandle, osWaitForever);
    vl53l0x_read_range_single(tof_interface, &detected_depth_raw, false);

    printf("Depth: %d\n", detected_depth_raw);

    vl53l0x_prepare_sample(tof_interface);
  }
  /* USER CODE END StartTaskDepthDetect */
}

/* USER CODE BEGIN Header_StartTaskReadMotorCommands */
/**
* @brief Function implementing the readMotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskReadMotorCommands */
void StartTaskReadMotorCommands(void *argument)
{
  /* USER CODE BEGIN StartTaskReadMotorCommands */
  BoatCommand_t command;
  /* Infinite loop */
  for(;;)
  {
    // Wait for flags to clear
    osThreadFlagsWait(THREAD_FLAG_DRIVING, osFlagsNoClear, osWaitForever);
    if (osMessageQueueGet(queueBoatCommandHandle, (void *)&command, NULL, osWaitForever) == osOK)
    {
      // TODO: where will this queue get filled from?
      // TODO: diff drive PWM command
    }
  }
  /* USER CODE END StartTaskReadMotorCommands */
}

/* USER CODE BEGIN Header_StartTaskSendData */
/**
* @brief Function implementing the sendDataTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskSendData */
void StartTaskSendData(void *argument)
{
  /* USER CODE BEGIN StartTaskSendData */
  /* Infinite loop */
  for(;;)
  {
    // TODO: broadcast info over 
    osDelay(1);
  }
  /* USER CODE END StartTaskSendData */
}

/* callbackMotorTimeout function */
void callbackMotorTimeout(void *argument)
{
  /* USER CODE BEGIN callbackMotorTimeout */
	osThreadFlagsSet(motorTmoutTaskHandle, 0x0001);
  /* USER CODE END callbackMotorTimeout */
}

/* callbackSensorRead function */
void callbackSensorRead(void *argument)
{
  /* USER CODE BEGIN callbackSensorRead */

  /* USER CODE END callbackSensorRead */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
