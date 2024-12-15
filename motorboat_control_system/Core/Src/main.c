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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "vl53l0x_driver.h"
#include "stm32l475e_iot01_accelero.h"
#include "app_bluenrg_ms.h"
#include "sample_service.h"
#include "hci_tl.h"
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

typedef enum {
	kCommandMotion = 0,
	kCommandState = 1
} BoatCommandType_t;

typedef enum {
	kRecoveryRequest = 0,
	kAnchorBoat = 1
} RequestedState_t;

#define RX_BUFFER_SIZE (5)

typedef union __attribute__((packed)) {
	struct __attribute__((packed)) {
		BoatCommandType_t cmd_type : 8;
		union {
			struct __attribute__((packed)) {

				int16_t angle;   // (-32767, 32768] -> (-pi, pi]
				uint16_t speed;  // (0 -> 65536] -> PWM duty cycle
			} motion;
			struct __attribute__((packed)) {
				RequestedState_t state: 8;
				uint32_t reserved : 24;
			} state;
		} cmd;
	} fields;
	uint8_t buffer[RX_BUFFER_SIZE]; // 5 bytes right now...
} BoatCommand_t;

typedef enum {
  kDirectionCCW = +1,
  kDirectionNull = 0,
  kDirectionCW = -1
} Direction_t; // CCW (+), CW (-)

typedef struct {
    TIM_HandleTypeDef *timer;  // Pointer to the timer (e.g., TIM2, TIM3)
    uint32_t channel;         // PWM channel (e.g., TIM_CHANNEL_1)
} MotorTimerConfig_t;

typedef enum{
	PWM_OK = 0,
	PWM_ERROR = 1
} PWMstatus_t;

typedef struct {
  uint16_t duty_cycle: 7; // 0-100
  bool timeout: 1;
  bool is_alive: 1;
  bool is_idle: 1;
  uint8_t direction: 2; // represent as u2 int (CW: 0, NULL: 1, CCW: 2)
  uint8_t reserved : 4; // for alignment
} MotorStatus_t; // total size: 2 bytes

#define NUM_MOTORS       (4)
#define NUM_SPARE_MOTORS (1)
#define BLE_PACKET_SIZE  (20)

typedef union {
	 struct __attribute__((packed)) {
		  BoatState_t boat_state: 2;
		  bool control_active: 1;
		  bool collision_detected: 1;
		  bool depth_exceeded: 1;
		  bool anchor_lifted: 1;
		  uint8_t reserved : 2; // for alignment                       // 1 byte
		  uint16_t depth_mm: 16;                                           // 2 bytes
		  int8_t tilt_roll: 8;                                            // 1 byte
		  int8_t tilt_pitch: 8;                                           // 1 byte
		  uint32_t tick: 32;                                               // 4 bytes
		  MotorStatus_t motor_statuses[NUM_MOTORS + NUM_SPARE_MOTORS]; // 10 bytes
		  uint8_t reserved2;                                           // 1 byte, padding for clean 20 piece
	} fields;
	uint8_t buffer[BLE_PACKET_SIZE];
} SystemInformation_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define THREAD_FLAG_DRIVING 0x1
#define THREAD_FLAG_DEPTH_READING_READY 0x1
#define THREAD_FLAG_COLLISION_DETECTED 0x1
#define THREAD_FLAG_CHECK_MOTOR_TIMEOUT 0x1

#define MOTOR_FAILURE_CHANCE 20.0f // percent chance of motor failure from collision
#define MOTOR_TIMEOUT_INTERVAL 2000 // motor idle timeount in ms

#define DEPTH_RANGE_MAXIMUM_MM                1000 // 1m
#define ACCEL_SAMPLING_RATE                   100 // 10 Hz
#define BLE_TRANSMISSION_RATE                 100 // 10 Hz. TODO: increase as high as we can...
#define MAX_REPORTED_TILT_DEG                 120 // within u8

#define COMMAND_MSG_QUEUE_PRI                 (10)
#define COMMAND_MSG_QUEUE_TIMEOUT             (0)    // No timeout
#define STATE_MSG_QUEUE_PRI                   (10)
#define STATE_MSG_QUEUE_TIMEOUT               (0) // TODO: set timeout when we start servicing this queue

#define PWM_MAX_VALUE                         (100)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// Convert convert 7-bit duty cycle value to correct timer CRR using PWM frequency
#define DUTY_TO_CCR(duty_cycle) \
    (((uint32_t)(duty_cycle) * (TIM_ARR + 1)) / 100)

#define RAD_2_DEG(x) ((x) * 180 / 3.14159265f)
#define SIGN(x) ((x) != 0 ? ((x) >= 0 ? 1 : -1) : 0)
#define CLAMP(x, _max_) ((SIGN(x)) * ((x) > (_max_) ? (_max_) : x))
#define RESCALE(x, _in_max_, _out_max_) ((x) * (_out_max_) / (_in_max_))
#define DIRECTION_TO_S2(x) ((x) + 1)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;

I2C_HandleTypeDef hi2c2;

QSPI_HandleTypeDef hqspi;

RNG_HandleTypeDef hrng;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
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
/* Definitions for serviceBLETask */
osThreadId_t serviceBLETaskHandle;
uint32_t serviceBLETaskBuffer[ 256 ];
osStaticThreadDef_t serviceBLETaskControlBlock;
const osThreadAttr_t serviceBLETask_attributes = {
  .name = "serviceBLETask",
  .cb_mem = &serviceBLETaskControlBlock,
  .cb_size = sizeof(serviceBLETaskControlBlock),
  .stack_mem = &serviceBLETaskBuffer[0],
  .stack_size = sizeof(serviceBLETaskBuffer),
  .priority = (osPriority_t) osPriorityHigh2,
};
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
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for tiltDetectTask */
osThreadId_t tiltDetectTaskHandle;
uint32_t tiltDetectionTaBuffer[ 256 ];
osStaticThreadDef_t tiltDetectionTaControlBlock;
const osThreadAttr_t tiltDetectTask_attributes = {
  .name = "tiltDetectTask",
  .cb_mem = &tiltDetectionTaControlBlock,
  .cb_size = sizeof(tiltDetectionTaControlBlock),
  .stack_mem = &tiltDetectionTaBuffer[0],
  .stack_size = sizeof(tiltDetectionTaBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for collisionDetect */
osThreadId_t collisionDetectHandle;
uint32_t collisionDetectBuffer[ 128 ];
osStaticThreadDef_t collisionDetectControlBlock;
const osThreadAttr_t collisionDetect_attributes = {
  .name = "collisionDetect",
  .cb_mem = &collisionDetectControlBlock,
  .cb_size = sizeof(collisionDetectControlBlock),
  .stack_mem = &collisionDetectBuffer[0],
  .stack_size = sizeof(collisionDetectBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for boatControlTask */
osThreadId_t boatControlTaskHandle;
uint32_t boatControlTaskBuffer[ 128 ];
osStaticThreadDef_t boatControlTaskControlBlock;
const osThreadAttr_t boatControlTask_attributes = {
  .name = "boatControlTask",
  .cb_mem = &boatControlTaskControlBlock,
  .cb_size = sizeof(boatControlTaskControlBlock),
  .stack_mem = &boatControlTaskBuffer[0],
  .stack_size = sizeof(boatControlTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for queueBoatCommand */
osMessageQueueId_t queueBoatCommandHandle;
const osMessageQueueAttr_t queueBoatCommand_attributes = {
  .name = "queueBoatCommand"
};
/* Definitions for queueBoatReqState */
osMessageQueueId_t queueBoatReqStateHandle;
const osMessageQueueAttr_t queueBoatReqState_attributes = {
  .name = "queueBoatReqState"
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
/* USER CODE BEGIN PV */
VL53l0X_Interface_t *tof_intf = NULL;

// Create an array mapping motors to their timers and channels
MotorTimerConfig_t motorTimerConfig[NUM_MOTORS + NUM_SPARE_MOTORS] = {
    {&htim2, TIM_CHANNEL_1}, // Motor 1 -> TIM2, Channel 1
    {&htim2, TIM_CHANNEL_2}, // Motor 2 -> TIM2, Channel 2
    {&htim2, TIM_CHANNEL_3}, // Motor 3 -> TIM2, Channel 3
    {&htim2, TIM_CHANNEL_4}, // Motor 4 -> TIM2, Channel 4
    {&htim3, TIM_CHANNEL_1}, // Motor 5 -> TIM3, Channel 1
};

volatile static SystemInformation_t system_information = {
	.fields = {
		  .anchor_lifted = false,
		  .boat_state = kBoatIdle,
		  .collision_detected = false,
		  .control_active = false,
		  .depth_exceeded = false,
		  .motor_statuses = {
			  {.timeout = false, .is_alive = true, .is_idle = false, .direction = kDirectionNull, .duty_cycle = DUTY_TO_CCR(0)},
			  {.timeout = false, .is_alive = true, .is_idle = false, .direction = kDirectionNull, .duty_cycle = DUTY_TO_CCR(0)},
			  {.timeout = false, .is_alive = true, .is_idle = false, .direction = kDirectionNull, .duty_cycle = DUTY_TO_CCR(0)},
			  {.timeout = false, .is_alive = true, .is_idle = false, .direction = kDirectionNull, .duty_cycle = DUTY_TO_CCR(0)},
			  {.timeout = false, .is_alive = true, .is_idle = true,  .direction = kDirectionNull, .duty_cycle = DUTY_TO_CCR(0)}
		  }
	}
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_I2C2_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RNG_Init(void);
void StartDefaultTask(void *argument);
void StartTaskBoatSM(void *argument);
void StartTaskDepthDetect(void *argument);
void StartBLECommTask(void *argument);
void StartTaskMotorTmout(void *argument);
void StartTiltDetection(void *argument);
void startCollisionDetectTask(void *argument);
void StartBoatControl(void *argument);
void callbackMotorTimeout(void *argument);
void callbackSensorRead(void *argument);

/* USER CODE BEGIN PFP */
PWMstatus_t initMotorPWM(volatile SystemInformation_t *system_info);
PWMstatus_t updateMotorDutyCycle(volatile SystemInformation_t *system_info, uint8_t motor_index, uint16_t duty_cycle);
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
    	// If we triggered the ToF sensor ISR, mark as ready and unblock the depth detection task
    	vl53l0x_set_isr_flag();

    	if (depthDetectTaskHandle != NULL)
    	{
			// Unblock thread flags task
			osThreadFlagsSet(depthDetectTaskHandle, THREAD_FLAG_DEPTH_READING_READY);
    	}
    }

    if(GPIO_Pin == GPIO_PIN_13)
    {
    	// Collision detected
    	osThreadFlagsSet(collisionDetectHandle, THREAD_FLAG_COLLISION_DETECTED);
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DFSDM1_Init();
  MX_I2C2_Init();
  MX_QUADSPI_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */
  initMotorPWM(&system_information);

  MX_BlueNRG_MS_Init();

  if (BSP_ACCELERO_Init() != 0)
  {
    printf("Failed to initialize acceleromter\n");
    Error_Handler();
  }

  if (sizeof(system_information.buffer) > 20)
  {
    printf("Packet size too for BLE transmission\n");
    Error_Handler();
  }

  tof_intf = vl53l0x_init(&hi2c2, 0x52, VL53L0X_XSHUT_GPIO_Port, VL53L0X_XSHUT_Pin);
  if (tof_intf == NULL)
  {
	  printf("Failed to initialize VL53L0X\n");
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
  if(osTimerStart(timerMotorTimeoutHandle, 100) != osOK)
  {
	  Error_Handler();
  }
  if(osTimerStart(timerSensorReadHandle, 100) != osOK)
  {
	  Error_Handler();
  }
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of queueBoatCommand */
  queueBoatCommandHandle = osMessageQueueNew (1, sizeof(BoatCommand_t), &queueBoatCommand_attributes);

  /* creation of queueBoatReqState */
  queueBoatReqStateHandle = osMessageQueueNew (30, sizeof(BoatCommand_t), &queueBoatReqState_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of boatSMTask */
  boatSMTaskHandle = osThreadNew(StartTaskBoatSM, NULL, &boatSMTask_attributes);

  /* creation of depthDetectTask */
  depthDetectTaskHandle = osThreadNew(StartTaskDepthDetect, NULL, &depthDetectTask_attributes);

  /* creation of serviceBLETask */
  serviceBLETaskHandle = osThreadNew(StartBLECommTask, NULL, &serviceBLETask_attributes);

  /* creation of motorTmoutTask */
  motorTmoutTaskHandle = osThreadNew(StartTaskMotorTmout, NULL, &motorTmoutTask_attributes);

  /* creation of tiltDetectTask */
  tiltDetectTaskHandle = osThreadNew(StartTiltDetection, NULL, &tiltDetectTask_attributes);

  /* creation of collisionDetect */
  collisionDetectHandle = osThreadNew(startCollisionDetectTask, NULL, &collisionDetect_attributes);

  /* creation of boatControlTask */
  boatControlTaskHandle = osThreadNew(StartBoatControl, NULL, &boatControlTask_attributes);

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
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_RNG;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = TIM_PSC;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = TIM_ARR;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = TIM_PSC;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = TIM_ARR;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_COMBINED_RESETTRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_OTG_FS_PWR_EN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPBTLE_RF_SPI3_CSN_GPIO_Port, SPBTLE_RF_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPBTLE_RF_RST_GPIO_Port, SPBTLE_RF_RST_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : ARD_D1_Pin */
  GPIO_InitStruct.Pin = ARD_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(ARD_D1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D7_Pin */
  GPIO_InitStruct.Pin = ARD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D13_Pin ARD_D11_Pin */
  GPIO_InitStruct.Pin = ARD_D13_Pin|ARD_D11_Pin;
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

  /*Configure GPIO pin : SPBTLE_RF_RST_Pin */
  GPIO_InitStruct.Pin = SPBTLE_RF_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPBTLE_RF_RST_GPIO_Port, &GPIO_InitStruct);

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

// Use motor state to initialize PWM
PWMstatus_t initMotorPWM(volatile SystemInformation_t *system_info)
{
	uint16_t duty_cycle;

	// Initialize PWM duty cycles
	for(int i = 0; i < NUM_MOTORS + NUM_SPARE_MOTORS; i++)
	{
		duty_cycle = system_info->fields.motor_statuses[i].duty_cycle;

		if (duty_cycle > PWM_MAX_VALUE)
		{
			printf("Duty cycle out of range for motor index: %u", i);
			duty_cycle = 0;
			return PWM_ERROR;
		}
		// Set timer CCR register to correct duty cycle
		__HAL_TIM_SET_COMPARE(motorTimerConfig[i].timer,
				motorTimerConfig[i].channel,
				DUTY_TO_CCR(duty_cycle));

		// Start PWM on channel
		if(HAL_TIM_PWM_Start(motorTimerConfig[i].timer,motorTimerConfig[i].channel) != HAL_OK)
		{
			printf("Failed to start PWM for motor index: %u", i);
			return PWM_ERROR;
		}
	}
	return PWM_OK;
}

// Update motor state with new PWM values and update timers
PWMstatus_t updateMotorDutyCycle(volatile SystemInformation_t *system_info, uint8_t motor_index, uint16_t duty_cycle)
{
    // Check motor index is valid
    if (motor_index >= NUM_MOTORS + NUM_SPARE_MOTORS) {
        printf("Invalid motor index: %u\n", motor_index);
        return PWM_ERROR;
    }

    if (duty_cycle > 0x7F)
    {
    	printf("Duty cycle out of range for motor index: %u\n", motor_index);
    	return PWM_ERROR;
    }

    system_info->fields.motor_statuses[motor_index].duty_cycle = duty_cycle;

    __HAL_TIM_SET_COMPARE(motorTimerConfig[motor_index].timer,
    		motorTimerConfig[motor_index].channel, DUTY_TO_CCR(duty_cycle));
    return PWM_OK;
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
		switch (system_information.fields.boat_state)
		{
		case kBoatIdle:
			// Stopped exit: User input applied (i.e. gas pedal)
			if (system_information.fields.control_active)
			{
				system_information.fields.boat_state = kBoatDriving;
				// TODO: unblock command processor here
//				osThreadFlagsSet(readDataTaskHandle, THREAD_FLAG_DRIVING);
			}
			break;
		case kBoatDriving:
			// Driving exit: User input removed
			//               Collision detected or depth exceeded -> anchored
			if (!system_information.fields.control_active)
			{
				// Clear flags
				// TODO: disable command processor here
//				osThreadFlagsSet(readDataTaskHandle, 0x0);

				// If no control is active, return to "idle" state
				system_information.fields.boat_state = kBoatIdle;
			}
			else if (system_information.fields.collision_detected || system_information.fields.depth_exceeded)
			{
				// Clear flags
//				osThreadFlagsSet(readDataTaskHandle, 0x0);
				// TODO: disable command processor here

				// If an error occurs (i.e. collision detection or depth exceeded),
				system_information.fields.boat_state = kBoatError;
			}
			break;
		case kBoatAnchored:
			if (system_information.fields.anchor_lifted)
			{
				// Once the anchor is listed, we now move back to "idle" state
				system_information.fields.boat_state = kBoatIdle;
			}
			break;
		case kBoatError:
			if (!system_information.fields.collision_detected && !system_information.fields.depth_exceeded)
			{
				// If error conditions have cleared, return to "idle" state
				system_information.fields.boat_state = kBoatIdle;
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
  uint16_t range;
  /* Infinite loop */
  for(;;)
  {
	  if (vl53l0x_prepare_sample(tof_intf) != HAL_OK)
	  {
		  printf("Failed to prepare range sample\n");
	  }
	  if ((osThreadFlagsWait(THREAD_FLAG_DEPTH_READING_READY, osFlagsWaitAll, 1000) & osFlagsError) != 0)
	  {
		  // Flag timeout
		  printf("Error waiting for range ISR\n");
		  vl53l0x_read_range_single(tof_intf, &range, true);
	  }
	  else
	  {
		  vl53l0x_read_range_single(tof_intf, &range, false);
	  }

	  osMutexAcquire(mutexSystemInfoHandle, osWaitForever);
	  system_information.fields.depth_mm = range;
	  system_information.fields.depth_exceeded = range > DEPTH_RANGE_MAXIMUM_MM;
	  osMutexRelease(mutexSystemInfoHandle);
  }
  /* USER CODE END StartTaskDepthDetect */
}

/* USER CODE BEGIN Header_StartBLECommTask */
/**
* @brief Function implementing the serviceBLETask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBLECommTask */
void StartBLECommTask(void *argument)
{
  /* USER CODE BEGIN StartBLECommTask */
  /* Infinite loop */
	BoatCommand_t *cmd;
	uint8_t buf_tx[20];
	uint8_t *buf_rx;
	uint8_t n_received_bytes;
	uint32_t tick = osKernelGetTickCount();
	for(;;)
	{
		// Increment SW timer
		tick += BLE_TRANSMISSION_RATE;

		// Run BlueNRG process
		MX_BlueNRG_MS_Process();

		// Acquire the system info mutex
		osMutexAcquire(mutexSystemInfoHandle, osWaitForever);
		system_information.fields.tick = tick - BLE_TRANSMISSION_RATE; // remove the rate calc from the stamped time
		memcpy((void *)buf_tx, (void *)system_information.buffer, sizeof(system_information.buffer));
		osMutexRelease(mutexSystemInfoHandle);

		// Send data
		sendData(buf_tx, sizeof(buf_tx));
		buf_rx = get_latest_received_sample(&n_received_bytes);
		if (buf_rx != NULL)
		{
			// Sample is ready to enqueue
//			printf("Received %d bytes: ", n_received_bytes);
//			for (uint8_t i = 0; i < n_received_bytes; ++i)
//			{
//				printf("%c", buf_rx[i]);
//			}
//			printf("\n");
			if (n_received_bytes != 5)
			{
				printf("Incompatible buffer sizes (%d != 5)\n", n_received_bytes);
			}
			else
			{
				cmd = (BoatCommand_t *)buf_rx;
//				printf("raw: %02X%02X%02X%02X%02X\n", cmd->buffer[0], cmd->buffer[1], cmd->buffer[2], cmd->buffer[3], cmd->buffer[4]);

				switch (cmd->fields.cmd_type)
				{
					case kCommandMotion:
						printf("%d %d %d\n", cmd->fields.cmd_type, cmd->fields.cmd.motion.angle, cmd->fields.cmd.motion.speed);
						// No timeout, just put if there is any space
						osMessageQueuePut(queueBoatCommandHandle, (void *)cmd->buffer, COMMAND_MSG_QUEUE_PRI, COMMAND_MSG_QUEUE_TIMEOUT);
						break;
					case kCommandState:
//						printf("%d %d %ld\n", cmd->fields.cmd_type, cmd->fields.cmd.state.state, cmd->fields.cmd.state.reserved);
						// TODO: state queue...
						osMessageQueuePut(queueBoatReqStateHandle, (void *)cmd->buffer, STATE_MSG_QUEUE_PRI, STATE_MSG_QUEUE_TIMEOUT);
						break;
					default:
						// Invalid message type, ignore
						break;
				}
			}

		}
		osDelayUntil(tick);
	}
  /* USER CODE END StartBLECommTask */
}

/* USER CODE BEGIN Header_StartTaskMotorTmout */
/**
* @brief Function implementing the motorTmoutTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskMotorTmout */
void StartTaskMotorTmout(void *argument)
{
  /* USER CODE BEGIN StartTaskMotorTmout */
	uint32_t current_tick;
	uint32_t idle_start_time [NUM_MOTORS + NUM_SPARE_MOTORS] = {osKernelGetTickCount()};

	/* Infinite loop */
	for(;;)
	{
		osThreadFlagsWait(THREAD_FLAG_CHECK_MOTOR_TIMEOUT, osFlagsWaitAny, osWaitForever);

		osMutexAcquire(mutexSystemInfoHandle, osWaitForever);
		current_tick = osKernelGetTickCount();

		for(int i=0; i<(NUM_MOTORS + NUM_SPARE_MOTORS);i++){
			// only check idle status of active motors
			if(system_information.fields.motor_statuses[i].is_alive )
			{
				// Look at motors that are currently off
				if (system_information.fields.motor_statuses[i].duty_cycle == 0)
				{
					// If motor not already idle, and timeout condition met, set idle
					if (system_information.fields.motor_statuses[i].is_idle == false
							&& (current_tick - idle_start_time[i]) >= MOTOR_TIMEOUT_INTERVAL)
					{
						system_information.fields.motor_statuses[i].is_idle = true;
						printf("Motor %d idle\n", i);
					}
				}
				// If motor has non-zero PWN, turn off idle and store time
				else
				{
					system_information.fields.motor_statuses[i].is_idle = false;
					idle_start_time[i] = current_tick;
				}
			}
		}

		osMutexRelease(mutexSystemInfoHandle);
	}
  /* USER CODE END StartTaskMotorTmout */
}

/* USER CODE BEGIN Header_StartTiltDetection */
/**
* @brief Function implementing the tiltDetectionTa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTiltDetection */
void StartTiltDetection(void *argument)
{
  /* USER CODE BEGIN StartTiltDetection */
  int16_t accel_buf[3];
  float roll, pitch;
  uint32_t tick = osKernelGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    tick += ACCEL_SAMPLING_RATE;
    BSP_ACCELERO_AccGetXYZ(accel_buf);

    // Roll
    roll = RAD_2_DEG(atan2(accel_buf[1], accel_buf[2]));

    // Pitch
    pitch = RAD_2_DEG(
      atan2(
        -accel_buf[0], 
        sqrtf(accel_buf[1]*accel_buf[1] + accel_buf[2]*accel_buf[2])
      )
    );

    osMutexAcquire(mutexSystemInfoHandle, osWaitForever);
    system_information.fields.tilt_roll = (int8_t)(CLAMP(roll, MAX_REPORTED_TILT_DEG));
    system_information.fields.tilt_pitch = (int8_t)(CLAMP(pitch, MAX_REPORTED_TILT_DEG));
    osMutexRelease(mutexSystemInfoHandle);
    
    osDelayUntil(tick);
  }
  /* USER CODE END StartTiltDetection */
}

/* USER CODE BEGIN Header_startCollisionDetectTask */
/**
* @brief Function implementing the collisionDetect thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startCollisionDetectTask */
void startCollisionDetectTask(void *argument)
{
  /* USER CODE BEGIN startCollisionDetectTask */
	/* Infinite loop */
	uint32_t randNum;
	for(;;)
	{
		// Wait for pushbutton interrupt to execute
		osThreadFlagsWait(THREAD_FLAG_COLLISION_DETECTED, osFlagsWaitAny, osWaitForever);

		// Generate random number to determine if a collision was severe enough to cause a motor to fail
		HAL_RNG_GenerateRandomNumber(&hrng, &randNum);

		// Normalize the random number to a range of 0-100
		float normalizedRandNum = (randNum / (float)UINT32_MAX) * 100.0f;

		// Set collision flag
		osMutexAcquire(mutexSystemInfoHandle, osWaitForever);
		system_information.fields.collision_detected = true;
		osMutexRelease(mutexSystemInfoHandle);

		// Determine if collision caused motor failure
		if (normalizedRandNum < MOTOR_FAILURE_CHANCE)
		{
			// determine which motor failed, spare can also fail
			int failed_motor_index = (int)(randNum % (NUM_MOTORS + NUM_SPARE_MOTORS));
			printf("motor %d failed\n", failed_motor_index);

			osMutexAcquire(mutexSystemInfoHandle, osWaitForever);
			system_information.fields.motor_statuses[failed_motor_index].is_alive = false;
			osMutexRelease(mutexSystemInfoHandle);
		}

	}
  /* USER CODE END startCollisionDetectTask */
}

/* USER CODE BEGIN Header_StartBoatControl */
/**
* @brief Function implementing the boatControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBoatControl */
void StartBoatControl(void *argument)
{
  /* USER CODE BEGIN StartBoatControl */
  BoatCommand_t command;
  uint8_t pri;
  /* Infinite loop */
  for(;;)
  {
    if (osMessageQueueGet(queueBoatCommandHandle, (void *)&command, &pri, osWaitForever) == osOK)
    {
      // Once we get a command from the queue, apply it to our motors
    	osMutexAcquire(mutexSystemInfoHandle, osWaitForever);
    	// hard-code motor 0 as "angle" motor
    	system_information.fields.motor_statuses[0].direction = DIRECTION_TO_S2(SIGN(command.fields.cmd.motion.angle));
		updateMotorDutyCycle(
			&system_information, 0, abs(RESCALE(command.fields.cmd.motion.angle, INT16_MAX, PWM_MAX_VALUE))
		);

		// for now, all thrust motors just go forward
		system_information.fields.motor_statuses[1].direction = DIRECTION_TO_S2(kDirectionCCW);
		system_information.fields.motor_statuses[2].direction = DIRECTION_TO_S2(kDirectionCCW);
		system_information.fields.motor_statuses[3].direction = DIRECTION_TO_S2(kDirectionCCW);
		updateMotorDutyCycle(
			&system_information, 1, abs(RESCALE(command.fields.cmd.motion.speed, UINT16_MAX, PWM_MAX_VALUE))
		);
		updateMotorDutyCycle(
			&system_information, 2, abs(RESCALE(command.fields.cmd.motion.speed, UINT16_MAX, PWM_MAX_VALUE))
		);
		updateMotorDutyCycle(
			&system_information, 3, abs(RESCALE(command.fields.cmd.motion.speed, UINT16_MAX, PWM_MAX_VALUE))
		);

		osMutexRelease(mutexSystemInfoHandle);
    }
  }
  /* USER CODE END StartBoatControl */
}

/* callbackMotorTimeout function */
void callbackMotorTimeout(void *argument)
{
  /* USER CODE BEGIN callbackMotorTimeout */
	osThreadFlagsSet(motorTmoutTaskHandle, THREAD_FLAG_CHECK_MOTOR_TIMEOUT);
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
	while(1);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
