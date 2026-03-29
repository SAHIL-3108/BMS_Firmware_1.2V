/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : BMS Firmware — STM32G473CET6
 ******************************************************************************
 *
 * VERIFIED PIN MAP (from schematic BMS_FM_1.2V):
 * ─────────────────────────────────────────────────────────────────────────
 *  PA0  │ V_SENSE     │ ADC1_IN1        │ Pack voltage sense (analog)
 *  PA2  │ USART_TX    │ USART2_TX       │ Debug UART TX
 *  PA3  │ USART_RX    │ USART2_RX       │ Debug UART RX
 *  PA4  │ NSS         │ SPI1_NSS        │ EEPROM chip select
 *  PA5  │ SCLK        │ SPI1_SCK        │ EEPROM clock
 *  PA6  │ MISO        │ SPI1_MISO       │ EEPROM data in
 *  PA7  │ MOSI        │ SPI1_MOSI       │ EEPROM data out
 *  PA8  │ IM_SWITCH   │ GPIO_OUTPUT     │ Current monitor enable (HIGH=on)
 *  PA9  │ SCL         │ I2C2_SCL        │ BQ76952 AFE primary bus
 *  PA11 │ D-          │ USB_DM          │ USB full-speed
 *  PA12 │ D+          │ USB_DP          │ USB full-speed
 *  PA13 │ SWDIO       │ SYS_JTMS        │ SWD debug
 *  PA14 │ SWCLK       │ SYS_JTCK        │ SWD debug
 *  PB5  │ SDA         │ I2C3_SDA        │ Secondary I2C bus
 *  PB12 │ CAN_RX      │ FDCAN2_RX       │ CAN bus receive
 *  PB13 │ CAN_TX      │ FDCAN2_TX       │ CAN bus transmit
 *  PB14 │ TX_EN       │ GPIO_OUTPUT     │ CAN transceiver enable (HIGH=on)
 *  PB15 │ nALERT      │ EXTI_FALLING    │ BQ76952 fault alert (active-LOW)
 *
 * PENDING (share Sheet C/D to complete):
 *  ???  │ CONTACTOR_+ │ GPIO_OUTPUT     │ Main positive contactor
 *  ???  │ CONTACTOR_- │ GPIO_OUTPUT     │ Main negative contactor
 *  ???  │ PRECHARGE   │ GPIO_OUTPUT     │ Precharge contactor
 *  ???  │ CHARGER_DET │ GPIO_INPUT/EXTI │ Charger plug detection
 *  ???  │ COOLING_PWM │ TIM_CH          │ Cooling fan PWM
 *  ???  │ HEATING_PWM │ TIM_CH          │ Cell heater PWM
 *
 * CORRECTIONS vs previous version:
 *  [1] PB14 = CAN TX_EN, not contactor — drive HIGH before CAN start
 *  [2] PB15 = nALERT active-LOW, EXTI FALLING edge, not charger detect rising
 *  [3] PA8  = IM_SWITCH output HIGH on boot, not digital input
 *  [4] FDCAN2 bit timing fixed: 500kbps @ 170MHz
 *      Prescaler=17, SJW=4, TimeSeg1=10, TimeSeg2=9
 *  [5] Contactor/charger/cooling functions are safe stubs until Sheet C/D
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "battery_structs.h"
#include "bms_config.h"
#include "soc_algorithm.h"
#include "fault_manager.h"
#include "thermal_model.h"
#include "cell_balancing.h"
#include "can_stack.h"
#include "afe_bq76952.h"
#include "eeprom_manager.h"
#include "stm32g4xx_hal_pwr_ex.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	EVT_NONE = 0U,
	EVT_POST_PASS = 1U,
	EVT_POST_FAIL = 2U,
	EVT_CONTACTOR_CLOSE_REQ = 3U,
	EVT_PRECHARGE_DONE = 4U,
	EVT_PRECHARGE_TIMEOUT = 5U,
	EVT_CHARGER_CONNECTED = 6U,
	EVT_CHARGER_DISCONNECTED = 7U,
	EVT_FAULT_LEVEL3 = 8U,
	EVT_FAULT_CLEARED = 9U,
	EVT_SLEEP_REQUEST = 10U,
	EVT_WAKEUP = 11U,
	EVT_SOC_FULL = 12U,
	EVT_AFE_ALERT = 13U, /* NEW: BQ76952 nALERT fired        */
} BmsEvent_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ── Verified from schematic ─────────────────────────────────────────── */
#define CAN_TXEN_PORT       GPIOB
#define CAN_TXEN_PIN        GPIO_PIN_14   /* PB14 = TX_EN, HIGH = enabled  */

#define AFE_ALERT_PORT      GPIOB
#define AFE_ALERT_PIN       GPIO_PIN_15   /* PB15 = nALERT, active-LOW     */

#define IMON_SW_PORT        GPIOA
#define IMON_SW_PIN         GPIO_PIN_8    /* PA8  = IM_SWITCH, HIGH = on   */

#define EEPROM_CS_PORT      GPIOA
#define EEPROM_CS_PIN       GPIO_PIN_4    /* PA4  = SPI1_NSS               */

/* ── Pending — update when Sheet C/D is shared ───────────────────────── */
/* Contactor GPIOs — set to safe dummy values until confirmed             */
#define CONTACTOR_MAIN_PORT     GPIOC        /* TODO: replace with real port */
#define CONTACTOR_MAIN_PIN      GPIO_PIN_13  /* TODO: replace with real pin  */
#define CONTACTOR_PRE_PORT      GPIOC        /* TODO: replace with real port */
#define CONTACTOR_PRE_PIN       GPIO_PIN_14  /* TODO: replace with real pin  */
#define CHARGER_DET_PORT        GPIOC        /* TODO: replace with real port */
#define CHARGER_DET_PIN         GPIO_PIN_15  /* TODO: replace with real pin  */

/* ── ADC ─────────────────────────────────────────────────────────────── */
#define BUS_VOLTAGE_ADC_CH      ADC_CHANNEL_1  /* PA0 = ADC1_IN1            */
#define ADC_VREF_MV             (3300U)
#define ADC_FULL_SCALE          (4096U)
#define BUS_DIVIDER_RATIO       (100U)          /* 100:1 → ~400V pack        */

/* ── EEPROM SPI commands ─────────────────────────────────────────────── */
#define EEPROM_CMD_WREN         (0x06U)
#define EEPROM_CMD_WRITE        (0x02U)
#define EEPROM_CMD_READ         (0x03U)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

FDCAN_HandleTypeDef hfdcan2;

I2C_HandleTypeDef hi2c3;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

PCD_HandleTypeDef hpcd_USB_FS;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* USER CODE BEGIN PV */
BatteryPack_t g_BatteryPack;

static TaskHandle_t s_hTaskSafety = NULL;
static TaskHandle_t s_hTaskThermal = NULL;
static TaskHandle_t s_hTaskSoc = NULL;
static TaskHandle_t s_hTaskBalancing = NULL;
static TaskHandle_t s_hTaskCan = NULL;
static TaskHandle_t s_hTaskEeprom = NULL;

static SemaphoreHandle_t s_hPackMutex = NULL;
static QueueHandle_t s_hEventQueue = NULL;

/* Additional peripheral handles */
static I2C_HandleTypeDef hi2c2; /* PA9 SCL  — BQ76952 AFE bus        */
static TIM_HandleTypeDef htim3; /* PWM — cooling fan + heater         */

/* nALERT latch — set from EXTI15 falling-edge callback */
static volatile bool s_AfeAlertFired = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C3_Init(void);
static void MX_IWDG_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_PWM_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */
void Bms_Init(void);
void Task_Safety(void *pv);
void Task_Thermal(void *pv);
void Task_SocEstimation(void *pv);
void Task_Balancing(void *pv);
void Task_CanTelemetry(void *pv);
void Task_EepromManager(void *pv);
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
  MX_FDCAN2_Init();
  MX_USB_PCD_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C3_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

	/* Additional BMS peripherals */
	MX_I2C2_Init();

	MX_TIM3_PWM_Init();

	/* Start FDCAN2 after init */
	if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK) {
		Error_Handler();
	}

	/* Initialize BMS software modules */
	Bms_Init();

	/* RTOS primitives */
	s_hPackMutex = xSemaphoreCreateMutex();
	s_hEventQueue = xQueueCreate(16U, sizeof(BmsEvent_t));

	/* Create BMS tasks */
	xTaskCreate(Task_Safety, "Safety", TASK_STACK_SAFETY_WORDS, NULL,
			TASK_PRIORITY_SAFETY, &s_hTaskSafety);
	xTaskCreate(Task_Thermal, "Thermal", TASK_STACK_THERMAL_WORDS, NULL,
			TASK_PRIORITY_THERMAL, &s_hTaskThermal);
	xTaskCreate(Task_SocEstimation, "SOC", TASK_STACK_SOC_WORDS, NULL,
			TASK_PRIORITY_SOC, &s_hTaskSoc);
	xTaskCreate(Task_Balancing, "Balance", TASK_STACK_THERMAL_WORDS, NULL,
			TASK_PRIORITY_BALANCING, &s_hTaskBalancing);
	xTaskCreate(Task_CanTelemetry, "CAN_TX", TASK_STACK_CAN_WORDS, NULL,
			TASK_PRIORITY_CAN_TX, &s_hTaskCan);
	xTaskCreate(Task_EepromManager, "EEPROM", TASK_STACK_THERMAL_WORDS, NULL,
			TASK_PRIORITY_EEPROM, &s_hTaskEeprom);

	/* Heap sanity check — fires Error_Handler if heap is too small */
	configASSERT(xPortGetFreeHeapSize() > 2048U);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48
                              |RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
      Error_Handler();
  }
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */
  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */
  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 16;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 1;
  hfdcan2.Init.NominalTimeSeg2 = 1;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */
  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x40B285C2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */
  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */
  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.HwFlowCtl = UART_HWCONTROL_RTS;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */
  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */
  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */
  /* USER CODE END USB_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PG10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_15;
  GPIO_InitStruct.Mode =  GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB11 PB3 PB4 PB6
                           PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

	/* =========================================================================
	 * ADDITIONAL PERIPHERAL INITS
	 * ========================================================================= */

	/**
	 * @brief I2C2 — PA9 SCL, BQ76952 AFE, 400kHz
	 *        Timing 0x00802172 = 400kHz @ 170MHz PCLK1
	 */
	static void MX_I2C2_Init(void) {
		__HAL_RCC_I2C2_CLK_ENABLE();
		hi2c2.Instance = I2C2;
		hi2c2.Init.Timing = 0x00802172U;
		hi2c2.Init.OwnAddress1 = 0U;
		hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
		hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
		hi2c2.Init.OwnAddress2 = 0U;
		hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
		hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
		if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
			Error_Handler();
		}
	}


	/**
	 * @brief ADC1 — PA0 channel IN1, 12-bit, software triggered
	 */

	/**
	 * @brief IWDG — 100ms independent watchdog
	 *        LSI ~32kHz / 16 = 2kHz tick, reload 200 → 100ms
	 */

	/**
	 * @brief TIM3 PWM — 1kHz, two channels
	 *        CH1 = cooling fan PWM  (assign GPIO when Sheet C/D available)
	 *        CH2 = heater PWM       (assign GPIO when Sheet C/D available)
	 *        170MHz / (169+1) / (999+1) = 1kHz
	 */
	static void MX_TIM3_PWM_Init(void) {
		TIM_OC_InitTypeDef sConfig = { 0 };
		__HAL_RCC_TIM3_CLK_ENABLE();

		htim3.Instance = TIM3;
		htim3.Init.Prescaler = 169U;
		htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim3.Init.Period = 999U;
		htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
		if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
			Error_Handler();
		}

		sConfig.OCMode = TIM_OCMODE_PWM1;
		sConfig.Pulse = 0U;
		sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfig.OCFastMode = TIM_OCFAST_DISABLE;
		if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfig, TIM_CHANNEL_1)
				!= HAL_OK) {
			Error_Handler();
		}
		if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfig, TIM_CHANNEL_2)
				!= HAL_OK) {
			Error_Handler();
		}

		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	}

	/* =========================================================================
	 * HAL BRIDGE — maps BMS module calls → STM32G4 HAL
	 * ========================================================================= */

	uint32_t HAL_GetTickMs(void) {
		return HAL_GetTick();
	}

	void HAL_Watchdog_Kick(void) {
		(void) HAL_IWDG_Refresh(&hiwdg);
	}

	/* ── Contactor bridge ────────────────────────────────────────────────────
	 * TODO: replace CONTACTOR_MAIN_PORT/PIN with real pins from Sheet C/D.
	 * Currently stubbed safe — output is on a dummy PC13 pin which is analog
	 * mode so it has no electrical effect until you update the defines above.
	 * ─────────────────────────────────────────────────────────────────────── */
	void HAL_Contactor_OpenAll(void) {
		HAL_GPIO_WritePin(CONTACTOR_MAIN_PORT, CONTACTOR_MAIN_PIN,
				GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CONTACTOR_PRE_PORT, CONTACTOR_PRE_PIN,
				GPIO_PIN_RESET);
	}

	void HAL_Contactor_CloseMain(void) {
		HAL_GPIO_WritePin(CONTACTOR_MAIN_PORT, CONTACTOR_MAIN_PIN,
				GPIO_PIN_SET);
	}

	void HAL_Contactor_ClosePrecharge(void) {
		HAL_GPIO_WritePin(CONTACTOR_PRE_PORT, CONTACTOR_PRE_PIN, GPIO_PIN_SET);
	}

	bool HAL_Contactor_IsChargerPlugged(void) {
		/* TODO: read real charger detect GPIO from Sheet C/D */
		/* Returning false keeps BMS in STANDBY safely until pin is known */
		return false;
	}

	/* ── Bus voltage ─────────────────────────────────────────────────────── */
	uint32_t HAL_BusVoltage_ReadMv(void) {
		uint32_t raw = (uint32_t) HAL_ADC_ReadChannel(ADC_CHANNEL_1);
		return (raw * ADC_VREF_MV * BUS_DIVIDER_RATIO) / ADC_FULL_SCALE;
	}


	/* ── I2C bridge (BQ76952 on I2C2 / PA9) ──────────────────────────────── */
	bool HAL_I2C_Write(uint8_t dev_addr, const uint8_t *p_buf, uint8_t len,
			uint32_t timeout_ms) {
		return (HAL_I2C_Master_Transmit(&hi2c2, (uint16_t) (dev_addr << 1U),
				(uint8_t*) p_buf, (uint16_t) len, timeout_ms) == HAL_OK);
	}

	bool HAL_I2C_Read(uint8_t dev_addr, uint8_t reg, uint8_t *p_buf,
			uint8_t len, uint32_t timeout_ms) {
		if (HAL_I2C_Master_Transmit(&hi2c2, (uint16_t) (dev_addr << 1U), &reg,
				1U, timeout_ms) != HAL_OK) {
			return false;
		}
		return (HAL_I2C_Master_Receive(&hi2c2, (uint16_t) (dev_addr << 1U),
				p_buf, (uint16_t) len, timeout_ms) == HAL_OK);
	}

	/* ── CAN bridge (FDCAN2 / PB12=RX PB13=TX) ───────────────────────────── */
	bool HAL_CAN_Transmit(uint32_t id, const uint8_t *p_data, uint8_t dlc,
	bool extended) {
		FDCAN_TxHeaderTypeDef hdr = { 0 };
		hdr.Identifier = id;
		hdr.IdType = extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
		hdr.TxFrameType = FDCAN_DATA_FRAME;
		hdr.DataLength = (uint32_t) dlc << 16U;
		hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
		hdr.BitRateSwitch = FDCAN_BRS_OFF;
		hdr.FDFormat = FDCAN_CLASSIC_CAN;
		hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
		hdr.MessageMarker = 0U;
		return (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &hdr, (uint8_t*) p_data)
				== HAL_OK);
	}

	bool HAL_CAN_Receive(uint32_t *p_id, uint8_t *p_data, uint8_t *p_dlc) {
		FDCAN_RxHeaderTypeDef hdr = { 0 };
		if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0) == 0U) {
			return false;
		}
		if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &hdr, p_data)
				!= HAL_OK) {
			return false;
		}
		*p_id = hdr.Identifier;
		*p_dlc = (uint8_t) ((hdr.DataLength >> 16U) & 0x0FU);
		return true;
	}

	/* ── ADC bridge ──────────────────────────────────────────────────────── */
	uint16_t HAL_ADC_ReadChannel(uint8_t channel) {
		ADC_ChannelConfTypeDef cfg = { 0 };
		cfg.Channel = (uint32_t) channel;
		cfg.Rank = ADC_REGULAR_RANK_1;
		cfg.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
		cfg.SingleDiff = ADC_SINGLE_ENDED;
		cfg.OffsetNumber = ADC_OFFSET_NONE;
		cfg.Offset = 0U;
		if (HAL_ADC_ConfigChannel(&hadc1, &cfg) != HAL_OK) {
			return 0U;
		}
		HAL_ADC_Start(&hadc1);
		if (HAL_ADC_PollForConversion(&hadc1, 10U) == HAL_OK) {
			uint32_t v = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Stop(&hadc1);
			return (uint16_t) (v & 0x0FFFU);
		}
		HAL_ADC_Stop(&hadc1);
		return 0U;
	}

	/* ── PWM bridge (TIM3 CH1=cooling, CH2=heating) ─────────────────────── */
	void HAL_Cooling_SetDuty(float duty_0_to_1) {
		uint32_t pulse = (uint32_t) (duty_0_to_1 * 999.0F);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
	}

	void HAL_Heating_SetDuty(float duty_0_to_1) {
		uint32_t pulse = (uint32_t) (duty_0_to_1 * 999.0F);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse);
	}

	/* ── SPI EEPROM bridge (SPI1 / PA4=NSS) ─────────────────────────────── */
	bool HAL_EEPROM_Write(uint16_t address, const uint8_t *p_data, uint16_t len) {
		uint8_t cmd[3];
		HAL_StatusTypeDef s;

		cmd[0] = EEPROM_CMD_WREN;
		HAL_GPIO_WritePin(EEPROM_CS_PORT, EEPROM_CS_PIN, GPIO_PIN_RESET);
		s = HAL_SPI_Transmit(&hspi1, cmd, 1U, 10U);
		HAL_GPIO_WritePin(EEPROM_CS_PORT, EEPROM_CS_PIN, GPIO_PIN_SET);
		if (s != HAL_OK) {
			return false;
		}

		cmd[0] = EEPROM_CMD_WRITE;
		cmd[1] = (uint8_t) ((address >> 8U) & 0xFFU);
		cmd[2] = (uint8_t) (address & 0xFFU);
		HAL_GPIO_WritePin(EEPROM_CS_PORT, EEPROM_CS_PIN, GPIO_PIN_RESET);
		s = HAL_SPI_Transmit(&hspi1, cmd, 3U, 10U);
		if (s == HAL_OK) {
			s = HAL_SPI_Transmit(&hspi1, (uint8_t*) p_data, len, 50U);
		}
		HAL_GPIO_WritePin(EEPROM_CS_PORT, EEPROM_CS_PIN, GPIO_PIN_SET);
		HAL_Delay(5U);
		return (s == HAL_OK);
	}

	bool HAL_EEPROM_Read(uint16_t address, uint8_t *p_data, uint16_t len) {
		uint8_t cmd[3];
		HAL_StatusTypeDef s;

		cmd[0] = EEPROM_CMD_READ;
		cmd[1] = (uint8_t) ((address >> 8U) & 0xFFU);
		cmd[2] = (uint8_t) (address & 0xFFU);
		HAL_GPIO_WritePin(EEPROM_CS_PORT, EEPROM_CS_PIN, GPIO_PIN_RESET);
		s = HAL_SPI_Transmit(&hspi1, cmd, 3U, 10U);
		if (s == HAL_OK) {
			s = HAL_SPI_Receive(&hspi1, p_data, len, 50U);
		}
		HAL_GPIO_WritePin(EEPROM_CS_PORT, EEPROM_CS_PIN, GPIO_PIN_SET);
		return (s == HAL_OK);
	}

	/* ── GPIO EXTI callback ──────────────────────────────────────────────── */
	/**
	 * @brief  EXTI callback
	 *         PB15 = nALERT falling edge → BQ76952 hardware fault detected
	 *
	 * CHANGE: was "charger detect rising" — now "AFE alert falling"
	 */
	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
		if (GPIO_Pin == AFE_ALERT_PIN) /* PB15 nALERT fell LOW */
		{
			s_AfeAlertFired = true; /* Processed in safety task           */
		}
	}

	/* =========================================================================
	 * BMS INIT
	 * ========================================================================= */
	static void Bms_Init(void) {
		(void) memset(&g_BatteryPack, 0, sizeof(BatteryPack_t));
		g_BatteryPack.state = BMS_STATE_INIT;
		g_BatteryPack.previous_state = BMS_STATE_INIT;

		float saved_soc = SOC_INITIAL_PERCENT;
		float saved_soh = 100.0F;
		uint32_t saved_cycles = 0U;
		Eeprom_LoadPackState(&saved_soc, &saved_soh, &saved_cycles);

		Soc_Init(&g_BatteryPack.soc_soh, saved_soc, saved_soh);
		g_BatteryPack.soc_soh.cycle_count = saved_cycles;
		Fault_Init(&g_BatteryPack);
		Thermal_Init(&g_BatteryPack.thermal);
		Balance_Init();
		Can_Init(&g_BatteryPack);

		if (!AFE_Init()) {
			Fault_Assert(&g_BatteryPack, FAULT_AFE_COMM_TIMEOUT);
		}

		g_BatteryPack.max_charge_current_a = PACK_CURRENT_CHARGE_MAX_A;
		g_BatteryPack.max_discharge_current_a = PACK_CURRENT_DISCHARGE_MAX_A;

		/* Contactors open, TX_EN already HIGH from GPIO init */
		HAL_Contactor_OpenAll();
		g_BatteryPack.power_path.main_positive = CONTACTOR_OPEN;
		g_BatteryPack.power_path.main_negative = CONTACTOR_OPEN;
		g_BatteryPack.power_path.precharge = CONTACTOR_OPEN;
	}

	/* =========================================================================
	 * RTOS TASKS
	 * ========================================================================= */

	static void Task_Safety(void *pv) {
		(void) pv;
		TickType_t last_wake = xTaskGetTickCount();

		Bms_State_PowerOnSelfTest();

		for (;;) {
			HAL_Watchdog_Kick();

			/* Handle nALERT from BQ76952 — check latch set by EXTI callback */
			if (s_AfeAlertFired) {
				s_AfeAlertFired = false;
				FaultCode_t afe_fault = FAULT_NONE;
				(void) AFE_ReadSafetyFlags(0U, &afe_fault);
				if (afe_fault != FAULT_NONE) {
					Fault_Assert(&g_BatteryPack, afe_fault);
				}
			}

			Fault_RunSafetyChecks(&g_BatteryPack);

			BmsEvent_t event = Bms_DetectEvents();
			BmsEvent_t queued = EVT_NONE;
			if (xQueueReceive(s_hEventQueue, &queued, 0) == pdTRUE) {
				if (queued != EVT_NONE) {
					event = queued;
				}
			}
			if (event != EVT_NONE) {
				Bms_RunFsm(event);
			}

			switch (g_BatteryPack.state) {
			case BMS_STATE_STANDBY:
				Bms_State_Standby();
				break;
			case BMS_STATE_PRECHARGE:
				Bms_State_Precharge();
				break;
			case BMS_STATE_DRIVE:
				Bms_State_Drive();
				break;
			case BMS_STATE_CHARGING:
				Bms_State_Charging();
				break;
			case BMS_STATE_BALANCING:
				Bms_State_Balancing();
				break;
			case BMS_STATE_FAULT:
				Bms_State_Fault();
				break;
			case BMS_STATE_DEEP_SLEEP:
				Bms_State_DeepSleep();
				break;
			default:
				break;
			}

			g_BatteryPack.uptime_seconds =
					xTaskGetTickCount() / configTICK_RATE_HZ;
			vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_PERIOD_SAFETY_MS));
		}
	}

	static void Task_Thermal(void *pv) {
		(void) pv;
		TickType_t last_wake = xTaskGetTickCount();
		for (;;) {
			Thermal_UpdateSensors(&g_BatteryPack.thermal);
			Thermal_RunControl(&g_BatteryPack.thermal, g_BatteryPack.state);
			g_BatteryPack.pack_max_temp_dc =
					g_BatteryPack.thermal.max_temperature_dc;
			g_BatteryPack.pack_min_temp_dc =
					g_BatteryPack.thermal.min_temperature_dc;
			vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_PERIOD_THERMAL_MS));
		}
	}

	static void Task_SocEstimation(void *pv) {
		(void) pv;
		TickType_t last_wake = xTaskGetTickCount();
		uint32_t last_tick = HAL_GetTick();
		for (;;) {
			uint32_t now = HAL_GetTick();
			float dt_s = (float) (now - last_tick) / 1000.0F;
			last_tick = now;

			if ((g_BatteryPack.state != BMS_STATE_DEEP_SLEEP)
					&& (g_BatteryPack.state != BMS_STATE_INIT)) {
				float vt_mv = (float) g_BatteryPack.pack_voltage_mv
						/ (float) BMS_NUM_CELLS_SERIES;
				float current = (float) g_BatteryPack.pack_current_ma / 1000.0F;
				(void) Soc_Update(&g_BatteryPack.soc_soh, vt_mv, current, dt_s);

				uint8_t m;
				for (m = 0U; m < BMS_NUM_MODULES; m++) {
					(void) AFE_ReadCellVoltages(m, &g_BatteryPack.modules[m]);
				}
				(void) AFE_ReadPackCurrent(0U, &g_BatteryPack.pack_current_ma);
			}
			vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_PERIOD_SOC_MS));
		}
	}

	static void Task_Balancing(void *pv) {
		(void) pv;
		TickType_t last_wake = xTaskGetTickCount();
		for (;;) {
			Balance_Run(&g_BatteryPack);
			vTaskDelayUntil(&last_wake,
					pdMS_TO_TICKS(TASK_PERIOD_BALANCING_MS));
		}
	}

	static void Task_CanTelemetry(void *pv) {
		(void) pv;
		TickType_t last_wake = xTaskGetTickCount();
		for (;;) {
			Can_ProcessRxQueue(&g_BatteryPack);
			Can_TransmitTelemetry(&g_BatteryPack);
			vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_PERIOD_CAN_TX_MS));
		}
	}

	static void Task_EepromManager(void *pv) {
		(void) pv;
		TickType_t last_wake = xTaskGetTickCount();
		for (;;) {
			Eeprom_SavePackState(g_BatteryPack.soc_soh.soc_percent,
					g_BatteryPack.soc_soh.soh_percent,
					g_BatteryPack.soc_soh.cycle_count);
			Eeprom_SaveFaultLog(&g_BatteryPack.faults);
			vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_PERIOD_EEPROM_MS));
		}
	}

	/* =========================================================================
	 * FINITE STATE MACHINE
	 * ========================================================================= */
	static void Bms_RunFsm(BmsEvent_t event) {
		BmsState_t cur = g_BatteryPack.state;
		BmsState_t next = cur;

		switch (cur) {
		case BMS_STATE_POWER_ON_SELF_TEST:
			if (event == EVT_POST_PASS) {
				next = BMS_STATE_STANDBY;
			} else if (event == EVT_POST_FAIL) {
				next = BMS_STATE_FAULT;
			}
			break;
		case BMS_STATE_STANDBY:
			if (event == EVT_CONTACTOR_CLOSE_REQ) {
				next = BMS_STATE_PRECHARGE;
			} else if (event == EVT_CHARGER_CONNECTED) {
				next = BMS_STATE_CHARGING;
			} else if (event == EVT_SLEEP_REQUEST) {
				next = BMS_STATE_DEEP_SLEEP;
			} else if (event == EVT_FAULT_LEVEL3) {
				next = BMS_STATE_FAULT;
			}
			break;
		case BMS_STATE_PRECHARGE:
			if (event == EVT_PRECHARGE_DONE) {
				next = BMS_STATE_DRIVE;
			} else if (event == EVT_PRECHARGE_TIMEOUT) {
				next = BMS_STATE_FAULT;
			} else if (event == EVT_FAULT_LEVEL3) {
				next = BMS_STATE_FAULT;
			}
			break;
		case BMS_STATE_DRIVE:
			if (event == EVT_FAULT_LEVEL3) {
				next = BMS_STATE_FAULT;
			} else if (event == EVT_CHARGER_CONNECTED) {
				next = BMS_STATE_CHARGING;
			}
			break;
		case BMS_STATE_CHARGING:
			if (event == EVT_SOC_FULL) {
				next = BMS_STATE_BALANCING;
			} else if (event == EVT_CHARGER_DISCONNECTED) {
				next = BMS_STATE_STANDBY;
			} else if (event == EVT_FAULT_LEVEL3) {
				next = BMS_STATE_FAULT;
			}
			break;
		case BMS_STATE_BALANCING:
			if (event == EVT_CHARGER_DISCONNECTED) {
				next = BMS_STATE_STANDBY;
			} else if (event == EVT_FAULT_LEVEL3) {
				next = BMS_STATE_FAULT;
			}
			break;
		case BMS_STATE_FAULT:
			if (event == EVT_FAULT_CLEARED) {
				next = BMS_STATE_STANDBY;
			}
			break;
		case BMS_STATE_DEEP_SLEEP:
			if (event == EVT_WAKEUP) {
				next = BMS_STATE_STANDBY;
			}
			break;
		default:
			break;
		}

		if (next != cur) {
			Bms_EnterState(next);
		}
	}

	static void Bms_EnterState(BmsState_t new_state) {
		g_BatteryPack.previous_state = g_BatteryPack.state;
		g_BatteryPack.state = new_state;
		g_BatteryPack.state_entry_tick = HAL_GetTick();

		switch (new_state) {
		case BMS_STATE_STANDBY:
			HAL_Contactor_OpenAll();
			g_BatteryPack.power_path.main_positive = CONTACTOR_OPEN;
			g_BatteryPack.power_path.main_negative = CONTACTOR_OPEN;
			g_BatteryPack.power_path.precharge = CONTACTOR_OPEN;
			g_BatteryPack.power_path.precharge_complete = false;
			break;
		case BMS_STATE_PRECHARGE:
			HAL_Contactor_ClosePrecharge();
			g_BatteryPack.power_path.precharge = CONTACTOR_CLOSED;
			g_BatteryPack.power_path.precharge_start_tick = HAL_GetTick();
			g_BatteryPack.power_path.precharge_complete = false;
			break;
		case BMS_STATE_DRIVE:
			HAL_Contactor_CloseMain();
			g_BatteryPack.power_path.main_positive = CONTACTOR_CLOSED;
			g_BatteryPack.power_path.main_negative = CONTACTOR_CLOSED;
			g_BatteryPack.power_path.precharge = CONTACTOR_OPEN;
			break;
		case BMS_STATE_CHARGING:
			g_BatteryPack.power_path.charge_port = CONTACTOR_CLOSED;
			break;
		case BMS_STATE_FAULT:
			HAL_Contactor_OpenAll();
			g_BatteryPack.power_path.main_positive = CONTACTOR_OPEN;
			g_BatteryPack.power_path.main_negative = CONTACTOR_OPEN;
			g_BatteryPack.power_path.precharge = CONTACTOR_OPEN;
			g_BatteryPack.power_path.charge_port = CONTACTOR_OPEN;
			Balance_DisableAllCells(&g_BatteryPack);
			break;
		case BMS_STATE_DEEP_SLEEP:
			HAL_Contactor_OpenAll();
			Balance_DisableAllCells(&g_BatteryPack);
			HAL_DeepSleep_Enter();
			break;
		default:
			break;
		}
	}

	static void Bms_State_PowerOnSelfTest(void) {
		bool ok = true;
		uint8_t m, c;

		for (m = 0U; m < BMS_NUM_MODULES; m++) {
			if (!AFE_IsModuleOnline(m)) {
				ok = false;
			}
		}
		for (m = 0U; m < BMS_NUM_MODULES; m++) {
			for (c = 0U; c < BMS_CELLS_PER_MODULE; c++) {
				uint16_t v = g_BatteryPack.modules[m].cells[c].voltage_mv;
				if ((v < CELL_VOLTAGE_ABSOLUTE_MIN_MV)
						|| (v > CELL_VOLTAGE_ABSOLUTE_MAX_MV)) {
					ok = false;
				}
			}
		}
		if (HAL_BusVoltage_ReadMv() > (g_BatteryPack.pack_voltage_mv / 5U)) {
			Fault_Assert(&g_BatteryPack, FAULT_CONTACTOR_WELD);
			ok = false;
		}
		BmsEvent_t evt = ok ? EVT_POST_PASS : EVT_POST_FAIL;
		(void) xQueueSend(s_hEventQueue, &evt, 0);
		Bms_EnterState(BMS_STATE_STANDBY);
	}

	static void Bms_State_Standby(void) {
		if (HAL_Contactor_IsChargerPlugged()
				&& (g_BatteryPack.comms.charger_status != CHARGER_NOT_CONNECTED)) {
			BmsEvent_t evt = EVT_CHARGER_CONNECTED;
			(void) xQueueSend(s_hEventQueue, &evt, 0);
		}
	}

	static void Bms_State_Precharge(void) {
		uint32_t elapsed = HAL_GetTick()
				- g_BatteryPack.power_path.precharge_start_tick;
		if (elapsed >= PRECHARGE_TIMEOUT_MS) {
			Fault_Assert(&g_BatteryPack, FAULT_PRECHARGE_TIMEOUT);
			BmsEvent_t evt = EVT_PRECHARGE_TIMEOUT;
			(void) xQueueSend(s_hEventQueue, &evt, 0);
			return;
		}
		uint32_t target = (g_BatteryPack.pack_voltage_mv * PRECHARGE_TARGET_PCT)
				/ 100U;
		if (HAL_BusVoltage_ReadMv() >= target) {
			g_BatteryPack.power_path.precharge_complete = true;
			BmsEvent_t evt = EVT_PRECHARGE_DONE;
			(void) xQueueSend(s_hEventQueue, &evt, 0);
		}
	}

	static void Bms_State_Drive(void) {
		if (HAL_Contactor_IsChargerPlugged()) {
			BmsEvent_t evt = EVT_CHARGER_CONNECTED;
			(void) xQueueSend(s_hEventQueue, &evt, 0);
		}
	}

	static void Bms_State_Charging(void) {
		if (g_BatteryPack.soc_soh.soc_percent >= SOC_MAX_PERCENT) {
			Soc_OnFullChargeCycleComplete(&g_BatteryPack.soc_soh,
					g_BatteryPack.soc_soh.coulombs_accumulated);
			BmsEvent_t evt = EVT_SOC_FULL;
			(void) xQueueSend(s_hEventQueue, &evt, 0);
		}
		if (!HAL_Contactor_IsChargerPlugged()) {
			BmsEvent_t evt = EVT_CHARGER_DISCONNECTED;
			(void) xQueueSend(s_hEventQueue, &evt, 0);
		}
	}

	static void Bms_State_Balancing(void) {
		if (!HAL_Contactor_IsChargerPlugged()) {
			Balance_DisableAllCells(&g_BatteryPack);
			BmsEvent_t evt = EVT_CHARGER_DISCONNECTED;
			(void) xQueueSend(s_hEventQueue, &evt, 0);
		}
	}

	static void Bms_State_Fault(void) {
		if (g_BatteryPack.faults.active_fault_count == 0U) {
			BmsEvent_t evt = EVT_FAULT_CLEARED;
			(void) xQueueSend(s_hEventQueue, &evt, 0);
		}
	}

	static void Bms_State_DeepSleep(void) {
	}

	static BmsEvent_t Bms_DetectEvents(void) {
		if ((g_BatteryPack.faults.highest_severity == FAULT_LEVEL_3)
				&& (g_BatteryPack.state != BMS_STATE_FAULT)) {
			return EVT_FAULT_LEVEL3;
		}
		return EVT_NONE;
	}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
	/**
	 * @brief  Default task — idle only. All BMS logic is in dedicated tasks.
	 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
		for (;;) {
			osDelay(1000U);
		}
  /* USER CODE END 5 */
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
  if (htim->Instance == TIM6)
  {
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
		__disable_irq();
		HAL_GPIO_WritePin(CONTACTOR_MAIN_PORT, CONTACTOR_MAIN_PIN,
				GPIO_PIN_RESET);
		while (1) {
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
}
#endif /* USE_FULL_ASSERT */
