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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include <inttypes.h>  // ganz oben für PRIX32


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
FDCAN_HandleTypeDef hfdcan1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_FDCAN1_Init(void);
/* USER CODE BEGIN PFP */
#define SAMPLE_COUNT 1000
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} AccelSample;

AccelSample accelBuffer[SAMPLE_COUNT];
volatile uint16_t sampleIndex = 0;
volatile uint8_t recording = 0;
volatile uint8_t triggered = 0;
volatile uint8_t startSampling = 0;
#define CAN_ID_HEADER 0x321
#define CAN_DLC 8
volatile uint8_t can_send_requested = 0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define ADXL_CS_LOW()  HAL_GPIO_WritePin(SPI_CS_PIN_GPIO_Port, SPI_CS_PIN_Pin, GPIO_PIN_RESET)
#define ADXL_CS_HIGH() HAL_GPIO_WritePin(SPI_CS_PIN_GPIO_Port, SPI_CS_PIN_Pin, GPIO_PIN_SET)

uint8_t calculate_crc8(uint8_t *data, uint16_t length)
{
    uint8_t crc = 0x00;
    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
        }
    }
    return crc;
}

uint8_t ADXL345_ReadRegister(uint8_t reg)
{
    uint8_t tx[] = { 0x80 | reg, 0x00 }; // 0x80 = Lesezugriff
    uint8_t rx[2];

    ADXL_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
    ADXL_CS_HIGH();

    return rx[1]; // Registerwert steckt im zweiten Byte
}

void ADXL345_WriteRegister(uint8_t reg, uint8_t value)
{
    uint8_t tx[2];
    tx[0] = reg & 0x3F;     // Bit 7 = 0 = Write, Bit 6 = 0 = kein Multi
    tx[1] = value;

    ADXL_CS_LOW();
    HAL_Delay(1);           // WICHTIG: kleiner Delay vor dem Transfer
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    HAL_Delay(1);           // WICHTIG: kleiner Delay nach dem Transfer
    ADXL_CS_HIGH();
}
void ADXL345_Init(void)
{
    // ±8g: write 0x02 to DATA_FORMAT (0x31)
    ADXL345_WriteRegister(0x31, 0x02);

    // 1 kHz output data rate: write 0x0F to BW_RATE (0x2C)
    ADXL345_WriteRegister(0x2C, 0x0F);

    // Measurement mode: write 0x08 to POWER_CTL (0x2D)
    ADXL345_WriteRegister(0x2D, 0x08);
}
void ADXL345_ReadXYZ(int16_t* x, int16_t* y, int16_t* z)
{
    uint8_t tx[7] = { 0x80 | 0x40 | 0x32 }; // Read | Multi-byte | Start at 0x32
    uint8_t rx[7] = {0};

    ADXL_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 7, HAL_MAX_DELAY);
    ADXL_CS_HIGH();

    *x = (int16_t)((rx[2] << 8) | rx[1]);
    *y = (int16_t)((rx[4] << 8) | rx[3]);
    *z = (int16_t)((rx[6] << 8) | rx[5]);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2 && recording && sampleIndex < SAMPLE_COUNT)
    {
        ADXL345_ReadXYZ(&accelBuffer[sampleIndex].x,
                        &accelBuffer[sampleIndex].y,
                        &accelBuffer[sampleIndex].z);
        sampleIndex++;

        if (sampleIndex >= SAMPLE_COUNT)
        {
            recording = 0; // fertig
        }
    }
}

void send_data_over_can(void)
{
    FDCAN_TxHeaderTypeDef txHeader = {0};
    txHeader.Identifier = CAN_ID_HEADER;
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = FDCAN_DLC_BYTES_8;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0;

    uint8_t txData[8];
    uint32_t totalBytes = SAMPLE_COUNT * sizeof(AccelSample); // 1000 × 6 = 6000
    uint16_t totalLen = totalBytes + 1; // +1 CRC

    //  1. Header senden (2 Byte Länge)
    txData[0] = (uint8_t)(totalLen >> 8);
    txData[1] = (uint8_t)(totalLen & 0xFF);
    memset(&txData[2], 0xFF, 6); // Rest füllen
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData);
    HAL_Delay(1); // Optional: Bus entlasten

    // 2. Payload vorbereiten
    uint8_t rawData[totalLen];
    memcpy(rawData, (uint8_t*)accelBuffer, totalBytes);

    // CRC8 berechnen
    uint8_t crc = 0;
    for (uint16_t i = 0; i < totalBytes; i++)
    {
        crc ^= rawData[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    rawData[totalBytes] = crc;
    // 🔎 Debug-Ausgabe
    printf("➡️  CRC berechnet: 0x%02X (vor dem Senden)\r\n", crc);
    printf("📦 Gesamtlänge (inkl. CRC): %u Bytes\n", totalLen);
    // 📤 3. In 8-Byte-Pakete aufteilen und senden
    for (uint16_t i = 0; i < totalLen; i += 8)
    {
        uint8_t len = (totalLen - i >= 8) ? 8 : (totalLen - i);
        memset(txData, 0xFF, 8);
        memcpy(txData, &rawData[i], len);

        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData);
        HAL_Delay(1); // Optional für Stabilität
    }

    printf("CAN-Übertragung abgeschlossen (%d Bytes)\r\n", totalLen);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
    {
        // 🔧 Diese beiden Variablen MÜSSEN hier oben deklariert sein
        FDCAN_RxHeaderTypeDef rxHeader;
        uint8_t rxData[8];

        // ✅ Jetzt dürfen wir sie verwenden
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) {
            printf("❌ Fehler beim Lesen der CAN-Nachricht\r\n");
            return;
        }

        if (rxHeader.Identifier == 0x123 && rxData[0] == 0xAB)
        {
            printf("📥 CAN-Anfrage empfangen (ID=0x123, Data=0xAB)\r\n");
            can_send_requested = 1;  // ⬅ außerhalb des IRQ senden!
        }
        else
        {
            printf("⚠️ Falsche Nachricht: ID=0x%03" PRIX32 ", Data=0x%02X\r\n", rxHeader.Identifier, rxData[0]);
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
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
      Error_Handler();
  }
  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_Delay(100); // optional: kurz warten nach Reset

  uint8_t who_am_i = ADXL345_ReadRegister(0x00); // DEVID-Register
  printf("ADXL345 WHO_AM_I: 0x%02X\r\n", who_am_i);
  ADXL345_Init();
  printf("ADXL345 init done\r\n");
  uint8_t power_ctl = ADXL345_ReadRegister(0x2D);
  printf("POWER_CTL: 0x%02X\r\n", power_ctl);


  while (1)
  {
	    // Trigger prüfen
	    if (triggered == 0 && HAL_GPIO_ReadPin(Trigger_GPIO_Port, Trigger_Pin) == GPIO_PIN_SET)
	    {
	        printf(" Trigger erkannt\n");
	        triggered = 1;
	        startSampling = 1;
	    }

	    // Aufnahme starten
	    if (startSampling && recording == 0)
	    {
	        printf("Aufzeichnung startet...\r\n");
	        sampleIndex = 0;
	        recording = 1;
	        startSampling = 0;
	        BSP_LED_On(LED_GREEN);
	    }

	    // Ausgabe nach der Aufzeichnung
	    if (recording == 0 && sampleIndex == SAMPLE_COUNT)
	    {
	        printf(" Aufzeichnung abgeschlossen – sende Daten:\r\n");

	        for (int i = 0; i < SAMPLE_COUNT; i++)
	        {
	            printf("%d,%d,%d\r\n", accelBuffer[i].x, accelBuffer[i].y, accelBuffer[i].z);
	            HAL_Delay(1);  // optional: UART entlasten
	        }

	        sampleIndex = 0;
	        triggered = 0;
	        BSP_LED_Off(LED_GREEN);
	    }

	    // 🔁 WICHTIG: CAN-Sendeanforderung auswerten – IMMER prüfen!
	    if (can_send_requested)
	    {
	        can_send_requested = 0;
	        send_data_over_can();  // wird außerhalb vom IRQ sicher ausgeführt
	    }




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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 34;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 6;
  hfdcan1.Init.NominalTimeSeg2 = 3;
  hfdcan1.Init.DataPrescaler = 34;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
  FDCAN_FilterTypeDef sFilter;
  sFilter.IdType = FDCAN_STANDARD_ID;
  sFilter.FilterIndex = 0;
  sFilter.FilterType = FDCAN_FILTER_MASK;
  sFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilter.FilterID1 = 0x123;
  sFilter.FilterID2 = 0x7FF; // Alle Bits vergleichen
  HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilter);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  /* USER CODE END FDCAN1_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 169;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS_PIN_GPIO_Port, SPI_CS_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SPI_CS_PIN_Pin */
  GPIO_InitStruct.Pin = SPI_CS_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_CS_PIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Trigger_Pin */
  GPIO_InitStruct.Pin = Trigger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Trigger_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* --- FDCAN1 TX / RX ------------------------------------------------------- */
  GPIO_InitStruct.Pin       = GPIO_PIN_12 | GPIO_PIN_11;   // PA12 = TX, PA11 = RX
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;             // *** WICHTIG ***
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
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
