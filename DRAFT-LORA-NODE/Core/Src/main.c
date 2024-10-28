/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "rtc.h"
#include "subghz.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>
//lora
#include "radio_driver.h"
#include "stm32wlxx_nucleo.h"
//sensors
#include "BME280_STM32.h"
// drivers for oled
#include "ssd1306.h"
#include "fonts.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
  STATE_NULL,
  STATE_MASTER,
  STATE_SLAVE
} state_t;

typedef enum
{
  SSTATE_NULL,
  SSTATE_RX,
  SSTATE_TX
} substate_t;

typedef struct
{
  state_t state;
  substate_t subState;
  uint32_t rxTimeout;
  uint32_t rxMargin;
  uint32_t randomDelay;
  char rxBuffer[RX_BUFFER_SIZE];
  uint8_t rxSize;
} pingPongFSM_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

//#define RF_FREQUENCY                                433000000 //868000000 /* Hz */
//#define TX_OUTPUT_POWER                             14        /* dBm */
//#define LORA_BANDWIDTH                              0         /* Hz */
//#define LORA_SPREADING_FACTOR                       12
//#define LORA_CODINGRATE                             1
//#define LORA_PREAMBLE_LENGTH                        8         /* Same for Tx and Rx */
//#define LORA_SYMBOL_TIMEOUT                         5         /* Symbols */
#define RF_FREQUENCY                                433000000  // Choose based on your region
#define TX_OUTPUT_POWER                             14         // dBm, adjust for your region (14 dBm in Europe)
#define LORA_BANDWIDTH                              0          // 0 = 125 kHz, 1 = 250 kHz, 2 = 500 kHz
#define LORA_SPREADING_FACTOR                       12         // SF12 for maximum range
#define LORA_CODINGRATE                             4          // CR 4/8 for maximum robustness
#define LORA_PREAMBLE_LENGTH                        8          // Preamble length
#define LORA_SYMBOL_TIMEOUT                         5          // Symbols
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define NID 002

#define TRUE 1
#define FALSE 0
// OLED
#define OLEDenable 1 //0 to turn it off
#define BUFFER_SIZE 50
char bufnum[BUFFER_SIZE];
// OLED END
// BME280
#define BMEenable 1
float Temperature, Pressure, Humidity;
// BME280 END
//SoilMoisture
float SM;
float wetS = 1750;//1151; // set per medium
float dryS = 3785; // set per medium
float mS, cS;
#define CALIBRATION 1
//SoilMoiature END
// DS18B20
float ST;
uint8_t Presence = 0;

#define DS18B20_PORT Onewire_GPIO_Port
#define DS18B20_PIN Onewire_Pin
// DS18B20 END
//lora
volatile uint8_t txCompleteFlag = 0;
void (*volatile eventReceptor)(pingPongFSM_t *const fsm);
PacketParams_t packetParams;  // TODO: this is lazy...
const RadioLoRaBandwidths_t Bandwidths[] = { LORA_BW_125, LORA_BW_250, LORA_BW_500 };
//lora END
char uniqueID[25];
char txData[256];
uint8_t rxBuffer[256];
//sleep
long int wakeUpCount = 0;
long int halfMinutes = 1; // number of 30 second increments you wan the device to sleep
//sleep END
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void radioInit(void);
void RadioOnDioIrq(RadioIrqMasks_t radioIrq);
void eventTxDone(pingPongFSM_t *const fsm);
void eventRxDone(pingPongFSM_t *const fsm);
void eventTxTimeout(pingPongFSM_t *const fsm);
void eventRxTimeout(pingPongFSM_t *const fsm);
void eventRxError(pingPongFSM_t *const fsm);
void enterMasterRx(pingPongFSM_t *const fsm);
void enterSlaveRx(pingPongFSM_t *const fsm);
void enterMasterTx(pingPongFSM_t *const fsm);
void enterSlaveTx(pingPongFSM_t *const fsm);
void transitionRxDone(pingPongFSM_t *const fsm);
//lora
void LEDstartup(void);
void receiveMODE(uint8_t *rxBuffer, uint8_t bufferSize, uint32_t timeout);
int checkRX(uint8_t *rxBuffer, uint8_t bufferSize);
void transmitData(char *data);
//sensors
uint8_t DS18B20_Start (void);
uint8_t DS18B20_Read (void);
void soilMOISTUREinit(void);
float soilMOISTURE(void);
float soilTEMP(void);
void printOLED(float SM, float ST, float AT, float AP, float AH);
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

//  pingPongFSM_t fsm;
  char uartBuff[100];

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Enable GPIO Clocks
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // DEBUG_SUBGHZSPI_{NSSOUT, SCKOUT, MSIOOUT, MOSIOUT} pins
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF13_DEBUG_SUBGHZSPI;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // DEBUG_RF_{HSE32RDY, NRESET} pins
  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
  GPIO_InitStruct.Alternate = GPIO_AF13_DEBUG_RF;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // DEBUG_RF_{SMPSRDY, LDORDY} pins
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_4;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // RF_BUSY pin
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Alternate = GPIO_AF6_RF_BUSY;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // RF_{IRQ0, IRQ1, IRQ2} pins
  GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_8;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SUBGHZ_Init();
  MX_USART2_UART_Init();
  MX_ADC_Init();
  MX_TIM2_Init();
  MX_I2C3_Init();
  MX_I2C2_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  SSD1306_Clear(); // clear screen of old info before operation

  HAL_TIM_Base_Start(&htim2); // start timer used for us delays

  if(BMEenable) BME280_Config(OSRS_2, OSRS_16, OSRS_1, MODE_NORMAL, T_SB_0p5, IIR_16);

  // Calibrate The ADC On Power-Up For Better Accuracy
  HAL_GPIO_WritePin(GPIOB, SM_Power_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_ADCEx_Calibration_Start(&hadc);
  HAL_GPIO_WritePin(GPIOB, SM_Power_Pin, GPIO_PIN_RESET);

  SSD1306_Init (); // initialise the display TODO : for debugging only
  soilMOISTUREinit();

  //BSP_LED_Init(LED_GREEN);//might interfere with measurements
  //BSP_LED_Init(LED_RED);
  //BSP_LED_Init(LED_BLUE);
  //BSP_LED_Init(LED_GREEN);

	/* Check and handle if the system was resumed from StandBy mode */
	if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB) == RESET)
	{
	  MX_RTC_Init();
	}
	else
	{
	  /* Clear Standby flag */
	  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

	  /* Enable access to RTC domain for following wake-up source configuration */
	  HAL_PWR_EnableBkUpAccess();
	  __HAL_RCC_RTCAPB_CLK_ENABLE();
	}

  DBGMCU->CR &= ~(DBGMCU_CR_DBG_SLEEP | DBGMCU_CR_DBG_STOP | DBGMCU_CR_DBG_STANDBY);

  //strcpy(uartBuff, "\nAngus is doing his best\n---------------\r\n");
  //HAL_UART_Transmit(&huart2, (uint8_t *)uartBuff, strlen(uartBuff), HAL_MAX_DELAY);
//  sprintf(uartBuff, "LORA_MODULATION\r\nLORA_BW=%d Hz\r\nLORA_SF=%d\r\n", (1 << LORA_BANDWIDTH) * 125, LORA_SPREADING_FACTOR);
//  HAL_UART_Transmit(&huart2, (uint8_t *)uartBuff, strlen(uartBuff), HAL_MAX_DELAY);
  radioInit();

  getUniqueID(uniqueID);
  transmitData((uint8_t *)uniqueID);

  printFlowerArt();
  LEDstartup();




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //take measurements
	  if (BMEenable){
		  BME280_WakeUP();
		  BME280_Measure();
	  }

	  SM = soilMOISTURE();
	  ST = soilTEMP();
	  if (OLEDenable) printOLED(SM, ST, Temperature, Pressure, Humidity);//TODO: for debugging

	  //TODO: put data into formatted string txData
	  formatSensorData(txData);

	  //HAL_Delay(5000);

	  //Transmit Data
	  BSP_LED_On(LED_RED);
	  transmitData((uint8_t *)txData);

	  __HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);  // Disable UART RX interrupt

//	  HAL_Delay(100);
		// Wait for LoRa transmission to complete before entering Stop 2 mode
	  waitForLoRaTxComplete();
	  BSP_LED_Off(LED_RED);
	  sleep(halfMinutes);
	  //HAL_Delay(2000);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.LSIDiv = RCC_LSI_DIV1;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void sleep(long int hMinutes)
{
	/* Deactivate RTC wake-up timer */
	HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

	/* Clear all related wakeup flags */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

	/* Re-enable wakeup source: configure the Wakeup timer */
	// if you do this too fast then things get garbled
	HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0xEA60, RTC_WAKEUPCLOCK_RTCCLK_DIV16, 0);
	BSP_LED_On(LED_GREEN);

	// Disable peripherals before sleep
	HAL_ADC_DeInit(&hadc);
	HAL_UART_DeInit(&huart2);
	HAL_I2C_DeInit(&hi2c2);
	HAL_I2C_DeInit(&hi2c3);

	// Disable unnecessary clocks before entering sleep mode
	    __HAL_RCC_USART2_CLK_DISABLE();
	    __HAL_RCC_ADC_CLK_DISABLE();
	    __HAL_RCC_I2C2_CLK_DISABLE();
	    __HAL_RCC_I2C3_CLK_DISABLE();
	    __HAL_RCC_TIM2_CLK_DISABLE();


	/* Enter Stop 2 Mode */
	HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
	// code will resume from here
	// Increment wake-up count and check if 1 minute (2 wake-ups) is reached
	    wakeUpCount++;

	    // If we have woken up twice (for a total of 60 seconds), reset count
	    if (wakeUpCount >= hMinutes)
	    {
	        wakeUpCount = 0;  // Reset counter
	        // Reinitialize clock and peripherals
	    }
	    else
	    {
	        // Enter Stop 2 Mode again for duration of period
	        sleep(hMinutes);
	    }


	    // Upon waking up, re-enable the clocks
		SystemClock_Config();
		__HAL_RCC_USART2_CLK_ENABLE();
		__HAL_RCC_ADC_CLK_ENABLE();
		__HAL_RCC_I2C2_CLK_ENABLE();
		__HAL_RCC_I2C3_CLK_ENABLE();
		__HAL_RCC_TIM2_CLK_ENABLE();

	BSP_LED_Off(LED_GREEN);



	//reinit some things
	SystemClock_Config();
	MX_TIM2_Init();
	HAL_TIM_Base_Start(&htim2); // Restart the timer if it is used for delays

	MX_USART2_UART_Init();
	MX_I2C2_Init();
	MX_I2C3_Init();
	MX_ADC_Init();

	MX_SUBGHZ_Init();
	radioInit(); // Custom radio initialization function


}

void handleRxTimeout(void)
{
    char uartBuff[50];
    // Apply workaround to reset certain timing registers after RX timeout
    SUBGRF_WriteRegister(0x0920, 0x00);
    SUBGRF_WriteRegister(0x0944, (SUBGRF_ReadRegister(0x0944) | 0x02));

    strcpy(uartBuff, "RX Timeout!\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)uartBuff, strlen(uartBuff), HAL_MAX_DELAY);

    // Clear interrupt status
    SUBGRF_ClearIrqStatus(IRQ_RX_TX_TIMEOUT);

    // Re-set RX mode explicitly after timeout handling
    SUBGRF_SetRx(3000 << 6); // Ensure radio is back in RX mode
}

void handleRxError(void)
{
    char uartBuff[50];
    // Apply workaround to reset certain timing registers after CRC error
    SUBGRF_WriteRegister(0x0920, 0x00);
    SUBGRF_WriteRegister(0x0944, (SUBGRF_ReadRegister(0x0944) | 0x02));

    strcpy(uartBuff, "RX Error: CRC Error Detected!\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)uartBuff, strlen(uartBuff), HAL_MAX_DELAY);

    // Clear interrupt status
    SUBGRF_ClearIrqStatus(IRQ_CRC_ERROR);

    // Re-set RX mode explicitly after error handling
    SUBGRF_SetRx(3000 << 6); // Ensure radio is back in RX mode
}


void handleRxDone(bool reset)
{
    char uartBuff[100];
//    uint8_t rxBuffer[256] = {0};  // Buffer to hold received data
    uint8_t rxSize = 0;
    PacketStatus_t packetStatus;

    // Apply workaround to reset certain timing registers after RX
    SUBGRF_WriteRegister(0x0920, 0x00); // Specific workaround for RX timing sequence
    SUBGRF_WriteRegister(0x0944, (SUBGRF_ReadRegister(0x0944) | 0x02)); // Another timing-related workaround

    // Retrieve the payload from the radio buffer
    SUBGRF_GetPayload((uint8_t *)rxBuffer, &rxSize, 0xFF);

    // Debug: Print the actual size of the received data
    snprintf(uartBuff, sizeof(uartBuff), "Payload Length: %d\r\n", rxSize);
    HAL_UART_Transmit(&huart2, (uint8_t *)uartBuff, strlen(uartBuff), HAL_MAX_DELAY);

    // Get the packet status
    SUBGRF_GetPacketStatus(&packetStatus);

    // Print the RSSI and SNR values
    // Display RSSI, SNR, and frequency error values
       sprintf(uartBuff, "Received Data: %s  \r\nRSSI=%d dBm \r\nSignal RSSI=%d dBm \r\nSNR=%d dB \r\nFreq Error=%lu Hz\r\n",
               rxBuffer,
       		   packetStatus.Params.LoRa.RssiPkt,
               packetStatus.Params.LoRa.SignalRssiPkt,
               packetStatus.Params.LoRa.SnrPkt,
               packetStatus.Params.LoRa.FreqError);
       HAL_UART_Transmit(&huart2, (uint8_t *)uartBuff, strlen(uartBuff), HAL_MAX_DELAY);


    // Debugging read directly from buffer registers
    for (int i = 0; i < rxSize; i++)
    {
        uint8_t byteRead = SUBGRF_ReadRegister(0x00 + i); // Assuming buffer starts at 0x00
        //snprintf(uartBuff, sizeof(uartBuff), "Byte %d: %02X\r\n", i, byteRead);
        //HAL_UART_Transmit(&huart2, (uint8_t *)uartBuff, strlen(uartBuff), HAL_MAX_DELAY);
    }

    // Print the received data in hexadecimal format for better diagnostics
    HAL_UART_Transmit(&huart2, (uint8_t *)"Data: ", 6, HAL_MAX_DELAY);
    for (int i = 0; i < rxSize; i++)
    {
        snprintf(uartBuff, sizeof(uartBuff), "%02X ", rxBuffer[i]);
        HAL_UART_Transmit(&huart2, (uint8_t *)uartBuff, strlen(uartBuff), HAL_MAX_DELAY);
    }
    HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);

    // Clear interrupt status and re-enable interrupts
    SUBGRF_ClearIrqStatus(IRQ_RX_DONE);

    if(reset){
    	// Re-set RX mode explicitly after each handling
    	SUBGRF_SetRx(3000 << 6); // Ensure radio is back in RX mode
    }

}

/**
  * @brief  Radio IRQ handler
  * @param  radioIrq Interrupt type (TX done, RX done, etc.)
  * @retval None
  */
//void RadioOnDioIrq(RadioIrqMasks_t radioIrq)
//{
//    // Handles the radio interrupts
//    if (radioIrq & IRQ_RX_DONE)
//        handleRxDone(TRUE);
//    else if (radioIrq & IRQ_RX_TX_TIMEOUT)
//        handleRxTimeout();
//    else if (radioIrq & IRQ_CRC_ERROR)
//        handleRxError();
//}

void receiveINIT(int timeout)
{
    // Set up radio to receive data
    SUBGRF_SetDioIrqParams(
        IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR, // Enable specific interrupts
        IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR,
        IRQ_RADIO_NONE,
        IRQ_RADIO_NONE
    );

    // Set the radio to RX mode with a timeout
	  SUBGRF_SetSwitch(RFO_LP, RFSWITCH_RX);  // Set RF switch to RX mode
	  packetParams.Params.LoRa.PayloadLength = sizeof(rxBuffer);  // Set expected payload length
	  SUBGRF_SetPacketParams(&packetParams);  // Apply packet parameters
	  SUBGRF_SetRx(timeout << 6);  // Set RX timeout (3000 ms)
}

void listenRX(void)
{
    // Wait for an event (IRQ handler will trigger the appropriate handler)
    while (1)
    {
        // Read the interrupt status from the radio
        uint16_t irqStatus = SUBGRF_GetIrqStatus();

        // Print the current IRQ status for debugging
//          char uartBuff[50];
//          snprintf(uartBuff, sizeof(uartBuff), "IRQ Status: 0x%04X\r\n", irqStatus);
//          HAL_UART_Transmit(&huart2, (uint8_t *)uartBuff, strlen(uartBuff), HAL_MAX_DELAY);

        // Check for specific interrupt events
        if (irqStatus & IRQ_RX_DONE)  // Check if RX is done
        {
            handleRxDone(TRUE);  // Handle the received data
            BSP_LED_Toggle(LED_BLUE);
            break;
        }
        else if (irqStatus & IRQ_RX_TX_TIMEOUT)  // Check for timeout
        {
            handleRxTimeout();  // Handle the timeout
            break;
        }
        else if (irqStatus & IRQ_CRC_ERROR)  // Check for CRC error
        {
            handleRxError();  // Handle the error
            break;
        }

        // Add a small delay to prevent flooding UART with messages
        HAL_Delay(5000);
    }
}

/**
  * @brief  Prints a flower art with a message to the UART.
  * @retval None
  */
void printFlowerArt(void)
{
    const char *flowerArt =
    "        _\r\n"
    "       (_)                   \r\n"
    "   /\\  | |  /\\        \r\n"
    "  /  \\_|_|_/  \\     \r\n"
    "  \\   `\"\"\"`   /     \r\n"
    "   \\_________/         \r\n"
    "       | |                   \r\n"
    "       | |                   \r\n"
    "       | |                   \r\n"
    "       | |                   \r\n"
    "      _|_|_                \r\n"
    "    /_/   \\_\\            \r\n\n"
    "Angus's Peony Monitor\r\n\n";

    // Transmit the art and message to UART
    HAL_UART_Transmit(&huart2, (uint8_t *)flowerArt, strlen(flowerArt), HAL_MAX_DELAY);
}

/**
 * @brief Fetches the STM32 board's unique ID and stores it in a string.
 * @param uniqueID A pointer to a character array where the unique ID will be stored.
 *                 Ensure the array is large enough to hold the formatted string (e.g., 25 bytes).
 */
void getUniqueID(char *uniqueID) {
    // Fetch the unique ID from the registers
    uint32_t id0 = HAL_GetUIDw0();  // First part of the unique ID
    uint32_t id1 = HAL_GetUIDw1();  // Second part of the unique ID
    uint32_t id2 = HAL_GetUIDw2();  // Third part of the unique ID

    // Format the unique ID into a readable string
    snprintf(uniqueID, 25, "%08X%08X%08X", id0, id1, id2);
}

/**
 * @brief Formats the sensor data into a string for transmission.
 * @param txData Pointer to the buffer where the formatted data will be stored.
 * @param SM Soil Moisture (format: xxx.xx).
 * @param ST Soil Temperature (format: xx.xx).
 * @param AT Air Temperature (format: xx.xx).
 * @param AP Air Pressure (format: xxxx).
 * @param AH Air Humidity (format: xx.xx).
 */
void formatSensorData(char *txData) {
    // Use snprintf to ensure the values are formatted as required.
    // %07.2f formats SM to xxx.xx, %05.2f formats ST, AT, AH to xx.xx, and %04.0f formats AP to xxxx
    sprintf(txData, "%f", SM);

    // Optional: Print the formatted data for debugging
    char uartBuff[256];
    snprintf((char *)txData, 256, "$%u:%06.2f:%05.2f:%05.2f:%04.0f:%05.2f",NID, SM, ST, Temperature, Pressure, Humidity);
    HAL_UART_Transmit(&huart2, (uint8_t *)uartBuff, strlen(uartBuff), HAL_MAX_DELAY);
}


void delay (uint16_t us)// us delay
{
	__HAL_TIM_SET_COUNTER(&htim2,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim2) < us);  // wait for the counter to reach the us input in the parameter
}

float soilTEMP(void)
{
	uint8_t Temp_byte1, Temp_byte2;
	int16_t TEMP;

	  Presence = DS18B20_Start ();
	  DS18B20_Write (0xCC);  // skip ROM
	  DS18B20_Write (0x44);  // convert t

	  Presence = DS18B20_Start ();
	  DS18B20_Write (0xCC);  // skip ROM
	  DS18B20_Write (0xBE);  // Read Scratch-pad

      Temp_byte1 = DS18B20_Read();
	  Temp_byte2 = DS18B20_Read();
	  TEMP = ((Temp_byte2<<8))|Temp_byte1;
	  return (float)TEMP/16.0;  // resolution is 0.0625

}

uint8_t DS18B20_Start (void)
{
	uint8_t Response = 0;
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);   // set the pin as output
	HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin low
	delay (480);   // delay according to datasheet

	Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);    // set the pin as input

	delay (80);    // delay according to datasheet
		if (!(HAL_GPIO_ReadPin (DS18B20_PORT, DS18B20_PIN))) Response = 1;    // if the pin is low i.e the presence pulse is detected
		else Response = -1;

		delay (400); // 480 us delay totally.

		return Response;

}

void DS18B20_Write (uint8_t data)
{
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);  // set as output

	for (int i=0; i<8; i++)
	{
		if ((data & (1<<i))!=0)  // if the bit is high
		{
			// write 1
			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);  // set as output
			HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin LOW
			delay (1);  // wait for 1 us

			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);  // set as input
			delay (50);  // wait for 60 us
		}

		else  // if the bit is low
		{
			// write 0
			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);
			HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin LOW
			delay (50);  // wait for 60 us

			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);
		}
	}
}


uint8_t DS18B20_Read(void)
{
    uint8_t value = 0;
    Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);

    for (int i = 0; i < 8; i++)
    {
        Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);
        HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin LOW
        delay(2);  // wait for 2 us
        Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);
        if (HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN))
        {
            value |= 1 << i;  // read = 1
        }
        delay(60);  // wait for 60 us
    }
    return value;
}


void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}



void soilMOISTUREinit(void)
{
	mS = (-100.0)/(dryS - wetS);
	cS = - mS*dryS;
}

void ClearBuffer(char *buffer, uint16_t size) {
    for (uint16_t i = 0; i < size; i++) {
        buffer[i] = 0;  // Set each element to 0
    }
}


float soilMOISTURE(void)
{
	float temp = 0.9;
	HAL_GPIO_WritePin(GPIOB, SM_Power_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	// Start ADC Conversion
	HAL_ADC_Start(&hadc);
	// Poll ADC1 Perihperal & TimeOut = 1mSec
	HAL_ADC_PollForConversion(&hadc, 1);
	// Read The ADC Conversion Result
	temp = (float)HAL_ADC_GetValue(&hadc);
	if(CALIBRATION == 0)
	{
		temp = mS*temp + cS;
	}
	HAL_GPIO_WritePin(GPIOB, SM_Power_Pin, GPIO_PIN_RESET);
	return temp;
}

void printOLED(float SM, float ST,  float AT, float AP, float AH)
{
	// TODO: for debugging only

	//HAL_Delay(100);

	SSD1306_GotoXY (5,5); // goto 10, 10
	SSD1306_Puts ("SM: ", &Font_7x10, 1); // print Hello
	sprintf (bufnum, "%.2f",SM);//move number into buffer
	SSD1306_Puts (bufnum, &Font_7x10, 1);
	SSD1306_GotoXY (5,5); // goto 10, 10
	SSD1306_Puts ("SM: ", &Font_7x10, 1); // print Hello
	sprintf (bufnum, "%.2f",SM);//move number into buffer
	SSD1306_Puts (bufnum, &Font_7x10, 1);

	SSD1306_GotoXY (5, 17);
	SSD1306_Puts ("ST: ", &Font_7x10, 1); // print Hello
	ClearBuffer(bufnum, BUFFER_SIZE);
	sprintf (bufnum, "%.2f",ST);//move number into buffer
	SSD1306_Puts (bufnum, &Font_7x10, 1);

	SSD1306_GotoXY (5, 29);
	SSD1306_Puts ("AT: ", &Font_7x10, 1); // print Hello
	ClearBuffer(bufnum, BUFFER_SIZE);
	sprintf (bufnum, "%.2f",AT);//move number into buffer
	SSD1306_Puts (bufnum, &Font_7x10, 1);

	SSD1306_GotoXY (5, 41);
	SSD1306_Puts ("AP: ", &Font_7x10, 1); // print Hello
	ClearBuffer(bufnum, BUFFER_SIZE);
	sprintf (bufnum, "%.2f",AP);//move number into buffer
	SSD1306_Puts (bufnum, &Font_7x10, 1);

	SSD1306_GotoXY (5, 53);
	SSD1306_Puts ("AH: ", &Font_7x10, 1); // print Hello
	ClearBuffer(bufnum, BUFFER_SIZE);
	sprintf (bufnum, "%.2f",AH);//move number into buffer
	SSD1306_Puts (bufnum, &Font_7x10, 1);

	ClearBuffer(bufnum, BUFFER_SIZE);
	SSD1306_UpdateScreen(); // update screen

}

// Function to wait for the LoRa transmission to complete
void waitForLoRaTxComplete()
{
    // Wait until the TX complete flag is set by the TX_DONE interrupt
    while (!txCompleteFlag)
    {
        HAL_Delay(100);  // Small delay to avoid busy-waiting
    }

    // Transmission complete, reset the flag
    txCompleteFlag = 0;
}

/**
  * @brief  Transmit data using Sub-GHz radio
  * @param  data Pointer to data to be transmitted
  * @retval None
  */
void transmitData(char *data)
{
	txCompleteFlag=0;
	// Print the data being transmitted to UART
	char uartBuff[256];  // Buffer for UART message
	snprintf(uartBuff, sizeof(uartBuff), "Transmitting Data: %s\r\n", data);
	HAL_UART_Transmit(&huart2, (uint8_t *)uartBuff, strlen(uartBuff), HAL_MAX_DELAY);
    // Configure IRQ for TX Done and TX Timeout
    SUBGRF_SetDioIrqParams( IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                            IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                            IRQ_RADIO_NONE,
                            IRQ_RADIO_NONE );

    // Set the radio to TX mode
    SUBGRF_SetSwitch(RFO_LP, RFSWITCH_TX);

    // Workaround before each packet transmission (if required by the chip)
    SUBGRF_WriteRegister(0x0889, (SUBGRF_ReadRegister(0x0889) | 0x04));

    // Set payload length
    packetParams.Params.LoRa.PayloadLength = strlen(data);
    SUBGRF_SetPacketParams(&packetParams);

    // Send the payload
    SUBGRF_SendPayload((uint8_t *)data, packetParams.Params.LoRa.PayloadLength, 0);

    // Print debug information (optional)
    HAL_UART_Transmit(&huart2, (uint8_t *)"Transmitting Data...\r\n", 22, HAL_MAX_DELAY);
}



/**
  * @brief  Receive data using Sub-GHz radio
  * @param  rxBuffer Pointer to buffer where received data will be stored
  * @param  timeout Time to wait for reception (in milliseconds)
  * @retval None
  */
/**
  * @brief  Receive data using Sub-GHz radio
  * @param  rxBuffer Pointer to buffer where received data will be stored
  * @param  bufferSize Size of the rxBuffer
  * @param  timeout Time to wait for reception (in milliseconds)
  * @retval int Returns 0 if data is received successfully, -1 if timeout or error occurs
  */
void receiveMODE(uint8_t *rxBuffer, uint8_t bufferSize, uint32_t timeout)
{
    // Configure IRQ for RX Done, Timeout, and CRC Error
    SUBGRF_SetDioIrqParams(IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR,
                           IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR,
                           IRQ_RADIO_NONE,
                           IRQ_RADIO_NONE);

    // Set the radio to RX mode
    SUBGRF_SetSwitch(RFO_LP, RFSWITCH_RX);

    // Set the expected packet length and timeout
    packetParams.Params.LoRa.PayloadLength = bufferSize;
    SUBGRF_SetPacketParams(&packetParams);
    SUBGRF_SetRx(timeout << 6);  // Timeout in milliseconds (<< 6 converts to LoRa timing units)

}


int checkRX(uint8_t *rxBuffer, uint8_t bufferSize)
{
    // Wait until an event occurs (RX Done, Timeout, or Error)
//    uint32_t startTime = HAL_GetTick();
//    while (1)
//    {
//        // Check the elapsed time to handle timeout manually
//        if ((HAL_GetTick() - startTime) > timeout)
//        {
//            // Timeout occurred
//            HAL_UART_Transmit(&huart2, (uint8_t *)"Receive Timeout!\r\n", 18, HAL_MAX_DELAY);
//            return -1;
//        }

        // Check if RX Done, TX Timeout, or CRC Error interrupts occurred
        RadioIrqMasks_t irqStatus = SUBGRF_GetIrqStatus();
        SUBGRF_ClearIrqStatus(irqStatus);  // Clear the interrupt flags

        if (irqStatus & IRQ_RX_DONE)
        {
            // Data received successfully
            uint8_t rxSize;
            SUBGRF_GetPayload(rxBuffer, &rxSize, bufferSize);
            char uartBuff[100];
            sprintf(uartBuff, "Data received: %s\r..\n", rxBuffer);
            HAL_UART_Transmit(&huart2, (uint8_t *)uartBuff, strlen(uartBuff), HAL_MAX_DELAY);

            BSP_LED_Toggle(LED_GREEN);
            irqStatus = 0;
            return 0;
        }
        else if (irqStatus & IRQ_RX_TX_TIMEOUT)
        {
            // Timeout occurred
            HAL_UART_Transmit(&huart2, (uint8_t *)"Receive Timeout!\r\n", 18, HAL_MAX_DELAY);

            irqStatus = 0;
            return -1;
        }
        else if (irqStatus & IRQ_CRC_ERROR)
        {
            // CRC error occurred
            HAL_UART_Transmit(&huart2, (uint8_t *)"CRC Error!\r\n", 12, HAL_MAX_DELAY);

            irqStatus = 0;
            return -1;
        }
//    }
}

void LEDstartup(void)
{
    // Flash each LED 3 times
    for (int i = 0; i < 3; i++)
    {
        // Toggle the Green LED
        //BSP_LED_Toggle(LED_GREEN);
        BSP_LED_Toggle(LED_RED);
        BSP_LED_Toggle(LED_BLUE);
        HAL_Delay(200); // Delay 200ms

        // Toggle the  LED again to turn it off
        //BSP_LED_Toggle(LED_GREEN);
        BSP_LED_Toggle(LED_RED);
        BSP_LED_Toggle(LED_BLUE);
        HAL_Delay(200);

    }
}

/**
  * @brief  Initialize the Sub-GHz radio and dependent hardware.
  * @retval None
  */
void radioInit(void)
{
  // Initialize the hardware (SPI bus, TCXO control, RF switch)
  SUBGRF_Init(RadioOnDioIrq);

  // Use DCDC converter if `DCDC_ENABLE` is defined in radio_conf.h
  // "By default, the SMPS clock detection is disabled and must be enabled before enabling the SMPS." (6.1 in RM0453)
  SUBGRF_WriteRegister(SUBGHZ_SMPSC0R, (SUBGRF_ReadRegister(SUBGHZ_SMPSC0R) | SMPS_CLK_DET_ENABLE));
  SUBGRF_SetRegulatorMode();

  // Use the whole 256-byte buffer for both TX and RX
  SUBGRF_SetBufferBaseAddress(0x00, 0x00);

  SUBGRF_SetRfFrequency(RF_FREQUENCY);
  SUBGRF_SetRfTxPower(TX_OUTPUT_POWER);
  SUBGRF_SetStopRxTimerOnPreambleDetect(false);

  SUBGRF_SetPacketType(PACKET_TYPE_LORA);

  SUBGRF_WriteRegister( REG_LR_SYNCWORD, ( LORA_MAC_PRIVATE_SYNCWORD >> 8 ) & 0xFF );
  SUBGRF_WriteRegister( REG_LR_SYNCWORD + 1, LORA_MAC_PRIVATE_SYNCWORD & 0xFF );

  ModulationParams_t modulationParams;
  modulationParams.PacketType = PACKET_TYPE_LORA;
  modulationParams.Params.LoRa.Bandwidth = Bandwidths[LORA_BANDWIDTH];
  modulationParams.Params.LoRa.CodingRate = (RadioLoRaCodingRates_t)LORA_CODINGRATE;
  modulationParams.Params.LoRa.LowDatarateOptimize = 0x00;
  modulationParams.Params.LoRa.SpreadingFactor = (RadioLoRaSpreadingFactors_t)LORA_SPREADING_FACTOR;
  SUBGRF_SetModulationParams(&modulationParams);

  packetParams.PacketType = PACKET_TYPE_LORA;
  packetParams.Params.LoRa.CrcMode = LORA_CRC_ON;
  packetParams.Params.LoRa.HeaderType = LORA_PACKET_VARIABLE_LENGTH;
  packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
  packetParams.Params.LoRa.PayloadLength = 0xFF;
  packetParams.Params.LoRa.PreambleLength = LORA_PREAMBLE_LENGTH;
  SUBGRF_SetPacketParams(&packetParams);

  //SUBGRF_SetLoRaSymbNumTimeout(LORA_SYMBOL_TIMEOUT);

  // WORKAROUND - Optimizing the Inverted IQ Operation, see DS_SX1261-2_V1.2 datasheet chapter 15.4
  // RegIqPolaritySetup @address 0x0736
  SUBGRF_WriteRegister( 0x0736, SUBGRF_ReadRegister( 0x0736 ) | ( 1 << 2 ) );
}


///**
//  * @brief  Receive data trough SUBGHZSPI peripheral
//  * @param  radioIrq  interrupt pending status information
//  * @retval None
//  */
//void RadioOnDioIrq(RadioIrqMasks_t radioIrq)
//{
//  switch (radioIrq)
//  {
//    case IRQ_TX_DONE:
//      eventReceptor = eventTxDone;
//      break;
//    case IRQ_RX_DONE:
//      eventReceptor = eventRxDone;
//      break;
//    case IRQ_RX_TX_TIMEOUT:
//      if (SUBGRF_GetOperatingMode() == MODE_TX)
//      {
//        eventReceptor = eventTxTimeout;
//      }
//      else if (SUBGRF_GetOperatingMode() == MODE_RX)
//      {
//        eventReceptor = eventRxTimeout;
//      }
//      break;
//    case IRQ_CRC_ERROR:
//      eventReceptor = eventRxError;
//      break;
//    default:
//      break;
//  }
//}
/**
  * @brief  Radio IRQ handler
  * @param  radioIrq Interrupt type (TX done, RX done, etc.)
  * @retval None
  */
void RadioOnDioIrq(RadioIrqMasks_t radioIrq)
{
    switch (radioIrq)
    {
        case IRQ_TX_DONE:
            HAL_UART_Transmit(&huart2, (uint8_t *)"TX Done\r\n", 9, HAL_MAX_DELAY);
            txCompleteFlag = 1;  // Set flag when transmission is complete
            break;
        case IRQ_RX_DONE:
        	handleRxDone(TRUE);
            eventReceptor = eventRxDone;
            break;
        case IRQ_RX_TX_TIMEOUT:
        	handleRxTimeout();
            eventReceptor = eventRxTimeout;
            break;
        case IRQ_CRC_ERROR:
        	handleRxError();
            eventReceptor = eventRxError;
            break;
        default:
            break;
    }
}

/**
  * @brief  Process the TX Done event
  * @param  fsm pointer to FSM context
  * @retval None
  */
void eventTxDone(pingPongFSM_t *const fsm)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)"Event TX Done\r\n", 15, HAL_MAX_DELAY);
  switch (fsm->state)
  {
    case STATE_MASTER:
      switch (fsm->subState)
      {
        case SSTATE_TX:
          enterMasterRx(fsm);
          fsm->subState = SSTATE_RX;
          break;
        default:
          break;
      }
      break;
    case STATE_SLAVE:
      switch (fsm->subState)
      {
        case SSTATE_TX:
          enterSlaveRx(fsm);
          fsm->subState = SSTATE_RX;
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}


/**
  * @brief  Process the RX Done event
  * @param  fsm pointer to FSM context
  * @retval None
  */
void eventRxDone(pingPongFSM_t *const fsm)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)"Event RX Done\r\n", 15, HAL_MAX_DELAY);
  switch(fsm->state)
  {
    case STATE_MASTER:
      switch (fsm->subState)
      {
        case SSTATE_RX:
          transitionRxDone(fsm);
          if (strncmp(fsm->rxBuffer, "PONG", 4) == 0)
          {
            BSP_LED_Off(LED_GREEN);
            BSP_LED_Toggle(LED_RED);
            enterMasterTx(fsm);
            fsm->subState = SSTATE_TX;
          }
          else if (strncmp(fsm->rxBuffer, "PING", 4) == 0)
          {
            enterSlaveRx(fsm);
            fsm->state = STATE_SLAVE;
          }
          else
          {
            enterMasterRx(fsm);
          }
          break;
        default:
          break;
      }
      break;
    case STATE_SLAVE:
      switch (fsm->subState)
      {
        case SSTATE_RX:
          transitionRxDone(fsm);
          if (strncmp(fsm->rxBuffer, "PING", 4) == 0)
          {
            BSP_LED_Off(LED_RED);
            BSP_LED_Toggle(LED_GREEN);
            enterSlaveTx(fsm);
            fsm->subState = SSTATE_TX;
          }
          else
          {
            enterMasterRx(fsm);
            fsm->state = STATE_MASTER;
          }
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}


/**
  * @brief  Process the TX Timeout event
  * @param  fsm pointer to FSM context
  * @retval None
  */
void eventTxTimeout(pingPongFSM_t *const fsm)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)"Event TX Timeout\r\n", 18, HAL_MAX_DELAY);
  switch (fsm->state)
  {
    case STATE_MASTER:
      switch (fsm->subState)
      {
        case SSTATE_TX:
          enterMasterRx(fsm);
          fsm->subState = SSTATE_RX;
          break;
        default:
          break;
      }
      break;
    case STATE_SLAVE:
      switch (fsm->subState)
      {
        case SSTATE_TX:
          enterSlaveRx(fsm);
          fsm->subState = SSTATE_RX;
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}


/**
  * @brief  Process the RX Timeout event
  * @param  fsm pointer to FSM context
  * @retval None
  */
void eventRxTimeout(pingPongFSM_t *const fsm)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)"Event RX Timeout\r\n", 18, HAL_MAX_DELAY);
  switch (fsm->state)
  {
    case STATE_MASTER:
      switch (fsm->subState)
      {
        case SSTATE_RX:
          HAL_Delay(fsm->randomDelay);
          enterMasterTx(fsm);
          fsm->subState = SSTATE_TX;
          break;
        default:
          break;
      }
      break;
    case STATE_SLAVE:
      switch (fsm->subState)
      {
        case SSTATE_RX:
          enterSlaveRx(fsm);
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}


/**
  * @brief  Process the RX Error event
  * @param  fsm pointer to FSM context
  * @retval None
  */
void eventRxError(pingPongFSM_t *const fsm)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)"Event Rx Error\r\n", 16, HAL_MAX_DELAY);
  switch (fsm->state)
  {
    case STATE_MASTER:
      switch (fsm->subState)
      {
        case SSTATE_RX:
          HAL_Delay(fsm->randomDelay);
          enterMasterTx(fsm);
          fsm->subState = SSTATE_TX;
          break;
        default:
          break;
      }
      break;
    case STATE_SLAVE:
      switch (fsm->subState)
      {
        case SSTATE_RX:
          enterSlaveRx(fsm);
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}


/**
  * @brief  Entry actions for the RX sub-state of the Master state
  * @param  fsm pointer to FSM context
  * @retval None
  */
void enterMasterRx(pingPongFSM_t *const fsm)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)"Master Rx start\r\n", 17, HAL_MAX_DELAY);
  SUBGRF_SetDioIrqParams( IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
                          IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
                          IRQ_RADIO_NONE,
                          IRQ_RADIO_NONE );
  SUBGRF_SetSwitch(RFO_LP, RFSWITCH_RX);
  packetParams.Params.LoRa.PayloadLength = 0xFF;
  SUBGRF_SetPacketParams(&packetParams);
  SUBGRF_SetRx(fsm->rxTimeout << 6);
}


/**
  * @brief  Entry actions for the RX sub-state of the Slave state
  * @param  fsm pointer to FSM context
  * @retval None
  */
void enterSlaveRx(pingPongFSM_t *const fsm)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)"Slave Rx start\r\n", 16, HAL_MAX_DELAY);
  SUBGRF_SetDioIrqParams( IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
                          IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
                          IRQ_RADIO_NONE,
                          IRQ_RADIO_NONE );
  SUBGRF_SetSwitch(RFO_LP, RFSWITCH_RX);
  packetParams.Params.LoRa.PayloadLength = 0xFF;
  SUBGRF_SetPacketParams(&packetParams);
  SUBGRF_SetRx(fsm->rxTimeout << 6);
}


/**
  * @brief  Entry actions for the TX sub-state of the Master state
  * @param  fsm pointer to FSM context
  * @retval None
  */
void enterMasterTx(pingPongFSM_t *const fsm)
{
  HAL_Delay(fsm->rxMargin);

  HAL_UART_Transmit(&huart2, (uint8_t *)"...PING\r\n", 9, HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart2, (uint8_t *)"Master Tx start\r\n", 17, HAL_MAX_DELAY);
  SUBGRF_SetDioIrqParams( IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                          IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                          IRQ_RADIO_NONE,
                          IRQ_RADIO_NONE );
  SUBGRF_SetSwitch(RFO_LP, RFSWITCH_TX);
  // Workaround 5.1 in DS.SX1261-2.W.APP (before each packet transmission)
  SUBGRF_WriteRegister(0x0889, (SUBGRF_ReadRegister(0x0889) | 0x04));
  packetParams.Params.LoRa.PayloadLength = 0x4;
  SUBGRF_SetPacketParams(&packetParams);
  SUBGRF_SendPayload((uint8_t *)"PING", 4, 0);
}


/**
  * @brief  Entry actions for the TX sub-state of the Slave state
  * @param  fsm pointer to FSM context
  * @retval None
  */
void enterSlaveTx(pingPongFSM_t *const fsm)
{
  HAL_Delay(fsm->rxMargin);

  HAL_UART_Transmit(&huart2, (uint8_t *)"...PONG\r\n", 9, HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart2, (uint8_t *)"Slave Tx start\r\n", 16, HAL_MAX_DELAY);
  SUBGRF_SetDioIrqParams( IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                          IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                          IRQ_RADIO_NONE,
                          IRQ_RADIO_NONE );
  SUBGRF_SetSwitch(RFO_LP, RFSWITCH_TX);
  // Workaround 5.1 in DS.SX1261-2.W.APP (before each packet transmission)
  SUBGRF_WriteRegister(0x0889, (SUBGRF_ReadRegister(0x0889) | 0x04));
  packetParams.Params.LoRa.PayloadLength = 0x4;
  SUBGRF_SetPacketParams(&packetParams);
  SUBGRF_SendPayload((uint8_t *)"PONG", 4, 0);
}


/**
  * @brief  Transition actions executed on every RX Done event (helper function)
  * @param  fsm pointer to FSM context
  * @retval None
  */
void transitionRxDone(pingPongFSM_t *const fsm)
{
  PacketStatus_t packetStatus;
  char uartBuff[50];

  // Workaround 15.3 in DS.SX1261-2.W.APP (because following RX w/ timeout sequence)
  SUBGRF_WriteRegister(0x0920, 0x00);
  SUBGRF_WriteRegister(0x0944, (SUBGRF_ReadRegister(0x0944) | 0x02));

  SUBGRF_GetPayload((uint8_t *)fsm->rxBuffer, &fsm->rxSize, 0xFF);
  SUBGRF_GetPacketStatus(&packetStatus);

  sprintf(uartBuff, "RssiValue=%d dBm, SnrValue=%d Hz\r\n", packetStatus.Params.LoRa.RssiPkt, packetStatus.Params.LoRa.SnrPkt);
  HAL_UART_Transmit(&huart2, (uint8_t *)uartBuff, strlen(uartBuff), HAL_MAX_DELAY);
}

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
