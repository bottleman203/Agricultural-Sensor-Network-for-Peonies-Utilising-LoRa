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
#include "subghz.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>
#include "radio_driver.h"
#include "stm32wlxx_nucleo.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//typedef enum
//{
//  STATE_NULL,
//  STATE_MASTER,
//  STATE_SLAVE
//} state_t;
//
//typedef enum
//{
//  SSTATE_NULL,
//  SSTATE_RX,
//  SSTATE_TX
//} substate_t;
//
//typedef struct
//{
//  state_t state;
//  substate_t subState;
//  uint32_t rxTimeout;
//  uint32_t rxMargin;
//  uint32_t randomDelay;
//  char rxBuffer[RX_BUFFER_SIZE];
//  uint8_t rxSize;
//} pingPongFSM_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define RF_FREQUENCY                                433000000 //868000000 /* Hz */
#define TX_OUTPUT_POWER                             14        /* dBm */
#define LORA_BANDWIDTH                              0         /* Hz */
#define LORA_SPREADING_FACTOR                       7
#define LORA_CODINGRATE                             1
#define LORA_PREAMBLE_LENGTH                        8         /* Same for Tx and Rx */
#define LORA_SYMBOL_TIMEOUT                         5         /* Symbols */

#define UART_BUFF_SIZE 256
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// void (*volatile eventReceptor)(pingPongFSM_t *const fsm);

PacketParams_t packetParams;
const RadioLoRaBandwidths_t Bandwidths[] = {LORA_BW_125, LORA_BW_250, LORA_BW_500};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void radioInit(void);
void handleRxDone(void);
void handleRxTimeout(void);
void handleRxError(void);
void RadioOnDioIrq(RadioIrqMasks_t radioIrq);

void LEDstartup(void);

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
	int8_t rxBuffer[256] = {0}; // Buffer for received data

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /*** GPIO Configuration (for debugging) ***/
  /* DEBUG_SUBGHZSPI_NSSOUT = PA4
   * DEBUG_SUBGHZSPI_SCKOUT = PA5
   * DEBUG_SUBGHZSPI_MISOOUT = PA6
   * DEBUG_SUBGHZSPI_MOSIOUT = PA7
   * DEBUG_RF_HSE32RDY = PA10
   * DEBUG_RF_NRESET = PA11
   * DEBUG_RF_SMPSRDY = PB2
   * DEBUG_RF_DTB1 = PB3 <---- Conflicts with RF_IRQ0
   * DEBUG_RF_LDORDY = PB4
   * RF_BUSY = PA12
   * RF_IRQ0 = PB3
   * RF_IRQ1 = PB5
   * RF_IRQ2 = PB8
   */

//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//  // Enable GPIO Clocks
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//
//  // DEBUG_SUBGHZSPI_{NSSOUT, SCKOUT, MSIOOUT, MOSIOUT} pins
//  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
//  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//  GPIO_InitStruct.Alternate = GPIO_AF13_DEBUG_SUBGHZSPI;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//  // DEBUG_RF_{HSE32RDY, NRESET} pins
//  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
//  GPIO_InitStruct.Alternate = GPIO_AF13_DEBUG_RF;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//  // DEBUG_RF_{SMPSRDY, LDORDY} pins
//  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_4;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//  // RF_BUSY pin
//  GPIO_InitStruct.Pin = GPIO_PIN_12;
//  GPIO_InitStruct.Alternate = GPIO_AF6_RF_BUSY;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//  // RF_{IRQ0, IRQ1, IRQ2} pins
//  GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_8;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SUBGHZ_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_RED);
  BSP_LED_Init(LED_BLUE);

  strcpy(uartBuff, "\n\rPING PONG Receiver\r\n---------------\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)uartBuff, strlen(uartBuff), HAL_MAX_DELAY);
//  sprintf(uartBuff, "LORA_MODULATION\r\nLORA_BW=%d Hz\r\nLORA_SF=%d\r\n", (1 << LORA_BANDWIDTH) * 125, LORA_SPREADING_FACTOR);
//  HAL_UART_Transmit(&huart2, (uint8_t *)uartBuff, strlen(uartBuff), HAL_MAX_DELAY);
  radioInit();
  printFlowerArt();
  LEDstartup();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
	  SUBGRF_SetRx(3000 << 6);  // Set RX timeout (3000 ms)

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
              handleRxDone();  // Handle the received data
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
          HAL_Delay(1000);
      }

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
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

void LEDstartup(void)
{
    // Flash each LED 3 times
    for (int i = 0; i < 3; i++)
    {
        // Toggle the Green LED
        BSP_LED_Toggle(LED_GREEN);
        BSP_LED_Toggle(LED_RED);
        BSP_LED_Toggle(LED_BLUE);
        HAL_Delay(200); // Delay 200ms

        // Toggle the  LED again to turn it off
        BSP_LED_Toggle(LED_GREEN);
        BSP_LED_Toggle(LED_RED);
        BSP_LED_Toggle(LED_BLUE);
        HAL_Delay(200);

    }
}


void radioInit(void)
{
    SUBGRF_Init(RadioOnDioIrq);

    SUBGRF_WriteRegister(SUBGHZ_SMPSC0R, (SUBGRF_ReadRegister(SUBGHZ_SMPSC0R) | SMPS_CLK_DET_ENABLE));
    SUBGRF_SetRegulatorMode();

    SUBGRF_SetBufferBaseAddress(0x00, 0x00);
    SUBGRF_SetRfFrequency(RF_FREQUENCY);
    SUBGRF_SetRfTxPower(TX_OUTPUT_POWER);
    SUBGRF_SetStopRxTimerOnPreambleDetect(false);

    SUBGRF_SetPacketType(PACKET_TYPE_LORA);

    SUBGRF_WriteRegister(REG_LR_SYNCWORD, (LORA_MAC_PRIVATE_SYNCWORD >> 8) & 0xFF);
    SUBGRF_WriteRegister(REG_LR_SYNCWORD + 1, LORA_MAC_PRIVATE_SYNCWORD & 0xFF);

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

    SUBGRF_WriteRegister(0x0736, SUBGRF_ReadRegister(0x0736) | (1 << 2));
}

/**
  * @brief  Radio IRQ handler
  * @param  radioIrq Interrupt type (TX done, RX done, etc.)
  * @retval None
  */
void RadioOnDioIrq(RadioIrqMasks_t radioIrq)
{
    // Handles the radio interrupts
    if (radioIrq & IRQ_RX_DONE)
        handleRxDone();
    else if (radioIrq & IRQ_RX_TX_TIMEOUT)
        handleRxTimeout();
    else if (radioIrq & IRQ_CRC_ERROR)
        handleRxError();
}

//void handleRxDone(void)
//{
//    char uartBuff[100];
//    uint8_t rxBuffer[256] = {0};  // Buffer to hold received data
//    uint8_t rxSize = 0;
//    PacketStatus_t packetStatus;
//
//    // Apply workaround to reset certain timing registers after RX
//    SUBGRF_WriteRegister(0x0920, 0x00); // Specific workaround for RX timing sequence
//    SUBGRF_WriteRegister(0x0944, (SUBGRF_ReadRegister(0x0944) | 0x02)); // Another timing-related workaround
//
//    // Retrieve the payload from the radio buffer
//    SUBGRF_GetPayload(rxBuffer, &rxSize, sizeof(rxBuffer));
//
//    // Get the packet status
//    SUBGRF_GetPacketStatus(&packetStatus);
//
//    // Print the RSSI and SNR values
//    snprintf(uartBuff, sizeof(uartBuff), "Received Data: RSSI=%d dBm, SNR=%d\r\n", packetStatus.Params.LoRa.RssiPkt, packetStatus.Params.LoRa.SnrPkt);
//    HAL_UART_Transmit(&huart2, (uint8_t *)uartBuff, strlen(uartBuff), HAL_MAX_DELAY);
//
//    // Print the received data in hexadecimal format for better diagnostics
//    HAL_UART_Transmit(&huart2, (uint8_t *)"Data: ", 6, HAL_MAX_DELAY);
//    for (int i = 0; i < rxSize; i++)
//    {
//        snprintf(uartBuff, sizeof(uartBuff), "%02X ", rxBuffer[i]);
//        HAL_UART_Transmit(&huart2, (uint8_t *)uartBuff, strlen(uartBuff), HAL_MAX_DELAY);
//    }
//    HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);
//
//    // Clear interrupt status and re-enable interrupts
//    SUBGRF_ClearIrqStatus(IRQ_RX_DONE);
//
//    // Re-set RX mode explicitly after each handling
//    SUBGRF_SetRx(3000 << 6); // Ensure radio is back in RX mode
//}

void handleRxDone(void)
{
    char uartBuff[100];
    uint8_t rxBuffer[256] = {0};  // Buffer to hold received data
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

    // Re-set RX mode explicitly after each handling
    SUBGRF_SetRx(3000 << 6); // Ensure radio is back in RX mode
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
