/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_SPI_SLAVE_Init(uint8_t const* tx, uint8_t const* rx, uint32_t size);
static void MX_SPI_MASTER_Init(uint8_t const* tx, uint8_t const* rx, uint32_t size);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
#define SPI_SLAVE SPI1
#define SPI_SLAVE_RX_STREAM LL_DMA_STREAM_2
#define SPI_SLAVE_RX_CHANNEL LL_DMA_CHANNEL_3
#define SPI_SLAVE_TX_STREAM LL_DMA_STREAM_3
#define SPI_SLAVE_TX_CHANNEL LL_DMA_CHANNEL_3
#define SPI_MASTER SPI4
#define SPI_MASTER_RX_STREAM LL_DMA_STREAM_0
#define SPI_MASTER_RX_CHANNEL LL_DMA_CHANNEL_4
#define SPI_MASTER_TX_STREAM LL_DMA_STREAM_1
#define SPI_MASTER_TX_CHANNEL LL_DMA_CHANNEL_4

/* USER CODE BEGIN 0 */
static void SPI_Init(SPI_TypeDef* spix, uint32_t mode) {
    LL_SPI_InitTypeDef opt = {
        .TransferDirection = LL_SPI_FULL_DUPLEX,
        .Mode = mode,
        .DataWidth = LL_SPI_DATAWIDTH_8BIT,
        .ClockPolarity = LL_SPI_POLARITY_HIGH,
        .ClockPhase = LL_SPI_PHASE_2EDGE,
        .NSS = LL_SPI_NSS_SOFT,
        .BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV16,
        .BitOrder = LL_SPI_MSB_FIRST,
        .CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE,
        .CRCPoly = 7,
    };
    LL_SPI_Init(spix, &opt);
    LL_SPI_SetStandard(spix, LL_SPI_PROTOCOL_MOTOROLA);
    LL_SPI_DisableNSSPulseMgt(spix);
    LL_SPI_Enable(spix);
    LL_SPI_EnableDMAReq_RX(spix);
    LL_SPI_EnableDMAReq_TX(spix);
}

/**
 * @brief This function handles EXTI line4 interrupt.
 */
void EXTI4_IRQHandler(void) {
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_4) != RESET) {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_4);
        LL_SPI_DeInit(SPI_SLAVE);
        SPI_Init(SPI_SLAVE, LL_SPI_MODE_SLAVE);
        LL_DMA_DisableStream(DMA2, SPI_SLAVE_RX_STREAM);
        LL_DMA_DisableStream(DMA2, SPI_SLAVE_TX_STREAM);
        LL_SPI_ClearFlag_OVR(SPI_SLAVE);
        LL_DMA_EnableStream(DMA2, SPI_SLAVE_RX_STREAM);
        LL_DMA_EnableStream(DMA2, SPI_SLAVE_TX_STREAM);
    }
}
static int matched(uint8_t const* a, uint8_t const* b, uint32_t size) {
    for (int i = 0; i < size; ++i) {
        if (a[i] != b[i]) {
            return 0;
        }
    }
    return 1;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);

    // NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    /* System interrupt init*/

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
#define N 42
    uint8_t tx_slave[N] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
        20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41 };
    uint8_t rx_slave[N] = { 0 };
    uint8_t tx_master[N] = { 0x40, 0x8b, 0x02, 0x05, 0x38, 0x90, 0x3f, 0x19, 0x51, 0x28, 0x90, 0xa8,
        0x15, 0x18, 0x7f, 0xed, 0x7a, 0xb1, 0x47, 0x61, 0xd4, 0x76, 0x0d, 0xe3, 0xea, 0x4f, 0xd7,
        0x57, 0x6b, 0xef, 0x27, 0x26, 0xbb, 0xf2, 0x28, 0x1b, 0x48, 0x4c, 0x98, 0x59, 0x29, 0xfb };
    uint8_t rx_master[N] = { 0 };
    MX_SPI_SLAVE_Init(tx_slave, rx_slave, N);
    MX_SPI_MASTER_Init(tx_master, rx_master, N);
    /* USER CODE BEGIN 2 */
    LL_GPIO_InitTypeDef led = {
        .Pin = LD1_Pin,
        .Mode = LL_GPIO_MODE_OUTPUT,
        .Speed = LL_GPIO_SPEED_FREQ_LOW,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
        .Pull = LL_GPIO_PULL_DOWN,
        .Alternate = LL_GPIO_AF_0,
    };
    LL_GPIO_Init(LD1_GPIO_Port, &led);
    led.Pin = LD2_Pin;
    LL_GPIO_Init(LD2_GPIO_Port, &led);
    led.Pin = LD3_Pin;
    LL_GPIO_Init(LD3_GPIO_Port, &led);

    /* USER CODE END 2 */

    LL_GPIO_ResetOutputPin(GPIOE, LL_GPIO_PIN_4); // Master NSS
    LL_DMA_EnableStream(DMA2, SPI_MASTER_RX_STREAM);
    LL_DMA_EnableStream(DMA2, SPI_MASTER_TX_STREAM);

    while (LL_DMA_IsEnabledStream(DMA2, SPI_MASTER_TX_STREAM) ||
           LL_DMA_IsEnabledStream(DMA2, SPI_MASTER_RX_STREAM) ||
           LL_DMA_IsEnabledStream(DMA2, SPI_SLAVE_TX_STREAM) ||
           LL_DMA_IsEnabledStream(DMA2, SPI_SLAVE_RX_STREAM))
        ;
    LL_GPIO_SetOutputPin(GPIOE, LL_GPIO_PIN_4); // Master NSS
    LL_GPIO_SetOutputPin(LD1_GPIO_Port, LD1_Pin);
    if (matched(rx_master, tx_slave, N)) {
        LL_GPIO_SetOutputPin(LD2_GPIO_Port, LD2_Pin);
    }
    if (matched(rx_slave, tx_master, N)) {
        LL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    }
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_7);

    if (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_7) {
        Error_Handler();
    }
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
    LL_PWR_EnableOverDriveMode();
    LL_RCC_HSE_Enable();

    /* Wait till HSE is ready */
    while (LL_RCC_HSE_IsReady() != 1) {}
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_4, 216, LL_RCC_PLLP_DIV_2);
    LL_RCC_PLL_Enable();

    /* Wait till PLL is ready */
    while (LL_RCC_PLL_IsReady() != 1) {}
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

    /* Wait till System clock is ready */
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {}
    LL_Init1msTick(216 * 1000 * 1000);
    LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
    LL_SetSystemCoreClock(216 * 1000 * 1000);
}

/**
 * @brief SPI_SLAVE Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI_SLAVE_Init(uint8_t const* tx, uint8_t const* rx, uint32_t size) {

    /* USER CODE BEGIN SPI_SLAVE_Init 0 */

    /* USER CODE END SPI_SLAVE_Init 0 */

    /* Peripheral clock enable */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    /**SPI_SLAVE GPIO Configuration
     PA5   ------> SPI_SLAVE_SCK
     PA4   ------> SPI_SLAVE_NSS
     PA6   ------> SPI_SLAVE_MISO
     PB5   ------> SPI_SLAVE_MOSI
     */
    LL_GPIO_InitTypeDef sck = {
        .Pin = LL_GPIO_PIN_5,
        .Mode = LL_GPIO_MODE_ALTERNATE,
        .Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
        .Pull = LL_GPIO_PULL_NO,
        .Alternate = LL_GPIO_AF_5,
    };
    LL_GPIO_Init(GPIOA, &sck);

    LL_GPIO_InitTypeDef miso = {
        .Pin = LL_GPIO_PIN_6,
        .Mode = LL_GPIO_MODE_ALTERNATE,
        .Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
        .Pull = LL_GPIO_PULL_NO,
        .Alternate = LL_GPIO_AF_5,
    };
    LL_GPIO_Init(GPIOA, &miso);

    LL_GPIO_InitTypeDef mosi = {
        .Pin = LL_GPIO_PIN_5,
        .Mode = LL_GPIO_MODE_ALTERNATE,
        .Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
        .Pull = LL_GPIO_PULL_NO,
        .Alternate = LL_GPIO_AF_5,
    };
    LL_GPIO_Init(GPIOB, &mosi);

    /* NSS */
    LL_EXTI_InitTypeDef nss = {
        .Line_0_31 = LL_EXTI_LINE_4,
        .LineCommand = ENABLE,
        .Mode = LL_EXTI_MODE_IT,
        .Trigger = LL_EXTI_TRIGGER_FALLING,
    };
    LL_EXTI_Init(&nss);
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE4);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_4, LL_GPIO_PULL_NO);

    NVIC_SetPriority(EXTI4_IRQn, 4);
    NVIC_EnableIRQ(EXTI4_IRQn);

    /* SPI_SLAVE DMA Init */

    /* SPI_SLAVE_RX Init */
    LL_DMA_SetChannelSelection(DMA2, SPI_SLAVE_RX_STREAM, SPI_SLAVE_RX_CHANNEL);
    LL_DMA_SetDataTransferDirection(DMA2, SPI_SLAVE_RX_STREAM, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetStreamPriorityLevel(DMA2, SPI_SLAVE_RX_STREAM, LL_DMA_PRIORITY_VERYHIGH);
    LL_DMA_SetMode(DMA2, SPI_SLAVE_RX_STREAM, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA2, SPI_SLAVE_RX_STREAM, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA2, SPI_SLAVE_RX_STREAM, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA2, SPI_SLAVE_RX_STREAM, LL_DMA_PDATAALIGN_HALFWORD);
    LL_DMA_SetMemorySize(DMA2, SPI_SLAVE_RX_STREAM, LL_DMA_MDATAALIGN_HALFWORD);
    LL_DMA_DisableFifoMode(DMA2, SPI_SLAVE_RX_STREAM);
    LL_DMA_ConfigAddresses(DMA2, SPI_SLAVE_RX_STREAM, LL_SPI_DMA_GetRegAddr(SPI_SLAVE),
        (uint32_t)rx, LL_DMA_GetDataTransferDirection(DMA2, SPI_SLAVE_RX_STREAM));
    LL_DMA_SetDataLength(DMA2, SPI_SLAVE_RX_STREAM, size / 2);

    /* SPI_SLAVE_TX Init */
    LL_DMA_SetChannelSelection(DMA2, SPI_SLAVE_TX_STREAM, SPI_SLAVE_TX_CHANNEL);
    LL_DMA_SetDataTransferDirection(DMA2, SPI_SLAVE_TX_STREAM, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetStreamPriorityLevel(DMA2, SPI_SLAVE_TX_STREAM, LL_DMA_PRIORITY_VERYHIGH);
    LL_DMA_SetMode(DMA2, SPI_SLAVE_TX_STREAM, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA2, SPI_SLAVE_TX_STREAM, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA2, SPI_SLAVE_TX_STREAM, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA2, SPI_SLAVE_TX_STREAM, LL_DMA_PDATAALIGN_HALFWORD);
    LL_DMA_SetMemorySize(DMA2, SPI_SLAVE_TX_STREAM, LL_DMA_MDATAALIGN_HALFWORD);
    LL_DMA_DisableFifoMode(DMA2, SPI_SLAVE_TX_STREAM);
    LL_DMA_ConfigAddresses(DMA2, SPI_SLAVE_TX_STREAM, (uint32_t)tx,
        LL_SPI_DMA_GetRegAddr(SPI_SLAVE),
        LL_DMA_GetDataTransferDirection(DMA2, SPI_SLAVE_TX_STREAM));
    LL_DMA_SetDataLength(DMA2, SPI_SLAVE_TX_STREAM, size / 2);
}

/**
 * @brief SPI_MASTER Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI_MASTER_Init(uint8_t const* tx, uint8_t const* rx, uint32_t size) {
    /* USER CODE BEGIN SPI_MASTER_Init 0 */

    /* USER CODE END SPI_MASTER_Init 0 */

    /* Peripheral clock enable */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI4);

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
    /**SPI_MASTER GPIO Configuration
     PE2   ------> SPI_MASTER_SCK
     PE4   ------> SPI_MASTER_NSS
     PE5   ------> SPI_MASTER_MISO
     PE6   ------> SPI_MASTER_MOSI
     */
    LL_GPIO_InitTypeDef sck = {
        .Pin = LL_GPIO_PIN_2,
        .Mode = LL_GPIO_MODE_ALTERNATE,
        .Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
        .Pull = LL_GPIO_PULL_NO,
        .Alternate = LL_GPIO_AF_5,
    };
    LL_GPIO_Init(GPIOE, &sck);

    LL_GPIO_InitTypeDef miso = {
        .Pin = LL_GPIO_PIN_5,
        .Mode = LL_GPIO_MODE_ALTERNATE,
        .Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
        .Pull = LL_GPIO_PULL_NO,
        .Alternate = LL_GPIO_AF_5,
    };
    LL_GPIO_Init(GPIOE, &miso);

    LL_GPIO_InitTypeDef mosi = {
        .Pin = LL_GPIO_PIN_6,
        .Mode = LL_GPIO_MODE_ALTERNATE,
        .Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
        .Pull = LL_GPIO_PULL_NO,
        .Alternate = LL_GPIO_AF_5,
    };
    LL_GPIO_Init(GPIOE, &mosi);

    LL_GPIO_InitTypeDef nss = {
        .Pin = LL_GPIO_PIN_4,
        .Mode = LL_GPIO_MODE_OUTPUT,
        .Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
        .Pull = LL_GPIO_PULL_UP,
        .Alternate = LL_GPIO_AF_0,
    };
    LL_GPIO_Init(GPIOE, &nss);
    LL_GPIO_SetOutputPin(GPIOE, LL_GPIO_PIN_4);

    /* SPI_MASTER DMA Init */

    /* SPI_MASTER_RX Init */
    LL_DMA_SetChannelSelection(DMA2, SPI_MASTER_RX_STREAM, SPI_MASTER_RX_CHANNEL);
    LL_DMA_SetDataTransferDirection(DMA2, SPI_MASTER_RX_STREAM, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetStreamPriorityLevel(DMA2, SPI_MASTER_RX_STREAM, LL_DMA_PRIORITY_VERYHIGH);
    LL_DMA_SetMode(DMA2, SPI_MASTER_RX_STREAM, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA2, SPI_MASTER_RX_STREAM, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA2, SPI_MASTER_RX_STREAM, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA2, SPI_MASTER_RX_STREAM, LL_DMA_PDATAALIGN_HALFWORD);
    LL_DMA_SetMemorySize(DMA2, SPI_MASTER_RX_STREAM, LL_DMA_MDATAALIGN_HALFWORD);
    LL_DMA_DisableFifoMode(DMA2, SPI_MASTER_RX_STREAM);
    LL_DMA_ConfigAddresses(DMA2, SPI_MASTER_RX_STREAM, LL_SPI_DMA_GetRegAddr(SPI_MASTER),
        (uint32_t)rx, LL_DMA_GetDataTransferDirection(DMA2, SPI_MASTER_RX_STREAM));
    LL_DMA_SetDataLength(DMA2, SPI_MASTER_RX_STREAM, size / 2);

    /* SPI_MASTER_TX Init */
    LL_DMA_SetChannelSelection(DMA2, SPI_MASTER_TX_STREAM, SPI_MASTER_TX_CHANNEL);
    LL_DMA_SetDataTransferDirection(DMA2, SPI_MASTER_TX_STREAM, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetStreamPriorityLevel(DMA2, SPI_MASTER_TX_STREAM, LL_DMA_PRIORITY_VERYHIGH);
    LL_DMA_SetMode(DMA2, SPI_MASTER_TX_STREAM, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA2, SPI_MASTER_TX_STREAM, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA2, SPI_MASTER_TX_STREAM, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA2, SPI_MASTER_TX_STREAM, LL_DMA_PDATAALIGN_HALFWORD);
    LL_DMA_SetMemorySize(DMA2, SPI_MASTER_TX_STREAM, LL_DMA_MDATAALIGN_HALFWORD);
    LL_DMA_DisableFifoMode(DMA2, SPI_MASTER_TX_STREAM);
    LL_DMA_ConfigAddresses(DMA2, SPI_MASTER_TX_STREAM, (uint32_t)tx,
        LL_SPI_DMA_GetRegAddr(SPI_MASTER),
        LL_DMA_GetDataTransferDirection(DMA2, SPI_MASTER_TX_STREAM));
    LL_DMA_SetDataLength(DMA2, SPI_MASTER_TX_STREAM, size / 2);

    SPI_Init(SPI_MASTER, LL_SPI_MODE_MASTER);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(uint8_t* file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
