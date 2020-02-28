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
static void MX_SPI_SLAVE_Init(uint8_t const* tx_buf, uint8_t const* rx_buf, uint32_t size);
static void MX_SPI_MASTER_Init(uint8_t const* tx_buf, uint8_t const* rx_buf, uint32_t size);
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
        LL_SPI_FULL_DUPLEX,            // TransferDirection
        mode,                          // Mode
        LL_SPI_DATAWIDTH_8BIT,         // DataWidth
        LL_SPI_POLARITY_HIGH,          // ClockPolarity
        LL_SPI_PHASE_2EDGE,            // ClockPhase
        LL_SPI_NSS_SOFT,               // NSS
        LL_SPI_BAUDRATEPRESCALER_DIV2, // BaudRate
        LL_SPI_MSB_FIRST,              // BitOrder
        LL_SPI_CRCCALCULATION_DISABLE, // CRCCalculation
        7,                             // CRCPoly
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
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
#define N 64
    uint8_t tx_slave[N] = "spi slave tx buffer";
    uint8_t rx_slave[N] = { 0 };
    uint8_t tx_master[N] = "spi master tx buffer";
    uint8_t rx_master[N] = { 0 };
    MX_SPI_SLAVE_Init(tx_slave, rx_slave, N);
    MX_SPI_MASTER_Init(tx_master, rx_master, N);
    /* USER CODE BEGIN 2 */
    LL_GPIO_InitTypeDef led = {
        LD1_Pin,                 // Pin
        LL_GPIO_MODE_OUTPUT,     // Mode
        LL_GPIO_SPEED_FREQ_LOW,  // Speed
        LL_GPIO_OUTPUT_PUSHPULL, // OutputType
        LL_GPIO_PULL_DOWN,       // Pull
        LL_GPIO_AF_0             // Alternate
    };
    LL_GPIO_Init(LD1_GPIO_Port, &led);
    led.Pin = LD2_Pin;
    LL_GPIO_Init(LD2_GPIO_Port, &led);
    led.Pin = LD3_Pin;
    LL_GPIO_Init(LD3_GPIO_Port, &led);

    /* USER CODE END 2 */

    LL_GPIO_ResetOutputPin(GPIOE, LL_GPIO_PIN_4); // Master NSS
    LL_mDelay(1);
    LL_DMA_EnableStream(DMA2, SPI_MASTER_RX_STREAM);
    LL_DMA_EnableStream(DMA2, SPI_MASTER_TX_STREAM);
    LL_mDelay(1);

    while (LL_DMA_IsEnabledStream(DMA2, SPI_MASTER_TX_STREAM))
        ;
    while (LL_DMA_IsEnabledStream(DMA2, SPI_SLAVE_TX_STREAM))
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
static void MX_SPI_SLAVE_Init(uint8_t const* tx_buf, uint8_t const* rx_buf, uint32_t size) {

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
    LL_GPIO_InitTypeDef sck = { 0 };
    sck.Pin = LL_GPIO_PIN_5;
    sck.Mode = LL_GPIO_MODE_ALTERNATE;
    sck.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    sck.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    sck.Pull = LL_GPIO_PULL_NO;
    sck.Alternate = LL_GPIO_AF_5;
    LL_GPIO_Init(GPIOA, &sck);

    LL_GPIO_InitTypeDef miso = { 0 };
    miso.Pin = LL_GPIO_PIN_6;
    miso.Mode = LL_GPIO_MODE_ALTERNATE;
    miso.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    miso.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    miso.Pull = LL_GPIO_PULL_NO;
    miso.Alternate = LL_GPIO_AF_5;
    LL_GPIO_Init(GPIOA, &miso);

    LL_GPIO_InitTypeDef mosi = { 0 };
    mosi.Pin = LL_GPIO_PIN_5;
    mosi.Mode = LL_GPIO_MODE_ALTERNATE;
    mosi.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    mosi.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    mosi.Pull = LL_GPIO_PULL_NO;
    mosi.Alternate = LL_GPIO_AF_5;
    LL_GPIO_Init(GPIOB, &mosi);

    /* NSS */
    LL_EXTI_InitTypeDef nss = { 0 };
    nss.Line_0_31 = LL_EXTI_LINE_4;
    nss.LineCommand = ENABLE;
    nss.Mode = LL_EXTI_MODE_IT;
    nss.Trigger = LL_EXTI_TRIGGER_FALLING;
    LL_EXTI_Init(&nss);
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE4);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_4, LL_GPIO_PULL_NO);

    NVIC_SetPriority(EXTI4_IRQn, 4);
    NVIC_EnableIRQ(EXTI4_IRQn);

    /* SPI_SLAVE DMA Init */

    /* SPI_SLAVE_RX Init */
    LL_DMA_InitTypeDef rx = {
        LL_SPI_DMA_GetRegAddr(SPI_SLAVE),  // PeriphOrM2MSrcAddress
        (uint32_t)rx_buf,                  // MemoryOrM2MDstAddress
        LL_DMA_DIRECTION_PERIPH_TO_MEMORY, // Direction
        LL_DMA_MODE_NORMAL,                // Mode
        LL_DMA_PERIPH_NOINCREMENT,         // PeriphOrM2MSrcIncMode
        LL_DMA_MEMORY_INCREMENT,           // MemoryOrM2MDstIncMode
        LL_DMA_PDATAALIGN_BYTE,            // PeriphOrM2MSrcDataSize
        LL_DMA_MDATAALIGN_BYTE,            // MemoryOrM2MDstDataSize
        size,                              // NbData
        SPI_SLAVE_RX_CHANNEL,              // Channel
        LL_DMA_PRIORITY_VERYHIGH,          // Priority
        LL_DMA_FIFOMODE_DISABLE,           // FIFOMode
        LL_DMA_FIFOTHRESHOLD_1_4,          // FIFOThreshold
        LL_DMA_MBURST_SINGLE,              // MemBurst
        LL_DMA_PBURST_SINGLE,              // PeriphBurst
    };
    LL_DMA_Init(DMA2, SPI_SLAVE_RX_STREAM, &rx);

    /* SPI_SLAVE_TX Init */
    LL_DMA_InitTypeDef tx = {
        (uint32_t)tx_buf,                  // PeriphOrM2MSrcAddress
        LL_SPI_DMA_GetRegAddr(SPI_SLAVE),  // MemoryOrM2MDstAddress
        LL_DMA_DIRECTION_MEMORY_TO_PERIPH, // Direction
        LL_DMA_MODE_NORMAL,                // Mode
        LL_DMA_PERIPH_NOINCREMENT,         // PeriphOrM2MSrcIncMode
        LL_DMA_MEMORY_INCREMENT,           // MemoryOrM2MDstIncMode
        LL_DMA_PDATAALIGN_BYTE,            // PeriphOrM2MSrcDataSize
        LL_DMA_MDATAALIGN_BYTE,            // MemoryOrM2MDstDataSize
        size,                              // NbData
        SPI_SLAVE_TX_CHANNEL,              // Channel
        LL_DMA_PRIORITY_VERYHIGH,          // Priority
        LL_DMA_FIFOMODE_DISABLE,           // FIFOMode
        LL_DMA_FIFOTHRESHOLD_1_4,          // FIFOThreshold
        LL_DMA_MBURST_SINGLE,              // MemBurst
        LL_DMA_PBURST_SINGLE,              // PeriphBurst
    };
    LL_DMA_Init(DMA2, SPI_SLAVE_TX_STREAM, &tx);
}

/**
 * @brief SPI_MASTER Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI_MASTER_Init(uint8_t const* tx_buf, uint8_t const* rx_buf, uint32_t size) {
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
    LL_GPIO_InitTypeDef sck = { 0 };
    sck.Pin = LL_GPIO_PIN_2;
    sck.Mode = LL_GPIO_MODE_ALTERNATE;
    sck.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    sck.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    sck.Pull = LL_GPIO_PULL_NO;
    sck.Alternate = LL_GPIO_AF_5;
    LL_GPIO_Init(GPIOE, &sck);

    LL_GPIO_InitTypeDef miso = { 0 };
    miso.Pin = LL_GPIO_PIN_5;
    miso.Mode = LL_GPIO_MODE_ALTERNATE;
    miso.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    miso.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    miso.Pull = LL_GPIO_PULL_NO;
    miso.Alternate = LL_GPIO_AF_5;
    LL_GPIO_Init(GPIOE, &miso);

    LL_GPIO_InitTypeDef mosi = { 0 };
    mosi.Pin = LL_GPIO_PIN_6;
    mosi.Mode = LL_GPIO_MODE_ALTERNATE;
    mosi.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    mosi.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    mosi.Pull = LL_GPIO_PULL_NO;
    mosi.Alternate = LL_GPIO_AF_5;
    LL_GPIO_Init(GPIOE, &mosi);

    LL_GPIO_InitTypeDef nss = { 0 };
    nss.Pin = LL_GPIO_PIN_4;
    nss.Mode = LL_GPIO_MODE_OUTPUT;
    nss.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    nss.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    nss.Pull = LL_GPIO_PULL_UP;
    nss.Alternate = LL_GPIO_AF_0;
    LL_GPIO_Init(GPIOE, &nss);
    LL_GPIO_SetOutputPin(GPIOE, LL_GPIO_PIN_4);

    /* SPI_MASTER DMA Init */

    /* SPI_MASTER_RX Init */
    LL_DMA_InitTypeDef rx = {
        LL_SPI_DMA_GetRegAddr(SPI_MASTER), // PeriphOrM2MSrcAddress
        (uint32_t)rx_buf,                  // MemoryOrM2MDstAddress
        LL_DMA_DIRECTION_PERIPH_TO_MEMORY, // Direction
        LL_DMA_MODE_NORMAL,                // Mode
        LL_DMA_PERIPH_NOINCREMENT,         // PeriphOrM2MSrcIncMode
        LL_DMA_MEMORY_INCREMENT,           // MemoryOrM2MDstIncMode
        LL_DMA_PDATAALIGN_BYTE,            // PeriphOrM2MSrcDataSize
        LL_DMA_MDATAALIGN_BYTE,            // MemoryOrM2MDstDataSize
        size,                              // NbData
        SPI_MASTER_RX_CHANNEL,             // Channel
        LL_DMA_PRIORITY_VERYHIGH,          // Priority
        LL_DMA_FIFOMODE_DISABLE,           // FIFOMode
        LL_DMA_FIFOTHRESHOLD_1_4,          // FIFOThreshold
        LL_DMA_MBURST_SINGLE,              // MemBurst
        LL_DMA_PBURST_SINGLE,              // PeriphBurst
    };
    LL_DMA_Init(DMA2, SPI_MASTER_RX_STREAM, &rx);

    /* SPI_MASTER_TX Init */
    LL_DMA_InitTypeDef tx = {
        (uint32_t)tx_buf,                  // PeriphOrM2MSrcAddress
        LL_SPI_DMA_GetRegAddr(SPI_MASTER), // MemoryOrM2MDstAddress
        LL_DMA_DIRECTION_MEMORY_TO_PERIPH, // Direction
        LL_DMA_MODE_NORMAL,                // Mode
        LL_DMA_PERIPH_NOINCREMENT,         // PeriphOrM2MSrcIncMode
        LL_DMA_MEMORY_INCREMENT,           // MemoryOrM2MDstIncMode
        LL_DMA_PDATAALIGN_BYTE,            // PeriphOrM2MSrcDataSize
        LL_DMA_MDATAALIGN_BYTE,            // MemoryOrM2MDstDataSize
        size,                              // NbData
        SPI_MASTER_TX_CHANNEL,             // Channel
        LL_DMA_PRIORITY_VERYHIGH,          // Priority
        LL_DMA_FIFOMODE_DISABLE,           // FIFOMode
        LL_DMA_FIFOTHRESHOLD_1_4,          // FIFOThreshold
        LL_DMA_MBURST_SINGLE,              // MemBurst
        LL_DMA_PBURST_SINGLE,              // PeriphBurst
    };
    LL_DMA_Init(DMA2, SPI_MASTER_TX_STREAM, &tx);

    /* USER CODE BEGIN SPI_MASTER_Init 1 */

    /* USER CODE END SPI_MASTER_Init 1 */
    /* SPI_MASTER parameter configuration*/
    SPI_Init(SPI_MASTER, LL_SPI_MODE_MASTER);
    /* USER CODE END SPI_MASTER_Init 2 */
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
