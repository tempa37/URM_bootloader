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
#include "stm32f0xx.h" 
//#include "system_stm32f0xx.c"
//#include "stm32f0xx_ll_bus.h"  
#include "mbcrc.h"

#define APP_ADDR               0x08001000u  //Это адаптированный загрузчик от UMVH
#define END_APP_ADDR           0x08007BFFu


typedef void (*pAppEntry)(void);

#define buf_size_rx             100

#define FLASH_CFG_PAGE_ADDR    0x08007C00u //разрешаем запись только с этого адреса и далее

#define UPDATE_FLAG            0x08007C04u
#define UPDATE_FLAG2           0x08007C08u
#define FLASH_PAGE_SIZE        1024U

#define UPDATE_TIMEOUT_LOOPS 100000U    
#define ID                      0x01

#define HEADER_SIZE           6   // ID(1)+FUNC(1)+PKT_H(1)+PKT_L(1)+TOT_H(1)+TOT_L(1)
#define CRC_SIZE              2
#define FUNC_CODE_FIRMWARE    0x2A
#define MODBUS_ILLEGAL_FUNCTION    0x01
#define IS_IN_SAME_PAGE(addr, base) (((addr) & ~(FLASH_PAGE_SIZE-1u)) == (base)) //проверка, лежитат ли адреса на странице
#define CONFIG_PAGE_BASE   0x08007C00  // 0x08007C00 Адрес страницы с настройками калибровки и прочее

uint32_t tick = 0;
volatile uint16_t number = 1;

static uint16_t s_pageBuf[FLASH_PAGE_SIZE/2];  // 1 КБ = 512 halfword
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

uint8_t receive_buf[buf_size_rx] = {0};
unsigned char ID_1 = ID;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
void MY_UARTEx_RxEventCallback(uint16_t Size);
void JumpToApp(void);
void Update(void);
void SwitchToReceive(void);

static void flash_unlock(void);
static void flash_lock(void);
static uint8_t flash_program_halfword(uint32_t addr, uint16_t data);
static void send_ack(uint16_t packet);
void OS_usave_packet(uint16_t rx_len);
void ERROR_handler(uint8_t exception_code);
static void uart_tx(const uint8_t *buf, uint16_t len);
uint16_t FW_CalcCrc_ExcludeTail2(void);
static inline uint16_t swap_bytes16(uint16_t x);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void JumpToApp(void) {
    uint32_t *app_vec = (uint32_t *)APP_ADDR;
    // 1) Отключаем все прерывания
    __disable_irq();
    // 2) Устанавливаем MSP = первый элемент векторной таблицы приложения
    __set_MSP(app_vec[0]);


    // 3) сбросить USART1
    LL_USART_Disable(USART1);
    LL_APB1_GRP2_ForceReset(LL_APB1_GRP2_PERIPH_USART1);
    LL_APB1_GRP2_ReleaseReset(LL_APB1_GRP2_PERIPH_USART1);

    // 4) сбросить DMA‑канал (тот, что RX)
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_AHB1_GRP1_ForceReset(LL_AHB1_GRP1_PERIPH_DMA1);
    LL_AHB1_GRP1_ReleaseReset(LL_AHB1_GRP1_PERIPH_DMA1);

    // 5) отключить и сбросить NVIC
    NVIC_DisableIRQ(USART1_IRQn);
    NVIC_ClearPendingIRQ(USART1_IRQn);

    // 6) Получаем адрес Reset_Handler приложения
    pAppEntry entry = (pAppEntry)app_vec[1];
    // 7) Прыгаем в приложение
    entry();
    while (1) { /* не придём сюда */ }
}

static inline uint16_t swap_bytes16(uint16_t x)
{
    return (uint16_t)((x >> 8) | (x << 8));
}

static int Flash_ReadPageToBuf(uint32_t pageBase)
{
    // просто копируем байты страницы в RAM
    const uint16_t *src = (const uint16_t *)pageBase;
    for (uint32_t i = 0; i < FLASH_PAGE_SIZE/2; ++i) {
        s_pageBuf[i] = src[i];
    }
    return 0;
}

static int Flash_ErasePage(uint32_t pageBase)
{
    flash_unlock();
    FLASH->SR = FLASH_SR_EOP | FLASH_SR_WRPERR | FLASH_SR_PGERR;
    FLASH->CR |= FLASH_CR_PER;
    FLASH->AR  = pageBase;
    FLASH->CR |= FLASH_CR_STRT;
    while (FLASH->SR & FLASH_SR_BSY) {}
    FLASH->CR &= ~FLASH_CR_PER;
    if (FLASH->SR & (FLASH_SR_WRPERR | FLASH_SR_PGERR)) { flash_lock(); return -1; }
    FLASH->SR = FLASH_SR_EOP;
    flash_lock();
    return 0;
}

static int Flash_ProgramPageFromBuf(uint32_t pageBase)
{
    flash_unlock();
    for (uint32_t i = 0; i < FLASH_PAGE_SIZE/2; ++i) {
        uint32_t addr = pageBase + i*2;
        uint16_t hw   = s_pageBuf[i];
        if (hw == 0xFFFFu) continue;               // можно не писать стёртые полусловa
        if (flash_program_halfword(addr, hw)) {    // твоя функция
            flash_lock();
            return -1;
        }
    }
    flash_lock();
    return 0;
}

/* Сохранить страницу, изменив только два флага */
int Flash_PreservePageExceptOS(uint32_t flag1_addr, uint16_t flag1_val,
                               uint32_t flag2_addr, uint16_t flag2_val)
{
    const uint32_t pageBase = (flag1_addr & ~(FLASH_PAGE_SIZE-1u));
    if (!IS_IN_SAME_PAGE(flag2_addr, pageBase)) return -2; // флаги должны быть на одной странице

    __disable_irq();                     // чтобы никто не читал эту страницу во время операции

    // 1) копируем страницу в RAM
    Flash_ReadPageToBuf(pageBase);

    // 2) правим только флаги в копии
    s_pageBuf[(flag1_addr - pageBase)/2] = flag1_val;   // например 0xFFFF чтобы «стереть» флаг
    s_pageBuf[(flag2_addr - pageBase)/2] = flag2_val;   // или нужное служебное значение

    // 3) стираем страницу
    if (Flash_ErasePage(pageBase)) { __enable_irq(); return -3; }

    // 4) заливаем обратно всю страницу из буфера
    if (Flash_ProgramPageFromBuf(pageBase)) { __enable_irq(); return -4; }

    __enable_irq();
    return 0;
}



/* глобальная статическая переменная для адреса записи */
static uint32_t flash_write_addr = APP_ADDR;

/* вспомогательные функции для работы с флеш */
static inline void flash_unlock(void) {
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
    }
}

static inline void flash_lock(void) {
    FLASH->CR |= FLASH_CR_LOCK;
}

/* программирует в флеш half‑word, возвращает 0 при успехе */
static uint8_t flash_program_halfword(uint32_t addr, uint16_t data) {
    flash_unlock();
    FLASH->CR |= FLASH_CR_PG;
    *(__IO uint16_t*)addr = data;
    while (FLASH->SR & FLASH_SR_BSY);
    FLASH->CR &= ~FLASH_CR_PG;
    if (FLASH->SR & (FLASH_SR_WRPERR | FLASH_SR_PGERR)) return 1;
    FLASH->SR = FLASH_SR_EOP | FLASH_SR_WRPERR | FLASH_SR_PGERR;
    return 0;
}


static void uart_tx(const uint8_t *buf, uint16_t len) 
{
  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_12);
  for (uint16_t i=0; i<len; i++) {
    while (!LL_USART_IsActiveFlag_TXE(USART1));
    LL_USART_TransmitData8(USART1, buf[i]);
  }
  while (!LL_USART_IsActiveFlag_TC(USART1));
  LL_USART_ClearFlag_TC(USART1);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_12);
  SwitchToReceive();
}


static void send_ack(uint16_t packet) 
{
    uint8_t resp[6];
    resp[0] = ID_1;
    resp[1] = FUNC_CODE_FIRMWARE;
    resp[2] = (packet >> 8) & 0xFF;
    resp[3] = packet & 0xFF;
    uint16_t crc = mbcrc(resp, 4);
    resp[4] = (crc >> 8) & 0xFF;
    resp[5] =  crc        & 0xFF;
    
    
    uart_tx(resp, 6);
}


uint16_t FW_CalcCrc_ExcludeTail2(void)
{
    const uint32_t start = APP_ADDR;
    const uint32_t end_inclusive = END_APP_ADDR;

    uint32_t len = (end_inclusive - start + 1u);
    //uint32_t len = (end_inclusive - start);
    if (len < 2u) {
        return 0xFFFFu; // или 0, как удобнее обрабатывать ошибку
    }

    len -= 2u; // исключаем последние 2 байта (CRC)
    return mbcrc((unsigned char *)start, (int32_t)len);
}


void OS_usave_packet(uint16_t rx_len) 
{
    volatile uint16_t pkt_num   = (receive_buf[2] << 8) | receive_buf[3];
    uint16_t total_pkts= (receive_buf[4] << 8) | receive_buf[5];
    uint8_t *data      = &receive_buf[HEADER_SIZE];
    uint16_t data_len  = rx_len - HEADER_SIZE - CRC_SIZE;
    
    
    
    
    if(number == pkt_num)
    {
      uint16_t i;
      /* записываем payload пополам: два байта = half‑word */
      for (i = 0; (i+1) < data_len; i += 2) {
          uint16_t half = data[i] | (data[i+1] << 8);
          flash_program_halfword(flash_write_addr, half);
          flash_write_addr += 2; 
      }
      
      if (i < data_len) 
      {
          // остался один байт в data[i], а data[i+1] уже CRC;
          // зальём последний байт + 0xFF:
          // счетчик не увеличиваем, подразумеваем что это последний пакет
          uint16_t half = data[i] | (0xFF << 8);
          flash_program_halfword(flash_write_addr, half);
          //flash_write_addr += 1;
      }

      
      
      
      number++;
      /* отправляем подтверждение */
      send_ack(pkt_num);

      /* если это последний пакет — пишем флаг и прыгаем */
      if (pkt_num == total_pkts) {
          uint16_t temp = 0x0000;
          flash_program_halfword(UPDATE_FLAG2, temp);
          flash_lock();
          //JumpToApp();
      }
    }
    else
    {
      send_ack(0xFFFF);
      flash_program_halfword(UPDATE_FLAG, 0x1111u);
      flash_lock();
    }
}

void ERROR_handler(uint8_t exception_code)
{
    uint8_t resp[5];
    uint8_t func_code = (receive_buf[1] | 0x80);  // оригинальный код функции с битом ошибки

    resp[0] = ID_1;           // адрес ведомого
    resp[1] = func_code;      // код функции + 0x80
    resp[2] = exception_code; // код исключения

    // CRC по первым 3 байтам
    uint16_t crc = mbcrc(resp, 3);
    resp[3] = (crc >> 8) & 0xFF;
    resp[4] =  crc        & 0xFF;

    uart_tx(resp, 5);
}



void MY_UARTEx_RxEventCallback(uint16_t Size)
{  
    uint16_t rx_length = Size;
    
    tick = 0;
    
    uint16_t checksum = 0;
    checksum = mbcrc(receive_buf, (rx_length-2)); //make CRC data    
    
    if ((receive_buf[rx_length - 2] == (uint8_t)((checksum >> 8) & 0xFF)) && 
       (receive_buf[rx_length - 1] == (uint8_t)(checksum & 0xFF))) {
         
      if(receive_buf[0] == ID_1){

          switch (receive_buf[1])
          {

               case 0x2A:
                OS_usave_packet(rx_length);
                break;
                  
               default:                                                           //errors handler
                ERROR_handler(MODBUS_ILLEGAL_FUNCTION);                           //MODBUS ILLEGAL FUNCTION//
                break;
           }       
         }
        else{ 
          SwitchToReceive();
        }
    } else {
      SwitchToReceive();
    }
  }






/**
 * Функция стирает область памяти под приложение
 *
 * К сожалению LL функции стирания секторов 
 * на F0 недоступны, поэтому тут работа с регистрами
 *
 */
void Update(void) 
{
    uint32_t pageAddr;

    // 0) Приводим страницу с настройками/флагами к нужным значениям флагов,
    //    сохранив ВСЁ остальное на странице (настройки).
    //    Здесь ставим "ожидание прошивки": UPDATE_FLAG2 = 0x1111, UPDATE_FLAG = 0xFFFF
    Flash_PreservePageExceptOS(UPDATE_FLAG,  0xFFFFu,
                               UPDATE_FLAG2, 0x1111u);

    // 1) Стираем ТОЛЬКО страницы приложения, кроме страницы настроек
    flash_unlock();
    FLASH->SR = FLASH_SR_EOP | FLASH_SR_WRPERR | FLASH_SR_PGERR;

    for (pageAddr = APP_ADDR; pageAddr < END_APP_ADDR; pageAddr += FLASH_PAGE_SIZE)
    {
        if (pageAddr == CONFIG_PAGE_BASE)   // не трогаем 0x08007C00
            continue;

        FLASH->CR |=  FLASH_CR_PER;
        FLASH->AR  =  pageAddr;
        FLASH->CR |=  FLASH_CR_STRT;
        while (FLASH->SR & FLASH_SR_BSY) { }

        FLASH->CR &= ~FLASH_CR_PER;
        if (FLASH->SR & (FLASH_SR_WRPERR | FLASH_SR_PGERR))
            break;

        FLASH->SR = FLASH_SR_EOP;
    }
    flash_lock();

    // 2) Сбрасываем адрес записи под новую прошивку
    flash_write_addr = APP_ADDR;
    number = 1;
}
    
    
void delay(uint16_t time) {

    for (int i = 0; i < time; i++) {
        __NOP();
    }
    
}





void SwitchToReceive(void) {
  
  
    for (uint16_t i = 0; i < buf_size_rx; ++i) {
        receive_buf[i] = 0;
    }
  
    // Сброс флагов ошибки и IDLE
    LL_USART_ClearFlag_IDLE(USART1);
    LL_USART_ClearFlag_FE(USART1);
    LL_USART_ClearFlag_NE(USART1);
    LL_USART_ClearFlag_ORE(USART1);

    // Сбросить длину DMA до полного буфера и запустить приём
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, buf_size_rx);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
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
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 3);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  //SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  LL_USART_EnableIT_IDLE(USART1);
  
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)&USART1->RDR);
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)receive_buf);
  LL_USART_EnableDMAReq_RX(USART1);
  
  
  SwitchToReceive();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  

  
  
  while (1)
  {
    volatile uint16_t flag = *(volatile uint16_t *)UPDATE_FLAG;
    volatile uint16_t flag2 = *(volatile uint16_t *)UPDATE_FLAG2;
    
    
    volatile uint16_t crc_os = FW_CalcCrc_ExcludeTail2();
    
    crc_os = swap_bytes16(crc_os);
    
    volatile uint16_t OS = *(volatile uint16_t *)(END_APP_ADDR - 1);
    
    
    
    if(flag == 0x1111u)
    {
      Update();
    }
    else
    {
            if(crc_os == OS)
            {
              JumpToApp();
            }
            else
            {
              Update();
            }
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

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**USART1 GPIO Configuration
  PB6   ------> USART1_TX
  PB7   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  


  /* USART1 DMA Init */

  /* USART1_RX Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_BYTE);

  /* USART1_TX Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, 0);
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 56000;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_DisableIT_CTS(USART1);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Включаем тактирование порта B */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

    /* --- PB5: DE (RS-485 driver enable) --- */
    GPIO_InitStruct.Pin        = LL_GPIO_PIN_12;
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    /* Изначально в режиме приёма */
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_12);

    /* --- PB6: USART1_TX, PB7: USART1_RX (AF0) --- */
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate  = LL_GPIO_AF_0;

    /* PB6 = TX */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PB7 = RX */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
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
