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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUF_SIZE     128   /* DMA'nın yazdığı dairesel tampon */
#define LINE_MAX        48    /* bir komut satırının en fazla uzunluğu */
#define N_SERVO         4
#define TICK_MS         20    /* servo güncelleme periyodu = PWM periyodu */
#define VARSAYILAN_HIZ  60.0f /* derece/saniye, süre verilmezse */
#define MIN_SURE_MS     200
#define MAX_SURE_MS     10000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
static uint8_t rx_dma[RX_BUF_SIZE];   /* DMA buraya yazar */
static uint16_t rx_okunan = 0;        /* tamponda en son nereye kadar okuduk */
static volatile uint8_t uart_hata = 0; /* UART hatası oldu, alım yeniden başlatılacak */
static volatile uint32_t uart_hata_sayisi = 0; /* D komutu gösterir */

static char satir[LINE_MAX];          /* birleştirilmekte olan satır */
static uint8_t satir_len = 0;
static uint8_t satir_tasti = 0;

static char komut[LINE_MAX];          /* ana döngünün işleyeceği hazır satır */
static volatile uint8_t komut_hazir = 0;

/* Hareket durumu. Live Expressions'a eklersen canlı izleyebilirsin. */
static float aci_simdi[N_SERVO] = {90, 90, 90, 0};
static float aci_bas[N_SERVO];
static float aci_hedef[N_SERVO];
static uint32_t hareket_bas_ms = 0;
static uint32_t hareket_sure_ms = 0;
static volatile uint8_t hareket_var = 0;
static uint32_t son_tick_ms = 0;

static volatile uint32_t i2c_hata = 0; /* başarısız PCA9685 yazma sayısı (D komutu gösterir) */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define PCA_ADDR (0x40 << 1)

/* Her servonun guvenli aci sinirlari (derece) */
/*                               taban omuz dirsek kiskac */
static const float SERVO_MIN[4] = {  5,   50,   55,    0 };
static const float SERVO_MAX[4] = {145,  145,  145,  145 };

void PCA9685_Write(uint8_t reg, uint8_t val) {
  if (HAL_I2C_Mem_Write(&hi2c1, PCA_ADDR, reg, 1, &val, 1, 10) != HAL_OK) i2c_hata++;
}

void PCA9685_Init(void) {
  PCA9685_Write(0x00, 0x10);
  PCA9685_Write(0xFE, 121);
  PCA9685_Write(0x00, 0x20);
  HAL_Delay(1);
}

void Servo_SetAngle(uint8_t ch, float derece) {
  if (ch > 3) return;                                   /* sadece 0-3 kanallari var */
  if (derece < SERVO_MIN[ch]) derece = SERVO_MIN[ch];   /* alt sinira kirp */
  if (derece > SERVO_MAX[ch]) derece = SERVO_MAX[ch];   /* ust sinira kirp */

  uint16_t us = 500 + derece * 2000.0f / 180.0f;
  uint16_t tick = us * 4096UL / 20000;
  uint8_t d[4] = {0, 0, tick & 0xFF, tick >> 8};
  if (HAL_I2C_Mem_Write(&hi2c1, PCA_ADDR, 0x06 + 4 * ch, 1, d, 4, 10) != HAL_OK) i2c_hata++;
}

/* ---- UART komut protokolü (Pi -> STM32), her satır '\n' ile biter ----
 *   P                        -> PONG
 *   M,taban,omuz,dirsek,kiskac[,ms] -> OK ... DONE
 *   H                        -> OK ... DONE   (90,90,90,0)
 *   S                        -> POS,a0,a1,a2,a3,MOVING|IDLE
 *   X                        -> STOPPED
 *   Hatalar: ERR,BUSY  ERR,RANGE,<kanal>  ERR,FORMAT  ERR,UNKNOWN  ERR,OVERFLOW
 */

/* Pi'ye bir satır gönder ("\n" otomatik eklenir). */
static void Cevap(const char *s)
{
  char b[128];
  int n = snprintf(b, sizeof(b), "%s\n", s);
  if (n >= (int)sizeof(b)) n = sizeof(b) - 1;
  HAL_UART_Transmit(&huart2, (uint8_t *)b, n, 50);
}

/* Gelen bir baytı satıra ekler; '\n' gelince satırı komut olarak hazırlar.
 * Ana döngüden çağrılır (kesmeden değil). */
static void Bayt_Isle(uint8_t c)
{
  if (c == '\r') return;
  if (c == '\n') {
    if (satir_tasti) {
      satir_tasti = 0;
      satir_len = 0;
      strcpy(komut, "!OVERFLOW");
      komut_hazir = 1;
      return;
    }
    if (satir_len == 0) return;           /* boş satırı yok say */
    satir[satir_len] = '\0';
    memcpy(komut, satir, satir_len + 1);
    komut_hazir = 1;
    satir_len = 0;
    return;
  }
  if (satir_len < LINE_MAX - 1) satir[satir_len++] = (char)c;
  else satir_tasti = 1;
}

/* Alımı sıfırdan başlatır: DMA tamponun başından yazar, biz de baştan okuruz. */
static void Uart_Alim_Baslat(void)
{
  HAL_UART_AbortReceive(&huart2);         /* ne durumda olursa olsun alımı temizle */
  if (HAL_UART_Receive_DMA(&huart2, rx_dma, RX_BUF_SIZE) == HAL_OK) {
    rx_okunan = 0;
    satir_len = 0;                        /* yarım kalmış satır çöp olabilir */
    satir_tasti = 0;
    uart_hata = 0;
  }
  /* olmazsa (DMA henüz durmamışsa) ana döngü bir sonraki turda tekrar dener */
}

/* Ana döngüden sürekli çağrılır. DMA'nın tamponda şu an nereye yazdığını
 * doğrudan donanımdan (NDTR) okur, aradaki yeni baytları işler.
 * Bir komut hazır olunca durur; kalan baytlar tamponda bekler, kaybolmaz. */
static void Uart_Oku(void)
{
  /* Hata olduysa ya da alım bir şekilde durduysa temiz başlat. DMA hâlâ
   * durdurulmaktaysa (HAL hata sırasında) bekle, bitince uart_hata gelir. */
  if (uart_hata || (huart2.RxState != HAL_UART_STATE_BUSY_RX &&
                    huart2.hdmarx->State == HAL_DMA_STATE_READY)) {
    Uart_Alim_Baslat();
    return;
  }
  uint16_t yaz = RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);
  if (yaz >= RX_BUF_SIZE) yaz = 0;
  while (rx_okunan != yaz && !komut_hazir) {
    Bayt_Isle(rx_dma[rx_okunan]);
    rx_okunan = (rx_okunan + 1) % RX_BUF_SIZE;
  }
}

/* Gürültü, çerçeve, taşma vb. UART hatası: HAL DMA alımını durdurur.
 * Burada (kesme içinde) sadece işaret koyarız, yeniden başlatmayı ana döngü yapar. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART2) return;
  uart_hata = 1;
  uart_hata_sayisi++;
}

/* Yumuşak başla-yumuşak dur eğrisi: 0..1 -> 0..1 (smoothstep). */
static float Yumusat(float s)
{
  if (s <= 0.0f) return 0.0f;
  if (s >= 1.0f) return 1.0f;
  return s * s * (3.0f - 2.0f * s);
}

/* Dört eklemi aynı anda başlatıp aynı anda bitiren hareketi kurar.
 * sure_ms = 0 ise en çok dönecek eklemin açısına göre süre hesaplanır. */
static void Hareket_Baslat(const float hedef[N_SERVO], uint32_t sure_ms)
{
  float en_buyuk = 0.0f;
  for (int i = 0; i < N_SERVO; i++) {
    aci_bas[i] = aci_simdi[i];
    aci_hedef[i] = hedef[i];
    float fark = fabsf(hedef[i] - aci_simdi[i]);
    if (fark > en_buyuk) en_buyuk = fark;
  }
  if (sure_ms == 0) sure_ms = (uint32_t)(en_buyuk / VARSAYILAN_HIZ * 1000.0f);
  if (sure_ms < MIN_SURE_MS) sure_ms = MIN_SURE_MS;
  if (sure_ms > MAX_SURE_MS) sure_ms = MAX_SURE_MS;

  hareket_sure_ms = sure_ms;
  hareket_bas_ms = HAL_GetTick();
  hareket_var = 1;
}

/* Ana döngüden sürekli çağrılır; her 20 ms'de bir servoları günceller. */
static void Hareket_Guncelle(void)
{
  uint32_t simdi = HAL_GetTick();
  if (simdi - son_tick_ms < TICK_MS) return;
  son_tick_ms = simdi;
  if (!hareket_var) return;

  float s = (float)(simdi - hareket_bas_ms) / (float)hareket_sure_ms;
  float k = Yumusat(s);
  for (int i = 0; i < N_SERVO; i++) {
    aci_simdi[i] = aci_bas[i] + (aci_hedef[i] - aci_bas[i]) * k;
    Servo_SetAngle(i, aci_simdi[i]);
  }
  if (s >= 1.0f) {
    hareket_var = 0;
    Cevap("DONE");
  }
}

/* ",90,110,75,30" ya da ",90,110,75,30,800" kısmını çözer.
 * Dönüş: okunan sayı adedi, format bozuksa -1. */
static int Sayilari_Oku(const char *p, long out[], int en_fazla)
{
  int n = 0;
  while (*p) {
    if (*p != ',') return -1;
    p++;
    char *son;
    long v = strtol(p, &son, 10);
    if (son == p) return -1;              /* virgülden sonra sayı yok */
    if (n >= en_fazla) return -1;
    out[n++] = v;
    p = son;
  }
  return n;
}

/* Ana döngüde, hazır bir komut varsa işler. */
static void Komut_Isle(void)
{
  if (!komut_hazir) return;
  char k[LINE_MAX];
  strcpy(k, komut);
  komut_hazir = 0;

  if (strcmp(k, "!OVERFLOW") == 0) { Cevap("ERR,OVERFLOW"); return; }

  switch (k[0]) {
  case 'P':
    Cevap("PONG");
    break;

  case 'S': {
    char b[64];
    snprintf(b, sizeof(b), "POS,%d,%d,%d,%d,%s",
             (int)(aci_simdi[0] + 0.5f), (int)(aci_simdi[1] + 0.5f),
             (int)(aci_simdi[2] + 0.5f), (int)(aci_simdi[3] + 0.5f),
             hareket_var ? "MOVING" : "IDLE");
    Cevap(b);
    break;
  }

  case 'D': {                             /* tanı: PCA9685 cevap veriyor mu, ayarları ne */
    uint8_t mode1 = 0xFF, pre = 0xFF;
    char b[120];
    const char *var = (HAL_I2C_IsDeviceReady(&hi2c1, PCA_ADDR, 2, 10) == HAL_OK) ? "OK" : "YOK";
    HAL_I2C_Mem_Read(&hi2c1, PCA_ADDR, 0x00, 1, &mode1, 1, 10);
    HAL_I2C_Mem_Read(&hi2c1, PCA_ADDR, 0xFE, 1, &pre, 1, 10);
    int n = snprintf(b, sizeof(b), "DIAG,PCA=%s,MODE1=0x%02X,PRE=%u,I2CERR=%lu,UARTERR=%lu,BUL=",
                     var, mode1, pre, (unsigned long)i2c_hata,
                     (unsigned long)uart_hata_sayisi);
    /* I2C taraması: cevap veren bütün adresler (en fazla 8 tane yazılır) */
    int bulunan = 0;
    for (uint8_t a = 1; a < 128; a++) {
      if (HAL_I2C_IsDeviceReady(&hi2c1, a << 1, 1, 2) == HAL_OK && bulunan < 8) {
        n += snprintf(b + n, sizeof(b) - n, "%s%02X", bulunan ? " " : "", a);
        bulunan++;
      }
    }
    if (bulunan == 0) snprintf(b + n, sizeof(b) - n, "HIC");
    Cevap(b);
    break;
  }

  case 'X':
    hareket_var = 0;                      /* olduğu yerde kalır */
    Cevap("STOPPED");
    break;

  case 'H': {
    if (hareket_var) { Cevap("ERR,BUSY"); break; }
    const float ev[N_SERVO] = {90, 90, 90, 0};
    Cevap("OK");
    Hareket_Baslat(ev, 0);
    break;
  }

  case 'M': {
    if (hareket_var) { Cevap("ERR,BUSY"); break; }
    long v[5];
    int n = Sayilari_Oku(&k[1], v, 5);
    if (n != 4 && n != 5) { Cevap("ERR,FORMAT"); break; }

    float hedef[N_SERVO];
    for (int i = 0; i < N_SERVO; i++) {
      /* Sınır dışı açıyı sessizce kırpmak yerine reddediyoruz */
      if (v[i] < SERVO_MIN[i] || v[i] > SERVO_MAX[i]) {
        char b[24];
        snprintf(b, sizeof(b), "ERR,RANGE,%d", i);
        Cevap(b);
        return;
      }
      hedef[i] = (float)v[i];
    }
    uint32_t sure = (n == 5 && v[4] > 0) ? (uint32_t)v[4] : 0;
    Cevap("OK");
    Hareket_Baslat(hedef, sure);
    break;
  }

  default:
    Cevap("ERR,UNKNOWN");
    break;
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(50);                          /* PCA9685 beslemesi otursun (eski I2C taraması bu
                                             beklemeyi farkında olmadan sağlıyordu) */
  PCA9685_Init();
  for (int i = 0; i < N_SERVO; i++) {     /* açılışta bilinen konuma git */
    Servo_SetAngle(i, aci_simdi[i]);
    HAL_Delay(200);                       /* hepsi birden akım çekmesin */
  }
  Uart_Alim_Baslat();
  Cevap("READY");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Uart_Oku();
    Komut_Isle();
    Hareket_Guncelle();
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
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
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

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
