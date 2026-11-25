/* =====================================================================
 * ROBOT KOL - STM32F407 tarafı: Pi'den UART ile komut alma + rampalı hareket
 *
 * Bu dosya tek başına derlenmez. Her bölümü main.c'deki aynı isimli
 * USER CODE bloğunun içine kopyala. Daha önce yazdığımız PCA9685 kodu
 * (PCA9685_Write, PCA9685_Init, Servo_SetAngle, SERVO_MIN/MAX) aynen kalıyor.
 *
 * Bağlantı: Pi GPIO14 (TX) -> PA3 (RX), Pi GPIO15 (RX) -> PA2 (TX), GND ortak
 * UART: 115200 8N1, USART2, RX DMA Circular (CubeMX'te zaten ayarlı)
 *
 * ---------------------------------------------------------------------
 * PROTOKOL (her satır '\n' ile biter, açılar tam sayı derece)
 *
 *   Pi -> STM32                      STM32 -> Pi
 *   P                                PONG              (bağlantı testi)
 *   M,taban,omuz,dirsek,kiskac       OK  ... sonra DONE
 *   M,taban,omuz,dirsek,kiskac,ms    OK  ... sonra DONE (süre ms ile)
 *   H                                OK  ... sonra DONE (90,90,90,0 konumu)
 *   S                                POS,a0,a1,a2,a3,MOVING|IDLE
 *   X                                STOPPED           (hareketi o anda dondur)
 *
 *   Hatalar: ERR,BUSY  ERR,RANGE,<kanal>  ERR,FORMAT  ERR,UNKNOWN  ERR,OVERFLOW
 * ===================================================================== */


/* ---------------------------------------------------------------------
 * USER CODE BEGIN Includes
 * ------------------------------------------------------------------- */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>


/* ---------------------------------------------------------------------
 * USER CODE BEGIN PD   (sabitler)
 * ------------------------------------------------------------------- */
#define RX_BUF_SIZE     128   /* DMA'nın yazdığı dairesel tampon */
#define LINE_MAX        48    /* bir komut satırının en fazla uzunluğu */
#define N_SERVO         4
#define TICK_MS         20    /* servo güncelleme periyodu = PWM periyodu */
#define VARSAYILAN_HIZ  60.0f /* derece/saniye, süre verilmezse */
#define MIN_SURE_MS     200
#define MAX_SURE_MS     10000


/* ---------------------------------------------------------------------
 * USER CODE BEGIN PV   (global değişkenler)
 *
 * Eski bulunan[] / adet / kanal / hedef değişkenlerini istersen silebilirsin;
 * artık Live Expressions yerine Pi'den komut göndereceğiz.
 * ------------------------------------------------------------------- */
static uint8_t rx_dma[RX_BUF_SIZE];   /* DMA buraya yazar */
static uint16_t rx_okunan = 0;        /* tamponda en son nereye kadar okuduk */

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


/* ---------------------------------------------------------------------
 * USER CODE BEGIN 0   (fonksiyonlar)
 *
 * Bu kısmı mevcut Servo_SetAngle fonksiyonunun ALTINA ekle.
 * Eski Servo_Move fonksiyonunu silebilirsin, yerini Hareket_* aldı.
 * ------------------------------------------------------------------- */

/* Pi'ye bir satır gönder ("\n" otomatik eklenir). */
static void Cevap(const char *s)
{
  char b[64];
  int n = snprintf(b, sizeof(b), "%s\n", s);
  HAL_UART_Transmit(&huart2, (uint8_t *)b, n, 50);
}

/* DMA tamponundan gelen baytları satıra ekler. Kesme içinde çalışır,
 * bu yüzden kısa tutuldu: sadece kopyalar, komutu ana döngü işler. */
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
    if (!komut_hazir) {                   /* önceki komut işlenmediyse yenisini atla */
      memcpy(komut, satir, satir_len + 1);
      komut_hazir = 1;
    }
    satir_len = 0;
    return;
  }
  if (satir_len < LINE_MAX - 1) satir[satir_len++] = (char)c;
  else satir_tasti = 1;
}

/* UART hattı bir an susunca (IDLE) ya da tampon yarı/tam dolunca HAL bunu çağırır.
 * Circular modda Size = DMA'nın tamponda şu an bulunduğu konum. */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance != USART2) return;

  if (Size != rx_okunan) {
    if (Size > rx_okunan) {
      for (uint16_t i = rx_okunan; i < Size; i++) Bayt_Isle(rx_dma[i]);
    } else {                               /* tampon başa sardı */
      for (uint16_t i = rx_okunan; i < RX_BUF_SIZE; i++) Bayt_Isle(rx_dma[i]);
      for (uint16_t i = 0; i < Size; i++) Bayt_Isle(rx_dma[i]);
    }
    rx_okunan = (Size == RX_BUF_SIZE) ? 0 : Size;
  }
}

/* Gürültü vb. bir UART hatasında alımı yeniden başlat. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART2) return;
  rx_okunan = 0;
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_dma, RX_BUF_SIZE);
}

/* Yumuşak başla-yumuşak dur eğrisi: 0..1 -> 0..1 (smoothstep).
 * Hareket başında ve sonunda hız sıfır olduğu için kol sarsılmaz. */
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

/* "M,90,110,75,30" ya da "M,90,110,75,30,800" satırını çözer.
 * Dönüş: okunan sayı adedi, format bozuksa -1. sscanf yerine strtol
 * kullandık; daha küçük kod ve float printf/scanf ayarı gerektirmiyor. */
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
  __disable_irq();                        /* kopyalarken kesme yazmasın */
  strcpy(k, komut);
  komut_hazir = 0;
  __enable_irq();

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
      /* Sınır dışı açıyı sessizce kırpmak yerine reddediyoruz:
       * Pi tarafındaki bir hata böylece hemen görünür olur. */
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


/* ---------------------------------------------------------------------
 * USER CODE BEGIN 2   (while döngüsünden önce, bir kez çalışır)
 *
 * Eski ortalama döngüsü ve tarama kodu yerine bunu koy.
 * PCA9685_Init() satırı burada kalmalı.
 * ------------------------------------------------------------------- */
#if 0  /* --- kopyalanacak kısım başlangıcı --- */
  PCA9685_Init();
  for (int i = 0; i < N_SERVO; i++) {     /* açılışta bilinen konuma git */
    Servo_SetAngle(i, aci_simdi[i]);
    HAL_Delay(200);                       /* hepsi birden akım çekmesin */
  }
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_dma, RX_BUF_SIZE);
  Cevap("READY");
#endif /* --- kopyalanacak kısım sonu --- */


/* ---------------------------------------------------------------------
 * USER CODE BEGIN 3   (while(1) içinde)
 *
 * Eski Servo_SetAngle(kanal, hedef) / HAL_Delay satırlarını SİL.
 * Döngüde HAL_Delay kullanma; Hareket_Guncelle zamanı kendisi tutuyor.
 * ------------------------------------------------------------------- */
#if 0  /* --- kopyalanacak kısım başlangıcı --- */
    Komut_Isle();
    Hareket_Guncelle();
#endif /* --- kopyalanacak kısım sonu --- */
