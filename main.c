/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : SMART_FIRE_MASTER - STM32 me
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  STATE_OFF = 0, STATE_STABILIZING, STATE_OK, STATE_ALERT,
  STATE_ALARM, STATE_FAULT, STATE_CONTROL
} SysState_t;

typedef struct {
  int gas, flame, temp10, hum10;
  uint32_t lastTick;
  uint8_t  online;
} SlaveData_t;

typedef struct {
  UART_HandleTypeDef *huart;
  uint8_t  rxByte;
  char     line[64];
  uint8_t  idx;
  char     msg[64];
  volatile uint8_t ready;
} UartLine_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PASSWORD          "1234"
#define GAS_ALERT_TH      1500
#define GAS_ALARM_TH      2500
#define SLAVE_TIMEOUT_MS  3000UL
#define STABILIZE_MS      20000UL
#define DOOR_OPEN_MS      30000UL
#define CONTROL_MS        (15UL * 60UL * 1000UL)
#define MSG_SHOW_MS       2000UL

#define SERVO_CLOSE_US    500
#define SERVO_OPEN_US     1500

#define LCD_ADDR          (0x27 << 1)   /* thu 0x3F neu LCD khong hien */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
static const char *stateName[] = {"OFF","STABILIZING","OK","ALERT","ALARM","FAULT","CONTROL"};

static SysState_t  sysState = STATE_STABILIZING;
static uint32_t    stateTick = 0;
static uint8_t     systemOn = 1;

static SlaveData_t slave[2];
static UartLine_t  uS1  = {.huart = &huart1};
static UartLine_t  uS2  = {.huart = &huart2};
static UartLine_t  uEsp = {.huart = &huart3};

static const char keymap[4][4] = {
  {'1','2','3','A'}, {'4','5','6','B'}, {'7','8','9','C'}, {'*','0','#','D'}
};
static GPIO_TypeDef *rowPort[4] = {GPIOB, GPIOB, GPIOB, GPIOB};
static const uint16_t rowPin[4] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_12, GPIO_PIN_13};
static GPIO_TypeDef *colPort[4] = {GPIOB, GPIOB, GPIOB, GPIOA};
static const uint16_t colPin[4] = {GPIO_PIN_14, GPIO_PIN_15, GPIO_PIN_8, GPIO_PIN_8};
static char     pwBuf[9];
static uint8_t  pwLen = 0;
static uint8_t  wrongCount = 0;

static uint8_t  doorOpen = 0;
static uint8_t  doorByPassword = 0;
static uint32_t doorTick = 0;

static uint8_t  controlReq = 0;
static uint32_t controlTick = 0;

static char     lcdMsg[17] = "";
static uint32_t lcdMsgTick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
static void LCD_Init(void);
static void LCD_Line(uint8_t row, const char *s);
static void LCD_ShowMsg(const char *s);
static void LCD_Task(void);
static char Keypad_Scan(void);
static void Keypad_Task(void);
static void Servo_Set(uint8_t open);
static void Uart_StartRx(UartLine_t *u);
static void Uart_Task(void);
static void State_Update(void);
static void State_Enter(SysState_t s);
static void Send_ToSlaves(void);
static void Send_ToEsp(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  __HAL_TIM_SET_AUTORELOAD(&htim2, 19999);      /* 1 MHz / 20000 = 50 Hz */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  Servo_Set(0);

  LCD_Init();
  LCD_Line(0, "SMART FIRE SYS");
  LCD_Line(1, "Khoi dong...");

  Uart_StartRx(&uS1);
  Uart_StartRx(&uS2);
  Uart_StartRx(&uEsp);

  State_Enter(STATE_STABILIZING);

  uint32_t tKey = 0, tSM = 0, tLcd = 0, tSlave = 0, tEsp = 0;
  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    Uart_Task();

    if (now - tKey   >= 20)   { tKey = now;   Keypad_Task(); }
    if (now - tSM    >= 100)  { tSM = now;    State_Update(); }
    if (now - tLcd   >= 300)  { tLcd = now;   LCD_Task(); }
    if (now - tSlave >= 500)  { tSlave = now; Send_ToSlaves(); }
    if (now - tEsp   >= 2000) { tEsp = now;   Send_ToEsp(); }
  }
  /* USER CODE END 3 */
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
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
}

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim2);
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART2_UART_Init(void)
{
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
}

static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/* ======================= LCD 1602 I2C ======================= */
static void LCD_Send(uint8_t v, uint8_t rs)
{
  uint8_t hi = v & 0xF0, lo = (uint8_t)((v << 4) & 0xF0);
  uint8_t d[4] = { (uint8_t)(hi | 0x0C | rs), (uint8_t)(hi | 0x08 | rs),
                   (uint8_t)(lo | 0x0C | rs), (uint8_t)(lo | 0x08 | rs) };
  HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, d, 4, 10);
}

static void LCD_Init(void)
{
  HAL_Delay(50);
  LCD_Send(0x33, 0); HAL_Delay(5);
  LCD_Send(0x32, 0); HAL_Delay(1);
  LCD_Send(0x28, 0);
  LCD_Send(0x0C, 0);
  LCD_Send(0x06, 0);
  LCD_Send(0x01, 0); HAL_Delay(2);
}

static void LCD_Line(uint8_t row, const char *s)
{
  char buf[17];
  snprintf(buf, sizeof(buf), "%-16s", s);
  LCD_Send(row ? 0xC0 : 0x80, 0);
  for (uint8_t i = 0; i < 16; i++) LCD_Send((uint8_t)buf[i], 1);
}

static void LCD_ShowMsg(const char *s)
{
  strncpy(lcdMsg, s, 16); lcdMsg[16] = 0;
  lcdMsgTick = HAL_GetTick();
}

static void LCD_Task(void)
{
  char l0[17], l1[17];
  uint32_t now = HAL_GetTick();
  snprintf(l0, sizeof(l0), "ST:%s", stateName[sysState]);

  if (lcdMsg[0] && now - lcdMsgTick < MSG_SHOW_MS)
    snprintf(l1, sizeof(l1), "%s", lcdMsg);
  else if (pwLen)
  {
    char stars[9] = "********"; stars[pwLen] = 0;
    snprintf(l1, sizeof(l1), "MK:%s", stars);
  }
  else
  {
    lcdMsg[0] = 0;
    switch (sysState)
    {
      case STATE_OFF:
        snprintf(l1, sizeof(l1), "Bam A de bat"); break;
      case STATE_STABILIZING:
      {
        uint32_t el = now - stateTick;
        unsigned long rem = (el < STABILIZE_MS) ? (STABILIZE_MS - el) / 1000UL : 0;
        snprintf(l1, sizeof(l1), "Lam nong %lus", rem); break;
      }
      case STATE_ALARM:
        snprintf(l1, sizeof(l1), "!!! CHAY !!!"); break;
      case STATE_FAULT:
        snprintf(l1, sizeof(l1), "Loi cam bien"); break;
      case STATE_CONTROL:
      {
        uint32_t el = now - controlTick;
        unsigned long rem = (el < CONTROL_MS) ? (CONTROL_MS - el) / 60000UL + 1 : 0;
        snprintf(l1, sizeof(l1), "Cloud %lu phut", rem); break;
      }
      default:
        snprintf(l1, sizeof(l1), "G%d/%d %s", slave[0].gas, slave[1].gas,
                 doorOpen ? "MO" : "DONG"); break;
    }
  }
  LCD_Line(0, l0);
  LCD_Line(1, l1);
}

/* ============================ SERVO ============================ */
static void Servo_Set(uint8_t open)
{
  doorOpen = open;
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, open ? SERVO_OPEN_US : SERVO_CLOSE_US);
}

/* ============================ KEYPAD ============================ */
static char Keypad_Scan(void)
{
  char key = 0;
  for (uint8_t r = 0; r < 4 && !key; r++)
  {
    for (uint8_t i = 0; i < 4; i++) HAL_GPIO_WritePin(rowPort[i], rowPin[i], GPIO_PIN_SET);
    HAL_GPIO_WritePin(rowPort[r], rowPin[r], GPIO_PIN_RESET);
    for (volatile int d = 0; d < 50; d++);
    for (uint8_t c = 0; c < 4; c++)
      if (HAL_GPIO_ReadPin(colPort[c], colPin[c]) == GPIO_PIN_RESET) { key = keymap[r][c]; break; }
  }
  for (uint8_t i = 0; i < 4; i++) HAL_GPIO_WritePin(rowPort[i], rowPin[i], GPIO_PIN_SET);
  return key;
}

static void Keypad_Task(void)
{
  static char last = 0, stable = 0;
  static uint8_t cnt = 0;
  char k = Keypad_Scan();

  if (k == last) { if (cnt < 3) cnt++; } else { cnt = 0; last = k; }
  if (cnt != 2 || k == stable) { if (cnt >= 2) stable = k; return; }
  stable = k;
  if (!k) return;

  if (k == 'A')
  {
    systemOn = !systemOn;
    State_Enter(systemOn ? STATE_STABILIZING : STATE_OFF);
    pwLen = 0;
    return;
  }
  if (sysState == STATE_OFF) return;

  if (k == '*') { pwLen = 0; return; }
  if (k == '#')
  {
    pwBuf[pwLen] = 0;
    if (strcmp(pwBuf, PASSWORD) == 0)
    {
      wrongCount = 0;
      if (sysState == STATE_ALARM)
        LCD_ShowMsg("Dang bao chay!");
      else
      {
        Servo_Set(1);
        doorByPassword = 1;
        doorTick = HAL_GetTick();
        LCD_ShowMsg("Dung - Mo cua");
      }
    }
    else
    {
      wrongCount++;
      LCD_ShowMsg(wrongCount >= 3 ? "Sai 3 lan!" : "Sai mat khau");
    }
    pwLen = 0;
    return;
  }
  if (k >= '0' && k <= '9' && pwLen < 8) pwBuf[pwLen++] = k;
}

/* ============================ UART ============================ */
static void Uart_StartRx(UartLine_t *u)
{
  HAL_UART_Receive_IT(u->huart, &u->rxByte, 1);
}

static void Uart_Feed(UartLine_t *u)
{
  char c = (char)u->rxByte;
  if (c == '\n' || c == '\r')
  {
    if (u->idx && !u->ready)
    {
      u->line[u->idx] = 0;
      memcpy(u->msg, u->line, u->idx + 1);
      u->ready = 1;
    }
    u->idx = 0;
  }
  else if (u->idx < sizeof(u->line) - 1) u->line[u->idx++] = c;
  else u->idx = 0;
}

static UartLine_t *Uart_Find(UART_HandleTypeDef *huart)
{
  if (huart == &huart1) return &uS1;
  if (huart == &huart2) return &uS2;
  if (huart == &huart3) return &uEsp;
  return NULL;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  UartLine_t *u = Uart_Find(huart);
  if (!u) return;
  Uart_Feed(u);
  HAL_UART_Receive_IT(huart, &u->rxByte, 1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  UartLine_t *u = Uart_Find(huart);
  if (u) HAL_UART_Receive_IT(huart, &u->rxByte, 1);
}

static void Slave_Parse(uint8_t id, const char *s)
{
  int g, f, t, h;
  if (sscanf(s, "S,%d,%d,%d,%d", &g, &f, &t, &h) == 4)
  {
    slave[id].gas = g; slave[id].flame = f;
    slave[id].temp10 = t; slave[id].hum10 = h;
    slave[id].lastTick = HAL_GetTick();
    slave[id].online = 1;
  }
}

static void Esp_Parse(const char *s)
{
  if      (strcmp(s, "CMD,CONTROL") == 0) controlReq = 1;
  else if (strcmp(s, "CMD,OPEN") == 0 && sysState != STATE_OFF)
  { Servo_Set(1); doorByPassword = 1; doorTick = HAL_GetTick(); }
  else if (strcmp(s, "CMD,CLOSE") == 0 && sysState != STATE_ALARM)
  { Servo_Set(0); doorByPassword = 0; }
  else if (strcmp(s, "CMD,OFF") == 0) { systemOn = 0; State_Enter(STATE_OFF); }
  else if (strcmp(s, "CMD,ON") == 0 && !systemOn) { systemOn = 1; State_Enter(STATE_STABILIZING); }
}

static void Uart_Task(void)
{
  if (uS1.ready)  { Slave_Parse(0, uS1.msg); uS1.ready = 0; }
  if (uS2.ready)  { Slave_Parse(1, uS2.msg); uS2.ready = 0; }
  if (uEsp.ready) { Esp_Parse(uEsp.msg);     uEsp.ready = 0; }
}

/* ========================= STATE MACHINE ========================= */
static void State_Enter(SysState_t s)
{
  if (s == sysState && s != STATE_STABILIZING) return;
  SysState_t prev = sysState;
  sysState = s;
  stateTick = HAL_GetTick();

  switch (s)
  {
    case STATE_OFF:
      Servo_Set(0); doorByPassword = 0; controlReq = 0;
      break;
    case STATE_STABILIZING:
      slave[0].online = slave[1].online = 0;
      break;
    case STATE_ALARM:
      Servo_Set(1);
      doorByPassword = 0;
      controlReq = 0;
      break;
    case STATE_CONTROL:
      controlTick = HAL_GetTick();
      break;
    default:
      if (prev == STATE_ALARM) Servo_Set(0);
      break;
  }
  Send_ToSlaves();
}

static void State_Update(void)
{
  uint32_t now = HAL_GetTick();

  for (uint8_t i = 0; i < 2; i++)
    if (slave[i].online && now - slave[i].lastTick > SLAVE_TIMEOUT_MS) slave[i].online = 0;

  if (doorByPassword && now - doorTick >= DOOR_OPEN_MS)
  { doorByPassword = 0; if (sysState != STATE_ALARM) Servo_Set(0); }

  if (!systemOn) return;
  if (sysState == STATE_STABILIZING && now - stateTick < STABILIZE_MS) return;

  uint8_t fire = 0, alert = 0;
  for (uint8_t i = 0; i < 2; i++)
  {
    if (!slave[i].online) continue;
    if (slave[i].flame || slave[i].gas >= GAS_ALARM_TH) fire = 1;
    else if (slave[i].gas >= GAS_ALERT_TH) alert = 1;
  }
  uint8_t fault = !slave[0].online || !slave[1].online;

  if (sysState == STATE_CONTROL && now - controlTick >= CONTROL_MS) controlReq = 0;

  if      (fire)       State_Enter(STATE_ALARM);
  else if (fault)      State_Enter(STATE_FAULT);
  else if (alert)      State_Enter(STATE_ALERT);
  else if (controlReq) State_Enter(STATE_CONTROL);
  else                 State_Enter(STATE_OK);
}

/* ========================== GUI DU LIEU ========================== */
static void Send_ToSlaves(void)
{
  char buf[12];
  int n = snprintf(buf, sizeof(buf), "ST,%d\n", (int)sysState);
  HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)n, 10);
  HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)n, 10);
}

static void Send_ToEsp(void)
{
  char buf[220];
  int n = snprintf(buf, sizeof(buf),
    "{\"state\":\"%s\",\"door\":%d,"
    "\"gas1\":%d,\"flame1\":%d,\"temp1\":%d,\"hum1\":%d,\"on1\":%d,"
    "\"gas2\":%d,\"flame2\":%d,\"temp2\":%d,\"hum2\":%d,\"on2\":%d}\n",
    stateName[sysState], doorOpen,
    slave[0].gas, slave[0].flame, slave[0].temp10, slave[0].hum10, slave[0].online,
    slave[1].gas, slave[1].flame, slave[1].temp10, slave[1].hum10, slave[1].online);
  if (n > (int)sizeof(buf) - 1) n = sizeof(buf) - 1;
  HAL_UART_Transmit(&huart3, (uint8_t *)buf, (uint16_t)n, 50);
}
/* USER CODE END 4 */

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
