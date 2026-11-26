/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
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
#include "spi.h"

/* USER CODE BEGIN 0 */
#include <string.h>
#include "usart.h"

/* ==========================================================================
 *  SPI2 从机：三种测量模式的数据帧接收 + 串口屏显示
 *
 *  物理层   SPI2 从机，模式 3（CPOL=1 / CPHA=1），8 bit，MSB first
 *  分帧     字节流 + 软同步（不按 16 字节硬切，见 Stream_GetFrame）
 *  帧格式   见 spi.h 顶部（AA AA / AA BB / AA CC 三种，统一 16 字节）
 *
 *  屏幕分工（三个文本控件，不含任何原始码）
 *    t0  当前测量模式        频率与周期测量 / 时间间隔测量 / 脉冲占空比测量
 *    t1  主测量值            F=1.00001MHz / Ts=11.6900us / D=49.99%
 *    t2  周期（仅频率模式）  T=1.00000us
 *
 *  精度约定
 *    频率、周期：单位后 5 位小数
 *    时间间隔  ：单位后 4 位小数
 *    占空比    ：0.01%（两位小数）
 *  全部用整数定点拼字符串，不依赖 printf / 浮点。
 * ========================================================================== */

/* ---------- 接收缓冲（双缓冲 ping-pong）---------- */
static uint8_t           rx_buf[2][SPI_FRAME_LEN];
static volatile uint8_t  rx_sel      = 0U;   /* SPI 当前正在写的缓冲号 */

/* ---------- 字节流：所有收到的字节按序堆在这里，分帧在软件里做 ---------- */
static uint8_t           stream[SPI_STREAM_MAX];
static volatile uint16_t stream_wr   = 0U;   /* 写指针（仅 SPI 回调里加） */
static uint16_t          stream_rd   = 0U;   /* 读指针（仅主循环里加） */

#define STREAM_MASK   ((uint16_t)(SPI_STREAM_MAX - 1U))

/* ---------- 状态 ---------- */
static uint32_t          frame_tick  = 0U;   /* 上次收到帧的时刻 */
static uint32_t          data_tick   = 0U;   /* 上次刷新屏的时刻 */
static uint8_t           show_data   = 0U;   /* 1 = 已经显示过数据 */
static uint8_t           last_mode   = 0U;   /* 上一次的帧头第 2 字节 */
static uint32_t          stall_cnt   = 0U;   /* 半帧卡死次数（诊断） */
static uint32_t          resync_cnt  = 0U;   /* 滑过的无效字节数（诊断） */

/* ==========================================================================
 *  串口屏指令 —— 先拼帧，再按指令逐条发出（指令之间留间隙）
 *
 *  一次刷新（可能含 t0/t1/t2 三条）先全部拼进 sc_buf，每条自带 3 个 0xFF；
 *  SC_Flush() 再按 0xFF 0xFF 0xFF 把整帧切开，一条一次 Transmit，
 *  两条之间插 SC_CMD_GAP_MS 毫秒的间隙。
 *
 *  为什么“切开 + 留间隙”而不是一次灌完：
 *    参照已验证可用的 ImpedScope 工程（USE/screen.c）——同样是
 *    tX.txt="值" + 3×0xFF，它是一条一条发、条间 HAL_Delay(2)，
 *    注释写明“921600 下一次性灌多条会让屏幕整页重画，分开送每条才能被
 *    及时处理”。屏的 HMI 固件收到指令要解析、算字宽、重绘，
 *    连续灌进去它来不及消费，表现为不刷新或只刷新一部分。
 *
 *    但指令**内部**不能有间隙（拼帧的意义）：一条指令的字节必须连成一串，
 *    否则屏的字节间隔超时会把半条指令丢掉。
 *
 *  SC_CMD_GAP_MS 置 0 时退化为“整帧一次 Transmit”，用于对照测试。
 * ========================================================================== */
static char     sc_buf[SC_BUF_MAX];
static uint16_t sc_len = 0U;

/* 开始一帧（清空拼帧缓冲） */
static void SC_Begin(void)
{
  sc_len = 0U;
}

/* 追加一条： <obj>.txt="<pre><txt>" FF FF FF
 * pre 为 NULL 或 prelen=0 表示不带前缀；装不下时整条丢弃，不写坏缓冲 */
static void SC_Add(const char *obj, const char *pre, uint16_t prelen,
                   const char *txt, uint16_t len)
{
  uint16_t need = (uint16_t)(SC_OBJ_LEN + 6U + prelen + len + 1U + 3U);

  if (((uint32_t)sc_len + (uint32_t)need) > (uint32_t)SC_BUF_MAX)
  {
    return;
  }

  memcpy(&sc_buf[sc_len], obj, SC_OBJ_LEN);
  sc_len = (uint16_t)(sc_len + SC_OBJ_LEN);
  memcpy(&sc_buf[sc_len], ".txt=\"", 6U);
  sc_len = (uint16_t)(sc_len + 6U);

  if ((pre != NULL) && (prelen != 0U))
  {
    memcpy(&sc_buf[sc_len], pre, prelen);
    sc_len = (uint16_t)(sc_len + prelen);
  }
  if (len != 0U)
  {
    memcpy(&sc_buf[sc_len], txt, len);
    sc_len = (uint16_t)(sc_len + len);
  }

  sc_buf[sc_len++] = '"';
  sc_buf[sc_len++] = (char)0xFF;      /* 指令结束符：3 个原始字节 0xFF */
  sc_buf[sc_len++] = (char)0xFF;
  sc_buf[sc_len++] = (char)0xFF;
}

/* 把整帧发出去：按 0xFF 0xFF 0xFF 切成一条条，指令之间插间隙。
 * 文本里不会出现 0xFF（ASCII/GBK 都没有该字节），切分不会误判。 */
static void SC_Flush(void)
{
  uint16_t pos = 0U;

  if (sc_len == 0U)
  {
    return;
  }

#if (SC_CMD_GAP_MS == 0U)
  /* 对照模式：整帧一次灌完 */
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)sc_buf, sc_len, SC_TX_TIMEOUT_MS);
#else
  while (pos < sc_len)
  {
    uint16_t end = sc_len;
    uint16_t i;

    for (i = pos; (uint16_t)(i + 2U) < sc_len; ++i)
    {
      if (((uint8_t)sc_buf[i] == 0xFFU) &&
          ((uint8_t)sc_buf[i + 1U] == 0xFFU) &&
          ((uint8_t)sc_buf[i + 2U] == 0xFFU))
      {
        end = (uint16_t)(i + 3U);
        break;
      }
    }

    if (end <= pos)
    {
      break;                             /* 空帧，退出（正常不会出现） */
    }

    (void)HAL_UART_Transmit(&huart1, (uint8_t *)&sc_buf[pos],
                            (uint16_t)(end - pos), SC_TX_TIMEOUT_MS);
    pos = end;

    if (pos < sc_len)
    {
      HAL_Delay(SC_CMD_GAP_MS);           /* 给屏解析 + 重绘的时间 */
    }
  }
#endif

  sc_len = 0U;
}

/* ---------- 无符号整数转十进制，左补零到 width 位（width ≤ 10） ---------- */
static uint16_t U32_ToStrN(char *dst, uint32_t v, uint8_t width)
{
  char     tmp[10];
  uint8_t  k = 0U;
  uint16_t n = 0U;

  do
  {
    tmp[k++] = (char)('0' + (v % 10U));
    v /= 10U;
  } while (v != 0U);

  while (k < width)
  {
    tmp[k++] = '0';
  }
  while (k != 0U)
  {
    dst[n++] = tmp[--k];
  }
  return n;
}

/* ==========================================================================
 *  定点分数 → 5 位小数文本（频率 / 周期共用一套思路）
 *
 *  频率和周期本质上都是两个整数的比值：
 *      低频（f < 1 MHz）  f = REF_CLK_HZ / n0      T = n0 / REF_CLK_HZ
 *      高频（f ≥ 1 MHz）  f = nx / 1               T = 1 / nx
 *  所以统一按 "num/den" 传入，避免先折成整数 Hz 再丢掉小数。
 *
 *  全程 uint64 定点，无浮点、无 printf、不调 libm。
 *  溢出核对：
 *      频率  num×1e5 ≤ 3e8×1e5 = 3e13
 *      周期  num×1e11 ≤ den/1e3×1e11 = den×1e8 ≤ 3e16   （us 档 num 必然 < den/1e3）
 *  均远小于 uint64 上限 1.8e19。
 * ========================================================================== */

/* ---------- 频率：f = num/den Hz → "1.00001MHz" / "1.00001kHz" / "1.00001Hz" ---------- */
static uint16_t Freq5_Ratio(char *dst, uint64_t num, uint64_t den)
{
  static const uint64_t unit_of[3] = { 1ULL, 1000ULL, 1000000ULL };
  const char *suf;
  uint64_t    v, d;
  uint8_t     k, sl, tries;
  uint32_t    ip, fp;
  uint16_t    n = 0U;

  if (den == 0ULL)
  {
    memcpy(dst, "---", 3U);
    return 3U;
  }

  /* 粗选档位：整数商判量级，不会溢出 */
  v = num / den;
  if      (v >= 1000000ULL) { k = 2U; }      /* MHz */
  else if (v >= 1000ULL)    { k = 1U; }      /* kHz */
  else                      { k = 0U; }      /* Hz  */

  for (tries = 0U; ; tries++)
  {
    d = den * unit_of[k];
    v = (num * 100000ULL + d / 2ULL) / d;    /* 单位 1e-5 Hz，四舍五入 */
    ip = (uint32_t)(v / 100000ULL);
    fp = (uint32_t)(v % 100000ULL);

    /* 四舍五入把 999.99999x 顶成 1000.00000 → 升一档重算 */
    if ((ip < 1000U) || (k >= 2U) || (tries != 0U))
    {
      break;
    }
    k++;
  }

  suf = (k == 2U) ? "MHz" : ((k == 1U) ? "kHz" : "Hz");
  sl  = (k == 0U) ? 2U : 3U;

  n = (uint16_t)(n + U32_ToStrN(&dst[n], ip, 1U));
  dst[n++] = '.';
  n = (uint16_t)(n + U32_ToStrN(&dst[n], fp, 5U));
  memcpy(&dst[n], suf, sl);
  return (uint16_t)(n + sl);
}

/* ---------- 周期：T = num/den 秒 → "1.00000s" / "1.00000ms" / "1.00000us" ---------- */
static uint16_t Period5_Ratio(char *dst, uint64_t num, uint64_t den)
{
  const char *suf;
  uint64_t    v, mul;
  uint8_t     k, sl, tries;
  uint32_t    ip, fp;
  uint16_t    n = 0U;

  if ((den == 0ULL) || (num == 0ULL))
  {
    memcpy(dst, "---", 3U);
    return 3U;
  }

  /* 档位：T ≥ 1s → s；T ≥ 1ms → ms；否则 us */
  if      (num >= den)             { k = 2U; }
  else if (num * 1000ULL >= den)   { k = 1U; }
  else                             { k = 0U; }

  for (tries = 0U; ; tries++)
  {
    /* mul 把 T 换算成该单位下的 1e-5：s→1e5、ms→1e8、us→1e11 */
    mul = (k == 2U) ? 100000ULL
        : ((k == 1U) ? 100000000ULL : 100000000000ULL);
    v  = (num * mul + den / 2ULL) / den;
    ip = (uint32_t)(v / 100000ULL);
    fp = (uint32_t)(v % 100000ULL);

    /* 999.99999us 四舍五入成 1000.00000us → 升级显示为 1.00000ms */
    if ((ip < 1000U) || (k >= 2U) || (tries != 0U))
    {
      break;
    }
    k++;
  }

  suf = (k == 2U) ? "s" : ((k == 1U) ? "ms" : "us");
  sl  = (k == 2U) ? 1U : 2U;

  n = (uint16_t)(n + U32_ToStrN(&dst[n], ip, 1U));
  dst[n++] = '.';
  n = (uint16_t)(n + U32_ToStrN(&dst[n], fp, 5U));
  memcpy(&dst[n], suf, sl);
  return (uint16_t)(n + sl);
}

/* ==========================================================================
 *  时间间隔 → "11.6900us" / "1.0000ms" / "1.0000s"（4 位小数）
 *  ps = 粗计数 × 5000 + TDC 小数
 * ========================================================================== */
static uint16_t Time4_ToText(char *dst, uint64_t ps)
{
  const char *suf;
  uint8_t     suflen;
  uint64_t    div, ip, rem, fp;
  uint16_t    n = 0U;
  uint8_t     bumped = 0U;

  /* 若四舍五入后进位到 1000.0000，说明已经够下一档，升级单位重算一次 */
  for (;;)
  {
    if (ps >= 1000000000000ULL)      { div = 1000000000000ULL; suf = "s";  suflen = 1U; }
    else if (ps >= 1000000000ULL)    { div = 1000000000ULL;    suf = "ms"; suflen = 2U; }
    else                             { div = 1000000ULL;       suf = "us"; suflen = 2U; }

    ip  = ps / div;
    rem = ps % div;
    fp  = (rem * 10000ULL + div / 2ULL) / div;      /* 4 位小数 */

    if ((fp < 10000ULL) || (bumped != 0U))
    {
      break;
    }
    ps += (div - rem);            /* 进位到下一个整单位 */
    bumped = 1U;
  }

  if (fp >= 10000ULL)
  {
    fp = 0ULL;
  }

  n = (uint16_t)(n + U32_ToStrN(&dst[n], (uint32_t)ip, 1U));
  dst[n++] = '.';
  n = (uint16_t)(n + U32_ToStrN(&dst[n], (uint32_t)fp, 4U));
  memcpy(&dst[n], suf, suflen);
  n = (uint16_t)(n + suflen);
  return n;
}

/* ==========================================================================
 *  占空比 → "49.99%"（0.01% 分辨率，两位小数）
 * ========================================================================== */
static uint16_t Duty2_ToText(char *dst, uint64_t h_ps, uint64_t p_ps)
{
  uint64_t v;                        /* 单位 0.01% */
  uint16_t n = 0U;

  if (p_ps == 0ULL)
  {
    memcpy(dst, "---", 3U);
    return 3U;
  }

  v = (h_ps * 10000ULL + p_ps / 2ULL) / p_ps;
  if (v > 10000ULL)                  /* 上限 100.00% */
  {
    v = 10000ULL;
  }

  n = (uint16_t)(n + U32_ToStrN(&dst[n], (uint32_t)(v / 100ULL), 1U));
  dst[n++] = '.';
  n = (uint16_t)(n + U32_ToStrN(&dst[n], (uint32_t)(v % 100ULL), 2U));
  dst[n++] = '%';
  return n;
}

/* ---------- 挂接收：一次固定 SPI_FRAME_LEN（16）字节 ---------- */
static void SPI_Slave_RxArm(uint8_t idx)
{
  (void)HAL_SPI_Receive_IT(&hspi2, rx_buf[idx], SPI_FRAME_LEN);
}

/* ==========================================================================
 *  从字节流里提取一帧（定长 16，AA + 模式字节 + 00 F0）
 *
 *  四重条件同时满足才认：
 *    [rd+0]  == 0xAA
 *    [rd+1]  ∈ {0xAA, 0xBB, 0xCC}      （三选一，同时确定模式）
 *    [rd+14] == 0x00
 *    [rd+15] == 0xF0
 *  不满足就滑 1 字节重找；缓冲区被写爆时丢最旧的。
 * ========================================================================== */
static uint8_t Stream_GetFrame(uint8_t *out)
{
  uint16_t avail = (uint16_t)(stream_wr - stream_rd);
  uint16_t i;
  uint8_t  m;

  if (avail > (uint16_t)SPI_STREAM_MAX)
  {
    stream_rd = (uint16_t)(stream_wr - (uint16_t)SPI_STREAM_MAX);
    avail     = (uint16_t)SPI_STREAM_MAX;
    resync_cnt++;
  }

  while (avail >= (uint16_t)SPI_FRAME_LEN)
  {
    m = stream[(uint16_t)(stream_rd + 1U) & STREAM_MASK];

    if ((stream[stream_rd & STREAM_MASK] == (uint8_t)SPI_HEAD_MARK) &&
        ((m == (uint8_t)SPI_MODE_FREQ) ||
         (m == (uint8_t)SPI_MODE_TIME) ||
         (m == (uint8_t)SPI_MODE_DUTY)) &&
        (stream[(uint16_t)(stream_rd + SPI_TAIL_IDX) & STREAM_MASK] ==
         (uint8_t)SPI_TAIL0) &&
        (stream[(uint16_t)(stream_rd + SPI_TAIL_IDX + 1U) & STREAM_MASK] ==
         (uint8_t)SPI_TAIL1))
    {
      for (i = 0U; i < (uint16_t)SPI_FRAME_LEN; i++)
      {
        out[i] = stream[(uint16_t)(stream_rd + i) & STREAM_MASK];
      }
      stream_rd = (uint16_t)(stream_rd + (uint16_t)SPI_FRAME_LEN);
      return 1U;
    }

    stream_rd++;
    avail--;
    resync_cnt++;
  }

  return 0U;
}

/* ---------- 半帧卡死看门狗 ---------- */
static void SPI_CheckRxStall(void)
{
#if (SPI_RX_STALL_MS != 0U)
  static uint16_t last_cnt = SPI_FRAME_LEN;
  static uint32_t last_ms  = 0U;
  uint16_t cnt;
  uint32_t now = HAL_GetTick();

  if (hspi2.State != HAL_SPI_STATE_BUSY_RX)
  {
    last_cnt = SPI_FRAME_LEN;
    last_ms  = now;
    return;
  }

  cnt = (uint16_t)hspi2.RxXferCount;

  if (cnt != last_cnt)
  {
    last_cnt = cnt;
    last_ms  = now;
    return;
  }

  if ((cnt >= SPI_FRAME_LEN) ||
      ((uint32_t)(now - last_ms) < (uint32_t)SPI_RX_STALL_MS))
  {
    return;
  }

  (void)HAL_SPI_Abort(&hspi2);
  stall_cnt++;
  last_cnt = SPI_FRAME_LEN;
  last_ms  = now;
  SPI_Slave_RxArm(rx_sel);
#endif
}
/* USER CODE END 0 */

SPI_HandleTypeDef hspi2;

/* SPI2 init function */
void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */
  /* 使能 SPI2 中断（stm32h7xx_it.c 里没有 SPI2_IRQHandler，向量函数写在
     本文件末尾）。优先级按普通外设给即可，没有别的中断跟它抢。 */
  HAL_NVIC_SetPriority(SPI2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(SPI2_IRQn);

  /* 挂上 16 字节接收，之后由 RX 完成回调原地重挂，永不停 */
  SPI_Slave_RxArm(0U);
  /* USER CODE END SPI2_Init 2 */

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspInit 0 */

  /* USER CODE END SPI2_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI2;
    PeriphClkInitStruct.PLL2.PLL2M = 5;
    PeriphClkInitStruct.PLL2.PLL2N = 128;
    PeriphClkInitStruct.PLL2.PLL2P = 25;
    PeriphClkInitStruct.PLL2.PLL2Q = 2;
    PeriphClkInitStruct.PLL2.PLL2R = 2;
    PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
    PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
    PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
    PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* SPI2 clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI2 GPIO Configuration
    PC1     ------> SPI2_MOSI
    PB10     ------> SPI2_SCK
    PB12     ------> SPI2_NSS
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* SCK 加内部上拉：模式 3 下空闲电平应为高。CS 为高时 SPI 已被硬件
       取消选中，这里只是防止 FPGA 未上电/未驱动时 SCK 悬空。 */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PB12 = SPI2_NSS（硬件 NSS 输入）。
       NSS 由 SPI 外设自己采样：低电平选中、移位；高电平取消选中、
       移位器复位。因此不需要 GPIO 输入、不需要 EXTI、不需要软件 NSS。 */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_MspInit 1 */

  /* USER CODE END SPI2_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();

    /**SPI2 GPIO Configuration
    PC1     ------> SPI2_MOSI
    PB10     ------> SPI2_SCK
    PB12     ------> SPI2_NSS
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_1);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_12);

  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/* ==========================================================================
 *  SPI2 中断向量（CubeMX 未勾 NVIC，所以向量函数写在这里）
 *  如果以后在 CubeMX 里勾上了，请删掉这个函数，避免重复定义。
 * ========================================================================== */
void SPI2_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi2);
}

/* ==========================================================================
 *  接收完成回调：16 字节收满 → 换缓冲立刻重挂，再把这一段追加进字节流
 *  这里**不判断帧边界**，分帧统一交给主循环的 Stream_GetFrame()。
 * ========================================================================== */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  uint8_t  done;
  uint16_t i;
  uint16_t w;

  if (hspi->Instance != SPI2)
  {
    return;
  }

  done    = rx_sel;
  rx_sel ^= 1U;                        /* 0↔1 交替 */
  SPI_Slave_RxArm(rx_sel);             /* 先重挂，间隙最短 */

  w = stream_wr;
  for (i = 0U; i < (uint16_t)SPI_FRAME_LEN; i++)
  {
    stream[w & STREAM_MASK] = rx_buf[done][i];
    w++;
  }
  stream_wr = w;
}

/* ==========================================================================
 *  错误回调（OVR 溢出等）：清标志后重新挂接收，不中断数据流
 * ========================================================================== */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance != SPI2)
  {
    return;
  }

  __HAL_SPI_CLEAR_OVRFLAG(hspi);
  hspi->ErrorCode = HAL_SPI_ERROR_NONE;

  if (hspi->State == HAL_SPI_STATE_READY)
  {
    rx_sel ^= 1U;
    SPI_Slave_RxArm(rx_sel);
  }
}

/* ==========================================================================
 *  主循环里调用：从字节流提帧 → 按模式解析 → 发 t0/t1/t2
 *
 *  模式 001 (AA AA)  t0="频率与周期测量"    t1="F=..."    t2="T=..."
 *  模式 010 (AA BB)  t0="时间间隔测量"      t1="Ts=..."
 *  模式 100 (AA CC)  t0="脉冲占空比测量"    t1="D=..."
 *
 *  模式切换时立即刷新（不受 20 Hz 节流限制），并把 t2 清空，
 *  避免上一模式的周期残留在屏上。
 * ========================================================================== */
void SPI_Slave_Process(void)
{
  static char    t_main[SPI_TEXT_MAX];
  static char    t_sub[SPI_TEXT_MAX];
  static uint8_t buf[SPI_FRAME_LEN];

  uint8_t  got  = 0U;
  uint8_t  mode;
  uint8_t  force;
  uint16_t n1 = 0U;
  uint16_t n2 = 0U;
  uint32_t now = HAL_GetTick();

  /* ---------- 0. 半帧卡死自愈 ---------- */
  SPI_CheckRxStall();

  /* ---------- 1. 消费流里所有完整帧，只保留最后一帧 ---------- */
  while (Stream_GetFrame(buf) != 0U)
  {
    got = 1U;
  }

  if (got == 0U)
  {
    if ((SPI_WAIT_ALWAYS != 0U) || (show_data == 0U))
    {
      if ((uint32_t)(now - frame_tick) >= (uint32_t)SPI_WAIT_RESEND_MS)
      {
        frame_tick = now;
        SC_Begin();
        SC_Add(SC_OBJ_MODE, NULL, 0U, SPI_WAIT_TEXT, SPI_WAIT_LEN);
        SC_Flush();
      }
    }
    return;
  }

  frame_tick = now;
  mode  = buf[1];                          /* Stream_GetFrame 已校验过 */
  force = (mode != last_mode) ? 1U : 0U;

  if (force != 0U)
  {
    last_mode = mode;
  }
  else if ((uint32_t)(now - data_tick) < (uint32_t)SPI_DATA_TX_MIN_GAP_MS)
  {
    return;                                /* 同模式限速 20 Hz */
  }

  data_tick = now;
  show_data = 1U;

  /* 本次刷新的所有指令都拼进同一帧，最后一次发出 */
  SC_Begin();

  if ((force != 0U) && (mode != (uint8_t)SPI_MODE_FREQ))
  {
    SC_Add(SC_OBJ_SUB, NULL, 0U, "", 0U);  /* 非频率模式：清掉 t2 */
  }

  switch (mode)
  {
    /* ================= 模式 001：等精度频率计 =================
     *
     *  分界 1 MHz，判据用 n0 而不是 nx（n0 不受毛刺影响）：
     *      n0 > 300   ⇔  f < 1 MHz  →  频率、周期全部由 n0 推
     *      n0 ≤ 300   ⇔  f ≥ 1 MHz  →  频率用 nx，周期用 1/f
     *
     *  为什么低频放弃 nx：1 kHz 时 nx 只有 1000，一个毛刺（几十个计数）
     *  就是百分之几的误差；而 n0 = 300000，±1 拍仅 3e-6。反过来 1 MHz
     *  时 n0 只剩 300，量化 0.33%，此时 nx ≥ 1e6 才是好选择。
     *
     *  为什么高频周期不用 n0：n0 ≤ 300 只有 3 位有效数字，
     *  而 1/nx 的相对误差仅 1/nx ≈ 1e-6，差两个数量级。
     *
     *  低频时 f 与 T 都从 n0 推出，所以恒有 f×T = 1，不会互相矛盾。
     *
     *  nx 固定扣 FREQ_NX_OFFSET，当前为 0（不扣）；扣减后饱和到 0。
     */
    case SPI_MODE_FREQ:
    {
      uint32_t nx = (((uint32_t)buf[4] & 0x07UL) << 24) |
                    ((uint32_t)buf[5] << 16) |
                    ((uint32_t)buf[6] << 8)  |  (uint32_t)buf[7];
      uint32_t n0 = (((uint32_t)buf[8] & 0x1FUL) << 24) |
                    ((uint32_t)buf[9] << 16) |
                    ((uint32_t)buf[10] << 8) |  (uint32_t)buf[11];

#if (FREQ_NX_OFFSET != 0UL)
      /* 固定扣减，饱和处理（避免 0 − N 下溢）；OFFSET=0 时这段不参与编译 */
      nx = (nx > (uint32_t)FREQ_NX_OFFSET) ? (nx - (uint32_t)FREQ_NX_OFFSET) : 0U;
#endif

#if (FREQ_CAL_PPMX10 != 0UL)
      /* 比例标定：读数偏高 N×0.1 ppm → 压回 (1e7−N)/1e7。
         N=0 时这段不参与编译，nx 原样。 */
      nx = (uint32_t)(((uint64_t)nx * (10000000ULL - (uint64_t)FREQ_CAL_PPMX10)
                       + 5000000ULL) / 10000000ULL);
#endif

      if (n0 > FREQ_SPLIT_N0)
      {
        /* ---- 测周模式（f < 1 MHz）：只信 n0，完全不看 nx ---- */
        n1 = Freq5_Ratio(t_main, REF_CLK_CAL_HZ, (uint64_t)n0);
        n2 = Period5_Ratio(t_sub, (uint64_t)n0, REF_CLK_CAL_HZ);
      }
      else
      {
        /* ---- 等精度模式（f ≥ 1 MHz）：频率用 nx，周期用 1/f ---- */
#if (FREQ_GATE_MS == 1000UL)
        n1 = Freq5_Ratio(t_main, (uint64_t)nx, 1ULL);
        n2 = Period5_Ratio(t_sub, 1ULL, (uint64_t)nx);
#else
        n1 = Freq5_Ratio(t_main, (uint64_t)nx * 1000ULL, (uint64_t)FREQ_GATE_MS);
        n2 = Period5_Ratio(t_sub, (uint64_t)FREQ_GATE_MS, (uint64_t)nx * 1000ULL);
#endif
      }

      SC_Add(SC_OBJ_MODE, NULL, 0U, SC_MODE_FREQ, SC_TXT_LEN(SC_MODE_FREQ));
      SC_Add(SC_OBJ_MAIN, SC_PRE_F, SC_PRE_F_LEN, t_main, n1);
      SC_Add(SC_OBJ_SUB,  SC_PRE_T, SC_PRE_T_LEN, t_sub,  n2);
      break;
    }

    /* ================= 模式 010：时间间隔 =================
       T = 粗计数 × 5 ns + TDC 小数(ps)                        */
    case SPI_MODE_TIME:
    {
      uint32_t coarse = ((uint32_t)buf[4] << 24) | ((uint32_t)buf[5] << 16) |
                        ((uint32_t)buf[6] << 8)  |  (uint32_t)buf[7];
      uint32_t ps     = ((uint32_t)buf[8] << 24) | ((uint32_t)buf[9] << 16) |
                        ((uint32_t)buf[10] << 8) |  (uint32_t)buf[11];
      uint64_t t_ps   = (uint64_t)coarse * T1_COARSE_PS + (uint64_t)ps;

      n1 = Time4_ToText(t_main, t_ps);

      SC_Add(SC_OBJ_MODE, NULL, 0U, SC_MODE_TIME, SC_TXT_LEN(SC_MODE_TIME));
      SC_Add(SC_OBJ_MAIN, SC_PRE_TS, SC_PRE_TS_LEN, t_main, n1);
      break;
    }

    /* ================= 模式 100：方波占空比 =================
       H = H_c × 5000 + d_h × tau − 2600
       P = P_c × 5000 + d_p × tau
       duty = H / P
       tau = 19.5 ps 不是整数，所以全部换算到 0.1 ps 再算，
       比值与单位无关，最后一步除法不受影响。
       offset 是固定 2600 ps，不随 tau 缩放；只扣 H，P 不扣。      */
    case SPI_MODE_DUTY:
    {
      uint32_t p_c = (((uint32_t)buf[6]  << 24) | ((uint32_t)buf[7]  << 16) |
                      ((uint32_t)buf[8]  << 8)  |  (uint32_t)buf[9])  & 0x0FFFFFFFUL;
      uint32_t h_c = (((uint32_t)buf[10] << 24) | ((uint32_t)buf[11] << 16) |
                      ((uint32_t)buf[12] << 8)  |  (uint32_t)buf[13]) & 0x0FFFFFFFUL;
      int32_t  d_p = (int32_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]) & 0x03FF;
      int32_t  d_h = (int32_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]) & 0x03FF;
      int64_t  h_x, p_x;                 /* 单位 0.1 ps（= 100 fs） */

      if ((d_p & 0x0200) != 0) { d_p -= 0x0400; }   /* 10 位补码 → 有符号 */
      if ((d_h & 0x0200) != 0) { d_h -= 0x0400; }

#if (P1_USE_TDC != 0)
      /* 5000 ps → 50000，195 = 19.5 ps，2600 ps → 26000 */
      h_x = (int64_t)h_c * 50000
            + (int64_t)d_h * (int64_t)P1_TDC_TAU_PSX10
            - (int64_t)P1_H_OFFSET_PS * 10;
      p_x = (int64_t)p_c * 50000
            + (int64_t)d_p * (int64_t)P1_TDC_TAU_PSX10;
#else
      h_x = (int64_t)h_c * 50000;
      p_x = (int64_t)p_c * 50000;
#endif

      if (h_x < 0)   { h_x = 0; }
      if (p_x <= 0)  { p_x = 0; }

      n1 = Duty2_ToText(t_main, (uint64_t)h_x, (uint64_t)p_x);

      SC_Add(SC_OBJ_MODE, NULL, 0U, SC_MODE_DUTY, SC_TXT_LEN(SC_MODE_DUTY));
      SC_Add(SC_OBJ_MAIN, SC_PRE_D, SC_PRE_D_LEN, t_main, n1);
      break;
    }

    default:
      break;
  }

  SC_Flush();                              /* 整帧一次发出 */
}
/* USER CODE END 1 */
