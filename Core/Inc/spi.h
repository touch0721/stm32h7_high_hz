/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.h
  * @brief   This file contains all the function prototypes for
  *          the spi.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN Private defines */

/* ==========================================================================
 *  数据帧格式（统一 16 字节，大端，MSB first）
 *
 *  ┌ 模式 001  帧头 AA AA —— 等精度频率计（频率 + 周期）
 *  │   b0 b1  = AA AA
 *  │   b2 b3  = 00 00
 *  │   b4     = {5'b0, nx[26:24]}
 *  │   b5..b7 = nx[23:0]          nx = 1 s 闸门内的上升沿数
 *  │   b8     = {3'b0, n0[28:24]}
 *  │   b9..11 = n0[23:0]          n0 = 周期，单位 300 MHz 拍
 *  │   b12 b13= 00 00
 *  │   b14 b15= 00 F0
 *  │
 *  ├ 模式 010  帧头 AA BB —— 时间间隔
 *  │   b2 b3  = 00 00
 *  │   b4..b7 = t1_data[63:32]   粗计数，200 MHz（5 ns/拍）
 *  │   b8..b11= t1_data[31:0]    TDC 小数（ps）
 *  │   b12..15= 00 00 00 F0
 *  │
 *  └ 模式 100  帧头 AA CC —— 方波占空比
 *      b2 b3  = d_p（10 位补码，高 6 位 0）
 *      b4 b5  = d_h（10 位补码，高 6 位 0）
 *      b6..b9 = P_c（28 位有效）
 *      b10..13= H_c（28 位有效）
 *      b14 b15= 00 F0
 *
 *  分帧：**不按 16 字节硬切**。FPGA 帧间不拉高 CS，NSS 边界和帧边界
 *        对不齐，所以字节先全部进 stream[]，再按
 *        [b0]=AA + [b1] in {AA,BB,CC} + [b14]=00 + [b15]=F0 提帧。
 * ========================================================================== */
#define SPI_FRAME_LEN         16U
#define SPI_TAIL_IDX          (SPI_FRAME_LEN - 2U)   /* = 14 */

/* 字节流环形缓冲，必须是 2 的幂（512 B ≈ 32 帧） */
#define SPI_STREAM_MAX        512U

/* 帧同步字 */
#define SPI_HEAD_MARK         0xAAU                  /* 帧头第 1 字节 */
#define SPI_MODE_FREQ         0xAAU                  /* 模式 001 */
#define SPI_MODE_TIME         0xBBU                  /* 模式 010 */
#define SPI_MODE_DUTY         0xCCU                  /* 模式 100 */
#define SPI_TAIL0             0x00U
#define SPI_TAIL1             0xF0U

/* ---- 换算常数 ---- */
#define REF_CLK_HZ            300000000ULL   /* n0 的拍频 = 300 MHz */
#define FREQ_GATE_MS          1000UL         /* nx 的闸门长度：1 s */
#define T1_COARSE_PS          5000ULL        /* t1 粗计数 1 拍 = 5 ns（200 MHz） */

/* ---- nx 固定扣减（默认关闭）----
 *  0 = 不扣，f = nx（当前设置）
 *  N = 高频分支按 f = nx − N 计算，只影响 f ≥ 1 MHz 那一段；
 *      低频分支本来就用 n0 推频率，不受影响。
 *  扣减带饱和：nx < N 时结果按 0 处理，不会下溢成 0xFFFFFFFF。 */
#define FREQ_NX_OFFSET        0UL

/* ---- 频率标定系数（0.1 ppm 为单位）——修"比例误差"，不是扣固定计数 ----
 *  实测读数比真值高 N×0.1 ppm 时填 N。
 *  38   = 当前设置，按 50 MHz 那组数据标定（读数偏高 3.8 ppm）
 *  0    = 关闭修正，f = nx 原样
 *
 *  标定依据（2026-11-25 现场数据）：
 *      nx=50,000,190 → 50.00019MHz（真值 50 MHz，高 3.8 ppm）
 *      nx=25,000,100 → 25.00010MHz（高 4.0 ppm）
 *      nx=12,000,050 → 12.00005MHz（高 4.2 ppm）
 *  三点线性拟合 a = 1.0000036、b ≈ 10 计数（b 在屏 10 Hz 分辨率内不可分辨），
 *  故按比例误差处理。50 MHz 那点分辨率最好（末位 = 0.2 ppm），以它为准。
 *
 *  作用方式（两条路径同一个系数，方向一致，过 1 MHz 分界不跳变）：
 *      低频（n0）：f = REF_CLK_CAL_HZ / n0
 *      高频（nx）：f = nx × (1e7 − N) / 1e7
 *  物理含义：参考时钟实际比 300 MHz 慢 N×0.1 ppm 时闸门变长同样比例，
 *            两条路径都同比例偏高，所以一个系数能同时修好。
 *
 *  注意：若根因是"闸门计数器多计了若干拍"而不是参考时钟偏，
 *        这个系数只该修高频路径，低频路径本来是对的 —— 先做 100 Hz 对照测试。 */
#define FREQ_CAL_PPMX10       38UL

/* 标定后的参考频率：300 MHz 慢 0.1×N ppm → 例如 N=38 → 299,998,860 */
#define REF_CLK_CAL_HZ        (REF_CLK_HZ - (REF_CLK_HZ / 10000000ULL) * (uint64_t)FREQ_CAL_PPMX10)

/* ---- 测周 / 等精度 分界：1 MHz ----
 *  f <  1 MHz  →  频率和周期都只由 n0 推（nx 太小，毛刺影响太大）
 *  f ≥  1 MHz  →  频率用 nx（等精度），周期用 1/f
 *
 *  判据用 n0，不用 nx —— n0 不受毛刺影响：
 *      n0 = REF_CLK_HZ / f，f = 1 MHz 时 n0 = 300
 *      所以 n0 > 300 ⇔ f < 1 MHz
 *
 *  两端量化误差对照：
 *      f = 1 kHz   nx=1000  →  毛刺 50 个计数就是 5%，n0=300000 却是 3e-6
 *      f = 1 MHz   n0=300   →  量化 0.33%，而 nx=1e6 只有 1e-6
 *  分界点正是两边优劣反转的地方。 */
#define FREQ_SPLIT_HZ         1000000UL
#define FREQ_SPLIT_N0         ((uint32_t)(REF_CLK_HZ / (uint64_t)FREQ_SPLIT_HZ))  /* = 300 */

/* ---- TDC 标定值（p1 / t1 共用同一组延迟线）----
 *
 *  延迟线级数 TAPS = 384，单级延迟 tau ≈ 18.6 ~ 19.5 ps：
 *      tau × TAPS = 19.5 × 384 = 7.49 ns
 *  一个粗拍只有 5 ns，所以延迟线是**过覆盖**的，多出约 2.5 ns 余量。
 *  这不是浪费：PVT 让 tau 变小时（19.5 → 18.6，降 4.8%），
 *  384 级仍覆盖 7.14 ns > 5 ns，不会出现空码/漏格。
 *  理论下限 tau = 5000 / 384 = 13.0 ps，实测 18.6 还有 43% 余量。
 *
 *  两个数千万别混：
 *      TAPS = 384   物理延迟线级数 → 决定码的位宽（0~383，需 9 位）
 *      NTAP = 256   一个 5 ns 粗拍覆盖的有效格数 = 5000 / tau ≈ 5000/19.5
 *  "5 ns 内有多少格"问的是 NTAP，不是 TAPS。
 *
 *  d（码差）的合理范围 = ±NTAP ≈ ±256，不是 ±384。
 *  若实测码值落在 256~383，说明零头已绕过一个整拍，应减 NTAP 再使用。
 *  10 位补码可表示 ±512，容纳 384 级有余量，不会溢出。
 *
 *  tau 标定：tau = 5000 / (码_max - 码_min)
 *    输入频率必须和 200 MHz **不成整数比**（用 1.001 MHz，不能用 1.000 MHz，
 *    否则相位每帧不动，码扫不出跨度）。连采取全跨度。
 *
 *  tau 的 PVT 漂移必须重视：18.6 与 19.5 差 4.8%，
 *  在 1 MHz、d=250 时会带来约 0.02% 的占空比变化（两格）。
 *  要守住 0.01%，就得定期重标，或按温度查表。
 *
 *  为了不丢 19.5 的小数，tau 以 **0.1 ps 为单位**存整数：
 *      195 = 19.5 ps      （若标定得 18.6，填 186）
 *  代码里所有时间量统一乘 10 换算到 0.1 ps，比值不受影响。
 *
 *  H 路 offset 是**固定的 2600 ps**，与 tau 无关 ——
 *  它是两条 TDC 链之间的固定延迟差（含首级触发/布线），
 *  不随延迟线每级的 tau 缩放：
 *      H = H_c × 5000 + d_h × tau − 2600
 *  按 FPGA 侧约定，只扣在 H 上，P 不扣。
 *
 *  offset 自检：输入确定的 50.00% 方波，
 *    1 MHz 下应显示 49.74%（= 50% − 2600ps/1us），
 *    10 kHz 下应显示 49.9974%（≈50.00%）。
 *
 *  填好后把 P1_USE_TDC 置 1；未标定时用 0，只用整拍 H_c/P_c
 *  （1 MHz 时分辨率约 0.5%，达不到 0.01%）。 */
#define P1_TDC_TAPS           384U          /* 延迟线级数（码范围 0~383） */
#define P1_TDC_NTAP_PIT       256U          /* 一个 5 ns 粗拍的有效格数 */
#define P1_TDC_TAU_PSX10      195U          /* 单级 tau = 19.5 ps，单位 0.1 ps */
#define P1_USE_TDC            1
#define P1_H_OFFSET_PS        2600U         /* H 路固定 offset（ps），与 tau 无关 */

/* ---- 串口屏控件 ---- */
#define SC_OBJ_MODE           "t0"    /* 当前测量模式 */
#define SC_OBJ_MAIN           "t1"    /* 主值：频率 / 时间间隔 / 占空比 */
#define SC_OBJ_SUB            "t2"    /* 副值：周期（仅模式 001） */
#define SC_OBJ_LEN            2U

/* ---- 屏幕模式名文案 / 中文字库编码 ----
 *  1 = GBK / GB2312 中文（当前默认，大彩、淘晶驰、迪文等屏默认中文字库）
 *  0 = UTF-8 中文（字库按 UTF-8 生成时才用）
 *  2 = 英文 FREQ / TIME / DUTY（排除中文字库问题时的对照档）
 *  两种中文编码的字节长度不同，所以长度一律用 SC_TXT_LEN() 求，
 *  不要再写死数字。 */
#ifndef SC_CN_USE_GBK
#define SC_CN_USE_GBK         1
#endif

#if (SC_CN_USE_GBK == 1)
#define SC_MODE_FREQ   "\xC6\xB5\xC2\xCA\xD3\xEB\xD6\xDC\xC6\xDA\xB2\xE2\xC1\xBF"  /* 频率与周期测量 */
#define SC_MODE_TIME   "\xCA\xB1\xBC\xE4\xBC\xE4\xB8\xF4\xB2\xE2\xC1\xBF"          /* 时间间隔测量   */
#define SC_MODE_DUTY   "\xC2\xF6\xB3\xE5\xD5\xBC\xBF\xD5\xB1\xC8\xB2\xE2\xC1\xBF"  /* 脉冲占空比测量 */
#elif (SC_CN_USE_GBK == 0)
#define SC_MODE_FREQ   "频率与周期测量"
#define SC_MODE_TIME   "时间间隔测量"
#define SC_MODE_DUTY   "脉冲占空比测量"
#else
#define SC_MODE_FREQ   "FREQ"
#define SC_MODE_TIME   "TIME"
#define SC_MODE_DUTY   "DUTY"
#endif

/* 字符串字面量的字节长度（不含结尾 '\0'） */
#define SC_TXT_LEN(s)         ((uint16_t)(sizeof(s) - 1U))

/* ---- 数值前缀 ---- */
#define SC_PRE_F              "F="     /* 频率   F=999.99900kHz */
#define SC_PRE_F_LEN          2U
#define SC_PRE_T              "T="     /* 周期   T=1.00000us    */
#define SC_PRE_T_LEN          2U
#define SC_PRE_TS             "Ts="    /* 时间   Ts=0.0988us    */
#define SC_PRE_TS_LEN         3U
#define SC_PRE_D              "D="     /* 占空比 D=49.87%       */
#define SC_PRE_D_LEN          2U

/* ---- 刷新节流 ----
 * 同模式最快 5 Hz（模式切换仍然立即刷新）。
 * 串口屏每条指令都要解析 + 算字宽 + 重绘，20 Hz × 3 条 = 60 条/秒会把它的
 * 刷新队列灌满，表现就是屏上"卡住不动"。参照已验证可用的 ImpedScope 工程，
 * 它是 500 ms 刷 10 个控件（20 条/秒），这里 200 ms 刷 2~3 个控件（10~15 条/秒）。 */
#define SPI_DATA_TX_MIN_GAP_MS  200U

/* ---- 空闲提示 ---- */
#define SPI_WAIT_TEXT         "spi wait"
#define SPI_WAIT_LEN          8U
#define SPI_WAIT_RESEND_MS    500U
#define SPI_WAIT_ALWAYS       0        /* 0 = 收到数据后不再覆盖 */

/* 一帧文本的最大长度（不含 "tX.txt=\"\"" 和 3 个 0xFF）
   最长 "D=100.00%"、UTF-8 下的中文模式名 21 字节，留足余量 */
#define SPI_TEXT_MAX          40U

/* 一条指令拼进发送缓冲的最大占用： "tX" + .txt=" (6) + 40 + " + 3 个 0xFF */
#define SC_CMD_MAX            (SC_OBJ_LEN + 6U + SPI_TEXT_MAX + 1U + 3U)

/* 整帧发送缓冲：一次刷新最多 3 条（t0/t1/t2）+ 余量 */
#define SC_BUF_MAX            (3U * SC_CMD_MAX + 8U)

/* 整帧一次性发送的超时（ms）。一帧最多约 164 字节，921600 下不到 2 ms，
   100 已经非常宽松；屏或线断了也不会把主循环卡死 */
#define SC_TX_TIMEOUT_MS      100U

/* 两条指令之间留给串口屏解析/重绘的间隙（ms），0 = 整帧一次灌完。
 * 取值参照已验证可用的 ImpedScope 工程（USE/screen.c 的 SCREEN_CMD_GAP_MS）：
 * 921600 下连续灌多条，屏来不及消费，会不刷新或只刷新一部分。 */
#ifndef SC_CMD_GAP_MS
#define SC_CMD_GAP_MS         2U
#endif

/* 半帧卡死看门狗（ms）：一个 16 字节块只收到一部分、之后长时间无新字节，
   说明传输被截断；放弃这块重挂，避免 HAL 卡在 BUSY_RX。0 = 关闭。 */
#define SPI_RX_STALL_MS       200U

/* USER CODE END Private defines */

void MX_SPI2_Init(void);

/* USER CODE BEGIN Prototypes */
void SPI_Slave_Process(void);      /* 主循环中调用：提帧 → 按模式解析 → 送屏 */
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */
