#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include "esp_err.h"

esp_err_t current_sense_init(void);
esp_err_t current_sense_calibrate(void);
esp_err_t current_sense_read(int *iu_raw, int *iv_raw);

/**
 * @brief 读取 U、V 两相电流，单位为 A。
 *
 * 函数内部会把 ADC 原始值转换为电压，再根据 INA240 的增益和分流电阻
 * 换算为实际电流。调用前必须完成 current_sense_init() 和
 * current_sense_calibrate()。
 */
esp_err_t current_sense_read_amperes(float *iu_a, float *iv_a);

/**
 * @brief 读取三相电流，单位为 A。
 *
 * 当前硬件只测量 U、V 两相，W 相根据三相电流和为 0 重构：
 * iw = -(iu + iv)。
 */
esp_err_t current_sense_read_three_phase(
	float *iu_a,
	float *iv_a,
	float *iw_a);

/**
 * @brief 诊断用：扫描"启动 ADC 的 PWM 相位"，定位采样瞬间真正落在低侧导通
 *        窗口的启动相位。
 *
 * adc_oneshot_read 从调用到真正采样存在固定延迟 T。当 T 大于一个 PWM 周期
 * 时，"先等窗口再读"是无效的——采样瞬间早已飞出窗口。本函数通过在各个相位
 * 启动读取并记录采样瞬间的计数值，直接测出 T 与各相位的命中情况。
 *
 * @param steps  把一个 PWM 周期（2 * COMPARE_MAX_TICKS）分成多少档
 * @param repeat 每档重复采样次数，用于观察抖动
 */
void current_sense_phase_sweep(int steps, int repeat);

#endif // CURRENT_SENSE_H
