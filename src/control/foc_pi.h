#ifndef FOC_PI_H
#define FOC_PI_H

/**
 * @brief PI 控制器的运行状态和参数
 *
 * 电流环中：
 *   输入是电流误差，单位 A；
 *   输出是电压指令，单位 V。
 */
typedef struct
{
	float kp;           // 比例系数，单位 V/A
	float ki;           // 积分系数，单位 V/(A*s)
	float integral;     // 当前积分项，单位 V
	float output_min;   // 输出下限，单位 V
	float output_max;   // 输出上限，单位 V
} foc_pi_controller_t;

/**
 * @brief 初始化 PI 控制器
 *
 * 初始化时会把积分项清零，避免上电时带着旧的控制量。
 */
void foc_pi_init(
	foc_pi_controller_t *controller,
	float kp,
	float ki,
	float output_min,
	float output_max);

/**
 * @brief 执行一次 PI 计算
 *
 * @param controller PI 控制器状态
 * @param error 当前误差，单位 A
 * @param dt_s 两次计算之间的时间，单位 s
 * @return PI 输出，单位 V
 */
float foc_pi_update(
	foc_pi_controller_t *controller,
	float error,
	float dt_s);

/**
 * @brief 清零积分项
 */
void foc_pi_reset(foc_pi_controller_t *controller);

#endif // FOC_PI_H
