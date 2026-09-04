#ifndef FOC_CONTROLLER_H
#define FOC_CONTROLLER_H

#include "esp_err.h"
#include "foc_pi.h"

/**
 * @brief 单次 FOC 计算的输入数据。
 *
 * 上层任务负责读取传感器，并把本次采样结果填入这个结构体。
 */
typedef struct
{
	float iu_a;                 // U相实际电流，单位 A
	float iv_a;                 // V相实际电流，单位 A
	float iw_a;                 // W相实际电流，单位 A
	float electrical_angle_rad;        // 电流采样时刻的转子电角度，单位 rad
	float output_electrical_angle_rad; // 预计PWM生效时刻的电角度，单位 rad
	float id_ref_a;             // d轴目标电流，单位 A
	float iq_ref_a;             // q轴目标电流，单位 A
	float dt_s;                 // 本次控制周期，单位 s
	float bus_voltage_v;        // 直流母线电压，单位 V
	float electrical_velocity_rad_s; // 电角速度，单位 rad/s
	float motor_phase_resistance_ohm; // 电机相电阻，单位 ohm
	float motor_inductance_d_h;      // d轴电感，单位 H
	float motor_inductance_q_h;      // q轴电感，单位 H
	float motor_flux_linkage_wb;     // 永磁体磁链，单位 Wb
} foc_controller_input_t;

/**
 * @brief 单次 FOC 计算的输出数据。
 *
 * 保留中间结果，方便串口调试和检查每一步的数值。
 */
typedef struct
{
	float i_alpha_a;       // Clarke变换后的alpha轴电流，单位 A
	float i_beta_a;        // Clarke变换后的beta轴电流，单位 A
	float i_d_a;           // Park变换后的d轴实际电流，单位 A
	float i_q_a;           // Park变换后的q轴实际电流，单位 A
	float id_error_a;      // d轴电流误差，单位 A
	float iq_error_a;      // q轴电流误差，单位 A
	float vd_v;            // d轴总电压指令，单位 V
	float vq_v;            // q轴总电压指令，单位 V
	float vd_decoupling_v; // d轴解耦/反电势前馈，单位 V
	float vq_decoupling_v; // q轴解耦/反电势前馈，单位 V
	float voltage_vector_limit_v; // dq电压矢量限幅，单位 V
	float v_alpha_v;       // alpha轴电压指令，单位 V
	float v_beta_v;        // beta轴电压指令，单位 V
	float u_voltage_v;     // U相电压指令，单位 V
	float v_voltage_v;     // V相电压指令，单位 V
	float w_voltage_v;     // W相电压指令，单位 V
	float duty_u;          // U相PWM占空比，范围0.0～1.0
	float duty_v;          // V相PWM占空比，范围0.0～1.0
	float duty_w;          // W相PWM占空比，范围0.0～1.0
} foc_controller_output_t;

/**
 * @brief FOC 控制器状态。
 *
 * d轴和q轴各有一个独立的PI控制器。
 */
typedef struct
{
	foc_pi_controller_t id_pi;
	foc_pi_controller_t iq_pi;
} foc_controller_t;

/**
 * @brief 初始化 FOC 控制器。
 *
 * 初始化两个电流PI，并清零它们的积分项。
 */
void foc_controller_init(
	foc_controller_t *controller,
	float id_kp,
	float id_ki,
	float iq_kp,
	float iq_ki,
	float voltage_min_v,
	float voltage_max_v);

/**
 * @brief 清除 d 轴和 q 轴 PI 的积分项。
 */
void foc_controller_reset(
	foc_controller_t *controller);

/**
 * @brief 执行一次完整的 FOC 数学计算。
 *
 * 数据流程：
 * 三相电流 → Clarke → Park → 两个PI → 逆Park
 * → 逆Clarke → SVPWM占空比
 *
 * 该函数只计算数据，不会直接启动或更新PWM硬件。
 */
esp_err_t foc_controller_step(
	foc_controller_t *controller,
	const foc_controller_input_t *input,
	foc_controller_output_t *output);

#endif // FOC_CONTROLLER_H
