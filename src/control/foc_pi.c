#include "foc_pi.h"

#include <stddef.h>
#include <math.h>

void foc_pi_init(
	foc_pi_controller_t *controller,
	float kp,
	float ki,
	float output_min,
	float output_max)
{
	// 空指针不能解引用，否则会导致程序异常。
	if (controller == NULL)
	{
		return;
	}

	controller->kp = kp;
	controller->ki = ki;
	controller->integral = 0.0f;
	controller->output_min = output_min;
	controller->output_max = output_max;
}
// 发现电流不对
// 计算执行器需要施加多大电压
// 电机电流被电压改变
// 再次测量电流
// 电流闭环控制。
float foc_pi_update(
	foc_pi_controller_t *controller,
	float error,
	float dt_s)
{
	// dt 必须大于 0，否则积分计算没有意义。
	if (controller == NULL)
	{
		return 0.0f;
	}

	if (!isfinite(error) ||
		!isfinite(dt_s) ||
		dt_s <= 0.0f)
	{
		return 0.0f;
	}

	// PI 参数和当前积分项必须是正常有限数值，且输出上下限不能反向。
	if (!isfinite(controller->kp) ||
		!isfinite(controller->ki) ||
		!isfinite(controller->integral) ||
		!isfinite(controller->output_min) ||
		!isfinite(controller->output_max) ||
		controller->output_min > controller->output_max)
	{
		return 0.0f;
	}

	// 先计算“假设本次积分全部加入”时的积分项和输出。
	float integral_candidate =
		controller->integral + controller->ki * error * dt_s;
	float output = controller->kp * error + integral_candidate;

	// 输出限幅，同时避免积分项在饱和时无限累加。
	if (output > controller->output_max)
	{
		output = controller->output_max;

		// 误差为负时，下一次积分会让输出离开上限，允许积分更新。
		if (error < 0.0f)
		{
			controller->integral = integral_candidate;
		}
	}
	else if (output < controller->output_min)
	{
		output = controller->output_min;

		// 误差为正时，下一次积分会让输出离开下限，允许积分更新。
		if (error > 0.0f)
		{
			controller->integral = integral_candidate;
		}
	}
	else
	{
		// 输出没有饱和，正常保存本次积分结果。
		controller->integral = integral_candidate;
	}

	return output;
}

void foc_pi_reset(foc_pi_controller_t *controller)
{
	if (controller == NULL)
	{
		return;
	}

	// 只清除积分项，比例参数和输出限幅保持不变。
	controller->integral = 0.0f;
}
