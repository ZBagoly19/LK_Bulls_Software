/*
 * File: DC_MOTOR_cfg.c
 * Driver Name: [[ DC MOTOR ]]
 * SW Layer:   ECUAL
 * Created on: Jun 28, 2020
 * Author:     Khaled Magdy
 * -------------------------------------------
 * For More Information, Tutorials, etc.
 * Visit Website: www.DeepBlueMbedded.com
 *
 */

#include "../DC_MOTOR/DC_MOTOR.h"

const DC_MOTOR_CfgType DC_MOTOR_CfgParam[DC_MOTOR_UNITS] =
{
	// PWM 1 Configurations
    {
		TIM8,
		TIM_CHANNEL_2,	//PIN_57 on C8
		72,
		DC_MOTOR_F_PWM,
		DC_MOTOR_PWM_RES
	},
    // PWM 1 Configurations
	{
		TIM8,
		TIM_CHANNEL_3,	//PIN_40 on C7
		72,
		DC_MOTOR_F_PWM,
		DC_MOTOR_PWM_RES
	}
};
