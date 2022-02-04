/*
 * File: SERVO_cfg.c
 * Driver Name: [[ SERVO Motor ]]
 * SW Layer:   ECUAL
 * Created on: Jun 28, 2020
 * Author:     Khaled Magdy
 * -------------------------------------------
 * For More Information, Tutorials, etc.
 * Visit Website: www.DeepBlueMbedded.com
 *
 */

#include "SERVO.h"

const SERVO_CfgType SERVO_CfgParam[SERVO_NUM] =
{
	// Servo Motor 1 Configurations
    {
	    GPIOB,
		GPIO_PIN_15,
		TIM12,
		&TIM12->CCR2,
		TIM_CHANNEL_2,
		45000000,
		1.49,	// jobbra korm. 0 "fok"		//1.3 mar tobb, mint amit a felfuggesztes bir
		2.2		// balra korm. 180 "fok"	//
    }
};
