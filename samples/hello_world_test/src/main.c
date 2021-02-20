/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
#include <drivers/pwm.h>
#include <drivers/uart.h>
#include <kernel.h>

#define DT_ALIAS_PWM_0_LABEL "PWM_2"
#define PWM_LED0_NODE "PWM_4"
#define MY_UART_RC "UART_1"
#define MY_UART_MON "UART_2"


#ifndef DT_ALIAS_PWM_0_LABEL
#error "PWM device label not defined"
#endif

#ifndef PWM_LED0_NODE
#error "PWM device label not defined"
#endif


#define SBUS_FAILSAFE_INACTIVE 0
#define SBUS_FAILSAFE_ACTIVE   1
#define SBUS_STARTBYTE         0x0f
#define SBUS_ENDBYTE           0x00


/* period of servo motor signal ->  20ms (50Hz) */
#define PERIOD (USEC_PER_SEC / 50U)

// #define IMU_1 (const char *const) MPU6050



void main(void)
{
	
	printk("Hello Samyak! %s\n", CONFIG_BOARD);


	const struct device *i2c_dev;	
	// const char *const label = DT_LABEL(DT_INST(0, invensense_mpu6050));
	// const struct device *mpu6050 = device_get_binding(label);
	
	const char *const IMU_1 = "MPU6050";
	// i2c_dev = device_get_binding(label);
	

	// printk(DT_LABEL(DT_INST(0, invensense_mpu6050)));
	
	i2c_dev = device_get_binding(IMU_1);

	if (!i2c_dev){
		printk("Cannot find UART1 device!\n");
		return;
	}
	else
	{
		printk("UARt1  device is found \n");
	}




}

