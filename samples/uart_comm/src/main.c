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
#include <kernel.h>
#include <drivers/uart.h>
#include <drivers/i2c.h>


// Getting the Device Nodes using the function "DT_NODELABEL(label_name)"


#define SLEEP_TIME_S 1 /* Pause time */
#define I2C_1 "i2c1"


/* USEC_PER_SEC - Time in microsecond; Period of servo motor signal ->  20ms (50Hz) */


void main(void)
{

	
	const struct device *i2c_dev;

	

	

	i2c_dev = device_get_binding(I2C_1);

	if (!i2c_dev){
		printk("Cannot find I2C Device!\n");
		return;
	}
	else
	{
		printk("I2C  device is found \n");
	}


}