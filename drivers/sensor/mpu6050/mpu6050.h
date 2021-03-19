/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_MPU6050_MPU6050_H_
#define ZEPHYR_DRIVERS_SENSOR_MPU6050_MPU6050_H_

#include <device.h>
#include <drivers/gpio.h>
#include <sys/util.h>
#include <zephyr/types.h>

#define MPU6050_REG_CHIP_ID		0x75
#define MPU6050_CHIP_ID			0x68

#define MPU6050_REG_GYRO_CFG		0x1B
#define MPU6050_GYRO_FS_SHIFT		3

#define MPU6050_REG_ACCEL_CFG		0x1C
#define MPU6050_ACCEL_FS_SHIFT		3

#define MPU6050_REG_INT_EN		0x38
#define MPU6050_DRDY_EN			BIT(0)

#define MPU6050_REG_DATA_START		0x3B

#define MPU6050_REG_PWR_MGMT1		0x6B
#define MPU6050_SLEEP_EN		BIT(6)

#define MPU6050_RA_USER_CTRL        0x6A

#define MPU6050_USERCTRL_DMP_EN_BIT             BIT(7)
#define MPU6050_USERCTRL_FIFO_EN_BIT            BIT(6)
#define MPU6050_USERCTRL_I2C_MST_EN_BIT         BIT(5)
#define MPU6050_USERCTRL_I2C_IF_DIS_BIT         BIT(4)
#define MPU6050_USERCTRL_DMP_RESET_BIT          BIT(3)
#define MPU6050_USERCTRL_FIFO_RESET_BIT         BIT(2)
#define MPU6050_USERCTRL_I2C_MST_RESET_BIT      BIT(1)
#define MPU6050_USERCTRL_SIG_COND_RESET_BIT     BIT(0)

/* 
	(*) Why the heck two different registers for the lower bit and higher bit?
	--> The answer to this question is that, when using the IMU Zero Library for calibrating the IMU, every axis gets 
		two values the lower and upper offset, the USR in the macro stands for the User set offset to the IMU.

 */
#define MPU6050_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU6050_RA_XG_OFFS_USRL     0x14

#define MPU6050_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU6050_RA_YG_OFFS_USRL     0x16

#define MPU6050_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU6050_RA_ZG_OFFS_USRL     0x18



/* measured in degrees/sec x10 to avoid floating point */
static const uint16_t mpu6050_gyro_sensitivity_x10[] = {
	1310, 655, 328, 164
};



struct mpu6050_data {
	const struct device *i2c;

	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	uint16_t accel_sensitivity_shift;

	int16_t temp;

	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	uint16_t gyro_sensitivity_x10;

#ifdef CONFIG_MPU6050_TRIGGER
	const struct device *dev;
	const struct device *gpio;
	struct gpio_callback gpio_cb;

	struct sensor_trigger data_ready_trigger;
	sensor_trigger_handler_t data_ready_handler;

#if defined(CONFIG_MPU6050_TRIGGER_OWN_THREAD)
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_MPU6050_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem gpio_sem;
#elif defined(CONFIG_MPU6050_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
#endif

#endif /* CONFIG_MPU6050_TRIGGER */
};




struct mpu6050_config {
	const char *i2c_label;
	uint16_t i2c_addr;
#ifdef CONFIG_MPU6050_TRIGGER
	uint8_t int_pin;
	uint8_t int_flags;
	const char *int_label;
#endif /* CONFIG_MPU6050_TRIGGER */
};





#ifdef CONFIG_MPU6050_TRIGGER
int mpu6050_trigger_set(const struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler);

int mpu6050_init_interrupt(const struct device *dev);
#endif

#endif /* __SENSOR_MPU6050__ */
