
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


// Getting the Device Nodes using the function "DT_NODELABEL(label_name)"



#define MOTOR_1_PWM_LABEL "pwm1"
#define MOTOR_2_PWM_LABEL "pwm2"
#define MOTOR_3_PWM_LABEL "pwm3"
#define MOTOR_4_PWM_LABEL "pwm4"


/* USEC_PER_SEC - Time in microsecond; Period of servo motor signal ->  20ms (50Hz) */
#define PERIOD (USEC_PER_SEC / 50U)  

/* all in micro second */
#define STEP 100  /* PWM pulse step */
#define MINPULSEWIDTH 1000  /* Servo 0 degrees */
#define MIDDLEPULSEWIDTH 1500 /* Servo 90 degrees */
#define MAXPULSEWIDTH 2000  /* Servo 180 degrees */
#define SLEEP_TIME_S 1  /* Pause time */

#define MIN_PERIOD_USEC (USEC_PER_SEC / 64U)
#define MAX_PERIOD_USEC USEC_PER_SEC


void PWM_control(uint8_t *dir, uint32_t *pulse_width)
{
	if (*dir == 1U)
	{
		if (*pulse_width < MAXPULSEWIDTH)
		{
			*pulse_width += STEP;
		}
		else
		{
			*dir = 0U;
		}
	}
	else if (*dir == 0U)
	{
		if (*pulse_width > MINPULSEWIDTH)
		{
			*pulse_width -= STEP;
		}
		else
		{
			*dir = 1U;
		}
	}
}



/**
 *  Function to change servo position to one of three available in sequence: 0->90->180->90->0->...
 *
 * @param[in, out] dir pointer to direction variable.
 * @param[in, out] pulse_width pointer to variable with pulse width value.
 */
void PWM_position(uint8_t *dir, uint32_t *pulse_width){
	if (*dir == 0U){
		*pulse_width = MINPULSEWIDTH;
		*dir = 1U;
	}
	else if (*dir == 1U){
		*pulse_width = MIDDLEPULSEWIDTH;
		*dir = 2U;
	}
	else if (*dir == 2U){
		*pulse_width = MAXPULSEWIDTH;
		*dir = 3U;
	}
	else if (*dir == 3U){
		*pulse_width = MIDDLEPULSEWIDTH;
		*dir = 0U;
	}
}

/**
 *  Function to decoded pulse width to degrees.
 *
 * @param[out] degrees pointer to variable which contain angular position info.
 * @param[in] pulse_width pointer to variable with pulse width value.
 */

void get_deegrees(uint8_t *degrees, uint32_t *pulse_width){
	*degrees =  (uint8_t)((float)(*pulse_width - MINPULSEWIDTH) / (float)(MAXPULSEWIDTH - MINPULSEWIDTH) * 180.0);
}


void main(void){
	printk("Hello World! %s\n", CONFIG_BOARD);
	printk("Hello Samyak! %s\n", CONFIG_BOARD);

	printk("Servo control program %s\n", CONFIG_BOARD);

	struct device *pwm_dev;
	struct device *pwm_dev1;

	/* Set PWM starting positions as 0 degrees */
	uint32_t pulse_width = MINPULSEWIDTH;
	uint8_t dir = 0U;
	uint8_t degrees = 0U;

	pwm_dev = device_get_binding(MOTOR_1_PWM_LABEL);
	pwm_dev1 = device_get_binding(MOTOR_2_PWM_LABEL);

	if (!pwm_dev){
		printk("Cannot find PWM device! %s\n", CONFIG_BOARD);
		return;
	}
	else{
		printk("PWM device find %s\n", CONFIG_BOARD);
		printk("%s\n", MOTOR_1_PWM_LABEL);
	}


	if (!pwm_dev1){
		printk("Cannot find PWM device1! %s\n", CONFIG_BOARD);
		return;
	}
	else{
		printk("PWM device find %s\n", CONFIG_BOARD);
		printk("%s\n", MOTOR_2_PWM_LABEL);
	}

	while (1){
		printk("PWM device cycle\n");

		if (pwm_pin_set_usec(pwm_dev, 1, PERIOD, pulse_width, 0) ){
			printk("PWM pin set fails\n");
			return;
		}

		if (pwm_pin_set_usec(pwm_dev1, 3, PERIOD, pulse_width, 0) ){
			printk("PWM pin set fails in Dev1 \n");
			return;
		}

		get_deegrees(&degrees, &pulse_width);

		printk("PWM pulse width: %d\n", pulse_width);

		printk("Degrees: %d\n", degrees);

		printk("Current direction: %d\n", dir);

		PWM_control(&dir, &pulse_width);

		PWM_position(&dir, &pulse_width);

		printk("Set direction %d\n", dir);

		printk("\n");

		k_sleep(K_SECONDS(SLEEP_TIME_S*5));
	}

}