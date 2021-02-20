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
// #include "uart_comm.h"
// Getting the Device Nodes using the function "DT_NODELABEL(label_name)"


#define SLEEP_TIME_S 1 /* Pause time */
#define MY_UART_2 "UART_2"
#define MY_UART_1 "UART_1"


/* USEC_PER_SEC - Time in microsecond; Period of servo motor signal ->  20ms (50Hz) */


// unsigned char data_read;
uint16_t buffer[16];

bool lostFrame;
bool failSafe;

// Uart Configure
struct uart_config uart_cfg_check;
const struct uart_config uart_cfg = {
		.baudrate = 100000,
		.parity = UART_CFG_PARITY_EVEN,
		.stop_bits = UART_CFG_STOP_BITS_2,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE
	};




void main(void)
{

	printk("UART communicantio program\n");

	const struct device *uart2_dev;
	const struct device *uart1_dev;
	

	
	// uart2_dev = device_get_binding(MY_UART_2);

	// if (!uart2_dev){
	// 	printk("Cannot find UART2 device!\n");
	// 	return;
	// }
	// else
	// {
	// 	printk("UARt2  device is found \n");
	// }

	uart1_dev = device_get_binding(MY_UART_1);

	if (!uart1_dev){
		printk("Cannot find UART1 device!\n");
		return;
	}
	else
	{
		printk("UARt1  device is found \n");
	}

	/* Verify configure() - set device configuration using data in cfg */
	int ret = uart_configure(uart1_dev, &uart_cfg);

	if (ret == 0) 
	{
		// return TC_SKIP;
		printk("Configure Successful \n");
	}
	else
	{
		printk("Configure unSuccessful \n");
	}
	


	int myval = -1;
	int count = 1;

	uint16_t data_read;

	// printk("will comeback! %d\n", myval);
	// int j=0;
	// if((begin()) == 0){
			
	// 	while(true){
			
	// 		// printk();
	// 		bool readRet = read(&buffer[0], &failSafe, &lostFrame, ret, uart1_dev);

	// 		if (readRet == true){
	// 			j++;
	// 			printk("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	// 			printk("data number : %d \n", j);
	// 			for(int i= 0; i<=16; i++){
				
	// 				printk("%d - \n", buffer[i]);
	// 			}
	// 			printk("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");

	// 		}else{
	// 			printk("################################################################\n");
	// 			printk("Garbage data received, but not header frame,\n");
	// 			printk("Garbage data number : %d \n", j);
	// 			for(int i= 0; i<=16; i++){
					
	// 				printk("%d -- \n", buffer[i]);
	// 			}
	// 			printk("################################################################\n");
	// 		}
	// 	}
		
	// }
	// else{
	// 		printk("Begin failed");
	// }

	while (true)
	{
		
		// printk("Before reading the data! %d\n", myval); // Using printk function before uart_poll_in creates  a priotiy problem in the UART bus relase don't use it over here.
		myval = uart_poll_in(uart1_dev, &data_read);
		// printk("Just after reading the UART data %d \n", myval);

		// k_sleep(K_SECONDS(SLEEP_TIME_S));

		if (myval== 0)
		{
			buffer[count] = (uint8_t)data_read;
			printk("After polling %d \n", myval);

			for(int i= 0; i<=24; i++){
				printk("%d", buffer[i]);
			}
			
			printk("\n");
			// k_sleep(K_SECONDS(SLEEP_TIME_S*2));

		}

	}

}