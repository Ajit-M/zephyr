/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT invensense_mpu6050



#include <drivers/i2c.h>
#include <init.h>
#include <sys/byteorder.h>
#include <drivers/sensor.h>
#include <logging/log.h>
#include <stdlib.h>
#include <math.h>
#include "mpu6050.h"



LOG_MODULE_REGISTER(MPU6050, CONFIG_SENSOR_LOG_LEVEL); //Registers the module in the Zephyr Kernel 






/* see "Accelerometer Measurements" section from register map description */
static void mpu6050_convert_accel(struct sensor_value *val, int16_t raw_val,
				  uint16_t sensitivity_shift)
{
	int64_t conv_val;

	conv_val = ((int64_t)raw_val * SENSOR_G) >> sensitivity_shift;
	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}



 
/* see "Gyroscope Measurements" section from register map description */
static void mpu6050_convert_gyro(struct sensor_value *val, int16_t raw_val,
				 uint16_t sensitivity_x10)
{
	int64_t conv_val;

	conv_val = ((int64_t)raw_val * SENSOR_PI * 10) /
		   (sensitivity_x10 * 180U);
	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}





/* see "Temperature Measurement" section from register map description */
static inline void mpu6050_convert_temp(struct sensor_value *val,
					int16_t raw_val)
{
	val->val1 = raw_val / 340 + 36;
	val->val2 = ((int64_t)(raw_val % 340) * 1000000) / 340 + 530000;

	if (val->val2 < 0) {
		val->val1--;
		val->val2 += 1000000;
	} else if (val->val2 >= 1000000) {
		val->val1++;
		val->val2 -= 1000000;
	}
}



/* 
	(*) This function is used for accessing the fetched data from the sensor, that is stored in the device object 
 */


static int mpu6050_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct mpu6050_data *drv_data = dev->data;

	switch (chan) {
	
	case SENSOR_CHAN_ACCEL_XYZ:
		mpu6050_convert_accel(val, drv_data->accel_x,
				      drv_data->accel_sensitivity_shift);
		mpu6050_convert_accel(val + 1, drv_data->accel_y,
				      drv_data->accel_sensitivity_shift);
		mpu6050_convert_accel(val + 2, drv_data->accel_z,
				      drv_data->accel_sensitivity_shift);
		break;

	case SENSOR_CHAN_ACCEL_X:
		mpu6050_convert_accel(val, drv_data->accel_x,
				      drv_data->accel_sensitivity_shift);
		break;

	case SENSOR_CHAN_ACCEL_Y:
		mpu6050_convert_accel(val, drv_data->accel_y,
				      drv_data->accel_sensitivity_shift);
		break;

	case SENSOR_CHAN_ACCEL_Z:
		mpu6050_convert_accel(val, drv_data->accel_z,
				      drv_data->accel_sensitivity_shift);
		break;

	case SENSOR_CHAN_GYRO_XYZ:
		mpu6050_convert_gyro(val, drv_data->gyro_x,
				     drv_data->gyro_sensitivity_x10);
		mpu6050_convert_gyro(val + 1, drv_data->gyro_y,
				     drv_data->gyro_sensitivity_x10);
		mpu6050_convert_gyro(val + 2, drv_data->gyro_z,
				     drv_data->gyro_sensitivity_x10);
		break;

	case SENSOR_CHAN_GYRO_X:
		mpu6050_convert_gyro(val, drv_data->gyro_x,
				     drv_data->gyro_sensitivity_x10);
		break;


	case SENSOR_CHAN_GYRO_Y:
		mpu6050_convert_gyro(val, drv_data->gyro_y,
				     drv_data->gyro_sensitivity_x10);
		break;

	case SENSOR_CHAN_GYRO_Z:
		mpu6050_convert_gyro(val, drv_data->gyro_z,
				     drv_data->gyro_sensitivity_x10);
		break;

	default: /* chan == SENSOR_CHAN_DIE_TEMP */
		mpu6050_convert_temp(val, drv_data->temp);
	}

	return 0;
}


/* 
	(*) Sample Fetch only runs once but i guess it will be set the sensor enum for choosing the channels
	(*) Q) So where th
 */


static int mpu6050_sample_fetch(const struct device *dev,
				enum sensor_channel chan)
{
	struct mpu6050_data *drv_data = dev->data;
	const struct mpu6050_config *cfg = dev->config;
	int16_t buf[7];

	/* 
		(*) Burst reading of the device register, it is read and stored in the buffer, which is
		then shifted to the memory assigned to the device data structure object. 
		(*) A point to note that, before assigning the data to the struct it is converted to the 
		16-bit integer from big-endian to the host endianness
		(*) This function needs to be called everytime the task needs the data, then 

	 */


	if (i2c_burst_read(drv_data->i2c, cfg->i2c_addr,
			   MPU6050_REG_DATA_START, (uint8_t *)buf, 14) < 0) {
		LOG_ERR("Failed to read data sample.");
		return -EIO;
	}

	/* 
		Endianness conversion
	 */

	drv_data->accel_x = sys_be16_to_cpu(buf[0]);
	drv_data->accel_y = sys_be16_to_cpu(buf[1]);
	drv_data->accel_z = sys_be16_to_cp
	if (i2c_burst_read(drv_data->i2c, cfg->i2c_addr,
			   MPU6050_REG_DATA_START, (uint8_t *)buf, 14) < 0) {
		LOG_ERR("Failed to read data sample.");
		return -EIO;
	}
u(buf[2]);
	drv_data->temp = sys_be16_to_cpu(buf[3]);
	drv_data->gyro_x = sys_be16_to_cpu(buf[4]);
	drv_data->gyro_y = sys_be16_to_cpu(buf[5]);
	drv_data->gyro_z = sys_be16_to_cpu(buf[6]);

	return 0;
}




static const struct sensor_driver_api mpu6050_driver_api = {
#if CONFIG_MPU6050_TRIGGER
	.trigger_set = mpu6050_trigger_set,
#endif
	.sample_fetch = mpu6050_sample_fetch,
	.channel_get = mpu6050_channel_get,
};






int mpu6050_init(const struct device *dev){	

		printk("Initialization of the device from the init fuction \n");

		struct mpu6050_data *drv_data = dev->data;
		const struct mpu6050_config *cfg = dev->config;
		uint8_t id, i;

	// ------------------------------------------------------------------------------------------------
		
		testConnection(drv_data, cfg);

	// ------------------------------------------------------------------------------------------------

		/* set accelerometer full-scale range */
		for (i = 0U; i < 4; i++) {
			if (BIT(i+1) == CONFIG_MPU6050_ACCEL_FS) {
				break;
			}
		}

		if (i == 4U) {
			LOG_ERR("Invalid value for accel full-scale range.");
			return -EINVAL;
		}
		

		if (i2c_reg_write_byte(drv_data->i2c, cfg->i2c_addr,
					MPU6050_REG_ACCEL_CFG,
					i << MPU6050_ACCEL_FS_SHIFT) < 0) {
			LOG_ERR("Failed to write accel full-scale range.");
			return -EIO;
		}

		drv_data->accel_sensitivity_shift = 14 - i;


	// ------------------------------------------------------------------------------------------------


		/* set gyroscope full-scale range */
		for (i = 0U; i < 4; i++) {
			if (BIT(i) * 250 == CONFIG_MPU6050_GYRO_FS) {
				break;
			}
		}

		if (i == 4U) {
			LOG_ERR("Invalid value for gyro full-scale range.");
			return -EINVAL;
		}

		if (i2c_reg_write_byte(drv_data->i2c, cfg->i2c_addr,
					MPU6050_REG_GYRO_CFG,
					i << MPU6050_GYRO_FS_SHIFT) < 0) {
			LOG_ERR("Failed to write gyro full-scale range.");
			return -EIO;
		}

		drv_data->gyro_sensitivity_x10 = mpu6050_gyro_sensitivity_x10[i];

	// ------------------------------------------------------------------------------------------------



	#ifdef CONFIG_MPU6050_TRIGGER
		if (mpu6050_init_interrupt(dev) < 0) {
			LOG_DBG("Failed to initialize interrupts.");
			return -EIO;
		}
	#endif

		return 0;
}

/**
 * @param drv_data User defined struct for storing the read data from the mpu6050 registers.
 * @param cfg User defined struct for storing the mpu6050 configuration data such as, i2c label, registers 
 * and interrupt pin.   
 * 
 * @brief This function is responsible for looking up if the label exist, testing the connection, checking 
 * the chip ID and waking up the chip.
 * 
*/
int testConnection(struct mpu6050_data *drv_data, const struct mpu6050_config *cfg) {
    
    drv_data->i2c = device_get_binding(cfg->i2c_label);
        if (drv_data->i2c == NULL) {
                LOG_ERR("Failed to get pointer to %s device",
                            cfg->i2c_label);
                return -EINVAL;
        }

        /* check chip ID */
        if (i2c_reg_read_byte(drv_data->i2c, cfg->i2c_addr,
                              MPU6050_REG_CHIP_ID, &id) < 0) {
                LOG_ERR("Failed to read chip ID.");
                return -EIO;
        }

        if (id != MPU6050_CHIP_ID) {
                LOG_ERR("Invalid chip ID.");
                return -EINVAL;
        }
		/* wake up chip */
		if (i2c_reg_update_byte(drv_data->i2c, cfg->i2c_addr,
					MPU6050_REG_PWR_MGMT1, MPU6050_SLEEP_EN,
					0) < 0) {
			LOG_ERR("Failed to wake up chip.");
			return -EIO;
		}
}



/**
 * @brief User defined Gyro Offset
 * 
*/
int setXGyroOffset(struct mpu6050_data *drv_data, const struct mpu6050_config *cfg, int8_t offset){

	//Wrting the User defined Gyroscope X axis offset
	if (i2c_reg_write_byte(drv_data->i2c, cfg->i2c_addr, MPU6050_RA_XG_OFFS_USRH,	offset) < 0) {
			LOG_ERR("Failed to write Gyroscope X axis offset.");
			return -EIO;
		}
}


/**
 * 
*/
int setYGyroOffset(struct mpu6050_data *drv_data, const struct mpu6050_config *cfg, int8_t offset){

	//Wrting the User defined Gyroscope Y axis offset
	if (i2c_reg_write_byte(drv_data->i2c, cfg->i2c_addr, MPU6050_RA_YG_OFFS_USRH,	offset) < 0) {
			LOG_ERR("Failed to write Gyroscope Y axis offset.");
			return -EIO;
		}
}


/**
 * 
*/
int setZGyroOffset(struct mpu6050_data *drv_data, const struct mpu6050_config *cfg, int8_t offset){

	//Wrting the User defined Gyroscope Z axis offset
	if (i2c_reg_write_byte(drv_data->i2c, cfg->i2c_addr, MPU6050_RA_USER_CTRL,	offset) < 0) {
			LOG_ERR("Failed to write Gyroscope Z axis offset.");
			return -EIO;
		}
		
}


/**
 *  @brief This function is intended to remove the offsets from the gyroscope and accelerometer measurements, previously 
 * an averaging method was used on the colleted samples (data) of the sensors output and an mean was found. But recently a
 * guy named "ZHomeSlice" has implemented a faster and simpler method to find offset using PID (mainly PI).  
 * 
 * 
 *  Q) Why tf this getDeviceID() < 0x38 ? (I understand that there is difference between registers for the accel offset 
 * but how does this differentiate the things)
*/
void PID(struct mpu6050_data *drv_data, const struct mpu6050_config *cfg, uint8_t ReadAddress, float kP,float kI, uint8_t Loops){
	
	uint8_t SaveAddress = (ReadAddress == 0x3B)?((getDeviceID() < 0x38 )? 0x06:0x77):0x13;
	int16_t  Data;
	float Reading;
	int16_t BitZero[3]; 
	uint8_t shift =(SaveAddress == 0x77)?3:2;
	float Error, PTerm, ITerm[3];
	int16_t eSample;
	uint32_t eSum;
 

	// Serial.write('>');
	for (int i = 0; i < 3; i++) {
		i2c_burst_read(drv_data->i2c, cfg->i2c_addr,SaveAddress + (i * shift), &Data, 2);
				Reading = Data;
			if(SaveAddress != 0x13){
				BitZero[i] = Data & 1;										 // Capture Bit Zero to properly handle Accelerometer calibration
				ITerm[i] = ((float)Reading) * 8;
				} else {
				ITerm[i] = Reading * 4;
			}
		}


	for (int L = 0; L < Loops; L++) {
		eSample = 0;
		for (int c = 0; c < 100; c++) {// 100 PI Calculations
			eSum = 0;
			for (int i = 0; i < 3; i++) {
				i2c_burst_read(drv_data->i2c, cfg->i2c_addr,ReadAddress + (i * 2), &Data, 2);
				Reading = Data;
				if ((ReadAddress == 0x3B)&&(i == 2)) Reading -= 16384;	//remove Gravity
				Error = -Reading;
				eSum += abs(Reading); // Need to change this function as well this is an arduino function
				PTerm = kP * Error;
				ITerm[i] += (Error * 0.001) * kI;				// Integral term 1000 Calculations a second = 0.001
				if(SaveAddress != 0x13){
					Data = round((PTerm + ITerm[i] ) / 8);		//Compute PID Output
					Data = ((Data)&0xFFFE) |BitZero[i];			// Insert Bit0 Saved at beginning
				} else Data = round((PTerm + ITerm[i] ) / 4);	//Compute PID Output
				i2c_burst_write(drv_data->i2c, cfg->i2c_addr,SaveAddress + (i * shift),&Data, 2);
			}
			if((c == 99) && eSum > 1000){						// Error is still to great to continue 
				c = 0;
				// Serial.write('*');
			}
			if((eSum * ((ReadAddress == 0x3B)?.05: 1)) < 5) eSample++;	// Successfully found offsets prepare to  advance
			if((eSum < 100) && (c > 10) && (eSample >= 10)) break;		// Advance to next Loop
			delay(1);
		}
		// Serial.write('.');
		kP *= .75;
		kI *= .75;
		for (int i = 0; i < 3; i++){
			if(SaveAddress != 0x13) {
				Data = round((ITerm[i] ) / 8);		//Compute PID Output
				Data = ((Data)&0xFFFE) |BitZero[i];	// Insert Bit0 Saved at beginning
			} else Data = round((ITerm[i]) / 4);
			i2c_burst_write(drv_data->i2c, cfg->i2c_addr,SaveAddress + (i * shift),&Data, 2);
		}
	}


	resetFIFO(drv_data, cfg);
	resetDMP();

}


// USER_CTRL register (DMP function)

/**
 * 
*/
void resetDMP(struct mpu6050_data *drv_data, const struct mpu6050_config *cfg) {
	if(i2c_reg_update_byte(drv_data->i2c, cfg->i2c_addr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_RESET_BIT, true) < 0){
		return -EIO;
	}
}

/** Reset the FIFO.
 * This bit resets the FIFO buffer when set to 1 while FIFO_EN equals 0. This
 * bit automatically clears to 0 after the reset has been triggered.
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_RESET_BIT
 */
void resetFIFO(struct mpu6050_data *drv_data, const struct mpu6050_config *cfg) {
	if(i2c_reg_update_byte(drv_data->i2c, cfg->i2c_addr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, true) < 0){
		return -EIO;
	}
}

/**
  @brief      Fully calibrate Gyro from ZERO in about 6-7 Loops 600-700 readings
*/
int GyroCalibrate(int8_t Loops){
	double kP = 0.3;
	double kI = 90;
	float x;
	x = (100 - map(Loops, 1, 5, 20, 0)) * .01;
	kP *= x;
	kI *= x;

	PID( 0x43,  kP, kI,  Loops);
	
}


/**
  @brief      Fully calibrate Accel from ZERO in about 6-7 Loops 600-700 readings
*/
int AccelCalibrate(int8_t Loops){

	float kP = 0.3;
	float kI = 20;
	float x;
	x = (100 - map(Loops, 1, 5, 20, 0)) * .01;  //Need to write this map function as well
	kP *= x;
	kI *= x;
	PID( 0x3B, kP, kI,  Loops);
	

}



/**
 * 
*/
int setDMPEnabled(struct mpu6050_data *drv_data, const struct mpu6050_config *cfg){
	
	// This function OR's the old value and the new value stored in the register. 	

	if (i2c_reg_update_byte(drv_data->i2c, cfg->i2c_addr, MPU6050_RA_ZG_OFFS_USRH,	MPU6050_USERCTRL_DMP_EN_BIT, true) < 0) {
			LOG_ERR("Failed to write Gyroscope Z axis offset.");
			return -EIO;
		}

}




// PWR_MGMT_1 register

/** Trigger a full device reset.
 * A small delay of ~50ms may be desirable after triggering a reset.
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_DEVICE_RESET_BIT
 */
int reset(struct mpu6050_data *drv_data, const struct mpu6050_config *cfg){
	
	if (i2c_reg_update_byte(drv_data->i2c, cfg->i2c_addr, MPU6050_RA_USER_CTRL,	MPU6050_USERCTRL_SIG_COND_RESET_BIT, true) < 0) {
			LOG_ERR("Failed to write Gyroscope Z axis offset.");
			return -EIO;
		}
	
}


/** Set sleep mode status.
 * @param enabled New sleep mode enabled status
 * @see getSleepEnabled()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
void setSleepEnabled(struct mpu6050_data *drv_data, const struct mpu6050_config *cfg, bool enable) {

	
	if (i2c_reg_update_byte(drv_data->i2c, cfg->i2c_addr, MPU6050_RA_PWR_MGMT_1, MPU6050_USERCTRL_SIG_COND_RESET_BIT, enable) < 0) {
			LOG_ERR("Failed to write Gyroscope Z axis offset.");
			return -EIO;
		}
}


/** Set the I2C address of the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param address New address for specified slave
 * @see getSlaveAddress()
 * @see MPU6050_RA_I2C_SLV0_ADDR
 */
void setSlaveAddress(uint8_t num, uint8_t address) {
    if (num > 3) return;
	if(i2c_reg_write_byte(drv_data->i2c, cfg->i2c_addr, MPU6050_RA_I2C_SLV0_ADDR + num*3, address) < 0){
		LOG_ERR("Failed to write Gyroscope Z axis offset.");
		return -EIO;
	}	
	
    
}


/** Set I2C Master Mode enabled status.
 * @param enabled New I2C Master Mode enabled status
 * @see getI2CMasterModeEnabled()
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_I2C_MST_EN_BIT
 */
void setI2CMasterModeEnabled(bool enabled) {

	if (i2c_reg_update_byte(drv_data->i2c, cfg->i2c_addr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enable) < 0) {
			LOG_ERR("Failed to write Gyroscope Z axis offset.");
			return -EIO;
	}
   
}

/** Reset the I2C Master.
 * This bit resets the I2C Master when set to 1 while I2C_MST_EN equals 0.
 * This bit automatically clears to 0 after the reset has been triggered.
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_I2C_MST_RESET_BIT
 */
void resetI2CMaster() {

	if (i2c_reg_update_byte(drv_data->i2c, cfg->i2c_addr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_RESET_BIT, enable) < 0) {
			LOG_ERR("Failed to write Gyroscope Z axis offset.");
			return -EIO;
	}
   
}

/** Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 * @param source New clock source setting
 * @see getClockSource()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CLKSEL_BIT
 * @see MPU6050_PWR1_CLKSEL_LENGTH
 */
void setClockSource(uint8_t source) {

	if (i2c_reg_update_byte(drv_data->i2c, cfg->i2c_addr, MPU6050_RA_PWR_MGMT_1, MPU6050_USERCTRL_I2C_MST_RESET_BIT, enable) < 0) {
			LOG_ERR("Failed to write Gyroscope Z axis offset.");
			return -EIO;
	}

    I2Cdev::writeBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

/**
 * 
*/
int dmpGetFifoPacketSize(){

	
}


/**
 * 
*/
int getFifoCount() {


}


/**
 * 
*/
int resetFifoCount(){


}


/**
 * 
*/
int getFifoBytes(){


}



/**
 * 
*/
int dmpGetQuaternion(){


}



/**
 * 
*/
int dmpGetEuler() {


}


/**
 * 
*/
int dmpGetGravity(){


}


/**
 * 
*/
int dmpGetYawPitchRoll(){


}


/**
 * 
*/
int dmpGetAccel(){


}


/**
 * 
*/
int linearAccel(){


}

/**
 * 
*/
int dmpGetLinearAccelInWorld(){


}



static struct mpu6050_data mpu6050_driver;

static const struct mpu6050_config mpu6050_cfg = {
	.i2c_label = DT_INST_BUS_LABEL(0),
	.i2c_addr = DT_INST_REG_ADDR(0),
#ifdef CONFIG_MPU6050_TRIGGER
	.int_pin = DT_INST_GPIO_PIN(0, int_gpios),
	.int_flags = DT_INST_GPIO_FLAGS(0, int_gpios),
	.int_label = DT_INST_GPIO_LABEL(0, int_gpios),
#endif /* CONFIG_MPU6050_TRIGGER */
};

DEVICE_DT_INST_DEFINE(0, mpu6050_init, device_pm_control_nop,
		    &mpu6050_driver, &mpu6050_cfg,
		    POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
		    &mpu6050_driver_api);
