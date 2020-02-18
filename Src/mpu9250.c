/*
 * mpu9250.c
 *
 *  Created on: Jan 12, 2020
 *      Author: Ivan
 */

/* Private includes ----------------------------------------------------------*/
#include <string.h>
#include <math.h>

#include "mpu9250.h"
#include "i2c.h"
#include "main.h"


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/



/* Private user code ---------------------------------------------------------*/

/* writes a byte to MPU9250 register given a register address and data */
static int MPU9250_writeRegister(uint8_t subAddress, uint8_t data) {
	HAL_StatusTypeDef retval;

	/*
	 DevAddress Target device address: The device 7 bits address value
	 in datasheet must be shifted to the left before calling the interface
	 */
	retval = HAL_I2C_Mem_Write(&hi2c1, MPU9250_I2C_ADDRESS << 1, subAddress, I2C_MEMADD_SIZE_8BIT, &data, 1, 1);
	if (retval != HAL_OK) {
		return -1;	// failure
	} else {
		return 1;	// success
	}
}


/* reads registers from MPU9250 given a starting register address, number of bytes, and a pointer to store data */
static int MPU9250_readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest) {
	HAL_StatusTypeDef retval;

	/*
	 DevAddress Target device address: The device 7 bits address value
	 in datasheet must be shifted to the left before calling the interface
	 */
	retval = HAL_I2C_Mem_Read(&hi2c1, MPU9250_I2C_ADDRESS << 1, subAddress, I2C_MEMADD_SIZE_8BIT, dest, count, 1);

	if (retval != HAL_OK) {
		return -1;	// failure
	} else {
		return 1;	// success
	}
}


/* writes a register to the AK8963 given a register address and data */
static int MPU9250_writeAK8963Register(uint8_t subAddress, uint8_t data) {

	// set slave 0 to the AK8963 and set for write
	if (MPU9250_writeRegister(I2C_SLV0_ADDR, AK8963_I2C_ADDR) < 0) {
		return -1;
	}
	// set the register to the desired AK8963 sub address
	if (MPU9250_writeRegister(I2C_SLV0_REG, subAddress) < 0) {
		return -2;
	}
	// store the data for write
	if (MPU9250_writeRegister(I2C_SLV0_DO, data) < 0) {
		return -3;
	}
	// enable I2C and send 1 byte
	if (MPU9250_writeRegister(I2C_SLV0_CTRL, I2C_SLV0_EN | (uint8_t) 1) < 0) {
		return -4;
	}

	return 1;	// success
}


/* reads registers from the AK8963 */
static int MPU9250_readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t *dest) {

	// set slave 0 to the AK8963 and set for read
	if (MPU9250_writeRegister(I2C_SLV0_ADDR, AK8963_I2C_ADDR | I2C_READ_FLAG)
			< 0) {
		return -1;
	}
	// set the register to the desired AK8963 sub address
	if (MPU9250_writeRegister(I2C_SLV0_REG, subAddress) < 0) {
		return -2;
	}
	// enable I2C and request the bytes
	if (MPU9250_writeRegister(I2C_SLV0_CTRL, I2C_SLV0_EN | count) < 0) {
		return -3;
	}

	HAL_Delay(1); // takes some time for these registers to fill

	// read the bytes off the MPU9250 EXT_SENS_DATA registers
	return MPU9250_readRegisters(EXT_SENS_DATA_00, count, dest);
}


/* gets the MPU9250 WHO_AM_I register value, expected to be 0x71 */
static uint8_t MPU9250_whoAmI(void) {
	uint8_t value;

	// read the WHO AM I register
	if (MPU9250_readRegisters(WHO_AM_I, 1, &value) < 0) {
		return -1;
	}
	// return the register value
	return value;
}


/* gets the AK8963 WHO_AM_I register value, expected to be 0x48 */
static uint8_t MPU9250_whoAmIAK8963(void) {
	uint8_t value;

	// read the WHO AM I register
	if (MPU9250_readAK8963Registers(AK8963_WHO_AM_I, 1, &value) < 0) {
		return -1;
	}
	// return the register value
	return value;
}


/* transforms val from x_start and x_end to y */
static float map(float val, float x_start, float x_end, float y_start, float y_end) {
	float ret = y_start;

	if (val >= x_start && val <= x_end && x_start != x_end) {
		ret = y_start + (val - x_start) * (y_end - y_start) / (x_end - x_start);
	}

	return ret;
}


/* starts communication with the MPU-9250 */
int MPU9250_begin(struct MPU9250_Handle_s *MPU9250_Handle) {

	// Init struct constants
	// i2c
	MPU9250_Handle->_i2cRate = 400000; // 400 kHz

	// scale factors
	MPU9250_Handle->_tempScale = 333.87f;
	MPU9250_Handle->_tempOffset = 21.0f;

	// gyro bias estimation
	MPU9250_Handle->_numSamples = 100;
	MPU9250_Handle->_gxb = 0.0f;
	MPU9250_Handle->_gyb = 0.0f;
	MPU9250_Handle->_gzb = 0.0f;

	// accel bias and scale factor estimation
	MPU9250_Handle->_axb = 0.0f;
	MPU9250_Handle->_ayb = 0.0f;
	MPU9250_Handle->_azb = 0.0f;
	MPU9250_Handle->_axs = 1.0f;
	MPU9250_Handle->_ays = 1.0f;
	MPU9250_Handle->_azs = 1.0f;

	// magnetometer bias and scale factor estimation
	MPU9250_Handle->_maxCounts = 1000;
	MPU9250_Handle->_deltaThresh = 0.3f;
	MPU9250_Handle->_coeff = 8;
	MPU9250_Handle->_hxs = 1.0f;
	MPU9250_Handle->_hys = 1.0f;
	MPU9250_Handle->_hzs = 1.0f;
	MPU9250_Handle->_hxb = 0.0f;
	MPU9250_Handle->_hyb = 0.0f;
	MPU9250_Handle->_hzb = 0.0f;

	// transformation matrix
	/* transform the accel and gyro axes to match the magnetometer axes */
	MPU9250_Handle->_tX[0] = 0;
	MPU9250_Handle->_tX[1] = 1;
	MPU9250_Handle->_tX[2] = 0;

	MPU9250_Handle->_tY[0] = 1;
	MPU9250_Handle->_tY[1] = 0;
	MPU9250_Handle->_tY[2] = 0;

	MPU9250_Handle->_tZ[0] =  0;
	MPU9250_Handle->_tZ[1] =  0;
	MPU9250_Handle->_tZ[2] = -1;

	// constants
	MPU9250_Handle->_G = 9.807f;
	MPU9250_Handle->_d2r = 3.14159265359f / 180.0f;

	// select clock source to gyro
	if (MPU9250_writeRegister(PWR_MGMNT_1, CLOCK_SEL_PLL) < 0) {
		return -1;
	}

	// enable I2C master mode
	if (MPU9250_writeRegister(USER_CTRL, I2C_MST_EN) < 0) {
		return -2;
	}

	// set the I2C bus speed to 400 kHz
	if (MPU9250_writeRegister(I2C_MST_CTRL, I2C_MST_CLK) < 0) {
		return -3;
	}

	// set AK8963 to Power Down
	MPU9250_writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);

	// reset the MPU9250
	MPU9250_writeRegister(PWR_MGMNT_1, PWR_RESET);

	// wait for MPU-9250 to come back up
	HAL_Delay(1);

	// reset the AK8963
	MPU9250_writeAK8963Register(AK8963_CNTL2, AK8963_RESET);

	// select clock source to gyro
	if (MPU9250_writeRegister(PWR_MGMNT_1, CLOCK_SEL_PLL) < 0) {
		return -4;
	}

	// check the WHO AM I byte, expected value is 0x71 (decimal 113) or 0x73 (decimal 115)
	if ((MPU9250_whoAmI() != 113) && (MPU9250_whoAmI() != 115)) {
		return -5;
	}

	// enable accelerometer and gyro
	if (MPU9250_writeRegister(PWR_MGMNT_2, SEN_ENABLE) < 0) {
		return -6;
	}

	// setting accel range to 16G as default
	MPU9250_Handle->_accelScale = MPU9250_Handle->_G * 16.0f / 32767.5f; // setting the accel scale to 16G
	MPU9250_Handle->_accelRange = ACCEL_RANGE_16G;
	if (MPU9250_writeRegister(ACCEL_CONFIG, ACCEL_FS_SEL_16G) < 0) {
		return -7;
	}

	// setting the gyro range to 2000DPS as default
	MPU9250_Handle->_gyroScale = 2000.0f / 32767.5f * MPU9250_Handle->_d2r; // setting the gyro scale to 2000DPS
	MPU9250_Handle->_gyroRange = GYRO_RANGE_2000DPS;
	if (MPU9250_writeRegister(GYRO_CONFIG, GYRO_FS_SEL_2000DPS) < 0) {
		return -8;
	}

	// setting bandwidth to 184Hz as default
	MPU9250_Handle->_bandwidth = DLPF_BANDWIDTH_184HZ;
	if (MPU9250_writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_184) < 0) {
		return -9;
	}

	// setting gyro bandwidth to 184Hz
	if (MPU9250_writeRegister(CONFIG, GYRO_DLPF_184) < 0) {
		return -10;
	}

	// setting the sample rate divider to 0 as default
	MPU9250_Handle->_srd = 0;
	if (MPU9250_writeRegister(SMPDIV, 0x00) < 0) {
		return -11;
	}


	// enable I2C master mode
	if (MPU9250_writeRegister(USER_CTRL, I2C_MST_EN) < 0) {
		return -12;
	}

	// set the I2C bus speed to 400 kHz
	if (MPU9250_writeRegister(I2C_MST_CTRL, I2C_MST_CLK) < 0) {
		return -13;
	}

	// check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
	if (MPU9250_whoAmIAK8963() != 72) {
		return -14;
	}

	/* get the magnetometer calibration */
	// set AK8963 to Power Down
	if (MPU9250_writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN) < 0) {
		return -15;
	}

	HAL_Delay(100); // long wait between AK8963 mode changes

	// set AK8963 to FUSE ROM access
	if (MPU9250_writeAK8963Register(AK8963_CNTL1, AK8963_FUSE_ROM) < 0) {
		return -16;

	}

	HAL_Delay(100); // long wait between AK8963 mode changes

	// read the AK8963 ASA registers and compute magnetometer scale factors
	MPU9250_readAK8963Registers(AK8963_ASA, 3, MPU9250_Handle->_buffer);

	MPU9250_Handle->_magScaleX = ((((float) MPU9250_Handle->_buffer[0]) - 128.0f)
									/ (256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
	MPU9250_Handle->_magScaleY = ((((float) MPU9250_Handle->_buffer[1]) - 128.0f)
									/ (256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
	MPU9250_Handle->_magScaleZ = ((((float) MPU9250_Handle->_buffer[2]) - 128.0f)
									/ (256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla

	// set AK8963 to Power Down
	if (MPU9250_writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN) < 0) {
		return -17;
	}

	HAL_Delay(100); // long wait between AK8963 mode changes

	// set AK8963 to 16 bit resolution, 100 Hz update rate
	if (MPU9250_writeAK8963Register(AK8963_CNTL1, AK8963_CNT_MEAS2) < 0) {
		return -18;
	}

	HAL_Delay(100); // long wait between AK8963 mode changes

	// select clock source to gyro
	if (MPU9250_writeRegister(PWR_MGMNT_1, CLOCK_SEL_PLL) < 0) {
		return -19;
	}

	// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
	MPU9250_readAK8963Registers(AK8963_HXL, 7, MPU9250_Handle->_buffer);

	// estimate gyro bias
	if (MPU9250_calibrateGyro(MPU9250_Handle) < 0) {
		return -20;
	}

	// successful init, return 1
	return 1;
}

/* sets the accelerometer full scale range to values other than default */
int MPU9250_setAccelRange(AccelRange_t range, struct MPU9250_Handle_s *MPU9250_Handle) {
	switch (range) {
		case ACCEL_RANGE_2G: {
			// setting the accel range to 2G
			if (MPU9250_writeRegister(ACCEL_CONFIG, ACCEL_FS_SEL_2G) < 0) {
				return -1;
			}
			MPU9250_Handle->_accelScale = MPU9250_Handle->_G * 2.0f / 32767.5f; // setting the accel scale to 2G
			break;
		}
		case ACCEL_RANGE_4G: {
			// setting the accel range to 4G
			if (MPU9250_writeRegister(ACCEL_CONFIG, ACCEL_FS_SEL_4G) < 0) {
				return -1;
			}
			MPU9250_Handle->_accelScale = MPU9250_Handle->_G * 4.0f / 32767.5f; // setting the accel scale to 4G
			break;
		}
		case ACCEL_RANGE_8G: {
			// setting the accel range to 8G
			if (MPU9250_writeRegister(ACCEL_CONFIG, ACCEL_FS_SEL_8G) < 0) {
				return -1;
			}
			MPU9250_Handle->_accelScale = MPU9250_Handle->_G * 8.0f / 32767.5f; // setting the accel scale to 8G
			break;
		}
		case ACCEL_RANGE_16G: {
			// setting the accel range to 16G
			if (MPU9250_writeRegister(ACCEL_CONFIG, ACCEL_FS_SEL_16G) < 0) {
				return -1;
			}
			MPU9250_Handle->_accelScale = MPU9250_Handle->_G * 16.0f / 32767.5f; // setting the accel scale to 16G
			break;
		}
	}
	
	MPU9250_Handle->_accelRange = range;

	return 1;
}

/* sets the gyro full scale range to values other than default */
int MPU9250_setGyroRange(GyroRange_t range, struct MPU9250_Handle_s *MPU9250_Handle) {
	switch (range) {
		case GYRO_RANGE_250DPS: {
			// setting the gyro range to 250DPS
			if (MPU9250_writeRegister(GYRO_CONFIG, GYRO_FS_SEL_250DPS) < 0) {
				return -1;
			}
			MPU9250_Handle->_gyroScale = 250.0f / 32767.5f * MPU9250_Handle->_d2r; // setting the gyro scale to 250DPS
			break;
		}
		case GYRO_RANGE_500DPS: {
			// setting the gyro range to 500DPS
			if (MPU9250_writeRegister(GYRO_CONFIG, GYRO_FS_SEL_500DPS) < 0) {
				return -1;
			}
			MPU9250_Handle->_gyroScale = 500.0f / 32767.5f * MPU9250_Handle->_d2r; // setting the gyro scale to 500DPS
			break;
		}
		case GYRO_RANGE_1000DPS: {
			// setting the gyro range to 1000DPS
			if (MPU9250_writeRegister(GYRO_CONFIG, GYRO_FS_SEL_1000DPS) < 0) {
				return -1;
			}
			MPU9250_Handle->_gyroScale = 1000.0f / 32767.5f * MPU9250_Handle->_d2r; // setting the gyro scale to 1000DPS
			break;
		}
		case GYRO_RANGE_2000DPS: {
			// setting the gyro range to 2000DPS
			if (MPU9250_writeRegister(GYRO_CONFIG, GYRO_FS_SEL_2000DPS) < 0) {
				return -1;
			}
			MPU9250_Handle->_gyroScale = 2000.0f / 32767.5f * MPU9250_Handle->_d2r; // setting the gyro scale to 2000DPS
			break;
		}
	}
	
	MPU9250_Handle->_gyroRange = range;
	return 1;
}

/* sets the DLPF bandwidth to values other than default */
int MPU9250_setDlpfBandwidth(DlpfBandwidth_t bandwidth, struct MPU9250_Handle_s *MPU9250_Handle) {
	switch (bandwidth) {
		case DLPF_BANDWIDTH_184HZ: {
			if (MPU9250_writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_184) < 0) { // setting accel bandwidth to 184Hz
				return -1;
			}
			if (MPU9250_writeRegister(CONFIG, GYRO_DLPF_184) < 0) { // setting gyro bandwidth to 184Hz
				return -2;
			}
			break;
		}
		case DLPF_BANDWIDTH_92HZ: {
			if (MPU9250_writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_92) < 0) { // setting accel bandwidth to 92Hz
				return -1;
			}
			if (MPU9250_writeRegister(CONFIG, GYRO_DLPF_92) < 0) { // setting gyro bandwidth to 92Hz
				return -2;
			}
			break;
		}
		case DLPF_BANDWIDTH_41HZ: {
			if (MPU9250_writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_41) < 0) { // setting accel bandwidth to 41Hz
				return -1;
			}
			if (MPU9250_writeRegister(CONFIG, GYRO_DLPF_41) < 0) { // setting gyro bandwidth to 41Hz
				return -2;
			}
			break;
		}
		case DLPF_BANDWIDTH_20HZ: {
			if (MPU9250_writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_20) < 0) { // setting accel bandwidth to 20Hz
				return -1;
			}
			if (MPU9250_writeRegister(CONFIG, GYRO_DLPF_20) < 0) { // setting gyro bandwidth to 20Hz
				return -2;
			}
			break;
		}
		case DLPF_BANDWIDTH_10HZ: {
			if (MPU9250_writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_10) < 0) { // setting accel bandwidth to 10Hz
				return -1;
			}
			if (MPU9250_writeRegister(CONFIG, GYRO_DLPF_10) < 0) { // setting gyro bandwidth to 10Hz
				return -2;
			}
			break;
		}
		case DLPF_BANDWIDTH_5HZ: {
			if (MPU9250_writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_5) < 0) { // setting accel bandwidth to 5Hz
				return -1;
			}
			if (MPU9250_writeRegister(CONFIG, GYRO_DLPF_5) < 0) { // setting gyro bandwidth to 5Hz
				return -2;
			}
			break;
		}
	}

	MPU9250_Handle->_bandwidth = bandwidth;
	return 1;
}

/* sets the sample rate divider to values other than default */
int MPU9250_setSrd(uint8_t srd, struct MPU9250_Handle_s *MPU9250_Handle) {
	/* setting the sample rate divider to 19 to facilitate setting up magnetometer */
	if (MPU9250_writeRegister(SMPDIV, 19) < 0) { // setting the sample rate divider
		return -1;
	}

	if (srd > 9) {
		// set AK8963 to Power Down
		if (MPU9250_writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN) < 0) {
			return -2;
		}
		HAL_Delay(100); // long wait between AK8963 mode changes
		// set AK8963 to 16 bit resolution, 8 Hz update rate
		if (MPU9250_writeAK8963Register(AK8963_CNTL1, AK8963_CNT_MEAS1) < 0) {
			return -3;
		}
		HAL_Delay(100); // long wait between AK8963 mode changes
		// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
		MPU9250_readAK8963Registers(AK8963_HXL, 7, MPU9250_Handle->_buffer);
	} else {
		// set AK8963 to Power Down
		if (MPU9250_writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN) < 0) {
			return -2;
		}
		HAL_Delay(100); // long wait between AK8963 mode changes
		// set AK8963 to 16 bit resolution, 100 Hz update rate
		if (MPU9250_writeAK8963Register(AK8963_CNTL1, AK8963_CNT_MEAS2) < 0) {
			return -3;
		}
		HAL_Delay(100); // long wait between AK8963 mode changes
		// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
		MPU9250_readAK8963Registers(AK8963_HXL, 7, MPU9250_Handle->_buffer);
	}
	/* setting the sample rate divider */
	if (MPU9250_writeRegister(SMPDIV, srd) < 0) { // setting the sample rate divider
		return -4;
	}
	
	MPU9250_Handle->_srd = srd;
	return 1;
}

/* enables the data ready interrupt */
int MPU9250_enableDataReadyInterrupt(void) {
	/* setting the interrupt */
	if (MPU9250_writeRegister(INT_PIN_CFG, INT_PULSE_50US) < 0) { // setup interrupt, 50 us pulse
		return -1;
	}
	if (MPU9250_writeRegister(INT_ENABLE, INT_RAW_RDY_EN) < 0) { // set to data ready
		return -2;
	}

	return 1;
}

/* disables the data ready interrupt */
int MPU9250_disableDataReadyInterrupt(void) {
	if (MPU9250_writeRegister(INT_ENABLE, INT_DISABLE) < 0) { // disable interrupt
		return -1;
	}
	
	return 1;
}

/* configures and enables wake on motion, low power mode */
int MPU9250_enableWakeOnMotion(float womThresh_mg, LpAccelOdr_t odr, struct MPU9250_Handle_s *MPU9250_Handle) {
	// set AK8963 to Power Down
	MPU9250_writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);
	// reset the MPU9250
	MPU9250_writeRegister(PWR_MGMNT_1, PWR_RESET);
	// wait for MPU-9250 to come back up
	HAL_Delay(1);
	if (MPU9250_writeRegister(PWR_MGMNT_1, 0x00) < 0) { // cycle 0, sleep 0, standby 0
		return -1;
	}
	if (MPU9250_writeRegister(PWR_MGMNT_2, DIS_GYRO) < 0) { // disable gyro measurements
		return -2;
	}
	if (MPU9250_writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_184) < 0) { // setting accel bandwidth to 184Hz
		return -3;
	}
	if (MPU9250_writeRegister(INT_ENABLE, INT_WOM_EN) < 0) { // enabling interrupt to wake on motion
		return -4;
	}
	if (MPU9250_writeRegister(MOT_DETECT_CTRL,
			(ACCEL_INTEL_EN | ACCEL_INTEL_MODE)) < 0) { // enabling accel hardware intelligence
		return -5;
	}

	MPU9250_Handle->_womThreshold = map(womThresh_mg, 0, 1020, 0, 255);
	if (MPU9250_writeRegister(WOM_THR, MPU9250_Handle->_womThreshold) < 0) { // setting wake on motion threshold
		return -6;
	}
	if (MPU9250_writeRegister(LP_ACCEL_ODR, (uint8_t) odr) < 0) { // set frequency of wakeup
		return -7;
	}
	if (MPU9250_writeRegister(PWR_MGMNT_1, PWR_CYCLE) < 0) { // switch to accel low power mode
		return -8;
	}
	return 1;
}

/* configures and enables the FIFO buffer  */
int MPU9250FIFO_enableFifo(uint32_t accel, uint32_t gyro, uint32_t mag, uint32_t temp, 
							struct MPU9250_Handle_s *MPU9250_Handle) {

	// convert input parameters to 0 or 1 if they are not
	accel = (accel > 0) ? 1 : 0;
	gyro  = (gyro  > 0) ? 1 : 0;
	mag   = (mag   > 0) ? 1 : 0;
	temp  = (temp  > 0) ? 1 : 0;

	if (MPU9250_writeRegister(USER_CTRL, (0x40 | I2C_MST_EN)) < 0) {
		return -1;
	}

	if (MPU9250_writeRegister(FIFO_EN,
			(accel * FIFO_ACCEL)	|
			(gyro  * FIFO_GYRO) 	|
			(mag   * FIFO_MAG) 		|
			(temp  * FIFO_TEMP)	) < 0) {

		return -2;
	}

	MPU9250_Handle->_enFifoAccel = accel;
	MPU9250_Handle->_enFifoGyro = gyro;
	MPU9250_Handle->_enFifoMag = mag;
	MPU9250_Handle->_enFifoTemp = temp;

	MPU9250_Handle->_fifoFrameSize = accel * 6 + gyro * 6 + mag * 7 + temp * 2;

	return 1;
}

/* reads the most current data from MPU9250 and stores in buffer */
int MPU9250_readSensor(struct MPU9250_Handle_s *MPU9250_Handle) {

	// grab the data from the MPU9250
	if (MPU9250_readRegisters(ACCEL_OUT, 21, MPU9250_Handle->_buffer) < 0) {
		return -1;
	}

	// combine into 16 bit values
	MPU9250_Handle->_axcounts = (((int16_t) MPU9250_Handle->_buffer[0])  << 8) | MPU9250_Handle->_buffer[1];
	MPU9250_Handle->_aycounts = (((int16_t) MPU9250_Handle->_buffer[2])  << 8) | MPU9250_Handle->_buffer[3];
	MPU9250_Handle->_azcounts = (((int16_t) MPU9250_Handle->_buffer[4])  << 8) | MPU9250_Handle->_buffer[5];
	MPU9250_Handle->_tcounts  = (((int16_t) MPU9250_Handle->_buffer[6])  << 8) | MPU9250_Handle->_buffer[7];
	MPU9250_Handle->_gxcounts = (((int16_t) MPU9250_Handle->_buffer[8])  << 8) | MPU9250_Handle->_buffer[9];
	MPU9250_Handle->_gycounts = (((int16_t) MPU9250_Handle->_buffer[10]) << 8) | MPU9250_Handle->_buffer[11];
	MPU9250_Handle->_gzcounts = (((int16_t) MPU9250_Handle->_buffer[12]) << 8) | MPU9250_Handle->_buffer[13];
	MPU9250_Handle->_hxcounts = (((int16_t) MPU9250_Handle->_buffer[15]) << 8) | MPU9250_Handle->_buffer[14];
	MPU9250_Handle->_hycounts = (((int16_t) MPU9250_Handle->_buffer[17]) << 8) | MPU9250_Handle->_buffer[16];
	MPU9250_Handle->_hzcounts = (((int16_t) MPU9250_Handle->_buffer[19]) << 8) | MPU9250_Handle->_buffer[18];

	// transform and convert to float values
	MPU9250_Handle->_ax = (((float) (MPU9250_Handle->_tX[0] * MPU9250_Handle->_axcounts + MPU9250_Handle->_tX[1] * MPU9250_Handle->_aycounts + MPU9250_Handle->_tX[2] * MPU9250_Handle->_azcounts) * MPU9250_Handle->_accelScale) - MPU9250_Handle->_axb) * MPU9250_Handle->_axs;
	MPU9250_Handle->_ay = (((float) (MPU9250_Handle->_tY[0] * MPU9250_Handle->_axcounts + MPU9250_Handle->_tY[1] * MPU9250_Handle->_aycounts + MPU9250_Handle->_tY[2] * MPU9250_Handle->_azcounts) * MPU9250_Handle->_accelScale) - MPU9250_Handle->_ayb) * MPU9250_Handle->_ays;
	MPU9250_Handle->_az = (((float) (MPU9250_Handle->_tZ[0] * MPU9250_Handle->_axcounts + MPU9250_Handle->_tZ[1] * MPU9250_Handle->_aycounts + MPU9250_Handle->_tZ[2] * MPU9250_Handle->_azcounts) * MPU9250_Handle->_accelScale) - MPU9250_Handle->_azb) * MPU9250_Handle->_azs;

	MPU9250_Handle->_gx = ((float) (MPU9250_Handle->_tX[0] * MPU9250_Handle->_gxcounts + MPU9250_Handle->_tX[1] * MPU9250_Handle->_gycounts + MPU9250_Handle->_tX[2] * MPU9250_Handle->_gzcounts) * MPU9250_Handle->_gyroScale) - MPU9250_Handle->_gxb;
	MPU9250_Handle->_gy = ((float) (MPU9250_Handle->_tY[0] * MPU9250_Handle->_gxcounts + MPU9250_Handle->_tY[1] * MPU9250_Handle->_gycounts + MPU9250_Handle->_tY[2] * MPU9250_Handle->_gzcounts) * MPU9250_Handle->_gyroScale) - MPU9250_Handle->_gyb;
	MPU9250_Handle->_gz = ((float) (MPU9250_Handle->_tZ[0] * MPU9250_Handle->_gxcounts + MPU9250_Handle->_tZ[1] * MPU9250_Handle->_gycounts + MPU9250_Handle->_tZ[2] * MPU9250_Handle->_gzcounts) * MPU9250_Handle->_gyroScale) - MPU9250_Handle->_gzb;

	MPU9250_Handle->_hx = (((float) (MPU9250_Handle->_hxcounts) * MPU9250_Handle->_magScaleX) - MPU9250_Handle->_hxb) * MPU9250_Handle->_hxs;
	MPU9250_Handle->_hy = (((float) (MPU9250_Handle->_hycounts) * MPU9250_Handle->_magScaleY) - MPU9250_Handle->_hyb) * MPU9250_Handle->_hys;
	MPU9250_Handle->_hz = (((float) (MPU9250_Handle->_hzcounts) * MPU9250_Handle->_magScaleZ) - MPU9250_Handle->_hzb) * MPU9250_Handle->_hzs;

	MPU9250_Handle->_t  = ((((float) MPU9250_Handle->_tcounts) - MPU9250_Handle->_tempOffset) / MPU9250_Handle->_tempScale) + MPU9250_Handle->_tempOffset;

	return 1;
}

/* returns the accelerometer measurement in the x direction, m/s/s */
float MPU9250_getAccelX_mss(const struct MPU9250_Handle_s *MPU9250_Handle) {
	return MPU9250_Handle->_ax;
}

/* returns the accelerometer measurement in the y direction, m/s/s */
float MPU9250_getAccelY_mss(const struct MPU9250_Handle_s *MPU9250_Handle) {
	return MPU9250_Handle->_ay;
}

/* returns the accelerometer measurement in the z direction, m/s/s */
float MPU9250_getAccelZ_mss(const struct MPU9250_Handle_s *MPU9250_Handle) {
	return MPU9250_Handle->_az;
}

/* returns the gyroscope measurement in the x direction, rad/s */
float MPU9250_getGyroX_rads(const struct MPU9250_Handle_s *MPU9250_Handle) {
	return MPU9250_Handle->_gx;
}

/* returns the gyroscope measurement in the y direction, rad/s */
float MPU9250_getGyroY_rads(const struct MPU9250_Handle_s *MPU9250_Handle) {
	return MPU9250_Handle->_gy;
}

/* returns the gyroscope measurement in the z direction, rad/s */
float MPU9250_getGyroZ_rads(const struct MPU9250_Handle_s *MPU9250_Handle) {
	return MPU9250_Handle->_gz;
}

/* returns the magnetometer measurement in the x direction, uT */
float MPU9250_getMagX_uT(const struct MPU9250_Handle_s *MPU9250_Handle) {
	return MPU9250_Handle->_hx;
}

/* returns the magnetometer measurement in the y direction, uT */
float MPU9250_getMagY_uT(const struct MPU9250_Handle_s *MPU9250_Handle) {
	return MPU9250_Handle->_hy;
}

/* returns the magnetometer measurement in the z direction, uT */
float MPU9250_getMagZ_uT(const struct MPU9250_Handle_s *MPU9250_Handle) {
	return MPU9250_Handle->_hz;
}

/* returns the die temperature, C */
float MPU9250_getTemperature_C(const struct MPU9250_Handle_s *MPU9250_Handle) {
	return MPU9250_Handle->_t;
}

/* reads data from the MPU9250 FIFO and stores in buffer */
int MPU9250FIFO_readFifo(struct MPU9250_Handle_s *MPU9250_Handle) {

	// get the fifo size
	MPU9250_readRegisters(FIFO_COUNT, 2, MPU9250_Handle->_buffer);
	MPU9250_Handle->_fifoSize = (((uint16_t) (MPU9250_Handle->_buffer[0] & 0x0F)) << 8) + (((uint16_t) MPU9250_Handle->_buffer[1]));

	// read and parse the buffer
	for (uint32_t i = 0; i < MPU9250_Handle->_fifoSize / MPU9250_Handle->_fifoFrameSize; i++) {
		// grab the data from the MPU9250
		if (MPU9250_readRegisters(FIFO_READ, MPU9250_Handle->_fifoFrameSize, MPU9250_Handle->_buffer) < 0) {
			return -1;
		}
		if (MPU9250_Handle->_enFifoAccel) {
			// combine into 16 bit values
			MPU9250_Handle->_axcounts = (((int16_t) MPU9250_Handle->_buffer[0]) << 8) | MPU9250_Handle->_buffer[1];
			MPU9250_Handle->_aycounts = (((int16_t) MPU9250_Handle->_buffer[2]) << 8) | MPU9250_Handle->_buffer[3];
			MPU9250_Handle->_azcounts = (((int16_t) MPU9250_Handle->_buffer[4]) << 8) | MPU9250_Handle->_buffer[5];
			
			// transform and convert to float values
			MPU9250_Handle->_axFifo[i] = (((float) (MPU9250_Handle->_tX[0] * MPU9250_Handle->_axcounts + MPU9250_Handle->_tX[1] * MPU9250_Handle->_aycounts + MPU9250_Handle->_tX[2] * MPU9250_Handle->_azcounts) * MPU9250_Handle->_accelScale) - MPU9250_Handle->_axb) * MPU9250_Handle->_axs;
			MPU9250_Handle->_ayFifo[i] = (((float) (MPU9250_Handle->_tY[0] * MPU9250_Handle->_axcounts + MPU9250_Handle->_tY[1] * MPU9250_Handle->_aycounts + MPU9250_Handle->_tY[2] * MPU9250_Handle->_azcounts) * MPU9250_Handle->_accelScale) - MPU9250_Handle->_ayb) * MPU9250_Handle->_ays;
			MPU9250_Handle->_azFifo[i] = (((float) (MPU9250_Handle->_tZ[0] * MPU9250_Handle->_axcounts + MPU9250_Handle->_tZ[1] * MPU9250_Handle->_aycounts + MPU9250_Handle->_tZ[2] * MPU9250_Handle->_azcounts) * MPU9250_Handle->_accelScale) - MPU9250_Handle->_azb) * MPU9250_Handle->_azs;
			MPU9250_Handle->_aSize = MPU9250_Handle->_fifoSize / MPU9250_Handle->_fifoFrameSize;
		}
		if (MPU9250_Handle->_enFifoTemp) {
			// combine into 16 bit values
			MPU9250_Handle->_tcounts = (((int16_t) MPU9250_Handle->_buffer[0 + MPU9250_Handle->_enFifoAccel * 6]) << 8) | MPU9250_Handle->_buffer[1 + MPU9250_Handle->_enFifoAccel * 6];
			// transform and convert to float values
			MPU9250_Handle->_tFifo[i] = ((((float) MPU9250_Handle->_tcounts) - MPU9250_Handle->_tempOffset) / MPU9250_Handle->_tempScale) + MPU9250_Handle->_tempOffset;
			MPU9250_Handle->_tSize = MPU9250_Handle->_fifoSize / MPU9250_Handle->_fifoFrameSize;
		}
		if (MPU9250_Handle->_enFifoGyro) {
			// combine into 16 bit values
			MPU9250_Handle->_gxcounts = (((int16_t) MPU9250_Handle->_buffer[0 + MPU9250_Handle->_enFifoAccel * 6 + MPU9250_Handle->_enFifoTemp * 2]) << 8) | MPU9250_Handle->_buffer[1 + MPU9250_Handle->_enFifoAccel * 6 + MPU9250_Handle->_enFifoTemp * 2];
			MPU9250_Handle->_gycounts = (((int16_t) MPU9250_Handle->_buffer[2 + MPU9250_Handle->_enFifoAccel * 6 + MPU9250_Handle->_enFifoTemp * 2]) << 8) | MPU9250_Handle->_buffer[3 + MPU9250_Handle->_enFifoAccel * 6 + MPU9250_Handle->_enFifoTemp * 2];
			MPU9250_Handle->_gzcounts = (((int16_t) MPU9250_Handle->_buffer[4 + MPU9250_Handle->_enFifoAccel * 6 + MPU9250_Handle->_enFifoTemp * 2]) << 8) | MPU9250_Handle->_buffer[5 + MPU9250_Handle->_enFifoAccel * 6 + MPU9250_Handle->_enFifoTemp * 2];
			// transform and convert to float values
			MPU9250_Handle->_gxFifo[i] = ((float) (MPU9250_Handle->_tX[0] * MPU9250_Handle->_gxcounts + MPU9250_Handle->_tX[1] * MPU9250_Handle->_gycounts + MPU9250_Handle->_tX[2] * MPU9250_Handle->_gzcounts) * MPU9250_Handle->_gyroScale) - MPU9250_Handle->_gxb;
			MPU9250_Handle->_gyFifo[i] = ((float) (MPU9250_Handle->_tY[0] * MPU9250_Handle->_gxcounts + MPU9250_Handle->_tY[1] * MPU9250_Handle->_gycounts + MPU9250_Handle->_tY[2] * MPU9250_Handle->_gzcounts) * MPU9250_Handle->_gyroScale) - MPU9250_Handle->_gyb;
			MPU9250_Handle->_gzFifo[i] = ((float) (MPU9250_Handle->_tZ[0] * MPU9250_Handle->_gxcounts + MPU9250_Handle->_tZ[1] * MPU9250_Handle->_gycounts + MPU9250_Handle->_tZ[2] * MPU9250_Handle->_gzcounts) * MPU9250_Handle->_gyroScale) - MPU9250_Handle->_gzb;
			MPU9250_Handle->_gSize = MPU9250_Handle->_fifoSize / MPU9250_Handle->_fifoFrameSize;
		}
		if (MPU9250_Handle->_enFifoMag) {
			// combine into 16 bit values
			MPU9250_Handle->_hxcounts = (((int16_t) MPU9250_Handle->_buffer[1 + MPU9250_Handle->_enFifoAccel * 6
					+ MPU9250_Handle->_enFifoTemp * 2 + MPU9250_Handle->_enFifoGyro * 6]) << 8)
					| MPU9250_Handle->_buffer[0 + MPU9250_Handle->_enFifoAccel * 6 + MPU9250_Handle->_enFifoTemp * 2
							+ MPU9250_Handle->_enFifoGyro * 6];
			MPU9250_Handle->_hycounts = (((int16_t) MPU9250_Handle->_buffer[3 + MPU9250_Handle->_enFifoAccel * 6
					+ MPU9250_Handle->_enFifoTemp * 2 + MPU9250_Handle->_enFifoGyro * 6]) << 8)
					| MPU9250_Handle->_buffer[2 + MPU9250_Handle->_enFifoAccel * 6 + MPU9250_Handle->_enFifoTemp * 2
							+ MPU9250_Handle->_enFifoGyro * 6];
			MPU9250_Handle->_hzcounts = (((int16_t) MPU9250_Handle->_buffer[5 + MPU9250_Handle->_enFifoAccel * 6
					+ MPU9250_Handle->_enFifoTemp * 2 + MPU9250_Handle->_enFifoGyro * 6]) << 8)
					| MPU9250_Handle->_buffer[4 + MPU9250_Handle->_enFifoAccel * 6 + MPU9250_Handle->_enFifoTemp * 2
							+ MPU9250_Handle->_enFifoGyro * 6];

			// transform and convert to float values
			MPU9250_Handle->_hxFifo[i] = (((float) (MPU9250_Handle->_hxcounts) * MPU9250_Handle->_magScaleX) - MPU9250_Handle->_hxb) * MPU9250_Handle->_hxs;
			MPU9250_Handle->_hyFifo[i] = (((float) (MPU9250_Handle->_hycounts) * MPU9250_Handle->_magScaleY) - MPU9250_Handle->_hyb) * MPU9250_Handle->_hys;
			MPU9250_Handle->_hzFifo[i] = (((float) (MPU9250_Handle->_hzcounts) * MPU9250_Handle->_magScaleZ) - MPU9250_Handle->_hzb) * MPU9250_Handle->_hzs;
			MPU9250_Handle->_hSize = MPU9250_Handle->_fifoSize / MPU9250_Handle->_fifoFrameSize;
		}
	}

	return 1;
}

/* returns the accelerometer FIFO size and data in the x direction, m/s/s */
void MPU9250FIFO_getFifoAccelX_mss(uint32_t *size, float* data, struct MPU9250_Handle_s *MPU9250_Handle) {
	*size = MPU9250_Handle->_aSize;
	memcpy(data, MPU9250_Handle->_axFifo, MPU9250_Handle->_aSize * sizeof(float));
}

/* returns the accelerometer FIFO size and data in the y direction, m/s/s */
void MPU9250FIFO_getFifoAccelY_mss(uint32_t *size, float* data, struct MPU9250_Handle_s *MPU9250_Handle) {
	*size = MPU9250_Handle->_aSize;
	memcpy(data, MPU9250_Handle->_ayFifo, MPU9250_Handle->_aSize * sizeof(float));
}

/* returns the accelerometer FIFO size and data in the z direction, m/s/s */
void MPU9250FIFO_getFifoAccelZ_mss(uint32_t *size, float* data, struct MPU9250_Handle_s *MPU9250_Handle) {
	*size = MPU9250_Handle->_aSize;
	memcpy(data, MPU9250_Handle->_azFifo, MPU9250_Handle->_aSize * sizeof(float));
}

/* returns the gyroscope FIFO size and data in the x direction, rad/s */
void MPU9250FIFO_getFifoGyroX_rads(uint32_t *size, float* data, struct MPU9250_Handle_s *MPU9250_Handle) {
	*size = MPU9250_Handle->_gSize;
	memcpy(data, MPU9250_Handle->_gxFifo, MPU9250_Handle->_gSize * sizeof(float));
}

/* returns the gyroscope FIFO size and data in the y direction, rad/s */
void MPU9250FIFO_getFifoGyroY_rads(uint32_t *size, float* data, struct MPU9250_Handle_s *MPU9250_Handle) {
	*size = MPU9250_Handle->_gSize;
	memcpy(data, MPU9250_Handle->_gyFifo, MPU9250_Handle->_gSize * sizeof(float));
}

/* returns the gyroscope FIFO size and data in the z direction, rad/s */
void MPU9250FIFO_getFifoGyroZ_rads(uint32_t *size, float* data, struct MPU9250_Handle_s *MPU9250_Handle) {
	*size = MPU9250_Handle->_gSize;
	memcpy(data, MPU9250_Handle->_gzFifo, MPU9250_Handle->_gSize * sizeof(float));
}

/* returns the magnetometer FIFO size and data in the x direction, uT */
void MPU9250FIFO_getFifoMagX_uT(uint32_t *size, float* data, struct MPU9250_Handle_s *MPU9250_Handle) {
	*size = MPU9250_Handle->_hSize;
	memcpy(data, MPU9250_Handle->_hxFifo, MPU9250_Handle->_hSize * sizeof(float));
}

/* returns the magnetometer FIFO size and data in the y direction, uT */
void MPU9250FIFO_getFifoMagY_uT(uint32_t *size, float* data, struct MPU9250_Handle_s *MPU9250_Handle) {
	*size = MPU9250_Handle->_hSize;
	memcpy(data, MPU9250_Handle->_hyFifo, MPU9250_Handle->_hSize * sizeof(float));
}

/* returns the magnetometer FIFO size and data in the z direction, uT */
void MPU9250FIFO_getFifoMagZ_uT(uint32_t *size, float* data, struct MPU9250_Handle_s *MPU9250_Handle) {
	*size = MPU9250_Handle->_hSize;
	memcpy(data, MPU9250_Handle->_hzFifo, MPU9250_Handle->_hSize * sizeof(float));
}

/* returns the die temperature FIFO size and data, C */
void MPU9250FIFO_getFifoTemperature_C(uint32_t *size, float* data, struct MPU9250_Handle_s *MPU9250_Handle) {
	*size = MPU9250_Handle->_tSize;
	memcpy(data, MPU9250_Handle->_tFifo, MPU9250_Handle->_tSize * sizeof(float));
}

/* estimates the gyro biases */
int MPU9250_calibrateGyro(struct MPU9250_Handle_s *MPU9250_Handle) {
	// set the range, bandwidth, and srd
	if (MPU9250_setGyroRange(GYRO_RANGE_250DPS, MPU9250_Handle) < 0) {
		return -1;
	}
	if (MPU9250_setDlpfBandwidth(DLPF_BANDWIDTH_20HZ, MPU9250_Handle) < 0) {
		return -2;
	}
	if (MPU9250_setSrd(19, MPU9250_Handle) < 0) {
		return -3;
	}

	// take samples and find bias
	MPU9250_Handle->_gxbD = 0;
	MPU9250_Handle->_gybD = 0;
	MPU9250_Handle->_gzbD = 0;
	float GyroX_rads;
	float GyroY_rads;
	float GyroZ_rads;

	for (uint32_t i = 0; i < MPU9250_Handle->_numSamples; i++) {
		MPU9250_readSensor(MPU9250_Handle);

		GyroX_rads = MPU9250_getGyroX_rads(MPU9250_Handle);
		GyroY_rads = MPU9250_getGyroY_rads(MPU9250_Handle);
		GyroZ_rads = MPU9250_getGyroZ_rads(MPU9250_Handle);

		MPU9250_Handle->_gxbD += (GyroX_rads + MPU9250_Handle->_gxb) / ((double) MPU9250_Handle->_numSamples);
		MPU9250_Handle->_gybD += (GyroY_rads + MPU9250_Handle->_gyb) / ((double) MPU9250_Handle->_numSamples);
		MPU9250_Handle->_gzbD += (GyroZ_rads + MPU9250_Handle->_gzb) / ((double) MPU9250_Handle->_numSamples);
		HAL_Delay(20);
	}

	MPU9250_Handle->_gxb = (float) MPU9250_Handle->_gxbD;
	MPU9250_Handle->_gyb = (float) MPU9250_Handle->_gybD;
	MPU9250_Handle->_gzb = (float) MPU9250_Handle->_gzbD;

	// set the range, bandwidth, and srd back to what they were
	if (MPU9250_setGyroRange(MPU9250_Handle->_gyroRange, MPU9250_Handle) < 0) {
		return -4;
	}
	if (MPU9250_setDlpfBandwidth(MPU9250_Handle->_bandwidth, MPU9250_Handle) < 0) {
		return -5;
	}
	if (MPU9250_setSrd(MPU9250_Handle->_srd, MPU9250_Handle) < 0) {
		return -6;
	}

	return 1;
}

/* returns the gyro bias in the X direction, rad/s */
float MPU9250_getGyroBiasX_rads(const struct MPU9250_Handle_s *MPU9250_Handle) {
	return MPU9250_Handle->_gxb;
}

/* returns the gyro bias in the Y direction, rad/s */
float MPU9250_getGyroBiasY_rads(const struct MPU9250_Handle_s *MPU9250_Handle) {
	return MPU9250_Handle->_gyb;
}

/* returns the gyro bias in the Z direction, rad/s */
float MPU9250_getGyroBiasZ_rads(const struct MPU9250_Handle_s *MPU9250_Handle) {
	return MPU9250_Handle->_gzb;
}

/* sets the gyro bias in the X direction to bias, rad/s */
void MPU9250_setGyroBiasX_rads(float bias, struct MPU9250_Handle_s *MPU9250_Handle) {
	MPU9250_Handle->_gxb = bias;
}

/* sets the gyro bias in the Y direction to bias, rad/s */
void MPU9250_setGyroBiasY_rads(float bias, struct MPU9250_Handle_s *MPU9250_Handle) {
	MPU9250_Handle->_gyb = bias;
}

/* sets the gyro bias in the Z direction to bias, rad/s */
void MPU9250_setGyroBiasZ_rads(float bias, struct MPU9250_Handle_s *MPU9250_Handle) {
	MPU9250_Handle->_gzb = bias;
}

/* finds bias and scale factor calibration for the accelerometer,
 this should be run for each axis in each direction (6 total) to find
 the min and max values along each */
int MPU9250_calibrateAccel(struct MPU9250_Handle_s *MPU9250_Handle) {
	// set the range, bandwidth, and srd
	if (MPU9250_setAccelRange(ACCEL_RANGE_2G, MPU9250_Handle) < 0) {
		return -1;
	}
	if (MPU9250_setDlpfBandwidth(DLPF_BANDWIDTH_20HZ, MPU9250_Handle) < 0) {
		return -2;
	}
	if (MPU9250_setSrd(19, MPU9250_Handle) < 0) {
		return -3;
	}

	// take samples and find min / max
	MPU9250_Handle->_axbD = 0;
	MPU9250_Handle->_aybD = 0;
	MPU9250_Handle->_azbD = 0;
	float AccelX_mss;
	float AccelY_mss;
	float AccelZ_mss;
	for (uint32_t i = 0; i < MPU9250_Handle->_numSamples; i++) {
		MPU9250_readSensor(MPU9250_Handle);

		AccelX_mss = MPU9250_getAccelX_mss(MPU9250_Handle);
		AccelZ_mss = MPU9250_getAccelX_mss(MPU9250_Handle);
		AccelY_mss = MPU9250_getAccelX_mss(MPU9250_Handle);

		MPU9250_Handle->_axbD += (AccelX_mss / MPU9250_Handle->_axs + MPU9250_Handle->_axb)
				/ ((double) MPU9250_Handle->_numSamples);
		MPU9250_Handle->_aybD += (AccelY_mss / MPU9250_Handle->_ays + MPU9250_Handle->_ayb)
				/ ((double) MPU9250_Handle->_numSamples);
		MPU9250_Handle->_azbD += (AccelZ_mss / MPU9250_Handle->_azs + MPU9250_Handle->_azb)
				/ ((double) MPU9250_Handle->_numSamples);

		HAL_Delay(20);
	}
	if (MPU9250_Handle->_axbD > 9.0f) {
		MPU9250_Handle->_axmax = (float) MPU9250_Handle->_axbD;
	}
	if (MPU9250_Handle->_aybD > 9.0f) {
		MPU9250_Handle->_aymax = (float) MPU9250_Handle->_aybD;
	}
	if (MPU9250_Handle->_azbD > 9.0f) {
		MPU9250_Handle->_azmax = (float) MPU9250_Handle->_azbD;
	}
	if (MPU9250_Handle->_axbD < -9.0f) {
		MPU9250_Handle->_axmin = (float) MPU9250_Handle->_axbD;
	}
	if (MPU9250_Handle->_aybD < -9.0f) {
		MPU9250_Handle->_aymin = (float) MPU9250_Handle->_aybD;
	}
	if (MPU9250_Handle->_azbD < -9.0f) {
		MPU9250_Handle->_azmin = (float) MPU9250_Handle->_azbD;
	}

	// find bias and scale factor
	if ((fabs(MPU9250_Handle->_axmin) > 9.0f) && (fabs(MPU9250_Handle->_axmax) > 9.0f)) {
		MPU9250_Handle->_axb = (MPU9250_Handle->_axmin + MPU9250_Handle->_axmax) / 2.0f;
		MPU9250_Handle->_axs = MPU9250_Handle->_G / ((fabs(MPU9250_Handle->_axmin) + fabs(MPU9250_Handle->_axmax)) / 2.0f);
	}
	if ((fabs(MPU9250_Handle->_aymin) > 9.0f) && (fabs(MPU9250_Handle->_aymax) > 9.0f)) {
		MPU9250_Handle->_ayb = (MPU9250_Handle->_aymin + MPU9250_Handle->_aymax) / 2.0f;
		MPU9250_Handle->_ays = MPU9250_Handle->_G / ((fabs(MPU9250_Handle->_aymin) + fabs(MPU9250_Handle->_aymax)) / 2.0f);
	}
	if ((fabs(MPU9250_Handle->_azmin) > 9.0f) && (fabs(MPU9250_Handle->_azmax) > 9.0f)) {
		MPU9250_Handle->_azb = (MPU9250_Handle->_azmin + MPU9250_Handle->_azmax) / 2.0f;
		MPU9250_Handle->_azs = MPU9250_Handle->_G / ((fabs(MPU9250_Handle->_azmin) + fabs(MPU9250_Handle->_azmax)) / 2.0f);
	}

	// set the range, bandwidth, and srd back to what they were
	if (MPU9250_setAccelRange(MPU9250_Handle->_accelRange, MPU9250_Handle) < 0) {
		return -4;
	}
	if (MPU9250_setDlpfBandwidth(MPU9250_Handle->_bandwidth, MPU9250_Handle) < 0) {
		return -5;
	}
	if (MPU9250_setSrd(MPU9250_Handle->_srd, MPU9250_Handle) < 0) {
		return -6;
	}

	return 1;
}

/* returns the accelerometer bias in the X direction, m/s/s */
float MPU9250_getAccelBiasX_mss(const struct MPU9250_Handle_s *MPU9250_Handle) {
	return MPU9250_Handle->_axb;
}

/* returns the accelerometer scale factor in the X direction */
float MPU9250_getAccelScaleFactorX(const struct MPU9250_Handle_s *MPU9250_Handle) {
	return MPU9250_Handle->_axs;
}

/* returns the accelerometer bias in the Y direction, m/s/s */
float MPU9250_getAccelBiasY_mss(const struct MPU9250_Handle_s *MPU9250_Handle) {
	return MPU9250_Handle->_ayb;
}

/* returns the accelerometer scale factor in the Y direction */
float MPU9250_getAccelScaleFactorY(const struct MPU9250_Handle_s *MPU9250_Handle) {
	return MPU9250_Handle->_ays;
}

/* returns the accelerometer bias in the Z direction, m/s/s */
float MPU9250_getAccelBiasZ_mss(const struct MPU9250_Handle_s *MPU9250_Handle) {
	return MPU9250_Handle->_azb;
}

/* returns the accelerometer scale factor in the Z direction */
float MPU9250_getAccelScaleFactorZ(const struct MPU9250_Handle_s *MPU9250_Handle) {
	return MPU9250_Handle->_azs;
}

/* sets the accelerometer bias (m/s/s) and scale factor in the X direction */
void MPU9250_setAccelCalX(float bias, float scaleFactor, struct MPU9250_Handle_s *MPU9250_Handle) {
	MPU9250_Handle->_axb = bias;
	MPU9250_Handle->_axs = scaleFactor;
}

/* sets the accelerometer bias (m/s/s) and scale factor in the Y direction */
void MPU9250_setAccelCalY(float bias, float scaleFactor, struct MPU9250_Handle_s *MPU9250_Handle) {
	MPU9250_Handle->_ayb = bias;
	MPU9250_Handle->_ays = scaleFactor;
}

/* sets the accelerometer bias (m/s/s) and scale factor in the Z direction */
void MPU9250_setAccelCalZ(float bias, float scaleFactor, struct MPU9250_Handle_s *MPU9250_Handle) {
	MPU9250_Handle->_azb = bias;
	MPU9250_Handle->_azs = scaleFactor;
}

/* finds bias and scale factor calibration for the magnetometer,
 the sensor should be rotated in a figure 8 motion until complete */
int MPU9250_calibrateMag(struct MPU9250_Handle_s *MPU9250_Handle) {
	// set the srd
	if (MPU9250_setSrd(19, MPU9250_Handle) < 0) {
		return -1;
	}

	// get a starting set of data
	MPU9250_readSensor(MPU9250_Handle);

	MPU9250_Handle->_hxmax = MPU9250_getMagX_uT(MPU9250_Handle);
	MPU9250_Handle->_hxmin = MPU9250_getMagX_uT(MPU9250_Handle);
	MPU9250_Handle->_hymax = MPU9250_getMagY_uT(MPU9250_Handle);
	MPU9250_Handle->_hymin = MPU9250_getMagY_uT(MPU9250_Handle);
	MPU9250_Handle->_hzmax = MPU9250_getMagZ_uT(MPU9250_Handle);
	MPU9250_Handle->_hzmin = MPU9250_getMagZ_uT(MPU9250_Handle);

	// collect data to find max / min in each channel
	MPU9250_Handle->_counter = 0;
	float MagX_uT;
	float MagY_uT;
	float MagZ_uT;
	while (MPU9250_Handle->_counter < MPU9250_Handle->_maxCounts) {
		MPU9250_Handle->_delta = 0.0f;
		MPU9250_Handle->_framedelta = 0.0f;
		
		MPU9250_readSensor(MPU9250_Handle);

		MagX_uT = MPU9250_getMagX_uT(MPU9250_Handle);
		MagY_uT = MPU9250_getMagY_uT(MPU9250_Handle);
		MagZ_uT = MPU9250_getMagZ_uT(MPU9250_Handle);


		MPU9250_Handle->_hxfilt = (MPU9250_Handle->_hxfilt * ((float) MPU9250_Handle->_coeff - 1)
				+ (MagX_uT / MPU9250_Handle->_hxs + MPU9250_Handle->_hxb)) / ((float) MPU9250_Handle->_coeff);
		MPU9250_Handle->_hyfilt = (MPU9250_Handle->_hyfilt * ((float) MPU9250_Handle->_coeff - 1)
				+ (MagY_uT / MPU9250_Handle->_hys + MPU9250_Handle->_hyb)) / ((float) MPU9250_Handle->_coeff);
		MPU9250_Handle->_hzfilt = (MPU9250_Handle->_hzfilt * ((float) MPU9250_Handle->_coeff - 1)
				+ (MagZ_uT / MPU9250_Handle->_hzs + MPU9250_Handle->_hzb)) / ((float) MPU9250_Handle->_coeff);
		
		if (MPU9250_Handle->_hxfilt > MPU9250_Handle->_hxmax) {
			MPU9250_Handle->_delta = MPU9250_Handle->_hxfilt - MPU9250_Handle->_hxmax;
			MPU9250_Handle->_hxmax = MPU9250_Handle->_hxfilt;
		}

		if (MPU9250_Handle->_delta > MPU9250_Handle->_framedelta) {
			MPU9250_Handle->_framedelta = MPU9250_Handle->_delta;
		}

		if (MPU9250_Handle->_hyfilt > MPU9250_Handle->_hymax) {
			MPU9250_Handle->_delta = MPU9250_Handle->_hyfilt - MPU9250_Handle->_hymax;
			MPU9250_Handle->_hymax = MPU9250_Handle->_hyfilt;
		}

		if (MPU9250_Handle->_delta > MPU9250_Handle->_framedelta) {
			MPU9250_Handle->_framedelta = MPU9250_Handle->_delta;
		}

		if (MPU9250_Handle->_hzfilt > MPU9250_Handle->_hzmax) {
			MPU9250_Handle->_delta = MPU9250_Handle->_hzfilt - MPU9250_Handle->_hzmax;
			MPU9250_Handle->_hzmax = MPU9250_Handle->_hzfilt;
		}

		if (MPU9250_Handle->_delta > MPU9250_Handle->_framedelta) {
			MPU9250_Handle->_framedelta = MPU9250_Handle->_delta;
		}

		if (MPU9250_Handle->_hxfilt < MPU9250_Handle->_hxmin) {
			MPU9250_Handle->_delta = fabs(MPU9250_Handle->_hxfilt - MPU9250_Handle->_hxmin);
			MPU9250_Handle->_hxmin = MPU9250_Handle->_hxfilt;
		}

		if (MPU9250_Handle->_delta > MPU9250_Handle->_framedelta) {
			MPU9250_Handle->_framedelta = MPU9250_Handle->_delta;
		}

		if (MPU9250_Handle->_hyfilt < MPU9250_Handle->_hymin) {
			MPU9250_Handle->_delta = fabs(MPU9250_Handle->_hyfilt - MPU9250_Handle->_hymin);
			MPU9250_Handle->_hymin = MPU9250_Handle->_hyfilt;
		}

		if (MPU9250_Handle->_delta > MPU9250_Handle->_framedelta) {
			MPU9250_Handle->_framedelta = MPU9250_Handle->_delta;
		}

		if (MPU9250_Handle->_hzfilt < MPU9250_Handle->_hzmin) {
			MPU9250_Handle->_delta = fabs(MPU9250_Handle->_hzfilt - MPU9250_Handle->_hzmin);
			MPU9250_Handle->_hzmin = MPU9250_Handle->_hzfilt;
		}

		if (MPU9250_Handle->_delta > MPU9250_Handle->_framedelta) {
			MPU9250_Handle->_framedelta = MPU9250_Handle->_delta;
		}

		if (MPU9250_Handle->_framedelta > MPU9250_Handle->_deltaThresh) {
			MPU9250_Handle->_counter = 0;
		} else {
			MPU9250_Handle->_counter++;
		}
		
		HAL_Delay(20);
	}

	// find the magnetometer bias
	MPU9250_Handle->_hxb = (MPU9250_Handle->_hxmax + MPU9250_Handle->_hxmin) / 2.0f;
	MPU9250_Handle->_hyb = (MPU9250_Handle->_hymax + MPU9250_Handle->_hymin) / 2.0f;
	MPU9250_Handle->_hzb = (MPU9250_Handle->_hzmax + MPU9250_Handle->_hzmin) / 2.0f;

	// find the magnetometer scale factor
	MPU9250_Handle->_hxs = (MPU9250_Handle->_hxmax - MPU9250_Handle->_hxmin) / 2.0f;
	MPU9250_Handle->_hys = (MPU9250_Handle->_hymax - MPU9250_Handle->_hymin) / 2.0f;
	MPU9250_Handle->_hzs = (MPU9250_Handle->_hzmax - MPU9250_Handle->_hzmin) / 2.0f;
	MPU9250_Handle->_avgs = (MPU9250_Handle->_hxs + MPU9250_Handle->_hys + MPU9250_Handle->_hzs) / 3.0f;
	MPU9250_Handle->_hxs = MPU9250_Handle->_avgs / MPU9250_Handle->_hxs;
	MPU9250_Handle->_hys = MPU9250_Handle->_avgs / MPU9250_Handle->_hys;
	MPU9250_Handle->_hzs = MPU9250_Handle->_avgs / MPU9250_Handle->_hzs;

	// set the srd back to what it was
	if (MPU9250_setSrd(MPU9250_Handle->_srd, MPU9250_Handle) < 0) {
		return -2;
	}

	return 1;
}

/* returns the magnetometer bias in the X direction, uT */
float MPU9250_getMagBiasX_uT(const struct MPU9250_Handle_s *MPU9250_Handle) {
	return MPU9250_Handle->_hxb;
}

/* returns the magnetometer scale factor in the X direction */
float MPU9250_getMagScaleFactorX(const struct MPU9250_Handle_s *MPU9250_Handle) {
	return MPU9250_Handle->_hxs;
}

/* returns the magnetometer bias in the Y direction, uT */
float MPU9250_getMagBiasY_uT(const struct MPU9250_Handle_s *MPU9250_Handle) {
	return MPU9250_Handle->_hyb;
}

/* returns the magnetometer scale factor in the Y direction */
float MPU9250_getMagScaleFactorY(const struct MPU9250_Handle_s *MPU9250_Handle) {
	return MPU9250_Handle->_hys;
}

/* returns the magnetometer bias in the Z direction, uT */
float MPU9250_getMagBiasZ_uT(const struct MPU9250_Handle_s *MPU9250_Handle) {
	return MPU9250_Handle->_hzb;
}

/* returns the magnetometer scale factor in the Z direction */
float MPU9250_getMagScaleFactorZ(const struct MPU9250_Handle_s *MPU9250_Handle) {
	return MPU9250_Handle->_hzs;
}

/* sets the magnetometer bias (uT) and scale factor in the X direction */
void MPU9250_setMagCalX(float bias, float scaleFactor, struct MPU9250_Handle_s *MPU9250_Handle) {
	MPU9250_Handle->_hxb = bias;
	MPU9250_Handle->_hxs = scaleFactor;
}

/* sets the magnetometer bias (uT) and scale factor in the Y direction */
void MPU9250_setMagCalY(float bias, float scaleFactor, struct MPU9250_Handle_s *MPU9250_Handle) {
	MPU9250_Handle->_hyb = bias;
	MPU9250_Handle->_hys = scaleFactor;
}

/* sets the magnetometer bias (uT) and scale factor in the Z direction */
void MPU9250_setMagCalZ(float bias, float scaleFactor, struct MPU9250_Handle_s *MPU9250_Handle) {
	MPU9250_Handle->_hzb = bias;
	MPU9250_Handle->_hzs = scaleFactor;
}
