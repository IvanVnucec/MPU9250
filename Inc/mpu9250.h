/*
 * mpu9250.h
 *
 *  Created on: Jan 12, 2020
 *      Author: Ivan
 */

#ifndef MPU9250_H_
#define MPU9250_H_

#include <stdint.h>

typedef enum GyroRange {
	GYRO_RANGE_250DPS, GYRO_RANGE_500DPS, GYRO_RANGE_1000DPS, GYRO_RANGE_2000DPS
} GyroRange_t;

typedef enum AccelRange {
	ACCEL_RANGE_2G, ACCEL_RANGE_4G, ACCEL_RANGE_8G, ACCEL_RANGE_16G
} AccelRange_t;

typedef enum DlpfBandwidth {
	DLPF_BANDWIDTH_184HZ,
	DLPF_BANDWIDTH_92HZ,
	DLPF_BANDWIDTH_41HZ,
	DLPF_BANDWIDTH_20HZ,
	DLPF_BANDWIDTH_10HZ,
	DLPF_BANDWIDTH_5HZ
} DlpfBandwidth_t;

typedef enum LpAccelOdr {
	LP_ACCEL_ODR_0_24HZ = 0,
	LP_ACCEL_ODR_0_49HZ = 1,
	LP_ACCEL_ODR_0_98HZ = 2,
	LP_ACCEL_ODR_1_95HZ = 3,
	LP_ACCEL_ODR_3_91HZ = 4,
	LP_ACCEL_ODR_7_81HZ = 5,
	LP_ACCEL_ODR_15_63HZ = 6,
	LP_ACCEL_ODR_31_25HZ = 7,
	LP_ACCEL_ODR_62_50HZ = 8,
	LP_ACCEL_ODR_125HZ = 9,
	LP_ACCEL_ODR_250HZ = 10,
	LP_ACCEL_ODR_500HZ = 11
} LpAccelOdr_t;

// MPU9250 registers
#define ACCEL_OUT 			0x3B
#define GYRO_OUT 			0x43
#define TEMP_OUT 			0x41
#define EXT_SENS_DATA_00 	0x49
#define ACCEL_CONFIG 		0x1C
#define ACCEL_FS_SEL_2G 	0x00
#define ACCEL_FS_SEL_4G 	0x08
#define ACCEL_FS_SEL_8G 	0x10
#define ACCEL_FS_SEL_16G    0x18
#define GYRO_CONFIG     	0x1B
#define GYRO_FS_SEL_250DPS 	0x00
#define GYRO_FS_SEL_500DPS 	0x08
#define GYRO_FS_SEL_1000DPS 0x10
#define GYRO_FS_SEL_2000DPS 0x18
#define ACCEL_CONFIG2 		0x1D
#define ACCEL_DLPF_184 		0x01
#define ACCEL_DLPF_92 		0x02
#define ACCEL_DLPF_41 		0x03
#define ACCEL_DLPF_20 		0x04
#define ACCEL_DLPF_10 		0x05
#define ACCEL_DLPF_5 		0x06
#define CONFIG 				0x1A
#define GYRO_DLPF_184 		0x01
#define GYRO_DLPF_92 		0x02
#define GYRO_DLPF_41 		0x03
#define GYRO_DLPF_20 		0x04
#define GYRO_DLPF_10 		0x05
#define GYRO_DLPF_5 		0x06
#define SMPDIV 				0x19
#define INT_PIN_CFG 		0x37
#define INT_ENABLE 			0x38
#define INT_DISABLE 		0x00
#define INT_PULSE_50US 		0x00
#define INT_WOM_EN 			0x40
#define INT_RAW_RDY_EN 		0x01
#define PWR_MGMNT_1 		0x6B
#define PWR_CYCLE 			0x20
#define PWR_RESET 			0x80
#define CLOCK_SEL_PLL 		0x01
#define PWR_MGMNT_2 		0x6C
#define SEN_ENABLE 			0x00
#define DIS_GYRO 			0x07
#define USER_CTRL 			0x6A
#define I2C_MST_EN 			0x20
#define I2C_MST_CLK 		0x0D
#define I2C_MST_CTRL 		0x24
#define I2C_SLV0_ADDR 		0x25
#define I2C_SLV0_REG 		0x26
#define I2C_SLV0_DO 		0x63
#define I2C_SLV0_CTRL 		0x27
#define I2C_SLV0_EN 		0x80
#define I2C_READ_FLAG 		0x80
#define MOT_DETECT_CTRL 	0x69
#define ACCEL_INTEL_EN 		0x80
#define ACCEL_INTEL_MODE 	0x40
#define LP_ACCEL_ODR 		0x1E
#define WOM_THR 			0x1F
#define WHO_AM_I 			0x75
#define FIFO_EN 			0x23
#define FIFO_TEMP 			0x80
#define FIFO_GYRO 			0x70
#define FIFO_ACCEL 			0x08
#define FIFO_MAG 			0x01
#define FIFO_COUNT 			0x72
#define FIFO_READ 			0x74

// AK8963 registers
#define AK8963_I2C_ADDR 	0x0C
#define AK8963_HXL 			0x03
#define AK8963_CNTL1 		0x0A
#define AK8963_PWR_DOWN 	0x00
#define AK8963_CNT_MEAS1 	0x12
#define AK8963_CNT_MEAS2 	0x16
#define AK8963_FUSE_ROM 	0x0F
#define AK8963_CNTL2 		0x0B
#define AK8963_RESET 		0x01
#define AK8963_ASA 			0x10
#define AK8963_WHO_AM_I 	0x00

int MPU9250_begin();
int MPU9250_setAccelRange(AccelRange_t range);
int MPU9250_setGyroRange(GyroRange_t range);
int MPU9250_setDlpfBandwidth(DlpfBandwidth_t bandwidth);
int MPU9250_setSrd(uint8_t srd);
int MPU9250_enableDataReadyInterrupt();
int MPU9250_disableDataReadyInterrupt();
int MPU9250_enableWakeOnMotion(float womThresh_mg, LpAccelOdr_t odr);
int MPU9250_readSensor();
float MPU9250_getAccelX_mss();
float MPU9250_getAccelY_mss();
float MPU9250_getAccelZ_mss();
float MPU9250_getGyroX_rads();
float MPU9250_getGyroY_rads();
float MPU9250_getGyroZ_rads();
float MPU9250_getMagX_uT();
float MPU9250_getMagY_uT();
float MPU9250_getMagZ_uT();
float MPU9250_getTemperature_C();

int MPU9250_calibrateGyro();
float MPU9250_getGyroBiasX_rads();
float MPU9250_getGyroBiasY_rads();
float MPU9250_getGyroBiasZ_rads();
void MPU9250_setGyroBiasX_rads(float bias);
void MPU9250_setGyroBiasY_rads(float bias);
void MPU9250_setGyroBiasZ_rads(float bias);
int MPU9250_calibrateAccel();
float MPU9250_getAccelBiasX_mss();
float MPU9250_getAccelScaleFactorX();
float MPU9250_getAccelBiasY_mss();
float MPU9250_getAccelScaleFactorY();
float MPU9250_getAccelBiasZ_mss();
float MPU9250_getAccelScaleFactorZ();
void MPU9250_setAccelCalX(float bias, float scaleFactor);
void MPU9250_setAccelCalY(float bias, float scaleFactor);
void MPU9250_setAccelCalZ(float bias, float scaleFactor);
int MPU9250_calibrateMag();
float MPU9250_getMagBiasX_uT();
float MPU9250_getMagScaleFactorX();
float MPU9250_getMagBiasY_uT();
float MPU9250_getMagScaleFactorY();
float MPU9250_getMagBiasZ_uT();
float MPU9250_getMagScaleFactorZ();
void MPU9250_setMagCalX(float bias, float scaleFactor);
void MPU9250_setMagCalY(float bias, float scaleFactor);
void MPU9250_setMagCalZ(float bias, float scaleFactor);

int MPU9250FIFO_enableFifo(uint32_t accel, uint32_t gyro, uint32_t mag,
		uint32_t temp);
int MPU9250FIFO_readFifo(void);
void MPU9250FIFO_getFifoAccelX_mss(uint32_t *size, float* data);
void MPU9250FIFO_getFifoAccelY_mss(uint32_t *size, float* data);
void MPU9250FIFO_getFifoAccelZ_mss(uint32_t *size, float* data);
void MPU9250FIFO_getFifoGyroX_rads(uint32_t *size, float* data);
void MPU9250FIFO_getFifoGyroY_rads(uint32_t *size, float* data);
void MPU9250FIFO_getFifoGyroZ_rads(uint32_t *size, float* data);
void MPU9250FIFO_getFifoMagX_uT(uint32_t *size, float* data);
void MPU9250FIFO_getFifoMagY_uT(uint32_t *size, float* data);
void MPU9250FIFO_getFifoMagZ_uT(uint32_t *size, float* data);
void MPU9250FIFO_getFifoTemperature_C(uint32_t *size, float* data);

#endif /* MPU9250_H_ */
