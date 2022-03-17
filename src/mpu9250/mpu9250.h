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


typedef int (*MPU9250_writeRegister_T)(uint8_t subAddress, uint8_t data);
typedef int (*MPU9250_readRegisters_T)(uint8_t subAddress, uint8_t count, uint8_t* dest);
typedef int (*MPU9250_delayMiliSec_T)(float ms);


typedef struct {
	// i2c
	MPU9250_writeRegister_T MPU9250_writeRegister;
	MPU9250_readRegisters_T MPU9250_readRegisters;
	MPU9250_delayMiliSec_T  MPU9250_delayMiliSec;

	uint32_t _i2cRate; // 400 kHz
	uint32_t _numBytes; // number of bytes received from I2C

	// track success of interacting with sensor
	int _status;

	// buffer for reading from sensor
	uint8_t _buffer[21];

	// data counts
	int16_t _axcounts, _aycounts, _azcounts;
	int16_t _gxcounts, _gycounts, _gzcounts;
	int16_t _hxcounts, _hycounts, _hzcounts;
	int16_t _tcounts;

	// data buffer
	float _ax, _ay, _az;
	float _gx, _gy, _gz;
	float _hx, _hy, _hz;
	float _t;

	// wake on motion
	uint8_t _womThreshold;

	// scale factors
	float _accelScale;
	float _gyroScale;
	float _magScaleX, _magScaleY, _magScaleZ;
	float _tempScale;
	float _tempOffset;

	// configuration
	AccelRange_t _accelRange;
	GyroRange_t _gyroRange;
	DlpfBandwidth_t _bandwidth;
	uint8_t _srd;

	// gyro bias estimation
	uint32_t _numSamples;
	double _gxbD, _gybD, _gzbD;
	float _gxb, _gyb, _gzb;

	// accel bias and scale factor estimation
	double _axbD, _aybD, _azbD;
	float _axmax, _aymax, _azmax;
	float _axmin, _aymin, _azmin;
	float _axb, _ayb, _azb;
	float _axs;
	float _ays;
	float _azs;

	// magnetometer bias and scale factor estimation
	uint16_t _maxCounts;
	float _deltaThresh;
	uint8_t _coeff;
	uint16_t _counter;
	float _framedelta, _delta;
	float _hxfilt, _hyfilt, _hzfilt;
	float _hxmax, _hymax, _hzmax;
	float _hxmin, _hymin, _hzmin;
	float _hxb, _hyb, _hzb;
	float _hxs;
	float _hys;
	float _hzs;
	float _avgs;

	// transformation matrix
	/* transform the accel and gyro axes to match the magnetometer axes */
	int16_t _tX[3];
	int16_t _tY[3];
	int16_t _tZ[3];

	// constants
	float _G;
	float _d2r;

	// fifo
	uint32_t _enFifoAccel, _enFifoGyro, _enFifoMag, _enFifoTemp;
	uint32_t _fifoSize, _fifoFrameSize;
	float _axFifo[85], _ayFifo[85], _azFifo[85];
	uint32_t _aSize;
	float _gxFifo[85], _gyFifo[85], _gzFifo[85];
	uint32_t _gSize;
	float _hxFifo[73], _hyFifo[73], _hzFifo[73];
	uint32_t _hSize;
	float _tFifo[256];
	uint32_t _tSize;
} MPU9250_Handle_s;


// MPU9250 I2C Address
#define MPU9250_I2C_ADDRESS 0x68

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

int MPU9250_begin(MPU9250_writeRegister_T MPU9250_writeRegister,
	MPU9250_readRegisters_T MPU9250_readRegisters,
	MPU9250_delayMiliSec_T  MPU9250_delayMiliSec,
	MPU9250_Handle_s *MPU9250_Handle);

int MPU9250_setAccelRange(AccelRange_t range, MPU9250_Handle_s *MPU9250_Handle);
int MPU9250_setGyroRange(GyroRange_t range, MPU9250_Handle_s *MPU9250_Handle);
int MPU9250_setDlpfBandwidth(DlpfBandwidth_t bandwidth, MPU9250_Handle_s *MPU9250_Handle);
int MPU9250_setSrd(uint8_t srd, MPU9250_Handle_s *MPU9250_Handle);

int MPU9250_enableDataReadyInterrupt(MPU9250_Handle_s *MPU9250_Handle);
int MPU9250_disableDataReadyInterrupt(MPU9250_Handle_s *MPU9250_Handle);

int MPU9250_enableWakeOnMotion(float womThresh_mg, LpAccelOdr_t odr, MPU9250_Handle_s *MPU9250_Handle);

int MPU9250_readSensor(MPU9250_Handle_s *MPU9250_Handle);

float MPU9250_getAccelX_mss(const MPU9250_Handle_s *MPU9250_Handle);
float MPU9250_getAccelY_mss(const MPU9250_Handle_s *MPU9250_Handle);
float MPU9250_getAccelZ_mss(const MPU9250_Handle_s *MPU9250_Handle);
float MPU9250_getGyroX_rads(const MPU9250_Handle_s *MPU9250_Handle);
float MPU9250_getGyroY_rads(const MPU9250_Handle_s *MPU9250_Handle);
float MPU9250_getGyroZ_rads(const MPU9250_Handle_s *MPU9250_Handle);
float MPU9250_getMagX_uT(const MPU9250_Handle_s *MPU9250_Handle);
float MPU9250_getMagY_uT(const MPU9250_Handle_s *MPU9250_Handle);
float MPU9250_getMagZ_uT(const MPU9250_Handle_s *MPU9250_Handle);
float MPU9250_getTemperature_C(const MPU9250_Handle_s *MPU9250_Handle);

int MPU9250_calibrateGyro();

float MPU9250_getGyroBiasX_rads(const MPU9250_Handle_s *MPU9250_Handle);
float MPU9250_getGyroBiasY_rads(const MPU9250_Handle_s *MPU9250_Handle);
float MPU9250_getGyroBiasZ_rads(const MPU9250_Handle_s *MPU9250_Handle);

void MPU9250_setGyroBiasX_rads(float bias, MPU9250_Handle_s *MPU9250_Handle);
void MPU9250_setGyroBiasY_rads(float bias, MPU9250_Handle_s *MPU9250_Handle);
void MPU9250_setGyroBiasZ_rads(float bias, MPU9250_Handle_s *MPU9250_Handle);

int MPU9250_calibrateAccel(MPU9250_Handle_s *MPU9250_Handle);

float MPU9250_getAccelBiasX_mss(const MPU9250_Handle_s *MPU9250_Handle);
float MPU9250_getAccelScaleFactorX(const MPU9250_Handle_s *MPU9250_Handle);
float MPU9250_getAccelBiasY_mss(const MPU9250_Handle_s *MPU9250_Handle);
float MPU9250_getAccelScaleFactorY(const MPU9250_Handle_s *MPU9250_Handle);
float MPU9250_getAccelBiasZ_mss(const MPU9250_Handle_s *MPU9250_Handle);
float MPU9250_getAccelScaleFactorZ(const MPU9250_Handle_s *MPU9250_Handle);

void MPU9250_setAccelCalX(float bias, float scaleFactor, MPU9250_Handle_s *MPU9250_Handle);
void MPU9250_setAccelCalY(float bias, float scaleFactor, MPU9250_Handle_s *MPU9250_Handle);
void MPU9250_setAccelCalZ(float bias, float scaleFactor, MPU9250_Handle_s *MPU9250_Handle);

int MPU9250_calibrateMag(MPU9250_Handle_s *MPU9250_Handle);

float MPU9250_getMagBiasX_uT(const MPU9250_Handle_s *MPU9250_Handle);
float MPU9250_getMagScaleFactorX(const MPU9250_Handle_s *MPU9250_Handle);
float MPU9250_getMagBiasY_uT(const MPU9250_Handle_s *MPU9250_Handle);
float MPU9250_getMagScaleFactorY(const MPU9250_Handle_s *MPU9250_Handle);
float MPU9250_getMagBiasZ_uT(const MPU9250_Handle_s *MPU9250_Handle);
float MPU9250_getMagScaleFactorZ(const MPU9250_Handle_s *MPU9250_Handle);

void MPU9250_setMagCalX(float bias, float scaleFactor, MPU9250_Handle_s *MPU9250_Handle);
void MPU9250_setMagCalY(float bias, float scaleFactor, MPU9250_Handle_s *MPU9250_Handle);
void MPU9250_setMagCalZ(float bias, float scaleFactor, MPU9250_Handle_s *MPU9250_Handle);

int MPU9250FIFO_enableFifo(uint32_t accel, uint32_t gyro, uint32_t mag,
		uint32_t temp, MPU9250_Handle_s *MPU9250_Handle);

int MPU9250FIFO_readFifo(MPU9250_Handle_s *MPU9250_Handle);

void MPU9250FIFO_getFifoAccelX_mss(uint32_t *size, float* data, MPU9250_Handle_s *MPU9250_Handle);
void MPU9250FIFO_getFifoAccelY_mss(uint32_t *size, float* data, MPU9250_Handle_s *MPU9250_Handle);
void MPU9250FIFO_getFifoAccelZ_mss(uint32_t *size, float* data, MPU9250_Handle_s *MPU9250_Handle);

void MPU9250FIFO_getFifoGyroX_rads(uint32_t *size, float* data, MPU9250_Handle_s *MPU9250_Handle);
void MPU9250FIFO_getFifoGyroY_rads(uint32_t *size, float* data, MPU9250_Handle_s *MPU9250_Handle);
void MPU9250FIFO_getFifoGyroZ_rads(uint32_t *size, float* data, MPU9250_Handle_s *MPU9250_Handle);

void MPU9250FIFO_getFifoMagX_uT(uint32_t *size, float* data, MPU9250_Handle_s *MPU9250_Handle);
void MPU9250FIFO_getFifoMagY_uT(uint32_t *size, float* data, MPU9250_Handle_s *MPU9250_Handle);
void MPU9250FIFO_getFifoMagZ_uT(uint32_t *size, float* data, MPU9250_Handle_s *MPU9250_Handle);

void MPU9250FIFO_getFifoTemperature_C(uint32_t *size, float* data, MPU9250_Handle_s *MPU9250_Handle);

#endif /* MPU9250_H_ */
