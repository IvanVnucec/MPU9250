# MPU9250
Driver for MPU9250 written in C language.

# Description
The InvenSense MPU-9250 is a System in Package (SiP) that combines two chips: the MPU-6500 three-axis gyroscope and three-axis accelerometer; and the AK8963 three-axis magnetometer. The MPU-9250 supports I2C, up to 400 kHz. The following selectable full scale sensor ranges are available:

| Gyroscope Full Scale Range | Accelerometer Full Scale Range | Magnetometer Full Scale Range |
| --- | --- | ---  |
| +/- 250 (deg/s)  | +/- 2 (g)  | +/- 4800 (uT) |
| +/- 500 (deg/s)  | +/- 4 (g)  | |
| +/- 1000 (deg/s) | +/- 8 (g)  | |
| +/- 2000 (deg/s) | +/- 16 (g) | |

The MPU-9250 samples the gyroscopes, accelerometers, and magnetometers with 16 bit analog to digital converters. It also features programmable digital filters, a precision clock, an embedded temperature sensor, programmable interrupts (including wake on motion), and a 512 byte FIFO buffer.

# Usage
This library supports I2C communication.

## Installation
Simply clone or download this library into your libraries folder. 

### Common Setup Functions
The following functions are used to setup the MPU-9250 sensor. These should be called once before data collection. The *begin* function should always be used. Optionally, the *setAccelRange* and *setGyroRange*, *setDlpfBandwidth*, and *setSrd* functions can be used to set the accelerometer and gyroscope full scale ranges, DLPF bandwidth, and SRD to values other than default. The *enableDataReadyInterrupt* and *disableDataReadyInterrupt* control whether the MPU-9250 generates an interrupt on data ready. The *enableWakeOnMotion* puts the MPU-9250 into a low power mode and enables an interrupt when motion detected is above a given threshold. Finally, *enableFifo* sets up and enables the FIFO buffer. These functions are described in detail, below.

<b>struct MPU9250_Handle_s *MPU9250_Handle**
This handler is used when calling MPU9250 functions. Handler should be initialized somewere in the main.c file.

```C
struct MPU9250_Handle_s MPU9250_Handle;
```

<b>int begin(struct MPU9250_Handle_s *MPU9250_Handle)**
This should be called in your setup function. It initializes communication with the MPU-9250, sets up the sensor for reading data, and estimates the gyro bias, which is removed from the sensor data. This function returns a positive value on a successful initialization and returns a negative value on an unsuccesful initialization. If unsuccessful, please check your wiring or try resetting power to the sensor. The following is an example of setting up the MPU-9250.

```C
int status;
status = MPU9250_begin(&MPU9250_Handle);
```

#### Configuration Functions

<b>(optional) int MPU9250_setAccelRange(AccelRange range, struct MPU9250_Handle_s *MPU9250_Handle)**
This function sets the accelerometer full scale range to the given  value. By default, if this function is not called, a full scale range of +/- 16 g will be used. The enumerated accelerometer full scale ranges are:

| Accelerometer Name | Accelerometer Full Scale Range | 
| ------------------ | ------------------------------ | 
| ACCEL_RANGE_2G     | +/- 2 (g)                      |
| ACCEL_RANGE_4G     | +/- 4 (g)                      | 
| ACCEL_RANGE_8G     | +/- 8 (g)                      |
| ACCEL_RANGE_16G    | +/- 16 (g)                     | 

This function returns a positive value on success and a negative value on failure. The following is an example of selecting an accelerometer full scale range of +/- 8g.

```C
status = MPU9250_setAccelRange(ACCEL_RANGE_8G, &MPU9250_Handle);
```

<b>(optional) int MPU9250_setGyroRange(GyroRange_t range, struct MPU9250_Handle_s *MPU9250_Handle)**
This function sets the gyroscope full scale range to the given  value. By default, if this function is not called, a full scale range of +/- 2000 deg/s will be used. The enumerated gyroscope full scale ranges are:

| Gyroscope Name     | Gyroscope Full Scale Range |
| ------------------ | -------------------------- |
| GYRO_RANGE_250DPS  | +/- 250 (deg/s)            |
| GYRO_RANGE_500DPS  | +/- 500 (deg/s)            |
| GYRO_RANGE_1000DPS | +/- 1000 (deg/s)           |
| GYRO_RANGE_2000DPS | +/- 2000 (deg/s)           | 

This function returns a positive value on success and a negative value on failure. The following is an example of selecting an gyroscope full scale range of +/- 250 deg/s.

```C
status = MPU9250_setGyroRange(GYRO_RANGE_250DPS, &MPU9250_Handle);
```

<b>(optional) int MPU9250_setDlpfBandwidth(DlpfBandwidth_t bandwidth, struct MPU9250_Handle_s *MPU9250_Handle)**
This is an optional function to set the programmable Digital Low Pass Filter (DLPF) bandwidth. By default, if this function is not called, a DLPF bandwidth of 184 Hz is used. The following DLPF bandwidths are supported:

| Bandwidth Name | DLPF Bandwidth | Gyroscope Delay | Accelerometer Delay | Temperature Bandwidth | Temperature Delay |
| --- | --- | --- | --- | --- | --- |
| DLPF_BANDWIDTH_184HZ | 184 Hz | 2.9 ms   | 5.8 ms   | 188 Hz | 1.9 ms  |
| DLPF_BANDWIDTH_92HZ  | 92 Hz  | 3.9 ms   | 7.8 ms   | 98 Hz  | 2.8 ms  |
| DLPF_BANDWIDTH_41HZ  | 41 Hz  | 5.9 ms   | 11.8 ms  | 42 Hz  | 4.8 ms  |
| DLPF_BANDWIDTH_20HZ  | 20 Hz  | 9.9 ms   | 19.8 ms  | 20 Hz  | 8.3 ms  |
| DLPF_BANDWIDTH_10HZ  | 10 Hz  | 17.85 ms | 35.7 ms  | 10 Hz  | 13.4 ms |
| DLPF_BANDWIDTH_5HZ   | 5 Hz   | 33.48 ms | 66.96 ms | 5 Hz   | 18.6 ms |

This function returns a positive value on success and a negative value on failure. The following is an example of selecting a DLPF bandwidth of 20 Hz.

```C
status = MPU9250_setDlpfBandwidth(DLPF_BANDWIDTH_20HZ, &MPU9250_Handle);
```

<b>(optional) int MPU9250_setSrd(uint8_t srd, struct MPU9250_Handle_s *MPU9250_Handle)**
This is an optional function to set the data output rate. The data output rate is set by a sample rate divider, *uint8_t SRD*. The data output rate is then given by:

*Data Output Rate = 1000 / (1 + SRD)*

By default, if this function is not called, an SRD of 0 is used resulting in a data output rate of 1000 Hz. This allows the data output rate for the gyroscopes, accelerometers, and temperature sensor to be set between 3.9 Hz and 1000 Hz. Note that data should be read at or above the selected rate. In order to prevent aliasing, the data should be sampled at twice the frequency of the DLPF bandwidth or higher. For example, this means for a DLPF bandwidth set to 41 Hz, the data output rate and data collection should be at frequencies of 82 Hz or higher.

The magnetometer is fixed to an output rate of: 
* 100 Hz for frequencies of 100 Hz or above (SRD less than or equal to 9)
* 8 Hz for frequencies below 100 Hz (SRD greater than 9)

When the data is read above the selected output rate, the read data will be stagnant. For example, when the output rate is selected to 1000 Hz, the magnetometer data will be the same for 10 sequential frames. 

This function returns a positive value on success and a negative value on failure. The following is an example of selecting an SRD of 9, resulting in a data output rate of 100 Hz.

```C
status = MPU9250_setSrd(9, &MPU9250_Handle);
```

<b>(optional) int MPU9250_enableDataReadyInterrupt(struct MPU9250_Handle_s *MPU9250_Handle)**
An interrupt is tied to the data output rate. The MPU-9250 *INT* pin will issue a 50us pulse when data is ready. This is extremely useful for using interrupts to clock data collection that should occur at a regular interval. This function enables this interrupt, which will occur at a frequency given by the SRD. This function returns a positive value on success and a negative value on failure. The following is an example of enabling the data ready interrupt.

```C
status = MPU9250_enableDataReadyInterrupt(&MPU9250_Handle);
```

<b>(optional) int MPU9250_disableDataReadyInterrupt(struct MPU9250_Handle_s *MPU9250_Handle)**
This function disables the data ready interrupt, described above. This function returns a positive value on success and a negative value on failure. The following is an example of disabling the data ready interrupt.

```C
status = MPU9250_disableDataReadyInterrupt(&MPU9250_Handle);
```

#### Calibration Functions

<b>(optional) int MPU9250_calibrateGyro(struct MPU9250_Handle_s *MPU9250_Handle)**
The gyro bias is automatically estimated during the *MPU9250_begin()* function and removed from sensor measurements. This function will re-estimate the gyro bias and remove the new bias from future sensor measurements. The sensor should be stationary during this process. This function returns a positive value on success and a negative value on failure. The following is an example of estimating new gyro biases.

```C
status = MPU9250_calibrateGyro(&MPU9250_Handle);
```

<b>(optional) float MPU9250_getGyroBiasX_rads(struct MPU9250_Handle_s *MPU9250_Handle)**
This function returns the current gyro bias in the X direction in units of rad/s. 

```C
float gxb;
gxb = MPU9250_getGyroBiasX_rads(&MPU9250_Handle);
```

<b>(optional) float MPU9250_getGyroBiasY_rads(struct MPU9250_Handle_s *MPU9250_Handle)**
This function returns the current gyro bias in the Y direction in units of rad/s.

```C
float gyb;
gyb = MPU9250_getGyroBiasY_rads(&MPU9250_Handle);
```

<b>(optional) float MPU9250_getGyroBiasZ_rads(struct MPU9250_Handle_s *MPU9250_Handle)**
This function returns the current gyro bias in the Z direction in units of rad/s.

```C
float gzb;
gzb = MPU9250_getGyroBiasZ_rads(&MPU9250_Handle);
```

<b>(optional) void MPU9250_setGyroBiasX_rads(float bias, struct MPU9250_Handle_s *MPU9250_Handle)**
This function sets the gyro bias being used in the X direction to the input value in units of rad/s.

```C
float gxb = 0.001; // gyro bias of 0.001 rad/s
MPU9250_setGyroBiasX_rads(gxb, &MPU9250_Handle);
```

<b>(optional) void MPU9250_setGyroBiasY_rads(float bias, struct MPU9250_Handle_s *MPU9250_Handle)**
This function sets the gyro bias being used in the Y direction to the input value in units of rad/s.

```C
float gyb = 0.001; // gyro bias of 0.001 rad/s
MPU9250_setGyroBiasY_rads(gyb, &MPU9250_Handle);
```

<b>(optional) void MPU9250_setGyroBiasZ_rads(float bias, struct MPU9250_Handle_s *MPU9250_Handle)**
This function sets the gyro bias being used in the Z direction to the input value in units of rad/s.

```C
float gzb = 0.001; // gyro bias of 0.001 rad/s
MPU9250_setGyroBiasZ_rads(gzb, &MPU9250_Handle);
```

<b>(optional) int MPU9250_calibrateAccel(struct MPU9250_Handle_s *MPU9250_Handle)**
This function will estimate the bias and scale factor needed to calibrate the accelerometers. This function works one axis at a time and needs to be run for all 6 sensor orientations. After it has collected enough sensor data, it will estimate the bias and scale factor for all three accelerometer channels and apply these corrections to the measured data. Accelerometer calibration only needs to be performed once on the IMU, the get and set functions detailed below can be used to retrieve the estimated bias and scale factors and use them during future power cycles or operations with the MPU9250. This function returns a positive value on success and a negative value on failure.

```C
status = MPU9250_calibrateAccel(&MPU9250_Handle);
```

<b>(optional) float MPU9250_getAccelBiasX_mss(struct MPU9250_Handle_s *MPU9250_Handle)**
This function returns the current accelerometer bias in the X direction in units of m/s/s.

```C
float axb;
axb = MPU9250_getAccelBiasX_mss(&MPU9250_Handle);
```

<b>(optional) float MPU9250_getAccelScaleFactorX(struct MPU9250_Handle_s *MPU9250_Handle)**
This function returns the current accelerometer scale factor in the X direction.

```C
float axs;
axs = MPU9250_getAccelScaleFactorX(&MPU9250_Handle);
```

<b>(optional) float MPU9250_getAccelBiasY_mss(struct MPU9250_Handle_s *MPU9250_Handle)**
This function returns the current accelerometer bias in the Y direction in units of m/s/s.

```C
float ayb;
ayb = MPU9250_getAccelBiasY_mss(&MPU9250_Handle);
```

<b>(optional) float MPU9250_getAccelScaleFactorY(struct MPU9250_Handle_s *MPU9250_Handle)**
This function returns the current accelerometer scale factor in the Y direction.

```C
float ays;
ays = MPU9250_getAccelScaleFactorY(&MPU9250_Handle);
```

<b>(optional) float MPU9250_getAccelBiasZ_mss(struct MPU9250_Handle_s *MPU9250_Handle)**
This function returns the current accelerometer bias in the Z direction in units of m/s/s.

```C
float azb;
azb = MPU9250_getAccelBiasZ_mss(&MPU9250_Handle);
```

<b>(optional) float getAccelScaleFactorZ(struct MPU9250_Handle_s *MPU9250_Handle)**
This function returns the current accelerometer scale factor in the Z direction.

```C
float azs;
azs = MPU9250_getAccelScaleFactorZ(&MPU9250_Handle);
```

<b>(optional) void MPU9250_setAccelCalX(float bias,float scaleFactor, struct MPU9250_Handle_s *MPU9250_Handle)**
This function sets the accelerometer bias (m/s/s) and scale factor being used in the X direction to the input values.

```C
float axb = 0.01; // accel bias of 0.01 m/s/s
float axs = 0.97; // accel scale factor of 0.97
MPU9250_setAccelCalX(axb, axs, &MPU9250_Handle);
```

<b>(optional) void MPU9250_setAccelCalY(float bias,float scaleFactor, struct MPU9250_Handle_s *MPU9250_Handle)**
This function sets the accelerometer bias (m/s/s) and scale factor being used in the Y direction to the input values.

```C
float ayb = 0.01; // accel bias of 0.01 m/s/s
float ays = 0.97; // accel scale factor of 0.97
MPU9250_setAccelCalY(ayb, ays, &MPU9250_Handle);
```

<b>(optional) void MPU9250_setAccelCalZ(float bias,float scaleFactor, struct MPU9250_Handle_s *MPU9250_Handle)**
This function sets the accelerometer bias (m/s/s) and scale factor being used in the Z direction to the input values.

```C
float azb = 0.01; // accel bias of 0.01 m/s/s
float azs = 0.97; // accel scale factor of 0.97
MPU9250_setAccelCalZ(azb, azs, &MPU9250_Handle);
```

<b>(optional) int MPU9250_calibrateMag(struct MPU9250_Handle_s *MPU9250_Handle)**
This function will estimate the bias and scale factor needed to calibrate the magnetometers. This function works on all the sensor axes at once, you should continuously and slowly move the sensor in a figure 8 while the function is running. After it has collected enough sensor data, it will estimate the bias and scale factor for all three magnetometer channels and apply these corrections to the measured data. Magnetometer calibration only needs to be performed once on the IMU, unless the eletrical or magnetic environment changes. The get and set functions detailed below can be used to retrieve the estimated bias and scale factors and use them during future power cycles or operations with the MPU9250_ This function returns a positive value on success and a negative value on failure.

```C
status = MPU9250_calibrateMag(&MPU9250_Handle);
```

<b>(optional) float MPU9250_getMagBiasX_uT(struct MPU9250_Handle_s *MPU9250_Handle)**
This function returns the current magnetometer bias in the X direction in units of uT.

```C
float hxb;
hxb = MPU9250_getMagBiasX_uT(&MPU9250_Handle);
```

<b>(optional) float MPU9250_getMagScaleFactorX(struct MPU9250_Handle_s *MPU9250_Handle)**
This function returns the current magnetometer scale factor in the X direction.

```C
float hxs;
hxs = MPU9250_getMagScaleFactorX(&MPU9250_Handle);
```

<b>(optional) float MPU9250_getMagBiasY_uT(struct MPU9250_Handle_s *MPU9250_Handle)**
This function returns the current magnetometer bias in the Y direction in units of uT.

```C
float hyb;
hyb = MPU9250_getMagBiasY_uT(&MPU9250_Handle);
```

<b>(optional) float MPU9250_getMagScaleFactorY(struct MPU9250_Handle_s *MPU9250_Handle)**
This function returns the current magnetometer scale factor in the Y direction.

```C
float hys;
hys = MPU9250_getMagScaleFactorY(&MPU9250_Handle);
```

<b>(optional) float MPU9250_getMagBiasZ_uT(struct MPU9250_Handle_s *MPU9250_Handle)**
This function returns the current magnetometer bias in the Z direction in units of uT.

```C
float hzb;
hzb = MPU9250_getMagBiasZ_uT(&MPU9250_Handle);
```

<b>(optional) float MPU9250_getMagScaleFactorZ(struct MPU9250_Handle_s *MPU9250_Handle)**
This function returns the current magnetometer scale factor in the Z direction.

```C
float hzs;
hzs = MPU9250_getMagScaleFactorZ(&MPU9250_Handle);
```

<b>(optional) void MPU9250_setMagCalX(float bias,float scaleFactor, struct MPU9250_Handle_s *MPU9250_Handle)**
This function sets the magnetometer bias (uT) and scale factor being used in the X direction to the input values.

```C
float hxb = 10.0; // mag bias of 10 uT
float hxs = 0.97; // mag scale factor of 0.97
MPU9250_setMagCalX(hxb, hxs, &MPU9250_Handle);
```

<b>(optional) void MPU9250_setMagCalY(float bias,float scaleFactor, struct MPU9250_Handle_s *MPU9250_Handle)**
This function sets the magnetometer bias (uT) and scale factor being used in the Y direction to the input values.

```C
float hyb = 10.0; // mag bias of 10 uT
float hys = 0.97; // mag scale factor of 0.97
MPU9250_setMagCalY(hyb, hys, &MPU9250_Handle);
```

<b>(optional) void MPU9250_setMagCalZ(float bias,float scaleFactor, struct MPU9250_Handle_s *MPU9250_Handle)**
This function sets the magnetometer bias (uT) and scale factor being used in the Z direction to the input values.

```C
float hzb = 10.0; // mag bias of 10 uT
float hzs = 0.97; // mag scale factor of 0.97
MPU9250_setMagCalZ(hzb, hzs, &MPU9250_Handle);
```

#### Wake on Motion Setup

<b>(optional) int MPU9250_enableWakeOnMotion(float womThresh_mg,LpAccelOdr odr, struct MPU9250_Handle_s *MPU9250_Handle)**
This function enables the MPU-9250 wake on motion interrupt functionality. It places the MPU-9250 into a low power state, with the MPU-9250 waking up at an interval determined by the Low Power Accelerometer Output Data Rate. If the accelerometer detects motion in excess of the threshold given, it generates a 50us pulse from the MPU-9250 INT pin. The following enumerated Low Power Accelerometer Output Data Rates are supported:

| LpAccelOdr Name      | Output Data Rate |
| ------------------   | ---------------- |
| LP_ACCEL_ODR_0_24HZ  | 0.24 Hz          |
| LP_ACCEL_ODR_0_49HZ  | 0.49 Hz          |
| LP_ACCEL_ODR_0_98HZ  | 0.98 Hz          |
| LP_ACCEL_ODR_1_95HZ  | 1.95 Hz          | 
| LP_ACCEL_ODR_3_91HZ  | 3.91 Hz          |
| LP_ACCEL_ODR_7_81HZ  | 7.81 Hz          |
| LP_ACCEL_ODR_15_63HZ | 15.63 Hz         |
| LP_ACCEL_ODR_31_25HZ | 31.25 Hz         |
| LP_ACCEL_ODR_62_50HZ | 62.50 Hz         |
| LP_ACCEL_ODR_125HZ   | 125 Hz           |
| LP_ACCEL_ODR_250HZ   | 250 Hz           |
| LP_ACCEL_ODR_500HZ   | 500 Hz           |

The motion threshold is given as a float value between 0 and 1020 mg mapped, which is internally mapped to a single byte, 0-255 value. This function returns a positive value on success and a negative value on failure. The following is an example of enabling the wake on motion with a 400 mg threshold and a ODR of 31.25 Hz.

```C
status = MPU9250_enableWakeOnMotion(400, LP_ACCEL_ODR_31_25HZ, &MPU9250_Handle);
```

### Common Data Collection Functions
The functions below are used to collect data from the MPU-9250 sensor. Data is returned scaled to engineering units.

#### Real-Time Data Collection
<b>int MPU9250_readSensor(struct MPU9250_Handle_s *MPU9250_Handle)</b>reads the sensor and stores the newest data in a buffer, it should be called every time you would like to retrieve data from the sensor. This function returns a positive value on success and a negative value on failure.

```C
MPU9250_readSensor(&MPU9250_Handle);
```

<b>float MPU9250_getAccelX_mss(struct MPU9250_Handle_s *MPU9250_Handle)</b>gets the accelerometer value from the data buffer in the X direction and returns it in units of m/s/s.

```C
float ax;
ax = MPU9250_getAccelX_mss(&MPU9250_Handle);
```

<b>float MPU9250_getAccelY_mss(struct MPU9250_Handle_s *MPU9250_Handle)</b>gets the accelerometer value from the data buffer in the Y direction and returns it in units of m/s/s.

```C
float ay;
ay = MPU9250_getAccelY_mss(&MPU9250_Handle);
```

<b>float MPU9250_getAccelZ_mss(struct MPU9250_Handle_s *MPU9250_Handle)</b>gets the accelerometer value from the data buffer in the Z direction and returns it in units of m/s/s.

```C
float az;
az = MPU9250_getAccelZ_mss(&MPU9250_Handle);
```

<b>float MPU9250_getGyroX_rads(struct MPU9250_Handle_s *MPU9250_Handle)</b>gets the gyroscope value from the data buffer in the X direction and returns it in units of rad/s.

```C
float gx;
gx = MPU9250_getGyroX_rads(&MPU9250_Handle);
```

<b>float MPU9250_getGyroY_rads(struct MPU9250_Handle_s *MPU9250_Handle)</b>gets the gyroscope value from the data buffer in the Y direction and returns it in units of rad/s.

```C
float gy;
gy = MPU9250_getGyroY_rads(&MPU9250_Handle);
```

<b>float MPU9250_getGyroZ_rads(struct MPU9250_Handle_s *MPU9250_Handle)</b>gets the gyroscope value from the data buffer in the Z direction and returns it in units of rad/s.

```C
float gz;
gz = MPU9250_getGyroZ_rads(&MPU9250_Handle);
```

<b>float MPU9250_getMagX_uT(struct MPU9250_Handle_s *MPU9250_Handle)</b>gets the magnetometer value from the data buffer in the X direction and returns it in units of uT.

```C
float hx;
hx = MPU9250_getMagX_uT(&MPU9250_Handle);
```

<b>float MPU9250_getMagY_uT(struct MPU9250_Handle_s *MPU9250_Handle)</b>gets the magnetometer value from the data buffer in the Y direction and returns it in units of uT.

```C
float hy;
hy = MPU9250_getMagY_uT(&MPU9250_Handle);
```

<b>float MPU9250_getMagZ_uT(struct MPU9250_Handle_s *MPU9250_Handle)</b>gets the magnetometer value from the data buffer in the Z direction and returns it in units of uT.

```C
float hz;
hz = MPU9250_getMagZ_uT(&MPU9250_Handle);
```

<b>float MPU9250_getTemperature_C(struct MPU9250_Handle_s *MPU9250_Handle)</b>gets the die temperature value from the data buffer and returns it in units of C.

```C
float temperature;
temperature = MPU9250_getTemperature_C(&MPU9250_Handle);
```

### FIFO Setup
<b>(optional) int MPU9250_enableFifo(bool accel,bool gyro,bool mag,bool temp, struct MPU9250_Handle_s *MPU9250_Handle)**
This function configures and enables the MPU-9250 FIFO buffer. This 512 byte buffer samples data at the data output rate set by the SRD and enables the microcontroller to bulk read the data, reducing microcontroller workload for certain applications. It is configured with a set of boolean values describing which data to buffer in the FIFO: accelerometer, gyroscope, magnetometer, or temperature. The accelerometer and gyroscope data each take 6 bytes of space per sample while the magnetometer takes 7 bytes of space and the temperature 2 bytes. It's important to select only the data sources desired to ensure that the FIFO does not overrun between reading it. For example, enabling all of the data sources would take 21 bytes per sample allowing the FIFO to hold only 24 samples before overflowing. If only the accelerometer data is needed, this increases to 85 samples before overflowing. This function returns a positive value on success and a negative value on failure. The following is an example of enabling the FIFO to buffer accelerometer and gyroscope data. 

```C
status = MPU9250_enableFifo(1, 1, 0, 0, &MPU9250_Handle);
```

### FIFO Data Collection
<b>int MPU9250_readFifo(struct MPU9250_Handle_s *MPU9250_Handle)</b>reads the FIFO buffer from the MPU-9250, parses it and stores the data in buffers on the microcontroller. It should be called every time you would like to retrieve data from the FIFO buffer. This function returns a positive value on success and a negative value on failure.

```C
MPU9250_readFifo(&MPU9250_Handle);
```

<b>void MPU9250_getFifoAccelX_mss(size_t *size,float* data, struct MPU9250_Handle_s *MPU9250_Handle)</b>gets the accelerometer value from the data buffer in the X direction and returns it in units of m/s/s. The data is returned as an array along with the number of elements within that array. Ensure that the buffer you are transfering to has enough capacity to store the data.

```C
float ax[100];
size_t samples;
MPU9250_getFifoAccelX_mss(&samples, ax, &MPU9250_Handle);
```

<b>void MPU9250_getFifoAccelY_mss(size_t *size,float* data, struct MPU9250_Handle_s *MPU9250_Handle)</b>gets the accelerometer value from the data buffer in the Y direction and returns it in units of m/s/s. The data is returned as an array along with the number of elements within that array. Ensure that the buffer you are transfering to has enough capacity to store the data.

```C
float ay[100];
size_t samples;
MPU9250_getFifoAccelY_mss(&samples, ay, &MPU9250_Handle);
```

<b>void MPU9250_getFifoAccelZ_mss(size_t *size,float* data, struct MPU9250_Handle_s *MPU9250_Handle)</b>gets the accelerometer value from the data buffer in the Z direction and returns it in units of m/s/s. The data is returned as an array along with the number of elements within that array. Ensure that the buffer you are transfering to has enough capacity to store the data.

```C
float az[100];
size_t samples;
MPU9250_getFifoAccelZ_mss(&samples, az, &MPU9250_Handle);
```

<b>void MPU9250_getFifoGyroX_rads(size_t *size,float* data, struct MPU9250_Handle_s *MPU9250_Handle)</b>gets the gyroscope value from the data buffer in the X direction and returns it in units of rad/s. The data is returned as an array along with the number of elements within that array. Ensure that the buffer you are transfering to has enough capacity to store the data.

```C
float gx[100];
size_t samples;
MPU9250_getFifoGyroX_rads(&samples, gx, &MPU9250_Handle);
```

<b>void MPU9250_getFifoGyroY_rads(size_t *size,float* data, struct MPU9250_Handle_s *MPU9250_Handle)</b>gets the gyroscope value from the data buffer in the Y direction and returns it in units of rad/s. The data is returned as an array along with the number of elements within that array. Ensure that the buffer you are transfering to has enough capacity to store the data.

```C
float gy[100];
size_t samples;
MPU9250_getFifoGyroY_rads(&samples, gy, &MPU9250_Handle);
```

<b>void MPU9250_getFifoGyroZ_rads(size_t *size,float* data, struct MPU9250_Handle_s *MPU9250_Handle)</b>gets the gyroscope value from the data buffer in the Z direction and returns it in units of rad/s. The data is returned as an array along with the number of elements within that array. Ensure that the buffer you are transfering to has enough capacity to store the data.

```C
float gz[100];
size_t samples;
MPU9250_getFifoGyroZ_rads(&samples, gx, &MPU9250_Handle);
```

<b>void MPU9250_getFifoMagX_uT(size_t *size,float* data, struct MPU9250_Handle_s *MPU9250_Handle)</b>gets the magnetometer value from the data buffer in the X direction and returns it in units of uT. The data is returned as an array along with the number of elements within that array. Ensure that the buffer you are transfering to has enough capacity to store the data.

```C
float hx[100];
size_t samples;
MPU9250_getFifoMagX_uT(&samples, hx, &MPU9250_Handle);
```

<b>void MPU9250_getFifoMagY_uT(size_t *size,float* data, struct MPU9250_Handle_s *MPU9250_Handle)</b>gets the magnetometer value from the data buffer in the Y direction and returns it in units of uT. The data is returned as an array along with the number of elements within that array. Ensure that the buffer you are transfering to has enough capacity to store the data.

```C
float hy[100];
size_t samples;
MPU9250_getFifoMagY_uT(&samples, hy, &MPU9250_Handle);
```

<b>void MPU9250_getFifoMagZ_uT(size_t *size,float* data, struct MPU9250_Handle_s *MPU9250_Handle)</b>gets the magnetometer value from the data buffer in the Z direction and returns it in units of uT. The data is returned as an array along with the number of elements within that array. Ensure that the buffer you are transfering to has enough capacity to store the data.

```C
float hz[100];
size_t samples;
MPU9250_getFifoMagZ_uT(&samples, hz, &MPU9250_Handle);
```

<b>void MPU9250_getFifoTemperature_C(size_t *size,float* data, struct MPU9250_Handle_s *MPU9250_Handle)</b>gets the die temperature value from the data buffer and returns it in units of C. The data is returned as an array along with the number of elements within that array. Ensure that the buffer you are transfering to has enough capacity to store the data.

```C
float temp[100];
size_t samples;
MPU9250_getFifoTemperature_C(&samples, temp, &MPU9250_Handle);
```

# Contribution 
This library was being developed with the help of [Bolder Flight Systems MPU9250 Library](https://github.com/bolderflight/MPU9250).
