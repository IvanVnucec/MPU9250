#include "main.h"
#include "mpu9250/mpu9250.h"


int user_defined_i2c_write_function(uint8_t subAddress, uint8_t data)
{
    // TODO
}

int user_defined_i2c_read_function(uint8_t subAddress, uint8_t count, uint8_t* dest)
{
    // TODO
}

int user_defined_delay_function(float ms)
{
    // TODO
}

int main(void)
{
    int retval;
    MPU9250_Handle_s MPU9250_Handle;

    retval = MPU9250_begin(&user_defined_i2c_write_function, 
        &user_defined_i2c_read_function,
        &user_defined_delay_function,
        &MPU9250_Handle);

    if (retval < 0) {
        return -1; //failure
    }

    while (1)
    {
        if (MPU9250_readSensor(&MPU9250_Handle) > 0)
        {
            float accx = MPU9250_getAccelX_mss(&MPU9250_Handle);
            float accy = MPU9250_getAccelY_mss(&MPU9250_Handle);
            float accz = MPU9250_getAccelZ_mss(&MPU9250_Handle);

            float gyrx = MPU9250_getGyroX_rads(&MPU9250_Handle);
            float gyry = MPU9250_getGyroY_rads(&MPU9250_Handle);
            float gyrz = MPU9250_getGyroZ_rads(&MPU9250_Handle);

            float magx = MPU9250_getMagX_uT(&MPU9250_Handle);
            float magy = MPU9250_getMagY_uT(&MPU9250_Handle);
            float magz = MPU9250_getMagZ_uT(&MPU9250_Handle);

            float temp = MPU9250_getTemperature_C(&MPU9250_Handle);
        } else {
            return -1; // failure
        }
    }

    return 0;
}
