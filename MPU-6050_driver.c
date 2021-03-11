
#include "MPU-6050_driver.h"
#include "mcc_generated_files/i2c1.h"

//for DEBUG
#include "util.h"

#define DEV_ADDRESS 0x68
#define PWR_MGMT 0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
#define SMPLRT_DIV 0x19

struct Vec3D gyroOffset;
int16_t accOffset;

void writeByte(uint8_t addr, uint8_t val);

void sensorSetup() 
{
    writeByte(PWR_MGMT, 0x00);
    writeByte(SMPLRT_DIV, 8);
}

void sensorCalibration() 
{
    // gyro
    const uint8_t samples = 10;
    struct MPUData reading;
    struct Vec3D avg;
    struct Vec3D sum;
    int16_t accSum = 0;
    uint8_t i = 0;
    uint8_t j = 0;
    gyroOffset.x = 0;
    gyroOffset.y = 0;
    gyroOffset.z = 0;
    for(i=0; i<samples; i++) 
    {
        sum.x = 0;
        sum.y = 0;
        sum.z = 0;
        for(j=0; j<samples; j++) 
        {
            reading = sensorFetch();
            sum.x += reading.gyro.x/samples;
            sum.y += reading.gyro.y/samples;
            sum.z += reading.gyro.z/samples;
        }
        avg.x += sum.x/samples;
        avg.y += sum.y/samples;
        avg.z += sum.z/samples;
        accSum += reading.accl.x;
    }
    gyroOffset.x = avg.x;
    gyroOffset.y = avg.y;
    gyroOffset.z = avg.z;
    
    //accelerometer
    accOffset = accSum/samples;
}

void writeByte(uint8_t addr, uint8_t val)
{
    I2C1_MESSAGE_STATUS status;
    uint8_t data[2];
    data[0] = addr;
    data[1] = val;
    I2C1_MasterWrite(data, 2, DEV_ADDRESS, &status);
    while(status == I2C1_MESSAGE_PENDING);
}


struct MPUData sensorFetch()
{
    struct MPUData data;
    I2C1_MESSAGE_STATUS status;
    uint8_t out[1];
    uint8_t in[6];
    out[0] = GYRO_XOUT_H;
    I2C1_MasterWrite(out, 1, DEV_ADDRESS, &status);
    while(status == I2C1_MESSAGE_PENDING);
    I2C1_MasterRead(in, 6, DEV_ADDRESS, &status);
    while(status == I2C1_MESSAGE_PENDING);
    data.gyro.x = (in[1] | (in[0]<<8)) - gyroOffset.x;
    data.gyro.y = (in[3] | (in[2]<<8)) - gyroOffset.y;
    data.gyro.z = (in[5] | (in[4]<<8)) - gyroOffset.z;
    
    out[0] = ACCEL_XOUT_H;
    I2C1_MasterWrite(out, 1, DEV_ADDRESS, &status);
    while(status == I2C1_MESSAGE_PENDING);
    I2C1_MasterRead(in, 6, DEV_ADDRESS, &status);
    while(status == I2C1_MESSAGE_PENDING);
    data.accl.x = in[1] | (in[0]<<8) - accOffset;
    data.accl.y = in[3] | (in[2]<<8);
    data.accl.z = in[5] | (in[4]<<8);
    return data;
}

uint8_t sensorTest()
{
    I2C1_MESSAGE_STATUS status;
    uint8_t data[2];
    data[0] = 0x22;
    I2C1_MasterWrite(data, 1, DEV_ADDRESS, &status);
    while(status == I2C1_MESSAGE_PENDING);
    delay(100);
    print("I2C status: ");
    delay(10);
    printInt(status);
    delay(10);
    println("");
    return status;
}
