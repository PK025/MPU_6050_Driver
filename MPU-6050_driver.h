
#ifndef MPU_6050_DRIVER_H
#define	MPU_6050_DRIVER_H

#include <xc.h>


struct Vec3D {
    int16_t x;
    int16_t y;
    int16_t z;
};

struct MPUData {
    struct Vec3D gyro;
    struct Vec3D accl;
};


void sensorSetup();
void sensorCalibration();
struct MPUData sensorFetch();
uint8_t sensorTest();


#endif	/* MPU_6050_DRIVER_H */

