#pragma once


#include <MPU6050_light.h>



MPU6050 mpu(Wire);


// MPU6050 初始化
void MPU6050_Init(void)
{
    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);

    while(status!=0){ } // stop everything if could not connect to MPU6050

    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(1000);
    mpu.calcOffsets(true,true);
    Serial.println("Done!\n");
}
