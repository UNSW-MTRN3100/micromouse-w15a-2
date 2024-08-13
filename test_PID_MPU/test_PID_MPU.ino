#include "DualEncoder.hpp"
#include "EncoderOdometry.hpp"
#include "Motor.hpp"
#include "Config.hpp"
#include "Wire.h"
#include "OLED.hpp"
#include "MPU6050.hpp"




mtrn3100::DualEncoder encoder;
mtrn3100::Motor motor;


float front_distance = distance_per_cell * 1;
float turning_rotation = 0.5 * PI;



// 移动指令
char moving[] = "frfflflffrfffffrfffflflf";

uint8_t execution_times = 1;
uint8_t executed_time   = 0;

// 待修正的角度
float correct_angle = 0;
float error_angle = 0;


void setup() {
    
    // Monitor 初始化
    Serial.begin(9600);
    // I2C 初始化
    Wire.begin();
    // MPU6050 初始化
    MPU6050_Init();
    // OLED 初始化
    OLED_Init();

}




void loop() {
    

    mpu.update();
    correct_angle = mpu.getAngleZ();

    
    // 指令字符串里字符的个数
    uint8_t number = 0;
    while(moving[number] != '\0') {number++;}


    if(executed_time < execution_times)
    {
      
      for(uint8_t i = 0; i < number; i++)
      {
        char command = moving[i];
        switch (command)
        {
          case 'f':
            motor.Front(encoder, mpu, front_distance);
            
            correct_angle += 0;
            error_angle = (correct_angle - mpu.getAngleZ()) * PI / 180;
            motor.Turning(mpu, error_angle, P_correction, I_correction, D_correction);
            
            break;
          case 'b':
            motor.Front(encoder, mpu, -front_distance);
            
            correct_angle += 0;
            error_angle = (correct_angle - mpu.getAngleZ()) * PI / 180;
            motor.Turning(mpu, error_angle, P_correction, I_correction, D_correction);
            
            break;
          case 'l':
            motor.Turning(mpu, turning_rotation, P_turning, I_turning, D_turning);
            
            correct_angle += 90;
            error_angle = (correct_angle - mpu.getAngleZ()) * PI / 180;
            motor.Turning(mpu, error_angle, P_correction, I_correction, D_correction);
            
            break;
          case 'r':
            motor.Turning(mpu, -turning_rotation, P_turning, I_turning, D_turning);
            
            correct_angle += -90;
            error_angle = (correct_angle - mpu.getAngleZ()) * PI / 180;
            motor.Turning(mpu, error_angle, P_correction, I_correction, D_correction);
            
            break;
        }

      }


      executed_time++;

    }

    mpu.update();
    ShowData(mpu.getAngleZ());

}
