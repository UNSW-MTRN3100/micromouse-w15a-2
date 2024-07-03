#include "DualEncoder.hpp"
#include "EncoderOdometry.hpp"
#include "IMUOdometry.hpp"
#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

#define EN_1_A 2 //These are the pins for the PCB encoder
#define EN_1_B 7 //These are the pins for the PCB encoder
#define EN_2_A 3 //These are the pins for the PCB encoder
#define EN_2_B 8 //These are the pins for the PCB encoder

#define MOTOR_LEFT_PWM 9
#define MOTOR_LEFT_DIR 10
#define MOTOR_RIGHT_PWM 11
#define MOTOR_RIGHT_DIR 12

const float R = 16;
const float b = 96;


mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B,EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoder_odometry(R,b); //TASK1 TODO: IDENTIFY THE WHEEL RADIUS AND AXLE LENGTH
// mtrn3100::IMUOdometry IMU_odometry;




const float target_rotation = 4 * PI; // 1轮旋转的弧度值

// PID控制器参数
float Kp = 100;
float Ki = 0;
float Kd = 0;

float proportional = 0;
float integral = 0;
float derivative = 0;

float previous_error = 0;
float dt = 0.05; // 控制循环的时间间隔




void setup() {
    Serial.begin(9600);
    Wire.begin();


    //Set up the IMU
    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    
    
    pinMode(MOTOR_LEFT_PWM, OUTPUT);
    pinMode(MOTOR_LEFT_DIR, OUTPUT);
    pinMode(MOTOR_RIGHT_PWM, OUTPUT);
    pinMode(MOTOR_RIGHT_DIR, OUTPUT);
    
    
    while(status!=0){ } // stop everything if could not connect to MPU6050
    
    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(100);
    mpu.calcOffsets(true,true);
    Serial.println("Done!\n");
}




void loop() {

//UNCOMMENT FOR TASK 2: 
//THE DELAY IS REQUIRED OTHERWISE THE ENCODER DIFFERENCE IS TOO SMALL
    
    delay(dt*1000);

    
    float left_rotation = encoder.getLeftRotation();
    float right_rotation = encoder.getRightRotation();

    
    encoder_odometry.update(left_rotation,right_rotation);


    float error = target_rotation - ((left_rotation - right_rotation) / 2);

    proportional = error;
    integral = integral + error * dt;
    derivative = (error - previous_error) / dt;
    
    float output = Kp * error + Ki * integral + Kd * derivative;

    if (output>255) {output = 255;}


    previous_error = error;



    
    if (output >=0)
    {
      digitalWrite(MOTOR_LEFT_DIR,HIGH);
      digitalWrite(MOTOR_RIGHT_DIR, LOW);
      analogWrite(MOTOR_LEFT_PWM, output);
      analogWrite(MOTOR_RIGHT_PWM, output);
    }
    else
    {
      digitalWrite(MOTOR_LEFT_DIR, LOW);
      digitalWrite(MOTOR_RIGHT_DIR, HIGH);
      analogWrite(MOTOR_LEFT_PWM, -output);
      analogWrite(MOTOR_RIGHT_PWM, -output);
    }


    


//UNCOMMET FOR TASK 3:
//NOTE: IMU ODOMETRY IS REALLY BAD, THIS TASK EXISTS TO TEACH YOU WHY IMU ODOMETRY SUCKS, DO NOT SPEND TOO LONG ON IT
    //mpu.update();
    //IMU_odometry.update(mpu.getAccX(),mpu.getAccY());
    
    // Serial.print("ODOM:\t\t");




    Serial.print("left_rotation = ");
    Serial.print(left_rotation);
    Serial.print(",\t\t");
    Serial.print("right_rotation = ");
    Serial.print(right_rotation);
    Serial.print(",\t\t");
    // Serial.print("proportional = ");
    // Serial.print(proportional);
    // Serial.print(",\t\t");
    // Serial.print("integral = ");
    // Serial.print(integral);
    // Serial.print(",\t\t");
    // Serial.print("derivative = ");
    // Serial.print(derivative);
    // Serial.print(",\t\t");
    Serial.print("PID output = ");
    Serial.print(output);
    Serial.print(",\t\t");
  
    Serial.print("x = ");
    Serial.print(encoder_odometry.getX());
    Serial.print(",\t\t");

    Serial.println();
    // Serial.print(encoder_odometry.getY());
    // Serial.print(",\t\t");
    // Serial.print(encoder_odometry.getH());
    // Serial.println();
}
