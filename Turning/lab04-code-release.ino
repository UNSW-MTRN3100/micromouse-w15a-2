// #include "DualEncoder.hpp"
// #include "EncoderOdometry.hpp"
// #include "IMUOdometry.hpp"
// #include "Wire.h"
// #include <MPU6050_light.h>

// MPU6050 mpu(Wire);


// #define EN_1_A 2  //These are the pins for the PCB encoder
// #define EN_1_B 7  //These are the pins for the PCB encoder
// #define EN_2_A 3  //These are the pins for the PCB encoder
// #define EN_2_B 8  //These are the pins for the PCB encoder

// mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B, EN_2_A, EN_2_B);
// mtrn3100::EncoderOdometry encoder_odometry(16, 96);  //TASK1 TODO: IDENTIFY THE WHEEL RADIUS AND AXLE LENGTH
// mtrn3100::IMUOdometry IMU_odometry;



// void setup() {
//   Serial.begin(115200);
//   Wire.begin();

//   //Set up the IMU
//   byte status = mpu.begin();
//   Serial.print(F("MPU6050 status: "));
//   Serial.println(status);

//   Serial.println(F("Calculating offsets, do not move MPU6050"));
//   delay(1000);
//   mpu.calcOffsets(true, true);
//   Serial.println("Done!\n");
// }

// void loop() {

//   //UNCOMMENT FOR TASK 2:
//   //THE DELAY IS REQUIRED OTHERWISE THE ENCODER DIFFERENCE IS TOO SMALL
//   delay(100);
//   encoder_odometry.update(encoder.getLeftRotation(), encoder.getRightRotation());

//   //UNCOMMET FOR TASK 3:
//   //NOTE: IMU ODOMETRY IS REALLY BAD, THIS TASK EXISTS TO TEACH YOU WHY IMU ODOMETRY SUCKS, DO NOT SPEND TOO LONG ON IT
//   //mpu.update();
//   //IMU_odometry.update(mpu.getAccX(),mpu.getAccY());


//   Serial.print("ODOM:\t\t");
//   Serial.print(encoder_odometry.getX());
//   Serial.print(",\t\t");
//   Serial.print(encoder_odometry.getY());
//   Serial.print(",\t\t");
//   Serial.print(encoder_odometry.getH());
//   Serial.println();
// }
#include "DualEncoder.hpp"
#include "EncoderOdometry.hpp"
#include "IMUOdometry.hpp"
#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

#define EN_1_A 2  // These are the pins for the PCB encoder
#define EN_1_B 7  // These are the pins for the PCB encoder
#define EN_2_A 3  // These are the pins for the PCB encoder
#define EN_2_B 8  // These are the pins for the PCB encoder
#define MOTOR_LEFT_PIN1 9
#define MOTOR_LEFT_PIN2 10
#define MOTOR_RIGHT_PIN1 11
#define MOTOR_RIGHT_PIN2 12

mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B, EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoder_odometry(16, 96);  // TASK1 TODO: IDENTIFY THE WHEEL RADIUS AND AXLE LENGTH
mtrn3100::IMUOdometry IMU_odometry;

class SimplePID {
public:
    SimplePID(double kp, double ki, double kd) 
        : kp(kp), ki(ki), kd(kd), previousError(0), integral(0) {}
    
    float Lastoutput;

    double compute(double setpoint, double input) {
        double error = setpoint - input;
        integral += error;
        double derivative = error - previousError;
        double output = (kp * error + ki * integral + kd * derivative);
        previousError = error;
        Lastoutput=Lastoutput+output;
        Lastoutput = constrain(Lastoutput, -255, 255);
        return Lastoutput;
    }

private:
    double kp, ki, kd;
    double previousError;
    double integral;
};

SimplePID myPID(5.0, 0.0, 5.0); 

const double targetRadians = 2 * PI;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // 设置IMU
    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while(status!=0){ }

    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(1000);
    mpu.calcOffsets(true, true);
    Serial.println("Done!\n");

    pinMode(MOTOR_LEFT_PIN1, OUTPUT);
    pinMode(MOTOR_LEFT_PIN2, OUTPUT);
    pinMode(MOTOR_RIGHT_PIN1, OUTPUT);
    pinMode(MOTOR_RIGHT_PIN2, OUTPUT);
}

void loop() {
    encoder_odometry.update(encoder.getLeftRotation(), encoder.getRightRotation());

    //double input = encoder.getLeftRotation() - encoder.getRightRotation();
    double input = encoder_odometry.getH();
    double output = myPID.compute(targetRadians, input);
    int motorSpeed = constrain(output, -255, 255); 
    double error = targetRadians - abs(input);
    Serial.print("Input: ");
    Serial.print(input);
    Serial.print("\tOutput: ");
    Serial.print(output);
    Serial.print("\tH ");
    Serial.print(encoder_odometry.getH());
    Serial.println();

    if (abs(error) < 0.2) {
        analogWrite(MOTOR_LEFT_PIN1, 0);
        digitalWrite(MOTOR_LEFT_PIN2, LOW);
        analogWrite(MOTOR_RIGHT_PIN1, 0);
        digitalWrite(MOTOR_RIGHT_PIN2, LOW);
        Serial.println("Target reached. Stopping motors.");
        delay(10000);

    }else{
      if (motorSpeed > 0) {
        analogWrite(MOTOR_LEFT_PIN1, motorSpeed);
        digitalWrite(MOTOR_LEFT_PIN2, LOW);
        analogWrite(MOTOR_RIGHT_PIN1, motorSpeed);
        digitalWrite(MOTOR_RIGHT_PIN2, LOW);
    } else {
        
        analogWrite(MOTOR_LEFT_PIN1, -motorSpeed);
        digitalWrite(MOTOR_LEFT_PIN2, HIGH);   
        analogWrite(MOTOR_RIGHT_PIN1, -motorSpeed);
        digitalWrite(MOTOR_RIGHT_PIN2, HIGH);
      }
    }

    /*Serial.print("\t\t");
    Serial.print(encoder.getLeftRotation());
    Serial.print(",\t\t");
    Serial.print(encoder.getRightRotation());
    Serial.print(",\t\t");
    Serial.print(output);
    Serial.println();*/
    delay(100);
}





