#pragma once

#include "DualEncoder.hpp"
#include "EncoderOdometry.hpp"
#include "Config.hpp"
#include "PID.hpp"
#include "OLED.hpp"



namespace mtrn3100 {


class Motor {

public:

  Motor() {
    // 初始化 电机引脚
    pinMode(MOTOR_LEFT_PWM, OUTPUT);
    pinMode(MOTOR_LEFT_DIR, OUTPUT);
    pinMode(MOTOR_RIGHT_PWM, OUTPUT);
    pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  }


  // 前进
  void Front(DualEncoder& encoder, MPU6050& MPU, float target_distance)
  {
    float lastMillis = millis();

    EncoderOdometry encoder_odometry;
    
    // 分别用于控制 前进位置 和 两轮旋转位置 的 PID
    PID pid_X(P_front, I_front, D_front);
    PID pid_gyro(P_gyro, I_gyro, D_gyro);
    
    // 起始位置(GCF)
    float Initial_left_rotation = encoder.getLeftRotation();
    float Initial_right_rotation = encoder.getRightRotation();
    MPU.update();
    float Initial_angle = MPU.getAngleZ() * PI / 180;
    // 存储当前前进位置
    float last_distance_X = 0;
    // 静止次数
    uint8_t count = 0;

    while(1)
    {
      // 获取 左右轮的旋转角度
      float left_rotation = encoder.getLeftRotation() - Initial_left_rotation;
      float right_rotation = encoder.getRightRotation() - Initial_right_rotation;
      

      // 计算当前 位置和 角度
      encoder_odometry.update(left_rotation, right_rotation);
      MPU.update();

      float distance_X = encoder_odometry.getX();
      float distance_Y = encoder_odometry.getY();
      float angle_H = MPU.getAngleZ() * PI / 180 - Initial_angle;


      // PID计算输出值
      float output = pid_X.calculate(target_distance, distance_X, dt);
      
      if(output >= 0)
      {
        digitalWrite(MOTOR_LEFT_DIR,LOW);
        digitalWrite(MOTOR_RIGHT_DIR, HIGH);
      }
      else
      {
        digitalWrite(MOTOR_LEFT_DIR,HIGH);
        digitalWrite(MOTOR_RIGHT_DIR, LOW);
      }
      
      float output_pwm = constrain(fabs(output), 0, Front_PWM_max);


      // PID计算offset
      float output_offset_rate = constrain(1 - fabs(pid_gyro.calculate(0, angle_H, dt)), 0, 1);
      
      float output_pwm_adjusted = output_pwm * output_offset_rate;


      if(angle_H >= 0) 
      {
          if(output >= 0)
          {
            analogWrite(MOTOR_LEFT_PWM, output_pwm);
            analogWrite(MOTOR_RIGHT_PWM, output_pwm_adjusted);
          }
          else
          {
            analogWrite(MOTOR_LEFT_PWM, output_pwm_adjusted);
            analogWrite(MOTOR_RIGHT_PWM, output_pwm);
          }
      }
      else
      {
          if(output >= 0)
          {
            analogWrite(MOTOR_LEFT_PWM, output_pwm_adjusted);
            analogWrite(MOTOR_RIGHT_PWM, output_pwm);
          }
          else
          {
            analogWrite(MOTOR_LEFT_PWM, output_pwm);
            analogWrite(MOTOR_RIGHT_PWM, output_pwm_adjusted);
          }
      }



   
      // 静止判断
      if(fabs(distance_X-last_distance_X) < 1) {count ++;}
      else {count = 0;}
      if (count >= 10) 
      {
        analogWrite(MOTOR_LEFT_PWM, 0);
        analogWrite(MOTOR_RIGHT_PWM, 0);
        break;
      }
      // 存储本次自转位置
      last_distance_X = distance_X;


      // 显示当前陀螺仪角度
      ShowData(MPU.getAngleZ());
      // 保证单次step为50ms
      while(millis() - lastMillis < dt*1000) {;}
    }

  
  }




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




  // 转弯
  void Turning(MPU6050& MPU, float target_rotation, float P, float I, float D)
  {
    float lastMillis = millis();

    // 分别用于控制 自转位置 和 两轮旋转位置 的 PID
    PID pid_turning(P, I, D);
    
    // 起始位置(GCF)
    MPU.update();
    float Initial_angle = MPU.getAngleZ() * PI / 180;
    // 记录本次旋转位置
    float last_turning_rotation = 0;
    // 静止次数
    uint8_t count = 0;

    while(1)
    {
      // 计算 当前自转位置
      MPU.update();
      float turning_rotation = MPU.getAngleZ() * PI / 180 - Initial_angle;
      // PID计算输出值
      float output = pid_turning.calculate(target_rotation, turning_rotation, dt);

      
      // 输出调节电机
      if(output >= 0)
      {
        digitalWrite(MOTOR_LEFT_DIR,HIGH);
        digitalWrite(MOTOR_RIGHT_DIR, HIGH);
      }
      else
      {
        digitalWrite(MOTOR_LEFT_DIR,LOW);
        digitalWrite(MOTOR_RIGHT_DIR, LOW);
        output = -output;
      }
      
      analogWrite(MOTOR_LEFT_PWM, constrain(output, 0, Turning_PWM_max));
      analogWrite(MOTOR_RIGHT_PWM, constrain(output, 0, Turning_PWM_max));


  
      // 静止判断
      if(fabs(turning_rotation - last_turning_rotation) < 0.01) {count ++;}
      else {count = 0;}
      if (count >= 5) 
      {
        analogWrite(MOTOR_LEFT_PWM, 0);
        analogWrite(MOTOR_RIGHT_PWM, 0);
        break;
      }
      // 存储本次自转位置
      last_turning_rotation = turning_rotation;


      // 显示当前陀螺仪角度
      ShowData(MPU.getAngleZ());
      // 保证单次step为50ms
      while(millis() - lastMillis < dt*1000) {;}
    }

  }
  
  
};

}

