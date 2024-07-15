#pragma once

#include <Arduino.h>
#include "Config.hpp"


namespace mtrn3100 {


class DualEncoder {

public:
    
    DualEncoder() : mot1_int(EN_1_A), mot1_dir(EN_1_B), mot2_int(EN_2_A), mot2_dir(EN_2_B) {
        instance = this;

        pinMode(mot1_int, INPUT_PULLUP);
        pinMode(mot1_dir, INPUT_PULLUP);
        pinMode(mot2_int, INPUT_PULLUP);
        pinMode(mot2_dir, INPUT_PULLUP);
        
        attachInterrupt(digitalPinToInterrupt(mot1_int), DualEncoder::readLeftEncoderISR, RISING);
        attachInterrupt(digitalPinToInterrupt(mot2_int), DualEncoder::readRightEncoderISR, RISING);
    }


    // 编码器中断
    void readLeftEncoder() {
        noInterrupts();
        direction = digitalRead(mot1_dir) ? 1 : -1;
        l_count += direction;
        interrupts();
    }    
    void readRightEncoder() {
        noInterrupts();
        direction = digitalRead(mot2_dir) ? -1 : 1;
        r_count += direction;
        interrupts();
    }





    // 获取左右轮旋转角度
    float getLeftRotation() {
        return (static_cast<float>(l_count) / counts_per_revolution_left ) * 2 * PI;
    }
    float getRightRotation() {
        return (static_cast<float>(r_count) / counts_per_revolution_right ) * 2 * PI;
    }






// 直接绑定的 ISR
private:
    static void readLeftEncoderISR() {
        if (instance != nullptr) {
            instance->readLeftEncoder();
        }
    }
   static void readRightEncoderISR() {
        if (instance != nullptr) {
            instance->readRightEncoder();
        }
    }





public:
    const uint8_t mot1_int,mot1_dir,mot2_int,mot2_dir;
    volatile int8_t direction;
    volatile long l_count = 0;
    volatile long r_count = 0;
    uint32_t prev_time;
    bool read = false;



private:
    static DualEncoder* instance;
};



DualEncoder* DualEncoder::instance = nullptr;



}  // namespace mtrn3100
