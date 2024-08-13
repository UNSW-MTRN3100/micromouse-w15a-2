#pragma once


// 电机编码器引脚
#define EN_1_A 2
#define EN_1_B 7
#define EN_2_A 3
#define EN_2_B 8
// 电机驱动引脚
#define MOTOR_LEFT_PWM 11
#define MOTOR_LEFT_DIR 12
#define MOTOR_RIGHT_PWM 9
#define MOTOR_RIGHT_DIR 10


const float R = 16;
const float b = 100;

// 大轮
// const uint16_t distance_per_cell = 180;
// 小轮(demo)
const uint16_t distance_per_cell = 237;

uint16_t counts_per_revolution_left = 700;
uint16_t counts_per_revolution_right = 700;


const uint8_t Front_PWM_max = 200;
const uint8_t Turning_PWM_max = 150;
// const uint8_t Front_PWM_min = 15;
// const uint8_t Turning_PWM_min = 15;


const float dt = 0.05;

const float P_front = 25;
const float I_front = 0;
const float D_front = 0.7;

const float P_gyro = 15;
const float I_gyro = 0;
const float D_gyro = 0;

const float P_turning = 500;
const float I_turning = 0;
const float D_turning = 3;

const float P_correction = 1500;
const float I_correction = 0;
const float D_correction = 0;

