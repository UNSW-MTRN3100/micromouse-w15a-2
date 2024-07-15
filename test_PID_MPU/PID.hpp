#pragma once



class PID {

public:
    
    PID(float kp, float ki, float kd) : Kp(kp), Ki(ki), Kd(kd), integral(0), previous_error(0) {}

    float calculate(float setpoint, float measured_value, float dt) {
        float error = setpoint - measured_value;
        integral += error * dt;
        float derivative = (error - previous_error) / dt;
        float output = Kp * error + Ki * integral + Kd * derivative;
        previous_error = error;
        return output;
    }

private:
    float Kp, Ki, Kd;
    float integral;
    float previous_error;
};

