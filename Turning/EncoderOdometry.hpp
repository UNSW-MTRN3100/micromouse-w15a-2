#include <Arduino.h>

namespace mtrn3100 {
class EncoderOdometry {
public:
    EncoderOdometry(float radius, float wheelBase) : x(0), y(0), h(0), R(radius), L(wheelBase), lastLPos(0), lastRPos(0) {}

    void update(float leftValue, float rightValue) {
        // Calculate the change in encoder values
        float delta_left = leftValue - lastLPos;
        float delta_right = -(rightValue - lastRPos);

        // Update the last positions
        lastLPos = leftValue;
        lastRPos = rightValue;

        // Calculate the distance each wheel has traveled
        float dL = R * delta_left;
        float dR = R * delta_right;

        // Calculate the change in position and orientation
        float dC = (dL + dR) / 2.0;
        float dTheta = (dR - dL) / L;

        // Update x, y, h based on the kinematics equations
        x += dC * cos(h);
        y += dC * sin(h);
        h += dTheta;
    }

    float getX() const { return x; }
    float getY() const { return y; }
    float getH() const { return h; }

private:
    float x, y, h;
    const float R, L;
    float lastLPos, lastRPos;
};

}

