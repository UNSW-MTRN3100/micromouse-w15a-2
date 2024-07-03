#include <Arduino.h>

namespace mtrn3100 {
class EncoderOdometry {
public:
    EncoderOdometry(float radius, float wheelBase) : x(0), y(0), h(0), R(radius), L(wheelBase), lastLPos(0), lastRPos(0) {}

    //TODO: 完成这个函数
    void update(float leftValue, float rightValue) {
        // 计算每个轮子的旋转增量
        float dL = leftValue - lastLPos;
        float dR = -(rightValue - lastRPos);

        // 更新最后的编码器位置
        lastLPos = leftValue;
        lastRPos = rightValue;

        // 计算每个轮子的行进距离
        float leftDistance = dL * R;
        float rightDistance = dR * R;

        // 计算机器人的线位移和角位移
        float dC = (leftDistance + rightDistance) / 2.0; // 中心线位移
        float dTheta = (rightDistance - leftDistance) / L; // 朝向的变化

      
        // 计算新的位置
        x += dC * cos(h);
        y += dC * sin(h);
          // 更新机器人的朝向
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
