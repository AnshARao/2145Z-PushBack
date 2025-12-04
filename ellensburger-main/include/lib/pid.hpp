
#include <cmath>
class PID {

private:

    float kp;
    float ki;
    float kd;
    float prev_error = 0;
    float integral = 0;

public:

    PID(float p, float i, float d) : kp(p), ki(i), kd(d) {}

    float compute(float error) {
        if (std::signbit(error) != std::signbit(prev_error)) { integral = 0; }
        else { integral += error; }

        float derivative = error - prev_error;
        prev_error = error;
        
        return error * kp + integral * ki + derivative * kd;
    }

};