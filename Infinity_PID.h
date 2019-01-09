#ifndef Q_PID_H
#define Q_PID_H

#include "Arduino.h"
#include "Servo.h"

#define FL 0
#define FR 1
#define BL 2
#define BR 3

class Q_PID {
  public:
    Q_PID(int, int, int, int);
    void limit_output(float, float);
    void update(float[3], float[3]); // For use without timer interrupts
    void interrupt_update(float, float[], float[]); // For use with timer interrupts
    void act(float);
    void tune(float[3], float[3], float[3]);

  private:
    Servo* fl;
    Servo* fr;
    Servo* bl;
    Servo* br; // hardware pointers

    float motor_speeds[4]; // motor speeds
    float min_limit, max_limit; // motor speed limits (PWM duty cycle in ms)

    float K_I[3] = {0};
    float K_P[3] = {0};
    float K_D[3] = {0};
    float sums[3] = {0};
    float outputs[3] = {0};
    float oldErrors[3] = {0};
    float oldTimes[3] = {0}; // PID variables (initialized to 0)
};

#endif
