#include "Arduino.h"
#include "Infinity_PID.h"

/**
 * Constructor
 * @param flPin front left motor pin
 * @param frPin front right motor pin
 * @param blPin back left motor pin
 * @param brPin back right motor pin
 */
Q_PID::Q_PID(int flPin, int frPin, int blPin, int brPin) {
  this->fl->attach(flPin);
  this->fr->attach(frPin);
  this->bl->attach(blPin);
  this->br->attach(brPin);
}

/**
 * Sets lower and upper limits on PID output
 * @param min minimum limit
 * @param max maximum limit
 */
void Q_PID::limit_output(float min, float max) {
  this->min_limit = min;
  this->max_limit = max;
}

/**
 * Adjusts PID constants
 * @param array containing x, y, and z proportional constants
 * @param array containing x, y, and z integral constants
 * @param array containing x, y, and z derivative constants
 */
void Q_PID::tune(float p[3], float i[3], float d[3]) {
  memcpy(this->K_P, p, sizeof(this->K_P));
  memcpy(this->K_I, i, sizeof(this->K_I));
  memcpy(this->K_D, d, sizeof(this->K_D));
}

/**
 * Updates outputs with PID
 * @param array containing x, y, and z target angular velocities
 * @param array containing x, y, and z actual angular velocities
 */
void Q_PID::update(float target[3], float actual[3]) {
  for (int i = 0; i < 3; i++) {
    float error = target[i] - actual[i];
    float now = millis();
    float dt = now - this->oldTimes[i];
    this->sums[i] += dt * error;
    float ddt = (error - this->oldErrors[i]) / dt;
    this->outputs[i] = 0.5 * (this->K_P[i] * error
                        + this->K_I[i] * this->sums[i]
                        + this->K_D[i] * ddt);
    this->oldErrors[i] = error;
    this->oldTimes[i] = now;
  }
}

/**
 * Updates outputs with PID (for use with timer interrupt)
 * @param interrupt time in ms
 * @param array containing x, y, and z target angular velocities
 * @param array containing x, y, and z actual angular velocities
 */
void Q_PID::interrupt_update(float dt, float target[3], float actual[3]) {
  for (int i = 0; i < 3; i++) {
    float error = target[i] - actual[i];
    this->sums[i] += dt * error;
    float ddt = (error - this->oldErrors[i]) / dt;
    this->outputs[i] = 0.5 * (this->K_P[i] * error
                        + this->K_I[i] * this->sums[i]
                        + this->K_D[i] * ddt);
    this->oldErrors[i] = error;
  }
}

/**
 * //  CW    CCW
 * //  CCW    CW
 * Calculates motor speeds based on throttle and PID
 * @param throttle value in range [0, 1]
 */
void Q_PID::act(float throttle) {
  this->motor_speeds[FL] -= this->outputs[0];
  this->motor_speeds[BL] -= this->outputs[0];
  this->motor_speeds[FR] += this->outputs[0];
  this->motor_speeds[BR] += this->outputs[0];

  this->motor_speeds[FL] += this->outputs[1];
  this->motor_speeds[BL] -= this->outputs[1];
  this->motor_speeds[FR] += this->outputs[1];
  this->motor_speeds[BR] -= this->outputs[1];

  this->motor_speeds[FL] += this->outputs[2];
  this->motor_speeds[BL] -= this->outputs[2];
  this->motor_speeds[FR] -= this->outputs[2];
  this->motor_speeds[BR] += this->outputs[2];

  for (int i = 0; i < 4; i++) {
    this->motor_speeds[i] = constrain(this->motor_speeds[i], 0, 1);
  }

  this->fl->writeMicroseconds(map(this->motor_speeds[FL], 0, 1, this->min_limit, this->max_limit));
  this->fr->writeMicroseconds(map(this->motor_speeds[FR], 0, 1, this->min_limit, this->max_limit));
  this->bl->writeMicroseconds(map(this->motor_speeds[BL], 0, 1, this->min_limit, this->max_limit));
  this->br->writeMicroseconds(map(this->motor_speeds[BR], 0, 1, this->min_limit, this->max_limit));
}
