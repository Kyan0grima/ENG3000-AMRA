#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/* Outputs (standard units) */
extern float motor1_speed_rpm;
extern float motor2_speed_rpm;
extern float motor1_dist_mm;
extern float motor2_dist_mm;
extern float car_dist_mm;

void encoder_init(void);
void update_motor_speeds(uint32_t dt_us);
void update_motor_distances(void);
void reset_motor_distances(void);

#endif
