// encoder.c
//
// Encoder handling:
//   Motor 1: A = D12 (PB4), B = D11 (PB3)
//   Motor 2: A = D2  (PD2), B = D3  (PD3)
//
// Outputs are in standard units:
//   motorN_speed_rpm  : wheel speed in revolutions per minute (signed)
//   motorN_dist_mm    : distance travelled in millimetres (signed)
//   car_dist_mm       : average of both wheels, in millimetres

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include "encoder.h"

// ---- Mechanical parameters ------------------------------------------
#define WHEEL_DIAMETER_MM   43.0f
// CHECK: ticks per wheel rev after 4x quadrature decoding
// (= encoder CPR * 4 * gear ratio). Turn the wheel exactly one rev and
// read motorN_count to verify.
#define TICKS_PER_REV       360.0f
#define PI_F                3.14159265f

#define WHEEL_CIRC_MM       (PI_F * WHEEL_DIAMETER_MM)
#define MM_PER_TICK         (WHEEL_CIRC_MM / TICKS_PER_REV)
#define RPM_NUMERATOR       (60000000.0f / TICKS_PER_REV)       // ticks/us -> rpm

// CHECK: motor 1 and motor 2 build their 2-bit state with opposite bit order
// (see ENC1/ENC2 masks), so these signs may be compensating for that rather
// than for mounting. Spin both wheels forward by hand: both distances must
// go positive. If one is wrong, flip its sign here.
#define M1_DIR_SIGN         (-1)
#define M2_DIR_SIGN         (1)

// ---- Pin masks -------------------------------------------------------
#define ENC1_MASK_B   0x18   // PB3, PB4
#define ENC1_SHIFT    3
#define ENC2_MASK_D   0x0C   // PD2, PD3
#define ENC2_SHIFT    2

// ---- State -------------------------------------------------------
// 16-bit ISR counters (wrap silently, deltas stay valid up to +/-32767 ticks)
static volatile int16_t motor1_ticks = 0;
static volatile int16_t motor2_ticks = 0;

static uint8_t m1_old_state = 0;
static uint8_t m2_old_state = 0;
static int16_t last_m1 = 0;
static int16_t last_m2 = 0;
static int16_t last_m1_dist = 0;
static int16_t last_m2_dist = 0;

// Internal signed tick totals
static int32_t motor1_count = 0;
static int32_t motor2_count = 0;

// ---- Outputs (only written from main context, so not volatile) -------
float motor1_speed_rpm = 0.0f;
float motor2_speed_rpm = 0.0f;

float motor1_dist_mm = 0.0f;
float motor2_dist_mm = 0.0f;
float car_dist_mm    = 0.0f;

// 16-state quadrature lookup table: [old<<2 | new] -> direction
static const int8_t ENC_LUT[16] = {
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0
};

// 16-bit reads are not atomic on AVR
static inline int16_t read_ticks(volatile int16_t *p) {
    uint8_t s = SREG;
    cli();
    int16_t v = *p;
    SREG = s;
    return v;
}

// ---- Setup -------------------------------------------------------
void encoder_init(void) {
    DDRB  &= ~((1 << DDB3) | (1 << DDB4));
    PORTB |=  (1 << PORTB3) | (1 << PORTB4);

    DDRD  &= ~((1 << DDD2) | (1 << DDD3));
    PORTD |=  (1 << PORTD2) | (1 << PORTD3);

    // Read initial state BEFORE enabling interrupts
    m1_old_state = (PINB & ENC1_MASK_B) >> ENC1_SHIFT;
    m2_old_state = (PIND & ENC2_MASK_D) >> ENC2_SHIFT;

    PCMSK0 |= (1 << PCINT3)  | (1 << PCINT4);
    PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
    PCICR  |= (1 << PCIE0) | (1 << PCIE2);
}

ISR(PCINT0_vect) {
    uint8_t new_state = (PINB & ENC1_MASK_B) >> ENC1_SHIFT;
    uint8_t index = (m1_old_state << 2) | new_state;
    motor1_ticks += ENC_LUT[index];
    m1_old_state = new_state;
}

ISR(PCINT2_vect) {
    uint8_t new_state = (PIND & ENC2_MASK_D) >> ENC2_SHIFT;
    uint8_t index = (m2_old_state << 2) | new_state;
    motor2_ticks += ENC_LUT[index];
    m2_old_state = new_state;
}

// ---- Speed reader (RPM) ----------------------------------------------
// rpm = (ticks / TICKS_PER_REV) / (dt_us / 60e6)
// NOTE: one tick over ~22 ms is ~7.6 rpm of quantization. If the speed PID
// is noisy, low-pass the result, e.g.
//   motor1_speed_rpm += 0.5f * (new_rpm - motor1_speed_rpm);
void update_motor_speeds(uint32_t dt_us) {
    if (dt_us == 0) return;

    int16_t m1 = read_ticks(&motor1_ticks);
    int16_t m2 = read_ticks(&motor2_ticks);

    int16_t d1 = m1 - last_m1;
    int16_t d2 = m2 - last_m2;

    last_m1 = m1;
    last_m2 = m2;

    float inv_dt = 1.0f / (float)dt_us;

    motor1_speed_rpm = M1_DIR_SIGN * (float)d1 * RPM_NUMERATOR * inv_dt;
    motor2_speed_rpm = M2_DIR_SIGN * (float)d2 * RPM_NUMERATOR * inv_dt;
}

// ---- Distance reader (mm) ----------------------------------------------
void update_motor_distances(void) {
    int16_t m1 = read_ticks(&motor1_ticks);
    int16_t m2 = read_ticks(&motor2_ticks);

    int16_t d1 = m1 - last_m1_dist;
    int16_t d2 = m2 - last_m2_dist;

    last_m1_dist = m1;
    last_m2_dist = m2;

    motor1_count += (int32_t)M1_DIR_SIGN * d1;
    motor2_count += (int32_t)M2_DIR_SIGN * d2;

    motor1_dist_mm = (float)motor1_count * MM_PER_TICK;
    motor2_dist_mm = (float)motor2_count * MM_PER_TICK;
    car_dist_mm    = (motor1_dist_mm + motor2_dist_mm) * 0.5f;
}

void reset_motor_distances(void) {
    // read_ticks() saves/restores SREG, so this no longer forces sei()
    last_m1_dist = read_ticks(&motor1_ticks);
    last_m2_dist = read_ticks(&motor2_ticks);

    motor1_count = 0;
    motor2_count = 0;

    motor1_dist_mm = 0.0f;
    motor2_dist_mm = 0.0f;
    car_dist_mm    = 0.0f;
}
