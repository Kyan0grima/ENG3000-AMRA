// encoder.c
//
// Encoder handling extracted from racer.c and re-mapped to:
//   Motor 1: A = D12 (PB4), B = D11 (PB3)
//   Motor 2: A = D2  (PD2), B = D3  (PD3)
//
// D12/D11 live on PORTB  -> handled by PCINT0_vect (pin-change bank 0)
// D2/D3   live on PORTD  -> handled by PCINT2_vect (pin-change bank 2)
//
// Both channels are quadrature, so both edges of both A/B pins must be
// watched -> pin-change interrupts (not INT0/INT1, which only fire on one
// pin each).

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

// ---- Pin masks -------------------------------------------------------
// Motor 1: PB3 (D11) and PB4 (D12)
#define ENC1_MASK_B   0x18   // bits 3 and 4
#define ENC1_SHIFT    3

// Motor 2: PD2 (D2) and PD3 (D3)
#define ENC2_MASK_D   0x0C   // bits 2 and 3
#define ENC2_SHIFT    2

// ---- State -------------------------------------------------------
volatile uint8_t motor1_ticks = 0;
volatile uint8_t motor2_ticks = 0;

static uint8_t m1_old_state = 0;
static uint8_t m2_old_state = 0;
static uint8_t last_m1 = 0;
static uint8_t last_m2 = 0;

// Separate "last seen" snapshots for the distance accumulator, so it can be
// called at a different rate than update_motor_speeds() without the two
// consuming each other's ticks.
static uint8_t last_m1_dist = 0;
static uint8_t last_m2_dist = 0;
volatile int16_t motor1_count = 0;
volatile int16_t motor2_count = 0;

// Output speed variables, in encoder ticks per second.
// Read these from your main/control code.
volatile int16_t motor1_speed = 0;
volatile int16_t motor2_speed = 0;

// Running total distance, in encoder ticks, signed to reflect direction.
// Updated by update_motor_distances(). Read these from your main/control code.
volatile int16_t motor1_dist = 0;
volatile int16_t motor2_dist = 0;

volatile int16_t car_dist = 0;

// 16-state quadrature lookup table: [old<<2 | new] -> direction
static const int8_t ENC_LUT[16] = {
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0
};

// ---- Setup -------------------------------------------------------
void encoder_init(void) {
    // Motor 1: PB3, PB4 as inputs with pull-ups
    DDRB  &= ~((1 << DDB3) | (1 << DDB4));
    PORTB |=  (1 << PORTB3) | (1 << PORTB4);

    // Motor 2: PD2, PD3 as inputs with pull-ups
    DDRD  &= ~((1 << DDD2) | (1 << DDD3));
    PORTD |=  (1 << PORTD2) | (1 << PORTD3);

    // Enable pin-change interrupt banks 0 (PORTB) and 2 (PORTD)
    PCICR |= (1 << PCIE0) | (1 << PCIE2);

    // Unmask the specific pins
    PCMSK0 |= (1 << PCINT3) | (1 << PCINT4);   // PB3, PB4 (D11, D12)
    PCMSK2 |= (1 << PCINT18) | (1 << PCINT19); // PD2, PD3 (D2, D3)

    // Seed initial states so the first tick isn't miscalculated
    m1_old_state = (PINB & ENC1_MASK_B) >> ENC1_SHIFT;
    m2_old_state = (PIND & ENC2_MASK_D) >> ENC2_SHIFT;
}

// ---------------------------------------------------------
// Motor 1 encoder ISR (D11/D12, PORTB pin-change bank)
// ---------------------------------------------------------
ISR(PCINT0_vect) {
    uint8_t new_state = (PINB & ENC1_MASK_B) >> ENC1_SHIFT;
    uint8_t index = (m1_old_state << 2) | new_state;
    motor1_ticks += ENC_LUT[index];
    m1_old_state = new_state;
}

// ---------------------------------------------------------
// Motor 2 encoder ISR (D2/D3, PORTD pin-change bank)
// ---------------------------------------------------------
ISR(PCINT2_vect) {
    uint8_t new_state = (PIND & ENC2_MASK_D) >> ENC2_SHIFT;
    uint8_t index = (m2_old_state << 2) | new_state;
    motor2_ticks += ENC_LUT[index];
    m2_old_state = new_state; 
}

// ---- Compact speed reader -------------------------------------------------------
// Call this every time round your main loop, passing how many microseconds
// have elapsed since the previous call (e.g. from micros()). Works fine with
// a jittery/variable loop rate, unlike a hard-coded ticks-per-loop constant.
// Converts the tick delta since the last call into ticks-per-second and
// stores it in motor1_speed / motor2_speed. Assumes deltas stay within
// +/-127 between calls (true as long as this is called at least a few
// hundred times a second).
void update_motor_speeds(uint32_t dt_us) {
    if (dt_us == 0) return; // guard against div-by-zero on the very first call

    uint8_t m1 = motor1_ticks;
    uint8_t m2 = motor2_ticks;

    int8_t d1 = (int8_t)(m1 - last_m1);
    int8_t d2 = (int8_t)(m2 - last_m2);

    last_m1 = m1;
    last_m2 = m2;

    // ticks/sec = ticks * 1,000,000 / dt_us
    motor1_speed = -((int16_t)(((int32_t)d1 * 1000000L) / (int32_t)dt_us));
    motor2_speed = ((int16_t)(((int32_t)d2 * 1000000L) / (int32_t)dt_us));
}

// ---- Cumulative distance reader -------------------------------------------------------
// Call this periodically (e.g. once per main loop iteration, or any rate you
// like -- it doesn't need to be tied to update_motor_speeds()). It folds the
// tick delta since the last call into a running int16_t total in
// motor1_dist / motor2_dist, so the caller gets an ever-growing signed tick
// count instead of the raw ISR counters, which are only 8-bit and wrap
// silently.
//
// Same direction convention as update_motor_speeds(): motor1 is negated so
// that "forward" reads as positive on both motors, matching how the motors
// are physically mounted facing each other.
//
// Note: motor1_dist/motor2_dist are int16_t, so they will themselves wrap
// around after +/-32767 ticks. If you need an unbounded odometer, widen
// these to int32_t (and adjust the LUT-driven ticks counters' usage
// elsewhere accordingly -- the 8-bit ISR counters and (u)int8_t deltas here
// are unaffected either way).
void update_motor_distances(void) {
    uint8_t m1 = motor1_ticks;
    uint8_t m2 = motor2_ticks;

    int8_t d1 = (int8_t)(m1 - last_m1_dist);
    int8_t d2 = (int8_t)(m2 - last_m2_dist);

    last_m1_dist = m1;
    last_m2_dist = m2;

    motor1_count += -d1;
    motor2_count += d2;

    motor1_dist = motor1_count/24;
    motor2_dist = motor2_count/24;

    car_dist = (motor1_dist+motor2_dist)/2;
}