/*
 * Line follower: map lap + map-based following.
 *
 * HOW IT WORKS
 *   1. FIND_START : drive slowly on the sensor until the first start-of-straight
 *                   marker (all 8 sensors dark) is seen.
 *   2. MAPPING    : one full lap at MAP_SPEED_RPM, sensor-steered. The track is
 *                   cut into MARKS_PER_LAP segments (marker -> next marker). Each
 *                   segment is stored as a list of BIN_MM bins holding the
 *                   curvature the wheels actually drove (from encoder L/R
 *                   difference). The lap ends when the first marker is seen again.
 *   3. Profile    : curvature -> speed per bin (lateral-accel limit, then a
 *                   braking pass and an acceleration pass around the closed loop).
 *   4. RUN        : every later lap steers by feed-forward from the map
 *                   (curvature at the lookahead point), runs the stored speed
 *                   profile, and uses the line sensor only as a limited trim
 *                   (speed-scaled, see TRIM_LIMIT_*). Position along the map is
 *                   (segment, distance since last marker); every marker re-syncs
 *                   it, and a missed marker is dead-reckoned past.
 *
 * Curvature sign: positive = left turn (right wheel faster), same as the old
 * "positive error = line on the left" convention.
 *
 * CHANGES IN THIS VERSION
 *   - Fixed ~4 ms control loop (was ~27 ms: 20 ms delay + sensor + blocking printf).
 *   - printf throttled (every PRINT_EVERY loops) and off in RUN by default.
 *   - SPEED_CAP for stepping the run speed up gradually.
 *   - Map cleanup: SMOOTH_PASSES smoothing passes + CURV_DEADBAND so straights stay straight.
 *   - More conservative A_LAT / A_BRAKE defaults (measure A_BRAKE for your car!).
 *   - Line trim limit scales with speed; separate run-mode line gains.
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#define BAUD     115200
#define BAUD_TOL 3          /* 115200 @ 16 MHz is ~2.1% off; default tolerance is 2% */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/setbaud.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "encoder.h"


#define DIRA_PORT  PORTB
#define DIRA_DDR   DDRB
#define DIRA_BIT   PB0   //D8

#define DIRB_PORT  PORTD
#define DIRB_DDR   DDRD
#define DIRB_BIT   PD7   //D7

#define PWM_DDR    DDRB
#define PWMB_BIT   PB2   /* OC1B  - D10*/
#define PWMA_BIT   PB1   /* OC1A  - D9*/

#define CTRL_PORT  PORTD
#define CTRL_DDR   DDRD
#define CTRL_BIT   PD6  //D6


#define MAX_SPEED 900          /* rpm ceiling, per wheel */

/* ---- Loop timing / debug output (NEW) ---- */
#define LOOP_PERIOD_US    4000UL   /* fixed control period. Sensor read takes up to 2.5 ms,
                                    * so keep this >= ~4000 or the loop just free-runs.      */
#define PRINT_EVERY       10       /* print once per N loops                                 */
#define PRINT_IN_RUN      0        /* 1 = also print during MODE_RUN (costs ~4 ms per print) */

/* ---- Wheel-speed measurement window (NEW) ----
 * The encoder speed estimate is quantised: over a short window it is only a few
 * ticks, so rpm jumps around and the speed PID (kp 1.4) turns that into PWM chatter.
 * Measure speed over LOOP_PERIOD_US * SPEED_UPDATE_EVERY (= 20 ms, same as the old
 * loop) and hold it in between. Distances are still updated every loop. */
#define SPEED_UPDATE_EVERY 5

/* ---- Run-speed cap (NEW) ----
 * Step this up: 200 -> 300 -> 400 -> 500 -> MAX_SPEED (= uncapped).
 * Note where on the track it first leaves the line. */
#define SPEED_CAP         900.0f

int lap = 1;                   /* 1 = mapping lap, 2+ = map laps */
int checkpoint = 0;

/* ======================= Sensors ================================ */
typedef struct {
    volatile uint8_t *ddr;
    volatile uint8_t *port;
    volatile uint8_t *pin;
    uint8_t mask;
} io_pin_t;

static const io_pin_t sensorPins[8] = {
    { &DDRC, &PORTC, &PINC, (1 << 0) }, //A0
    { &DDRC, &PORTC, &PINC, (1 << 1) }, //A1
    { &DDRC, &PORTC, &PINC, (1 << 2) }, //A2
    { &DDRC, &PORTC, &PINC, (1 << 3) }, //A3
    { &DDRC, &PORTC, &PINC, (1 << 4) }, //A4
    { &DDRC, &PORTC, &PINC, (1 << 5) }, //A5
    { &DDRD, &PORTD, &PIND, (1 << 4) }, //D4
    { &DDRD, &PORTD, &PIND, (1 << 5) }, //D5
};

#define TIMEOUT_US 2500UL

static volatile uint32_t rawValues[8];
static volatile uint8_t  binaryValues[8];
static uint16_t avgVal = 0;


/* ======================= micros() via Timer0 ===================== */
static volatile uint32_t timer0_overflow_count = 0;

ISR(TIMER0_OVF_vect) {
    timer0_overflow_count++;
}

static void timer0_micros_init(void) {
    TCCR0A = 0;
    TCCR0B = (1 << CS01);        /* prescaler 8 -> 0.5 us per tick */
    TIMSK0 = (1 << TOIE0);
}

static uint32_t micros(void) {
    uint8_t  sreg = SREG;
    uint32_t ovf;
    uint8_t  tcnt;

    cli();
    ovf  = timer0_overflow_count;
    tcnt = TCNT0;
    if ((TIFR0 & (1 << TOV0)) && tcnt < 0xFF) {
        ovf++;
    }
    SREG = sreg;

    /* 256 ticks * 0.5 us = 128 us per overflow. Full 32-bit range (no early wrap). */
    return (ovf << 7) + (tcnt >> 1);
}


/* ======================= Hardware PWM: Timer1 ===================== */
static void timer1_pwm_init(void) {
    PWM_DDR |= (1 << PWMA_BIT) | (1 << PWMB_BIT);
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
    OCR1A = 0;
    OCR1B = 0;
}

/* CHECK: names are crossed relative to the pin defines above
 * (PWMA_BIT = OC1A, but setPWMA writes OCR1B). Left as-is because it is
 * correct only if it matches your wiring. If your left motor is on D9,
 * swap these. */
static inline void setPWMA(uint8_t duty) { OCR1B = duty; }
static inline void setPWMB(uint8_t duty) { OCR1A = duty; }


/* ======================= UART / printf ============================ */
static int uart_putchar(char c, FILE *stream) {
    if (c == '\n') uart_putchar('\r', stream);
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
    return 0;
}

static FILE uart_output;

static void uart_init(void) {
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
#if USE_2X
    UCSR0A |= (1 << U2X0);
#else
    UCSR0A &= ~(1 << U2X0);
#endif
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

    fdev_setup_stream(&uart_output, uart_putchar, NULL, _FDEV_SETUP_WRITE);
    stdout = &uart_output;
}


/* ======================= Sensor read =============================== */
static void ReadSensorsRaw(void) {
    uint8_t  i;
    uint8_t  pending = 0xFF;      /* bit i set = sensor i still high */
    uint32_t start, elapsed, total = 0;

    for (i = 0; i < 8; i++) {
        *sensorPins[i].ddr  |= sensorPins[i].mask;
        *sensorPins[i].port |= sensorPins[i].mask;
    }
    _delay_us(10);

    for (i = 0; i < 8; i++) {
        *sensorPins[i].ddr  &= ~sensorPins[i].mask;
        *sensorPins[i].port &= ~sensorPins[i].mask;
    }

    start = micros();
    while (pending) {
        elapsed = micros() - start;
        if (elapsed >= TIMEOUT_US) break;
        for (i = 0; i < 8; i++) {
            if ((pending & (1 << i)) && !(*sensorPins[i].pin & sensorPins[i].mask)) {
                rawValues[i] = elapsed;
                pending &= ~(1 << i);
            }
        }
    }

    for (i = 0; i < 8; i++) {
        if (pending & (1 << i)) rawValues[i] = TIMEOUT_US;
        total += rawValues[i];
    }
    avgVal = (uint16_t)(total / 8);

    for (i = 0; i < 8; i++) {
        binaryValues[i] = (rawValues[i] < avgVal) ? 1 : 0;
    }
}


/* ======================= Motor drivers ============================= */
static void forward(uint8_t L_Speed, uint8_t R_Speed) {
    DIRA_PORT |= (1 << DIRA_BIT);
    DIRB_PORT |= (1 << DIRB_BIT);
    setPWMA(L_Speed);
    setPWMB(R_Speed);
}

__attribute__((unused))
static void backward(uint8_t L_Speed, uint8_t R_Speed) {
    DIRA_PORT &= ~(1 << DIRA_BIT);
    DIRB_PORT &= ~(1 << DIRB_BIT);
    setPWMA(L_Speed);
    setPWMB(R_Speed);
}


/* ======================= Line position ============================= */
static int32_t lastError  = 0;
static int32_t lineOffset = 0;   /* measured "centered" bias, subtracted from every reading */
int32_t sum = 0;


static int32_t computeLinePosition(void) {
    static const int32_t weight[8] = { 3500, 2500, 1500, 500,
                                         -500,  -1500,  -2500,  -3500 };
    int64_t weightedSum = 0;
    sum = 0;


    for (uint8_t i = 0; i < 8; i++) {
        int32_t darkness = (int32_t)TIMEOUT_US - (int32_t)rawValues[i];
        if (darkness < 0) darkness = 0;
        weightedSum += (int64_t)weight[i] * darkness;
        sum += darkness;
    }


    if (sum < 300) {
        return (lastError < 0) ? -3500 : 3500;
    }


    return (int32_t)(weightedSum / sum) - lineOffset;
}


/* Run once at startup with the robot centered on the line.
 * Averages several raw readings (offset NOT applied here, since
 * lineOffset is what we're solving for) to get a stable zero point. */
static void calibrateLineOffset(void) {
    int64_t sum = 0;
    const uint8_t samples = 32;


    lineOffset = 0;


    for (uint8_t s = 0; s < samples; s++) {
        ReadSensorsRaw();
        sum += computeLinePosition();
        _delay_ms(5);
    }


    lineOffset = (int32_t)(sum / samples);
}


/* ======================= PID controller ============================= */
typedef struct {
    float kp, ki, kd;
    float integral;
    float integral_limit;   /* clamp |integral| to this */
    float prev_error;
} pid_ctrl_t;

static float pid_update(pid_ctrl_t *pid, float error, float dt) {
    if (dt <= 0.0f) return pid->kp * error; /* guard divide-by-zero */

    pid->integral += error * dt;
    if (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}

static void pid_reset(pid_ctrl_t *pid) {
    pid->integral    = 0.0f;
    pid->prev_error  = 0.0f;
}

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}


/* ---- Outer loop: line position -> steering correction (rpm) ---
 * Mapping lap: full authority with LINE_KP/LINE_KD.
 * Run laps:    limited trim on top of the map feed-forward, with its own
 *              gains (LINE_RUN_*) so the two can be tuned independently. */
#define LINE_KP                0.03f
#define LINE_KI                0.0f
#define LINE_KD                0.000f
#define LINE_INTEGRAL_LIMIT    500.0f
#define LINE_CORRECTION_LIMIT  800.0f

#define LINE_RUN_KP            0.07f     /* TUNE: run-mode trim gains (start equal to mapping) */
#define LINE_RUN_KD            0.004f

static pid_ctrl_t linePID = { LINE_KP, LINE_KI, LINE_KD, 0.0f, LINE_INTEGRAL_LIMIT, 0.0f };

/* ---- Inner loop: per-wheel speed (rpm) -> PWM duty (0-255) ----
 * Gain-scheduled ki, ported from MATLAB:
 *   P.L.pid = struct('kp',1.4,'ki',(2.4 - 0.00325*(REF_L - 300)),'kd',0,'limit',100)
 * The schedule is evaluated from each wheel's target speed (ref). */
#define SPEED_KP               1.4f
#define SPEED_KD               0.0f
#define SPEED_KI_BASE          2.4f
#define SPEED_KI_SLOPE         0.00325f
#define SPEED_KI_REF           300.0f
#define SPEED_KI_MIN           1.0f      /* sanity clamps: only active outside ~0-730 rpm */
#define SPEED_KI_MAX           3.4f
#define SPEED_INTEGRAL_LIMIT   100.0f

static float speed_ki(float ref_rpm) {
    float ki = SPEED_KI_BASE - SPEED_KI_SLOPE * (ref_rpm - SPEED_KI_REF);
    return clampf(ki, SPEED_KI_MIN, SPEED_KI_MAX);
}

static pid_ctrl_t speedPID_L = { SPEED_KP, SPEED_KI_BASE, SPEED_KD, 0.0f, SPEED_INTEGRAL_LIMIT, 0.0f };
static pid_ctrl_t speedPID_R = { SPEED_KP, SPEED_KI_BASE, SPEED_KD, 0.0f, SPEED_INTEGRAL_LIMIT, 0.0f };


/* ======================= Track map ================================== */
/* ---- Geometry ---- */
#define TRACK_W_MM        90.4f                          /* wheel to wheel */
#define WHEEL_D_MM        43.0f
#define K_RPM_TO_MMS      (3.14159265f * WHEEL_D_MM / 60.0f)   /* mm/s per rpm (~2.25) */

/* ---- Map storage ---- */
#define MARKS_PER_LAP     5          /* start-of-straight markers per lap = segments */
#define BIN_MM            20.0f      /* map resolution along the track               */
#define MAX_BINS          320        /* 6.4 m of track; MQ Racer lap is ~5.3 m       */
#define CURV_UNIT         0.0002f    /* 1/mm per int8 count (1 count = 1 encoder-tick difference per bin) */
#define SPEED_UNIT_RPM    4.0f       /* speedMap is uint8 in units of 4 rpm          */

/* ---- Map cleanup (NEW, TUNE) ---- */
#define SMOOTH_PASSES     3          /* number of [1 2 1]/4 passes (each blurs corner entries a bit) */
#define CURV_DEADBAND     3          /* |count| below this -> 0 (3 counts = 0.0006 1/mm, R > 1.6 m)  */

/* ---- Mapping lap ---- */
#define MAP_SPEED_RPM     180.0f     /* slow enough that a 20 mm marker is never skipped between loops */
#define MIN_MARKER_GAP_MM 150.0f     /* ignore marker re-triggers closer than this   */
#define CHECKPOINT_SUM    13000      /* sum of darkness that means "marker"          */

/* ---- Speed profile (TUNE) ----
 * A_LAT lowered from 5000 (try 2500-3000). A_BRAKE must reflect what the car
 * REALLY achieves: duty is clamped >= 0 so the car only coasts when the target
 * drops. Measure the coast-down rate from full speed and put it here. */
#define A_LAT_MM_S2       3400.0f    /* cornering grip budget                        */
#define A_BRAKE_MM_S2     2000.0f    /* braking into corners (MEASURE THIS)          */
#define A_ACCEL_MM_S2     3400.0f    /* accelerating out of corners                  */
#define MIN_SPEED_RPM     50.0f
#define ACCEL_RPM_S       (A_ACCEL_MM_S2 / K_RPM_TO_MMS)   /* runtime base-speed rise rate */

/* ---- Map following (TUNE) ---- */
#define LOOKAHEAD_MM      115.0f      /* read the map this far ahead (motor/steer lag) */
#define TRIM_LIMIT_MIN    900.0f     /* max line trim (rpm) at low speed             */
#define TRIM_LIMIT_MAX    900.0f     /* max line trim (rpm) at MAX_SPEED             */
#define MISS_MARGIN_MM    150.0f     /* past segment end by this much -> marker missed */
#define LOST_STOP_MM      500.0f     /* stop if the line is lost for this far        */
#define STOP_AFTER_LAP    0          /* 0 = run forever, N = stop after lap N        */

static int8_t   curvMap[MAX_BINS];   /* signed curvature, + = left                   */
static uint8_t  speedMap[MAX_BINS];  /* centreline speed in SPEED_UNIT_RPM           */
static uint16_t nBins = 0;
static uint16_t segStart[MARKS_PER_LAP];
static uint16_t segBins[MARKS_PER_LAP];
static float    segLen[MARKS_PER_LAP];   /* measured marker-to-marker length (mm)    */

static uint8_t  seg = 0;             /* current segment                              */
static float    segDist = 0.0f;      /* mm since the last marker                     */

/* mapping accumulators */
static float    binDist = 0.0f, binL = 0.0f, binR = 0.0f;
static uint8_t  mapFailed = 0;

typedef enum { MODE_FIND_START = 0, MODE_MAPPING, MODE_RUN, MODE_SENSOR_ONLY } run_mode_t;

static float rampSpeed = 0.0f;       /* rate-limited base speed */


/* Feed one loop of motion into the map while mapping. */
static void map_record(float dCar, float dL, float dR) {
    binDist += dCar;
    binL    += dL;
    binR    += dR;

    while (binDist >= BIN_MM) {
        if (nBins >= MAX_BINS) {                 /* track longer than the buffer */
            mapFailed = 1;
            binDist = binL = binR = 0.0f;
            return;
        }
        float f      = BIN_MM / binDist;         /* fraction of the accumulated motion that belongs to this bin */
        float kappa  = ((binR - binL) * f / TRACK_W_MM) / BIN_MM;   /* 1/mm, + = left */
        float u      = kappa / CURV_UNIT;
        int16_t q    = (int16_t)(u + (u >= 0.0f ? 0.5f : -0.5f));
        if (q >  127) q =  127;
        if (q < -127) q = -127;
        curvMap[nBins++] = (int8_t)q;

        binDist -= BIN_MM;
        binL    *= (1.0f - f);
        binR    *= (1.0f - f);
    }
}

/* Called when a marker ends the current segment during mapping. */
static void map_close_segment(void) {
    segBins[seg] = nBins - segStart[seg];
    segLen[seg]  = segDist;
    seg++;
    binDist = binL = binR = 0.0f;
    if (seg < MARKS_PER_LAP) segStart[seg] = nBins;
}

/* Smooth curvature, remove tiny curvature, then build the speed profile around the closed loop. */
static void map_finalize(void) {
    /* 1) [1 2 1]/4 smoothing inside each segment (removes line-wobble noise),
     *    repeated SMOOTH_PASSES times. */
    for (uint8_t p = 0; p < SMOOTH_PASSES; p++) {
        for (uint8_t s = 0; s < MARKS_PER_LAP; s++) {
            uint16_t a = segStart[s], n = segBins[s];
            if (n < 3) continue;
            int16_t prev = curvMap[a];
            for (uint16_t i = a + 1; i < a + n - 1; i++) {
                int16_t cur = curvMap[i];
                curvMap[i] = (int8_t)((prev + 2 * cur + curvMap[i + 1] + 2) >> 2);
                prev = cur;
            }
        }
    }

    /* 1b) deadband: near-zero curvature is wobble, not track, so keep straights straight.
     *     Done before the speed limit so straights get full speed. */
    for (uint16_t i = 0; i < nBins; i++) {
        if (abs((int)curvMap[i]) < CURV_DEADBAND) curvMap[i] = 0;
    }

    /* 2) per-bin speed limit from curvature */
    for (uint16_t i = 0; i < nBins; i++) {
        int16_t c = curvMap[i];
        if (c < 0) c = -c;
        float k = (float)c * CURV_UNIT;

        /* outer wheel = v * (1 + k*W/2) must stay under MAX_SPEED */
        float v = (float)MAX_SPEED / (1.0f + k * TRACK_W_MM * 0.5f);
        if (k > 1.0e-4f) {
            float vlat = sqrtf(A_LAT_MM_S2 / k) / K_RPM_TO_MMS;
            if (vlat < v) v = vlat;
        }
        v = clampf(v, MIN_SPEED_RPM, (float)MAX_SPEED);
        speedMap[i] = (uint8_t)(v / SPEED_UNIT_RPM);
    }

    /* 3) braking pass (backwards) and acceleration pass (forwards), twice so
     *    the limit propagates across the lap wrap. v^2 domain, in speed units. */
    const float k2   = K_RPM_TO_MMS * K_RPM_TO_MMS * SPEED_UNIT_RPM * SPEED_UNIT_RPM;
    const float kb   = 2.0f * A_BRAKE_MM_S2 * BIN_MM / k2;
    const float ka   = 2.0f * A_ACCEL_MM_S2 * BIN_MM / k2;

    for (uint8_t pass = 0; pass < 2; pass++) {
        for (int16_t i = (int16_t)nBins - 1; i >= 0; i--) {
            uint16_t nx = (uint16_t)(i + 1 >= (int16_t)nBins ? 0 : i + 1);
            float vn  = (float)speedMap[nx];
            float lim = vn * vn + kb;
            float vi  = (float)speedMap[i];
            if (vi * vi > lim) speedMap[i] = (uint8_t)sqrtf(lim);
        }
        for (uint16_t i = 0; i < nBins; i++) {
            uint16_t pv = (i == 0) ? (nBins - 1) : (i - 1);
            float vp  = (float)speedMap[pv];
            float lim = vp * vp + ka;
            float vi  = (float)speedMap[i];
            if (vi * vi > lim) speedMap[i] = (uint8_t)sqrtf(lim);
        }
    }
}

/* Map position (segment s, distance d into it) -> flat bin index.
 * d may run past the end of the segment (lookahead); it rolls into the next. */
static uint16_t map_index(uint8_t s, float d) {
    for (uint8_t guard = 0; guard < MARKS_PER_LAP; guard++) {
        float len = (float)segBins[s] * BIN_MM;
        if (d < len) break;
        d -= len;
        s = (s + 1 >= MARKS_PER_LAP) ? 0 : (uint8_t)(s + 1);
    }
    uint16_t b = (uint16_t)(d / BIN_MM);
    if (b >= segBins[s]) b = segBins[s] - 1;
    return segStart[s] + b;
}


/* ======================= Main ======================================= */
int main(void) {
    DIRA_DDR |= (1 << DIRA_BIT);
    DIRB_DDR |= (1 << DIRB_BIT);
    CTRL_DDR |= (1 << CTRL_BIT);
    CTRL_PORT |= (1 << CTRL_BIT);
    timer1_pwm_init();

    timer0_micros_init();
    encoder_init();
    uart_init();

    sei();

    /* Robot must be placed centered on the line before this runs.
     * Blocks for ~160 ms (32 samples x 5 ms). */
    calibrateLineOffset();

    pid_reset(&linePID);
    pid_reset(&speedPID_L);
    pid_reset(&speedPID_R);
    rampSpeed = 0.0f;

    run_mode_t mode = MODE_FIND_START;
    run_mode_t prevMode = mode;

    uint32_t last_loop_time  = micros();
    uint32_t last_speed_time = last_loop_time;

    float lastCar = (float)car_dist_mm;
    float lastM1  = (float)motor1_dist_mm;     /* motor1 = left  */
    float lastM2  = (float)motor2_dist_mm;     /* motor2 = right */
    float lostDist = 0.0f;
    uint16_t idx = 0;
    uint8_t printDiv = 0;
    uint8_t speedDiv = 0;

    for (;;) {
        uint32_t now = micros();

        float dt = (float)(now - last_loop_time) / 1000000.0f;
        last_loop_time = now;

        ReadSensorsRaw();
        int32_t error = computeLinePosition();
        lastError = error;

        if (++speedDiv >= SPEED_UPDATE_EVERY) {
            speedDiv = 0;
            update_motor_speeds(now - last_speed_time);
            last_speed_time = now;
        }
        update_motor_distances();

        /* --- motion since last loop --- */
        float carNow = (float)car_dist_mm;
        float m1Now  = (float)motor1_dist_mm;
        float m2Now  = (float)motor2_dist_mm;
        float dCar = carNow - lastCar;
        float dL   = m1Now  - lastM1;
        float dR   = m2Now  - lastM2;
        lastCar = carNow; lastM1 = m1Now; lastM2 = m2Now;

        segDist += dCar;

        uint8_t onMarker = (sum > CHECKPOINT_SUM);
        uint8_t lost     = (sum < 300);
        lostDist = lost ? (lostDist + dCar) : 0.0f;

        /* --- mode / map state machine --- */
        switch (mode) {
        case MODE_FIND_START:
            if (onMarker) {
                checkpoint++;
                nBins = 0;
                seg = 0;
                segStart[0] = 0;
                segDist = 0.0f;
                binDist = binL = binR = 0.0f;
                mapFailed = 0;
                mode = MODE_MAPPING;
            }
            break;

        case MODE_MAPPING:
            map_record(dCar, dL, dR);
            if (onMarker && segDist >= MIN_MARKER_GAP_MM) {
                checkpoint++;
                map_close_segment();
                segDist = 0.0f;
                if (seg >= MARKS_PER_LAP) {            /* back at the first marker: lap done */
                    if (mapFailed) {
                        mode = MODE_SENSOR_ONLY;       /* map unusable: keep following the sensor */
                    } else {
                        map_finalize();
                        seg = 0;
                        lap = 2;
                        mode = MODE_RUN;
                    }
                }
            }
            break;

        case MODE_RUN: {
            float len = segLen[seg];
            uint8_t advance = 0;
            if (onMarker && segDist >= 0.5f * len) {   /* real marker: re-sync */
                checkpoint++;
                segDist = 0.0f;
                advance = 1;
            } else if (segDist > len + MISS_MARGIN_MM) { /* marker missed: dead-reckon past it */
                segDist -= len;
                advance = 1;
            }
            if (advance) {
                seg++;
                if (seg >= MARKS_PER_LAP) { seg = 0; lap++; }
            }
        } break;

        case MODE_SENSOR_ONLY:
        default:
            break;
        }

        /* --- Outer loop: line position -> steering correction ---
         * Gains differ between mapping and run; reset the PID on a mode change
         * so stale integral/derivative state is not carried over. */
        if (mode != prevMode) {
            pid_reset(&linePID);
            prevMode = mode;
        }
        linePID.kp = (mode == MODE_RUN) ? LINE_RUN_KP : LINE_KP;
        linePID.kd = (mode == MODE_RUN) ? LINE_RUN_KD : LINE_KD;
        float correction = pid_update(&linePID, (float)error, dt);

        float goal;                    /* desired centreline speed (rpm) */
        float ff   = 0.0f;             /* feed-forward differential, = kappa * W / 2 */
        float trim;

        if (mode == MODE_RUN) {
            idx  = map_index(seg, segDist + LOOKAHEAD_MM);
            goal = (float)speedMap[idx] * SPEED_UNIT_RPM;
            if (goal > SPEED_CAP) goal = SPEED_CAP;
            ff   = (float)curvMap[idx] * CURV_UNIT * TRACK_W_MM * 0.5f;

            /* trim authority grows with speed: more correction needed when fast */
            float trimLimit = TRIM_LIMIT_MIN +
                (TRIM_LIMIT_MAX - TRIM_LIMIT_MIN) * (rampSpeed / (float)MAX_SPEED);
            /* sensor is only a trim; ignore it when the line is lost */
            trim = lost ? 0.0f : clampf(correction, -trimLimit, trimLimit);
        } else {
            goal = MAP_SPEED_RPM;
            trim = clampf(correction, -LINE_CORRECTION_LIMIT, LINE_CORRECTION_LIMIT);
        }

        if (lostDist > LOST_STOP_MM)                        goal = 0.0f;
        if (STOP_AFTER_LAP > 0 && lap > STOP_AFTER_LAP)     goal = 0.0f;

        /* --- Accel ramp: rises at ACCEL_RPM_S, falls immediately --- */
        {
            float rise = rampSpeed + ACCEL_RPM_S * dt;
            rampSpeed = (goal < rise) ? goal : rise;
        }
        float base = rampSpeed;

        trim = clampf(trim, -base, base);

        float targetL = clampf(base * (1.0f - ff) - trim, 0.0f, (float)MAX_SPEED);
        float targetR = clampf(base * (1.0f + ff) + trim, 0.0f, (float)MAX_SPEED);


        /* --- Inner loop: target speed -> PWM duty, per wheel --- */
        speedPID_L.ki = speed_ki(targetL);   /* gain-scheduled ki (from MATLAB) */
        speedPID_R.ki = speed_ki(targetR);

        float dutyL = pid_update(&speedPID_L, targetL - motor1_speed_rpm, dt);
        float dutyR = pid_update(&speedPID_R, targetR - motor2_speed_rpm, dt);

        uint8_t pwmL = (uint8_t)clampf(dutyL, 0.0f, 200.0f);
        uint8_t pwmR = (uint8_t)clampf(dutyR, 0.0f, 200.0f);

        forward(pwmL, pwmR);

        /* --- Throttled debug print (a blocking printf costs ~4 ms) --- */
        if (mode != MODE_RUN || PRINT_IN_RUN) {
            if (++printDiv >= PRINT_EVERY) {
                printDiv = 0;
                printf("m:%d lap:%d seg:%d d:%d err:%d base:%d chk:%d \n",
                       (int)mode, lap, (int)seg, (int)segDist, (int)error,
                       (int)base, (int)checkpoint);
            }
        }

        /* --- Fixed loop period (replaces _delay_ms(20)) --- */
        while ((micros() - now) < LOOP_PERIOD_US) { }
    }
}
