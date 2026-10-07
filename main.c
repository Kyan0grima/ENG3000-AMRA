main save


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


int MAX_SPEED    = 650; /* rpm ceiling - do not exceed */
int TARGET_SPEED = 650; /* current target rpm (ramped by decel()) */

int turnCounter = 0;
int lap = 1;
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

static void decel(float triggerDist, float decelDist, float minSpeed, float startSpeed) {
    float into = car_dist_mm - triggerDist;
    if (into <= 0.0f) return;
    if (into >= decelDist) {            /* past the zone: hold the final speed */
        TARGET_SPEED = (int)minSpeed;
        return;
    }
    float frac = into / decelDist;      /* 0 at trigger, 1 at end of zone */
    TARGET_SPEED = (int)clampf(startSpeed + (minSpeed - startSpeed) * frac,
                               minSpeed, (float)MAX_SPEED);
}


/* ---- Outer loop: line position -> steering correction (rpm) ---
 * Setpoint is always 0. Units are wheel rpm. TUNE on the real robot.
 * 700 rpm of correction against an 800 rpm ceiling is very aggressive;
 * consider lowering LINE_CORRECTION_LIMIT. */
#define LINE_KP                1.3f
#define LINE_KI                3.2f
#define LINE_KD                2.0f
#define LINE_INTEGRAL_LIMIT    800.0f
#define LINE_CORRECTION_LIMIT  700.0f

static pid_ctrl_t linePID = { LINE_KP, LINE_KI, LINE_KD, 0.0f, LINE_INTEGRAL_LIMIT, 0.0f };

/* ---- Inner loop: per-wheel speed (rpm) -> PWM duty (0-255) ----
 * Integral limit ~ 255 / KI so the integral can't wind up past what the
 * PWM output can actually use. */
#define SPEED_KP               1.0f
#define SPEED_KI               3.0f
#define SPEED_KD               0.0f
#define SPEED_INTEGRAL_LIMIT   150.0f

static pid_ctrl_t speedPID_L = { SPEED_KP, SPEED_KI, SPEED_KD, 0.0f, SPEED_INTEGRAL_LIMIT, 0.0f };
static pid_ctrl_t speedPID_R = { SPEED_KP, SPEED_KI, SPEED_KD, 0.0f, SPEED_INTEGRAL_LIMIT, 0.0f };


#define DEG2RAD  0.0174533f

/* Must be placed AFTER the linePID definition (or forward-declare linePID),
 * because it calls pid_reset(&linePID). */
static void turn(float triggerDist, float radius, float speed, int dir, float angle,
                 float *leftSpeed, float *rightSpeed) {
    static uint8_t turning = 0;
    static float startL, startR;

    if (car_dist_mm <= triggerDist) return;

    if (!turning) {                       /* first loop of the turn */
        turning = 1;
        startL = (float)motor1_dist_mm;   /* motor1 = left (assumed) */
        startR = (float)motor2_dist_mm;   /* motor2 = right (assumed) */
        pid_reset(&linePID);              /* drop stale integral */
    }

    float dL = (float)motor1_dist_mm - startL;
    float dR = (float)motor2_dist_mm - startR;

    /* dir 0 = right turn: left wheel is outer, so it travels further */
    float heading = ((dir == 0) ? (dL - dR) : (dR - dL)) / 90.4f;   /* radians */

    if (heading >= angle * DEG2RAD) {     /* turn complete */
        turning = 0;
        turnCounter++;
        pid_reset(&linePID);
        return;
    }

    float ratio = (2.0f * radius - 90.4f) / (2.0f * radius + 90.4f);
    if (ratio < 0.0f) ratio = 0.0f;

    if (dir == 0) { *leftSpeed = speed;         *rightSpeed = speed * ratio; }
    else          { *rightSpeed = speed;        *leftSpeed  = speed * ratio; }

    *leftSpeed  = clampf(*leftSpeed,  0.0f, 900.0f);
    *rightSpeed = clampf(*rightSpeed, 0.0f, 900.0f);
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

    uint32_t last_loop_time  = micros();
    uint32_t checkpointDebounce = micros();
    uint32_t last_speed_time = last_loop_time;

    for (;;) {
        uint32_t now = micros();

        float dt = (float)(now - last_loop_time) / 1000000.0f;
        last_loop_time = now;

        ReadSensorsRaw();
        int32_t error = computeLinePosition();
        lastError = error;

        update_motor_speeds(now - last_speed_time);
        last_speed_time = now;
        update_motor_distances();

        if ((now - checkpointDebounce) > 400000){
            if (sum>10000){
                checkpoint++;
                checkpointDebounce = micros();
            }
        }


        /* --- Outer loop: line position -> steering correction --- */
        float correction = pid_update(&linePID, (float)error, dt);
        correction = clampf(correction, -LINE_CORRECTION_LIMIT, LINE_CORRECTION_LIMIT);

        if (turnCounter == 0) {
            decel(80, 200, 200, MAX_SPEED);
        }
        if (car_dist_mm > 800){
            TARGET_SPEED = MAX_SPEED;
        }

        /* turnCounter == 1: do nothing, TARGET_SPEED holds at 200 */

        correction = clampf(correction, -(float)TARGET_SPEED, (float)TARGET_SPEED);

        float targetL = clampf((float)TARGET_SPEED - correction, 0.0f, (float)MAX_SPEED);
        float targetR = clampf((float)TARGET_SPEED + correction, 0.0f, (float)MAX_SPEED);

        // /* --- 3. Arcs --- */
        // if (turnCounter == 0 && checkpoint == 2) {
        //     turn(0.0f, 100.0f, (float)TARGET_SPEED, 1, 75.0f, &targetL, &targetR);  /* left */
        // }
        // else if (turnCounter == 1 && checkpoint == 2) {
        //     turn(0.0f,   100.0f, (float)TARGET_SPEED, 0, 75.0f, &targetL, &targetR);  /* right */
        // }



        /* --- Inner loop: target speed -> PWM duty, per wheel --- */
        float dutyL = pid_update(&speedPID_L, targetL - motor1_speed_rpm, dt);
        float dutyR = pid_update(&speedPID_R, targetR - motor2_speed_rpm, dt);

        uint8_t pwmL = (uint8_t)clampf(dutyL, 0.0f, 255.0f);
        uint8_t pwmR = (uint8_t)clampf(dutyR, 0.0f, 255.0f);

        //forward(pwmL, pwmR);

        printf("speed: %d \tspeed2: %d \terror: %d \tdist: %d \tcheck: %d \n",
               (int)motor1_speed_rpm, (int)motor2_speed_rpm, (int)error, (int)car_dist_mm, (int)checkpoint);

        _delay_ms(20);
    }
}
