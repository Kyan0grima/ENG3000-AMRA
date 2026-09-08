#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#define BAUD  115200


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/setbaud.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "encoder.c"


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


int MAX_SPEED = 2800; /* ticks/sec ceiling - do not exceed */


int turning = 0;
uint32_t turnStart = 0;


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
    TCCR0B = (1 << CS01);
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


    return (((ovf << 8) | tcnt) >> 1);
}


/* ======================= Hardware PWM: Timer1 ===================== */
static void timer1_pwm_init(void) {
    PWM_DDR |= (1 << PWMA_BIT) | (1 << PWMB_BIT);
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
    OCR1A = 0;
    OCR1B = 0;
}


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
    uint32_t start, elapsed, total;
    uint8_t  activeSensors = 8;


    for (i = 0; i < 8; i++) {
        *sensorPins[i].ddr  |= sensorPins[i].mask;
        *sensorPins[i].port |= sensorPins[i].mask;
    }
    _delay_us(10);


    for (i = 0; i < 8; i++) {
        *sensorPins[i].ddr  &= ~sensorPins[i].mask;
        *sensorPins[i].port &= ~sensorPins[i].mask;
        rawValues[i] = 0;
    }


    start = micros();
    while (((micros() - start) < TIMEOUT_US) && (activeSensors > 0)) {
        elapsed = micros() - start;
        for (i = 0; i < 8; i++) {
            if (rawValues[i] == 0 && !(*sensorPins[i].pin & sensorPins[i].mask)) {
                rawValues[i] = elapsed;
                activeSensors--;
            }
        }
    }


    total = 0;
    for (i = 0; i < 8; i++) {
        if (rawValues[i] == 0) rawValues[i] = TIMEOUT_US;
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


static void backward(uint8_t L_Speed, uint8_t R_Speed) {
    DIRA_PORT &= ~(1 << DIRA_BIT);
    DIRB_PORT &= ~(1 << DIRB_BIT);
    setPWMA(L_Speed);
    setPWMB(R_Speed);
}


/* ======================= Line position ============================= */
static int32_t lastError  = 0;
static int32_t lineOffset = 0;   /* measured "centered" bias, subtracted from every reading */
int64_t sum = 0;


static int32_t computeLinePosition(void) {
    static const int32_t weight[8] = { -3500, -2500, -1500, -500,
                                         500,  1500,  2500,  3500 };
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
    float integral_limit;   /* clamp |integral| to this, in output units */
    float prev_error;
} pid_t;


static float pid_update(pid_t *pid, float error, float dt) {
    if (dt <= 0.0f) return pid->kp * error; /* guard divide-by-zero on first call */


    pid->integral += error * dt;
    if (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;


    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;


    return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}


static void pid_reset(pid_t *pid) {
    pid->integral    = 0.0f;
    pid->prev_error  = 0.0f;
}


static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}


/* ---- Outer loop: line position -> steering correction (ticks/sec) ---
 * Setpoint is always 0 (computeLinePosition() already returns signed
 * error, positive = line to the right of the weight array's zero).
 * TUNE THESE on the real robot. */
#define LINE_KP                1.5f
#define LINE_KI                0.1f
#define LINE_KD                1.5f
#define LINE_INTEGRAL_LIMIT    2600.0f
#define LINE_CORRECTION_LIMIT  1900.0f  /* clamps how hard steering can pull L/R apart */


static pid_t linePID = { LINE_KP, LINE_KI, LINE_KD, 0.0f, LINE_INTEGRAL_LIMIT, 0.0f };


/* ---- Inner loop: per-wheel speed (ticks/sec) -> PWM duty (0-255) ----
 * TUNE THESE too - start with Kp only, add Ki once proportional-only
 * settles near target but with steady-state error. */
#define SPEED_KP               0.08f
#define SPEED_KI               0.1f
#define SPEED_KD               0.0f
#define SPEED_INTEGRAL_LIMIT   150.0f


static pid_t speedPID_L = { SPEED_KP, SPEED_KI, SPEED_KD, 0.0f, SPEED_INTEGRAL_LIMIT, 0.0f };
static pid_t speedPID_R = { SPEED_KP, SPEED_KI, SPEED_KD, 0.0f, SPEED_INTEGRAL_LIMIT, 0.0f };


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
     * Blocks for ~160ms (32 samples x 5ms). */
    calibrateLineOffset();


    pid_reset(&linePID);
    pid_reset(&speedPID_L);
    pid_reset(&speedPID_R);


    uint32_t last_loop_time  = micros();
    uint32_t last_speed_time = last_loop_time;




    for (;;) {
        uint32_t now = micros();
       
        float dt = (float)(now - last_loop_time) / 1000000.0f;
        last_loop_time = now;

        ReadSensorsRaw();
        int32_t error = computeLinePosition();
        lastError = error;

        /* Encoder speed update uses its own elapsed time, independent of
         * the float dt used for the PID loops above. */
        update_motor_speeds(now - last_speed_time);
        last_speed_time = now;
        update_motor_distances();

        if (sum<6000){MAX_SPEED = 1800;} else {MAX_SPEED = 3000;}


        /* --- Outer loop: line position -> steering correction --- */
        float correction = pid_update(&linePID, (float)error, dt);
        correction = clampf(correction, -LINE_CORRECTION_LIMIT, LINE_CORRECTION_LIMIT);


        float targetL = clampf((float)MAX_SPEED - correction, 0.0f, (float)MAX_SPEED);
        float targetR = clampf((float)MAX_SPEED + correction, 0.0f, (float)MAX_SPEED);


        /* --- Inner loop: target speed -> PWM duty, per wheel --- */
        float dutyL = pid_update(&speedPID_L, targetL - (float)motor1_speed, dt);
        float dutyR = pid_update(&speedPID_R, targetR - (float)motor2_speed, dt);


        uint8_t pwmL = (uint8_t)clampf(dutyL, 0.0f, 255.0f);
        uint8_t pwmR = (uint8_t)clampf(dutyR, 0.0f, 255.0f);
        //if ((40 < car_dist && car_dist < 90) || (360 < car_dist && car_dist < 430) || (500 < car_dist && car_dist < 580)) {MAX_SPEED = 2600;} else {MAX_SPEED = 3100;}


        forward(pwmL, pwmR);


        printf("err: %ld\tdist: %u\tspeed: %u\n",
           
            error, car_dist, MAX_SPEED);


        _delay_ms(20);
    }
}

