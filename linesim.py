#!/usr/bin/env python3
"""
Line-follower simulator.  Mirrors main.c + encoder.c logic (same PIDs, same
sensor maths, same 16-bit encoder counters, same loop timing) on a track you draw.

    pip install pygame
    python linesim.py

Controls
    Left mouse drag    draw line          Shift + drag   straight line
    Right mouse drag   erase
    SPACE run/pause    R reset robot      C clear track
    P place start at mouse, Q/E rotate start heading
    1/2/3/4 presets (oval / straight+turns / serpentine / MQ Racer track)
    UP/DOWN sim speed  T toggle trail     S/L save/load track.png
"""
import math, random, json, os
import pygame

# ============================ SIM / HARDWARE CONFIG =========================
PX_MM = 2.5
TRACK_W_MM, TRACK_H_MM = 2400, 1200      # MQ Racer base board
LINE_WIDTH_MM = 20
TRACK_WIDTH_MM = 90.4              # wheel to wheel (same as turn())
WHEEL_D_MM = 43.0
AXLE_TO_SENSOR_MM = 82.0           # sensor bar distance ahead of the axle
SENSOR_SPACING_MM = 8         # QTR-8RC pitch
SENSOR_SPOT_MM = 3.0
# Your code treats a SHORT decay time as "line" (darkness = TIMEOUT - raw), so
# the sim makes the line give a short time and the background time out.
SENSOR_LINE_RAW_US = 400
SENSOR_BG_RAW_US = 2500
SENSOR_NOISE_US = 15

# Gain-scheduled motor model, D = pwm/255. Coefficients are highest power first.
#   steady-state rpm = Omega_ss(D),  time constant (s) = tau(D)
MOTOR_MODEL = {
    'L': dict(omega=(5066.6, -10577.0, 8065.4, -1231.4),
              tau=(-8.8998, 17.515, -11.8, 3.0446)),
    'R': dict(omega=(7941.0, -15053.0, 10139.0, -1484.9),
              tau=(-3.356, 7.5919, -6.0796, 1.9791)),
}
TAU_MIN_S = 0.02                   # floor: the left tau polynomial goes negative above D~0.955
# Measured coasting (pwm below the Omega_ss root): y = -1.2562x + 2788.3
# Assumed x = ms, y = wheel rpm  ->  constant decel of 1.2562 rpm/ms.
COAST_DECEL_RPM_PER_S = 1.2562 * 1000.0

ENC1_PHYSICALLY_INVERTED = True    # motor 1 raw count goes negative going forward

# ============================ FIRMWARE CONSTANTS (mirror of main.c) =========
F_BAUD = 115200
TIMEOUT_US = 2500
MAX_SPEED = 650                    # int in the C
LINE_KP, LINE_KI, LINE_KD = 0.05, 0.3, 0.04
LINE_INTEGRAL_LIMIT = 800.0
LINE_CORRECTION_LIMIT = 700.0

# ---- Speed loop: ONE PID PER WHEEL, each with its own gains --------------
# Input is rpm error (target - measured), output is PWM duty 0-255.
#   kp, ki, kd : gains
#   limit      : clamp on |integral|  (the I term can contribute at most ki * limit duty)
SPEED_PID_L = dict(kp=1, ki=2.3, kd=0.0, limit=100.0)    # motor 1 (left)
SPEED_PID_R = dict(kp=1, ki=2.3, kd=0.0, limit=100.0)    # motor 2 (right)
MAX_DUTY = 200                     # cap on the speed-PID output, 0-255 (200 = D 0.78, where both tau fits agree)

LOOP_DELAY_US = 20000              # _delay_ms(20)
SIM_PRINTF_TIME = True             # printf blocks on the UART; adds ~5 ms to every loop
TICKS_PER_REV = 360.0
MM_PER_TICK = math.pi * WHEEL_D_MM / TICKS_PER_REV
RPM_NUMERATOR = 60000000.0 / TICKS_PER_REV
M1_DIR_SIGN, M2_DIR_SIGN = -1, 1
DEG2RAD = 0.0174533

CHECKPOINT_DEBOUNCE_US = 400000
CHECKPOINT_SUM = 10000

# Speed schedule: list of (trigger_mm, zone_mm, min_speed, start_speed, release_mm)
#   trigger_mm : car_dist_mm where the ramp starts
#   zone_mm    : length of the linear ramp start_speed -> min_speed
#   min_speed  : speed held after the ramp, until release_mm
#   start_speed: speed at the trigger point (normally MAX_SPEED)
#   release_mm : car_dist_mm where target speed goes back to MAX_SPEED
# Outside every zone the target is MAX_SPEED. If zones overlap, the later entry wins.
DECEL_POINTS = [
    (80,   200, 200, MAX_SPEED, 800),
    (3240, 200, 220, MAX_SPEED, 4900),
]
STOP_AT_CHECKPOINT = 1

# ---- PROPOSED changes (not in your C yet). Flip each to False to get the old behaviour ----
USE_ACCEL_RAMP = True              # limit how fast the base speed may rise (launch AND after every decel zone)
RAMP_RATE_RPM_S = 500.0            # default rise rate, rpm per second
USE_FEEDFORWARD = False            # per-wheel duty from inverse Omega_ss(D); PID only trims (tested: slower, see notes)
USE_ANTIWINDUP = False             # stop integrating while duty is saturated (tested: worse with this line PID)
# Trim PIDs used INSTEAD of SPEED_PID_L/R when USE_FEEDFORWARD is on (also one per wheel).
#   trim_limit : max |ki * integral| in duty counts
FF_PID_L = dict(kp=0.3, ki=1.0, kd=0.0, trim_limit=40.0)
FF_PID_R = dict(kp=0.3, ki=1.0, kd=0.0, trim_limit=40.0)

# ============================ UI CONFIG =====================================
W_PX, H_PX = int(TRACK_W_MM / PX_MM), int(TRACK_H_MM / PX_MM)
SIDEBAR = 330
WIN_H = max(H_PX, 600)
BG_COL, LINE_COL = (28, 28, 32), (245, 245, 245)
SPEEDS = [0.25, 0.5, 1, 2, 5, 10]


def i16(x):
    x &= 0xFFFF
    return x - 65536 if x > 32767 else x


def clampf(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


# ============================ PLANT (robot hardware) ========================
class Robot:
    def __init__(self, pose):
        self.reset(pose)

    def reset(self, pose):
        self.x, self.y, self.th = pose
        self.rpm_l = self.rpm_r = 0.0
        self.pwm_l = self.pwm_r = 0
        self.m1_ticks = self.m2_ticks = 0      # int16 ISR counters (wrap silently)
        self.acc1 = self.acc2 = 0.0
        self.true1 = self.true2 = 0            # un-wrapped, diagnostics only

    @staticmethod
    def _poly(c, d):
        return ((c[0] * d + c[1]) * d + c[2]) * d + c[3]

    def _omega_ss(self, m, pwm):
        d = clampf(pwm / 255.0, 0.0, 1.0)
        return max(0.0, self._poly(MOTOR_MODEL[m]['omega'], d))   # below the root = no drive

    def _tau(self, m, pwm):
        d = clampf(pwm / 255.0, 0.0, 1.0)
        return max(TAU_MIN_S, self._poly(MOTOR_MODEL[m]['tau'], d))

    def _lag(self, m, rpm, pwm, dt):
        target = self._omega_ss(m, pwm)
        if target <= 0.0:
            # no drive: linear coast-down (measured), never below 0
            return max(0.0, rpm - COAST_DECEL_RPM_PER_S * dt)
        tau = self._tau(m, pwm)
        rpm = rpm + (target - rpm) * (1 - math.exp(-dt / tau))
        return max(0.0, rpm)

    def step(self, dt):
        self.rpm_l = self._lag('L', self.rpm_l, self.pwm_l, dt)
        self.rpm_r = self._lag('R', self.rpm_r, self.pwm_r, dt)
        vl = self.rpm_l / 60 * math.pi * WHEEL_D_MM
        vr = self.rpm_r / 60 * math.pi * WHEEL_D_MM
        v, w = (vl + vr) / 2, (vr - vl) / TRACK_WIDTH_MM
        self.th -= w * dt                       # y-down screen: left turn = th decreases
        self.x += v * math.cos(self.th) * dt
        self.y += v * math.sin(self.th) * dt
        # encoder ticks (motor 1 = left, motor 2 = right)
        self.acc1 += self.rpm_l / 60 * dt * TICKS_PER_REV
        n1 = int(self.acc1); self.acc1 -= n1
        self.acc2 += self.rpm_r / 60 * dt * TICKS_PER_REV
        n2 = int(self.acc2); self.acc2 -= n2
        self.true1 += n1; self.true2 += n2
        s1 = -1 if ENC1_PHYSICALLY_INVERTED else 1
        self.m1_ticks = (self.m1_ticks + s1 * n1) & 0xFFFF
        self.m2_ticks = (self.m2_ticks + n2) & 0xFFFF


# ============================ FIRMWARE (mirror of C) ========================
class PID:
    def __init__(self, kp, ki, kd, lim):
        self.kp, self.ki, self.kd, self.lim = kp, ki, kd, lim
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev = 0.0

    def update(self, error, dt):
        if dt <= 0.0:
            return self.kp * error
        self.integral = clampf(self.integral + error * dt, -self.lim, self.lim)
        d = (error - self.prev) / dt
        self.prev = error
        return self.kp * error + self.ki * self.integral + self.kd * d

    def update_sat(self, error, dt, ff, lo, hi, antiwindup):
        """Same as update() plus a feedforward term and conditional-integration anti-windup."""
        if dt <= 0.0:
            return ff + self.kp * error
        new_int = clampf(self.integral + error * dt, -self.lim, self.lim)
        d = (error - self.prev) / dt
        self.prev = error
        out = ff + self.kp * error + self.ki * new_int + self.kd * d
        saturated_further = (out > hi and error > 0) or (out < lo and error < 0)
        if not (antiwindup and saturated_further):
            self.integral = new_int
        return out


def make_speed_pid(cfg):
    """Build a speed PID from one of the per-wheel config dicts."""
    if 'trim_limit' in cfg:                       # feedforward trim form
        lim = cfg['trim_limit'] / max(cfg['ki'], 1e-9)
    else:
        lim = cfg['limit']
    return PID(cfg['kp'], cfg['ki'], cfg['kd'], lim)


class Firmware:
    def __init__(self, world):
        self.m1_dist_mm = self.m2_dist_mm = 0.0

        self.w = world
        self.raw = [TIMEOUT_US] * 8
        self.lastError = 0
        self.lineOffset = 0
        self.sum = 0
        self.turnCounter = 0
        self.lap = 1
        self.checkpoint = 0
        self.target_speed = MAX_SPEED             # int TARGET_SPEED in the C
        # static state inside the C turn()
        self.turning = False
        self.turn_startL = self.turn_startR = 0.0

        self.ramp = 0.0                           # rate-limited base speed (rpm)
        self.accel_rate = RAMP_RATE_RPM_S         # rise rate currently in force (rpm/s)
        self.base = 0.0                           # base speed actually used this loop
        self.ff_tab = {m: [max(0.0, Robot._poly(MOTOR_MODEL[m]['omega'], p / 255.0))
                           for p in range(256)] for m in 'LR'}
        self.linePID = PID(LINE_KP, LINE_KI, LINE_KD, LINE_INTEGRAL_LIMIT)
        # one independent speed PID per wheel, each from its own gain set
        cfgL, cfgR = (FF_PID_L, FF_PID_R) if USE_FEEDFORWARD else (SPEED_PID_L, SPEED_PID_R)
        self.pidL = make_speed_pid(cfgL)
        self.pidR = make_speed_pid(cfgR)
        # encoder.c state
        self.last_m1 = self.last_m2 = 0
        self.last_m1d = self.last_m2d = 0
        self.m1_count = self.m2_count = 0
        self.m1_rpm = self.m2_rpm = 0.0
        self.car_dist_mm = 0.0
        self.last_true1 = self.last_true2 = 0
        self.tel = dict(error=0, correction=0.0, tL=0.0, tR=0.0, pwmL=0, pwmR=0,
                        loop_ms=0.0, wrap=False, lost=False)

    # ---- ReadSensorsRaw: returns how long it blocks (us)
    def read_sensors(self):
        self.raw = self.w.sample_sensors()
        return max(self.raw) + 10

    def compute_line_position(self):
        weight = (3500, 2500, 1500, 500, -500, -1500, -2500, -3500)
        ws = s = 0
        for i in range(8):
            d = max(0, TIMEOUT_US - self.raw[i])
            ws += weight[i] * d
            s += d
        self.sum = s
        if s < 300:
            return -3500 if self.lastError < 0 else 3500
        return int(ws / s) - self.lineOffset

    def calibrate(self):
        self.lineOffset = 0
        total = 0
        for _ in range(32):
            yield self.read_sensors()
            total += self.compute_line_position()
            yield 5000
        self.lineOffset = int(total / 32)

    # ---- encoder.c
    def update_motor_speeds(self, dt_us):
        if dt_us == 0:
            return
        r = self.w.robot
        d1 = i16(r.m1_ticks - self.last_m1)
        d2 = i16(r.m2_ticks - self.last_m2)
        self.last_m1, self.last_m2 = r.m1_ticks, r.m2_ticks
        self.tel['wrap'] = (abs(r.true1 - self.last_true1) > 32767 or
                            abs(r.true2 - self.last_true2) > 32767)
        self.last_true1, self.last_true2 = r.true1, r.true2
        inv = 1.0 / dt_us
        self.m1_rpm = M1_DIR_SIGN * d1 * RPM_NUMERATOR * inv
        self.m2_rpm = M2_DIR_SIGN * d2 * RPM_NUMERATOR * inv

    def update_motor_distances(self):
        r = self.w.robot
        d1 = i16(r.m1_ticks - self.last_m1d)
        d2 = i16(r.m2_ticks - self.last_m2d)
        self.last_m1d, self.last_m2d = r.m1_ticks, r.m2_ticks
        self.m1_count += M1_DIR_SIGN * d1
        self.m2_count += M2_DIR_SIGN * d2
        self.car_dist_mm = (self.m1_count + self.m2_count) * MM_PER_TICK * 0.5
        self.m1_dist_mm = self.m1_count * MM_PER_TICK
        self.m2_dist_mm = self.m2_count * MM_PER_TICK

    # ---- feedforward: inverse of Omega_ss (a 256-entry table in firmware)
    def ff_duty(self, m, rpm):
        if rpm <= 0.0:
            return 0.0
        tab = self.ff_tab[m]
        p0 = next((p for p in range(256) if tab[p] > 0.0), 255)   # first pwm that moves the wheel
        if rpm <= tab[p0]:
            return p0 * rpm / tab[p0]                              # below the stiction step
        for p in range(p0 + 1, 256):
            if tab[p] >= rpm:
                return p - 1 + (rpm - tab[p - 1]) / (tab[p] - tab[p - 1])
        return 255.0

    # ---- main.c: speed schedule (multiple decel points)
    def apply_speed_schedule(self):
        target = float(MAX_SPEED)
        for trig, zone, vmin, vstart, release, *opt in DECEL_POINTS:
            if not (trig < self.car_dist_mm < release):
                continue
            self.accel_rate = opt[0] if opt else RAMP_RATE_RPM_S   # kept after leaving the zone
            into = self.car_dist_mm - trig
            if into >= zone:
                target = float(vmin)
            else:
                frac = into / zone
                target = clampf(vstart + (vmin - vstart) * frac,
                                min(vmin, vstart), max(vmin, vstart))
        self.target_speed = int(clampf(target, 0.0, float(MAX_SPEED)))

    # ---- main.c: turn()  (calls are commented out in the C; gated by USE_TURN_ARCS)
    def turn(self, trigger_dist, radius, speed, direction, angle, tL, tR):
        if self.car_dist_mm <= trigger_dist:
            return tL, tR
        if not self.turning:                        # first loop of the turn
            self.turning = True
            self.turn_startL = self.m1_dist_mm
            self.turn_startR = self.m2_dist_mm
            self.linePID.reset()
        dL = self.m1_dist_mm - self.turn_startL
        dR = self.m2_dist_mm - self.turn_startR
        heading = ((dL - dR) if direction == 0 else (dR - dL)) / 90.4
        if heading >= angle * DEG2RAD:              # turn complete
            self.turning = False
            self.turnCounter += 1
            self.linePID.reset()
            return tL, tR
        ratio = max(0.0, (2.0 * radius - 90.4) / (2.0 * radius + 90.4))
        if direction == 0:
            tL, tR = speed, speed * ratio
        else:
            tR, tL = speed, speed * ratio
        return clampf(tL, 0.0, 900.0), clampf(tR, 0.0, 900.0)

    # ---- main()
    def run(self):
        w = self.w
        yield from self.calibrate()
        self.linePID.reset(); self.pidL.reset(); self.pidR.reset()
        last_loop = last_speed = checkpoint_debounce = w.micros()

        while True:
            now = w.micros()
            dt = ((now - last_loop) & 0xFFFFFFFF) / 1e6
            self.tel['loop_ms'] = dt * 1000
            last_loop = now

            yield self.read_sensors()                 # blocks ~2.5 ms
            error = self.compute_line_position()
            self.lastError = error

            self.update_motor_speeds((now - last_speed) & 0xFFFFFFFF)  # uses OLD `now`, like the C
            last_speed = now
            self.update_motor_distances()

            # checkpoint marker detection (all 8 sensors dark -> big sum), 400 ms debounce
            if ((now - checkpoint_debounce) & 0xFFFFFFFF) > CHECKPOINT_DEBOUNCE_US:
                if self.sum > CHECKPOINT_SUM:
                    self.checkpoint += 1
                    checkpoint_debounce = w.micros()

            # --- outer loop: line position -> steering correction
            corr = clampf(self.linePID.update(float(error), dt),
                          -LINE_CORRECTION_LIMIT, LINE_CORRECTION_LIMIT)

            # --- speed schedule (same order as the C)
            if self.turnCounter == 0:
                self.apply_speed_schedule()

            if USE_ACCEL_RAMP:                        # rises at accel_rate, falls immediately
                self.ramp = min(float(self.target_speed), self.ramp + self.accel_rate * dt)
                self.base = self.ramp
            else:
                self.base = float(self.target_speed)

            corr = clampf(corr, -self.base, self.base)

            tL = clampf(self.base - corr, 0.0, float(MAX_SPEED))
            tR = clampf(self.base + corr, 0.0, float(MAX_SPEED))

            # --- inner loop: target speed -> PWM duty, per wheel (independent PIDs)
            ffL = self.ff_duty('L', tL) if USE_FEEDFORWARD else 0.0
            ffR = self.ff_duty('R', tR) if USE_FEEDFORWARD else 0.0
            dutyL = self.pidL.update_sat(tL - self.m1_rpm, dt, ffL, 0.0, float(MAX_DUTY), USE_ANTIWINDUP)
            dutyR = self.pidR.update_sat(tR - self.m2_rpm, dt, ffR, 0.0, float(MAX_DUTY), USE_ANTIWINDUP)
            pwmL = int(clampf(dutyL, 0.0, float(MAX_DUTY)))
            pwmR = int(clampf(dutyR, 0.0, float(MAX_DUTY)))
            w.robot.pwm_l, w.robot.pwm_r = pwmL, pwmR   # forward()

            self.tel.update(error=error, correction=corr, tL=tL, tR=tR,
                            pwmL=pwmL, pwmR=pwmR, lost=self.sum < 300)

            # printf() blocks on the UART (1-byte buffer), then _delay_ms(20)
            delay = LOOP_DELAY_US
            if SIM_PRINTF_TIME:
                msg = (f"speed: {int(self.m1_rpm)} \tspeed2: {int(self.m2_rpm)} "
                       f"\terror: {int(error)} \tdist: {int(self.car_dist_mm)} "
                       f"\tcheck: {int(self.checkpoint)} \n")
                nchar = len(msg) + msg.count("\n")          # '\n' also sends '\r'
                delay += int(max(0, nchar - 1) * 10 / F_BAUD * 1e6)
            yield delay


# ============================ WORLD =========================================
class World:
    def __init__(self, track):
        self.track = track
        self.start_pose = (300.0, 1150.0, 0.0)
        self.show_trail = True
        self.reset()

    def reset(self):
        self.t_us = 0
        self.robot = Robot(self.start_pose)
        self.fw = Firmware(self)
        self.gen = self.fw.run()
        self.next_event = 0
        self.trail = []
        self.hist = []
        self._last_hist = 0

    def micros(self):
        return self.t_us & 0xFFFFFFFF

    def sensor_xy(self, i):
        r = self.robot
        lat = (3.5 - i) * SENSOR_SPACING_MM          # sensor 0 = LEFT (positive error = line left)
        return (r.x + math.cos(r.th) * AXLE_TO_SENSOR_MM + math.sin(r.th) * lat,
                r.y + math.sin(r.th) * AXLE_TO_SENSOR_MM - math.cos(r.th) * lat)

    def _is_line(self, xmm, ymm):
        px, py = int(xmm / PX_MM), int(ymm / PX_MM)
        if 0 <= px < W_PX and 0 <= py < H_PX:
            return self.track.get_at((px, py))[0] > 128
        return False

    def sample_sensors(self):
        s = SENSOR_SPOT_MM
        offs = ((0, 0), (s, 0), (-s, 0), (0, s), (0, -s))
        out = []
        for i in range(8):
            x, y = self.sensor_xy(i)
            c = sum(self._is_line(x + dx, y + dy) for dx, dy in offs) / len(offs)
            raw = SENSOR_BG_RAW_US - (SENSOR_BG_RAW_US - SENSOR_LINE_RAW_US) * c
            raw += random.gauss(0, SENSOR_NOISE_US)
            out.append(int(clampf(raw, 50, TIMEOUT_US)))
        return out

    def advance(self, total_us):
        end = self.t_us + total_us
        while self.t_us < end:
            while self.next_event <= self.t_us:
                self.next_event = self.t_us + max(1, int(next(self.gen)))
            step = min(1000, self.next_event - self.t_us, end - self.t_us)
            self.robot.step(step / 1e6)
            self.t_us += step
            if self.t_us - self._last_hist >= 20000:
                self._last_hist = self.t_us
                t = self.fw.tel
                self.hist.append((t['error'], self.fw.m1_rpm, self.fw.m2_rpm))
                self.hist = self.hist[-300:]
                self.trail.append((self.robot.x, self.robot.y))
                self.trail = self.trail[-6000:]

LW_PX = max(2, int(round(LINE_WIDTH_MM / PX_MM)))

def paint(track, a, b, color, width):
    pygame.draw.line(track, color, a, b, width)
    pygame.draw.circle(track, color, a, width // 2)
    pygame.draw.circle(track, color, b, width // 2)

def draw_polyline_mm(track, pts):
    p = [(int(x / PX_MM), int(y / PX_MM)) for x, y in pts]
    for a, b in zip(p, p[1:]):
        paint(track, a, b, LINE_COL, LW_PX)

class Turtle:
    def __init__(self, x, y, deg):
        self.x, self.y, self.th = x, y, math.radians(deg)
        self.pts = [(x, y)]
        self.marks = []

    def mark(self):
        self.marks.append((self.x, self.y, self.th))

    def fwd(self, d):
        self.x += math.cos(self.th) * d; self.y += math.sin(self.th) * d
        self.pts.append((self.x, self.y))

    def arc(self, r, deg, left=True):
        n = max(4, int(abs(deg) / 5))
        da = math.radians(deg) / n * (-1 if left else 1)
        chord = 2 * r * math.sin(abs(da) / 2)
        for _ in range(n):
            self.th += da / 2
            self.x += math.cos(self.th) * chord; self.y += math.sin(self.th) * chord
            self.th += da / 2
            self.pts.append((self.x, self.y))

def draw_marks(track, marks):
    """20 x 50 mm start-of-straight markers (20 along the track, 50 across)."""
    for x, y, th in marks:
        fx, fy, lx, ly = math.cos(th), math.sin(th), math.sin(th), -math.cos(th)
        pts = [((x + fx * a + lx * b) / PX_MM, (y + fy * a + ly * b) / PX_MM)
               for a, b in ((0, -25), (20, -25), (20, 25), (0, 25))]
        pygame.draw.polygon(track, LINE_COL, pts)

def mq_racer():
    """MQ Racer centreline, board 2400x1200. Sim X = length axis, Y = 1200 - width axis.
    Start line is the midpoint of the 800 mm straight; travel direction is the one
    where the first things after 400 mm are the left-then-right R100 bends."""
    t = Turtle(1450, 1050, 180)
    t.fwd(400);t.mark()
    t.arc(100, 90, False); t.arc(100, 90, True)
    t.fwd(500);t.mark()
    t.arc(200, 90, False)
    t.fwd(250);t.mark()
    t.arc(200, 90, False)
    t.fwd(1150);t.mark()
    t.arc(150, 90, False);t.arc(150, 90, True)
    t.fwd(300);t.mark()
    t.arc(150, 180, False)
    t.arc(100, 90, True); t.arc(150, 90, False)
    t.fwd(400)                                   # back to start line
    return t


def load_preset(world, n):
    world.track.fill(BG_COL)
    marks = []
    if n == 1:
        t = Turtle(600, 450, 0)
        t.fwd(800); t.arc(300, 180, False); t.fwd(800); t.arc(300, 180, False)
        pose = (600.0, 450.0, 0.0)
    elif n == 2:
        t = Turtle(200, 1000, 0)
        t.fwd(420); t.arc(200, 90, True); t.fwd(400); t.arc(200, 90, False); t.fwd(700)
        pose = (200.0, 1000.0, 0.0)
    elif n == 3:
        pts = [(x, 600 + 300 * math.sin(x / 250)) for x in range(200, 2201, 10)]
        t = Turtle(0, 0, 0); t.pts = pts
        pose = (200.0, pts[0][1], math.atan2(pts[1][1] - pts[0][1], 10))
    else:
        t = mq_racer()
        pose = (1544.0, 1050.0, math.pi)
        marks = t.marks
    draw_polyline_mm(world.track, t.pts)
    draw_marks(world.track, marks)
    world.start_pose = pose
    world.reset()


# ============================ RENDERING =====================================
def draw_robot(screen, world):
    r = world.robot

    def P(f, l):
        return ((r.x + math.cos(r.th) * f + math.sin(r.th) * l) / PX_MM,
                (r.y + math.sin(r.th) * f - math.cos(r.th) * l) / PX_MM)
    pygame.draw.polygon(screen, (70, 130, 220), [P(-35, -40), P(85, -40), P(85, 40), P(-35, 40)])
    for side in (1, -1):
        pygame.draw.polygon(screen, (20, 20, 20),
                            [P(-21.5, side * 40), P(21.5, side * 40), P(21.5, side * 54), P(-21.5, side * 54)])
    pygame.draw.polygon(screen, (255, 210, 60), [P(85, 0), P(60, 14), P(60, -14)])
    for i in range(8):
        x, y = world.sensor_xy(i)
        on = world.fw.raw[i] < 1250
        col = (255, 60, 60) if on else (120, 120, 120)
        pygame.draw.circle(screen, col, (int(x / PX_MM), int(y / PX_MM)), 3)
    pygame.draw.circle(screen, (255, 255, 255), (int(P(AXLE_TO_SENSOR_MM, 3.5 * SENSOR_SPACING_MM)[0]),
                                                int(P(AXLE_TO_SENSOR_MM, 3.5 * SENSOR_SPACING_MM)[1])), 5, 1)  # sensor 0 (left)


def plot(screen, rect, series, ymin, ymax, title, font):
    x, y, w, h = rect
    pygame.draw.rect(screen, (45, 45, 52), rect)
    z = y + h - (0 - ymin) / (ymax - ymin) * h
    if y <= z <= y + h:
        pygame.draw.line(screen, (90, 90, 100), (x, z), (x + w, z))
    for data, col in series:
        if len(data) > 1:
            pts = [(x + w * k / 300, y + h - (clampf(v, ymin, ymax) - ymin) / (ymax - ymin) * h)
                   for k, v in enumerate(data[-300:])]
            pygame.draw.lines(screen, col, False, pts, 1)
    screen.blit(font.render(title, True, (170, 170, 180)), (x + 3, y + 1))


def draw_sidebar(screen, world, running, speed, font):
    x0 = W_PX
    pygame.draw.rect(screen, (20, 20, 24), (x0, 0, SIDEBAR, WIN_H))
    fw, r, t = world.fw, world.robot, world.fw.tel
    lines = [
        (f"{'RUNNING' if running else 'PAUSED'}  x{speed}   t={world.t_us/1e6:6.2f}s", (255, 255, 255)),
        (f"loop {t['loop_ms']:5.1f} ms   car {fw.car_dist_mm:6.0f} mm", (200, 200, 200)),
        (f"turn#{fw.turnCounter}  checkpoint {fw.checkpoint}  sum {fw.sum}", (200, 200, 200)),
        (f"line error {t['error']:6d}   corr {t['correction']:7.1f}", (200, 200, 200)),
        (f"target  L {t['tL']:6.0f}  R {t['tR']:6.0f}  base {fw.base:4.0f}", (200, 200, 200)),
        (f"measured L {fw.m1_rpm:6.0f}  R {fw.m2_rpm:6.0f} rpm", (120, 220, 120)),
        (f"true     L {r.rpm_l:6.0f}  R {r.rpm_r:6.0f} rpm", (140, 140, 150)),
        (f"pwm     L {t['pwmL']:4d}  R {t['pwmR']:4d}", (200, 200, 200)),
    ]
    y = 8
    for txt, col in lines:
        screen.blit(font.render(txt, True, col), (x0 + 10, y)); y += 20
    if t['wrap']:
        screen.blit(font.render("! ENCODER 16-BIT WRAP (>32767 ticks/loop)", True, (255, 120, 60)), (x0 + 10, y))
    y += 20
    if t['lost']:
        screen.blit(font.render("! LINE LOST (sum < 300)", True, (255, 80, 80)), (x0 + 10, y))
    y += 24
    for i, raw in enumerate(fw.raw):                       # sensor bars (darkness as in code)
        d = max(0, TIMEOUT_US - raw) / TIMEOUT_US
        bx = x0 + 12 + i * 38
        pygame.draw.rect(screen, (45, 45, 52), (bx, y, 30, 50))
        pygame.draw.rect(screen, (255, 90, 90), (bx, y + 50 - int(50 * d), 30, int(50 * d)))
        screen.blit(font.render(f"{i}", True, (150, 150, 160)), (bx + 11, y + 52))
    y += 76
    h = world.hist
    plot(screen, (x0 + 10, y, SIDEBAR - 20, 110), [([e for e, _, _ in h], (255, 200, 80))],
         -3500, 3500, "line error (+-3500)", font)
    y += 120
    plot(screen, (x0 + 10, y, SIDEBAR - 20, 110),
         [([a for _, a, _ in h], (90, 200, 255)), ([b for _, _, b in h], (255, 120, 200))],
         -1200, 1200, "measured rpm L(blue) R(pink)", font)
    y += 125
    for s in ("SPACE run  R reset  C clear  1-4 presets",
              "LMB draw  Shift+LMB straight  RMB erase",
              "P place start  Q/E rotate  T trail",
              "UP/DOWN speed  S/L save/load"):
        screen.blit(font.render(s, True, (130, 130, 140)), (x0 + 10, y)); y += 17


# ============================ MAIN ==========================================
def main():
    pygame.init()
    screen = pygame.display.set_mode((W_PX + SIDEBAR, WIN_H))
    pygame.display.set_caption("Line follower sim")
    font = pygame.font.SysFont("consolas,menlo,dejavusansmono,monospace", 14)
    clock = pygame.time.Clock()
    track = pygame.Surface((W_PX, H_PX))
    world = World(track)
    load_preset(world, 4)
    running, sp_i = False, 2
    last = anchor = None
    erasing = False

    while True:
        frame = clock.tick(60) / 1000
        mods = pygame.key.get_mods()
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                return
            if e.type == pygame.KEYDOWN:
                k = e.key
                if k == pygame.K_ESCAPE: return
                elif k == pygame.K_SPACE: running = not running
                elif k == pygame.K_r: world.reset()
                elif k == pygame.K_c: track.fill(BG_COL); world.reset()
                elif k in (pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_4): load_preset(world, k - pygame.K_0)
                elif k == pygame.K_t: world.show_trail = not world.show_trail
                elif k == pygame.K_UP: sp_i = min(len(SPEEDS) - 1, sp_i + 1)
                elif k == pygame.K_DOWN: sp_i = max(0, sp_i - 1)
                elif k in (pygame.K_q, pygame.K_e):
                    x, y, th = world.start_pose
                    world.start_pose = (x, y, th + math.radians(-5 if k == pygame.K_q else 5))
                    world.reset()
                elif k == pygame.K_p:
                    mx, my = pygame.mouse.get_pos()
                    if mx < W_PX:
                        world.start_pose = (mx * PX_MM, my * PX_MM, world.start_pose[2]); world.reset()
                elif k == pygame.K_s:
                    pygame.image.save(track, "track.png")
                    json.dump(world.start_pose, open("track.json", "w"))
                elif k == pygame.K_l and os.path.exists("track.png"):
                    img = pygame.image.load("track.png")
                    track.blit(pygame.transform.scale(img, (W_PX, H_PX)), (0, 0))
                    if os.path.exists("track.json"):
                        world.start_pose = tuple(json.load(open("track.json")))
                    world.reset()

        if running:
            world.advance(int(min(frame, 0.05) * SPEEDS[sp_i] * 1e6))

        screen.blit(track, (0, 0))
        if world.show_trail and len(world.trail) > 1:
            pygame.draw.lines(screen, (80, 200, 120), False,
                              [(x / PX_MM, y / PX_MM) for x, y in world.trail], 1)
        sx, sy, sth = world.start_pose                      # red start line (visual only)
        pygame.draw.line(screen, (220, 40, 40),
                         ((sx + math.sin(sth) * 25) / PX_MM, (sy - math.cos(sth) * 25) / PX_MM),
                         ((sx - math.sin(sth) * 25) / PX_MM, (sy + math.cos(sth) * 25) / PX_MM), 2)
        if anchor is not None:
            pygame.draw.line(screen, (120, 160, 255), anchor, pygame.mouse.get_pos(), 2)
        draw_robot(screen, world)
        draw_sidebar(screen, world, running, SPEEDS[sp_i], font)
        pygame.display.flip()


if __name__ == "__main__":
    main()