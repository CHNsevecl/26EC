"""
Ball-on-pipe control law comparison (pixel-domain simulation)
=============================================================
Compares step response of:
  CUR : current main.cpp branch PID (deadband 20px, Kp linear fade, rpm==0 nudge=dead,
        no anti-windup, raw diff velocity, loss -> dx jumps to 0)
  NEW : optimized P-D with velocity feedback + stall boost + small deadband + smoothing
        + ball-loss hold.

Plant: x'' = G*theta (double integrator) + rolling resistance/stick threshold.
  theta commanded via first-order servo lag.
  Measurement noise + occasional frame drop.
Metrics: steady-state |e|, first enter +/-5px, settle (stay within +/-8px), overshoot.
"""
import math, random, statistics

# --- plant params (pixel domain) ---
G = 16000.0        # px/s^2 per rad  (0.1rad ~= 1 m/s^2, 1m ~= 1600px)
RESIST = 90.0      # px/s^2 stick/rolling-resistance threshold
SERVO_TAU = 0.03   # servo lag
DT = 0.02          # 50 Hz
NOISE = 1.5        # detection noise px (std)
DROP = 0.02        # frame-drop probability

random.seed(7)

class Plant:
    def __init__(self, x0):
        self.x, self.v, self.ta = x0, 0.0, 0.0
    def step(self, cmd):
        self.ta += (cmd - self.ta) * (DT / SERVO_TAU)
        a = G * math.sin(self.ta)
        if abs(self.v) < 2.0:
            if abs(a) <= RESIST:
                a, self.v = 0.0, 0.0
            else:
                a -= math.copysign(RESIST, a)
        else:
            a -= math.copysign(RESIST, self.v)
        self.v += a * DT
        self.x += self.v * DT

# ---------------- CUR: faithful copy of current main.cpp logic ----------------
class CurCtrl:
    def __init__(self):
        self.kp, self.ki, self.kd, self.k_dx = 0.0008, 0.0013, 0.0016, 1.5
        self.integral, self.prev_e, self.first_run = 0.0, 0.0, True
        self.rpm = 0.0
        self.prev_dx = None
        self.last_dx = 0.0
    def pid(self, err, kp_eff):
        p = kp_eff * err
        self.integral += self.ki * err * DT
        d = 0.0 if self.first_run else self.kd * (err - self.prev_e) / DT
        self.first_run = False
        self.prev_e = err
        return p + self.integral + d
    def run(self, e_meas, dt_ms, t):
        # visual thread: rpm = (dx-prev_dx)/dt, dx computed from ball pos
        if e_meas is None:
            # loss -> target reset to center -> dx jumps to 0
            e = 0.0
            self.rpm = 0.0
            self.prev_dx = None
            return 0.0
        e = e_meas
        if self.prev_dx is not None:
            self.rpm = (e - self.prev_dx) / dt_ms
        self.prev_dx = e
        kp_eff = self.kp if abs(e) > 100 else self.kp * abs(e) / 100.0
        DB = 20.0
        out = 0.0
        if e < -DB:
            if self.rpm * e < 0:
                f = 0.5 if e > -50 else 1.0
                out = self.pid(e + self.rpm * dt_ms * self.k_dx * f, kp_eff)
            elif e > -60 and self.rpm == 0.0:
                out = -0.03          # dead: rpm==0 exact
            else:
                out = self.pid(e, kp_eff)
        elif e > DB:
            if self.rpm * e < 0:
                f = 0.5 if e < 50 else 1.0
                out = self.pid(e + self.rpm * dt_ms * self.k_dx * f, kp_eff)
            elif e < 60 and self.rpm == 0.0:
                out = 0.03           # dead
            else:
                out = self.pid(e, kp_eff)
        return max(-0.5, min(0.5, out))

# ---------------- NEW: optimized P-D + velocity + stall boost ----------------
class NewCtrl:
    def __init__(self):
        self.Kp, self.Kd = 0.004, 0.001    # per px, per px/s  (wn~8, zeta~1)
        self.DB = 3.0
        self.V_STALL = 20.0                # px/s
        self.STALL_MULT = 3.0
        self.rpm_f = 0.0                   # px/s smoothed
        self.prev_x, self.prev_t = None, None
        self.last_e, self.ball_lost = 0.0, False
    def run(self, e_meas, dt_ms, t):
        # velocity estimate from consecutive good measurements (px/s)
        if e_meas is not None:
            if self.prev_x is not None and t - self.prev_t > 0:
                v = -(e_meas - self.prev_x) / (t - self.prev_t)   # +x velocity
            else:
                v = 0.0
            self.prev_x, self.prev_t = e_meas, t
            # low-pass
            self.rpm_f = 0.4 * self.rpm_f + 0.6 * v
            self.last_e = e_meas
            self.ball_lost = False
        else:
            self.ball_lost = True
            v = 0.0
        if self.ball_lost or abs(self.last_e) <= self.DB:
            return 0.0
        kp = self.Kp if abs(self.rpm_f) >= self.V_STALL else self.Kp * self.STALL_MULT
        out = kp * self.last_e - self.Kd * self.rpm_f
        return max(-0.5, min(0.5, out))

# ---------------- driver ----------------
def sim(factory, x0, T=12.0):
    p = Plant(x0); c = factory()
    times, xs = [], []
    t, prev_x, prev_t = 0.0, None, None
    while t < T:
        if random.random() < DROP:
            e_meas = None
        else:
            e_meas = -(p.x + random.gauss(0, NOISE))
        cmd = c.run(e_meas, DT * 1000.0, t)
        p.step(cmd)
        times.append(t); xs.append(p.x)
        t += DT
    half = xs[int(len(xs) * 0.6):]
    e_ss = statistics.mean(half); e_abs = statistics.mean(abs(v) for v in half)
    enter = next((times[i] for i, v in enumerate(xs) if abs(v) < 5), None)
    settle = next((times[i] for i in range(len(times)) if all(abs(v) < 8 for v in xs[i:])), None)
    overshoot = max(max(xs), -min(xs))
    return e_ss, e_abs, enter, settle, overshoot, xs, times

def run(name, factory, x0):
    e_ss, e_abs, enter, settle, ov, xs, times = sim(factory, x0)
    print(f"[{name}] x0={x0}px")
    print(f"  e_ss={e_ss:+.1f}px  |e|avg={e_abs:.1f}px")
    print(f"  enter+-5px={enter if enter else float('nan'):.2f}s  settle+-8px={settle if settle else float('nan'):.2f}s  overshoot={ov:.1f}px")
    print()
    return xs

print("="*64)
print("STEP 1: release from x=+200px (ball right, e=-200)")
print("="*64)
xs_cur = run("CUR", CurCtrl, 200.0)
xs_new = run("NEW", NewCtrl, 200.0)

print("="*64)
print("STEP 2: release from x=+80px (near deadband, exposes static error)")
print("="*64)
run("CUR", CurCtrl, 80.0)
run("NEW", NewCtrl, 80.0)

import json
json.dump({"cur": xs_cur, "new": xs_new, "t": [i*DT for i in range(len(xs_new))]},
          open("sim_result.json", "w"))
print("saved sim_result.json")
