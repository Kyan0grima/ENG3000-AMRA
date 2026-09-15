pid.kp = 1.0;
pid.ki = 0.1;
pid.kd = 0.01;
pid.integral = 0;
pid.integral_limit = 100;
pid.prev_error = 0;

%% Motor + Gearbox Model — Pololu #5185 (30:1 HPCB 6V w/ encoder)
% All quantities referenced to the GEARBOX OUTPUT shaft.

%% Datasheet values
V_nom       = 6;                  % V, rated voltage
I_stall     = 1.5;                % A, stall current
I_noload    = 0.150;              % A, no-load current
N_noload    = 1100;               % RPM, no-load speed (output shaft)
T_stall_kgcm= 0.45;                % kg*cm, extrapolated stall torque (output shaft)
gear_ratio  = 29.86;               % exact ratio (31*33*35*34)/(16*14*13*14)
enc_cpr_motor = 12;                % counts per rev, motor shaft
enc_cpr_output = enc_cpr_motor * gear_ratio;  % counts per rev, output shaft

%% Unit conversions
w_noload = N_noload * 2*pi/60;               % rad/s
T_stall  = T_stall_kgcm * 0.0980665;         % N*m  (1 kgf*cm = 0.0980665 N*m)

%% Derived electrical/mechanical parameters (output-shaft referenced)
R  = V_nom / I_stall;                         % ohms
Ke = (V_nom - I_noload*R) / w_noload;         % V/(rad/s)  back-EMF constant
Kt = T_stall / I_stall;                       % N*m/A      torque constant
b  = Kt * I_noload / w_noload;                % N*m/(rad/s) viscous friction

fprintf('R  = %.4f ohm\n', R);
fprintf('Ke = %.5f V/(rad/s)\n', Ke);
fprintf('Kt = %.5f N*m/A\n', Kt);
fprintf('b  = %.6e N*m/(rad/s)\n', b);

%% Inertia — NOT given by datasheet, must be estimated or identified
% Placeholder: assumes a mechanical time constant of ~20 ms, typical for
% small metal gearmotors this size. REPLACE this once you have real
% step-response data (see notes below).
tau_m_guess = 0.020;                          % seconds, PLACEHOLDER
b_eff = b + (Kt*Ke)/R;                        % effective damping incl. back-EMF loading
J = tau_m_guess * b_eff;                      % kg*m^2 (rough estimate)

fprintf('J (estimated) = %.4e kg*m^2\n', J);

%% Build the transfer function: input = voltage (V), output = speed (rad/s)
num = Kt/R;
den = [J, b_eff];
motor_tf = tf(num, den)

%% Quick check: does it reproduce the datasheet no-load speed?
w_ss_check = dcgain(motor_tf) * V_nom;
fprintf('Predicted no-load speed: %.1f RPM (datasheet: %.0f RPM)\n', ...
    w_ss_check*60/(2*pi), N_noload);