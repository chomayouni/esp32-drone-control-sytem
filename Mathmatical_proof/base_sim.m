%% Drone Linear Flight Simulation
% This simulation models the drone dynamics based on the Carbon Aeronautics Quadcopter Manual
% It implements a linear model that can be used to tune a 3D simulation in Simulink

% This was the starting simulation that I put all of the math into. It does not do much, but it is a good starting point for the rest of the simulations.
% It is a linear model that can be used to tune a 3D simulation in Simulink.
% I split the simulation into two parts: the first part is the linear model, and the second part is the PID controller.

%% Parameters and Constants
% Simulation time parameters
Ts = 0.004;                % Sample time (250 Hz control loop as per p.96)
sim_time = 10;             % Simulation time in seconds
t = 0:Ts:sim_time;         % Time vector
N = length(t);             % Number of samples

% Drone physical parameters
mass = 0.250;              % Drone mass in kg (p.201)
motor_distance_x = 0.08;   % Distance from center to motor in x direction in m (p.201)
motor_distance_y = 0.05;   % Distance from center to motor in y direction in m (p.201)
motor_mass = 0.008;        % Mass of each motor in kg (p.203)
esc_mass = 0.007;          % Mass of each ESC in kg (p.203)

% Moment of inertia calculations - based on p.203
Ix = 4 * motor_mass * motor_distance_x^2 + 4 * esc_mass * (motor_distance_x/2)^2;
Iy = 4 * motor_mass * motor_distance_y^2 + 4 * esc_mass * (motor_distance_y/2)^2;
Iz = 4 * motor_mass * (motor_distance_x^2 + motor_distance_y^2) + ...
     4 * esc_mass * ((motor_distance_x/2)^2 + (motor_distance_y/2)^2);

% Motor and thrust parameters
thrust_coeff = 160;        % Thrust coefficient (g per throttle) from p.191
kV_rating = 5000;          % Motor kV rating (p.195)
motor_tau = 0.03;          % Motor time constant in seconds (p.196)

% PID controller parameters for rate mode - from p.94
P_RateRoll = 0.6; I_RateRoll = 3.5; D_RateRoll = 0.03;
P_RatePitch = 0.6; I_RatePitch = 3.5; D_RatePitch = 0.03;
P_RateYaw = 2; I_RateYaw = 12; D_RateYaw = 0;

% Sensor parameters
sensor_cutoff = 2 * pi * 10; % 10 Hz low-pass filter cutoff frequency (p.198)

% System dynamics transfer functions - based on p.203-209
% Roll and pitch dynamics: 115/s
% Yaw dynamics: 4.8/s
% Vertical velocity dynamics: 2.5/s
roll_dynamics_num = 115;
roll_dynamics_den = [1 0];
pitch_dynamics_num = 115;
pitch_dynamics_den = [1 0];
yaw_dynamics_num = 4.8;
yaw_dynamics_den = [1 0];
vertical_dynamics_num = 2.5;
vertical_dynamics_den = [1 0];

% Motor dynamics transfer function: 1/(0.03s + 1)
motor_dynamics_num = 1;
motor_dynamics_den = [motor_tau 1];

% Sensor dynamics transfer function: (2*pi*10)/(s + 2*pi*10)
sensor_dynamics_num = sensor_cutoff;
sensor_dynamics_den = [1 sensor_cutoff];

%% Input Signals
% Initialize input signals (commands)
input_roll = zeros(N, 1);
input_pitch = zeros(N, 1);
input_yaw = zeros(N, 1);
input_throttle = zeros(N, 1);

% Create test input signals
% Step input for roll at t=1s
roll_step_idx = round(1/Ts) + 1;
input_roll(roll_step_idx:end) = 30;  % 30 degrees/s desired roll rate

% Step input for pitch at t=3s
pitch_step_idx = round(3/Ts) + 1;
input_pitch(pitch_step_idx:end) = 20;  % 20 degrees/s desired pitch rate

% Step input for yaw at t=5s
yaw_step_idx = round(5/Ts) + 1;
input_yaw(yaw_step_idx:end) = 15;  % 15 degrees/s desired yaw rate

% Step input for throttle at t=0.5s
throttle_step_idx = round(0.5/Ts) + 1;
input_throttle(throttle_step_idx:end) = 200;  % Throttle command in μs

%% Create System Models
% Create transfer functions
roll_dynamics_tf = tf(roll_dynamics_num, roll_dynamics_den);
pitch_dynamics_tf = tf(pitch_dynamics_num, pitch_dynamics_den);
yaw_dynamics_tf = tf(yaw_dynamics_num, yaw_dynamics_den);
vertical_dynamics_tf = tf(vertical_dynamics_num, vertical_dynamics_den);
motor_dynamics_tf = tf(motor_dynamics_num, motor_dynamics_den);
sensor_dynamics_tf = tf(sensor_dynamics_num, sensor_dynamics_den);

% Create discrete-time models for simulation
roll_dynamics_d = c2d(roll_dynamics_tf, Ts, 'zoh');
pitch_dynamics_d = c2d(pitch_dynamics_tf, Ts, 'zoh');
yaw_dynamics_d = c2d(yaw_dynamics_tf, Ts, 'zoh');
vertical_dynamics_d = c2d(vertical_dynamics_tf, Ts, 'zoh');
motor_dynamics_d = c2d(motor_dynamics_tf, Ts, 'zoh');
sensor_dynamics_d = c2d(sensor_dynamics_tf, Ts, 'zoh');

%% PID Controller Implementation
% Initialize controller variables
error_rate_roll = zeros(N, 1);
error_rate_pitch = zeros(N, 1);
error_rate_yaw = zeros(N, 1);

prev_error_rate_roll = 0;
prev_error_rate_pitch = 0;
prev_error_rate_yaw = 0;

prev_iterm_rate_roll = 0;
prev_iterm_rate_pitch = 0;
prev_iterm_rate_yaw = 0;

motor_input_roll = zeros(N, 1);
motor_input_pitch = zeros(N, 1);
motor_input_yaw = zeros(N, 1);
motor_input_throttle = zeros(N, 1);

% Initialize state variables
rate_roll_actual = zeros(N, 1);
rate_pitch_actual = zeros(N, 1);
rate_yaw_actual = zeros(N, 1);
vertical_velocity = zeros(N, 1);

rate_roll_measured = zeros(N, 1);
rate_pitch_measured = zeros(N, 1);
rate_yaw_measured = zeros(N, 1);

% Initialize motor outputs
motor_output1 = zeros(N, 1);
motor_output2 = zeros(N, 1);
motor_output3 = zeros(N, 1);
motor_output4 = zeros(N, 1);

% PID controller function (based on p.99)
function [output, error, iterm] = pid_controller(error, prev_error, prev_iterm, P, I, D, Ts)
    % Calculate P term
    pterm = P * error;
    
    % Calculate I term with anti-windup
    iterm = prev_iterm + I * (error + prev_error) * Ts / 2;
    if iterm > 400
        iterm = 400;
    elseif iterm < -400
        iterm = -400;
    end
    
    % Calculate D term
    dterm = D * (error - prev_error) / Ts;
    
    % Calculate PID output with saturation
    output = pterm + iterm + dterm;
    if output > 400
        output = 400;
    elseif output < -400
        output = -400;
    end
end

%% Simulation Loop
for k = 2:N
    % Get measured rotation rates (from previous actual values through sensor dynamics)
    % Using state-space equations directly instead of lsim for single-step simulation
    % For a first-order low-pass filter (2*pi*10)/(s + 2*pi*10)
    % Apply the filter: y(k) = a*y(k-1) + b*u(k-1) where
    % a = exp(-2*pi*10*Ts), b = 1-a
    a = exp(-sensor_cutoff * Ts);
    b = 1 - a;
    rate_roll_measured(k) = a * rate_roll_measured(k-1) + b * rate_roll_actual(k-1);
    rate_pitch_measured(k) = a * rate_pitch_measured(k-1) + b * rate_pitch_actual(k-1);
    rate_yaw_measured(k) = a * rate_yaw_measured(k-1) + b * rate_yaw_actual(k-1);
    
    % Calculate errors
    error_rate_roll(k) = input_roll(k) - rate_roll_measured(k);
    error_rate_pitch(k) = input_pitch(k) - rate_pitch_measured(k);
    error_rate_yaw(k) = input_yaw(k) - rate_yaw_measured(k);
    
    % Execute PID controllers
    [motor_input_roll(k), error_out_roll, iterm_roll] = pid_controller(error_rate_roll(k), prev_error_rate_roll, prev_iterm_rate_roll, P_RateRoll, I_RateRoll, D_RateRoll, Ts);
    [motor_input_pitch(k), error_out_pitch, iterm_pitch] = pid_controller(error_rate_pitch(k), prev_error_rate_pitch, prev_iterm_rate_pitch, P_RatePitch, I_RatePitch, D_RatePitch, Ts);
    [motor_input_yaw(k), error_out_yaw, iterm_yaw] = pid_controller(error_rate_yaw(k), prev_error_rate_yaw, prev_iterm_rate_yaw, P_RateYaw, I_RateYaw, D_RateYaw, Ts);
    
    % Update PID memory
    prev_error_rate_roll = error_out_roll;
    prev_error_rate_pitch = error_out_pitch;
    prev_error_rate_yaw = error_out_yaw;
    
    prev_iterm_rate_roll = iterm_roll;
    prev_iterm_rate_pitch = iterm_pitch;
    prev_iterm_rate_yaw = iterm_yaw;
    
    % Process throttle command
    motor_input_throttle(k) = input_throttle(k);
    
    % Calculate individual motor commands (based on p.103)
    motor_output1(k) = 1.024 * (motor_input_throttle(k) - motor_input_roll(k) - motor_input_pitch(k) - motor_input_yaw(k));
    motor_output2(k) = 1.024 * (motor_input_throttle(k) - motor_input_roll(k) + motor_input_pitch(k) + motor_input_yaw(k));
    motor_output3(k) = 1.024 * (motor_input_throttle(k) + motor_input_roll(k) + motor_input_pitch(k) - motor_input_yaw(k));
    motor_output4(k) = 1.024 * (motor_input_throttle(k) + motor_input_roll(k) - motor_input_pitch(k) + motor_input_yaw(k));
    
    % Apply motor dynamics (first-order system with time constant motor_tau)
    % y(k) = a*y(k-1) + b*u(k-1)
    a_motor = exp(-Ts/motor_tau);
    b_motor = 1 - a_motor;
    
    % Calculate motor outputs with dynamics
    % Initialize motor thrusts if they don't exist
    if ~exist('motor1_thrust', 'var') || isempty(motor1_thrust)
        motor1_thrust = 0;
    end
    if ~exist('motor2_thrust', 'var') || isempty(motor2_thrust)
        motor2_thrust = 0;
    end
    if ~exist('motor3_thrust', 'var') || isempty(motor3_thrust)
        motor3_thrust = 0;
    end
    if ~exist('motor4_thrust', 'var') || isempty(motor4_thrust)
        motor4_thrust = 0;
    end
    
    % Apply first-order dynamics
    motor1_thrust = a_motor * motor1_thrust + b_motor * motor_output1(k-1);
    motor2_thrust = a_motor * motor2_thrust + b_motor * motor_output2(k-1);
    motor3_thrust = a_motor * motor3_thrust + b_motor * motor_output3(k-1);
    motor4_thrust = a_motor * motor4_thrust + b_motor * motor_output4(k-1);
    
    % Apply quadcopter dynamics to calculate actual rotation rates
    roll_input = motor3_thrust + motor4_thrust - motor1_thrust - motor2_thrust;
    pitch_input = motor2_thrust + motor3_thrust - motor1_thrust - motor4_thrust;
    yaw_input = motor2_thrust + motor4_thrust - motor1_thrust - motor3_thrust;
    thrust_input = motor1_thrust + motor2_thrust + motor3_thrust + motor4_thrust;
    
    % For integrator systems (e.g., 115/s), the discrete update is:
    % y(k) = y(k-1) + Ts*gain*u(k-1)
    rate_roll_actual(k) = rate_roll_actual(k-1) + Ts * roll_dynamics_num * roll_input;
    rate_pitch_actual(k) = rate_pitch_actual(k-1) + Ts * pitch_dynamics_num * pitch_input;
    rate_yaw_actual(k) = rate_yaw_actual(k-1) + Ts * yaw_dynamics_num * yaw_input;
    vertical_velocity(k) = vertical_velocity(k-1) + Ts * vertical_dynamics_num * thrust_input;
end

%% Visualize Results
figure;

% Plot Roll Rate Response
subplot(4, 1, 1);
plot(t, input_roll, 'r--', t, rate_roll_actual, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Roll Rate (°/s)');
legend('Desired', 'Actual');
title('Roll Rate Response');

% Plot Pitch Rate Response
subplot(4, 1, 2);
plot(t, input_pitch, 'r--', t, rate_pitch_actual, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Pitch Rate (°/s)');
legend('Desired', 'Actual');
title('Pitch Rate Response');

% Plot Yaw Rate Response
subplot(4, 1, 3);
plot(t, input_yaw, 'r--', t, rate_yaw_actual, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Yaw Rate (°/s)');
legend('Desired', 'Actual');
title('Yaw Rate Response');

% Plot Motor Commands
subplot(4, 1, 4);
plot(t, motor_output1, 'r-', t, motor_output2, 'g-', t, motor_output3, 'b-', t, motor_output4, 'k-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Motor Command (μs)');
legend('Motor 1', 'Motor 2', 'Motor 3', 'Motor 4');
title('Motor Commands');

% Figure for motor inputs and vertical velocity
figure;
subplot(2, 1, 1);
plot(t, input_throttle, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Throttle Command (μs)');
title('Throttle Input');

subplot(2, 1, 2);
plot(t, vertical_velocity, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Vertical Velocity (cm/s)');
title('Vertical Velocity Response');

disp('Simulation completed successfully!');