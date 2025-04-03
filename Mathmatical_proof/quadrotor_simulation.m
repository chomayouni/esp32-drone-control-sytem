% filepath: c:\Users\conno\Documents\GitHub\esp32-drone-control-sytem\Mathmatical_proof\quadrotor_simulation.m
%% Quadrotor Drone Simulation
clear all; close all; clc;

% Load drone parameters
params = drone_params();

% Initialize state history
x_history = zeros(12, params.numSteps);
x_history(:, 1) = params.x0;

% Initialize motor history
motor_history = zeros(4, params.numSteps - 1); % Adjusted to match time steps minus one

% Main simulation loop
for k = 1:params.numSteps-1
    % Current state
    x = x_history(:, k);
    t = params.timeVec(k);
    
    % Desired position and derivatives
    % (In a real scenario, these could vary with time)
    xd = params.xd;
    xd_dot = params.xd_dot;
    xd_ddot = params.xd_ddot;
    psi_des = 0;  % Desired yaw angle
    
    % Position controller
    [F, phi_des, theta_des] = positionController(x, xd, xd_dot, xd_ddot, params);
    
    % Attitude controller
    M = attitudeController(x, phi_des, theta_des, psi_des, params);
    
    % Convert to motor commands
    motorSpeeds = forces2motor(F, M, params);
    motor_history(:, k) = motorSpeeds;
    
    % Simulate system dynamics (using simple Euler integration)
    xdot = droneStateDynamics(t, x, motorSpeeds, params);
    x_next = x + xdot * params.dt;
    x_history(:, k+1) = x_next;
end

% Extract position and orientation for plotting
pos_history = x_history(1:3, :);
euler_history = x_history(4:6, :);

% Plotting
figure(1)
subplot(3,1,1)
plot(params.timeVec, pos_history(1,:), 'LineWidth', 2)
hold on
plot(params.timeVec, ones(size(params.timeVec))*params.xd(1), 'r--', 'LineWidth', 1.5)
ylabel('X Position (m)')
grid on

subplot(3,1,2)
plot(params.timeVec, pos_history(2,:), 'LineWidth', 2)
hold on
plot(params.timeVec, ones(size(params.timeVec))*params.xd(2), 'r--', 'LineWidth', 1.5)
ylabel('Y Position (m)')
grid on

subplot(3,1,3)
plot(params.timeVec, pos_history(3,:), 'LineWidth', 2)
hold on
plot(params.timeVec, ones(size(params.timeVec))*params.xd(3), 'r--', 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('Z Position (m)')
grid on

figure(2)
subplot(3,1,1)
plot(params.timeVec, euler_history(1,:)*180/pi, 'LineWidth', 2)
ylabel('Roll (deg)')
grid on

subplot(3,1,2)
plot(params.timeVec, euler_history(2,:)*180/pi, 'LineWidth', 2)
ylabel('Pitch (deg)')
grid on

subplot(3,1,3)
plot(params.timeVec, euler_history(3,:)*180/pi, 'LineWidth', 2)
xlabel('Time (s)')
ylabel('Yaw (deg)')
grid on

figure(3)
for i = 1:4
    subplot(4,1,i)
    plot(params.timeVec(1:end-1), motor_history(i,:), 'LineWidth', 2)
    ylabel(['Motor ', num2str(i), ' Speed'])
    grid on
    if i == 4
        xlabel('Time (s)')
    end
end

% 3D Trajectory Visualization
figure(4)
plot3(pos_history(1,:), pos_history(2,:), pos_history(3,:), 'LineWidth', 2)
hold on
plot3(params.xd(1), params.xd(2), params.xd(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r')
plot3(params.x0(1), params.x0(2), params.x0(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g')
grid on
xlabel('X Position (m)')
ylabel('Y Position (m)')
zlabel('Z Position (m)')
title('Quadrotor 3D Trajectory')
legend('Trajectory', 'Target', 'Start')
axis equal

disp('Simulation complete!')