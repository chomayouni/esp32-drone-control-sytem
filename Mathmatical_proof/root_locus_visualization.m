% filepath: c:\Users\conno\Documents\GitHub\esp32-drone-control-sytem\Mathmatical_proof\root_locus_visualization.m
%% Root Locus Visualization for PID Controllers
clear all; close all; clc;
disp('Generating root locus plots');
% Load drone parameters
params = drone_params();

%% Position Controller (X, Y, Z)
% Transfer function: G(s) = Kp + Kd * s
s = tf('s');
Kp_pos = params.Kp_pos;
Kd_pos = params.Kd_pos;

% Root locus for X position controller
Gx = Kp_pos(1) + Kd_pos(1) * s;
figure;
rlocus(Gx);
title('Root Locus for X Position Controller');
grid on;

% Root locus for Y position controller
Gy = Kp_pos(2) + Kd_pos(2) * s;
figure;
rlocus(Gy);
title('Root Locus for Y Position Controller');
grid on;

% Root locus for Z position controller
Gz = Kp_pos(3) + Kd_pos(3) * s;
figure;
rlocus(Gz);
title('Root Locus for Z Position Controller');
grid on;

%% Attitude Controller (Roll, Pitch, Yaw)
% Transfer function: G(s) = Kp + Kd * s
Kp_att = params.Kp_att;
Kd_att = params.Kd_att;

% Root locus for Roll controller
G_roll = Kp_att(1) + Kd_att(1) * s;
figure;
rlocus(G_roll);
title('Root Locus for Roll Controller');
grid on;

% Root locus for Pitch controller
G_pitch = Kp_att(2) + Kd_att(2) * s;
figure;
rlocus(G_pitch);
title('Root Locus for Pitch Controller');
grid on;

% Root locus for Yaw controller
G_yaw = Kp_att(3) + Kd_att(3) * s;
figure;
rlocus(G_yaw);
title('Root Locus for Yaw Controller');
grid on;

disp('Root locus plots generated!');