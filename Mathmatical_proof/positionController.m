% filepath: c:\Users\conno\Documents\GitHub\esp32-drone-control-sytem\Mathmatical_proof\positionController.m
function [F, phi_des, theta_des] = positionController(x, xd, xd_dot, xd_ddot, params)
    % Position controller - computes desired force and attitude
    
    % Extract current state
    pos = x(1:3);
    vel = x(7:9);
    
    % Compute errors
    e_pos = pos - xd;
    e_vel = vel - xd_dot;
    
    % PD controller for acceleration
    a_des = -params.Kp_pos .* e_pos - params.Kd_pos .* e_vel + xd_ddot + [0; 0; params.g];
    
    % Total thrust needed
    F = params.mass * norm(a_des);
    
    % Desired attitude
    phi_des = atan2(a_des(2), sqrt(a_des(1)^2 + a_des(3)^2));
    theta_des = atan2(-a_des(1), a_des(3));
end