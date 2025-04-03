% filepath: c:\Users\conno\Documents\GitHub\esp32-drone-control-sytem\Mathmatical_proof\motor2forces.m
function [F, M] = motor2forces(motorSpeeds, params)
    % Convert motor speeds to thrust and moments
    
    F_i = params.kF * motorSpeeds.^2;  % Thrust from each motor
    
    % Total thrust
    F = sum(F_i);
    
    % Moments from each motor
    M = [
        params.armLength * (F_i(2) - F_i(4));  % Roll moment
        params.armLength * (F_i(1) - F_i(3));  % Pitch moment
        params.kM * (motorSpeeds(1)^2 - motorSpeeds(2)^2 + motorSpeeds(3)^2 - motorSpeeds(4)^2)  % Yaw moment
    ];
end