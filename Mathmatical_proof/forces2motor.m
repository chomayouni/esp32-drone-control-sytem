% filepath: c:\Users\conno\Documents\GitHub\esp32-drone-control-sytem\Mathmatical_proof\forces2motor.m
function motorSpeeds = forces2motor(F, M, params)
    % Convert desired thrust and moments to motor speeds
    
    % Allocation matrix inverse calculation
    A = [
        params.kF, params.kF, params.kF, params.kF;
        0, params.armLength*params.kF, 0, -params.armLength*params.kF;
        params.armLength*params.kF, 0, -params.armLength*params.kF, 0;
        params.kM, -params.kM, params.kM, -params.kM
    ];
    
    % Solve for squared motor speeds
    motorSpeeds_squared = A \ [F; M];
    
    % Ensure no negative values by clamping
    motorSpeeds_squared = max(motorSpeeds_squared, 0);
    
    % Take square root to get motor speeds
    motorSpeeds = sqrt(motorSpeeds_squared);
end