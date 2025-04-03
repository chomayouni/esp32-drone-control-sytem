% filepath: c:\Users\conno\Documents\GitHub\esp32-drone-control-sytem\Mathmatical_proof\attitudeController.m
function M = attitudeController(x, phi_des, theta_des, psi_des, params)
    % Attitude controller - computes desired moments
    
    % Extract current Euler angles and angular velocities
    phi = x(4);
    theta = x(5);
    psi = x(6);
    omega = x(10:12);
    
    % Compute attitude errors
    e_att = [phi - phi_des; theta - theta_des; psi - psi_des];
    
    % PD controller for torques
    M = -params.Kp_att .* e_att - params.Kd_att .* omega;
end