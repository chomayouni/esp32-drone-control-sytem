% filepath: c:\Users\conno\Documents\GitHub\esp32-drone-control-sytem\Mathmatical_proof\droneStateDynamics.m
function xdot = droneStateDynamics(t, x, motorSpeeds, params)
    % Drone state dynamics for ODE solver
    
    % Extract states
    pos = x(1:3);
    euler = x(4:6);
    vel = x(7:9);
    omega = x(10:12);
    
    % Get current rotation matrix
    R = rotationMatrix(euler(1), euler(2), euler(3));
    
    % Calculate thrust and moments from motor speeds
    [F, M] = motor2forces(motorSpeeds, params);
    
    % Translational dynamics (in world frame)
    F_thrust = [0; 0; F];
    F_world = R * F_thrust;
    
    % Add drag force proportional to velocity
    F_drag = -params.kD * vel;
    
    % Total acceleration
    acc = F_world / params.mass + [0; 0; -params.g] + F_drag / params.mass;
    
    % Rotational dynamics (in body frame)
    omega_dot = params.I \ (M - cross(omega, params.I * omega));
    
    % Euler angle rates
    phi = euler(1);
    theta = euler(2);
    psi = euler(3);
    
    % Conversion matrix: angular velocity to Euler rates
    W = [
        1, sin(phi)*tan(theta), cos(phi)*tan(theta);
        0, cos(phi), -sin(phi);
        0, sin(phi)/cos(theta), cos(phi)/cos(theta)
    ];
    
    % Calculate Euler angle derivatives
    euler_dot = W * omega;
    
    % Return state derivatives
    xdot = [vel; euler_dot; acc; omega_dot];
end