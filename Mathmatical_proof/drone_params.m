% filepath: c:\Users\conno\Documents\GitHub\esp32-drone-control-sytem\Mathmatical_proof\drone_params.m
function params = drone_params()
    % Drone physical parameters
    params.mass = 1.5;             % kg
    params.Ixx = 0.025;            % kg*m^2
    params.Iyy = 0.025;            % kg*m^2
    params.Izz = 0.050;            % kg*m^2
    params.I = diag([params.Ixx, params.Iyy, params.Izz]);
    params.g = 9.81;               % m/s^2
    params.armLength = 0.225;      % meters
    params.kF = 1.0e-5;            % N/(rad/s)^2
    params.kM = 1.0e-6;            % N*m/(rad/s)^2
    params.kD = 0.1;               % drag coefficient

    % Controller gains
    params.Kp_pos = [5; 5; 10];
    params.Kd_pos = [4; 4; 6]; 
    params.Kp_att = [40; 40; 10];
    params.Kd_att = [10; 10; 5];
    
    % Simulation parameters
    params.dt = 0.01;              % seconds
    params.timeTotal = 10;         % seconds
    params.timeVec = 0:params.dt:params.timeTotal;
    params.numSteps = length(params.timeVec);

    % Initial conditions
    params.x0 = zeros(12, 1);
    params.x0(3) = 0;              % Initial height (z position)

    % Desired states
    params.xd = [2; 2; 5];         % Desired position
    params.xd_dot = [0; 0; 0];     % Desired velocity
    params.xd_ddot = [0; 0; 0];    % Desired acceleration
end