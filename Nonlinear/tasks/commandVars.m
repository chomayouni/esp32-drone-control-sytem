%% Command Vars
% Copyright 2017-2023 The MathWorks, Inc.

% Yaw Command
Command.yawStepAmplitude = 0;
Command.yawStepTime = 0;
Command.yawStepDuration = 0;

% Pitch command
Command.pitchStepAmplitude = 0;
Command.pitchStepTime = 0;
Command.pitchStepDuration = 0;

% Roll Command
Command.rollStepAmplitude = 0;
Command.rollStepTime = 0; 
Command.rollStepDuration = 0;

% Altitude Command
Command.takeoffDuration = 0;
Command.altitude = 0;

% Joystick
Command.rollDeadZoneEnd = 0.2;
Command.rollDeadZoneStart = -0.2;
Command.rollSatUpper = 1;
Command.rollSatLower = -1;
Command.rollGain = 1;
Command.pitchDeadZoneEnd = 0.2;
Command.pitchDeadZoneStart = -0.2;
Command.pitchSatUpper = 5;
Command.pitchSatLower = -5;
Command.pitchGain = 6;
Command.yawDeadZoneEnd = 0.2;
Command.yawDeadZoneStart = -0.2;
Command.yawSatUpper = .5;
Command.yawSatLower = -.5;
Command.yawGain = .25;
Command.throttleGain = -5;