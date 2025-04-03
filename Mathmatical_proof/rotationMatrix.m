% filepath: c:\Users\conno\Documents\GitHub\esp32-drone-control-sytem\Mathmatical_proof\rotationMatrix.m
function R = rotationMatrix(phi, theta, psi)
    % Create rotation matrix from Euler angles (ZYX convention)
    R_x = [1, 0, 0; 0, cos(phi), -sin(phi); 0, sin(phi), cos(phi)];
    R_y = [cos(theta), 0, sin(theta); 0, 1, 0; -sin(theta), 0, cos(theta)];
    R_z = [cos(psi), -sin(psi), 0; sin(psi), cos(psi), 0; 0, 0, 1];
    R = R_z * R_y * R_x;
end