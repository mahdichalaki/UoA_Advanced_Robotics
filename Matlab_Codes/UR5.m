%% UR5 Robot Modeling and Dynamics - Advanced Robotics Project
% University of Tehran
% This script defines the kinematic and dynamic properties of the UR5 robot.

clear; clc;

%% Denavit-Hartenberg (DH) Parameters and Inertia Properties

% DH parameters: alpha (twist), a (link length), d (link offset)
alpha = [0, pi/2, 0, 0, -pi/2, pi/2];
a = [0, 0, 0.425, 0.39225, 0, 0];
d = [0.08916, 0, 0, 0.10915, 0.09456, 0.0823];

% Mass of each link (kg)
m = [3.7, 8.393, 2.275, 1.219, 1.219, 0.1879];

% Center of mass positions (meters)
p_c = [
    [0, -1.93, -26.51] * 1e-3;
    [212.5, 0, 113.36] * 1e-3;
    [272.32, 0, 26.5] * 1e-3;
    [0, 16.34, 107.35] * 1e-3;
    [0, -16.34, -1.8] * 1e-3;
    [0, 0, -1.159] * 1e-3
];

% Inertia matrices (kg.m^2)
Inertia = cat(3, ...
    diag([84, 64, 84]) * 1e-4, ...
    diag([78, 21, 21]) * 1e-4, ...
    diag([16, 462, 462]) * 1e-4, ...
    diag([16, 16, 9]) * 1e-4, ...
    diag([16, 16, 9]) * 1e-4, ...
    eye(3) * 1e-4 ...
);

%% Define symbolic variables
syms g q [6 1] dq [6 1] real

%% Rotation Matrices Using DH Parameters
for i = 1:6
    R(:,:,i) = [
        cos(q(i)), -sin(q(i)), 0;
        sin(q(i)) * cos(alpha(i)), cos(q(i)) * cos(alpha(i)), -sin(alpha(i));
        sin(q(i)) * sin(alpha(i)), cos(q(i)) * sin(alpha(i)), cos(alpha(i))
    ];
end

%% Position Vectors (from base to each joint)
p = [
    a(1); -sin(alpha(1)) * d(1); cos(alpha(1)) * d(1);
    a(2); -sin(alpha(2)) * d(2); cos(alpha(2)) * d(2);
    a(3); -sin(alpha(3)) * d(3); cos(alpha(3)) * d(3);
    a(4); -sin(alpha(4)) * d(4); cos(alpha(4)) * d(4);
    a(5); -sin(alpha(5)) * d(5); cos(alpha(5)) * d(5);
    a(6); -sin(alpha(6)) * d(6); cos(alpha(6)) * d(6)
];

%% Transformation Matrices and Forward Kinematics
T = eye(4);
for i = 1:6
    T_i = [R(:,:,i), p(3*i-2:3*i); zeros(1,3), 1];
    T = T * T_i;
end

%% Compute Centers of Mass (COM) Positions
p_c_total = cell(1, 6);
for i = 1:6
    p_c_total{i} = simplify(p(3*i-2:3*i) + R(:,:,i) * p_c(i,:)');
end

%% Jacobian Computation
J_v = cell(1,6);
J_o = cell(1,6);

for i = 1:6
    J_v{i} = jacobian(p_c_total{i}, q);
    if i == 1
        J_o{i} = [R(:,:,1)(:,3), zeros(3,5)];
    else
        J_o{i} = [J_o{i-1}, R(:,:,i)(:,3), zeros(3,6-i)];
    end
end

% End-effector Jacobian
Jacobian = [J_v{6}; J_o{6}];

%% Compute Inertia (Mass) Matrix
M = zeros(6);
for i = 1:6
    M = M + J_v{i}' * m(i) * eye(3) * J_v{i} + J_o{i}' * R(:,:,i) * Inertia(:,:,i) * R(:,:,i)' * J_o{i};
end
M = simplify(M);

%% Compute Coriolis and Centrifugal Matrix
C = sym(zeros(6,6));
for k = 1:6
    for s = 1:6
        C(k,s) = 0.5 * (diff(M(k,s), q) + diff(M(k,:), q(s)) - diff(M(:,s), q(k))) * dq;
    end
end
C = simplify(C);

%% Compute Potential Energy and Gravity Vector
P = 0;
for i = 1:6
    P = P + m(i) * [0, 0, g] * p_c_total{i};
end
G = simplify(jacobian(P, q).');

%% Save Data to Files
save('UR5_Matrices.mat', 'M', 'C', 'G', 'T', 'Jacobian');

% Export matrices to text files
writeMatrixToFile('UR5_M.txt', M);
writeMatrixToFile('UR5_C.txt', C);
writeMatrixToFile('UR5_G.txt', G);
writeMatrixToFile('UR5_T.txt', T);
writeMatrixToFile('UR5_J.txt', Jacobian);

disp('UR5 Robot Model Data Saved Successfully.');

%% Function to Write Matrix to Text File
function writeMatrixToFile(filename, matrix)
    fid = fopen(filename, 'w');
    fprintf(fid, '%s\n', char(matrix));
    fclose(fid);
end
