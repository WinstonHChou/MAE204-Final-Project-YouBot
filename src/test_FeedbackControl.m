clc; clear; close all;

addpath('external/ModernRobotics/packages/MATLAB/mr')
%% Test inputs provided in final project instructions 
load("youBotParams.mat")

% Robot Configuration
robotConfig = [0, 0, 0, 0, 0, 0.2, -1.6, 0];
q = [robotConfig, 0, 0, 0, 0]'; % wheels pos doesn't matter

% Computed from state vector, q given above
T_sb = [[cos(q(1)), -sin(q(1)), 0,  q(2)];
        [sin(q(1)),  cos(q(1)), 0,  q(3)];
        [       0,           0, 1,    z0];
        [       0,           0, 0,     1]];
X_0e = FKinBody(M_0e, B, robotConfig(4:8)');
X_se = T_sb * T_b0 * X_0e;

% Verify X_se matches X
X = [[ 0.170, 0, 0.985, 0.387];
     [     0, 1,     0,     0];
     [-0.985, 0, 0.170, 0.570];
     [     0, 0,     0,     1]];
fprintf("Errors between X and X_se: \n%f\n", sum(sum(X_se - X)))

% Xd 
Xd = [[ 0, 0, 1, 0.5];
      [ 0, 1, 0,   0]; 
      [-1, 0, 0, 0.5];
      [ 0, 0, 0,   1]];

% Xd,next
Xd_next = [[ 0, 0, 1, 0.6];
           [ 0, 1, 0,   0]; 
           [-1, 0, 0, 0.3];
           [ 0, 0, 0,   1]];

% dt 
dt = 0.01;

% Kp and Ki 
Kp = zeros(4);
Ki = zeros(4);

[endEffectorTwist_e, wheelSpeeds, jointSpeeds] = FeedbackControl(q, Xd, Xd_next, Kp, Ki, dt)