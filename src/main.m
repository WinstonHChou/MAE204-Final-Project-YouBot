%% Final Project
% MAE204
% Author: Winston Chou, Alexander Ivanov

clc; clear; close all;

addpath('external/ModernRobotics/packages/MATLAB/mr')
%% Parameters Setup
load("youBotParams.mat")
tasks = ["feedforward","best","overshoot","new task"];
task = tasks(1);

switch task
    case "new task"
        % Given the initial states, q_0
        q_0 = [0, 0, 0, pi/6, -pi/6, pi/3, -pi/2, pi/2, 0, 0, 0, 0]';

        % Define the initial configuration of the end-effector reference trajectory
        T_se_initial = eye(4);
        T_se_initial(3,4) = 0.5;

        % Define the initial and final configurations of the cube
        T_sc_initial = [[1, 0, 0,    1];
                        [0, 1, 0,    0];
                        [0, 0, 1, 0.025];
                        [0, 0, 0,    1]];
        T_sc_final = [[ 0, 1, 0,    0];
                      [-1, 0, 0,   -1];
                      [ 0, 0, 1, 0.025];
                      [ 0, 0, 0,    1]];

    otherwise
        % Given the initial states, q_0
        q_0 = [0, 0, 0, pi/6, -pi/6, pi/3, -pi/2, pi/2, 0, 0, 0, 0]';

        % Define the initial configuration of the end-effector reference trajectory
        T_se_initial = eye(4);
        T_se_initial(3,4) = 0.5;

        % Define the initial and final configurations of the cube
        T_sc_initial = [[1, 0, 0,    1];
                        [0, 1, 0,    0];
                        [0, 0, 1, 0.025];
                        [0, 0, 0,    1]];
        T_sc_final = [[ 0, 1, 0,    0];
                      [-1, 0, 0,   -1];
                      [ 0, 0, 1, 0.025];
                      [ 0, 0, 0,    1]];
end

% (Temp) Compute Current T_se (E-E frame to odom (Space) frame)
T_sb_0 = [[cos(q_0(1)), -sin(q_0(1)), 0,  q_0(2)];
          [sin(q_0(1)),  cos(q_0(1)), 0,  q_0(3)];
          [          0,            0, 1,      z0];
          [          0,            0, 0,       1]];
T_0e_0 = FKinBody(M_0e, B, q_0(4:8));
T_se_0 = T_sb_0 * T_b0 * T_0e_0;
disp(T_se_0)

% Define transformations for standoff and grasp configurations
T_ce_standoff = [[ 0, 0, 1,    0];
                 [ 0, 1, 0,    0];
                 [-1, 0, 0, 0.20];
                 [ 0, 0, 0,    1]];
T_ce_grasp = [[ 0, 0, 1,    1];
              [ 0, 1, 0,    0];
              [-1, 0, 0, 0.05];
              [ 0, 0, 0,    1]];

%% Trajectories

% Number of trajectory reference configurations per 0.01 seconds: k
k = 1;
[traj] = TrajectoryGenerator(T_se_initial, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff, k)

%% Output array to waypoint_array.csv
% waypoint_array.csv will be located in Matlab's current directory
% writematrix(waypoint_array,'waypoint_array.csv')