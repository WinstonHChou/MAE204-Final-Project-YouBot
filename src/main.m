%% Final Project
% MAE204
% Author: Winston Chou, Alexander Ivanov

clc; clear; close all;

addpath('external/ModernRobotics/packages/MATLAB/mr')
warning('off','all');
%% Parameters Setup
load("youBotParams.mat")

% Maximum Joint Velocity
max_joint_vel = 50;

% List of tasks
tasks = ["feedforward","best","overshoot","new task"];
task = tasks(4);

switch task
    case "new task"
        % Given the initial states, q_0
        q_0 = [0, 0, 0, pi/6, -pi/6, pi/3, -pi/2, pi/2, 0, 0, 0, 0]';

        % Define the initial configuration of the end-effector reference trajectory
        T_se_initial = eye(4);
        T_se_initial(3,4) = 0.5;

        %Temp
        q_0(4:8) = IKinBody(B, M_0e, T_se_initial, q_0(4:8), 1e-6, 1e-6)

        % Define the initial and final configurations of the cube
        T_sc_initial = [[1, 0, 0,    1];
                        [0, 1, 0,    0];
                        [0, 0, 1, 0.025];
                        [0, 0, 0,    1]];
        T_sc_final = [[ 0, 1, 0,    0];
                      [-1, 0, 0,   -1];
                      [ 0, 0, 1, 0.025];
                      [ 0, 0, 0,    1]];

        Kp = zeros(6,6);
        Ki = zeros(6,6);
        Kp = eye(6);

        FF_enabled = true;

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

% Compute Current T_se (E-E frame to odom (Space) frame)
T_sb_0 = [[cos(q_0(1)), -sin(q_0(1)), 0,  q_0(2)];
          [sin(q_0(1)),  cos(q_0(1)), 0,  q_0(3)];
          [          0,            0, 1,      z0];
          [          0,            0, 0,       1]];
T_0e_0 = FKinBody(M_0e, B, q_0(4:8));
T_se_0 = T_sb_0 * T_b0 * T_0e_0;

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
N = k / 0.01;
[traj, gripperStates] = TrajectoryGenerator(T_se_initial, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff, k);
dt = (length(traj)/N)/(length(traj) - 1);
t = 0:dt:length(traj)/N;

%% Simulations
% Initialize Actual Poses, Reference Poses, Error Twists
X = cell(1,length(traj));
X{1,1} = T_se_0;
Xd = traj;
dV_errors = zeros(6, length(traj));

% Initialize state vectors with gripper state
q_grip = zeros(13, length(traj));
[q_0_checked, joints_checked] = checkJointLimits(q_0);
if ~all(joints_checked)
    disp("Initial State Compromised! Not within the Joint Limits!");
    disp("Original Initial Arm Joint State (rad):");
    disp(q_0(4:8));
    disp("Constrained Initial Arm Joint State (rad):");
    disp(q_0_checked(4:8))
end
q_grip(1:12, 1) = q_0_checked;
q_grip(13, 1) = gripperStates(1);
joints_checked = true(6, 1);

% Main Simulation Loop
integral_reset = false;
for i = 1:(length(traj)-1)
    % True: move on to next step, False: repeat the step
    joints_good = false;
    fprintf("[Iteration #%d]\n", i)
    while ~joints_good
        [V, wheelSpeeds, jointSpeeds, dV_errors(:, i)] = FeedbackControl(q_grip(1:12, i), Xd{i}, Xd{i+1}, Kp, Ki, dt, FF_enabled, integral_reset);
        [q_grip(1:12, i+1), joints_checked] = NextState(q_grip(1:12, i), [jointSpeeds; wheelSpeeds], dt, max_joint_vel);
        q_grip(13, i+1) = gripperStates(i+1);
        joints_good = all(joints_checked);
        fprintf("Joints checked: %f %f %f %f %f\n", joints_checked);
        joints_good = true;
    end

    % Update Actual Pose at i+1
    T_sb = [[cos(q_grip(1,i+1)), -sin(q_grip(1,i+1)), 0,  q_grip(2,i+1)];
              [sin(q_grip(1,i+1)),  cos(q_grip(1,i+1)), 0,  q_grip(3,i+1)];
              [                 0,                   0, 1,             z0];
              [                 0,                   0, 0,              1]];
    T_0e = FKinBody(M_0e, B, q_grip(4:8,i+1));
    T_se = T_sb * T_b0 * T_0e;
    X{1, i+1} = T_se;
end

%% Output array to waypoint_array.csv
% waypoint_array.csv will be located in Matlab's current directory
writematrix(q_grip','state_array.csv')