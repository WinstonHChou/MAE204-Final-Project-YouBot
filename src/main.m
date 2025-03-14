%% Final Project
% MAE204
% Author: Winston Chou, Alexander Ivanov

clc; clear; close all;

addpath('external/ModernRobotics/packages/MATLAB/mr')
warning('off','all');
%% Parameters Setup
% load youBot Parameters
load("youBotParams.mat")

% Reset Integral Term Memory
FeedbackControl(zeros(12,1), zeros(4,4), zeros(4,4), zeros(6,6), zeros(6,6), 0.1, false, true);

% List of tasks
tasks = ["best","overshoot","new_task","feedforward","speed_limit"];
task = tasks(5); % USER INPUT: 1="best", 2="overshoot", 3="new_task", 4="feedforward", 5="speed_limit"

% If choose "new_task", set custom initial and goal poses 
% for the block/cube ([x, y, theta] in world frame {s})
cube_initial = [1, 1, 0];
cube_final = [1, -1, -pi/3];

% Maximum Joint Velocity
max_joint_vel = 40;

%% Tasks
switch task
    case "new_task"
        % Given the initial states, q_0
        q_0 = [0, 0, 0, pi/6, -pi/3, pi/6, -pi/3, pi/2, 0, 0, 0, 0]';

        % Define the initial configuration of the end-effector reference trajectory
        T_se_initial = [[ 0, 0, 1,   0];
                        [ 0, 1, 0,   0];
                        [-1, 0, 0, 0.5];
                        [ 0, 0, 0,   1]];

        % Define the initial and final configurations of the cube
        T_sc_initial = [[cos(cube_initial(3)), -sin(cube_initial(3)), 0,  cube_initial(1)];
                        [sin(cube_initial(3)),  cos(cube_initial(3)), 0,  cube_initial(2)];
                        [                   0,                     0, 1,            0.025];
                        [                   0,                     0, 0,                1]];
        T_sc_final = [[cos(cube_final(3)), -sin(cube_final(3)), 0,  cube_final(1)];
                      [sin(cube_final(3)),  cos(cube_final(3)), 0,  cube_final(2)];
                      [                 0,                   0, 1,          0.025];
                      [                 0,                   0, 0,              1]];

        Kp = eye(6);
        Ki = zeros(6,6);
        FF_enabled = true;

    otherwise
        % Given the initial states, q_0
        q_0 = [0, 0, 0, pi/6, -pi/3, pi/6, -pi/3, pi/2, 0, 0, 0, 0]';

        % Define the initial configuration of the end-effector reference trajectory
        T_se_initial = [[ 0, 0, 1,   0];
                        [ 0, 1, 0,   0];
                        [-1, 0, 0, 0.5];
                        [ 0, 0, 0,   1]];

        % Define the initial and final configurations of the cube
        T_sc_initial = [[1, 0, 0,    1];
                        [0, 1, 0,    0];
                        [0, 0, 1, 0.025];
                        [0, 0, 0,    1]];
        T_sc_final = [[ 0, 1, 0,    0];
                      [-1, 0, 0,   -1];
                      [ 0, 0, 1, 0.025];
                      [ 0, 0, 0,    1]];

        switch task
            case "best"
                Kp = eye(6)*3;
                Ki = zeros(6,6);
                FF_enabled = true;
            case "overshoot"
                Kp = eye(6)*5;
                Ki = eye(6)*0.5;
                FF_enabled = false;
            case "speed_limit"
                % Overide speed limit to see how it'd affect the
                % performance
                max_joint_vel = 5;
                Kp = eye(6)*3;
                Ki = zeros(6,6);
                FF_enabled = true;
            otherwise
                % Force the actual initial state close to the reference
                % initial waypoint (Fine Tuned, Do Not Change)
                q_0(2) = -0.478;
                q_0(4:8) = [0, 0.075, -0.642, -1.025, 0]';
                Kp = zeros(6,6);
                Ki = zeros(6,6);
                FF_enabled = true;
        end
end

% Compute Current T_se (E-E frame to odom (Space) frame)
T_sb_0 = [[cos(q_0(1)), -sin(q_0(1)), 0,  q_0(2)];
          [sin(q_0(1)),  cos(q_0(1)), 0,  q_0(3)];
          [          0,            0, 1,      z0];
          [          0,            0, 0,       1]];
T_0e_0 = FKinBody(M_0e, B, q_0(4:8));
T_se_0 = T_sb_0 * T_b0 * T_0e_0;

% Define transformations for standoff and grasp configurations
pitchOffset = pi/6;
rotY = [[ cos(pitchOffset), 0, sin(pitchOffset), 0];
        [                0, 1,                0, 0];
        [-sin(pitchOffset), 0, cos(pitchOffset), 0];
        [                0, 0,                0, 1]];
T_ce_standoff = [[ 0, 0, 1,    0];
                 [ 0, 1, 0,    0];
                 [-1, 0, 0, 0.20];
                 [ 0, 0, 0,    1]] * rotY;
T_ce_grasp = [[ 0, 0, 1, 0];
              [ 0, 1, 0, 0];
              [-1, 0, 0, 0];
              [ 0, 0, 0, 1]] * rotY;

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
Jb = cell(1,length(traj)-1);

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

% Simulation Loop
fig = uifigure;
dlg = uiprogressdlg(fig,'Title',sprintf('Running Simulation Loop: CASE="%s"', task));

integral_reset = false;
for i = 1:(length(traj)-1)
    [V, wheelSpeeds, jointSpeeds, dV_errors(:, i), Jb{:, i}] = FeedbackControl(q_grip(1:12, i), Xd{i}, Xd{i+1}, Kp, Ki, dt, FF_enabled, integral_reset);
    [q_grip(1:12, i+1), joints_checked] = NextState(q_grip(1:12, i), [jointSpeeds; wheelSpeeds], dt, max_joint_vel);
    q_grip(13, i+1) = gripperStates(i+1);

    % Update Actual Pose at i+1
    T_sb = [[cos(q_grip(1,i+1)), -sin(q_grip(1,i+1)), 0,  q_grip(2,i+1)];
            [sin(q_grip(1,i+1)),  cos(q_grip(1,i+1)), 0,  q_grip(3,i+1)];
            [                 0,                   0, 1,             z0];
            [                 0,                   0, 0,              1]];
    T_0e = FKinBody(M_0e, B, q_grip(4:8,i+1));
    T_se = T_sb * T_b0 * T_0e;
    X{1, i+1} = T_se;

     dlg.Value = i/(length(traj)-1);
     dlg.Message = sprintf("Iteration #%d / %d:\n\nJoints Checked: [%d, %d, %d, %d, %d]\n", i, length(traj)-1, joints_checked);
end

close(dlg);
delete(fig);

%% Plot
% Plot Twist Errors Along Time
figure(1)
hold on
plot(t,dV_errors(1,:))
plot(t,dV_errors(2,:))
plot(t,dV_errors(3,:))
plot(t,dV_errors(4,:))
plot(t,dV_errors(5,:))
plot(t,dV_errors(6,:))
xlabel("Time (sec)")
ylabel("Twist Error (rad/s, m/s)")
title("Trajectory Twist Errors")
legend('wx' , 'wy', 'wz', 'vx', 'vy', 'vz')

saveas(gcf, strcat('../png/',task,'_twist_error.png'));

% Post-Analysis on Manipulability
mu1_w = zeros(length(traj)-1);
mu1_v = zeros(length(traj)-1);
for i=1:(length(traj)-1)
    Jb_i = Jb{:,i};
    Jw = Jb_i(1:3,:);      Jv = Jb_i(4:end,:);
    Aw = Jw*Jw';           Av = Jv*Jv';
    [v_Aw,D_Aw] = eig(Aw); [v_Av,D_Av] = eig(Av);
    lambda_Aw = max(D_Aw); lambda_Av = max(D_Av);
    mu1_w(i) = sqrt(max(lambda_Aw) / min(lambda_Aw));
    mu1_v(i) = sqrt(max(lambda_Av) / min(lambda_Av));
end

% Plot Manipulabilities Along Time
figure(2)
hold on
plot(t(2:end), mu1_w)
plot(t(2:end), mu1_v)
xlabel("Time (sec)")
ylabel("Manipulabilities")
title("Trajectory Manipulability")
legend('Angular \mu_1' , 'Linear \mu_1', 'Location', 'best')

saveas(gcf, strcat('../png/',task,'_manipulability.png'));

%% Output array to CSV
% The csv files will be located in Matlab's current directory
writematrix(q_grip', strcat(task,'_state_array.csv'))