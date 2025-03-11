clear; close all; clc;

q_dot = [pi/4 pi/4 pi/4 pi/4 pi/4 -1 1 1 -1]';

dt = 0.01;
max_joint_vel = 200;

t = [0:dt:1];
q = zeros(12,length(t));
for i = 2:length(t)
    q(:, i) = NextState(q(:, i-1), q_dot, dt, max_joint_vel);
end
disp(q')

writematrix(q','testData_NextState.csv')