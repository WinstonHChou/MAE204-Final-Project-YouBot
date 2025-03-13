function [q_k_1, joints_checked] = NextState(q_k, q_dot_k, dt, max_joint_vel)
% *** KINEMATICS Simulator ***
% Takes q_k: The current state of the robot
%            (12 variables: 3 for chassis, 5 for arm, 4 for wheel angles)
%       q_dot_k: The joint and wheel velocities 
%            (9 variables: 5 for arm  ̇θ, 4 for wheels u)
%       dt: The timestep size dt (1 parameter)
%       max_joint_vel: The maximum joint and wheel velocity magnitude (1 parameter)
% 
% Returns q_k_1: The next state (configuration) of the robot (12 variables)
%         joints_checked: The joints within the arbitrary joint limits
% 
% The function NextState is based on a simple first-order Euler step:
% - new arm joint angles = (old arm joint angles) + (joint speeds)*dt
% - new wheel angles = (old wheel angles) + (wheel speeds)*dt
% - new chassis configuration is obtained from odometry, as described in Chapter 13.4  

    [q_dot_k, in_bound] = bound(q_dot_k, -max_joint_vel, max_joint_vel);

    q_k_1 = zeros(12,1);
    % Joints Update & Wheels Update
    q_k_1(4:8) = q_k(4:8) + q_dot_k(1:5)*dt;
    q_k_1(9:12) = q_k(9:12) + q_dot_k(6:9)*dt;
    
    % Odometry Update [phi, x, y]
    load("youBotParams.mat", 'F')
    wheel_speeds = q_dot_k(6:9);
    phi_k = q_k(1);

    V = F * wheel_speeds;
    dV_omg = V(1) * dt;
    dV_x = V(2) * dt;
    dV_y = V(3) * dt;
    % 2D pose exponential
    if dV_omg == 0
        dq_chassis_body = [0, dV_x, dV_y]';
    else
        dq_chassis_body = [dV_omg;
                           dV_x * sin(dV_omg)/dV_omg + dV_y * (cos(dV_omg) - 1)/dV_omg;
                           dV_x * (1 - cos(dV_omg))/dV_omg + dV_y * sin(dV_omg)/dV_omg];
    end
    rot_odom_body = [[1,          0,           0];
           [0, cos(phi_k), -sin(phi_k)];
           [0, sin(phi_k),  cos(phi_k)]];
    dq_chassis_odom = rot_odom_body * dq_chassis_body;
    q_k_1(1:3) = q_k(1:3) + dq_chassis_odom;

    [q_k_1, joints_checked] = checkJointLimits(q_k_1);
    % disp(joints_checked)

end