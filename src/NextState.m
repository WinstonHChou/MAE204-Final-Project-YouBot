function x_k_1 = NextState(x_k, q_k, dt, max_joint_vel)
% *** KINEMATICS Simulator ***
% Takes x_k: The current state of the robot
%            (12 variables: 3 for chassis, 5 for arm, 4 for wheel angles)
%       q_k: The joint and wheel velocities 
%            (9 variables: 5 for arm  ̇θ, 4 for wheels u)
%       dt: The timestep size dt (1 parameter)
%       max_joint_vel: The maximum joint and wheel velocity magnitude (1 parameter)
% 
% Returns x_k_1: The next state (configuration) of the robot (12 variables)
% 
% The function NextState is based on a simple first-order Euler step:
% • new arm joint angles = (old arm joint angles) + (joint speeds)∆t
% • new wheel angles = (old wheel angles) + (wheel speeds)∆t
% • new chassis configuration is obtained from odometry, as described in Chapter 13.4  


end