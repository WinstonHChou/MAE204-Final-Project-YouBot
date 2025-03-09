function q_k_1 = NextState(q_k, q_dot_k, dt, max_joint_vel)
% *** KINEMATICS Simulator ***
% Takes q_k: The current state of the robot
%            (12 variables: 3 for chassis, 5 for arm, 4 for wheel angles)
%       q_dot_k: The joint and wheel velocities 
%            (9 variables: 5 for arm  ̇θ, 4 for wheels u)
%       dt: The timestep size dt (1 parameter)
%       max_joint_vel: The maximum joint and wheel velocity magnitude (1 parameter)
% 
% Returns q_k_1: The next state (configuration) of the robot (12 variables)
% 
% The function NextState is based on a simple first-order Euler step:
% - new arm joint angles = (old arm joint angles) + (joint speeds)*dt
% - new wheel angles = (old wheel angles) + (wheel speeds)*dt
% - new chassis configuration is obtained from odometry, as described in Chapter 13.4  


end