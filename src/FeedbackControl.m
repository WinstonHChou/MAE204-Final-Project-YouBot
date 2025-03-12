function [endEffectorTwist_e, wheelSpeeds, jointSpeeds] = FeedbackControl(T_se, T_se_d, T_se_d_next, Kp, Ki, dt)
%FEEDBACKCONTROL Summary of this function goes here
%   Inputs: 
%   - T_se        : The current actual end-effector configuration 
%   - T_se_d      : The current reference end-effector configuration 
%   - T_se_d_next : The reference end-effector configuration at the next timestep
%   - Kp          : P gain matrix
%   - Ki          : I gain matrix
%   - dt          : timestep size
%   Outputs: 
%   - endEffectorTwist_e : The commanded end-effector twist expressed in
%                          end-effector frame
%   - wheelSpeeds        : Commanded wheel speeds (u)
%   - jointSpeeds        : Commanded arm joint speeds (theta_dot)

% Body Jacobian 

% Arm Jacobian 

end

