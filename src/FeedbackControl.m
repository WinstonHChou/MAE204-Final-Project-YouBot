function [endEffectorTwist_e, wheelSpeeds, jointSpeeds] = FeedbackControl(q, T_se_d, T_se_d_next, Kp, Ki, dt)
%FEEDBACKCONTROL Summary of this function goes here
%   Inputs: 
%   - q           : The current state vector (12 variables)
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

    % Running error 
    persistent errorControl
    load("youBotParams.mat")
    
    %% Base Body Jacobian
    % Compute Current T_sb (chassis (Body) frame to odom (Space) frame)
    T_sb = [[cos(q(1)), -sin(q(1)), 0,  q(2)];
            [sin(q(1)),  cos(q(1)), 0,  q(3)];
            [       0,           0, 1,    z0];
            [       0,           0, 0,     1]];
    % Compute Current T_0e (E-E frame to Arm Base frame)
    T_0e = FKinBody(M_0e, B, q(4:8));
    % Compute Current T_se (E-E frame to odom (Space) frame)
    T_se = T_sb * T_b0 * T_0e;
    % Compute Chassis Velocity Kinematics in 6D
    F6 = [zeros(1,4);
          zeros(1,4);
                   F;
          zeros(1,4)];
    % Compute Jbase relative to E-E frame
    Jbase = Adjoint(TransInv(T_0e) * TransInv(T_b0)) * F6;

    %% Arm Body Jacobian 
    % Check if joints are within arbitrary limits
    [empty, joints_checked] = checkJointLimits(q);
    % Compute Jarm based on given state vector, q
    Jarm = JacobianBody(B, q(4:8));
    % Ignore Jacobian of the joints who reaches the joint limits
    Jarm(:, ~joints_checked) = 0;
    
    %% youBot Body Jacobain
    Jb = [Jarm Jbase]; % order: q_dot = [theta_dot; wheel_speeds]
    % disp(Jb)

    %% Control
    % Xerr 
    Xerr = log(inv(T_se)*T_se_d);
    
    % Feedforward reference twist
    refTwist = (1/dt)*log(inv(T_se_d) * T_se_d_next);
    
    % Feedforward plus feedback control law (equation 13.37)
    % twist = 
    
    % Accumulate error every timestep
    errorControl = Xerr * dt;

end

