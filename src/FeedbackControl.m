function [endEffectorTwist_e, wheelSpeeds, jointSpeeds, dV_error] = FeedbackControl(q, T_se_d, T_se_d_next, Kp, Ki, dt)
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
%   - dV_error           : Current error twist

    % Running error 
    persistent prev_I;

    if isempty(prev_I)
        prev_I = zeros(6,1);
    end

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
    
    %% youBot Body Jacobian
    Jb = [Jbase Jarm];

    %% Control Law
    % Current error twist
    X_err = T_se \ T_se_d;
    dV_error = se3ToVec(MatrixLog6(T_se \ T_se_d));

    % Current reference twist
    X_ref = T_se_d \ T_se_d_next;
    Vd = (1/dt) * MatrixLog6(X_ref);
    Vd = se3ToVec(Vd);
    
    %% Feedback Control (PI Controller)
    % Closed-loop control based on transformation error
    % Proportional term
    P = Kp * dV_error;
    % Integral term
    I = prev_I + Ki * dV_error * dt;
    prev_I = I;

    %% FeedForward & Feedback Controller
    V = Adjoint(X_err) * Vd + P + I;
    
    % Given pinv tolerance to avoid sigularities
    q_dot = pinv(Jb, 1e-4) * V;
    
    endEffectorTwist_e = V;
    wheelSpeeds = q_dot(1:4);
    jointSpeeds = q_dot(5:9);

end

