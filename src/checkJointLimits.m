function [q_checked, joints_in_bound] = checkJointLimits(q)
% Takes q: The current state of the robot
%          (12 variables: 3 for chassis, 5 for arm, 4 for wheel angles)
% 
% Returns q_checked: The constrained state of the robot (12 variables)
%         joints_in_bound: if arm joints are in bound (1 array)

    % Arm Joints
    q_arm = q(4:8);
    
    % Joint Lower Bounds and Upper Bounds
    q_lowerBounds = [-inf, -3*pi/4, -5*pi/6, -5*pi/6, -inf]';
    q_upperBounds = [inf, pi/2, 5*pi/6, 5*pi/6, inf]';
    
    [q_arm, joints_in_bound] = bound(q_arm, q_lowerBounds, q_upperBounds);
    q_checked = [q(1:3); q_arm; q(9:12)];

end