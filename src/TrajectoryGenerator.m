function [traj, gripperStates] = TrajectoryGenerator(T_se_initial, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff, k)
% Trajectory Genrator
%   Inputs: 
%       - T_se_initial:  This is the initial configuration of the
%                        end-effector
%       - T_sc_initial:  The initial configuration of the cube
%       - T_sc_final:    The final configuration of the cube
%       - T_ce_grasp:    The configuration of the end-effector relative to
%                        the cube, while grasping
%       - T_ce_standoff: The standoff configuration of the end-effector
%                        above the cube, before and after grasping 
%       - k:             The number of trajectory reference configurations
%                        per 0.01 seconds. 
%   Outputs:
%       - traj         : A representation of the N configurations of the
%                        end-effector along the entire concatenated eight-segment
%                        reference trajectory. 
%       - gripperStates: Gripper states corresponds to each waypoint
    
    addpath('external/ModernRobotics/packages/MATLAB/mr')
    
    N = k / 0.01;
    
    T_se_standoff_intial = T_sc_initial * T_ce_standoff;
    T_se_grasp_initial = T_sc_initial * T_ce_grasp;
    
    T_se_standoff_final = T_sc_final * T_ce_standoff;    
    T_se_grasp_final = T_sc_final * T_ce_grasp;

    %% Initialize trajectories
    % Starting Position  --> Standoff over Cube
    % Standoff over Cube --> To End-Effector Grasp
    % End-Effector Grasp --> End-Effector Grasp 0.625
    % End-Effector Grasp --> Standoff over Cube
    % Standoff over Cube Initial --> Standoff over Cube Final
    % Standoff over Cube Final --> End-Effector Grasp
    % End-Effector Grasp --> End-Effector Grasp 0.625
    % End-Effector Grasp --> End-Effector Standoff
    
    X = {T_se_initial, T_se_standoff_intial, T_se_grasp_initial, T_se_grasp_initial, T_se_standoff_intial, T_se_standoff_final, T_se_grasp_final, T_se_grasp_final, T_se_standoff_final};
    Xstarts = X(1:end-1);
    Xends = X(2:end);
    desired_durations = [3, 1, 1, 2, 3, 2, 1, 1];
    grasp_states = [0, 0, 1, 1, 1, 1, 0, 0];

    % Initialize empty outputs
    n = floor(desired_durations*N);
    traj = cell(1,sum(n));
    gripperStates = zeros(1,sum(n));

    prev_sum_n = 0;
    for i = [1:length(Xstarts)]
        indices = 1+prev_sum_n : prev_sum_n+n(i);
        prev_sum_n = prev_sum_n + n(i);
        traj(1,indices) = ScrewTrajectory(Xstarts{i}, Xends{i}, desired_durations(i), n(i), 5);
        gripperStates(1,indices) = grasp_states(i);
    end
end