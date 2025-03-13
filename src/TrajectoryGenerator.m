function [traj] = TrajectoryGenerator(T_se_initial, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff, k)
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
%       - traj : A representation of the N configurations of the
%                end-effector along the entire concatenated eight-segment
%                reference trajectory. 
    
    addpath('external/ModernRobotics/packages/MATLAB/mr')
    
    % delete old csv file 
    delete('trajectory.csv')
    
    N = k / 0.01;
    
    T_se_standoff_intial = T_sc_initial * T_ce_standoff;
    
    T_se_standoff_final = T_sc_final * T_ce_standoff;
    T_se_standoff_final(3,4) = 0.15;
    
    T_se_grasp_final = T_se_standoff_final;
    T_se_grasp_final(3,4) = 0.05;

    %% Initialize trajectories
    % Starting Position  --> Standoff over Cube
    % Standoff over Cube --> To End-Effector Grasp
    % End-Effector Grasp --> End-Effector Grasp 0.625
    % End-Effector Grasp --> Standoff over Cube
    % Standoff over Cube Initial --> Standoff over Cube Final
    % Standoff over Cube Final --> End-Effector Grasp
    % End-Effector Grasp --> End-Effector Grasp 0.625
    % End-Effector Grasp --> End-Effector Standoff
    
    traj = cell(1,8*N);
    X = {T_se_initial, T_se_standoff_intial, T_ce_grasp, T_ce_grasp, T_se_standoff_intial, T_se_standoff_final, T_se_grasp_final, T_se_grasp_final, T_se_standoff_final};
    Xstarts = X(1:end-1);
    Xends = X(2:end);
    desired_durations = [3, 1, 1, 2, 3, 2, 1, 1];
    grasp_states = [0, 0, 1, 1, 1, 1, 0, 0];
    
    for i = [1:length(X)-1]
        traj(1,1+N*(i-1):N*i) = ScrewTrajectory(Xstarts{i}, Xends{i}, desired_durations(i), N, 5);
        trajectoryToCsv(traj(1,1+N*(i-1):N*i), grasp_states(i))
    end
end

function trajectoryToCsv(traj, gripperState)
    
    for ii = 1:length(traj)
        [R, p] = TransToRp( traj{1,ii} );
        segmentCsv = [reshape(R.',1,[]), p', gripperState];
        writematrix(segmentCsv, 'trajectory.csv', 'WriteMode','append')
    end 

end