function [traj] = TrajectoryGenerator(T_se_initial, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff, k)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% delete old csv file 
delete('trajectory.csv')

N = k / 0.01;

T_se_standoff_intial = T_sc_initial * T_ce_standoff;

T_se_standoff_final = T_sc_final * T_ce_standoff;
T_se_standoff_final(3,4) = 0.15;

T_se_grasp_final = T_se_standoff_final;
T_se_grasp_final(3,4) = 0.05;

% Starting Position  --> Standoff over Cube
traj1 = ScrewTrajectory(T_se_initial, T_se_standoff_intial, 3, N, 5);
trajectoryToCsv(traj1, 0)

% Standoff over Cube --> To End-Effector Grasp 
traj2 = ScrewTrajectory(T_se_standoff_intial, T_ce_grasp, 1, N, 5);
trajectoryToCsv(traj2, 0)

% End-Effector Grasp --> End-Effector Grasp 0.625
traj3 = ScrewTrajectory(T_ce_grasp, T_ce_grasp, 1, N, 5);
trajectoryToCsv(traj3, 1)

% End-Effector Grasp --> Standoff over Cube
traj4 = ScrewTrajectory(T_ce_grasp, T_se_standoff_intial, 2, N, 5);
trajectoryToCsv(traj4, 1)

% Standoff over Cube Initial --> Standoff over Cube Final
traj5 = ScrewTrajectory(T_se_standoff_intial, T_se_standoff_final, 3, N, 5);
trajectoryToCsv(traj5, 1)

% Standoff over Cube Final --> End-Effector Grasp 
traj6 = ScrewTrajectory(T_se_standoff_final, T_se_grasp_final, 2, N, 5);
trajectoryToCsv(traj6, 1)

% End-Effector Grasp --> End-Effector Grasp 0.625
traj7 = ScrewTrajectory(T_se_grasp_final, T_se_grasp_final, 1, N, 5);
trajectoryToCsv(traj7, 0)

% End-Effector Grasp --> End-Effector Standoff
traj8 = ScrewTrajectory(T_se_grasp_final, T_se_standoff_final, 1, N, 5);
trajectoryToCsv(traj8, 0)

traj = [traj1, traj2, traj3, traj4, traj5, traj6, traj7, traj8];

end

function trajectoryToCsv(traj, gripperState)
    
    for ii = 1:length(traj)
        [R, p] = TransToRp( traj{1,ii} );
        segmentCsv = [reshape(R.',1,[]), p', gripperState];
        writematrix(segmentCsv, 'trajectory.csv', 'WriteMode','append')
    end 

end