% Number of trajectory reference configurations per 0.01 seconds: k
k = 1;

% Define Inputs
T_se_initial = eye(4);
T_se_initial(3,4) = 0.5;

T_sc_initial = [[1, 0, 0, 1]; [0, 1, 0, 0]; [0, 0, 1, 0.05]; [0, 0, 0, 1]];

T_sc_final = [[0, 1, 0, 0]; [-1, 0, 0, -1]; [0, 0, 1, 0.05]; [0, 0, 0, 1]];

T_ce_standoff = [[0, 0, 1, 0]; [0, 1, 0, 0]; [-1, 0, 0, 0.20]; [0, 0, 0, 1]];

T_ce_grasp = [[0, 0, 1, 1]; [0, 1, 0, 0]; [-1, 0, 0, 0.05]; [0, 0, 0, 1]];

[traj, gripperStates] = TrajectoryGenerator(T_se_initial, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff, k);

Csv = zeros(length(traj), 13);
for ii = 1:length(traj)
    [R, p] = TransToRp( traj{1,ii} );
    Csv(ii,:) = [reshape(R.',1,[]), p', gripperStates(ii)];
end 
writematrix(Csv, 'testData_TrajectoryGenerator.csv')