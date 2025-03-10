%% Parameters
% lengths saved in meters
x_0e = 0.033;
x_b0 = 0.1662;

% chassis frame {b} is at a height of z0
z0 = 0.0963;
% youBot dimensions
z1 = 0.0026;
z2 = 0.147;
z3 = 0.155;
z4 = 0.135;
z5 = 0.2176;

% front-back distance between the wheels: 2*l
l = 0.47 / 2;
% side-to-side distance between the wheels: 2*w
w = 0.3 / 2;
% radius of the wheels: r
r = 0.0475;

%% Body Frame Screw Axes
B1 = [0,  0, 1,           0, x_0e, 0]';
B2 = [0, -1, 0, -(z3+z4+z5),    0, 0]';
B3 = [0, -1, 0,    -(z4+z5),    0, 0]';
B4 = [0, -1, 0,         -z5,    0, 0]';
B5 = [0,  0, 1,           0,    0, 0]';

B = [B1, B2, B3, B4, B5];

%% Zero E-E Config
% end-effector frame {e} to arm base frame {0}
M_0e = [[1, 0, 0,        x_0e];
        [0, 1, 0,           0];
        [0, 0, 1, z2+z3+z4+z5];
        [0, 0, 0,           1]];

%% youBot Configs
% arm base frame {0} to chassis frame {b} (static transform)
T_b0 = [[1, 0, 0, x_b0];
        [0, 1, 0,    0];
        [0, 0, 1,   z1];
        [0, 0, 0,    1]];
% Velocity Forward Kinematics of Chassis
F = (r/4) .* [[-1/(l + w), 1/(l + w), 1/(l + w), -1/(l + w)];
              [         1,         1,         1,          1];
              [        -1,         1,        -1,          1]];

save youBotParams.mat z0 l w r B M_0e T_b0 F
