clc; clear; close all;

%% Test inputs provided in final project instructions 

% Robot Configuration
robotConfig = [0, 0, 0, 0, 0, 0.2, -1.6, 0];

% Computed from robotConfig given above
X = [[0.170, 0.0, 0.985, 0.387];
     [0, 1, 0, 0];
     [-0.985, 0, 0.170, 0.570];
     [0, 0, 0, 1]];

% Xd 
Xd = [[0, 0, 1, 0.5] ;
      [0, 1, 0, 0]   ; 
      [-1, 0, 0, 0.5];
      [0, 0, 0, 1]];

% Xd,next
Xd_next = [[0, 0, 1, 0.6] ;
           [0, 1, 0, 0]   ; 
           [-1, 0, 0, 0.3];
           [0, 0, 0, 1]  ];

% dt 
dt = 0.01;

% Kp and Ki 
Kp = zeros(4);
Ki = zeros(4);


