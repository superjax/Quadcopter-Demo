global P

% Define lengths 
P.fuse_h = .5;
P.tail_h = .5;
P.fuse_l1 = .75;
P.fuse_l2 = .5;
P.fuse_l3 = 1.5;
P.wing_l = .75;
P.tw_l = .5;
P.fuse_w = .5;
P.wing_w = 2;
P.tw_w = 1;
    

% Define Vertices
P.V = [ ...
    %Points 1-6 assumes the fuselage is completely centered along body axis
    P.fuse_l1,          0,              0; ...            % P1
    P.fuse_l2,          P.fuse_w/2,     -P.fuse_h/2; ...  % P2 
    P.fuse_l2,          -P.fuse_w/2,    -P.fuse_h/2; ...  % P3
    P.fuse_l2,          -P.fuse_w/2,    P.fuse_h/2; ...   % P4
    
    P.fuse_l2,          P.fuse_w/2,     P.fuse_h/2; ...   % P5
    -P.fuse_l3,         0,              0; ...            % P6
    0,                  P.wing_w/2,     0; ...            % P7
    -P.wing_l,          P.wing_w/2,     0; ...            % P8
    
    -P.wing_l,          -P.wing_w/2,    0; ...            % P9
    0,                  -P.wing_w/2,    0; ...            % P10    
    P.tw_l-P.fuse_l3,   P.tw_w/2,       0; ...            % P11
    -P.fuse_l3,         P.tw_w/2,       0; ...            % P12
    
    -P.fuse_l3,         -P.tw_w/2,      0; ...            % P13
    P.tw_l-P.fuse_l3,   -P.tw_w/2,      0; ...            % P14
    P.tw_l-P.fuse_l3,   0,              0; ...            % P15
    -P.fuse_l3,         0,              -P.tail_h; ...    % P16
    ];
P.V = P.scale*P.V;

% Define Faces
P.F = [ ...
    1,  2,  3; ... % 1 nose top
    1,  3,  4; ... % 2 nose left
    1,  4,  5; ... % 3 nose bottom
    1,  2,  5; ... % 4 nose right
    
    3,  2,  6; ... % 5 fuselage top
    3,  4,  6; ... % 6 fuselage left
    4,  5,  6; ... % 7 fuselage bottom
    5,  2,  6; ... % 8 fuselage right
    
    8,  10, 9; ... % 9 wing bottom left triangle
    8,  10, 7; ... % 10 wing upper right triangle
    11, 12, 14;... % 11 tailwing upper right triangle
    12, 14, 13;... % 12 tailwing lower right triangle
    
    15, 16, 6; ... % 13 tail
    ];

% Colors for each face
P.patchcolors = [ ...
    [1,0,0]; ... % 1 red
    [0,1,0]; ... % 2 green
    [0,0,1]; ... % 3 blue
    [1,1,0]; ... % 4 yellow
    [0,1,1]; ... % 5 cyan
    [1,0,1]; ... % 6 magenta
    [.6,.4,0]; ... % 7 orange
    [.6,0,.4]; ... % 8 pink
    [.4,0,.6]; ... % 9 purple
    [.4,0,.6]; ... % 10 purple  same for the wing portions 
    [0,.4,.6]; ... % 11 light blue
    [0,.4,.6]; ... % 12 light blue
    [.4,.6,0]; ... % 13 light green
    ];