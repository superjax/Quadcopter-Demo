format compact
warning('off')
addpath(genpath(pwd))
run mrParams
run mrDefineGeometry
run computeGains
run sensorParams
%run get_moments


% Guidance Model Gains
P.b_phidot = 1;
P.b_phi = 1.3;
P.b_hdot = 1;
P.b_h = .7;
P.b_Va = 2;
P.b_chidot = 1;
P.b_chi = 1;