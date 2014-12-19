format compact
warning('off')
addpath(genpath(pwd))  % allows for files in subdirectories to be accessible by MATLAB
run mrParams
run mrDefineGeometry
run computeGains
run sensorParams

P.phi0 = .25;
P.theta0 = -.5;
P.psi0 = pi()/4;

P.lambda = 1000;