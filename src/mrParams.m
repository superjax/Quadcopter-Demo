
P.Ts = 0.01;

% physical parameters of airframe - determined in Quadcopter.xlsx
P.gravity = 9.81;
P.mass    = .321;
P.Jxx     = .002784;
P.Jyy     = .002784;
P.Jzz     = .004659;

% /todo: figure out what these do
P.L  = 0.25;
P.k1 = 1; %2.98*10e-6;
P.k2 = 1; %2.98*10e-6;
P.mu = 1;

% first cut at initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = 0;  % initial Down position (negative altitude)
P.u0     = 0;  % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 90*pi()/180;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate


% time constant for dirty derivative filter
P.tau = 0.15;

