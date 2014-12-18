% sensors.m
%   Compute the output of rate gyros, accelerometers, and pressure sensors
%

function y = sensors(uu, P)

    % relabel the inputs
%    pn      = uu(1);
%    pe      = uu(2);
    pd      = uu(3);
%    u       = uu(4);
%    v       = uu(5);
%    w       = uu(6);
    phi     = uu(7);
    theta   = uu(8);
    psi     = uu(9);
    p       = uu(10);
    q       = uu(11);
    r       = uu(12);
    F_x     = uu(13);
    F_y     = uu(14);
    F_z     = uu(15);
%    M_l     = uu(16);
%    M_m     = uu(17);
%    M_n     = uu(18);
    Va      = uu(19);
%    alpha   = uu(20);
%    beta    = uu(21);
%    wn      = uu(22);
%    we      = uu(23);
%    wd      = uu(24);
    
    % simulate rate gyros (units are rad/sec)
    P.sigma_gyro = .13*pi()/180;% rad/s
    y_gyro_x = p + P.sigma_gyro*randn(1);
    y_gyro_y = q + P.sigma_gyro*randn(1);
    y_gyro_z = r + P.sigma_gyro*randn(1);

    % simulate accelerometers (units of g)
    P.sigma_accel = .0025; % g
    y_accel_x = F_x/P.mass+P.gravity*sin(theta)+ P.sigma_accel*randn(1);
    y_accel_y = F_y/P.mass+P.gravity*cos(theta)*sin(phi)+P.sigma_accel*randn(1);
    y_accel_z = F_z/P.mass+P.gravity*cos(theta)*cos(phi)+P.sigma_accel*randn(1);

    % simulate pressure sensors
    P.beta_static_pres = .125; %kPa
    P.sigma_static_pres = .01; %kPa
    P.beta_diff_pres = .02; %kPa
    P.sigma_static_pres = .002; %kPa
    y_static_pres = P.rho*P.gravity*(P.h_ground-pd) + P.beta_static_pres + P.sigma_static_pres*randn(1);
    y_diff_pres = P.rho*Va^2/2 + P.beta_diff_pres + P.sigma_diff_pres*randn(1);

    % simulate magnetometer
    y_mag = psi + P.sigma_mag*randn(1);
    
    % construct total output
    y = [...
        y_gyro_x;...
        y_gyro_y;...
        y_gyro_z;...
        y_accel_x;...
        y_accel_y;...
        y_accel_z;...
        y_static_pres;...
        y_diff_pres;...
        y_mag;...
    ];

end



