% gps.m
%   Compute the output of gps sensor
%

function y = gps(uu, P)

    % relabel the inputs
    Va      = uu(1);
%    alpha   = uu(2);
%    beta    = uu(3);
    wn      = uu(4);
    we      = uu(5);
%    wd      = uu(6);
    pn      = uu(7);
    pe      = uu(8);
    pd      = uu(9);
%    u       = uu(10);
%    v       = uu(11);
%    w       = uu(12);
%    phi     = uu(13);
%    theta   = uu(14);
    psi     = uu(15);
%    p       = uu(16);
%    q       = uu(17);
%    r       = uu(18);
    t       = uu(19);
    
    
    persistent nu_minus1;
    if isempty(nu_minus1)
        nu_minus1 = [0;0;0];
    end

    % construct North, East, and altitude GPS measurements
    nu = [ exp(-P.k_GPS*P.Ts_gps)*nu_minus1(1)+P.sigma_n_gps*randn(1);...
           exp(-P.k_GPS*P.Ts_gps)*nu_minus1(2)+P.sigma_e_gps*randn(1);... 
           exp(-P.k_GPS*P.Ts_gps)*nu_minus1(3)+P.sigma_h_gps*randn(1)];
    y_gps_n = pn + nu(1);
    y_gps_e = pe + nu(2);
    y_gps_h = -pd + nu(3);
    
    nu_minus1 = nu;
    
    % construct groundspeed and course measurements
    Vg = sqrt((Va*cos(psi)+wn)^2+(Va*sin(psi)+wn)^2);
    y_gps_Vg = Vg+P.sigma_Vg_gps*randn(1);
    sigma_course_gps = P.sigma_Vg_gps/Vg;
    y_gps_course = atan2(Va*sin(psi)+we,Va*cos( psi)+wn)+sigma_course_gps*randn(1);

    % construct total output
    y = [...
        y_gps_n;...
        y_gps_e;...
        y_gps_h;...
        y_gps_Vg;...
        y_gps_course;...
        ];
    
end



