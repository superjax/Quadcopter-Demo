% sensor parameters
P.sigma_gyro = 0.0396; % standard deviation of gyros in rad/sec
P.sigma_accel = 0.00094; % standard deviation of accelerometers in g
P.sigma_static_pres = 0.3320; % standard deviation of static pressure sensor in Pascals
P.sigma_diff_pres = 2.3;  % standard deviation of diff pressure sensor in Pascals
P.beta_static_pres = 125;  % bias drift of pressure sensors in Pa
P.beta_diff_pres = 20;  % bias drift of pressure sensor in Pa
P.h_ground = 0;
P.bias_gyro_x = 0;
P.bias_gyro_y = 0;
P.bias_gyro_z = 0;


% GPS parameters
P.Ts_gps = 1; % sample rate of GPS in s
P.beta_gps = 1/16000; % 1/s
P.sigma_n_gps = 2.1;
P.sigma_e_gps = 2.1; 
P.sigma_h_gps = 4.0;
P.sigma_Vg_gps = .01;
P.k_GPS = 1/1100;
P.sigma_mag = .3*pi()/180;
P.beta_mag=1*pi()/180;
