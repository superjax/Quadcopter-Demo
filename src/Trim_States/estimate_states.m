% estimate_states
%   - estimate the MAV states using gyros, accels, pressure sensors, and
%   GPS.
%
% Outputs are:
%   pnhat    - estimated North position, 
%   pehat    - estimated East position, 
%   hhat     - estimated altitude, 
%   Vahat    - estimated airspeed, 
%   alphahat - estimated angle of attack
%   betahat  - estimated sideslip angle
%   phihat   - estimated roll angle, 
%   thetahat - estimated pitch angel, 
%   chihat   - estimated course, 
%   phat     - estimated roll rate, 
%   qhat     - estimated pitch rate, 
%   rhat     - estimated yaw rate,
%   Vghat    - estimated ground speed, 
%   wnhat    - estimate of North wind, 
%   wehat    - estimate of East wind
%   psihat   - estimate of heading angle
% 
% 
% Modified:  3/15/2010 - RB
%            5/18/2010 - RB
%

function xhat = estimate_states(uu, P)

   % rename inputs
   y_gyro_x      = uu(1);
   y_gyro_y      = uu(2);
   y_gyro_z      = uu(3);
   y_accel_x     = uu(4);
   y_accel_y     = uu(5);
   y_accel_z     = uu(6);
   y_static_pres = uu(7);
   y_diff_pres   = uu(8);
   y_mag         = uu(9);
   y_GPS_n       = uu(10);
   y_GPS_e       = uu(11);
   %y_GPS_h       = uu(12);
   y_GPS_Vg      = uu(13);
   y_GPS_chi     = uu(14);
   t             = uu(15);
   %% Initialize
     
   % Persistent Variables
   alpha = P.alpha_LPF;
   persistent y_gyro_x_old;
   persistent y_gyro_y_old;
   persistent y_gyro_z_old;
   persistent y_static_pres_old;
   persistent y_diff_pres_old;
   persistent x1_old;
   persistent x2_old;
   persistent y_accel_old;
   persistent P1;
   persistent P2;
   persistent GPS_old;
   persistent R1;
   persistent Vahat_old;
   persistent y_dVahat_old;
   persistent t_old;
   persistent y_mag_old;
   T_out = P.Ts;
   N = 20;
   
   
   if t==0
       P1 = zeros(2,2);
       P2 = zeros(7,7);
       x1_old = [P.phi0;P.theta0];
       x2_old = [P.pn0,P.pe0,P.Va0,P.psi0,P.wind_n,P.wind_e,P.psi0]';
       GPS_old = [y_GPS_n; y_GPS_e; y_GPS_Vg; 0];%y_GPS_chi];
       y_accel_old = [y_accel_x;y_accel_y;y_accel_z];
       y_gyro_x_old = y_gyro_x;
       y_gyro_y_old = y_gyro_y;
       y_gyro_z_old = y_gyro_z;
       y_static_pres_old = y_static_pres;
       y_diff_pres_old = y_diff_pres;
       y_mag_old = y_mag;
       t_old = -T_out;
       Vahat_old =0;
   end
  
   EKF_1_on = 1;
   EKF_2_on = 1;
   
   
   %% Simple Sensor Inversion
   % Find angular rates, altitude and airspeed by directly inverting the
   % sensor measurements
   
   % Angular Rates
   phat = LPF(alpha, y_gyro_x_old, y_gyro_x);
   qhat = LPF(alpha, y_gyro_y_old, y_gyro_y);
   rhat = LPF(alpha, y_gyro_z_old, y_gyro_z);
   y_gyro_x_old = y_gyro_x;
   y_gyro_y_old = y_gyro_y;
   y_gyro_z_old = y_gyro_z;
   
   % Altitude
   hhat = LPF(alpha, y_static_pres_old, y_static_pres)/(P.rho*P.gravity);
   y_static_pres_old = y_static_pres;
   
   % Airspeed
   Vahat = sqrt(abs(2/P.rho*LPF(alpha,y_diff_pres_old,y_diff_pres)));
   y_diff_pres_old = y_diff_pres;
   
   % Acceleration
   if t==0
       y_dVahat = 0;
   else
       y_dVahat = (Vahat-Vahat_old)/(t-t_old);
   end
   y_dVahat_old = y_dVahat;
   dVahat = LPF(.1,y_dVahat_old,y_dVahat);
   Vahat_old = Vahat;
   t_old = t;
   
   % These are the bad assumption measurements
   if not(EKF_1_on)
       % attitude
       phihat = atan2(LPF(alpha, y_accel_old(2),y_accel_y),LPF(alpha,y_accel_old(3),y_accel_z));
       thetahat = asin2((LPF(alpha,y_accel_old(1),y_accel_x))/P.gravity);
       y_accel_old = [y_accel_x;y_accel_y;y_accel_z];
   end
   if not(EKF_2_on)
       % Position
       GPS = [y_GPS_n; y_GPS_e; y_GPS_Vg; y_GPS_chi];
       pnhat = LPF(alpha,GPS_old(1),GPS(1));
       pehat = LPF(alpha,GPS_old(2),GPS(2));
       chihat = LPF(alpha,GPS_old(4),GPS(4));
       Vghat = LPF(alpha,GPS_old(3),GPS(3));
       wnhat = 0;
       wehat = 0;
       psihat = LPF(alpha,y_mag_old,y_mag);
       y_mag_old = y_mag;
       GPS_old = GPS;
   end
   
   
   
   %% EKF #1
   % Roll and Pitch Estimator
   if EKF_1_on
   % initialize
   xhat1 = x1_old;
   
   if mod(t,T_out) == 0
       for i = 1:N
       phihat = xhat1(1);
       thetahat = xhat1(2);
       cp = cos(phihat);
       sp = sin(phihat);
       tt = tan(thetahat);

       f1 = [phat + qhat*sp*tt + rhat*cp*tt;...
             qhat*cp - rhat*sp];

       xhat1 = xhat1+T_out/N*f1;
       end
   end
   x1_old = xhat1;
   phihat = xhat1(1);
   thetahat = xhat1(2);
   end
   
   
%    u = [phat;qhat;rhat;Vahat];
%    y = [y_accel_x;y_accel_y;y_accel_z];
%     % prediction step
%    if mod(t,T_out) == 0
%         f =   [u(1) + u(2)*sin(x1(1))*tan(x1(2))+u(3)*cos(x1(1))*tan(x1(2));...
%                u(2)*cos(x1(1))-u(3)*sin(x1(1))];
%        for i=N
%            x1 = x1 + (T_out/N)*f;
%            A = [u(2)*cos(x1(1))*tan(x1(2))-u(3)*sin(x1(1))*tan(x1(2)),   (u(2)*sin(x1(1))+u(3)*cos(x1(1)))/(cos(x1(2)))^2;...
%                -u(2)*sin(x1(1))-u(3)*cos(x1(1)),                       0];
%            P1 = P1 + (T_out/N)*(A*P1 + P1*A' + P.Q1);
%        end
%    end
   
%    if 0%not(y_accel_old == y) % new sensor measurement
%        R1 = diag([0.00000625,0.00000625,0.00000625]);
%        C1 = [ 0,                                  u(2)*u(4)*cos(x1(2))+P.gravity*cos(x1(2));...
%               -P.gravity*cos(x1(1))*cos(x1(2)),   -u(3)*u(4)*sin(x1(2))-u(1)*u(4)*cos(x1(2))+P.gravity*sin(x1(1))*sin(x1(2));...
%               P.gravity*sin(x1(1))*cos(x1(2)),    (u(2)*u(4)+P.gravity*cos(x1(1)))*sin(x1(2))];
%          
%        L1 =   P1*C1'/(R1+C1*P1*C1'); % Can be optimized later
%        P1 = (eye(2) - L1*C1)*P1;
%        h1 =  [u(2)*u(4)*sin(x1(2))+P.gravity*sin(x1(2));...
%               u(3)*u(4)*cos(x1(2))-u(1)*u(4)*sin(x1(2))-P.gravity*cos(x1(2))*sin(x1(1));...
%               -u(2)*u(4)*cos(x1(2))-P.gravity*cos(x1(2))*cos(x1(1))];
% 
%         x1_new = x1 + L1*(y - h1);
%         % check residual
%         if norm(x1_new-x1)>10*pi()/180;
%             % try with a larger R1 because we know that our measurements
%             % suck
%            disp('Toss Measurement');
%            R1 = diag([0.005,0.001,0.001]);
%            C1 = [ 0,                                P.gravity*cos(x1(2));...
%                   -P.gravity*cos(x1(2))*cos(x1(2)), P.gravity*sin(x1(2))*sin(x1(1));...
%                    P.gravity*cos(x1(2))*sin(x1(1)),  P.gravity*sin(x1(1))*sin(x1(2))];
%            L1 =   P1*C1'/(R1+C1*P1*C1');
%            P1 = (eye(2) - L1*C1)*P1;
%            h1 = [P.gravity*sin(x1(2))+dVahat;...
%                  u(3)*u(4)-P.gravity*cos(x1(2))*sin(x1(1));...
%                  u(2)*u(4)-P.gravity*cos(x1(2))*cos(x1(1))];
%            x1_new = x1 + L1*(y - h1);
%         end
%         if or(norm(x1_new-x1)>10*pi()/180,abs(x1(1))>pi()/2); % Still sucks
%             disp('Screw You, Stupid EKF');
%             x1_new = [atan2(LPF(alpha, y_accel_old(2),y_accel_y),LPF(alpha,y_accel_old(3),y_accel_z));...
%                       asin2((LPF(alpha,y_accel_old(1),y_accel_x-dVahat))/P.gravity)];
%         end
%         x1 = x1_new;
%    end
  
   
%    phihat = x1(1);
%    thetahat = x1(2);
%    x1_old = x1;
%    y_accel_old = y;
%    end
   
   
   %% EKF #2
   % Rest of the Estimator
   if EKF_2_on
   % initialize
   x2 = x2_old; % [pn,pe,Vg,chi,wn,we,psi]'
   u = [Vahat,qhat,rhat,phihat,thetahat]';
   Va = Vahat;
   q = qhat;
   r = rhat;
   sp = sin(phihat);
   cp = cos(phihat);
   tp = tan(phihat);
   ct = cos(thetahat);
   
   % Prediction Step
   if mod(t,T_out) == 0
       for i=1:N
        % Pull variables
        pn = x2(1);
        pe = x2(2);
        Vg = x2(3);
        chi = x2(4);
        wn = x2(5);
        we = x2(6);
        psi = x2(7);
        cpsi = cos(psi);
        spsi = sin(psi);
        
        psidot = q*(sp/ct) + r*(cp/ct);
        Vgdot = (Va*(cpsi + wn)*(-psidot*spsi)+(spsi+we)*(psidot*cpsi))/Vg;
        chidot = P.gravity/Vg*tp*cos(chi-psi);
        dvgdotdpsi = (-psidot*Va*(wn*cpsi+we*spsi))/Vg;
        dchidotdvg = -P.gravity/(Vg^2)*tp*cos(chi-psi);
        dchidotdchi = (-P.gravity/Vg)*tp*sin(chi-psi);
        dchidotdpsi = (P.gravity/Vg)*tp*sin(chi-psi);
        
        f = [Vg*cos(chi);...
             Vg*sin(chi);...
             Vgdot;...
             chidot;...
             0;...
             0;...
             psidot];
         
          x2 = x2+(T_out/N)*f;
          
          A = [0,    0,  cos(x2(4)),     -u(3)*sin(x2(4)),   0,  0,  0;...
               0,    0,  sin(x2(4)),     u(3)*cos(x2(4)),    0,  0,  0;...
               0,    0,  -Vgdot/x2(3)    0,                  -psidot*u(1)*sin(x2(7)),    psidot*u(1)*cos(x2(7)), dvgdotdpsi;...
               0,    0,  dchidotdvg,     dchidotdchi,        0,  0,  dchidotdpsi;...
               0,    0,  0,              0,                  0,  0,  0;...
               0,    0,  0,              0,                  0,  0,  0;...
               0,    0,  0,              0,                  0,  0,  0];
          P2 = P2 + (T_out/N)*(A*P2+P2*A'+P.Q2);
       end
   end
   
   % Measurement Step
   GPS = [y_GPS_n; y_GPS_e; y_GPS_Vg; y_GPS_chi];
   if not(GPS_old == GPS)
       % Update R Matrix
       R2 = diag([.21,.21,.05,0.05/x2(3),0,0]);
       % PseudoMeasurements
       y_wind_n = u(1)*cos(x2(7))+x2(5) - x2(3)*cos(x2(4));
       y_wind_e = u(1)*sin(x2(7))+x2(5) - x2(3)*sin(x2(4));
       % x2 = [pn,pe,Vg,chi,wn,we,psi] u = [Va q r phi theta]  (For Reference)
       y2 = [y_GPS_n, y_GPS_e, y_GPS_Vg, y_GPS_chi, 0, 0]';
       h2 = [x2(1),x2(2),x2(3),x2(4),y_wind_n,y_wind_e]';
       C2 = [1,  0,  0,  0,  0,  0,  0;...
            0,  1,  0,  0,  0,  0,  0;...
            0,  0,  1,  0,  0,  0,  0;...
            0,  0,  0,  1,  0,  0,  0;...
            0,  0,  -cos(x2(4)), x2(3)*sin(x2(4)),  1,  0,  -u(1)*sin(x2(7));...
            0,  0,  -sin(x2(4)), x2(3)*cos(x2(4)),  0,  1,  u(1)*cos(x2(7))];
       L2 = P2*C2'/(R2 + C2*P2*C2');
       P2 = (eye(7) - L2*C2)*P2;
       x2 = x2+L2*(y2-h2);
   end
   GPS_old = GPS;
   x2_old = x2;
   
   % pull out estimates
   pnhat = x2(1);
   pehat = x2(2);
   Vghat = x2(3);
   chihat = x2(4);
   wnhat = x2(5);
   wehat = x2(6);
   psihat = x2(7);
   
   end
   %% Finish Up

    % not estimating these states 
    alphahat = 0;
    betahat  = 0;
    bxhat    = 0;
    byhat    = 0;
    bzhat    = 0;
    
      xhat = [...
        pnhat;...
        pehat;...
        hhat;...
        Vahat;...
        alphahat;...
        betahat;...
        phihat;...
        thetahat;...
        chihat;...
        phat;...
        qhat;...
        rhat;...
        Vghat;...
        wnhat;...
        wehat;...
        psihat;...
        bxhat;...
        byhat;...
        bzhat;...
        ];
   end

function ynplus1 = LPF(alpha,yn,un)
    ynplus1 = alpha*yn+(1-alpha)*un;
end


function result = asin2(x)
    if x > 1
        x = 1;
    end
    if x < -1
        x = -1;
    end
    result = x;
end
        