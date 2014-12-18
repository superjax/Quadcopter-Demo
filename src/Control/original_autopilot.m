%% Autopilot
% Brings in inputs and sends them to the correct autopilot algorithm
function y = autopilot(uu,P)

global P;
    % process inputs
    NN = 0;
    pn       = uu(1+NN);  % inertial North position
    pe       = uu(2+NN);  % inertial East position
    h        = uu(3+NN);  % altitude
    Va       = uu(4+NN);  % airspeed
    alpha    = uu(5+NN);  % angle of attack
    beta     = uu(6+NN);  % side slip angle
    phi      = uu(7+NN);  % roll angle
    theta    = uu(8+NN);  % pitch angle
    chi      = uu(9+NN);  % course angle
    p        = uu(10+NN); % body frame roll rate
    q        = uu(11+NN); % body frame pitch rate
    r        = uu(12+NN); % body frame yaw rate
    Vg       = uu(13+NN); % ground speed
    wn       = uu(14+NN); % wind North
    we       = uu(15+NN); % wind East
    psi      = uu(16+NN); % heading
    Va_c     = uu(20+NN);  % commanded airspeed (m/s)
    h_c      = uu(21+NN);  % commanded altitude (m)
    chi_c    = uu(22+NN);  % commanded course (rad)
    t        = uu(23+NN);   % time
    
    autopilot_version = 1;
        % autopilot_version == 1 <- used for tuning
        % autopilot_version == 2 <- standard autopilot defined in book
        % autopilot_version == 3 <- Total Energy Control for longitudinal AP
    switch autopilot_version
        case 1,
           [delta, x_command] = autopilot_tuning(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P);
        case 2,
           [delta, x_command] = autopilot_uavbook(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P);
        case 3,
           [delta, x_command] = autopilot_TECS(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P);
    end
    y = [delta; x_command];
end


%% Autopilot Uavbook
% autopilot defined in the uavbook
function [delta, x_command] = autopilot_uavbook(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P)
    % define persistent variable for state of altitude state machine
    persistent altitude_state old_altitude_state;
    persistent flag;
    persistent t_flag old_t_flag;
    if t==0
        t_flag = 0;
        old_t_flag = 0;
        flag = 0;
    end
   
    if h<=P.altitude_take_off_zone,     
        altitude_state = 1; % Take Off
    elseif h<=h_c-P.altitude_hold_zone, 
        altitude_state = 2; % Climb
    elseif h>=h_c+P.altitude_hold_zone, 
        altitude_state = 3; % Descend
        t_flag = 1;
    else
        altitude_state = 4; % Hold
    end
    
    if altitude_state ~= old_altitude_state;
        debug = 1;
    end
    old_altitude_state = altitude_state;
    
    if t_flag ~= old_t_flag
        t_flag = 0;
    end
    old_t_flag =1;
    

    % implement state machine
    switch altitude_state,
        case 1, % Take-off zone
            delta_t = .7;
            theta_c = P.theta_take_off;          
            
        case 2,  % climb zone
            delta_t = .7;
            theta_c = airspeed_with_pitch_hold(Va_c, Va, flag, P);               
             
        case 3, % descend zone
            delta_t = 0;
            theta_c = airspeed_with_pitch_hold(Va_c, Va, flag, P);   

        case 4, % altitude hold zone
            delta_t = airspeed_with_throttle_hold(Va_c, Va, t_flag, P);
            theta_c = altitude_hold(h_c, h, flag, P); 
            
    end
    phi_c   = course_hold(chi_c, chi, r, flag, P);
    delta_a = roll_hold(phi_c, phi, p, P);
    delta_e = pitch_hold(theta_c, theta, q, P);
    delta_r = 0;
    % artificially saturation delta_t
    delta_t = sat(delta_t,1,0);

    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        %theta_c*P.K_theta_DC;... % theta
        theta_c;
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
    y = [delta; x_command];
 
end

%% Autopilot Tuning
% used to tune gains
function [delta, x_command] = autopilot_tuning(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P)
    mode = 5;
    switch mode
        case 1, % tune the roll loop
            phi_c = chi_c; % interpret chi_c to autopilot as course command
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
            delta_e = P.u_trim(1);
            delta_t = P.u_trim(4);
            theta_c = 0;
        case 2, % tune the course loop
            if t==0,
                phi_c   = course_hold(chi_c, chi, r, 0, P);
            else
                phi_c   = course_hold(chi_c, chi, r, 1, P);
            end                
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
            delta_e = P.u_trim(1);
            delta_t = P.u_trim(4);
            theta_c = 0;
        case 3, % tune the throttle to airspeed loop and pitch loop simultaneously
            theta_c = 20*pi/180 + h_c;
            chi_c = 0;
            if t==0,
                phi_c   = course_hold(chi_c, chi, r, 0, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 0, P);
           else
                phi_c   = course_hold(chi_c, chi, r, 1, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 1, P);
            end
            delta_e = pitch_hold(theta_c, theta, q, P);
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
        case 4, % tune the pitch to airspeed loop 
            chi_c = 0;
            delta_t = P.u_trim(4);
            if t==0,
                phi_c   = course_hold(chi_c, chi, r, 0, P);
                theta_c = airspeed_with_pitch_hold(Va_c, Va, 0, P);
           else
                phi_c   = course_hold(chi_c, chi, r, 1, P);
                theta_c = airspeed_with_pitch_hold(Va_c, Va, 1, P);
            end
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_e = pitch_hold(theta_c, theta, q, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
        case 5, % tune the pitch to altitude loop 
            if t==0,
                phi_c   = course_hold(chi_c, chi, r, 0, P);
                theta_c = altitude_hold(h_c, h, 0, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 0, P);
           else
                phi_c   = course_hold(chi_c, chi, r, 1, P);
                theta_c = altitude_hold(h_c, h, 1, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 1, P);
            end
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_e = pitch_hold(theta_c, theta, q, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
      end
    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        %theta_c*P.K_theta_DC;... % theta
        theta_c;
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
end
%% Autopilot Functions
function delta_a = roll_hold(phi_c, phi, p, P)  
    error = phi_c-phi;
    delta_a = sat(P.k_p_phi*error - P.k_d_phi*p,P.delta_a_max,-P.delta_a_max);
end

function phi_c = course_hold(chi_c, chi, r, flag, P)
    persistent integrator;
    persistent prev_error;
    if flag==0
        integrator = 0;
        prev_error = 0;
    end
    error = chi_c-chi;
    kp = P.k_p_chi;
    ki = P.k_i_chi;
    limits= [45*pi()/180,-45*pi()/180];
    integrator = integrator + (P.Ts/2)*(error+prev_error);
    prev_error = error;
    phi_c = sat(kp*error + ki*integrator,limits(1),limits(2));    
    % anti-windup scheme
    
end

function delta_r = sideslip_hold(beta,P,flag)
    persistent integrator;
    persistent prev_error;
    if flag==0
        integrator = 0;
        prev_error = 0;
    end 
    integrator = integrator + (P.Ts/2)*(beta+prev_error);
    prev_error = beta;
    delta_r =  sat(-P.k_p_beta*beta -P.k_i_beta*integrator,P.delta_r_max,-P.delta_r_max);
end

function delta_e = pitch_hold(theta_c, theta, q, P)
    error = theta_c-theta;
    delta_e = sat(P.k_p_theta*error - P.k_d_theta*q,P.delta_e_max*pi()/180,-P.delta_e_max*pi()/180);
end

function theta_c = altitude_hold(h_c, h, flag, P)
    persistent integrator;
    persistent prev_error;
    if flag==0
        integrator = 0;
        prev_error = 0;
    end
    error = h_c-h;
    integrator = integrator + (P.Ts/2)*(error+prev_error);
    prev_error = error;
    kp = P.k_p_h;
    ki = P.k_i_h;
    % Anti-windup scheme
    u = sat(kp*error + ki*integrator,30*pi()/180,-30*pi()/180);
    u_unsat = kp*error + ki*integrator;
    if u~=u_unsat && abs(integrator) > abs((u-kp*error)/ki)
        integrator = (u-kp*error)/ki;
    end
    theta_c = u;
end

function delta_t = airspeed_with_throttle_hold(Va_c, Va, flag, P)
    persistent integrator;
    persistent differentiator
    persistent prev_error;
    if flag==0
        integrator = 0;
        prev_error = 0;
        differentiator = 0;
    end  
    error = Va_c - Va;
    gains = [P.k_p_t,P.k_i_t,0];
    limits= [1,0];
    par = [ P.Ts , P.Tau_chi ];
    history = [integrator,differentiator,prev_error];
    [output,history] = PID(gains, error, limits, history, par, flag);
    integrator = history(1);
    differentiator = history(2);
    prev_error = history(3);
    delta_t = output(1);
end

function delta_h = airspeed_with_pitch_hold(Va_c,Va,flag,P)
persistent integrator;
    persistent differentiator
    persistent prev_error;
    if flag==0
        integrator = 0;
        prev_error = 0;
        differentiator = 0;
    end  
    error = Va_c - Va;
    gains = [P.k_p_v,P.k_i_v,0];
    limits= [.5,0];
    par = [ P.Ts , P.Tau_chi ];
    history = [integrator,differentiator,prev_error];
    [output,history] = PID(gains, error, limits, history, par, flag);
    integrator = history(1);
    differentiator = history(2);
    prev_error = history(3);
    delta_h = output(1);

end

function [output, history] = PID(gains, error, limits, history, par, flag)
kp = gains(1);
ki = gains(2);
kd = gains(3);
pos_limit = limits(1);
neg_limit = limits(2);
integrator = history(1);
differentiator = history(2);
prev_error = history(3);
ts = par(1);
tau = par(2);
    switch flag
        case 0
            integrator = 0;
            previous_error = 0;
            differentiator = 0;
    end
    integrator = integrator + (ts/2)*(error+prev_error);
    differentiator = (2*tau-ts)/(2*tau+ts)*differentiator + 2/(2*tau+ts)*(error-prev_error);
    prev_error = error;
   
    u = sat(kp*error+ki*integrator+kd*differentiator, pos_limit, neg_limit);
    if ki~=0
        u_unsat = kp*error+ki*integrator+kd*differentiator;
         if u ~= u_unsat
            %integrator = (u-kp*error-kd*differentiator)/ki;
         end
    end
    output = [u;0;0];
    history = [integrator;differentiator;prev_error];
end
    
%% Saturation Function
function out = sat(in, up_limit, low_limit)
  if in > up_limit,
      out = up_limit;
  elseif in < low_limit;
      out = low_limit;
  else
      out = in;
  end
end