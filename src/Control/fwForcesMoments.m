% forces_moments.m
%   Computes the forces and moments acting on the airframe. 
%
%   Output is
%       F     - forces
%       M     - moments
%       Va    - airspeed
%       alpha - angle of attack
%       beta  - sideslip angle
%       wind  - wind vector in the inertial frame
%

function out = forces_moments2(x, delta, wind,P)

    % relabel the inputs
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    delta_e = delta(1);
    delta_a = delta(2);
    delta_r = delta(3);
    delta_t = delta(4);
    w_ns    = wind(1); % steady wind - North
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis
    global P;
    
    %Gravity
    Fg = [-P.mass*P.gravity*sin(theta);...
          P.mass*P.gravity*cos(theta)*sin(phi);...
          P.mass*P.gravity*cos(theta)*cos(phi);];
    
    % Air Data
    Vw = rotate([w_ns;w_es;w_ds],phi,theta,psi) + [u_wg;v_wg;w_wg]; % B-frame
    Va = [u;v;w] - Vw; % B-frame
    Vamag = norm(Va);               % Airspeed Magnitude (scalar)
    alpha = atan2(Va(3),Va(1));      % Angle of Attack
    beta = asin(Va(2)/Vamag);       % Slideslip angle
    
      
    % Aerodynamic Constants
    A = 1/2*P.rho*Vamag^2*P.S_wing;
    C = P.c/(2*Vamag);
    B = P.b/(2*Vamag);
    
    % Aerodynamic Forces/Moments
    % Longitudinal
    Flift = ( C_L(alpha) + P.C_L_q*C*q + P.C_L_delta_e*delta_e );
    Fdrag = ( C_D(alpha) + P.C_D_q*C*q + P.C_D_delta_e*delta_e );
    Mm    = (P.C_m_0 + P.C_m_alpha*alpha + P.C_m_q*C*q + P.C_m_delta_e*delta_e);
    F = [cos(alpha), -sin(alpha); sin(alpha), cos(alpha)]*[-Fdrag;-Flift];
    % Lateral
    Fy    = ( P.C_Y_0 + P.C_Y_beta*beta + P.C_Y_p*B*p + P.C_Y_r*B*r + P.C_Y_delta_a*delta_a + P.C_Y_delta_r*delta_r );
    Mell  = ( P.C_ell_0 + P.C_ell_beta*beta + P.C_ell_p*B*p + P.C_ell_r*B*r + P.C_ell_delta_a*delta_a + P.C_ell_delta_r*delta_r );
    Mn    = ( P.C_n_0 + P.C_n_beta*beta + P.C_n_p*B*p + P.C_n_r*B*r + P.C_n_delta_a*delta_a + P.C_n_delta_r*delta_r );
    
    % Prop Forces/Moments
    Fp = 1/2*P.rho*P.S_prop*P.C_prop*[(P.k_motor*delta_t)^2-Vamag^2;0;0];
    Mp = -P.k_T_P*(P.k_Omega*delta_t)^2;
    
    Force  = Fg + A*[F(1);Fy;F(2)] + Fp;
    Torque = A*[B*Mell;C*Mm;B*Mn] + [Mp;0;0];

    out = [Force(1);Force(2);Force(3); Torque(1);Torque(2);Torque(3); Vamag; alpha; beta;Vw(1,1);Vw(2,1);Vw(3,1)];
end

function lift = C_L(alpha)
    global P
    sigma = (1+exp(-(P.M)*(alpha-P.alpha0)) + exp(P.M*(alpha+P.alpha0)))/...
            ((1+exp(-P.M*(alpha-P.alpha0)))*(1+exp(P.M*(alpha+P.alpha0))));
    lift = (1 - sigma) *  (P.C_L_0 + P.C_L_alpha*alpha) + sigma*(2*sign(alpha)*sin(alpha)^2*cos(alpha));
end

function drag = C_D(alpha)
    global P
    AR = P.b^2/P.S_wing;
    drag = P.C_D_p  +  (P.C_L_0 + P.C_L_alpha*alpha)^2 / (pi()*P.e*AR);
end

function XYZ=rotate(XYZ,phi,theta,psi)
  % define rotation matrix
  R_roll = [...
          1, 0, 0;...
          0, cos(phi), -sin(phi);...
          0, sin(phi), cos(phi)];
  R_pitch = [...
          cos(theta), 0, sin(theta);...
          0, 1, 0;...
          -sin(theta), 0, cos(theta)];
  R_yaw = [...
          cos(psi), -sin(psi), 0;...
          sin(psi), cos(psi), 0;...
          0, 0, 1];
  R = R_yaw*R_pitch*R_roll;
  % rotate vertices
  XYZ = R*XYZ;
end
