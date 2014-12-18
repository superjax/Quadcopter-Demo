function [A_lon,B_lon,A_lat,B_lat] = compute_ss_model(filename,x_trim,u_trim)
% x_trim is the trimmed state,
% u_trim is the trimmed input

%{
global P

pn = x_trim(1);
pe = x_trim(2);
pd = x_trim(3);
u = x_trim(4);
v = x_trim(5);
w = x_trim(6);
phi = x_trim(7);
theta = x_trim(8);
psi = x_trim(9);
p = x_trim(10);
q = x_trim(11);
r = x_trim(12);
beta = asin(v/(u^2+v^2+w^2)^(1/2));
alpha = atan2(w,u);
Va = sqrt(u^2+v^2+w^2);

de = u_trim(1);
da = u_trim(2);
dr = u_trim(3);
dt = u_trim(4);

% Formulas from page 81
Yv = (P.rho*P.S_wing*P.b*v)/(4*P.mass*P.Va0)*(P.C_Y_p*p + P.C_Y_r*r) +...  %check mass
     (P.rho*P.S_wing*v)/(P.mass)*(P.C_Y_0 + P.C_Y_beta*beta + P.C_Y_delta_a*da+P.C_Y_delta_r*dr) +...
     (P.rho*P.S_wing*P.C_Y_beta)/(2*P.mass)*sqrt(u^2+w^2);
Yp = w+(P.rho*Va*P.S_wing*P.b)/(4*P.mass)*P.C_Y_p;  %check mass
Yr = -u + (P.rho*Va*P.S_wing*P.b)/(4*P.mass)*P.C_Y_r; % check mass
Yda = (P.rho*Va^2*P.S_wing)/(2*P.mass)*P.C_Y_delta_a;
Ydr = (P.rho*Va^2*P.S_wing)/(2*P.mass)*P.C_Y_delta_r;

Lv = (P.rho*P.S_wing*P.b^2*v)/(4*Va)*(P.C_p_p*p+P.C_p_r*r) +...
     (P.rho*P.S_wing*P.b*v)*(P.C_p_0+P.C_p_beta*beta+P.C_p_delta_a*da +P.C_p_delta_r*dr) +...
     (P.rho*P.S_wing*P.b*P.C_p_beta)/2*sqrt(u^2+w^2);
Lp = P.T1*q + (P.rho*Va*P.S_wing*P.b^2)/4*P.C_p_p;
Lr = -P.T2*q + (P.rho*Va*P.S_wing*P.b^2)/4*P.C_p_r;
Lda = (P.rho*Va^2*P.S_wing*P.b)/2*P.C_p_delta_a;
Ldr = (P.rho*Va^2*P.S_wing*P.b)/2*P.C_p_delta_a;

Nv = (P.rho*P.S_wing*P.b^2*v)/(4*Va)*(P.C_r_p*p+P.C_r_r*r) +...
     (P.rho*P.S_wing*P.b*v)*(P.C_r_0+ P.C_r_beta*beta + P.C_r_delta_a*da + P.C_r_delta_r*dr) +...
     (P.rho*P.S_wing*P.b*P.C_r_beta)/2*sqrt(u^2+w^2);
Np = P.T7*q + (P.rho*Va*P.S_wing*P.b^2)/4*P.C_r_p;
Nr = -P.T7*q + (P.rho*Va*P.S_wing*P.b^2)/4*P.C_r_r;
Nda = P.rho*Va^2*P.S_wing*P.b/2*P.C_r_delta_a;
Ndr = P.rho*Va^2*P.S_wing*P.b/2*P.C_r_delta_r;

A_lat = [Yv,           Yp,             Yr,                 P.gravity*cos(theta)*cos(phi),                  0;...
        Lv,           Lp,             Lr,                 0,                                              0;...
        Nv,           Np,             Nr,                 0,                                              0;...
        0,            1,              cos(phi)*tan(theta),q*cos(phi)*tan(theta)-r*sin(phi)*tan(theta),    0;...
        0,            0,              cos(phi)*sec(theta),p*cos(phi)*sec(theta)-r*sin(phi)*sec(theta),    0];
    
B_lat = [Yda,   Ydr;...
         Lda,   Ldr;...
         Nda,   Ndr;...
         0,     0;...
         0,     0];

     
rho = P.rho;
S = P.S_wing;
m = P.mass;
c = P.c;
g = P.gravity;

% Calculate lift/drag parameters at trim
C_X_0       = -P.C_D_0*      cos(alpha) + P.C_L_0*      sin(alpha);
C_X_alpha   = -P.C_D_alpha*  cos(alpha) + P.C_L_alpha*  sin(alpha);
C_X_q       = -P.C_D_q*      cos(alpha) + P.C_L_q*      sin(alpha);
C_X_delta_e = -P.C_D_delta_e*cos(alpha) + P.C_L_delta_e*sin(alpha);
C_Z_0       = -P.C_D_0*      sin(alpha) + P.C_L_0*      cos(alpha);
C_Z_alpha   = -P.C_D_alpha*  sin(alpha) + P.C_L_alpha*  cos(alpha);
C_Z_q       = -P.C_D_q*      sin(alpha) + P.C_L_q*      cos(alpha);
C_Z_delta_e = -P.C_D_delta_e*sin(alpha) + P.C_L_delta_e*cos(alpha);


% Formulas from Page 86
Xu  = (u*rho*S)/m*(C_X_0+C_X_alpha*alpha+C_X_delta_e*de)...
      - (rho*S*w*C_X_alpha)/(2*m)...
      + (rho*S*c*C_X_q*u*q)/(4*m*Va)...
      - (rho*P.S_prop*P.C_prop*u)/m;
Xw  = -q + (w*rho*S)/m*(C_X_0+C_X_alpha*alpha+C_X_delta_e*de) ...
      + (rho*S*c*C_X_q*w*q)/(4*m*Va)...
      + (rho*S*C_X_alpha*u)/(2*m)...
      - (rho*P.S_prop*P.C_prop*w)/m;
Xq  = -w+(rho*Va*S*C_X_q*c)/(4*m);
Xde = (rho*Va^2*S*C_X_delta_e)/(2*m);
Xdt = (rho*P.S_prop*P.C_prop*P.k_motor^2*dt)/m;

Zu  = q + (u*rho*S)/m*(C_Z_0+C_Z_alpha*alpha + C_Z_delta_e*de)...
      - (rho*S*C_Z_alpha*w)/(2*m)...
      + (u*rho*S*C_Z_q*c*q)/(4*m*Va);
Zw  = (w*rho*S)/m*(C_Z_0+C_Z_alpha*alpha+C_Z_delta_e*de)...
      + (rho*S*C_Z_alpha*u)/(2*m)...
      + (rho*w*S*c*C_Z_q*q)/(4*m*Va);
Zq  = u + (rho*Va*S*C_Z_q*c)/(4*m);
Zde = (rho*Va^2*S*C_Z_delta_e)/(2*m);

Mu  = (u*rho*S*c)/P.Jy*(P.C_m_0 + P.C_m_alpha*alpha + P.C_m_delta_e*de)...
      - (rho*S*c*P.C_m_alpha*w)/(2*P.Jy)...
      + (rho*S*c^2*P.C_m_q*q*u)/(4*P.Jy*Va);
Mw  = (w*rho*S*c)/P.Jy*(P.C_m_0 + P.C_m_alpha*alpha + P.C_m_delta_e*de)...
      + (rho*S*c*P.C_m_alpha*u)/(2*P.Jy)...
      + (rho*Va*S*c^2*P.C_m_q*q*w)/(4*P.Jy*Va);
Mq  = (rho*Va*S*c^2*P.C_m_q)/(4*P.Jy);
Mde = (rho*Va^2*S*c*P.C_m_delta_e)/(2*P.Jy);

A_lon = [Xu,            Xw,             Xq,         -g*cos(theta),              0;...
         Zu,            Zw,             Zq,         -g*sin(theta),              0;...
         Mu,            Mw,             Mq,         0,                          0;...
         0,             0,              1,          0,                          0;...
         sin(theta),    -cos(theta),    0           u*cos(theta)+w*sin(theta),  0];

B_lon = [Xde,   Xdt;...
         Zde,   0;...
         Mde,   0;...
         0,     0;...
         0,     0];
%}

[A,B,C,D] = linmod(filename,x_trim,u_trim);

%       [ pn,pe,pd,u, v, w, phi,theta,psi,p,q,r]
A_lat = [ 0, 0, 0, 0, 1, 0, 0,  0,    0,  0,0,0;...
          0, 0, 0, 0, 0, 0, 0,  0,    0,  1,0,0;...
          0, 0, 0, 0, 0, 0, 0,  0,    0,  0,0,1;...
          0, 0, 0, 0, 0, 0, 1,  0,    0,  0,0,0;...
          0, 0, 0, 0, 0, 0, 0,  0,    1,  0,0,0];
      
A_lon = [ 0, 0, 0, 1, 0, 0, 0,  0,    0,  0,0,0;...
          0, 0, 0, 0, 0, 1, 0,  0,    0,  0,0,0;...
          0, 0, 0, 0, 0, 0, 0,  0,    0,  0,1,0;...
          0, 0, 0, 0, 0, 0, 0,  1,    0,  0,0,0;...
          0, 0, 0, 0, 0, 1, 0,  0,    0,  0,0,0];

%        da,de,dr,dt
B_lon = [0, 1, 0, 0;...
         0, 0, 0, 1];

B_lat = [1, 0, 0, 0;...
         0, 0, 1, 0];
      

B_lon = A_lon*B*B_lon';
B_lat = A_lat*B*B_lat';
           

A_lon = A_lon*A*A_lon';
A_lat = A_lat*A*A_lat';

end






