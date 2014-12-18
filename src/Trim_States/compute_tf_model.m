function [T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]...
    = compute_tf_model(x_trim,u_trim)
% x_trim is the trimmed state,
% u_trim is the trimmed input
Va_trim = (x_trim(4)^2+x_trim(5)^2+x_trim(6))^(1/2); % Assuming no wind
theta_trim = (x_trim(8));


global P

P.a_phi1 = -1/2*P.rho*Va_trim^2*P.S_wing*P.b*P.C_p_p*P.b/(2*Va_trim);
P.a_phi2 = 1/2*P.rho*Va_trim^2*P.S_wing*P.b*P.C_p_delta_a;

P.a_beta1 = -(P.rho*Va_trim*P.S_wing)/(2*P.mass)*P.C_Y_beta;
P.a_beta2 = (P.rho*Va_trim*P.S_wing)/(2*P.mass)*P.C_Y_delta_r;

P.a_theta1 = -(P.rho*Va_trim^2*P.c*P.S_wing)/(2*P.Jy)*P.C_m_q*P.c/(2*Va_trim);
P.a_theta2 = (P.rho*Va_trim^2*P.c*P.S_wing)/(2*P.Jy)*P.C_m_alpha;
P.a_theta3 = (P.rho*Va_trim^2*P.c*P.S_wing)/(2*P.Jy)*P.C_m_delta_e;

P.a_V1 = ((P.rho*P.Va0*P.S_wing)/P.mass)*(P.C_D_0+P.C_D_alpha*P.gamma+P.C_D_delta_e*u_trim(1))+P.rho*(P.S_prop)/P.mass*P.C_prop*P.Va0;  %Assumes no wind
P.a_V2 = P.rho*P.S_prop/P.mass * P.C_prop*P.k_motor^2*u_trim(4);
P.a_V3 = P.gravity*cos(x_trim(8)-P.gamma);





    
% define transfer functions
T_phi_delta_a   = tf([P.a_phi2],[1,P.a_phi1,0]);
T_chi_phi       = tf([P.gravity/Va_trim],[1,0]);
T_theta_delta_e = tf(P.a_theta3,[1,P.a_theta1,P.a_theta2]);
T_h_theta       = tf([Va_trim],[1,0]);
T_h_Va          = tf([theta_trim],[1,0]);
T_Va_delta_t    = tf([P.a_V2],[1,P.a_V1]);
T_Va_theta      = tf([-P.a_V3],[1,P.a_V1]);
T_v_delta_r     = tf([Va_trim*P.a_beta2],[1,P.a_beta1]);

