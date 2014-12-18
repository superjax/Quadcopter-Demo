function y = mrPositionController(uu,P)
%
% attitude controller for hexacopter
%

% process inputs
NN = 0;
pn    = uu(1+NN);  % inertial North position
pe    = uu(2+NN);  % inertial East position
pd    = uu(3+NN);  % inertial Down position
u     = uu(4+NN);  % velocity in North direction
v     = uu(5+NN);  % velocity in East direction
w     = uu(6+NN);  % velocity in Down direction
phi   = uu(7+NN);  % roll angle
theta = uu(8+NN);  % pitch angle
psi   = uu(9+NN);
p     = uu(10+NN); % body frame roll rate
q     = uu(11+NN); % body frame pitch rate
r     = uu(12+NN); % body frame yaw rate


NN = NN+12;
x_c   = uu(1+NN);  % commanded x
y_c   = uu(2+NN);  % commanded y
z_c   = uu(3+NN);  % commanded z
yaw_c = uu(4+NN);  % commanded yaw

NN = NN+4;
t     = uu(1+NN);  % time

% define PID loops
persistent pid_x;
persistent pid_y;
persistent pid_z;
persistent pid_yaw;

% Initialize autopilot
if t==0
    % Initialize PID loops
    pid_x = SimplePID('pid_x', ...
        P.p_gains.Kp_x, P.p_gains.Ki_x, P.p_gains.Kd_x, ...
        P.Ts, P.tau, P.pitch_c_max);
    pid_y = SimplePID('pid_y', ...
        P.p_gains.Kp_y, P.p_gains.Ki_y, P.p_gains.Kd_y, ...
        P.Ts, P.tau, P.roll_c_max);
    pid_z = SimplePID('pid_z', ...
        P.p_gains.Kp_z, P.p_gains.Ki_z, P.p_gains.Kd_z, ...
        P.Ts, P.tau, P.F_max);
    pid_yaw = SimplePID('pid_yaw', ...
        P.p_gains.Kp_yaw, P.p_gains.Ki_yaw, P.p_gains.Kd_yaw, ...
        P.Ts, P.tau, P.yaw_rate_c_max);
end

% Position controller
    cp = cos(phi);
    sp = sin(phi);
    ct = cos(theta);
    st = sin(theta);
    cs = cos(psi);
    ss = sin(psi);
xdot = [ct*cs, sp*st*cs-cp*ss, cp*st*cs+sp*ss; ct*ss, sp*st*ss+cp*cs, cp*st*ss-sp*cs; st, -sp*ct, -cp*ct]*[u;v;w];
pitch_c    = -pid_x.computeCommandDirect(x_c, pn,xdot(1)); 
roll_c     =  pid_y.computeCommandDirect(y_c, pe,xdot(2));   
thrust_c   = -pid_z.computeCommandDirect(z_c, pd,-xdot(3)) + P.p_gains.z_ff;
yaw_rate_c =  pid_yaw.computeCommand(yaw_c, psi);

R = [ cos(psi),-sin(psi),0;
      sin(psi),cos(psi),0;
      0,0,1];
  
rotated_c = R*[pitch_c;roll_c;0];

% control outputs
delta = [rotated_c(1); rotated_c(2); yaw_rate_c; thrust_c];

y = [delta];

end