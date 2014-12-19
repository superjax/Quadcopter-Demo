function [sys,x0,str,ts,simStateCompliance] = mrQuaternionDynamics(t,x,u,flag,P)

switch flag,

  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P);

  case 1,
    sys=mdlDerivatives(t,x,u,P);

  case 2,
    sys=mdlUpdate(t,x,u);

  case 3,
    sys=mdlOutputs(t,x,u);

  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  case 9,
    sys=mdlTerminate(t,x,u);

  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P)
sizes = simsizes;

sizes.NumContStates  = 13;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 12;
sizes.NumInputs      = 4;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
%convert from initial phi,theta,psi to quaternion
quat = convertEuler([P.phi0;P.theta0;P.psi0]);
x0  = [P.pn0; P.pe0; P.pd0; P.u0; P.v0; P.w0; quat; P.p0; P.q0; P.r0];
str = [];
ts  = [0 0];

simStateCompliance = 'UnknownSimState';

function sys=mdlDerivatives(t,x,uu,P)
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
    e0      = x(7);
    e1      = x(8);
    e2      = x(9);
    e3      = x(10);
    p       = x(11);
    q       = x(12);
    r       = x(13);

    % Inputs (Force and Torques - does not include gravity)
    F       = uu(1);
    T_x     = uu(2);
    T_y     = uu(3);
    T_z     = uu(4);
    
    % state derivative vector
    x_dot = zeros(13,1);
   
    % translational kinematics model
    tkm = [e1^2+e0^2-e2^2-e3^2, 2*(e1*e2-e3*e0),        2*(e1*e3 + e2*e0);...
           2*(e1*e2+e3*e0),     e2^2+e0^2-e1^2-e3^2,    2*(e2*e3-e1*e0);...
           2*(e1*e3-e2*e0),     2*(e2*e3+e1*e0),        e3^2+e0^2-e1^2-e2^2];
       
    x_dot(1:3) = tkm*[u; v; w];
    
    % translational dynamics model
    tdm1 = [r*v-q*w; 
            p*w-r*u; 
            q*u-p*v];
    % the force of gravity is not calculated in the forces and moment function   
    gravity = P.gravity*[2*(e1*e3-e2*e0);...
                         2*(e2*e3+e1*e0);...
                         e3^2+e0^2-e1^2-e2^2];
    x_dot(4:6) = tdm1 + gravity + (1/P.mass)*[0; 0; -F]; % (body frame)
    
    rkm = [0,    -p,     -q,        -r;...
           p,     0,      r,        -q;...
           q,     -r,     0,         p;...
           r,      q,     -p,        0];
    rkm = rkm + P.lambda*(1-norm([e0,e1,e2,e3])^2)*eye(4,4);
    x_dot(7:10) = 1/2*rkm*[e0;e1;e2;e3];
    
    % rotational dynamics model
    rdm1 = [(P.Jyy-P.Jzz)/P.Jxx*q*r;
            (P.Jzz-P.Jxx)/P.Jyy*p*r;
            (P.Jxx-P.Jyy)/P.Jzz*p*q];    
    rdm2 = [1/P.Jxx 0       0;
            0       1/P.Jyy 0;
            0       0       1/P.Jzz];     
    x_dot(11:13) = rdm1 + rdm2*[T_x; T_y; T_z];
sys = x_dot;


function sys=mdlUpdate(t,x,u)

sys = [];


function sys=mdlOutputs(t,x,u)

% convert to euler angles so the rest of the code doesn't explode
q = x(7:10);
euler = convertQuat(q);
out = [x(1:6);euler;x(11:13)];
if isreal(out)~=1
    pause = 1;
end
sys = out;


function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;
sys = t + sampleTime;


function sys=mdlTerminate(t,x,u)

sys = [];

function euler = convertQuat(q)
    test = q(1)*q(3)-q(2)*q(4);
    phi = atan2(2*(q(1)*q(2)+q(3)*q(4)),(q(1)^2+q(4)^2-q(2)^2-q(3)^2));
    psi = atan2(2*(q(1)*q(4)+q(2)*q(3)),(q(1)^2+q(2)^2-q(3)^2-q(4)^2));
    if test > .499 % singularity at north pole
       theta = pi()/2;
    else if test < -.499 % singularity at south pole
            theta = -pi()/2;
         else
            theta = asin(2*test);
         end
    end
    euler = [phi;theta;psi];
    
function quat = convertEuler(eu)
    phi     = eu(1);
    theta   = eu(2);
    psi     = eu(3);
    quat = [cos(psi/2)*cos(theta/2)*cos(phi/2)+sin(psi/2)*sin(theta/2)*sin(phi/2);...
            cos(psi/2)*cos(theta/2)*sin(phi/2)-sin(psi/2)*sin(theta/2)*cos(phi/2);...
            cos(psi/2)*sin(theta/2)*cos(phi/2)+sin(psi/2)*cos(theta/2)*sin(phi/2);...
            sin(psi/2)*cos(theta/2)*cos(phi/2)-cos(psi/2)*sin(theta/2)*sin(phi/2)];
        
        