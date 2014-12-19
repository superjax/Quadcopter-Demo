function [sys,x0,str,ts,simStateCompliance] = mrDynamics(t,x,u,flag,P)

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

sizes.NumContStates  = 12;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 12;
sizes.NumInputs      = 4;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0  = [P.pn0; P.pe0; P.pd0; P.u0; P.v0; P.w0; P.phi0; P.theta0; P.psi0; P.p0; P.q0; P.r0];
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
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    F       = uu(1);
    T_phi   = uu(2);
    T_theta = uu(3);
    T_psi   = uu(4);
    
    % state derivative vector
    x_dot = zeros(12,1);
    
    % calculate sines and cosines once for convenience and efficiency
    cp = cos(phi);
    sp = sin(phi);
    ct = cos(theta);
    st = sin(theta);
    tt = tan(theta);
    cs = cos(psi);
    ss = sin(psi);
  
    
     % translational kinematics model
    tkm = [ct*cs sp*st*cs-cp*ss cp*st*cs+sp*ss;
           ct*ss sp*st*ss+cp*cs cp*st*ss-sp*cs;
           -st   sp*ct          cp*ct];
       
    x_dot(1:3) = tkm*[u; v; w];
    
    % translational dynamics model
    tdm1 = [r*v-q*w; 
            p*w-r*u; 
            q*u-p*v];
       
    tdm2 = [-P.gravity*st; 
             P.gravity*ct*sp; 
             P.gravity*ct*cp];
       
    x_dot(4:6) = tdm1 + tdm2 + (1/P.mass)*[0; 0; -F];
    
    % rotational kinematic model
    rkm = [1 sp*tt cp*tt;
           0 cp    -sp;
           0 sp/ct cp/ct];
       
    x_dot(7:9) = rkm*[p; q; r];
    
    % rotational dynamics model
    rdm1 = [(P.Jyy-P.Jzz)/P.Jxx*q*r;
            (P.Jzz-P.Jxx)/P.Jyy*p*r;
            (P.Jxx-P.Jyy)/P.Jzz*p*q];
        
    rdm2 = [1/P.Jxx 0       0;
            0       1/P.Jyy 0;
            0       0       1/P.Jzz];
        
    x_dot(10:12) = rdm1 + rdm2*[T_phi; T_theta; T_psi];

sys = x_dot;


function sys=mdlUpdate(t,x,u)

sys = [];


function sys=mdlOutputs(t,x,u)

sys = x;


function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;
sys = t + sampleTime;


function sys=mdlTerminate(t,x,u)

sys = [];
