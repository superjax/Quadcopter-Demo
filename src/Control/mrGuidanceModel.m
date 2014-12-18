function [sys,x0,str,ts,simStateCompliance] = roll_guidance_model(t,x,u,flag,P)

    switch flag,
      case 0,
        [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P);
      case 1,
        sys=mdlDerivatives(t,x,u,P);
      case 2,
        sys=mdlUpdate(t,x,u);
      case 3,
        sys=mdlOutputs(t,x,u,P);
      case 4,
        sys=mdlGetTimeOfNextVarHit(t,x,u);
      case 9,
        sys=mdlTerminate(t,x,u);
      otherwise
        DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
    end
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

end

function sys=mdlDerivatives(t,x,u,P)
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


  
sys = [...
    pndot;...
    pedot;...
    pddot;...
    udot
    vdot
    wdot
    phidot
    thetadot
    psidot
    pdot
    qdot
    rdot
    ];
end

function sys=mdlUpdate(t,x,u)
sys = [];
end

function sys=mdlOutputs(t,x,u,P)
  pn     = x(1); % North position
  pe     = x(2); % East position
  chi    = x(3); % course
  chidot = x(4); % course rate
  h      = x(5); % altitude
  hdot   = x(6); % climb rate
  Va     = x(7); % airspeed
  
  alpha  = 0;
  beta   = 0;

  % wind speed
  wn = P.wind_n;
  we = P.wind_e;
  
  % solve for heading and groundspeed
  psi = chi - asin( (-P.wind_n*sin(chi)+P.wind_e*cos(chi))/Va );
  Vg  = [cos(chi), sin(chi)]*(Va*[cos(psi); sin(psi)] + [wn; we]); 
  
  % roll angle is given by psidot = g/V*tan(phi)
  phi     = atan(Vg*chidot/P.gravity);
  
  % letting theta equal flight path angle given by hdot = V sin(gamma)
  theta = asin2(hdot/Va);
    
  % set angular rates to zero
  p     = 0;
  q     = 0;
  r     = 0;
  
  
% output the same states that are returned by the state estimation block
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
% also need to return the normal state vector so that we don't need to
% change the drawing routine.
sys = [pn; pe; h; Va; alpha; beta; phi; theta; chi; p; q; r; Vg; wn; we; psi;...
    pn; pe; -h; Va; 0; 0; phi; theta; psi; p; q; r];
end

function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;
end

function sys=mdlTerminate(t,x,u)
sys = [];
end

function result=asin2(x)
    if x > 1
        x =1;
    else if x<-1
            x = -1
        end
    end
    result = asin(x);
end
