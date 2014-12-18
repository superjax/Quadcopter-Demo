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

    sizes.NumContStates  = 8;
    sizes.NumDiscStates  = 0;
    sizes.NumOutputs     = 16+12;
    sizes.NumInputs      = 3;
    sizes.DirFeedthrough = 0;
    sizes.NumSampleTimes = 1;   % at least one sample time is needed

    sys = simsizes(sizes);
    x0  = [...
        P.pn0;...    % initial North position
        P.pe0;...    % initial East position
        P.psi0;...   % initial heading
        0;...        % initial heading rate
        -P.pd0;...   % initial altitude
        0;...        % initial climb rate
        P.Va0;...    % initial airspeed
        P.phi0;...   % initial roll angle
        ];
    str = [];
    ts  = [0 0];
    simStateCompliance = 'UnknownSimState';
end

function sys=mdlDerivatives(t,x,u,P)
  pn     = x(1); % North position
  pe     = x(2); % East position
  chi    = x(3); % heading
  chidot = x(4); % heading rate
  h      = x(5); % altitude 
  hdot   = x(6); % climb rate
  Va     = x(7); % airspeed
  phi    = x(8); % roll angle
  Va_c   = u(1); % commanded airspeed
  h_c    = u(2); % commanded altitude
  phi_c  = u(3); % commanded heading angle
  
  wn = P.wind_n;
  we = P.wind_e;
  psi = chi-asin(1/(Va)*[wn,we]*[-sin(chi);cos(chi)]);  %eq 2.12
  
  % Assuming we do not command derivatives
  hdot_c = 0;
  chidot = P.gravity/Va*tan(phi);
  chiddot = P.gravity/Va*(1/cos(phi)^2);
  chi_c = 0; % modified to have course as an input
  
  pndot = Va*cos(psi)+wn;
  pedot = Va*sin(psi)+we;
  hddot = P.b_hdot*(hdot_c-hdot) + P.b_h*(h_c-h);
  Vadot = P.b_Va*(Va_c-Va);
  phidot = P.b_phi*(phi_c-phi);
  
  
sys = [...
    pndot;...
    pedot;...
    chidot;...
    chiddot;...
    hdot;...
    hddot;...
    Vadot;...
    phidot;...
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
