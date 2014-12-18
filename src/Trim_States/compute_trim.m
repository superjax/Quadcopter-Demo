function [Xt,Ut] = compute_trim(filename, Va, gamma, R)
% Va is the desired airspeed (m/s)
% gamma is the desired flight path angle (radians)
% R is the desired radius (m) - use (+) for right handed orbit, 
%                                   (-) for left handed orbit


% Create Trim Parameters
switch R
    case Inf
        DX0 = [0,0,-Va*sin(gamma),0,0,0,0,0,0,0,0,0]';    
    otherwise
        DX0 = [0,0,-Va*sin(gamma),0,0,0,0,0,Va/R,0,0,0]'; 
end
IDX = [3,4,5,6,7,8,9,10,11,12]';

X0 = [0,0,0,Va,0,0,0,gamma,0,0,0,0]';
IX = [];
U0 = [0,0,0,1]';
IU = [];
Y0 = [Va, 0, 0]';
IY = [1,3]';


% compute trim conditions
[Xt,Ut,Yt,DXt] = trim(filename,X0,U0,Y0,IX,IU,IY,DX0,IDX);

% check to make sure that the linearization worked (should be small)
norm(DXt(3:end)-DX0(3:end))

