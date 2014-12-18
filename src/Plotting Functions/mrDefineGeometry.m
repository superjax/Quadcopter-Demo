
if isfield(P,'x_quad') && isfield(P,'y_quad') && isfield(P,'z_quad') && isfield(P,'c_quad')
    % do nothing
else
    [P.x_quad, P.y_quad, P.z_quad, P.c_quad] = stlread2('quadcopter_lean.stl');
    [m,n] = size(P.c_quad);
    P.c_quad_blue = P.c_quad;
    P.c_quad_green = P.c_quad;
    P.c_quad_red = P.c_quad;
    P.scale = .05;
    P.x_quad = P.x_quad*P.scale;
    P.y_quad = P.y_quad*P.scale;
    P.z_quad = P.z_quad*P.scale;

    for i = 1:n
        P.c_quad_blue(:,i) = [1;1;1]; %not sure what these are doing
        P.c_quad_red(:,i) = [5;100;0];
        P.c_quad_green(:,i) = [0;1;1];
    end
end

