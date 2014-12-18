function mrDrawAircraft(uu,P)

    % process inputs to function
    pn1       = uu(1);  % inertial North position     
    pe1       = uu(2);  % inertial East position
    pd1       = uu(3);
%     pn2       = uu(4);
%     pe2       = uu(5);
%     pd2       = uu(6);
%     pn3       = uu(7);
%     pe3       = uu(8);
%     pd3       = uu(9);

    n = -6;
    u        = uu(10+n);       
    v        = uu(11+n);       
    w        = uu(12+n);       
    phi      = uu(13+n);  % roll angle         
    theta    = uu(14+n);  % pitch angle     
    psi      = uu(15+n);  % yaw angle     
    p        = uu(16+n); % roll rate
    q        = uu(17+n); % pitch rate     
    r        = uu(18+n); % yaw rate    
    x_c      = uu(19+n); % x command 
    y_c      = uu(20+n); % y command
    z_c      = uu(21+n); % z command
    yaw_c    = uu(22+n); % yaw command
    t        = uu(23+n); % time

    % define persistent variables 
    persistent spacecraft1_handle;
%     persistent spacecraft2_handle;
%     persistent spacecraft3_handle;
    persistent commanded_position_handle;
    persistent center;
    view_range = 10;
    close_enough_tolerance = 0.9;
    
    % first time function is called, initialize plot and persistent vars
    if t==0,
        figure(1), clf
        hold on;
        spacecraft1_handle = drawSpacecraftBody(P.x_quad, P.y_quad, P.z_quad, P.c_quad_red,...
                                               pn1,pe1,pd1,phi,theta,psi,...
                                               [],'normal');
%         spacecraft2_handle = drawSpacecraftBody(P.x_quad, P.y_quad, P.z_quad, P.c_quad_blue,...
%                                                pn2,pe2,pd2,phi,theta,psi,...
%                                                [],'normal');
%         spacecraft3_handle = drawSpacecraftBody(P.x_quad, P.y_quad, P.z_quad, P.c_quad_green,...
%                                                pn3,pe3,pd3,phi,theta,psi,...
%                                                [],'normal');
        commanded_position_handle = drawCommandedPosition(x_c,y_c,z_c,yaw_c,...
                                               []);
        title('Spacecraft')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        view(32,47)  % set the vieew angle for figure
%         center = [(pn1+pn2+pn3)/3;
%                   (pe1+pe2+pe3)/3;
%                   (pd1+pd2+pd3)/3];
        center = [pn1;pe1;pd1];
        %axis([center(1)-view_range,center(1)+view_range,
        %      center(2)-view_range,center(2)+view_range,
        %      center(3)-view_range,center(3)+view_range]);
        axis([center(1)-view_range,center(1)+view_range, ...
              center(2)-view_range,center(2)+view_range, ...
              center(3)-view_range,center(3)+view_range]);
        hold on
        
    % at every other time step, redraw base and rod
    else
        
        %figure(1);
%         pos = [(pn1+pn2+pn3)/3;
%                (pe1+pe2+pe3)/3;
%                (pd1+pd2+pd3)/3];
        pos = [pn1;pe1;pd1];
        close_enough_tolerance*[view_range;view_range;view_range] - abs(pos - center);
        if (min(close_enough_tolerance*[view_range;view_range;view_range] - abs(pos-center)) < 0)
            center = pos;
        end
        
        axis([center(1)-view_range,center(1)+view_range, ...
              center(2)-view_range,center(2)+view_range, ...
              center(3)-view_range,center(3)+view_range]);
        drawSpacecraftBody(P.x_quad, P.y_quad, P.z_quad, P.c_quad_red,...
                           pn1,pe1,pd1,phi,theta,psi,...
                           spacecraft1_handle);
%         drawSpacecraftBody(P.x_quad, P.y_quad, P.z_quad, P.c_quad_green,...
%                            pn2,pe2,pd2,phi,theta,psi,...
%                            spacecraft2_handle);
%         drawSpacecraftBody(P.x_quad, P.y_quad, P.z_quad, P.c_quad_blue,...
%                            pn3,pe3,pd3,phi,theta,psi,...
%                            spacecraft3_handle);
        drawCommandedPosition(x_c,y_c,z_c,yaw_c,...
                           commanded_position_handle);
    end
end

  
%=======================================================================
% drawSpacecraft
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawSpacecraftBody(X,Y,Z,C,...
                                     pn,pe,pd,phi,theta,psi,...
                                     handle,mode)
  length = size(X(1,:));
  X = [X(1,:),X(2,:),X(3,:)];
  Y = [Y(1,:),Y(2,:),Y(3,:)];
  Z = [Z(1,:),Z(2,:),Z(3,:)];
  
  V = [X;Y;Z];
  V = rotate(V, phi, theta, psi)';  % rotate spacecraft
  V = translate(V', pn, pe, pd)';  % translate spacecraft
  % transform vertices from NED to XYZ (for matlab rendering)
  R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
  V = V*R;
  
  length = length(2);
  X = zeros(3,length);
  Y = zeros(3,length);
  Z = zeros(3,length);
  
  for i = 1:length
      X(:,i) = [V(i,1);V(i+length,1);V(i+2*length,1)];
      Y(:,i) = [V(i,2);V(i+length,2);V(i+2*length,2)];
      Z(:,i) = [V(i,3);V(i+length,3);V(i+2*length,3)];
  end
  
  if isempty(handle),
  handle = patch(X,Y,Z,C);
  else
    set(handle,'XData', X, 'YData', Y, 'ZData', Z);
    drawnow
  end
end

function handle = drawCommandedPosition(x_c, y_c, z_c, yaw_c,...
                                     handle)
  V = translate([0; 0; 0], x_c, y_c, z_c)';  % translate spacecraft
  % transform vertices from NED to XYZ (for matlab rendering)
  R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
  V = V*R;
  
  if isempty(handle),
  handle = plot3(V(1),V(2),V(3),'*','markersize',10);
  else
    set(handle,'xdata',V(1),'ydata',V(2),'zdata',V(3));
    drawnow
  end
end

%%%%%%%%%%%%%%%%%%%%%%%
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function XYZ = translate(XYZ,pn,pe,pd)
  XYZ = XYZ + repmat([pn;pe;pd],1,size(XYZ,2));
end

  