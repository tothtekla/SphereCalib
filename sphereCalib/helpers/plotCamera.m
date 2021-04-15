function plotCamera( rot, trans, fu, fv, u0, v0, scale)
%
% Plot camera frustum and axes
%
% Extrinsic calibration parameters
% - rot -- rotation matrix, 3 by 3
% - trans -- translation vector 
% Camera instrinsics
% - fu, fv -- focal length with pixel scale
% - u0, v0 -- principal point
% Other
% scaling of the plot
%
points = [0 0 0;
          u0/fu, v0/fv, 1;
          u0/fu, -v0/fv, 1;
          -u0/fu, -v0/fv, 1;
          -u0/fu, v0/fv, 1;
          eye(3)];
      
points =(rot * scale * points')' + trans;

P_start = [points(1, :); points(1, :); points(1, :); points(1, :); ...
           points(2, :); points(3, :); points(4, :); points(5, :)];
P_end = [points(2, :); points(3, :); points(4, :); points(5, :);...
         points(3, :); points(4, :); points(5, :); points(2, :)];
  
%Plot camera frustrum
for i = 1:8
    plot3([P_start(i,1) P_end(i,1)], [P_start(i,2) P_end(i,2)], [P_start(i,3) P_end(i,3)], 'k');     
end

%Plot camea axes
plot3([trans(1) points(6,1)], [trans(2) points(6,2)], [trans(3) points(6,3)], 'r');
plot3([trans(1) points(7,1)], [trans(2) points(7,2)], [trans(3) points(7,3)], 'g');
plot3([trans(1) points(8,1)], [trans(2) points(8,2)], [trans(3) points(8,3)], 'b');
end