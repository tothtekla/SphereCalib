function plotLidar(rot, trans, scale)
%
% Plot schematic LiDAR as a cylinder and axes
%
% Extrinsic calibration parameters
% - rot -- rotation matrix, 3 by 3
% - trans -- translation vector 
% Other
% - scale -- scaling of the plot
%
u = 0:0.05:1;
u = 2 * pi * u;
N = length(u);

circle1 = [0.5*cos(u);  0.5*sin(u); -0.25*ones(1, N)];
circle2 = [0.5*cos(u);  0.5*sin(u);  0.25*ones(1, N)];
points = eye(3);
      
circle1 =(rot * scale * circle1)' + trans;
circle2 =(rot * scale * circle2)' + trans;
points =(rot * scale * points)' + trans;
  
%Plot camera frustrum
for i = 1 :(N - 1)
    plot3([circle1(i,1) circle1(i+1,1)], [circle1(i,2) circle1(i+1,2)], [circle1(i,3) circle1(i+1,3)], 'k');  
    plot3([circle2(i,1) circle2(i+1,1)], [circle2(i,2) circle2(i+1,2)], [circle2(i,3) circle2(i+1,3)], 'k');    
end

%Plot camea axes
plot3([trans(1) points(1,1)], [trans(2) points(1,2)], [trans(3) points(1,3)], 'r');
plot3([trans(1) points(2,1)], [trans(2) points(2,2)], [trans(3) points(2,3)], 'g');
plot3([trans(1) points(3,1)], [trans(2) points(3,2)], [trans(3) points(3,3)], 'b');
end