function writePly(fileName, points, colors)
%
% Write colored 3D point cloud to PLY file
%
% fileName - ply file name withouth extension
% points - point coordinates, N by 3 matrix
% colors - colors as uint8, N by 3 matrix
%
fileID = fopen(strcat(fileName,'.ply'), 'w');
fprintf(fileID,'ply\n');
fprintf(fileID,'format ascii 1.0\n');
fprintf(fileID,'element vertex %u\n', length(points));
fprintf(fileID,'property float x\n');
fprintf(fileID,'property float y\n');
fprintf(fileID,'property float z\n');
fprintf(fileID,'property uchar red\n');
fprintf(fileID,'property uchar green\n');
fprintf(fileID,'property uchar blue\n');
fprintf(fileID,'end_header\n');
data = [points colors]';
fprintf(fileID,'%3.7f %3.7f %3.7f %3u %3u %3u\n', data);
fclose(fileID);
end