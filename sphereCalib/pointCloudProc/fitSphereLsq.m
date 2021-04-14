function [S0, r] = fitSphereLsq(points)
%
% Sphere fitting with an iterative least-squares method 
% based on David Eberly, "Least Squares Fitting of Data"
%
% points: input coordinates, N by 3 matrix
% S0: estimated sphere center
% r: estimated sphere radius
%
iterations = 1000; % Maximal no. of fixpoint iterations
meanPts = mean(points);
numPts = length(points);
points = points - meanPts;

numS0Ini = floor(numPts / 4);
S0Ini = zeros(numS0Ini, 3);
idxs = randperm(length(points));
for i = 1 : numS0Ini
    j = (i - 1) * 4  + 1;
    rnd4 = points(idxs(j:(j+3)),:);
    [S0Ini(i, :), ~] =  fitSphere4p(rnd4);
end
S0 = median(S0Ini);

for j = 1 : iterations
    r_avg = 0;
    dir_avg = [0 0 0];
    S0_tmp = S0; 
    for i = 1 : numPts
        dir_i = S0 - points(i,1:3);
        r_i = sqrt(dot(dir_i,dir_i));
        r_avg = r_avg + r_i;
        dir_avg = dir_avg + ( (S0 - points(i,:)) / r_i);
    end

    S0 = (r_avg * dir_avg) / numPts^2; 
    r = r_avg / numPts;
    diff_new = S0 - S0_tmp; 
    if(eq(diff_new,[0 0 0]))
        break;
    end
end
S0 = S0 + meanPts;
end
