function [transErr, rotErr] = estimateTransformError2(transEst, rotEst, distGT, rotGT)
%
% Estimate the error of extrinsic parameters after pointset registration
%
% transEst, rotEst : estimated translation vector and 3 by 3 rotation matrix
% distGT, rotGT :  ground truth distance of sensors and 3 by 3 rotation matrix
% transError : translation error - norm of the Euclidean distance of the
%              vectors in meters
% rotErr : rotation error - angular error in degrees
%
transErr = abs(distGT - norm(transEst));
rotErr = acos((trace(rotGT'*rotEst)-1)/2)*180/pi;
end