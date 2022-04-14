%%{
f = FileReaderWriter();
[imgs, clouds] = readFiles(f, 5:10);

fu = 625;
u0 = 480;
fv = 625;
v0 = 300;
imgData = ImageData(imgs, f.imgNames, [fu fv], [u0 v0]);

[pointSet, pointSetMin] = generateEdges(imgData, 1);

rMinCam = 10/imgData.intrinsic.FocalLength(1);
rMaxCam = imgData.intrinsic.PrincipalPoint(1)/(2*imgData.intrinsic.FocalLength(1));
ransacThresholdCam = 4/imgData.intrinsic.FocalLength(1);

[pointSetInliers, S0] = detectSphere(pointSet,pointSetMin);
%}

%{
for i = 1:length(pointSetInliers)
    inliers = true(imgData.intrinsic.ImageSize);
    for j = 1:length(pointSetInliers(i).XY)
        inliers(pointSetInliers(i).XY(j, 2), pointSetInliers(i).XY(j, 1)) = false;
    end
    figure;
    imshow(inliers);
end
%}

%{
%input no. of sensors 1, 3

imgNames=dir('C:\Users\totht\Downloads\OneDrive_1_1-25-2022\ELTE-logo_vissza_4FPS_pictures');
cloudNames=dir('C:\Users\totht\Downloads\OneDrive_1_1-25-2022\ELTE-logo_vissza_4FPS_XYZ');
cloudNameStart = 'test_fn';
cloudNameEnd = '.xyz';


cloudNameMinLength = length(cloudNameStart) + length(cloudNameEnd);

cloudN = size(cloudNames, 1);
cloudIdxs = zeros(cloudN, 1);

for i = 1 : cloudN
   if length(cloudNames(i).name) > cloudNameMinLength &&...
           all(cloudNames(i).name(1:length(cloudNameStart)) == cloudNameStart) &&...
           all(cloudNames(i).name(end-length(cloudNameEnd)+1 : end) == cloudNameEnd)
       if cloudNames(i).bytes == 0
           warningMessage = strcat({'Empty file: '}, cloudNames(i).name);
           warning(warningMessage{1});
       end
       cloudIdxs(i) = str2double(cloudNames(i).name(length(cloudNameStart)+1 : end-length(cloudNameEnd)));
   else
       warningMessage = strcat({'No file loaded: '}, cloudNames(i).name);
       warning(warningMessage{1});
   end
end



cloudIdxs2 = sort(nonzeros(cloudIdxs));
%cloud = 'test_fn*.xyz';
%img = 'Dev*_Image_w960_h600_fn*';
%}
