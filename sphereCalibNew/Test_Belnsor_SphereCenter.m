function Test_Belnsor_SphereCenter
    %% Read files
    numFiles = 5; %1040;
    fu = 800;
    u0 = 960;
    fv = 800;
    v0 = 540;
    
    %%{
    [imgs, imgNames, s0_GT]  = readBlensorTestData(1, numFiles);
        
    imgData = ImageData(imgs, imgNames, [fu fv], [u0 v0]);
    tic;
    edgeThreshold = 0.3;
    imgEdge = cell(numFiles, 1);
    for i = 1:numFiles
        imgGray = rgb2gray(imgData.imgs{i});
        imgEdge{i} = edge(imgGray,'canny', edgeThreshold);
        [A, B] = find(imgEdge{i});    
    end
    toc;
    %}

    [pointSet, ~] = generateEdges(imgData, 1);
    load('blensor_sph_S0_e2.mat', 'pointSet');
    s0_GT = gets0_GT(numFiles);
    
    r = 0.25;
    %% Tests
    
    spheresDirect3pFit = SphereConverter.empty(0,numFiles);
    spheresSun16 =  SphereConverter.empty(0,numFiles);
    spheresShi19 =  SphereConverter.empty(0,numFiles);
    

    disp('.............................................');
    tStart  = tic;
    for i = 1:numFiles
        spheresDirect3pFit(i) = fitSphere(pointSet(i), r);
    end 
    tEndDirect3pFit  = toc(tStart);
    disp('.............................................');
    tStart  = tic;
    for i = 1:numFiles
        spheresSun16(i) = fitSphereSun16(pointSet(i), r);
    end
    tEndSun16  = toc(tStart);
    disp('.............................................');
    tStart  = tic;
    for i = 1:numFiles
        spheresShi19(i) = fitSphereShi19C(pointSet(i), r);
    end
    tEndShi19  = toc(tStart);
    %% Error estimation
    s0Direct3pFit = zeros(numFiles, 3);
    s0Sun16 = zeros(numFiles, 3);
    s0Shi19 = zeros(numFiles, 3);
    for i = 1: numFiles
        s0Direct3pFit(i, :) = spheresDirect3pFit(i).s0;
        s0Sun16(i, :) = spheresSun16(i).s0;
        s0Shi19(i, :) = spheresShi19(i).s0;
    end
    
    eDirect3pFit = s0Direct3pFit - s0_GT(1:numFiles, 1:3);
    eSun16 = s0Sun16 - s0_GT(1:numFiles, 1:3);
    eShi19 = s0Shi19 - s0_GT(1:numFiles, 1:3);
    
    errVec1 = vecnorm(eDirect3pFit');  % Errors
    sqErrVec1 = errVec1.^2;            % Squared Error
    mse1 = mean(sqErrVec1);            % Mean Squared Error
    rmse1 = sqrt(mse1);                % Root Mean Squared Error

    errVec2 = vecnorm(eSun16');
    sqErrVec2 = errVec2.^2;
    mse2 = mean(sqErrVec2);
    rmse2 = sqrt(mse2);

    errVec3 = vecnorm(eShi19');
    sqErrVec3 = errVec3.^2;
    mse3 = mean(sqErrVec3);
    rmse3 = sqrt(mse3);

    %% Plots
    save('blensor_sph_S0_e.mat','s0Direct3pFit', 's0Sun16', 's0Shi19',...
        'eDirect3pFit', 'eSun16', 'eShi19', 'mse1','mse2' , 'mse3', 'rmse1', 'rmse2', 'rmse3', 'pointSet' );

end

function [imgs, sphImageNames, s0_GT]  = readBlensorTestData(from, to)
    numFiles = 1040;%133;%133; %85 %25;
    numImgs = numFiles;
    sphImagePath = 'C:\Users\totht\ELTE\Data\Blender_Sphere_Tests_IJCV\';
    sphImageNames = cell(numFiles, 1);
    s0_GT = zeros(numFiles, 3);
    f = 1;
    for k = 15 : 40
        z = k / 20;
        l = k - 15;
        step = 1 / 20 + l*0.0065;
        for i = 0 : 9
            x = i * step;
            for j = 0 : 3
                y = j * step;
                num = 5 * 41 * i + 41 * j + l + 1;
                sphImageNames{f} = strcat('(', num2str(num), ').png');
                s0_GT(f, :) = [x y z];
                f = f + 1;
            end
        end
    end
    imgs = readImgs(sphImagePath, sphImageNames(from:to));
end

function imgs = readImgs(imgDir, imgNames)
    %
    % Read image
    % 
    % INPUT
    % Source file infos
    % - imgDir, imgNames -- image path and file names
    %
    %
    imgs = cell(length(imgNames), 1);
    
    for i= 1:length(imgNames)
        imgFile = strcat(imgDir,imgNames(i));
        imgs{i} = imread(char(imgFile));
    end
end

function s0_GT = gets0_GT(numFiles)
    s0_GT = zeros(numFiles, 3);
    f = 1;
    for k = 15 : 40
        z = k / 20;
        l = k - 15;
        step = 1 / 20 + l*0.0065;
        for i = 0 : 9
            x = i * step;
            for j = 0 : 3
                y = j * step;
                num = 5 * 41 * i + 41 * j + l + 1;
                sphImageNames{f} = strcat('(', num2str(num), ').png');
                s0_GT(f, :) = [x y z];
                f = f + 1;
            end
        end
    end
end
