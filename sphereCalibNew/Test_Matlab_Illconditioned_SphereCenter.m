function Test_Matlab_Illconditioned_SphereCenter
    %% Read files
    load('ellipse_full_contour00_045_015_075_025_800_960_600.mat','rndN_m', 'rndN_pix');
    rndN_m00 = rndN_m;
    rndN_pix00 = rndN_pix;
    load('ellipse_full_contour01_045_015_075_025_800_960_600.mat','rndN_m', 'rndN_pix');
    rndN_m01 = rndN_m;
    rndN_pix01 = rndN_pix;
    load('ellipse_full_contour02_045_015_075_025_800_960_600.mat','rndN_m', 'rndN_pix');
    rndN_m02 = rndN_m;
    rndN_pix02 = rndN_pix;
    load('ellipse_full_contour05_045_015_075_025_800_960_600.mat','rndN_m', 'rndN_pix');
    rndN_m05 = rndN_m;
    rndN_pix05 = rndN_pix;
    load('ellipse_full_contour10_045_015_075_025_800_960_600.mat','rndN_m', 'rndN_pix');
    rndN_m10 = rndN_m;
    rndN_pix10 = rndN_pix;

    fu = 800;
    u0 = 960;
    fv = 800;
    v0 = 600;

    sphere_GT = SphereConverter([0.45 0.15 0.75 0.25]);
    s0_GT = [0.45 0.15 0.75];
    testCase = 1;
    pointSet = PointSet2D.empty(0,5);
    pointSet(1) = PointSet2D(rndN_m00);
    pointSet(2) = PointSet2D(rndN_m01);
    pointSet(3) = PointSet2D(rndN_m02);
    pointSet(4) = PointSet2D(rndN_m05);
    pointSet(5) = PointSet2D(rndN_m10);
    
    r = 0.25;
    %% Tests
    
    spheresDirect3pFit = SphereConverter.empty(0,pointSet(1).N-2);
    spheresSun16 =  SphereConverter.empty(0,pointSet(1).N-2);
    spheresShi19 =  SphereConverter.empty(0,pointSet(1).N-2);
    

    disp('.............................................');
    %%{
    %time3p = 0;
    %for t = 1:1000
        tStart  = tic;
        for i = 1:testCase
            for j = 3 : pointSet(i).N
                spheresDirect3pFit(i, j-2) = fitSphere(pointSet(i), r, 1:j);
            end
        end 
        tEndDirect3pFit  = toc(tStart);
    %    time3p = time3p + tEndDirect3pFit;
    %end
    %time3p =  time3p/ 1000;
    %}
    disp('.............................................');
    time3p = 0;
    %for t = 1:1000
    %    tStart  = tic;
        for i = 1:testCase
            for j = 3 : pointSet(i).N
                spheresSun16(i,j-2) = fitSphereSun16(pointSet(i), r, 1:j);
            end
        end
        tEndSun16  = toc(tStart);
    %    time3p = time3p + tEndSun16;
    %end
    %time3p =  time3p/ 1000;
    disp('.............................................');
    tStart  = tic;
    for i = 1:testCase
        for j = 3 : pointSet(i).N
            spheresShi19(i,j-2) = fitSphereShi19C(pointSet(i), r, 1:j);
        end
    end
    tEndShi19  = toc(tStart);
    %% Error estimation
    s0Direct3pFit = zeros(testCase, pointSet(1).N-2, 3);
    s0Sun16 = zeros(testCase, pointSet(1).N-2, 3);
    s0Shi19 = zeros(testCase, pointSet(1).N-2, 3);
    for i = 1: testCase
        for j = 1 : pointSet(i).N -2
            s0Direct3pFit(i, j, :) = spheresDirect3pFit(i, j).s0;
            s0Sun16(i, j, :) = spheresSun16(i, j).s0;
            s0Shi19(i, j, :) = spheresShi19(i, j).s0;
        end
    end
    
    eDirect3pFit = zeros(testCase, pointSet(1).N-2);
    eSun16 = zeros(testCase, pointSet(1).N-2);
    eShi19 = zeros(testCase, pointSet(1).N-2);
    
    for i = 1: testCase
        for j = 1 : pointSet(i).N-2
            tmp = [s0Direct3pFit(i, j, 1) s0Direct3pFit(i, j, 2) s0Direct3pFit(i, j, 3)];
            eDirect3pFit(i,j) = norm(tmp - sphere_GT.s0);
            tmp = [s0Sun16(i, j, 1) s0Sun16(i, j, 2) s0Sun16(i, j, 3)];
            eSun16(i,j) = norm(tmp - sphere_GT.s0);
            tmp = [s0Shi19(i, j, 1) s0Shi19(i, j, 2) s0Shi19(i, j, 3)];
            eShi19(i,j) = norm(tmp - sphere_GT.s0);
        end
    end
    
    %%{
    mse1 = zeros(testCase,1);
    mse2 = zeros(testCase,1);
    mse3 = zeros(testCase,1);
    rmse1 = zeros(testCase,1);
    rmse2 = zeros(testCase,1);
    rmse3 = zeros(testCase,1);
    me1 = zeros(testCase,1);
    me2 = zeros(testCase,1);
    me3 = zeros(testCase,1);
    std1 = zeros(testCase,1);
    std2 = zeros(testCase,1);
    std3 = zeros(testCase,1);
    stde1 = zeros(testCase,1);
    stde2 = zeros(testCase,1);
    stde3 = zeros(testCase,1);
    
    

    for i = 1:testCase
            sqErrVec1 = eDirect3pFit(i, :).^2;            % Squared Error
            sqErrVec1 = sqErrVec1(~isnan(sqErrVec1));
            mse1(i) = mean(sqErrVec1);            % Mean Squared Error
            rmse1(i) = sqrt(mse1(i));                % Root Mean Squared Error
            std1(i) = std(sqErrVec1);             % Spread of SE
            eDirect3pFit2 = eDirect3pFit(i, ~isnan(eDirect3pFit(i, :)));
            me1(i) = mean(eDirect3pFit2);     % Mean err.
            stde1(i) = std(eDirect3pFit2);    % Spread of ME
        
            sqErrVec2 = eSun16(i, :).^2;
            sqErrVec2 = sqErrVec2(~isnan(sqErrVec2));
            mse2(i) = mean(sqErrVec2);
            rmse2(i) = sqrt(mse2(i));
            std2(i) = std(sqErrVec2);
            eSun162 = eSun16(i, ~isnan(eSun16(i, :)));
            me2(i) = mean(eSun162);     % Mean err.
            stde2(i) = std(eSun162);    % Spread of ME
        
            sqErrVec3 = eShi19(i, :).^2;
            sqErrVec3 = sqErrVec3(~isnan(sqErrVec3));
            mse3(i) = mean(sqErrVec3);
            rmse3(i) = sqrt(mse3(i));
            std3(i) = std(sqErrVec3);
            eShi192 = eShi19(i, ~isnan(eShi19(i, :)));
            me3(i) = mean(eShi192);     % Mean err.
            stde3(i) = std(eShi192);    % Spread of ME
    end
    
    table = [me1(1) stde1(1) rmse1(1);
             me2(1) stde2(1) rmse2(1);
             me3(1) stde3(1) rmse3(1)] 

    %}
    %% Plots
    figure;
    hold on;
    plot(1:1894, eDirect3pFit(1, :));
    plot(1:1894, eSun16(1, :));
    plot(1:1894, eShi19(1, :));

    figure;
    hold on;
    plot(1:1894, eDirect3pFit(5, :));
    plot(1:1894, eSun16(5, :));
    plot(1:1894, eShi19(5, :));

    save('illcond_plot.mat', 'eDirect3pFit', 'eSun16', 'eShi19');

    save('illconditioned_S0_e.mat','s0Direct3pFit', 's0Sun16', 's0Shi19',...
        'eDirect3pFit', 'eSun16', 'eShi19');
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
