classdef FileReaderWriter
    % Read & write image and point cloud data
    
    properties
        imgDir char
        cloudDir char
        idxs (:, 1) double
        
        imgNameStart char
        imgNameEnd char
        cloudNameStart char
        cloudNameEnd char     
    end
    
    properties (Dependent)
        imgNames
        cloudNames
    end
    
    methods    
        function imgNames = get.imgNames(obj)
            imgNames = cell(length(obj.idxs), 1);
            for i = 1:length(obj.idxs)
                imgNames{i} = strcat(obj.imgNameStart, num2str(obj.idxs(i)), obj.imgNameEnd);
            end
        end
        function cloudNames = get.cloudNames(obj)
            cloudNames = cell(length(obj.idxs), 1);
            for i = 1:length(obj.idxs)
                cloudNames{i} = strcat(obj.cloudNameStart, num2str(obj.idxs(i)), obj.cloudNameEnd);
            end
        end
        
        %TODO: OBJ1 OBJ2 FOLRERS EXIST?
        function out =  FileReaderWriter(obj1, obj2, isReader,...
                                         imgNameStart, imgNameEnd,...
                                         cloudNameStart, cloudNameEnd)
            arguments
                obj1 = 'C:\Users\totht\Downloads\OneDrive_1_1-25-2022\parkolo_gomb_cal_4FPS_pictures'; %FileReaderWriter or imgDir 
                obj2 char = 'C:\Users\totht\Downloads\OneDrive_1_1-25-2022\parkolo_gomb_cal_4FPS_XYZ';               %cloudDir
                isReader (1,1) logical = 1
                imgNameStart char = 'Dev0_Image_w960_h600_fn'
                imgNameEnd char = '.jpg'
                cloudNameStart char = 'test_fn'
                cloudNameEnd char = '.xyz'
            end
            if nargin == 1 && isa(obj1,'FileReaderWriter')
                out.imgDir = obj1.imgDir;
                out.cloudDir = obj1.cloudDir;
                out.idxs = obj1.idxs;
                out.imgNameStart = obj1.imgNameStart; 
                out.imgNameEnd = obj1.imgNameEnd;
                out.cloudNameStart = obj1.cloudNameStart;
                out.cloudNameEnd = obj1.cloudNameEnd;
            elseif ischar(obj1) && ischar(obj2) && isReader % DEV MÓDBAN CSAK! -> nargin >=2 && 
                out.imgDir = obj1;
                out.cloudDir = obj2;                
                out.imgNameStart = imgNameStart; 
                out.imgNameEnd = imgNameEnd;
                out.cloudNameStart = cloudNameStart;
                out.cloudNameEnd = cloudNameEnd;
                imgIdxs = FileReaderWriter.validateFileIdxs(obj1, imgNameStart, imgNameEnd);
                cloudIdxs = FileReaderWriter.validateFileIdxs(obj2, cloudNameStart, cloudNameEnd);
                out.idxs = cloudIdxs(ismember(cloudIdxs,imgIdxs));
                if length(out.idxs) < 3
                    error('Not enough input files names exist.');
                end
            end
        end
        
        function [imgs, clouds] = readFiles(obj, idxs)
            arguments
                obj FileReaderWriter
                idxs (:, 1) double = obj.idxs
            end
            imgs = readImgs(obj, idxs);
            clouds = readClouds(obj, idxs);
        end
        
        function imgs = readImgs(obj, idxs)
            arguments
                obj FileReaderWriter
                idxs (:, 1) double = obj.idxs
            end
            N = length(idxs);
            imgs = cell(N, 1);
            f = waitbar(0,'Reading images');
            for i = 1:N
                waitText = strcat({'Reading images: '}, num2str(i), '/', num2str(N));
                waitbar(i/N, f, waitText{1});
                imgFile = strcat(obj.imgDir,'\',obj.imgNames(idxs(i)));
                if ~isfile(imgFile)
                    errorMessage = strcat({'File does not exist: '}, num2str(idxs(i)));
                    error(errorMessage{1});
                end
                imgs{i} = imread(char(imgFile));
            end       
            waitbar(1,f,'Image reading finished!');
            imgs=imgs(~cellfun(@isempty,imgs));
            close(f);
        end
        
        function clouds = readClouds(obj, idxs)
            arguments
                obj FileReaderWriter
                idxs (:, 1) double = obj.idxs
            end
            N = length(idxs);
            clouds = cell(N, 1); 
            f = waitbar(0,'Reading point clouds');
            for i = 1 : N
                waitText = strcat({'Reading point clouds: '}, num2str(i), '/', num2str(N));
                waitbar(i/N, f, waitText{1});
                cloudFile = strcat(obj.cloudDir,'\',obj.cloudNames(idxs(i)));
                if ~isfile(cloudFile)
                    errorMessage = strcat({'File does not exist: '}, num2str(idxs(i)));
                    error(errorMessage{1});
                end
                clouds{i} = importdata(char(cloudFile));
                clouds{i} = clouds{i}(:, 1:3); 
            end
            waitbar(1,f,'Point cloud reading finished!');
            clouds=clouds(~cellfun(@isempty,clouds)); 
            close(f);
        end
    end
    
    methods (Static)
        function fileIdxs = validateFileIdxs(fileDir, fileNameStart, fileNameEnd) 
            fileNames=dir(fileDir);
            fileNameMinLength = length(fileNameStart) + length(fileNameEnd);

            fileN = size(fileNames, 1);
            fileIdxs = zeros(fileN, 1);

            for i = 1 : fileN
               if length(fileNames(i).name) > fileNameMinLength &&...
                       all(fileNames(i).name(1:length(fileNameStart)) == fileNameStart) &&...
                       all(fileNames(i).name(end-length(fileNameEnd)+1 : end) == fileNameEnd)
                   if fileNames(i).bytes == 0
                       warningMessage = strcat({'Empty file: '}, fileNames(i).name);
                       warning(warningMessage{1});
                   end
                   fileIdxs(i) = str2double(fileNames(i).name(length(fileNameStart)+1 : end-length(fileNameEnd)));
               else
                   warningMessage = strcat({'No file loaded: '}, fileNames(i).name);
                   warning(warningMessage{1});
               end
            end
            fileIdxs = sort(nonzeros(fileIdxs));
        end
    end
end

