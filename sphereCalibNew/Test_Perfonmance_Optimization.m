function Test_Perfonmance_Optimization

%{
1. No ellipse estimation to sphere fitting from points						
	i) 3pFit+Ell2Sphere vs Direct3pFit		
2. Estimating end-points of minor axis end-points instead of the major one  
3. Extra rotation matrix calculation
4. Measureing alpha directly 
	i)  3pFit
	ii) Direct3pFit
	ii) Ell2Sphere
%}

% 1-> fitSphereB -> fitEllipse+Ell2Sphere

% 2-> fitEllipseB -> minor axis e. p. 

% 3-> fitEllipseC, fitSphereC -> extra R

% 4-> fitEllipseD, fitSphereD, ell2SphereD -> alpha 


%Sphere parameters
x0 =3
y0 = 2
z0 = 4
r =0.1
numInliers = 200;%350 pöttyös
W = [x0^2 x0*y0 x0*z0 y0^2 y0*z0 z0^2]
%Camera parameters
f = 2000;%1050;
u0 = 2000;
v0 = 2000;
fu = f;
fv = f;
intrinsic = cameraIntrinsics([fu fv], [u0 v0], [2*u0, 2*v0]);
%Inlier parameters
rndNum = 25%350 pöttyös
precision_pix = 0;

%Generate rndNum inlier point
s = SphereConverter([x0, y0, z0, r]);
pE = sphereO2parametricEllipseO(s);
inliers_m = generateEllipsePoints(pE, numInliers);
%outliers_m = [-0.065 -0.011] + [0:0.001:0.01; zeros(11, 1)']';
%inliers_m = [inliers_m; outliers_m]; 
%{
if precision_pix == 0
    inliers_pix = round(ImageData.meter2pixel(inliers_m, intrinsic));
else
    inliers_pix = round(ImageData.meter2pixel(inliers_m, intrinsic) / precision_pix) * precision_pix;
end
inliers_pix_min = unique(inliers_pix,'rows');
idxs = randsample(length(inliers_m), rndNum);
rndN_pix = inliers_pix(idxs(:),:);
rndN_m = ImageData.pixel2meter(rndN_pix, intrinsic);
%}
rndN_m = inliers_m ;

pts = PointSet2D(rndN_m);
sphere = fitSphereA(pts, r);
sphere2 = fitSphere(pts, r);
