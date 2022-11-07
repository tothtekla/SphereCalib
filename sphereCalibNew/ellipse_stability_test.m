function ellipse_stability_test

Vx0 = 0:0.02:1
Vy0 = 0:0.02:1
Vz = 0.5:0.1:5
Vr = 0.05:0.02:0.25

f = 800;%2000;%1050;
u0 = 960;
v0 = 600;
fu = f;
fv = f;
intrinsic = cameraIntrinsics([fu fv], [u0 v0], [2*u0, 2*v0]);
numInliers = 1000;

i = 1;
for x0 = 0:0.02:1
    y0 = x0;
%for y0 = 0:0.02:1
for z0 = 0.5:0.1:5
for r = 0.05:0.02:0.25 
    s = SphereConverter([x0, y0, z0, r]);
    pE = sphereO2parametricEllipseO(s);
    inliers_m = generateEllipsePoints(pE, numInliers);
    pts = PointSet2D(inliers_m);

    sphereSun = fitSphereSun16(pts, r);
    sphereDir = fitSphere(pts, r);
    pE_Shi19 = fitEllipseShi19B(pts, r);
    sphereShi = parametricEllipseO2SphereO(pE_Shi19, r);
    %sphereShi = fitSphereShi19B(pts, r);

    eDir(i) = sphereError([x0 y0 z0], sphereDir.s0);
    eSun(i) = sphereError([x0 y0 z0], sphereSun.s0);
    eShi(i) = sphereError([x0 y0 z0], sphereShi.s0);
    i = i + 1;
end
end
end
%end

disp('done') ;
figure;
plot()



end

function e = sphereError(s1, s2)
    e = norm(s1-s2);
end