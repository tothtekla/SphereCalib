function C = CatmullRomSpline(p0, p1, p2, p3, tCR, alpha)
t0 = 0;
t1 = GetT( t0, alpha, p0, p1 );
t2 = GetT( t1, alpha, p1, p2 );
t3 = GetT( t2, alpha, p2, p3 );
t = (1-tCR)*t1 + tCR*t2;

A1 = ( t1-t )/( t1-t0 )*p0 + ( t-t0 )/( t1-t0 )*p1;
A2 = ( t2-t )/( t2-t1 )*p1 + ( t-t1 )/( t2-t1 )*p2;
A3 = ( t3-t )/( t3-t2 )*p2 + ( t-t2 )/( t3-t2 )*p3;
B1 = ( t2-t )/( t2-t0 )*A1 + ( t-t0 )/( t2-t0 )*A2;
B2 = ( t3-t )/( t3-t1 )*A2 + ( t-t1 )/( t3-t1 )*A3;
C  = ( t2-t )/( t2-t1 )*B1 + ( t-t1 )/( t2-t1 )*B2;
end

function t1 = GetT(t0, alpha, p0, p1)
distVec = p1 - p0;
dist = sqrt(sum(distVec.*distVec));
t1 = dist^alpha  + t0;
end