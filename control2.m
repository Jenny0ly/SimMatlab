function [wdes,u_law] = control2(A,L,udes)
    %H matrix weights
    wF  = 0.3;wMxy = 0.4;wMz = 0.2;
    wF1 = wF;wMx1 = wMxy;wMy1 = wMxy;wMz1 = wMz;
    wF2 = wF;wMx2 = wMxy;wMy2 = wMxy;wMz2 = wMz;
    v = [sqrt(wF1) sqrt(wMx1) sqrt(wMy1) sqrt(wMz1)...
         sqrt(wF2) sqrt(wMx2) sqrt(wMy2) sqrt(wMz2)];
    H = diag(v);
    %matrix (8) in mellinger's paper
    mA = A*H^-2*A';
    %input
    u = H^-2*A'*inv(mA);
    
    %optimal input
    u_law = u*udes;
    %with u_law calculate now wdes para implementar en el sistema
    wdes = desvelocities(u_law,L);
end 

function [wdes] = desvelocities(u_law,L)
global n kf km
wdes = [];
    kf = 6.11e-8;km = 1.5e-9;
    Aa = [kf kf kf kf;0 kf*L 0 -kf*L;-kf*L 0 kf*L 0;km -km km -km];
     for i=1:n
         %des velocity to the power of 2
         b = u_law(4*i-3:4*i);
         wdesi = inv(Aa)*b;
         for j=1:4
            wdesi(j) = sqrt(wdesi(j)); 
         end
        wdes = [wdes;wdesi];
     end
end