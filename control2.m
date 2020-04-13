function [wdes,u_law] = control2(u,L,udes)
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