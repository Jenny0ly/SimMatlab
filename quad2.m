function [Xr,wRq] = quad2(Q,B,Euler)
global n 
Xr = zeros(3,n);
for i=1:n
    Xr(:,i) = Q(:,i)-B';
    sentence = ['Relative position drone',num2str(i),' is: ' num2str(Xr(:,i)')];
    disp(sentence)
end
    %rotor 1 is (+) xqi and rotor 2 is (+) yqi
    %rotor 3 is (-) xqi and rotor 4 is (-) yqi
    %zb and zqi are parallel
    %% Euler angles (roll, pitch, yaw) quad1
    %degrees
    phi = Euler(1);theta = Euler(2);psi = Euler(3);
    cx = cosd(phi);sx = sind(phi);
    cy = cosd(theta);sy = sind(theta);
    cz = cosd(psi);sz = sind(psi);
    %from W to Q1
    rx = [1 0 0;0 cx -sx;0 sx cx];
    ry = [cy 0 sy;0 1 0;-sy 0 cy];
    rz = [cz -sz 0;sz cz 0;0 0 1];
    %Rotation matrix from W to Q1
    wRq = rz*rx*ry;
end