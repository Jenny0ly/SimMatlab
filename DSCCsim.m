clear all
close all
clc
%% Coordinate Frames 
%W      World frame    (xw,yw,zw) ENU (East, North, Upward)
xw = 0;yw = 0;zw = 0;
W = [xw;yw;zw];
%B      Body frame     (xb,yb,zb) coordinates of CoM of payload
h = 0.7;
xb = 3;yb = 3;zb = h/2;
B = [xb,yb,zb];
%Qi     Quad frame     (xqi, yqi,zqi) z is Upward
xq1 = 2.2;yq1 = yb;zq1 = 0.7;
xq2 = 3.8;yq2 = yb;zq2 = 0.7;
Q1 = [xq1;yq1;zq1];
Q2 = [xq2;yq2;zq2];
%xi,yi,zi position of Qi in B coordinate
%yawi relative yaw angle
%rotor 1 is (+) xqi and rotor 2 is (+) yqi
%rotor 3 is (-) xqi and rotor 4 is (-) yqi
%zb and zqi are parallel
%% Euler angles (roll, pitch, yaw) payload
phi = 0;theta = 0;psi = 0;
%radians
cx = cos(phi);sx = sin(phi);
cy = cos(theta);sy = sin(theta);
cz = cos(psi);sz = sin(psi);
%from W to B
rx = [1 0 0;0 cx -sx;0 sx cx];
ry = [cy 0 sy;0 1 0;-sy 0 cy];
rz = [cz -sz 0;sz cz 0;0 0 1];
%Rotation matrix from W to B
wRb = rz*rx*ry;
%% Euler angles (roll, pitch, yaw) quad1
phi1 = 0;theta1 = 0;psi1 = 0;
%radians
cx1 = cos(phi1);sx1 = sin(phi1);
cy1 = cos(theta1);sy1 = sin(theta1);
cz1 = cos(psi1);sz1 = sin(psi1);
%from W to B
rx1 = [1 0 0;0 cx1 -sx1;0 sx1 cx1];
ry1 = [cy1 0 sy1;0 1 0;-sy1 0 cy1];
rz1 = [cz1 -sz1 0;sz1 cz1 0;0 0 1];
%Rotation matrix from W to Q1
wRq1 = rz1*rx1*ry1;
%% Euler angles (roll, pitch, yaw) quad2
phi2 = 0;theta2 = 0;psi2 = 0;
%radians
cx2 = cos(phi2);sx2 = sin(phi2);
cy2 = cos(theta2);sy2 = sin(theta2);
cz2 = cos(psi2);sz2 = sin(psi2);
%from W to B
rx2 = [1 0 0;0 cx2 -sx2;0 sx2 cx2];
ry2 = [cy2 0 sy2;0 1 0;-sy2 0 cy2];
rz2 = [cz2 -sz2 0;sz2 cz2 0;0 0 1];
%Rotation matrix from W to B
wRq2 = rz2*rx2*ry2;
%% Layout of system
%plot Bframe in World
plotrefsys(B,wRb)
%plot Q1frame in World
plotrefsys(Q1,wRq1)
%plot Q2frame in World
plotrefsys(Q2,wRq2)
%plot payload
plotpayload(B)
title('System of 2 quadcopters and common payload')
%% Quad parameters and dynamics
% velocities in World frame
syms dphi dtheta dpsi
[v] = [cy 0 -cx*sy;0 1 0;sy 0 cx*cy]*[dphi;dtheta;dpsi];
p = v(1);q = v(2);r = v(3);
%% Payload parameters and dynamics
% position vector of CoM in World frame
Pb = norm(B);
Pq1 = norm(Q1);
Pq2 = norm(Q2);
layoutplot(W,B,Q1,Q2)


%% Quad variables 
%system
% n = 2;
% syms Fdes Mxdes Mydes Mzdes
% syms F Mx My Mxy Mz
% %quad 1
% syms w F1 Mx1 My1 Mz1 
% syms Fq1 Mxq1 Myq1 Mzq1
% syms  x1 y1 psi1
% c1 = cosd(psi1);
% s1 = sind(psi1);
% 
% %quad 2
% syms w F2 Mx2 My2 Mz2 
% syms Fq2 Mxq2 Myq2 Mzq2
% syms x2 y2 psi2
% c2 = cosd(psi2);
% s2 = sind(psi2);

