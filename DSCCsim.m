clear all
close all
clc
%% Coordinate Frames 
%W      World frame    (xw,yw,zw) ENU (East, North, Upward)
xw = 0;yw = 0;zw = 0;
W = [xw;yw;zw];
%Qi     Quad frame     (xqi, yqi,zqi) z is Upward
xq1 = 2.2;yq1 = 4;zq1 = 0.7;
xq2 = 3.8;yq2 = 4;zq2 = 0.7;
Q1 = [xq1;yq1;zq1];
Q2 = [xq2;yq2;zq2];
%B      Body frame     (xb,yb,zb) coordinates of CoM of payload
h = 0.7;
xb = 3;yb = 4;zb = h/2;
B = [xb,yb,zb];
%xi,yi,zi position of Qi in B coordinate
%yawi relative yaw angle
%rotor 1 is (+) xqi and rotor 2 is (+) yqi
%rotor 3 is (-) xqi and rotor 4 is (-) yqi
%zb and zqi are parallel
%Euler angles (roll, pitch, yaw)
%syms phi theta psi
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

%plot Bframe in World
plotrefsys(B,wRb)
%plot Q1frame in World
plotrefsys(Q1,wRb)
%plot Q2frame in World
plotrefsys(Q2,wRb)
%plot payload
plotpayload(B)

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

