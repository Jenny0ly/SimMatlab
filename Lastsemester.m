clear
clc
close all
global K A B L C
%% state nonlinear equation
syms xi x y z;
syms eta phi theta psi;
syms nu p q r;

%Inertia Matrix
syms Ixx Iyy Izz;
I = [Ixx 0 0;0 Iyy 0;0 0 Izz];

%angular velocity
syms wi w1 w2 w3 w4; 
%constant of lift and drag
syms kf km;
kf = 3.13e-15; km = 7.5e-7;
%force of motor
Fi = kf*wi^2;
%total thrust
T = kf*(w1^2+w2^2+w3^2+w4^2);
%momentum of motor
Mi = km*wi^2;
%roll momentum 
Mr = kf*(w4^2-w2^2);
%picth momentum
Mp = kf*(w1^2-w3^2);
%yaw momentum
My = km*(-w1^2+w2^2-w3^2+w4^2);

syms F1 F2 F3 F4;
syms M1 M2 M3 M4;
syms L; 

%% angular acceleration of quadcopter body frame
syms dp dq dr dnu
syms w1 w2 w3 w4 wgamma Ir

%% inputs of system
u1 = kf*(w1^2+w2^2+w3^2+w4^2); 
u2 = kf*(w4^2-w2^2); 
u3 = kf*(w1^2-w3^2);  
u4 = km*(-w1^2+w2^2-w3^2+w4^2);  
U = [u1;u2;u3;u4];

%% state vector 
syms dphi dtheta dpsi dz dy dx
state_x = [phi dphi theta dtheta psi dpsi z dz x dx y dy];

%% variables of system
%Euler angles in radians
cz = cos(state_x(1));
sz = sin(state_x(1));
cy = cos(state_x(3));
sy = sin(state_x(3));
cx = cos(state_x(5));
sx = sin(state_x(5));

%% coefficients confirmed m, g, Ixx, Iyy, Izz, L MISSING Ir
m = 0.063;g = 9.81;L = 0.18;Ir = 1.5e-5;
Ixx = 0.582857e-4; Iyy = 0.716914e-4; Izz = 1e-4;
wgamma = -w1+w2-w3+w4;
c1 = (Iyy-Izz)/Ixx; c2 = (Izz-Ixx)/Iyy; c3 = (Ixx-Iyy)/Izz;
c4 = Ir/Ixx; c5 = Ir/Iyy;
d1 = L/Ixx; d2 = L/Iyy; d3 = L/Izz;
hx = cx*sy*cz+sx*sz;
hy = cx*sy*sz-sx*cz;
hz = cx*cy;

%% non linear system
% %dpsi
% f1 = state_x(2);
% %ddpsi
% f2 = state_x(4)*state_x(6)*c1+d1*u2-c4*state_x(4)*wgamma;
% %dtheta
% f3 = state_x(4);
% %ddtheta
% f4 = state_x(2)*state_x(6)*c2+d2*u3-c5*state_x(2)*wgamma;
% %dpsi
% f5 = state_x(6);
% %ddpsi
% f6 = state_x(2)*state_x(4)*c3+d3*u4;
% %dz
% f7 = state_x(8);
% %ddz
% f8 = g-hz*u1/m;
% %dx
% f9 = state_x(10);
% %%dx
% f10 = -hx*u1/m;
% %dy
% f11 = state_x(12);
% %ddy
% f12 = -hy*u1/m;
% 
% Func = [f1;f2;f3;f4;f5;f6;f7;f8;f9;f10;f11;f12];

%% linear system about x = [0...0]' and u =[mg 0 0 0]'
A = zeros(12);
A(12,1) = g;
A(10,3) = -g;
av = [1 0 1 0 1 0 1 0 1 0 1];
A = A + diag(av,1);
% A=[ 0 1 0 0 0 0 0 0 0 0 0 0;...
%     0 0 0 0 0 0 0 0 0 0 0 0;...
%     0 0 0 1 0 0 0 0 0 0 0 0;...
%     0 0 0 0 0 0 0 0 0 0 0 0;...
%     0 0 0 0 0 1 0 0 0 0 0 0;...
%     0 0 0 0 0 0 0 0 0 0 0 0;...
%     0 0 0 0 0 0 0 1 0 0 0 0;...
%     0 0 0 0 0 0 0 0 0 0 0 0;...
%     0 0 0 0 0 0 0 0 0 1 0 0;...
%     0 0 -g 0 0 0 0 0 0 0 0 0;...
%     0 0 0 0 0 0 0 0 0 0 0 1;...
%     g 0 0 0 0 0 0 0 0 0 0 0]
B = [0 0 0 0;0 d1 0 0;0 0 0 0;0 0 d2 0;0 0 0 0;0 0 0 d3;...
     0 0 0 0;1/m 0 0 0;0 0 0 0;0 0 0 0;0 0 0 0;0 0 0 0];
C = [1 0 0 0 0 0 0 0 0 0 0 0;...
     0 0 1 0 0 0 0 0 0 0 0 0;...
     0 0 0 0 1 0 0 0 0 0 0 0;...
     0 0 0 0 0 0 1 0 0 0 0 0;...
     0 0 0 0 0 0 0 0 1 0 0 0;...
     0 0 0 0 0 0 0 0 0 0 1 0];
Dis = [0 0 0 0 0 0;...
    0 0 0 0 0 0;...
    0 0 0 0 0 0;...
    0 0 0 d1 0 0;...
    0 0 0 0 d2 0;...
    0 0 0 0 0 d3;...
    0 0 0 0 0 0;...
    1/m 0 0 0 0 0;...
    0 1/m 0 0 0 0;...
    0 0 1/m 0 0 0;...
    0 0 0 0 0 0;...
    0 0 0 0 0 0];
D = 0; 

B1 = B(:,1);
B2 = B(:,2);
B3 = B(:,3);
B4 = B(:,4);

%% controllability
P = ctrb(A,B);
rank(P);
if rank (P) == length(A)
    display('system is controllable')
else
    display('system is not controllable')
end
%% observability
Q = obsv(A,C);
rank(Q);
if rank (Q) == length(A)
    display('system is observable')
else
    display('system is not observable')
end

%% open loop system
sys = ss(A,B,C,D);
t = 0:0.01:4;
U = zeros(length(t),4);
x0 = [0,0,0,0,0,0,-10,0,10,0,10,0]';
[Yo,t,Xo] = lsim(sys,U,t,x0);% open-loop response

%% Feedback Closed-loop system I
%percentage oveershoot
OS = 1;
%settling time
Ts = 0.5;
%Damping ratio
dar = (-log(OS/100))/(sqrt(pi^2+log(OS/100)^2));
%frequency
wn = 4/(dar*Ts);
%desired characteristic equation: s^2+2*dar*wn*s+wn^2
charp = [1 2*dar*wn wn^2];
%desired eigenvalues
r = roots(charp);

%desired polynomials
for i=1:length(A)
% for i=1:length(A)
    if i<3 %dominant eigenvalues
        pp(i) = r(i);
    else %10 times greater than dominant ones
        pp(i) = 10*real(r(1))-i*2; 
    end
end
pp;
K = place(A,B,pp);
Acl = A-B*K;
sys_cl = ss(Acl,B,C,D);

t = 0:0.01:4;
[Yc,t,Xc] = lsim(sys_cl,U,t,x0); 

%% Observer based compensator I
t = 0:0.01:4;
xr0 = [0;0;0;0;0;0;-10;0;10;0;10;0;...
       0;0;0;0;0;0;-3;0;15;0;15;0];
for i=1:length(pp)
    if i==1
        ppp(i) = pp(end)-i;
    else
        ppp(i) = ppp(i-1)-i;
    end
end
L = place(A',C',ppp)';
Ahat = A-L*C;
Ar = [(A-B*K) B*K;zeros(size(A)) (A-L*C)];
Br = [B;zeros(size(B))];
Cr = [C zeros(size(C))];
sys_ob = ss(Ar,Br,Cr,0); % Create the closed-loop system with observer 
r = zeros(length(t),4); % Define zero reference
% input to go with t
[Yr,t,Xr] = lsim(sys_ob,r,t,xr0);

%% Steady-State LQR
Q = eye(12);
rho = 1;
R = rho*eye(4);
bb = B*inv(R)*B';
pbar = are(A,bb,Q); %solves algebraic Riccati eq
KLQR = inv(R)*B'*pbar; %state feedback gain
ALQR = A-B*KLQR; %closed loop matrix
syslqr = ss(ALQR,B,C,0);
[YLQR,t,XLQR] = lsim(syslqr,U,t,x0);

%% Comparison between observer compensator, feedback and LQR 
%z
figure
plot(t,Xo(:,7),'-',t,Xc(:,7),'--',t,XLQR(:,7),'-.')
title('Z position')
legend('Open loop', 'Cloosed loop', 'LQR')
grid on 
%x
figure
plot(t,Xo(:,9),'-',t,Xc(:,9),'--',t,XLQR(:,9),'-.',t,Xr(:,9))
title('X position')
legend('Open loop', 'Cloosed loop', 'LQR','Observer based compensator')
grid on 
%y
figure
plot(t,Xo(:,11),'-',t,Xc(:,11),'--',t,XLQR(:,11),'-.',t,Xr(:,11))
title('Y position')
legend('Open loop', 'Cloosed loop', 'LQR','Observer based compensator')
grid on 

Uc = -K*Xc';
ULQR = -inv(R)*B'*pbar*XLQR';
figure;
plot(t,Uc(1,:),t,ULQR(1,:)); 
grid on;
% axis([0 t(end) -10 10])
legend('Closed-loop','LQR');