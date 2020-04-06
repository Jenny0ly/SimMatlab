function [Xr,wRq1,wRq2] = quad2(Q,B,Euler1,Euler2)
%Qi     Quad frame     (xqi, yqi,zqi) z is Upward
    Q1 = Q(:,1);
    Q2 = Q(:,2);
    %xi,yi,zi position of Qi in B coordinate
    x1 = Q1(1)-B(1);y1 = Q1(2)-B(2);z1 = Q1(3)-B(3);
    Xr1 = [x1;y1;z1]; %relative position drone 1 in B frame 
    sentence = ['Relative position drone1: ', num2str(Xr1')];
    disp(sentence)
    x2 = Q2(1)-B(1);y2 = Q2(2)-B(2);z2 = Q2(3)-B(3);
    Xr2 = [x2;y2;z2];%relative position drone 2 in B frame
    sentence = ['Relative position drone2: ', num2str(Xr2')];
    disp(sentence)
    Xr = [Xr1 Xr2];
    %rotor 1 is (+) xqi and rotor 2 is (+) yqi
    %rotor 3 is (-) xqi and rotor 4 is (-) yqi
    %zb and zqi are parallel
    %% Euler angles (roll, pitch, yaw) quad1
    %degrees
    phi1 = Euler1(1);theta1 = Euler1(2);psi1 = Euler1(3);
    cx1 = cosd(phi1);sx1 = sind(phi1);
    cy1 = cosd(theta1);sy1 = sind(theta1);
    cz1 = cosd(psi1);sz1 = sind(psi1);
    %from W to Q1
    rx1 = [1 0 0;0 cx1 -sx1;0 sx1 cx1];
    ry1 = [cy1 0 sy1;0 1 0;-sy1 0 cy1];
    rz1 = [cz1 -sz1 0;sz1 cz1 0;0 0 1];
    %Rotation matrix from W to Q1
    wRq1 = rz1*rx1*ry1;
    %% Euler angles (roll, pitch, yaw) quad2
    %radians
    phi2 = Euler2(1);theta2 = Euler2(2);psi2 = Euler2(3);
    cx2 = cosd(phi2);sx2 = sind(phi2);
    cy2 = cosd(theta2);sy2 = sind(theta2);
    cz2 = cosd(psi2);sz2 = sind(psi2);
    %from W to Q2
    rx2 = [1 0 0;0 cx2 -sx2;0 sx2 cx2];
    ry2 = [cy2 0 sy2;0 1 0;-sy2 0 cy2];
    rz2 = [cz2 -sz2 0;sz2 cz2 0;0 0 1];
    %Rotation matrix from W to B
    wRq2 = rz2*rx2*ry2;
    
end