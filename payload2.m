function [wRb,bRq] = payload2(Eb,wRq)
%% Euler angles (roll, pitch, yaw) payload
    sentence6 = ['Angles of Payload: ', num2str(Eb')];
    disp(sentence6);
    %% rotation matrix if we known beforehand (not possible)
%     %degrees
%     phi = Eb(1);theta = Eb(2);psi = Eb(3);
%     cx = cosd(phi);sx = sind(phi);
%     cy = cosd(theta);sy = sind(theta);
%     cz = cosd(psi);sz = sind(psi);
%     %from W to B
%     rx = [1 0 0;0 cx -sx;0 sx cx];
%     ry = [cy 0 sy;0 1 0;-sy 0 cy];
%     rz = [cz -sz 0;sz cz 0;0 0 1];
%     %Rotation matrix from W to B
%     wRb = rz*rx*ry;

%% rotation matrix from known orientation wrt drone1 
    %degrees
    phi = Eb(1);theta = Eb(2);psi = Eb(3);
    cx = cosd(phi);sx = sind(phi);
    cy = cosd(theta);sy = sind(theta);
    cz = cosd(psi);sz = sind(psi);
    %from W to B
    rx = [1 0 0;0 cx -sx;0 sx cx];
    ry = [cy 0 sy;0 1 0;-sy 0 cy];
    rz = [cz -sz 0;sz cz 0;0 0 1];
    %Rotation matrix from W to B
    bRq = rz*rx*ry;


%% rotation matrix from known orientation wrt drone1 
    %degrees
    phi = Eb(1);theta = Eb(2);psi = Eb(3);
    cx = cosd(phi);sx = sind(phi);
    cy = cosd(theta);sy = sind(theta);
    cz = cosd(psi);sz = sind(psi);
    %from W to B
    rx = [1 0 0;0 cx -sx;0 sx cx];
    ry = [cy 0 sy;0 1 0;-sy 0 cy];
    rz = [cz -sz 0;sz cz 0;0 0 1];
    %Rotation matrix from W to B
    qRb = rz*rx*ry;
    
      wRb = wRq*qRb;
end