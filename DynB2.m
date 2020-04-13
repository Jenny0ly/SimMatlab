function [u] = DynB2(Xr,yaw)
    global n 
    v = [];
    c = cosd(yaw);s = sind(yaw);
    % A matrix
    A = [];
    
    for i=1:n
        x = Xr(1,i);y = Xr(2,i);
        a1 = [1;y;-x;0];
        a2 = [0;c;s;0];
        a3 = [0;-s;c;0];
        a4 = [0;0;0;1];
        Aa = [a1 a2 a3 a4];
        A = [A Aa];
    end

    %H matrix weights
    wF  = 0.3;wMxy = 0.4;wMz = 0.2;
    for i=1:n
        vv = [sqrt(wF) sqrt(wMxy) sqrt(wMxy) sqrt(wMz)];
        v = [v vv];
    end
    H = diag(v);
    %matrix (8) in mellinger's paper
    mA = A*H^-2*A';
    %input
    u = H^-2*A'*inv(mA);
end