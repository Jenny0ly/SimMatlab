function [ddrB,yawT,rT] = Settrajectory(B,Euler1,rB,drB)
global t tf tm 
    if t <= tm
        xdes = B(1);
        ydes = B(2);
        zdes = 0.2*t+B(3);
    else 
        xdes = B(1);
        ydes = B(2);
        zdes = 0.2*tm+B(3);
    end

    rT = [xdes;ydes;zdes];
    yawT = Euler1(3); %desired yaw angle
    %position control
    [ddrB] = desiredacc(rT,rB,drB);
end


function [ddrB] = desiredacc(rT,rB,drB) %100Hz
global tm t 
kp = 10;kd = 0.5;
    %3Dtrajectory control 
    if t<=tm 
        drT = [0;0;0.2];
    else 
        drT = [0;0;0];
    end
    ep = (rT - rB);
    ev = (drT - drB);
    ddrB = kp*ep + kd*ev;
end