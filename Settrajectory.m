function [ddrB,yawT,rT] = Settrajectory(B,Euler,rB,drB,z_f,K)
global t tf tm 
m_p = (z_f-B(3))/tm;
    if t <= tm
        xdes = B(1);
        ydes = B(2);
        zdes = m_p*t+B(3);
    else 
        xdes = B(1);
        ydes = B(2);
        zdes = m_p*tm+B(3);
    end

    rT = [xdes;ydes;zdes];
    yawT = Euler(3); %desired yaw angle
    %position control
    [ddrB] = desiredacc(rT,rB,drB,K);
end


function [ddrB] = desiredacc(rT,rB,drB,K) %100Hz
global tm t 
kp = K(1);kd = K(2);
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