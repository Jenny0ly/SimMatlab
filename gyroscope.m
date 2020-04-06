function [wq1,drq1] = gyroscope(Euler1,Euler1_ant,rB,rB_ant)
    global dt
    %angular velocity of quad1
    wq1 = ((Euler1-Euler1_ant)/dt);
    drq1 = ((rB-rB_ant)/dt);
end