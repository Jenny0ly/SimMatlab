clear all
close all
clc
format short
%% INPUT VARIABLES
beam = [0.3 0.7 2]; %dimension of beam, width,heigth,length 
mass = [0.7 1.6 1.6]; %mass of payload drone1 drone2 
B = [3,3-beam(1)/2,beam(2)]; %CoM position in world frame
Q = [2.2 3.8;3-beam(1)/2 3-beam(1)/2;0.7 0.7]; %drones position in world frame        
L = 0.225; %length of arm 450mm/2 
%% CALL MAIN FUNCTION
main(mass,B,Q,beam,L);

%% MAIN FUNCTION DEFINED
function main (mass,B,Q,beam,L)
    %% GLOBAL VARIABLES
    global n t tm tf dt g masst 
    t = 0; dt = 0.25;tf = 30;tm = 15;
    %% DISPLAY INPUT VALUES
    % Payloads mass
    sentence = ['Payload mass is: ', num2str(mass(1))];
    disp(sentence)
    % Payloads mass
    sentence = ['Drones mass is: ', num2str(mass(2))];
    disp(sentence)
    % Payloads CoM in world frame 
    sentence = ['Position of CoM is: ', num2str(B)];
    disp(sentence)
    % number of drones
    col = size(Q);
    n = col(2);
    sentence = ['Number of drones: ',num2str(n)];
    disp(sentence)
    % Position of drones in world frame 
    for i=1:n
        sentence = ['Position of drone: ',num2str(Q(:,i)')];
        disp(sentence)
    end
    %initial velocity
    w_init = zeros(4*n,1);
    %% Coordinate Frames 
    %W      World frame    (xw,yw,zw) ENU (East, North, Upward)
    xw = 0;yw = 0;zw = 0;
    Wf = [xw;yw;zw];
    %B      Body frame     (xb,yb,zb) coordinates of CoM of payload
    Eb = [0;0;0]; %euler angles for payload
    %Qi      quadi frame     (xqi,yqi,zqi) 
    Euler1 = [0;0;0]; %euler angles for quad1
    Euler2 = [0;0;0]; %euler angles for quad2
    %yawi relative yaw angle
    yaw1 = 0; %assume 0
    yaw2 = 0; %assume 0
    [Xr,wRq1,wRq2] = quad2(Q,B,Euler1,Euler2);
    [wRb,bRq1] = payload2(Eb,wRq1);
    Q1 = Q(:,1);
    Q2 = Q(:,2);
    Xr1 = Xr(1:3,1);
    Xr2 = Xr(1:3,2);
    %% Layout of system
    %plot Bframe in World
    plotrefsys2(B,wRb)
    %plot Q1frame in World
    plotrefsys2(Q1,wRq1)
    %plot Q2frame in World
    plotrefsys2(Q2,wRq2)
    %plot payload
    plotpayload2(B,beam)
    title('System of 2 quadcopters and common payload')
    
    %plot position vectors
    layoutplot2(Wf,B,Q1,Q2)

    %mass of entire system!
    masst = sum(mass);
    g = 9.81;
    zpos = B(3);zdes = B(3);
    forced1 = 0;forced2 = 0;
    %% EQUATIONS OF MOTION
    for i=1:tf/dt %15s until hover for now
        %time added to simulation 
        t = 0+i*dt;
        if t == dt
            rB = Q1-wRb*Xr1;       %position of CoM
            wq1 = [0;0;0];
            drq1 = [0;0;0];
            wb = bRq1*wq1;            %angular velocity of CoM
            drB = drq1-cross(wb,wRb*Xr1);   %velocity of CoM
            Edes_ant = [0;0;0];
            ddrB_ant = [0;0;0];
            [ddrB,yawT,rT] = Settrajectory(B,Euler1,rB,drB);
            [udes,Edes] = trajectoryControl(ddrB,yawT,wb,Eb,Edes_ant);
            zpos(i+1) = B(3);
            zdes(i+1) = rT(3);
        else 
            [Q1,drq1] = numericalmethod (ddrB,ddrB_ant,drB_ant,Q1_ant);
            rB = Q1-wRb*Xr1;       %position of CoM
            [wq1,~] = gyroscope(Edes,Euler1_ant,rB,rB_ant);
            wb = bRq1*wq1;            %angular velocity of CoM
            drB = drq1-cross(wb,wRb*Xr1);   %velocity of CoM
            [ddrB,yawT,rT] = Settrajectory(B,Euler1,rB,drB);
            [udes,Edes] = trajectoryControl(ddrB,yawT,wb,Eb,Edes_ant);
            zpos(i+1) = rB(3);
            zdes(i+1) = rT(3);
            ddrB_ant = ddrB;
        end
        
        %Dynamics in Body frame
        [A] = DynB2(Xr1,Xr2,yaw1,yaw2); %use RELATIVE variables!
        %commanded velocity
        [wdes,u_law] = control2(A,L,udes);
        %current speed in rotor W
        %Force and Moment by each motor of each drone
        [W,F,Mo] = motormodel2(wdes, w_init);
        
        %animated expected trajectory
        %take off --> hover
        %trajectory2(B,wRb,beam)
        
        % settings for next step in simulation
        w_init = W;
        drB_ant = drB;
        rB_ant = rB;
        Euler1_ant = Edes;
        Edes_ant = Edes;
        Q1_ant = Q1;
        
        Forced1(i+1) = u_law(1); %force applied by drone 1
        Forced2(i+1) = u_law(5); %force applied by drone 2
    end
    graphPos(zpos,zdes)
    graphFor(Forced1,Forced2)
end

function graphPos(zpos,zdes)
    global dt tm tf
    time = 0:dt:tf;
    figure(2)
    plot(time,zpos)
    grid on 
    hold on 
    plot(time,zdes,'--')
    title('Position of beam')
    legend('actual position','desired position','Location','southeast')
    xlabel('time [s]')
    ylabel('z Position [m]')
end

function graphFor(Forced1,Forced2)
    global dt tm tf
    time = 0:dt:tf;
    figure(3)
    plot(time,Forced1,'r')
    grid on 
    hold on 
    plot(time,Forced2,'b')
    title('Force applied by each quadcopter')
    legend('Force applied by drone 1','Force applied by drone 1','Location','southeast')
    xlabel('time [s]')
    ylabel('Force [N]')
end