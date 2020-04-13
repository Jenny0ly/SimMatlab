%% MAIN FUNCTION DEFINED
function [zpos,zdes,force] = DSCCsim2 (mass,B,Q,~,L,times,z_f,K_a)
    %% GLOBAL VARIABLES
    global n g
    global t tm tf dt 
    t = times(1);tm = times(2);tf = times(3);
    dt = 0.25;
    %% DISPLAY INPUT VALUES
    % total mass 
    sentence = ['Total mass is: ', num2str(mass)];
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
    %B      Body frame     (xb,yb,zb) coordinates of CoM of payload
    Eb = [0;0;0]; %euler angles for payload
    %Qi      quadi frame     (xqi,yqi,zqi) 
    Euler = [0;0;0]; %euler angles for quads (same position for everyone)!
    %yawi relative yaw angle
    yaw = 0; %assume 0 (same position for everyone)!
    [Xr,wRq] = quad2(Q,B,Euler);
    [wRb,bRq] = payload2(Eb,wRq);   
    g = 9.81;
    zpos = B(3);zdes = B(3);
    force = zeros(1,n);
    %Dynamics in Body frame
    [u] = DynB2(Xr,yaw); %use RELATIVE variables!
    %% EQUATIONS OF MOTION
    for i=1:tf/dt %15s until hover for now
        %time added to simulation 
        t = 0+i*dt;
        if t == dt
            rB = Q(:,1)-wRb*Xr(:,1);       %position of CoM
            wq = [0;0;0];
            drq = [0;0;0];
            wb = bRq*wq;            %angular velocity of CoM
            drB = drq-cross(wb,wRb*Xr(:,1));   %velocity of CoM
            Edes_ant = [0;0;0];
            ddrB_ant = [0;0;0];
            [ddrB,yawT,rT] = Settrajectory(B,Euler,rB,drB,z_f,K_a);
            [udes,Edes] = trajectoryControl(ddrB,yawT,wb,Eb,Edes_ant,mass);
            zpos(i+1) = B(3);
            zdes(i+1) = rT(3);
        else 
            [Q,drq] = numericalmethod(ddrB,ddrB_ant,drB_ant,Q_ant);
            rB = Q-wRb*Xr(:,1);       %position of CoM
            [wq,~] = gyroscope(Edes,Euler_ant,rB,rB_ant);
            wb = bRq*wq;            %angular velocity of CoM
            drB = drq-cross(wb,wRb*Xr(:,1));   %velocity of CoM
            [ddrB,yawT,rT] = Settrajectory(B,Euler,rB,drB,z_f,K_a);
            [udes,Edes] = trajectoryControl(ddrB,yawT,wb,Eb,Edes_ant,mass);
            zpos(i+1) = rB(3);
            zdes(i+1) = rT(3);
            ddrB_ant = ddrB;
        end
        
        %commanded velocity
        [wdes,u_law] = control2(u,L,udes);
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
        Euler_ant = Edes;
        Edes_ant = Edes;
        Q_ant = Q(:,1);
        
        for j=1:n
            force(i+1,j) = u_law(4*j-3); %force applied by drone i
        end
    end
    graphPos(zpos,zdes)
    graphFor(force)
end

function graphPos(zpos,zdes)
    global dt tf
    time = 0:dt:tf;
    figure(2)
    cla
    plot(time,zpos)
    grid on 
    hold on 
    plot(time,zdes,'--')
    title('Position of beam')
    legend('actual position','desired position','Location','southeast')
    xlabel('time [s]')
    ylabel('z Position [m]')
end

function graphFor(force)
    global dt tf n
    time = 0:dt:tf;
    figure(3)
    cla
    grid on 
    hold on 
    for i=1:n
        plot(time,force(:,i))
    end
    title('Force applied by each quadcopter')
    legend('Force applied by drone 1','Force applied by drone 1','Location','southeast')
    xlabel('time [s]')
    ylabel('Force [N]')
end