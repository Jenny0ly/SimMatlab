function [udes,Edes] = trajectoryControl(ddrB,yawT,wb,Eb,Edes_ant,mass)
global dt

    [Edes,Fdes] = position_control (ddrB,yawT,mass);
    %attitude control 
    wbdes = ((Edes-Edes_ant)/dt);
    [Mxdes,Mydes,Mzdes] = attitude(Eb,Edes,wb,wbdes);
    
    udes = [Fdes;Mxdes;Mydes;Mzdes];
    
end


function [Mxdes,Mydes,Mzdes] = attitude(Eb,Edes,wb,wbdes) %1kHz
kpx = 0.5;kdx = 0.5;
kpy = 0.5;kdy = 0.5;
kpz = 0.5;kdz = 0.5;
%eta = 1; wn = 0.9;
%kpx = Ixx*wn^2; kdx = 2*Ixx*eta*wn;
%kpy = Iyy*wn^2; kdy = 2*Iyy*eta*wn;
%kpz = Izz*wn^2; kdz = 2*Izz*eta*wn;

    %       kpx*(phides-phi) + kdx*(pdes-p)
    Mxdes = kpx*(Edes(1)-Eb(1)) + kdx*(wbdes(1)-wb(1));
    %       kpy*(thetades-theta) + kdy*(qdes-q)
    Mydes = kpy*(Edes(2)-Eb(2)) + kdy*(wbdes(2)-wb(2));
    %       kpz*(psides-psi) + kdz*(rdes-r)
    Mzdes = kpz*(Edes(3)-Eb(3)) + kdz*(wbdes(3)-wb(3));
end

function [Edes,Fdes] = position_control (ddrB,yawT,mass) %100Hz
global g 
    
    %desired angles and force
    phides = 1/g*(ddrB(1)*sind(yawT)-ddrB(2)*cosd(yawT));
    thetades = 1/g*(ddrB(1)*cosd(yawT)+ddrB(2)*sin(yawT));
    yawdes = yawT;
    Fdes = mass*(ddrB(3)+g);

    Edes = [phides;thetades;yawdes];
    
end