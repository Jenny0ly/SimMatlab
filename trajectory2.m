function trajectory2(B,wRb,beam)
time = 0;
tf = 15;
v = VideoWriter('Beam_trajectory.avi');
open(v);

% figure
xmin = 0;xmax = 5;
ymin = 0;ymax = 5;
zmin = 0;zmax = 3.5; 
    % Sample trajectory
    for i=1:50
        x(i) = B(1)*ones(length(time),1);
        y(i) = B(2)*ones(length(time),1);
        if time <= tf
            z(i) = B(3)+0.2*time;
        else 
            z(i) = B(3)+0.2*tf;
        end
        %% plots animated trajectory
        %plot3(x,y,z,'*r');
        %hold on 
        %grid on 
        %% plots animated payload
        p = [x(i) y(i) z(i)];
        figure
        plottrajectory(p,wRb,beam)
        axis([xmin xmax ymin ymax zmin zmax])
    %     pause(0.01);
        if mod(time,2) == 0
            frame(i) = getframe(gcf);
            writeVideo(v,frame(i));
        end
        time = 0+0.25*i;
    end
end
    
function plottrajectory(p,R,beam)
    %% CoM frame 
    xb_axis = R(:,1);
    yb_axis = R(:,2);
    zb_axis = R(:,3);
    %p is location of payload CoM
    plot3(p(1),p(2),p(3),'o','Color','k','LineWidth',0.5)
    hold on
    xlabel('x [m]')
    ylabel('y [m]')
    zlabel('z [m]')
    grid on
    %x axis red
    %y axis green
    %z axis blue
    plot3([p(1),p(1)+xb_axis(1)],[p(2),p(2)+xb_axis(2)],[p(3),p(3)+xb_axis(3)],'r','LineWidth',1)
    plot3([p(1),p(1)+yb_axis(1)],[p(2),p(2)+yb_axis(2)],[p(3),p(3)+yb_axis(3)],'g','LineWidth',1)
    plot3([p(1),p(1)+zb_axis(1)],[p(2),p(2)+zb_axis(2)],[p(3),p(3)+zb_axis(3)],'b','LineWidth',1)
    
    %% payload sketch
    %rectangular prism
    %need width(w),height(h),length(l) in meters
    w = beam(1);h = beam(2);l = beam(3);

    %face 2
    l_start = p(1)-l/2;l_end = p(1)+l/2;
    w_start = p(2);w_end = p(2)-w;%el prisma va hacia atras de mi dibujo
    h_start = p(3)-h/2;h_end = p(3)+h/2;
    vert = [l_start w_start h_start;...
            l_end w_start h_start;...
            l_end w_end h_start;...
            l_start w_end h_start;...
            l_start w_start h_end;...
            l_end w_start h_end;...
            l_end w_end h_end;...
            l_start w_end h_end;...
            ];
    fac = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8]; 
    patch('Vertices',vert,'Faces',fac,'FaceVertexCData',hsv(6),'FaceColor','w','FaceAlpha','0.3')
    view(3)
hold on 
end