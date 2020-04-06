function plotrefsys2(p,R)
    xb_axis = R(:,1);
    yb_axis = R(:,2);
    zb_axis = R(:,3);
    %p is location of payload CoM
    plot3(p(1),p(2),p(3),'o','Color','k','LineWidth',0.5)
    xlabel('x [m]')
    ylabel('y [m]')
    zlabel('z [m]')
    grid on
    hold on 
    %x axis red
    %y axis green
    %z axis blue
    plot3([p(1),p(1)+xb_axis(1)],[p(2),p(2)+xb_axis(2)],[p(3),p(3)+xb_axis(3)],'r','LineWidth',1)
    plot3([p(1),p(1)+yb_axis(1)],[p(2),p(2)+yb_axis(2)],[p(3),p(3)+yb_axis(3)],'g','LineWidth',1)
    plot3([p(1),p(1)+zb_axis(1)],[p(2),p(2)+zb_axis(2)],[p(3),p(3)+zb_axis(3)],'b','LineWidth',1)

    xmin = 0;xmax = 5;
    ymin = 0;ymax = 5;
    zmin = 0;zmax = 5; 
    axis([xmin xmax ymin ymax zmin zmax])

end