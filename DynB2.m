function [A] = DynB2(Xr1,Xr2,yaw1,yaw2)
    global n 
    c1 = cosd(yaw1);s1 = sind(yaw1);
    c2 = cosd(yaw2);s2 = sind(yaw2);
    x1 = Xr1(1);y1 = Xr1(2);
    x2 = Xr2(1);y2 = Xr2(2);
    % A matrix
    A = sym(zeros(4,4*n));
    a1 = [1 0 0 0;...
          y1 c1 -s1 0;...
          -x1 s1 c1 0;...
           0 0 0 1];
    a2 = [1 0 0 0;...
      y2 c2 -s2 0;...
      -x2 s2 c2 0;...
       0 0 0 1];
    % input a1 in A
    conn = [1 5 9 13;2 6 10 14;3 7 11 15;4 8 12 16];
    for i=1:length(conn)
       c = conn(:,i);
       A(c) = a1(:,i);
    end
    % input a2 in A
    conn2 = [17 21 25 29;18 22 26 30;19 23 27 31;20 24 28 32];
    for i=1:length(conn2)
       c = conn2(:,i);
       A(c) = a2(:,i);
    end
    A;
end