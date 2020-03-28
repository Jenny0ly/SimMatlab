function plotpayload (p)

%rectangular prism
%need width(w),height(h),length(l) in meters
w = 0.3;h = 0.7; l = 2;

%face 2
l_start = p(1)-l/2;l_end = p(1)+l/2;
w_start = p(2);w_end = p(2)-w;%el prisma va hacia atras de mi dibujo
h_start = 0;h_end = h;
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
% patch('Vertices',vert,'Faces',fac,'FaceVertexCData',hsv(6),'FaceColor','flat')
patch('Vertices',vert,'Faces',fac,'FaceVertexCData',hsv(6),'FaceColor','w','FaceAlpha','0.3')
view(3)
hold on


end