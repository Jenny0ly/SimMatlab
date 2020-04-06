function [W,F,Mo] = motormodel2(wdes, w_init)
global kf km k t n
k =  20;
W = zeros(4*n,1);
F = zeros(4*n,1);
Mo = zeros(4*n,1);
    %% motor model
    %equation relating rotor speed with commanded speed
    %dw = k*(wdes - w)
    %solution to ode
    for i=1:4*n
        W(i) = wdes(i) - (wdes(i) - w_init(i))*exp(-k*t);
    end
    %Force and moment in each motor of each drone 
        for i=1:length(W)
            F(i) = kf*W(i)^2;
            Mo(i) = km*W(i)^2;
        end
end