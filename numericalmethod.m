function [rB,drB] = numericalmethod (ddrB,ddrB_ant,drB_ant,Q1_ant)
    global dt
    %% obtain linear position
    %first obatain linear velocity
    drB = dt*(ddrB + ddrB_ant)/2;
    %obtain linear position 
    rB = dt*(drB + drB_ant)/2+Q1_ant;
end