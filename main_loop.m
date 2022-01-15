clear;clc;close all;

traj = init_trajectory();
figure;
h = plot(traj.x,traj.y,'-k');hold on;

L = 2.9; % [m] wheel base of vehicle

car = VehicleModel(L);
lqr_controller = LqrController(L);

%car.printState();
state = car.state();
target_v = 30/3.6;

ind =0;
dt = 0.02;
pe = 0;
pth_e = 0;

while ind < length(traj.x)-1
    if ind > 0
        delete(h);
    end
    %delta =2.0*pi/180;
    a = PIDcontrol(target_v,state.v,1);
    
    [delta,ind,e,th_e] =  lqr_controller.control(state,traj, pe, pth_e,dt);
    pe =e;
    pth_e = th_e;
    if abs(e)> 4
        fprintf('Error is too large!\n')
        break;
    end
    
    state = car.move(delta ,a,dt);
    car.saveState();
    
    h = plot(state.loc.x,state.loc.y,'bo');
    pause(0.01);
 
    hold on
end

car.plotHisTraj();
% figure;
% car.plotVel();


function [a] = PIDcontrol(target_v, current_v, Kp)
a = Kp * (target_v - current_v);
end
