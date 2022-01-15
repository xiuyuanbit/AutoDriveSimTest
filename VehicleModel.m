classdef VehicleModel < handle
    properties
        L_;      
        max_steer_;
        
        loc_;
        v_;  
        state_;
        his_states_;
    end
        
    methods
        function obj=VehicleModel(L)
            obj.L_ = L; % [m] wheel base of vehicle
            obj.max_steer_ = 30 * pi/180; % in rad 
            obj.loc_.x = 0;
            obj.loc_.y = 0;
            obj.loc_.yaw = 0;
            obj.v_ = 1;  
            
            obj.state_.loc = obj.loc_;
            obj.state_.v = obj.v_;
            
            obj.his_states_.x=[];
            obj.his_states_.y=[];
            obj.his_states_.yaw =[];
            obj.his_states_.v =[];    
        end
        
        function state=move(obj, delta, a, dt)
            delta = max(min(obj.max_steer_, delta), -obj.max_steer_);
            obj.loc_.x = obj.loc_.x + obj.v_ * cos(obj.loc_.yaw) * dt; % + randn(1)* 0.1;% 故意加测量噪声进去
            obj.loc_.y = obj.loc_.y + obj.v_ * sin(obj.loc_.yaw) * dt; % +  randn(1)* 0.1;
            obj.loc_.yaw = obj.loc_.yaw + obj.v_ / obj.L_ * tan(delta) * dt; %remember here the yaw is in rad
            obj.v_ = obj.v_ + a * dt;  % + randn(1) * 0.1;% 故意加噪声进去，模拟车辆无法正常实现命令的情形
            obj.state_.loc = obj.loc_;
            obj.state_.v = obj.v_;
            state = obj.state_;
        end
        
        function v = vel(obj)
            v = obj.v_;
        end
        
        function state=state(obj)
            state = obj.state_;
        end
        
        function printState(obj)
            %disp("Hello");
            str = sprintf('pos(%.3f m,%.3f m,%.3f rad),v=%.3fm/s',...
                        obj.loc_.x,obj.loc_.y,obj.loc_.yaw,obj.v_);
            disp(str);
        end
        
        function saveState(obj)
            obj.his_states_.x=[obj.his_states_.x;obj.loc_.x];
            obj.his_states_.y=[obj.his_states_.y;obj.loc_.y];
            obj.his_states_.yaw=[obj.his_states_.yaw;obj.loc_.yaw];
            obj.his_states_.v = [obj.his_states_.v;obj.v_];
        end
        
        function plotHisTraj(obj)
            plot(obj.his_states_.x,obj.his_states_.y,'.-');
        end
        
        function plotVel(obj)
            plot(obj.his_states_.v,'.-');
        end
        
    end
    
end