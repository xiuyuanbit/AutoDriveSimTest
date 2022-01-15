classdef VehicleModel < handle
    properties
        L_;      
        B_ = 1.8;
        DelayAcc = 0.2;
        DelaySteer = 0.3;
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
            obj.loc_.yaw = pi/6;
            obj.loc_.yaw_rate = 0;
            obj.v_ = 1;  
            
            obj.state_.loc = obj.loc_;
            obj.state_.v = obj.v_;
            obj.state_.a = 0;
            obj.state_.delta = 0;
            
            obj.his_states_.x=[];
            obj.his_states_.y=[];
            obj.his_states_.yaw =[];
            obj.his_states_.v =[];    
        end
        
        function state=move(obj, delta, a, dt)

            delta = max(min(obj.max_steer_, delta), -obj.max_steer_);
            
            % Add car actors time_delay
            obj.state_.a = obj.state_.a + (a - obj.state_.a) * dt/obj.DelayAcc;
            obj.state_.delta = obj.state_.delta + (delta - obj.state_.delta) * dt/obj.DelaySteer;
            
            obj.loc_.yaw_rate = obj.v_ * tan(obj.state_.delta)/obj.L_;
            obj.loc_.yaw = obj.loc_.yaw  + obj.loc_.yaw_rate *dt;
            obj.loc_.x = obj.loc_.x + obj.v_ * cos(obj.loc_.yaw) * dt; % + randn(1)* 0.1;% 故意加测量噪声进去
            obj.loc_.y = obj.loc_.y + obj.v_ * sin(obj.loc_.yaw) * dt; % +  randn(1)* 0.1;
            obj.v_ = obj.v_ + obj.state_.a * dt;  % + randn(1) * 0.1;% 故意加噪声进去，模拟车辆无法正常实现命令的情形
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
        
        function plotVehicle(obj,type)
            if nargin < 2
                type='car';
            end
            if strcmp(type, 'pt')
                len = length(obj.his_states_.x);
                if len >=2
                pt1= obj.state_.loc;
                pt2.x = obj.his_states_.x(len-1);
                pt2.y = obj.his_states_.y(len-1);
                 obj.plotLine(pt1,pt2,'tag','ax1','g');
                end
            end
            delta = obj.state_.delta;
            yaw = obj.state_.loc.yaw;
            pt0 = obj.state_.loc;
            L = obj.L_;
            B = obj.B_;
            WidthTire = 0.5;
            
            pt1 = obj.transTo(pt0,L*0.5,yaw);
            pt2 = obj.transTo(pt0,-L*0.5,yaw);
            pt11 = obj.transTo(pt1,B*0.5,pi/2 + yaw);
            pt12 = obj.transTo(pt1,-B*0.5,pi/2 + yaw);
            pt21 = obj.transTo(pt2,B*0.5,pi/2 + yaw);
            pt22 = obj.transTo(pt2,-B*0.5,pi/2 + yaw);
            pt111 = obj.transTo(pt11,WidthTire*0.5,delta+yaw);
            pt112 = obj.transTo(pt11,-WidthTire*0.5,delta+yaw);
            pt121 = obj.transTo(pt12,WidthTire*0.5,delta+yaw);
            pt122 = obj.transTo(pt12,-WidthTire*0.5,delta+yaw);
            
            obj.plotLine(pt1,pt2);
            obj.plotLine(pt11,pt12);
            obj.plotLine(pt21,pt22);  
            obj.plotLine(pt111,pt112);
            obj.plotLine(pt121,pt122);
            
            xlim([pt0.x-20,pt0.x+20])
            ylim([pt0.y-20,pt0.y+20])             
        end
        
        function pt=transTo(obj,pt0,dis,theta)
            pt.x = pt0.x + dis * cos(theta);
            pt.y = pt0.y + dis * sin(theta);            
        end
        
        function plotLine(obj,pt1,pt2,tag,ax,col)
            if nargin < 4
                tag = 'tag';
                ax = 'ax';
                col = 'b';
            end
            plot([pt1.x,pt2.x],[pt1.y,pt2.y],['.-',col],tag,ax);hold on;
        end
        
        function clearPlotVehicle(obj)
            LineObjects = findall(figure(1),'tag','ax');
            delete(LineObjects);
        end
        
    end
    
end