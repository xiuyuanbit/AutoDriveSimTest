classdef LqrController < handle
    properties
        L_;
        Q_;
        R_;
    end
    
    methods
        function obj=LqrController(L)
            obj.L_= L;
            obj.Q_ = eye(4);
            obj.Q_(1,1) = 39;
            obj.Q_(2,2) =0.8;
            obj.Q_(3,3) = 6.6;
            obj.Q_(4,4) = 37;
            
            obj.R_ = eye(1)* 1;
        end
        
        function [delta,ind,e,th_e] =  control(obj,state,traj, pe, pth_e,dt)
            %             x,y,v,yaw,cx,cy,cyaw,ck,
            x = state.loc.x;
            y = state.loc.y;
            yaw = state.loc.yaw;
            v = state.v;
            
            cx = traj.x;
            cy = traj.y;
            cyaw = traj.yaw;
            ck = traj.k;
            
            L = obj.L_;
            Q = obj.Q_;
            R = obj.R_;
            [ind, e] = obj.calc_target_index(state,traj);
            %ind   
            
            % e= error_check(x, y, yaw,e,cx,cy,ind)
            k =ck(ind);
            th_e = obj.pipi(yaw -cyaw(ind));
            A = zeros(4,4);
            
            A(1,1) = 1; A(1,2) = dt; A(2,3) = v; A(3,3) = 1; A(3,4) = dt;
            B= zeros(4,1);
            B(4,1) = v/L;
            [K,iter] = dlqr(A,B,Q,R);
            x = zeros(4,1);
            x(1,1)=e; x(2,1)= (e-pe)/dt; x(3,1) = th_e; x(4,1) = (th_e - pth_e)/dt;
            ff = atan(L * (k));
            fb = obj.pipi(-K * x);
            delta =   1*ff + 1*fb;
            %             ff
            %             fb
            %             iter
        end
        function [angle] = pipi(obj,angle) % the unit of angle is in rad;
            if (angle > pi)
                angle =  angle - 2*pi;
            elseif (angle < -pi)
                angle = angle + 2*pi;
            else
                angle = angle;
            end
        end
        function [Xn,iter] = solve_DARE(obj,A,B)
            Q = obj.Q_;
            R = obj.R_;
            
            X = Q;
            maxiter = 150;
            epsilon = 0.01;
            for i = 1:maxiter
                Xn = A' * X * A - A' * X * B * ((R + B' * X * B) \ B') * X * A +Q;
                if abs(Xn - X) <= epsilon
                    X = Xn;
                    break;
                end
                X = Xn;
            end
            iter = i;
        end
        function [K,iter] = dlqr (obj,A,B)
            Q = obj.Q_;
            R = obj.R_;
            [X,iter] = solve_DARE(A,B,Q,R);
            K = (B' * X * B + R) \ (B' * X * A);
        end
        function [ind, error] = calc_target_index(obj,state,traj)
            x = state.loc.x;
            y = state.loc.y;
            cx = traj.x;
            cy = traj.y;
            cyaw = traj.yaw;
            %x,y, cx,cy,cyaw;
            N =  length(cx);
            Distance = zeros(N,1);
            for i = 1:N
                Distance(i) =  sqrt((cx(i)-x)^2 + (cy(i)-y)^2);
            end
            [value, location]= min(Distance);
            ind = location;
            
            dx1 =  cx(ind) - x;
            dy1 =  cy(ind) - y ;
            angle = obj.pipi(cyaw(ind)-atan(dy1/dx1));
            heading = cyaw(ind)*180/pi;
            if y<cy(ind)
                error = -value;
            else
                error = value;
            end
            % error = value;
        end
    end
end