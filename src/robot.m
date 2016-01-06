classdef robot
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        R = @(a,q) quat2dcm([cos(q/2) sin(-q/2).*a]); % a is unit vector. to inertial
    end
    
    properties
        axis
        origin
        joint
        dof
        Te
        Tc
    end
    
    methods
        function obj = robot()
            % translate rotate
            obj.axis = [0 0 1; 0 1 0; 0 1 0; 0 0 1];
            obj.origin = [0 0 20e-3; 17.5e-3 0 50e-3; 0 0 146e-3; 190e-3 0 0];
            obj.joint = zeros(1, 4);
            obj.dof = 4;
            obj.Te = [obj.R([0 1 0], pi/4) [50e-3 0 -50e-3].'; 0 0 0 1];
            obj.Tc = [obj.R([1 0 1], pi) [0 0 100e-3].'; 0 0 0 1];

        end
        
        function J = jac0s(obj, q)
            syms q1 q2 q3 q4
            qs = [q1 q2 q3 q4];
            
            Ts = obj.fk(qs);
            Ss = Ts(1:3,4);
            Js = jacobian(Ss, qs);
            J = eval(subs(Js, qs, q));
        end
        
        function T = fk(obj, q)
            T = eye(4);
            for k = 1 : obj.dof
                T = T * [obj.Rns(obj.axis(k,:), q(k)) obj.origin(k,:).'; 0 0 0 1];
            end
            T = T * obj.Te;
        end
        
        function J = jac0(obj, q)
            
            J = zeros(6, obj.dof);
            idx = 1 : obj.dof;
            for k = 1 : obj.dof
                J(:,k) = obj.apply_jac0(q, idx==k);
            end
        end
        
        function sd = apply_jac0(obj, q, qd)
            
            w = zeros(3,1);
            v = zeros(3,1);
            Ri = eye(3);
            
            for k = 1 : obj.dof
                w = w + Ri * qd(k)*obj.axis(k,:).';
                v = v + Ri * cross(w, obj.origin(k,:).');
                
                Ri = robot.R(obj.axis(k,:), q(k)) * Ri;
            end
            
            sd = [v; w];
        end
        
        function J = jacn(obj, q)
                       
            J = obj.jac0(q);
            T = obj.fk(q);
            
            J = [T(1:3,1:3).'*J(1:3,:); T(1:3,1:3).'*J(4:6,:)];
        end
        
        function plot(obj, qq, Ts)

            axt = robot.plot_triad(eye(4), 0);
            
            hold on
            ax = plot3(0, 0, 0, 'o-');
            hold off
            axis square
            axis([-1 1 -1 1 -1 1]/2);
            xlabel('x'); ylabel('y'); zlabel('z')
            TT = zeros(4,4,obj.dof+2);
            
            for p = 1 : size(qq, 1)
                T = eye(4);
                q = qq(p,:);
                for k = 1 : obj.dof
                    TT(:,:,k) = T;
                    T = T * [obj.R(obj.axis(k,:), q(k)) obj.origin(k,:).'; 0 0 0 1];
                end
                TT(:,:,k+1) = T;
                T = T * obj.Te;
                TT(:,:,k+2) = T;

                robot.plot_triad(T, 0.1, axt);
                
                set(ax, 'XData', squeeze(TT(1,4,:)), 'YData', squeeze(TT(2,4,:)), 'ZData', squeeze(TT(3,4,:)));
                pause(Ts);
            end
            
        end
        
        function qd = control_law(obj, Tg, q)
%             Tg = obj.fk([1 0 0 0]);
            Te = obj.fk(obj.joint);

            T = Te \ Tg; % end-effector frame
            S = obj.get_pose(T);
            
            quat = S(4:7);
            theta = asin(sin(2*acos( quat(:,1) )));
            vec = quat(:,2:4) ./ repmat(sqrt(1-quat(:,1).^2), [1 3]);
            w = theta * vec;
            
            v = T(1:3,4)'; %% ee frame
            v0 = Tg(1:3,4).' - Te(1:3,4).'; %% w frame
            
            dSn = [v w]; % ee frame
            Jn = obj.jacn(q);
            J0 = obj.jac0(q);
            
            J = [J0(1:3,:); Jn(4:6,:)];
            dS = [v0 w];
            
%             dQ = J \ dS.';
            
            % select important rows
            mask = [1 1 1 0 0 1] == 1;
            Jc = J(mask,:);
            dSc = dS(mask);
            dQ = Jc \ dSc.';
            
            lambda = 0.01;
            qd = lambda * dQ.';

%             figure
%             obj.plot(obj.joint, 0)
%             hold on
%             robot.plot_triad(Tg, 0.1);
        end
            
    end
    
    methods(Static)
        
        function main()
            %% Test plot
            clear; clc
            
            r = robot();
            q = filter(ones(50,1)/50, 1, 3*(pi*rand(100,4) - pi/2));
            
            figure
            set(gcf, 'Position', [250 100 550 900])
            
            subplot(212); plot(q)
            
            subplot(211);
            
            Tg = r.fk(0.2*rand(1,4));
            robot.plot_triad(Tg, 0.1);
            
            hold on
            
            r.plot(q, 5/100)
            
            %% Test control law simple
            clear; clc; close all
            
            r = robot();
            
            Tg = r.fk([1 0 0 0]);
            
            r.control_law(Tg, zeros(1, 4))
            
            %% Test control law
            r = robot();
            
            Tg = r.fk([0.5 -0.5 0 0]);
            
            Fs = 10;
            t = 0:(1/Fs):10;
            
            q = zeros(1, 4);
            qq = zeros(length(t), 4);
            for k = 1 : length(t)
                dq = r.control_law(Tg, q);
                q = q + dq;
                qq(k,:) = q;
            end
            
            robot.plot_triad(Tg, 0.1);
            hold on
            r.plot(qq, 1/Fs)
            
            %% Test jacobians and fk
            r = robot();
            q = [0 0 0 0];
            r.fk(q);
            r.jac0(q)
            r.jacn(q)

        end
        
        function S = get_pose(T)
            
            quat = dcm2quat(T(1:3,1:3).');
            S = [T(1:3,4).' quat]; 
        end
        
        function angle = calc_ankleAngle(shin, foot)
                       
            q12 = quatmultiply( quatinv(shin.quat), foot.quat ); 
            q12 = q12 ./ repmat(sqrt(sum(q12.*q12,2)), [1 4]); % normalize
%             q12 = ZTools.fixQuat( q12 );
            theta = asin(sin(2*acos( q12(:,1) )));
            vec = q12(:,2:4) ./ repmat(sqrt(1-q12(:,1).^2), [1 3]);
            angle = vec .* repmat(theta, [1 3]);
            
%             figure;
%             subplot(311); plot(q12)
%             subplot(312); plot(theta)
%             subplot(313); plot(vec)
        end
        
        function ax = plot_triad(T, scale, varargin)
            
            triad = [[0 0 0; 1 0 0]; [0 0 0; 0 1 0]; [0 0 0; 0 0 1]]*scale;               
            triadT = (T * [triad ones(6,1)]').';
            
            if nargin == 2
                tmp = reshape(triadT(:,1:3), [2 3 3]);
                ax = plot3(squeeze(tmp(:,:,1)), squeeze(tmp(:,:,2)), squeeze(tmp(:,:,3)));
                
                set(ax(1), 'Color', [0.0000e+000 447.0000e-003 741.0000e-003]);
                set(ax(2), 'Color', [850.0000e-003 325.0000e-003 98.0000e-003]);
                set(ax(3), 'Color', [929.0000e-003 694.0000e-003 125.0000e-003]);
                
            else
                ax = varargin{1};
                set(ax(1), 'XData', triadT(1:2,1), 'YData', triadT(1:2,2), 'ZData', triadT(1:2,3));
                set(ax(2), 'XData', triadT(3:4,1), 'YData', triadT(3:4,2), 'ZData', triadT(3:4,3));
                set(ax(3), 'XData', triadT(5:6,1), 'YData', triadT(5:6,2), 'ZData', triadT(5:6,3));
            end  
        end
        
        function rot = Rns(a, q)
            if all(a == [1 0 0])
                rot = [1 0 0; 0 cos(q) -sin(q); 0 sin(q) cos(q)];
            elseif all(a == [0 1 0])
                rot = [cos(q) 0 sin(q); 0 1 0; -sin(q) 0 cos(q)];
            elseif all(a == [0 0 1])
                rot = [cos(q) -sin(q) 0; sin(q) cos(q) 0; 0 0 1];
            else
                error('nope')
            end
        end
    end
    
end

