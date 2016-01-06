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
    end
    
    methods
        function obj = robot()
            obj.axis = [0 0 1; 0 1 0; 0 1 0; 0 0 1];
            obj.origin = [0 0 0; 17.5e-3 0 70e-3; 0 0 146e-3; 190e-3 0 0];
            obj.joint = zeros(4,1);
            obj.dof = 4;
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
                       
            Ri = eye(3);
            for k = 1 : obj.dof
                Ri = Ri * robot.R(obj.axis(k,:), q(k));
            end
            
            J = obj.jac0(q);
            J = [Ri.'*J(1:3,:); Ri.'*J(4:6,:)];
        end
    end
    
    methods(Static)
        
        function main()
            r = robot()
            
            q = [1 0 0 0];
            qd = [0 0 0 1];
            r.jac0(q)
            r.jacn(q)
        end
        
    end
    
end

