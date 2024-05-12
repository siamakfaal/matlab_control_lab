classdef  rrarm < handle & basemodel
    properties(Access=public)
        m(2,1) double = [1; 1];
        l(2,1) double = [1; 1];
        I(2,1) double = [1; 1];
        c(2,1) double = [0.5; 0.5];
    end

    properties(Access=private)
        joint_radius_factor double = 15; % joint_radius = joint_radius_factor * min(l)
        joint_radius double;
        joint_color(1,3) double = [1, 1, 1];
        last_link_length double;
        link_thinkness_factor double = 3; % body_thickness = link_thickness_factors * min(l)
        gripper = struct('points',[],'a',0.2,'b',0.2,'c',0.3,'d',0.1);
    end

    methods(Access=public)
        function obj = rrarm(varargin)
            obj.name = "Planar RR Arm";
            obj.x0 = [0;0;0;0];
            obj.parse_model_inputs(varargin{:});            
            obj.set_visual_parameters();
        end
    end

    methods(Access=public)
        function dx = openloop(obj,t,x,u)
            dx([1,2],1) = x([3,4]);
            dx([3,4],1) = obj.M(t,x)\(obj.B(t,x)*u - obj.h(t,x));
        end
        
        function dx = closedloop(obj,~,x)
            dx = obj.openloop([],x,[0;0]); 
        end
        
        function E = energy(obj,t,x)
            % TODO: add potential
            E = x(3:4)'*(obj.M(t,x)*x(3:4))/2;
        end

        function mass_matrix = M(obj,~,x)
            mass_matrix(1,1) = obj.m(1)*obj.c(1)^2 + obj.I(1) + obj.m(2)*(obj.l(1)^2 + obj.c(2)^2 + 2*obj.l(1)*obj.c(2)*cos(x(2))) + obj.I(2);
            mass_matrix(2,2) = obj.m(2)*obj.c(2)^2 + obj.I(2);
            mass_matrix(1,2) = obj.m(2)*obj.l(1)*obj.c(2)*cos(x(2)) + obj.m(2)*obj.c(2)^2 + obj.I(2);
            mass_matrix(2,1) = mass_matrix(1,2);  
        end

        function nonconservative  = h(obj,~,x)
            nonconservative = obj.C(x)*x(3:4) + obj.G(x);
        end

        function inputmatrix = B(~,~,~)
            inputmatrix = eye(2);
        end

        function centripetal = C(obj,x)
            h = obj.m(2)*obj.l(1)*obj.c(2)*sin(x(2));
            centripetal(1,1) = -h*x(4);
            centripetal(1,2) = -h*x(3) - h*x(4);
            centripetal(2,1) = h*x(3);
            centripetal(2,2) = 0;
        end

        function gravitational = G(obj,x)
            gravitational(1,1) = obj.m(1) * obj.c(1) * obj.g * cos(x(1)) + obj.m(2) * obj.g * (obj.l(1) * obj.c(2) * cos(x(1) + x(2)) + obj.l(1) * cos(x(1)));
            gravitational(2,1) = obj.m(2) * obj.c(2) * obj.g * cos(x(1) + x(2));
        end

        function pose = stick_diagram(obj,x)
            R2 = obj.rot2(x(1)+x(2));
            p0 = [0;0];
            p1 = p0 + obj.rot2(x(1))*[obj.l(1);0];
            p2 = p1 + R2*[obj.l(2);0];
            pg = p1 + R2*[obj.last_link_length;0]; 
            gr = obj.gripper.points*R2';

            % Pose 1: Second Link
            pose(1).x = [p1(1), pg(1)];
            pose(1).y = [p1(2), pg(2)];
            pose(1).z = [0,0];

            % Pose 2: First Link
            pose(2).x = [p0(1), p1(1)];
            pose(2).y = [p0(2), p1(2)];
            pose(2).z = [0, 0];

            % Pose 3: Gripper
            pose(3).x = pg(1) + gr(:,1);
            pose(3).y = pg(2) + gr(:,2);
            pose(3).z = zeros(5,1);

            % Pose 4: Gripper Point
            pose(4).x = p2(1);
            pose(4).y = p2(2);
            pose(4).z = 0;
            
        end
    end

    methods(Access=private)
        function set_visual_parameters(obj)
            obj.joint_radius = min(obj.l) * obj.joint_radius_factor;
            obj.body_thickness = min(obj.l) * obj.link_thinkness_factor;
            obj.animation_axes_limits = 1.2*sum(obj.l)*[-1, 1; -1, 1; -1, 1];
            
            obj.features{1} = [];
            obj.features{2} = struct('Marker', 'o', ...
                'MarkerSize', obj.joint_radius, ...
                'MarkerFaceColor', obj.joint_color);
            obj.features{3} = [];
            obj.features{4} = struct('Marker', '.', ...
                'MarkerSize', obj.joint_radius/2);
        
            gr = obj.gripper;
            obj.gripper.points = min(obj.l)*[gr.b+gr.c, -gr.d;
                gr.b, -gr.a; 
                0, 0; 
                gr.b, gr.a;
                gr.b+gr.c, gr.d];

            obj.last_link_length = obj.l(2)-gr.b;
        end
    end
end
