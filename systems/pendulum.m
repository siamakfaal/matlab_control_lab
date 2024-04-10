classdef  pendulum < handle & basemodel
    properties(Access=public)
        m double = 1; % Mass of the bob [kg]
        l double = 1; % Lenght of the pendulum [m]
        b double = 1; % Damping coefficient [Nms]
    end

    properties(Access=public)
        x0 = [0;0];
    end

    properties(Access=private)
        bob_radius;    
        bob_radius_factor = 0.07; % bob_radius = bob_radius_factor * l
        bob_perimeter;
        bob_perimeter_div = 20;

        link_thinkness_factor = 3; % body_thickness = link_thickness_factors * l
        link_length;
    end

    methods(Access=public)
        function obj = pendulum(varargin)
            obj.name = "Pendulum";
            obj.parse_constructor_inputs(varargin{:});
            obj.shape_update_function = @(x) obj.pendulum_kinematics(x);
            
            obj.set_visual_parameters(obj.l * obj.bob_radius_factor, ...
                obj.l * obj.link_thinkness_factor, ...
                1.2*obj.l*[-1, 1; -1, 1; -1, 1]);
        end
    end

    methods(Access=public)
        function dx = openloop(obj,~,x,u)
            dx(1,1) = x(2);
            dx(2,1) = obj.g/obj.l*sin(x(1)) - obj.b*x(2) + 1/obj.m/obj.l^2*u;
        end
        
        function dx = closedloop(obj,~,x)
            dx = obj.openloop([],x,0);
        end

        function E = energy(obj,x)
            tmp = obj.m*obj.l;
            E = (tmp*obj.l*x(2)^2)/2 + tmp*obj.g*cos(x(1));
        end
    end

    methods(Access=public)
        function set_visual_parameters(obj, bob_radius, link_thickness, axes_limits)
            obj.bob_radius = bob_radius;
            obj.body_thickness = link_thickness;
            
            obj.link_length = obj.l - obj.bob_radius;
            Q = linspace(0, 2*pi, obj.bob_perimeter_div)';
            obj.bob_perimeter = bob_radius*[cos(Q), sin(Q), zeros(size(Q))];
            obj.animation_axes_limits = axes_limits;
        end
    end


    methods(Access=protected)
    end

    methods(Access=private)

        function pose = pendulum_kinematics(obj,x)
            cos_q = cos(x(1));
            sin_q = sin(x(1));
            % Pose 1 = Link
            pose(1).x = [0 -obj.link_length*sin_q];
            pose(1).y = [0 obj.link_length*cos_q];
            pose(1).z = [0 0];

            % Pose 2 = Bob
            pose(2).x = obj.bob_perimeter(:,1) - obj.l*sin_q;
            pose(2).y = obj.bob_perimeter(:,2) + obj.l*cos_q;
            pose(2).z = obj.bob_perimeter(:,3);
        end
    end
end