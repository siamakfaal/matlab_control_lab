classdef  satellite2d < handle & basemodel
    properties(Access=public)
        I double = 1;      % Satellite inertia
    end

    properties(Access=public)
        x0(2,1) double = [0;0];
    end

    properties(Access=private)
        body_scale double = 1;
        body_perimeter;
        heading_line;
    end

    methods(Access=public)
        function obj = satellite2d(varargin)
            obj.name = "Satellite 2d Model";
            obj.parse_constructor_inputs(varargin{:});
            obj.shape_update_function = @(x) obj.satellite2d_kinematics(x);
            
            obj.set_visual_parameters();
        end
    end

    methods(Access=public)
        function dx = openloop(obj,~,x,u)
            dx = [x(2); u/obj.I];
        end
        
        function dx = closedloop(obj,~,x)
            dx = obj.openloop([],x,0);
        end

        function E = energy(obj,x)
            E = obj.I*x(2)^2/2;
        end
        
        function mass_matrix = M(obj,~)
            mass_matrix = obj.I;
        end

        function centripetal = C(~,~)
            centripetal = 0;
        end

        function gravitational = G(~,~)
            gravitational = 0;
        end

        function noninertial = h(~,~)
            noninertial = 0;
        end
    end

    methods(Access=public)
        function set_visual_parameters(obj)
            a = obj.body_scale/5;
            b = obj.body_scale/10;
            c = obj.body_scale;
            Q = [a -a; a 0; a+b 0; a+b -b; c -b; c b; a+b b; a+b 0; a 0; a a];
            obj.body_perimeter = [Q; -Q; Q(1,:)];
            obj.heading_line = [-a a-b; a a-b];
            obj.animation_axes_limits = c*1.2*[-1 1; -1 1];
        end
    end


    methods(Access=protected)
    end

    methods(Access=private)

        function pose = satellite2d_kinematics(obj,x)
            R = obj.rot2(x(1));
            body_points = obj.body_perimeter*R';
            heading = obj.heading_line*R';
            % Pose 1: Satellite Body
            pose(1).x = body_points(:,1);
            pose(1).y = body_points(:,2);
            pose(1).z = zeros(21,1);
            % Pose 2: Heading line
            pose(2).x = heading(:,1);
            pose(2).y = heading(:,2);
            pose(2).z = zeros(2,1);
        end
    end
end
