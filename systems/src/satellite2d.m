classdef  satellite2d < handle & basemodel
    properties(Access=public)
        J double = 1; % Satellite inertia
    end

    properties(Access=private)
        body_scale double = 1;
        body_perimeter;
        heading_line;
    end

    methods
        function obj = satellite2d(varargin)
            obj.name = "Satellite 2d Model";
            obj.x0 = [0;0];
            obj.parse_model_inputs(varargin{:});
            obj.set_visual_parameters();
        end

        function dx = openloop(obj,~,x,u), dx = [x(2); u/obj.J]; end
        function dx = closedloop(obj,~,x), dx = obj.openloop([],x,0); end
        function E = energy(obj,~,x), E = obj.J*x(2)^2/2; end
        function mass_matrix = M(obj,~,~), mass_matrix = obj.J; end
        function nonconservative = h(~,~,~), nonconservative = 0; end
        function inputmatrix = B(obj,~,~), inputmatrix = 1/obj.J; end

        function pose = stick_diagram(obj,x)
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

    
    methods(Access=private)
        function set_visual_parameters(obj)
            a = obj.body_scale/5;
            b = obj.body_scale/10;
            c = obj.body_scale;
            Q = [a -a; a 0; a+b 0; a+b -b; c -b; c b; a+b b; a+b 0; a 0; a a];
            obj.body_perimeter = [Q; -Q; Q(1,:)];
            obj.heading_line = [-a a-b; a a-b];
            obj.animation_axes_limits = c*1.2*[-1 1; -1 1; -1 1];
        end
    end
end
