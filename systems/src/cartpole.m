classdef  cartpole < handle & basemodel
    properties(Access=public)
        m(2,1) double = [1; 1]; % [cart mass, pole mass]
        l double = 1;      % Pole lenght
        J double = 1;      % Pole inertia
        c double = 0.5;    % Pole center of mass
    end

    properties(Access=private)
        cart_box(5,2) double;
        cart_width_factor double = 0.4;
        cart_height_factor double = 0.2;
        com_marker_factor double = 15;
        link_thinkness_factor double = 3; % body_thickness = link_thickness_factors * min(l)
    end

    methods(Access=public)
        function obj = cartpole(varargin)
            obj.name = "Cart-Pole";
            obj.x0 = [0;0;0;0];
            obj.parse_model_inputs(varargin{:}); 
            obj.set_visual_parameters();
        end

        function dx = openloop(obj,t,x,u)
            dx([1,2],1) = x([3,4]);
            dx([3,4],1) = obj.M(t,x)\(obj.B(t,x)*u - obj.C([],x)*x([3,4]) - obj.G([],x));
        end
        
        function dx = closedloop(obj,~,x)
            dx = obj.openloop([],x,0);
        end

        function E = energy(obj,t,x)
            E = x(3:4)'*(obj.M(t,x)*x(3:4))/2 + obj.g*obj.m(2)*obj.c*cos(x(2));
        end

        function mass_matrix = M(obj,~,x)
            mass_matrix(1,1) = obj.m(1) + obj.m(2);
            mass_matrix(2,2) = obj.m(2)*obj.c^2 + obj.J;
            mass_matrix(1,2) = -obj.c*obj.m(2)*cos(x(2));
            mass_matrix(2,1) = mass_matrix(1,2);  
        end

        function centripetal = C(obj,~,x)
            centripetal(1,1) = 0;
            centripetal(1,2) = obj.c*obj.m(2)*sin(x(2))*x(4);
            centripetal(2,1) = 0;
            centripetal(2,2) = 0;
        end

        function gravitational = G(obj,~,x)
            gravitational(1,1) = 0;
            gravitational(2,1) = -obj.g*obj.c*obj.m(2)*sin(x(2));
        end

        function noninertial = h(obj,~,x)
            noninertial(1,1) = x(4)^2*obj.c*obj.m(2)*sin(x(2));
            noninertial(2,1) = -obj.g*obj.c*obj.m(2)*sin(x(2));
        end

        function inputmatrix = B(~,~,~)
            inputmatrix = [1;0];
        end

        function pose = stick_diagram(obj,x)
            % Pose 1: Cart
            pose(1).x = x(1) + obj.cart_box(:,1);
            pose(1).y = obj.cart_box(:,2);
            pose(1).z = zeros(5,1);
            % Pose 2: Pole
            pose(2).x = x(1) + [0, -obj.l*sin(x(2))];
            pose(2).y = [0, obj.l*cos(x(2))];
            pose(2).z = [0, 0];
            % Pose 3: Pole CoM
            pose(3).x = x(1) - obj.c*sin(x(2));
            pose(3).y = obj.c*cos(x(2));
            pose(3).z = 0;
        end
    end

    methods(Access=private)
        function set_visual_parameters(obj)
            cart_width = obj.l*obj.cart_width_factor;
            cart_height = obj.l*obj.cart_height_factor;
            obj.animation_axes_limits = obj.l*[-2.4 2.4; -1.2 1.2; -1 1];
            obj.cart_box = [cart_width*[1; -1; -1; 1; 1], cart_height*[1; 1; -1; -1; 1]]/2;  
            obj.body_thickness = obj.l*obj.link_thinkness_factor;
            
            obj.features{1} = [];
            obj.features{2} = [];
            obj.features{3} = struct('Marker', 'o', ...
                'MarkerSize', obj.l*obj.com_marker_factor, ...
                'MarkerFaceColor', [1 1 1]);
        end
    end
end
