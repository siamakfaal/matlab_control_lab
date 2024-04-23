classdef  cartpole < handle & basemodel
    properties(Access=public)
        m(2,1) double = [1; 1]; % [cart mass, pole mass]
        l double = 1;      % Pole lenght
        I double = 1;      % Pole inertia
        c double = 0.5;    % Pole center of mass
    end

    properties(Access=public)
        x0(4,1) double = [0;0;0;0];
    end

    properties(Access=private)
        cart_box(5,2) double;
        cart_width_factor double = 0.4;
        cart_height_factor double = 0.2;
        com_radius_factor double = 0.05;
        com_perimeter_div double = 20;
        com_perimeter;
        link_thinkness_factor double = 3; % body_thickness = link_thickness_factors * min(l)
    end

    methods(Access=public)
        function obj = cartpole(varargin)
            obj.name = "Cart-Pole";
            obj.parse_constructor_inputs(varargin{:});
            obj.shape_update_function = @(x) obj.cartpole_kinematics(x);
            
            obj.set_visual_parameters(obj.l*obj.cart_width_factor, ...
                obj.l*obj.cart_height_factor, ...
                obj.l*obj.com_radius_factor, ...
                obj.l*obj.link_thinkness_factor, ...
                obj.l*[-2.4 2.4; -1.2 1.2; 1 1]);
        end
    end

    methods(Access=public)
        function dx = openloop(obj,~,x,u)
            dx([1,2],1) = x([3,4]);
            dx([3,4],1) = obj.M(x)\(u - obj.C(x)*x([3,4]) - obj.G(x));
        end
        
        function dx = closedloop(obj,~,x)
            dx = obj.openloop([],x,[0;0]);
        end

        function E = energy(obj,x)
            v = x(3:4);
            E = v'*(obj.M(x)*v)/2 + obj.g*obj.m(2)*obj.c*cos(x(2));
        end
        
        function mass_matrix = M(obj, x)
            mass_matrix(1,1) = obj.m(1) + obj.m(2);
            mass_matrix(2,2) = obj.m(2)*obj.c^2 + obj.I;
            mass_matrix(1,2) = -obj.c*obj.m(2)*cos(x(2));
            mass_matrix(2,1) = mass_matrix(1,2);  
        end

        function centripetal = C(obj,x)
            centripetal(1,1) = 0;
            centripetal(1,2) = obj.c*obj.m(2)*sin(x(2))*x(4);
            centripetal(2,1) = 0;
            centripetal(2,2) = 0;
        end

        function gravitational = G(obj,x)
            gravitational(1,1) = 0;
            gravitational(2,1) = -obj.g*obj.c*obj.m(2)*sin(x(2));
        end

        function noninertial = h(obj,x)
            noninertial(1,1) = x(4)^2*obj.c*obj.m(2)*sin(x(2));
            noninertial(2,1) = -obj.g*obj.c*obj.m(2)*sin(x(2));
        end
    end

    methods(Access=public)
        function set_visual_parameters(obj, cart_width, cart_height, com_radius, link_thickness,  axes_limits)
            Q = linspace(0,2*pi,obj.com_perimeter_div)';
            obj.cart_box = [cart_width*[1; -1; -1; 1; 1], cart_height*[1; 1; -1; -1; 1]]/2;  
            obj.com_perimeter = com_radius*[cos(Q), sin(Q), zeros(obj.com_perimeter_div, 1)];
            obj.body_thickness = link_thickness;
            obj.animation_axes_limits = axes_limits;
        end
    end


    methods(Access=protected)
    end

    methods(Access=private)

        function pose = cartpole_kinematics(obj,x)
            % Pose 1: Cart
            pose(1).x = x(1) + obj.cart_box(:,1);
            pose(1).y = obj.cart_box(:,2);
            pose(1).z = zeros(5,1);

            % Pose 2: Pole
            pose(2).x = x(1) + [0, -obj.l*sin(x(2))];
            pose(2).y = [0, obj.l*cos(x(2))];
            pose(2).z = [0, 0];

            % Pose 3: Pole CoM
            pose(3).x = x(1) -obj.c*sin(x(2)) + obj.com_perimeter(:,1);
            pose(3).y = obj.c*cos(x(2)) + obj.com_perimeter(:,2);
            pose(3).z = obj.com_perimeter(:,3);
        end
    end
end
