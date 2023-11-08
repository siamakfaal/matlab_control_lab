classdef pendulum < handle
    properties(Access=public)
        m(1,1) double;  % Mass of the bob [kg]
        l(1,1) double;  % Link length [m]
        g(1,1) double;  % Gravitational acceleration [m/s^2]
    end

    properties(Access=public)
        dynamics function_handle
        energy function_handle
    end

    properties(Access=private)
        display_limit(2,1) double
        axis
        link
        bob
    end

    methods(Access=public)
        function obj = pendulum(m, l, g)
            obj.m = m;
            obj.l = l;
            obj.g = g;

            obj.dynamics = @(x, u) [x(2); g/l*sin(x(1)) + 1/m/l^2*u];
            obj.energy = @(x) 0.5*m*l^2*x(2)^2 + m*g*l*(cos(x(1)) - 1);

            obj.display_limit = 1.2*[-l l];
        end

        function info(obj)
            t = sym('t','real');
            x = sym('x%d',[2,1],'real');
            pretty(obj.dynamics(t,x));
        end

        function [t, x] = solve(obj, tspan, x0, control, xd)
            if length(tspan) < 2
                tspan = [0 tspan];
            end
            if nargin < 4
                control = @(t,x) 0;
            end
            if nargin < 5
                xd = [0;0];
            end

            x0_ = [x0; 0];

            dx_ = @(x, xd, u)[obj.dynamics(x, u); xd(1) - x(1)];
                
            [t, x] = ode45(@(t,x) dx_(x, xd, control(t,x)), tspan, x0_);
        end

    end

    methods(Access=public)
        function show(obj, x)
            if isempty(obj.axis) || ~isvalid(obj.axis)
                obj.draw([0, 0.4, 0.7], 20, 4);
            end
            x_bob = -obj.l*sin(x(1));
            y_bob = obj.l*cos(x(1));
            set(obj.link, XData=[0, x_bob], YData=[0, y_bob]);
            set(obj.bob, XData=x_bob, YData=y_bob);
        end

        function animate(obj, t, x)
            if isempty(obj.axis) || ~isvalid(obj.axis)
                obj.draw([0, 0.4, 0.7], 20, 4);
            end

            tic
            for i = 1:length(t)
                obj.show(x(i,:));
                pause(t(i)-toc);
            end
        end


        function draw(obj, color, bobsize, thickness)
            if isempty(obj.axis) || ~isvalid(obj.axis)
                obj.buildaxis();
            end

            obj.link = plot(obj.axis, 0, 0,...
                LineWidth=thickness, Color=color);

            obj.bob = plot(obj.axis, 0, 0,...
                Color=color,...
                Marker="o", MarkerSize=bobsize, ...
                MarkerFaceColor=color);
        end

        function plot(~, t, x, u)
            if nargin > 3
                u_ = zeros(length(t),1);
                for i = 1:length(t)
                    u_(i) = u(t(i), x(i,:)');
                end
            end
            numrows = nargin - 1;

            fig = figure();
            ax = gobjects(numrows, 1);
            ylabels = {"$q(t)$", "$\dot{q}(t)$", "$u(t)$"};

            for i = 1:numrows
                ax(i) = subplot(numrows,1,i,Parent=fig);
                set(ax(i), Box="on", NextPlot="add", XLim=t([1 end]),...
                    TickLabelInterpreter="latex",...
                    FontSize=14, XGrid="on", YGrid="on");
                xlabel(ax(i), "$t$", Interpreter="latex",...
                    FontSize=14);
                ylabel(ax(i), ylabels{i}, Interpreter="latex");
            end
            plot(ax(1), t, x(:,1), LineWidth=2);
            plot(ax(2), t, x(:,2), LineWidth=2);
            if nargin > 3
                plot(t, u_, LineWidth=2);
            end
        end
    end

    methods(Access=private)

        function buildaxis(obj)
            obj.axis = axes(figure(), NextPlot="add", Box="on",...
                DataAspectRatio=[1, 1, 1],...
                XLim=obj.display_limit, YLim=obj.display_limit,...
                XTick=0, YTick=0, XGrid="on", YGrid="on");
        end

    end
end