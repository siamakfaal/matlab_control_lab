classdef simulator < handle & basemodel

    %-- Properties --------------------------------------------------------
    properties(Access=public)
        model;
    end

    properties(Access=private)
    end

    %-- Methods -----------------------------------------------------------
    methods(Access=public) % Constructors
        function obj = simulator(varargin)
            obj.parse_constructor_inputs(varargin{:});
        end

        function sol = solve(obj, varargin)

            [tspan, x0, dynamics, control, isopenloop] = obj.parse_solve_inputs(varargin{:});

            [sol.t,sol.x] = ode45(dynamics, tspan, x0);

            sol.u = [];
            if isopenloop
                sol.u = zeros(length(sol.t),length(control(sol.t(1),sol.x(1,:)')));
                for k=1:length(sol.t)
                    sol.u(k,:) = control(sol.t(k),sol.x(k,:)')';
                end
            end
        end

    end

    methods(Access=public)
        function ax = plot(obj, sol)
            fig = figure();
            tiledlayout(fig, "horizontal");
            ax = nexttile;
            obj.set_plot_axes_properties(ax);
            for i = 1:size(sol.x,2)
                plot(ax, sol.t, sol.x(:,i), LineWidth=obj.line_width, ...
                    DisplayName=sprintf("$x_%d$",i));
            end

            set(ax,XLim=sol.t([1 end]));
            obj.set_legend_properties(legend(ax));
            obj.set_legend_properties(xlabel("$t$"));

            if ~isempty(sol.u)
                ax(2) = nexttile;
                obj.set_plot_axes_properties(ax(2));
                for i = 1:size(sol.u,2)
                    plot(ax(2), sol.t, sol.u(:,i), LineWidth=obj.line_width, ...
                        DisplayName=sprintf("$u_%d$",i));
                end
                set(ax(2),XLim=sol.t([1 end]));
                obj.set_legend_properties(legend(ax(2)));
                obj.set_legend_properties(xlabel("$t$"));
            end
        end

        function ax = compare(obj, sol, ref)
            fig = figure();
            tiledlayout(fig, "horizontal");

            for i = 1:size(sol.x,2)
                ax = nexttile;
                obj.set_plot_axes_properties(ax);
                plot(ax, sol.t, sol.x(:,i), LineWidth=obj.line_width, ...
                    DisplayName=sprintf("$x_%d$",i));
                if i < size(ref.x,2)
                    plot(ax, ref.t, ref.x(:,i), LineWidth=obj.line_width, ...
                        DisplayName=sprintf("$r_%d$",i));
                end
            end
            set(ax,XLim=sol.t([1 end]));
            obj.set_legend_properties(legend(ax));
            obj.set_legend_properties(xlabel("$t$"));
        end

        function ax = plotaxes(obj,ax)
            if nargin < 2
                ax = axes(Parent=figure);
            end
            obj.set_plot_axes_properties(ax);
        end

        function animate(obj, sol, ax)
            if nargin < 3
                ax = obj.model.animation_axis;
            end
            tic;
            k = 1;
            while k <= length(sol.t)
                ax = obj.model.draw(sol.x(k,:), ax);
                pause(sol.t(k)-toc);
                if ~isvalid(ax)
                    return
                end
                k = k + 1;
            end
        end
    end

    methods(Access=private)


        function [tspan, x0, dynamics, control, isopenloop] = parse_solve_inputs(obj, varargin)

            % tspan definition
            if ~isempty(varargin)
                tspan = varargin{1};
            elseif isprop(obj.model, 'tspan')
                tspan = obj.model.tspan;
            else
                error("tspan is not provided as an input.");
            end

            if isempty(tspan)
                error("tspan can not be empty.");
            end

            % x0 definition
            if length(varargin) > 1
                x0 = varargin{2};
            elseif isprop(obj.model, 'x0')
                x0 = obj.model.x0;
            else
                error("x0 is not provided as an input.");
            end

            if isempty(x0)
                error("x0 can not be empty.");
            end

            % Control
            control = [];
            if length(varargin) > 2
                control = varargin{3};
                if ismethod(obj.model, 'openloop') && isa(control, 'function_handle')
                    dynamics = @(t,x) obj.model.openloop(t, x, control(t, x));
                    isopenloop = true;
                elseif ismethod(obj.model, 'closedloop')
                    warning("Model: %s does not have openloop method. Deafulting to closedloop dynamics.", obj.model.name);
                    dynamics = @(t,x) obj.model.closedloop(t,x);
                    isopenloop = false;
                else
                    error("Model: %s does not have a closedloop method and control is not properly defined.", obj.model.name);
                end
            else
                % Default to closedloop if no control is specified and method exists
                if ismethod(obj.model, 'closedloop')
                    dynamics = @(t,x) obj.model.closedloop(t,x);
                    isopenloop = false;
                else
                    error("Model: %s does not have a closedloop method", obj.model.name);
                end
            end
        end
    end

end
