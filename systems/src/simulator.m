classdef simulator < handle
    properties
        model basemodel = satellite2d();
    end

    properties(Access=private)
        fontSize = 14;
        lineWidth = 1;
        animationAxis = [];
    end

    methods(Access=public)
        function obj = simulator(varargin)
            basemodel.parse_constructor_inputs(obj, varargin{:})
        end

        function sol = solve(obj, varargin)
            [tspan, x0, dynamics, control, isopenloop] = obj.parse_solve_inputs(varargin{:});
            [sol.t,sol.x] = ode45(dynamics, tspan, x0);
            sol. u = [];
            if isopenloop
                sol.u = obj.eval(control, sol); 
            end
        end

        function ax = plot(obj, sol)
            fig = figure();
            tiledlayout(fig, "flow");
            ax(1) = nexttile;
            obj.plot_trajectories(ax(1), sol.t, sol.x, "$x_%d$");
            if ~isempty(sol.u)
                ax(2) = nexttile;
                obj.plot_trajectories(ax(2), sol.t, sol.u, "$u_%d$");
            end
        end

        % function ax = compare(obj, sol, ref)
        %     fig = figure();
        %     tiledlayout(fig, "horizontal");
        % 
        %     for i = 1:size(sol.x,2)
        %         ax = nexttile;
        %         obj.set_plot_axes_properties(ax);
        %         plot(ax, sol.t, sol.x(:,i), LineWidth=obj.lineWidth, ...
        %             DisplayName=sprintf("$x_%d$",i));
        %         if i < size(ref.x,2)
        %             plot(ax, ref.t, ref.x(:,i), LineWidth=obj.lineWidth, ...
        %                 DisplayName=sprintf("$r_%d$",i));
        %         end
        %     end
        %     set(ax,XLim=sol.t([1 end]));
        %     obj.set_legend_properties(legend(ax), xlabel(ax, "$t$"));
        % end

        

        function ax = make_plot_axes(obj,ax)
            if nargin < 2, ax = axes(Parent=figure); end
            obj.set_axes_properties(ax);
        end

        function ax = make_animation_axis(obj,ax)
            if nargin < 2, ax = axes(Parent=figure); end
            obj.set_axes_properties(ax);
            [xlim, ylim, zlim] = obj.model.get_axis_limits;
            set(ax, DataAspectRatio = [1, 1, 1], ...
                Xlim = xlim, Ylim = ylim, Zlim = zlim, XTick=0, YTick=0);
        end


        function animate(obj, sol, ax)
            if nargin < 3, ax = obj.animationAxis; end
            if isempty(ax), ax = obj.make_animation_axis(); end
            obj.draw(sol.x(1,:), ax);
            k = 2;
            tic
            while k <= length(sol.t) && isvalid(ax)
                obj.draw(sol.x(k,:), ax);
                pause(sol.t(k)-toc);
                k = k + 1;
            end
        end

        function draw(obj, x, ax)
            if obj.model.isnewaxis(ax)
                obj.model.build_shape(ax);
            end
            obj.model.update_shape(x);
        end

        function set_legend_properties(obj, varargin)
            for i=1:numel(varargin)
                set(varargin{i}, Interpreter="latex", FontSize=obj.fontSize);
            end
        end
    end


    methods(Static)
        function z = eval(f, sol)
            num_time_points = numel(sol.t);
            if num_time_points < 2
                z = f(sol.t, sol.x);
            else
                m = numel(f(sol.t(1), sol.x(1,:)'));
                z = zeros(num_time_points, m);
                for k = 1:num_time_points
                    z(k,:) = f(sol.t(k), sol.x(k,:)');
                end
            end
        end
    end

    methods(Access=private)

        function ax = plot_trajectories(obj, ax, times, data, displayNameFormat)
            obj.set_axes_properties(ax);
            for i = 1:size(data, 2)
                plot(ax, times, data(:, i), 'LineWidth', obj.lineWidth, ...
                    'DisplayName', sprintf(displayNameFormat, i));
            end
            set(ax, xlim=times([1, end]));
            obj.set_legend_properties(xlabel(ax, "$t$"), legend(ax));
        end

        function set_axes_properties(obj, ax)
            set(ax, NextPlot="add", Box="on", XGrid="on", YGrid="on", ...
                TickLabelInterpreter="latex", FontSize=obj.fontSize);
        end

        function [tspan, x0, dynamics, control, isopenloop] = parse_solve_inputs(obj, varargin)
            p = inputParser;
            addOptional(p, 'tspan', obj.model.tspan);
            addOptional(p, 'x0', obj.model.x0);
            addOptional(p, 'control', []);
            parse(p, varargin{:});

            tspan = p.Results.tspan;
            x0 = p.Results.x0;
            control = p.Results.control;
            isopenloop = ~isempty(control);

            if isopenloop && ismethod(obj.model, 'openloop')
                dynamics = @(t, x) obj.model.openloop(t, x, control(t, x));
            elseif ismethod(obj.model, 'closedloop')
                dynamics = @(t, x) obj.model.closedloop(t, x);
            else
                error('Appropriate dynamics method not found in model.');
            end
        end
    end
end
