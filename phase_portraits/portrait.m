% PORTRAIT Class for drawing phase portraits of dynamical systems.
%
% This class provides functionalities for visualizing the trajectories of systems
% of differential equations in phase space, offering customizable options for the
% appearance of both the vector field and the trajectories.
%
% Properties:
%   canvas - A matrix defining the dimensions of the phase space to visualize.
%            Format: [xmin, xstep, xmax; ymin, ystep, ymax].
%            Default is [-1, 0.1, 0.1; -1, 0.1, 1]
%
%   xticks - A vector specifying the x-axis ticks for the phase portrait.
%
%   yticks - A vector specifying the y-axis ticks for the phase portrait.
%
%   quiver_color_map - Color map used for the quiver plot representing the vector field.
%                      Default is 'abyss'.
%
%   trajectory_color - RGB vector specifying the color of the trajectories.
%                      Default is [0, 0.447, 0.741].
%
%   quiver_line_width - Line width of the quiver plot vectors. Default is 2.
%
%   quiver_arrow_scale - Scale of the arrowheads in the quiver plot. Default is 0.1.
%
%   quiver_max_headsize - Maximum size of the arrowheads in the quiver plot. Default is 1.
%
%   trajectory_line_width - Line width of the trajectories. Default is 1.
%
%   label_font_size - Font size for the labels on the plot. Default is 25.
%
% Methods:
%   portrait - Constructor for the portrait class. Initializes a new instance with
%              a specified canvas and optional parameters for customizing the appearance.
%              Syntax: obj = portrait(canvas, 'PropertyName', PropertyValue, ...)
%
%   icgrid - Generates a grid of initial conditions based on the specified canvas dimensions
%            or on specified boundaries according to a direction string. The direction string
%            can contain the letters 'r' (right), 'l' (left), 'u' (up), 'd' (down), 'h' (horizontal),
%            and 'v' (vertical), indicating where the initial conditions should be placed.
%            x0 = obj.icgrid() generates a grid covering the entire canvas
%            x0 = obj.icgrid(directionString) Where 'directionString' is an optional argument
%               specifying the boundary directions for initial conditions.
%               If omitted, a grid covering the entire canvas is generated. omitted, a grid covering the entire canvas is generated.
%
%   draw - Draws the phase portrait for a given system of differential equations
%          using the specified initial conditions and over a given time interval.
%          Syntax: obj.draw(f, x0, tspan)
%          Where 'f' is a function handle to the system of differential equations,
%          'x0' is a matrix of initial conditions, and 'tspan' is the time interval
%          as a two-element vector [tstart, tend].
%
%   trajectories - Computes and plots the trajectories of the dynamical system for the
%                  given initial conditions over the specified time span. This method
%                  is typically called within the draw method.
%                  Syntax: obj.trajectories(f, x0, tspan)
%
%   directions - Generates a directional field (quiver plot) for the dynamical system
%                on the specified canvas. This method is typically called within the
%                draw method to overlay the directional field on the phase portrait.
%                Syntax: obj.directions(f)
%
%   buildaxes - Constructs and formats the axes for the phase portrait according to the
%               class properties such as xticks, yticks, and label_font_size. This method
%               ensures the phase portrait is properly scaled and labeled.
%               Syntax: obj.buildaxes()
%
%   findequilibria - Numerically finds the equilibrium points of the dynamical system within
%                    the specified canvas. Equilibrium points are those where the derivative
%                    (velocity) of the system is zero.
%                    Syntax: eqPoints = obj.findequilibria(f)
%                    Where 'f' is the function handle to the system of differential equations,
%                    and 'eqPoints' is an array of points (coordinates) where the system is in equilibrium.
%
% Example:
%   % Define a dynamical system
%   f = @(t, x)[x(1)+x(2)+x(1)^2+x(2)^2; x(1)-x(2)-x(1)^2+x(2)^2];
%
%   % Initialize the phase portrait object
%   phaseplane = portrait([-2.5, 0.2, 1; -2.5, 0.2, 1]);
%
%   % Generate a grid of initial conditions on the right and upper boundaries
%   x0 = phaseplane.icgrid('ru');
%
%   % Find equilibrium points of the system
%   eqPoints = phaseplane.findequilibria(f);
%
%   % Draw the phase portrait
%   phaseplane.draw(f, x0, [0, 10]);
%
% See also quiver, plot

classdef portrait < handle
    properties(Access=public)
        canvas
        xticks = [];
        yticks = [];
    end

    properties(Access=public)
        quiver_color_map = abyss;
        trajectory_color = [0, 0.447, 0.741];
        quiver_line_width = 2;
        quiver_arrow_scale = 0.1;
        quiver_max_headsize = 1;
        trajectory_line_width = 1;
        label_font_size = 25;
    end

    methods(Access=public)
        function obj = portrait(canvas, varargin)
            if nargin < 1
                canvas = [-1 0.1 1; -1 0.1 1];
            end
            obj.canvas = struct( ...
                'x1', struct('min', canvas(1,1), 'max', canvas(1,3), 'res', canvas(1,2)), ...
                'x2', struct('min', canvas(2,1), 'max', canvas(2,3), 'res', canvas(2,2)));
            
            % Loop through pairs of input arguments
            for i = 1:2:nargin-1
                propertyName = varargin{i};
                if ~ischar(propertyName) && ~isstring(propertyName)
                    error('Property names must be strings.');
                end
                
                % Check if the property exists
                if isprop(obj, propertyName)
                    % Set the property value
                    obj.(propertyName) = varargin{i+1};
                else
                    error('Invalid property name: %s', propertyName);
                end
            end

        end

        function x_eq = findequilibria(obj, f, serachres, tol)
            if nargin < 3
                x0 = obj.icgrid();
            else
                if length(serachres) < 2
                    serachres = [serachres serachres];
                end
                x1g = (obj.canvas.x1.min:serachres(1):obj.canvas.x1.max)';
                x2g = (obj.canvas.x2.min:serachres(2):obj.canvas.x2.max)';
                [x1, x2] = meshgrid(x1g, x2g);
                x0 = [x1(:), x2(:)];
            end

            options = optimoptions('fmincon',...
                Algorithm="interior-point",...
                OptimalityTolerance=1e-10,...
                MaxFunctionEvaluations=1e6,...
                Display='none');
            cost = @(x) norm(f(0,x));
            lb = [obj.canvas.x1.min; obj.canvas.x2.min];
            ub = [obj.canvas.x1.max; obj.canvas.x2.max];
            x_eq = [];
            for i = 1:size(x0,1)
                [x_candidate, f_x] = fmincon(cost,x0(i,:)', ...
                    [],[], [],[], ... % A, b, Aeq, beq
                    lb,ub, ...
                    [], ... % Nonlinear Constraints
                    options);
                if f_x < tol
                    x_eq(end+1,:) = x_candidate';
                end
            end

            x_eq = uniquetol(x_eq,tol,'ByRows',true);
        end

        function x0 = icgrid(obj, boundaries)
            x1g = (obj.canvas.x1.min:obj.canvas.x1.res:obj.canvas.x1.max)';
            x2g = (obj.canvas.x2.min:obj.canvas.x2.res:obj.canvas.x2.max)';
            if nargin < 2 || isempty(boundaries)
                [x1, x2] = meshgrid(x1g, x2g);
                x0 = [x1(:), x2(:)];
                return
            end

            x0 = [];
            a1 = ones(length(x1g),1);
            a2 = ones(length(x2g),1);
            for w = boundaries
                switch w
                    case 'l'
                        x0 = [x0; [obj.canvas.x1.min*a2,x2g]];
                    case 'd'
                        x0 = [x0; [x1g, obj.canvas.x2.min*a1]];
                    case 'r'
                        x0 = [x0; [obj.canvas.x1.max*a2,x2g]];
                    case 'u'
                        x0 = [x0; [x1g, obj.canvas.x2.max*a1]];
                    case 'h'
                        x0 = [x0; [x1g, zeros(length(x1g),1)]];
                    case 'v'
                        x0 = [x0; [zeros(length(x2g),1),x2g]];
                    otherwise
                        error('Unknown boundary.')
                end
            end
        end

        function ax = trajectories(obj, f, x0, tspan, ax)
            if nargin < 5
                ax = obj.buildaxes();
            end
            options = odeset(RelTol=1e-6, ...
                Events=@(t,x)obj.outofbound(t, x));

            x0 = reshape(x0,[length(x0),2]);

            for i = 1:size(x0,1)
                [~, x] = ode45(f,tspan,x0(i,:)', options);
                plot(ax, x(:,1), x(:,2), ...
                    Color=obj.trajectory_color,...
                    LineWidth=obj.trajectory_line_width);
            end
        end

        function ax = directions(obj, f, x0, ax)
            if nargin < 4
                ax = obj.buildaxes();
            end
            x0 = reshape(x0,[length(x0),2]);

            norm_dx = zeros(length(x0),1);
            dx = zeros(size(x0));
            cindex = linspace(0, 1, length(obj.quiver_color_map))';

            for i = 1:size(x0,1)
                dx(i,:) = f(0, x0(i,:)')';
                norm_dx(i) = norm(dx(i,:));
                dx(i,:) = dx(i,:)/norm_dx(i);
            end
            max_norm = max(max(norm_dx));
            for i = 1:size(x0,1)
                quiver(ax, x0(i,1), x0(i,2), dx(i,1), dx(i,2), 0.2,...
                    Color=interp1(cindex, obj.quiver_color_map, norm_dx(i)/max_norm),...
                    LineWidth=obj.quiver_line_width, ...
                    MaxHeadSize=obj.quiver_max_headsize);
            end
        end

        function ax = draw(obj,f,x0,tspan,ax)
            if nargin < 5
                ax = obj.buildaxes();
            end
            obj.trajectories(f,x0,tspan,ax);
            obj.directions(f,x0,ax);
        end

        function ax = buildaxes(obj, fig)
            if nargin < 2
                fig = figure('Colormap', obj.quiver_color_map);
            end
            ax = axes(fig,...
                NextPlot='add', ...
                Box='on', ...
                DataAspectRatio=[1, 1, 1],...
                XLim=[obj.canvas.x1.min, obj.canvas.x1.max], ...
                YLim=[obj.canvas.x2.min, obj.canvas.x2.max], ...
                TickLabelInterpreter='latex', ...
                FontSize=obj.label_font_size);

            if ~isempty(obj.xticks)
                set(ax, XTick=obj.xticks)
            end

            if ~isempty(obj.yticks)
                set(ax, YTick=obj.yticks)
            end

            xlabel(ax, '$x_1$', ...
                Interpreter='latex',...
                FontSize=obj.label_font_size);
            ylabel(ax, '$x_2$', ...
                Interpreter='latex',...
                FontSize=obj.label_font_size);
        end
    end

    methods(Access=private)
        function [position, isterminal, direction] = outofbound(obj, ~, x)
            x1_limit = [-0.2, 0.2] + [obj.canvas.x1.min, obj.canvas.x1.max];
            x2_limit = [-0.2, 0.2] + [obj.canvas.x2.min, obj.canvas.x2.max];
            position = (x1_limit(1) < x(1) & x(1) < x1_limit(2) & ...
                x2_limit(1) < x(2) & x(2) < x1_limit(2));
            isterminal = 1; % Halt integration
            direction = 0; % The zero can be approached from either side
        end
    end
end
