classdef basemodel < handle
    properties
        name string = "Model";  % Default model name
        g double = 9.81; % The gravitational acceleration [m/s^2]
        tspan double = [0, 10]; % Defaults simulation time in s
        x0 double; % Default initial state
    end

    properties(Access=protected)
        shape_handles = []; % Handles to the shape objects used in the animation
        animation_axes_limits double; % Axis limits for animation
        body_color double = lines(1);  % Color of the model body
        body_thickness double = 2; % Thickness of the model lines
        silhouette_color double = 0.5 * ones(1,3); % Color for model silhouette
        features = {};  % Additional features for model customization
    end

    methods(Abstract)
        dx = openloop(obj,t,x,u) % Defines open-loop dynamics
        dx = closedloop(obj,t,x) % Defines closed-loop dynamics
        E = energy(obj,t,x) % Total energy of the system
        mass_matrix = M(obj,t,x) % Mass matrix
        nonconservative  = h(obj,t,x) % Vector of non-conservative forces
        inputmatrix = B(obj,t,x) % Input matrix
        pose = stick_diagram(obj,x) % Calculates the pose for visualization
    end


    methods(Static)
        function parse_constructor_inputs(obj, varargin)
            for i = 1:2:length(varargin)-1
                propertyName = varargin{i};
                propertyValue = varargin{i+1};
                if ~ischar(propertyName) && ~isstring(propertyName)
                    error('Property names must be strings.');
                end

                if isprop(obj, propertyName)
                    expectedType = class(obj.(propertyName));
                    % Check if propertyValue is of the expected type or can be converted
                    if isa(propertyValue, expectedType) || all(isa([obj.(propertyName), propertyValue], expectedType))
                        obj.(propertyName) = propertyValue;
                    else
                        error('Mismatch in property type for %s. Expected %s, got %s.', propertyName, expectedType, class(propertyValue));
                    end
                else
                    error('Invalid property name: %s', propertyName);
                end
            end
        end

        function R = rotx(q), R = [1, 0, 0; 0, cos(q), -sin(q); 0, sin(q), cos(q)]; end
        function R = roty(q), R = [cos(q), 0, sin(q); 0, 1, 0; -sin(q), 0, cos(q)]; end
        function R = rotz(q), R = [cos(q), -sin(q), 0; sin(q), cos(q), 0; 0, 0, 1]; end
        function R = rot2(q), R = [cos(q), -sin(q); sin(q), cos(q)]; end
    end

    methods
        function  parse_model_inputs(obj, varargin)
            basemodel.parse_constructor_inputs(obj, varargin{:})
        end

        function [xlim, ylim, zlim] = get_axis_limits(obj)
            xlim = obj.animation_axes_limits(1,:);
            ylim = obj.animation_axes_limits(2,:);
            zlim = obj.animation_axes_limits(3,:);
        end

        function tf = isnewaxis(obj, ax)
            if isempty(obj.shape_handles) 
                tf = true;
            else
                tf = ax ~= ancestor(obj.shape_handles(1), 'axes');
            end
        end

        function update_shape(obj, x)
            pose = obj.stick_diagram(x);
            for i = 1:numel(obj.shape_handles)
                set(obj.shape_handles(i), ...
                    'XData', pose(i).x, ...
                    'YData', pose(i).y, ...
                    'ZData', pose(i).z);
            end
        end

        function build_shape(obj, ax)
            pose = obj.stick_diagram(obj.x0);
            obj.shape_handles = gobjects(numel(pose), 1);
            for i = 1:numel(pose)
                obj.shape_handles(i) = line(ax, ...
                    pose(i).x, pose(i).y, pose(i).z, ...
                    'LineWidth', obj.body_thickness, ...
                    'Color', obj.body_color);
                if ~isempty(obj.features) && i <= numel(obj.features) && ~isempty(obj.features{i})
                    properties = fieldnames(obj.features{i});
                    for p = 1:numel(properties)
                        set(obj.shape_handles(i), ...
                            properties{p}, obj.features{i}.(properties{p}));
                    end
                end
            end
        end
    end
end
