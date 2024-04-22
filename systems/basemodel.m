classdef basemodel < handle
    properties(Access=public)
        name string = "Model";
        g double = 9.81; % The gravitational acceleration [m/s^2]
        tspan(1,2) double = [0, 10];
    end

    properties(Access=protected) % Visualization and plotting parameters and handles
        font_size = 14;
        line_width = 1;

        animation_axis = [];
        animation_axes_limits = [-1, 1; -1, 1; -1, 1];
        
    end

    properties(Access=protected)
        shape_update_function;
        body_color = lines(1);
        body_thickness = 2;
        silhouette_color = 0.5*ones(1,3);
        shape_handles = [];
        features = {};
    end


    methods(Access=public)
        function obj = basemodel()
        end
    end

    methods(Access=public)
        function parse_constructor_inputs(obj, varargin)
            for i = 1:2:length(varargin)-1
                propertyName = varargin{i};
                if ~ischar(propertyName) && ~isstring(propertyName)
                    error('Property names must be strings.');
                end

                if isprop(obj, propertyName)
                    obj.(propertyName) = varargin{i+1};
                else
                    error('Invalid property name: %s', propertyName);
                end
            end
        end
    end

    methods(Access=public)
        function R = rotx(~,q)
            R = [1, 0, 0; 0, cos(q), -sin(q); 0, sin(q), cos(q)];
        end

        function R = roty(~,q)
            R = [cos(q), 0, sin(q); 0, 1, 0; -sin(q), 0, cos(q)];
        end

        function R = rotz(~,q)
            R = [cos(q), -sin(q), 0; sin(q), cos(q), 0; 0, 0, 1];
        end

        function R = rot2(~,q)
            R = [cos(q), -sin(q); sin(q), cos(q)];
        end
    end
    

    methods(Access=public)
        function ax = draw(obj, x, ax)
            [ax, isnewaxes] = obj.set_axis(ax);
            if isnewaxes
                obj.build_shape(ax);
            end
            obj.update_shape(x);
         end
    end

    methods(Access=protected)       
        function update_shape(obj,x)
            pose = obj.shape_update_function(x);
            for i=1:length(obj.shape_handles)
                set(obj.shape_handles(i), ...
                    XData=pose(i).x, YData=pose(i).y, ZData=pose(i).z);
            end
        end

        function build_shape(obj,ax)
            pose = obj.shape_update_function(obj.x0);
            obj.shape_handles = gobjects(length(pose), 1);
            for i = 1:length(pose)
                obj.shape_handles(i) = line(ax, ...
                    pose(i).x, pose(i).y, pose(i).z, ...
                    LineWidth=obj.body_thickness, ...
                    Color=obj.body_color);
                if ~isempty(obj.features) && i <= length(obj.features) && ~isempty(obj.features{i})
                    properties = fields(obj.features{i});
                    for p = 1:length(properties)
                        set(obj.shape_handles(i), ...
                            properties{p}, obj.features{i}.(properties{p}));
                    end
                end
            end
        end
    end


    methods(Access=protected)
        function [ax, isnewaxes] = set_axis(obj,ax)
            isnewaxes = false;
            if isempty(ax)
                ax = obj.animation_axis;
                if isempty(ax)
                    ax = obj.build_animation_axis();
                    isnewaxes = true;
                    return
                end
            end

            if ~isempty(obj.shape_handles)
                isnewaxes = ax ~= ancestor(obj.shape_handles(1), 'axes');
            end
        end

        function ax = build_animation_axis(obj)
            ax = axes(Parent=figure());
            obj.set_animation_axes_properties(ax);
            set(ax, Xlim = obj.animation_axes_limits(1,:), ...
                Ylim = obj.animation_axes_limits(1,:), ...
                Zlim = obj.animation_axes_limits(1,:), ...
                XTick=0, YTick=0, XGrid="on", YGrid="on");
            obj.animation_axis = ax;
        end


        function set_legend_properties(obj, handle)
            set(handle, Interpreter="latex",...
                FontSize=obj.font_size)
        end

        function set_plot_axes_properties(obj, ax)
            set(ax, NextPlot="add", ...
                Box="on", ...
                XGrid="on", YGrid="on", ...
                TickLabelInterpreter="latex",...
                FontSize=obj.font_size);
        end

        function set_animation_axes_properties(obj, ax)
            set(ax, NextPlot="add", ...
                Box="on", ...
                DataAspectRatio = [1, 1, 1],...
                TickLabelInterpreter="latex",...
                FontSize=obj.font_size);
        end
    end


end