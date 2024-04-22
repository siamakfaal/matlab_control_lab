classdef ocs < handle
    properties(Access=protected)
        n int16; % Size of the state vector
        m int16; % Size of the control vector

        % Dynamics
        f function_handle % dx = f(t,x,u)
        
        % Cost function
        h function_handle % h(tN, x(tN)) : cost associated with final time
        g function_handle % g(ti, x(ti), u(ti)) : accumulated cost at each time ti
    
    end

    properties(Access=private)
        % Time discretization
        N int16;

        % Initial Conditions
        t0 double; % Initial time
        x0; % Initial state

        % Indices
        u_base_idx int16;
    end



    methods(Access=public)
        function obj = ocs(n, m, f, h, g)
            obj.parse_system(n, m, f);
            [obj.h, obj.g] = parse_cost(h, g);
        end
        

        function sol = solve(obj, tspan, x0, u0, x_bound, u_bound, options)
            
            obj.t0 = tspan(1);
            t = tspan(2:end);

            obj.N = length(tspan) - 1;
            obj.u_base_idx = obj.N*obj.n;

            [obj.x0, z0] = obj.initialpoint(x0,u0);
            if nargin < 5
                options = obj.defaultoptions;
            end

            [lb, ub] = bounds(obj, x_bound, u_bound);

            z = fmincon(@(z)obj.cost(t,z), ...
                z0,...
                [], [], ... % A, b
                [], [], ... % Aeq, beq
                lb, ub, ...
                @(z)obj.nonlcon(t,z), ...
                options);

            sol.t = tspan;
            [sol.x,sol.u] = obj.decompose(z);
        end

        function ax = plot(obj, sol)
            fig = figure;
            ax(1) = subplot(1,2,1,Parent=fig);
            ax(2) = subplot(1,2,2,Parent=fig);
            set_axes_properties(ax);
            plot(ax(1), sol.t, sol.x, LineWidth=1);
            stairs(ax(2), sol.t(1:end-1), sol.u, LineWidth=1);
        end
    end

    methods(Access=private)

        function [x0,z0] = initialpoint(obj,x,u)
            if iscolumn(x) && numel(x) == obj.n
                x0 = x;
                x = repmat(x.', obj.N, 1);
            else
                if size(x,2) == obj.n
                    x0 = x(1,:)';
                    x = [x; repmat(x(end,:), obj.N - size(x,1), 1)];
                else
                    error("Size of x0 is not consistant with size of the state vector");
                end
            end
            if iscolumn(u) && numel(x) == obj.m
                u = repmat(u.', obj.N, 1);
            else
                if size(u,2) == obj.m
                    u = [u; repmat(u(end,:), obj.N - size(u,1), 1)];
                else
                    error("Size of u0 is not consistant with size of the input vector");
                end
            end
            z0  = obj.compose(x, u);
        end

        function [lb, ub] = bounds(obj, x_bound, u_bound)
            lb = zeros(obj.N*(obj.m+obj.n),1);
            ub = zeros(obj.N*(obj.m+obj.n),1);
            if isempty(x_bound)
                lb(1:obj.u_base_idx,1) = -inf;
                ub(1:obj.u_base_idx,1) = inf;
            else
                lb(1:obj.u_base_idx,1) = repmat(x_bound(:,1),obj.N,1);
                ub(1:obj.u_base_idx,1) = repmat(x_bound(:,2),obj.N,1);
            end

            if isempty(u_bound)
                lb(obj.u_base_idx+1 : end,1) = -inf;
                ub(obj.u_base_idx+1 : end,1) = inf;
            else
                lb(obj.u_base_idx+1 : end,1) = repmat(u_bound(:,1),obj.N,1);
                ub(obj.u_base_idx+1 : end,1) = repmat(u_bound(:,2),obj.N,1);
            end

        end

        function idx = xidx(obj, k)
            % k = 1 to N
            idx = (k-1)*obj.n+1 : k*obj.n;
        end

        function idx = uidx(obj, k)
            % k = 0 to N-1
            idx = obj.u_base_idx + (k*obj.m + 1 : (k+1)*obj.m);
        end

        function J = cost(obj, t, z)
            % COST: computes cost using g and h functions
            %   Input:
            %       z: an N*(n + m) x 1 column vector of the form
            %               [  x(t1)  ]
            %           z = [   ...   ]
            %               [  x(tN)  ]
            %               [  u(t0)  ]
            %               [   ...   ]
            %               [ u(tN-1) ]
            %
            %   Output:
            %       J: the total cost of the trajectory (t,x,u) based on
            %
            %                             /tN
            %           J = h(tN,x(tN)) + |   g(t,x,u) dt
            %                             /t0

            tf = t(obj.N);
            xf = z(obj.xidx(obj.N));
            g_pre = obj.g(obj.t0, obj.x0, z(obj.uidx(0)));
            t_pre = obj.t0;

            J = obj.h(tf, xf);
            for k=1:obj.N-1
                t_cur = t(k);
                g_cur = obj.g(t_cur, z(obj.xidx(k)), z(obj.uidx(k)));
                J = J + trapz(t_cur - t_pre, g_pre, g_cur);
                t_pre = t_cur;
                g_pre = g_cur;
            end
            g_cur = obj.g(tf, xf, z(obj.uidx(k)));
            J = J + trapz(tf - t_pre, g_pre, g_cur);
        end

        % NONLCON: Nonlinear constraints for the optimization problem
        %   Inputs:
        %       z: an N*(n + m) column vector in the form
        %               [  x(t1)  ]
        %           z = [   ...   ]
        %               [  x(tN)  ]
        %               [  u(t0)  ]
        %               [   ...   ]
        %               [ u(tN-1) ]
        %       t: an N by 1 vector of time instances
        %           t = [t1, t2, ... tN]'
        %       dt: differences between adjacent elements in t
        %           dt = [t2-t1, t3-t2, ... tN-t(N-1)];
        %       n: state-space dimension = length(y)
        %       m: control input dimension = length(u);
        %       N: length(t);
        %       f: function handle that defines the differential constraints
        %           dy/dt = f(t,y,u)
        %       C: function handle that defines nonlinear inequality constraints
        %           C(t,y,u) <= 0   for all t
        %       E: function handle that defines nonlinear equality constraints
        %           E(t,y,u) = 0   for all t
        %
        function [c, ceq] = nonlcon(obj, t, z)
            c = [];
            ceq = zeros(obj.u_base_idx,1);
            
            t_pre = obj.t0;
            x_pre = obj.x0;
            u_pre = z(obj.uidx(0));
            for k = 1:(obj.N-1)
                % Differential constraints
                t_cur = t(k);
                x_cur = z(obj.xidx(k));
                u_cur = z(obj.uidx(k));
                
                ceq(obj.xidx(k),1) = x_cur - x_pre - (t_cur - t_pre)*obj.f(t_pre, x_pre, u_pre);
                
                t_pre = t_cur;
                x_pre = x_cur;
                u_pre = u_cur;
            end
            
            ceq(obj.xidx(obj.N),1) = z(obj.xidx(obj.N)) - x_pre - (t(end) - t_pre)*obj.f(t_pre, x_pre, u_pre);
        end


        function z = compose(~,x,u)
            %COMPOSE: compose x and u matrices into z vector
            %   Inputs:
            %       x: an N by n matrix of the form               
            %               [ x1(t1) x2(t1) ... xn(t1) ]
            %           x = [ x1(t2) x2(t2) ... xn(t2) ]
            %               [  ...    ...        ...   ]
            %               [ x1(tN) x2(tN) ... xn(tN) ]
            %
            %       u: an N by m matrix of the form
            %               [   u1(t0)    u2(t0)    ...   um(t0)   ]
            %           u = [   u1(t1)    u2(t1)    ...   um(t1)   ]
            %               [     ...       ...            ...     ]
            %               [ u1(t(N-1)) u2(t(N-1)) ... um(t(N-1)) ]
            %
            %   Output:
            %       z: an N*(n + m) x 1 column vector of the form
            %               [  x(t1)  ]
            %           z = [   ...   ]
            %               [  x(tN)  ]
            %               [  u(t1)  ]
            %               [   ...   ]
            %               [ u(tN-1) ]
            
            z = [reshape(x,numel(x),1); reshape(u,numel(u),1)];
        end
        
        function [x,u] = decompose(obj,z)
            % DECOMPOSE: decompose vector x int y and u matrices
            %   Input:
            %       z: an N*(n + m) x 1 column vector of the form
            %               [  x(t1)  ]
            %           z = [   ...   ]
            %               [  x(tN)  ]
            %               [  u(t1)  ]
            %               [   ...   ]
            %               [ u(tN-1) ]
            %
            %   Outputs:
            %       x: an N+1 by n matrix of the form               
            %               [ x1(t0) x2(t0) ... xn(t0) ]
            %           x = [ x1(t1) x2(t1) ... xn(t1) ]
            %               [  ...    ...        ...   ]
            %               [ x1(tN) x2(tN) ... xn(tN) ]
            %
            %       u: an N by m matrix of the form
            %               [   u1(t0)    u2(t0)    ...   um(t0)   ]
            %           u = [   u1(t1)    u2(t1)    ...   um(t1)   ]
            %               [     ...       ...            ...     ]
            %               [ u1(t(N-1)) u2(t(N-1)) ... um(t(N-1)) ]
            
            x = [obj.x0, reshape(z(1:obj.N*obj.n,1), obj.n, obj.N)]';
            u = reshape(z(obj.u_base_idx+1:end,1),obj.m,obj.N)';
        end
    end

    methods(Access=private)
        function parse_system(obj, n, m, f)
            if n > 0
                obj.n = n;
            else
                error("n must be an int defining the size of the sate vector");
            end
            if m > 0
                obj.m = m;
            else
                error("m must be an int defining the size of the control vector");
            end
            if isa(f,"function_handle")
                obj.f = f;
            else
                error("f must be a function handle dx = f(t,x,u)");
            end
        end

        function options = defaultoptions(obj)
            options = optimoptions("fmincon", ...
                Algorithm="interior-point",...
                Display="off",... 'iter-detailed'
                MaxFunctionEvaluations=1e6,...
                MaxIterations=200);
        end
    end
end


function s = trapz(dt,f0,f1)
    s = dt*(f0+f1)/2;
end

function [h, g] = parse_cost(h, g)
    if isempty(h)
        h = @(t,x) 0;
    else
        if ~isa(h,"function_handle")
            error("h must be a function handle of the form h(t,x)");
        end
    end
    if isempty(g)
        g = @(t,x,u) 0;
    else
        if ~isa(g,"function_handle")
            error("g must be a function handle of the form g(t,x,u)");
        end
    end
end


function set_axes_properties(ax)
    for k = 1:length(ax)
        set(ax(k), NextPlot="add", ...
            Box="on", XGrid="on", YGrid="on", ...
            TickLabelInterpreter="LaTeX", ...
            FontSize=20);
    end
end



