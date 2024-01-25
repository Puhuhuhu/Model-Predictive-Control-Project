classdef NmpcControl < handle
    
    properties
        solver
        nx, nu, N
        nlp_x0
        nlp_lbx, nlp_ubx
        nlp_lbg, nlp_ubg
        nlp_p
        
        T_opt
        sol
        idx
        
        % Delay compensation
        rocket
        expected_delay
        mem_u
        
        % Warmstart
        nlp_lam_x0
        nlp_lam_g0
    end
    
    methods
        function obj = NmpcControl(rocket, tf, expected_delay)
            
            if nargin < 3, expected_delay = 0; end
           
            import casadi.*
            
            N_segs = ceil(tf/rocket.Ts); % MPC horizon
            nx = 12; % Number of states
            nu = 4;  % Number of inputs
            
            % Decision variables (symbolic)
            N = N_segs + 1; % Index of last point
            X_sym = SX.sym('X_sym', nx, N); % state trajectory
            U_sym = SX.sym('U_sym', nu, N-1); % control trajectory)
            
            % Parameters (symbolic)
            x0_sym  = SX.sym('x0_sym', nx, 1);  % initial state
            ref_sym = SX.sym('ref_sym', 4, 1);  % target position
            
            % Default state and input constraints
            ubx = inf(nx, 1);
            lbx = -inf(nx, 1);
            ubu = inf(nu, 1);
            lbu = -inf(nu, 1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % Constraint matrices F and M
            M1 = eye(nu);
            M2 = -eye(nu);
            M = [M1; M2];
            m = [0.26; 0.26; 80; 20;
                 0.26; 0.26; -50; 20];
            
            F = [0 0 0 0 1 0 0 0 0 0 0 0;
                 0 0 0 0 -1 0 0 0 0 0 0 0];
            f = [deg2rad(75); deg2rad(75)];

            % Matrix to match dimensions of tracked point and states
            D = zeros(nx, nu);
            D(10:12, 1:3) = eye(3);
            D(6,4) = 1;

            % Cost matrices Q and R
            Q = diag([40 40 40  2 2 1200  25  25  40  200 200 200]);
            R = diag([0.01 0.01 0.0001 0.001]);

            % Terminal cost
            [xs, us] = rocket.trim();
            sys = rocket.linearize(xs, us);
            sys_d = c2d(sys, rocket.Ts);
            [~,Qf,~] = dlqr(sys_d.A, sys_d.B, Q, R);
            
            % Discreatization of f with RK4
            h = tf/N;
            f_discrete = @(x,u) RK4(x,u,h,rocket);

            % Cost
            cost = 0;
            
            % Equality constraints (Casadi SX), each entry == 0
            eq_constr = [];
            
            % Inequality constraints (Casadi SX), each entry <= 0
            ineq_constr = [];
            
            % Set the constraints
            % initial condition
            eq_constr = [eq_constr; X_sym(:,1) - x0_sym(:,1)];
            for i=1:N-1
                eq_constr = [eq_constr; X_sym(:,i+1) - f_discrete(X_sym(:,i), U_sym(:,i))];
                ineq_constr = [ineq_constr; F*X_sym(:,i) - f];
                ineq_constr = [ineq_constr; M*U_sym(:,i) - m];
                cost = cost + (X_sym(:,i) - D*ref_sym)'*Q*(X_sym(:,i) - D*ref_sym) + U_sym(:,i)'*R*U_sym(:,i);
            end
            % Terminal cost
            cost = cost + (X_sym(:,N) - D*ref_sym)'*Qf*(X_sym(:,N) - D*ref_sym);

            % For box constraints on state and input, overwrite entries of
            % lbx, ubx, lbu, ubu defined above
            ubx(5,1) = deg2rad(75);
            lbx(5,1) = -deg2rad(75);
            ubu = [0.26; 0.26; 80; 20];
            lbu = [-0.26; -0.26; 50; -20];

            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % ---- Assemble NLP ------
            nlp_x = [X_sym(:); U_sym(:)];
            nlp_p = [x0_sym; ref_sym];
            nlp_f = cost;
            nlp_g = [eq_constr; ineq_constr];
            
            nlp = struct('x', nlp_x, 'p', nlp_p, 'f', nlp_f, 'g', nlp_g);
            
            % ---- Setup solver ------
            opts = struct('ipopt', struct('print_level', 0), 'print_time', false);
            obj.solver = nlpsol('solver', 'ipopt', nlp, opts);
            
            % ---- Assemble NLP bounds ----

            obj.nlp_x0  = zeros(size(nlp_x));
            
            obj.nlp_ubx = [repmat(ubx, N, 1); repmat(ubu, (N-1), 1)];
            obj.nlp_lbx = [repmat(lbx, N, 1); repmat(lbu, (N-1), 1)];
            
            obj.nlp_ubg = [zeros(size(eq_constr)); zeros(size(ineq_constr))];
            obj.nlp_lbg = [zeros(size(eq_constr)); -inf(size(ineq_constr))];
            
            obj.nlp_p = [zeros(size(x0_sym)); zeros(size(ref_sym))];
            
            obj.nlp_lam_x0 = [];
            obj.nlp_lam_g0 = [];
            
            obj.nx = nx;
            obj.nu = nu;
            obj.N = N;
            obj.T_opt = linspace(0, N * rocket.Ts, N);
            
            obj.idx.X = [1, obj.N * obj.nx];
            obj.idx.U = obj.idx.X(2) + [1, (obj.N-1) * obj.nu];
            obj.idx.u0 = obj.idx.U(1) + [0, obj.nu-1];
            
            % Members for delay compensation
            obj.rocket = rocket;
            obj.expected_delay = expected_delay;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE

            u_init = zeros(4, 1); % Replace this by a better initialization
            u_init = us;

            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            obj.mem_u = repmat(u_init, 1, expected_delay);
        end
        
        function [u, T_opt, X_opt, U_opt] = get_u(obj, x0, ref)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            delay = obj.expected_delay;
            mem_u = obj.mem_u;
            
            % Delay compensation: Predict x0 delay timesteps later.
            % Simulate x_ for 'delay' timesteps
            x_ = x0;
            % ...

            % Euler integration scheme for each sampling period of delay
            h = obj.rocket.Ts;

            for i=1:delay
                x_ = x_ + h*obj.rocket.f(x_, mem_u(:,i));
            end
            
            x0 = x_;
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Compute solution from x0
            obj.solve(x0, ref);
            
            % Evaluate u0
            nlp_x = obj.sol.x;
            id = obj.idx.u0;
            u = full( nlp_x(id(1):id(2)) );      
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            % Delay compensation: Save current u
            if obj.expected_delay > 0
                for i=1:delay-1
                    obj.mem_u(:,i) = obj.mem_u(:,i+1);
                end
                obj.mem_u(:,delay) = u;
            end
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            if nargout > 1, T_opt = obj.get_T_opt(); end
            if nargout > 2, X_opt = obj.get_X_opt(); end
            if nargout > 3, U_opt = obj.get_U_opt(); end
            return
            
            % Additional evaluation
            % Complete trajectory
            % % X_opt = full(reshape(nlp_x(idx_X(1):idx_X(2)), obj.nx, obj.N));
            % % U_opt = full(reshape(nlp_x(idx_U(1):idx_U(2)), obj.nu, obj.N - 1));
            % %
            % % cost_opt = full(sol.f);
            % % constr_opt = full(sol.g);
            % %
            % % stats = obj.solver.stats;
        end
        
        function solve(obj, x0, ref)
            
            % ---- Set the initial state and reference ----
            obj.nlp_p = [x0; ref];     % Initial condition
            obj.nlp_x0(1:obj.nx) = x0; % Initial guess consistent
            
            % ---- Solve the optimization problem ----
            args = {'x0', obj.nlp_x0, ...
                'lbg', obj.nlp_lbg, ...
                'ubg', obj.nlp_ubg, ...
                'lbx', obj.nlp_lbx, ...
                'ubx', obj.nlp_ubx, ...
                'p', obj.nlp_p, ...
                %                 'lam_x0', obj.nlp_lam_x0, ...
                %                 'lam_g0', obj.nlp_lam_g0
                };
            
            obj.sol = obj.solver(args{:});
            if obj.solver.stats.success ~= true
                solve_status_str = obj.solver.stats.return_status;
                fprintf([' [' class(obj) ': ' solve_status_str '] ']);
                obj.sol.x(obj.idx.u0) = nan;
            end
            
            % Use the current solution to speed up the next optimization
            obj.nlp_x0 = obj.sol.x;
            obj.nlp_lam_x0 = obj.sol.lam_x;
            obj.nlp_lam_g0 = obj.sol.lam_g;
        end
        function T_opt = get_T_opt(obj)
            T_opt = obj.T_opt;
        end
        function X_opt = get_X_opt(obj)
            nlp_x = obj.sol.x;
            id = obj.idx.X;
            X_opt = full(reshape(nlp_x(id(1):id(2)), obj.nx, obj.N));
        end
        function U_opt = get_U_opt(obj)
            nlp_x = obj.sol.x;
            id = obj.idx.U;
            U_opt = full(reshape(nlp_x(id(1):id(2)), obj.nu, obj.N - 1));
        end
    end
end

% RK4 Integrator taken from exerecise session 7
function [x_next] = RK4(X,U,h,rocket)

   k1 = rocket.f(X,         U);
   k2 = rocket.f(X+h/2*k1, U);
   k3 = rocket.f(X+h/2*k2, U);
   k4 = rocket.f(X+h*k3,   U);
   x_next = X + h/6*(k1+2*k2+2*k3+k4);
end