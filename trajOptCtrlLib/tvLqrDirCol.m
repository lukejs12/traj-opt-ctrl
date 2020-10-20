function [lqrParam, u_cl_fun, tIdxFun] = tvLqrDirCol(sys, lqrParam, tspan, x0, u0)
    % sys, sys.x_dot_sym, sys.x_dot_fun, sys.stateVars=[q1, q2, ... q1_dot, q2_dot, ...], sys.nStates
    % lqr.Qf, lqr.Q, lqr.S, lqr.nSteps
    
    t0 = tspan(1);
    tf = tspan(2);
    tIdxFun = @(t) -1+2*(t-t0)/(tf-t0);
    
    % Initialise xukOft()
    xuOft(0, x0, u0, tspan, sys);

    % Create lists of system physical property names and values
    paramNames = fieldnames(sys.param);
    for n = 1:length(paramNames), paramVals(n) = getfield(sys.param, paramNames{n}); end %#ok<AGROW,GFLD>    

    % Create symbolic x_dot function substituting in physical parameters
    physSys = subs(sys.x_dot_sym, paramNames, paramVals');

    % Linearize system
    A_lin_sym = jacobian(physSys, sys.stateVars); % df/dx
    B_lin_sym = jacobian(physSys, sys.inputVars(1));  % df/du
    
    A_lin_fun = matlabFunction(A_lin_sym, 'Vars', {[sys.stateVars; sys.inputVars]});
    B_lin_fun = matlabFunction(B_lin_sym, 'Vars', {[sys.stateVars; sys.inputVars]});
    
    % Create time-dependent (numeric) expressions for linearized A & B matrices
    % - these are actually dependent on the nominal trajectory state
    Alin_t = @(t) A_lin_fun(xuOft(t));
    Blin_t = @(t) B_lin_fun(xuOft(t));
    
    % Confirm the system is controllable along entire trajectory
    h = (tf-t0)/(lqrParam.nSteps-1);
    linSysCtrb = zeros(lqrParam.nSteps, 1);
    K = zeros(lqrParam.nSteps, sys.nStates);
    for n = 1:lqrParam.nSteps
        t = (n-1)*h;
        A = Alin_t(t);
        B = Blin_t(t);
        linSysCtrb(n) = rank(ctrb(A, B));
    end
    if ~all(linSysCtrb == sys.nStates)
        error('Not all linearized systems controllable');
    end

    R_inv = inv(lqrParam.R);
    S_f = lqrParam.Q_f;
    % Create anonymous S_dot function
    S_dot = @(t, S, u) -(S*Alin_t(t) + Alin_t(t)'*S - S*Blin_t(t)*R_inv*Blin_t(t)'*S + lqrParam.Q);
    S_dot_wrapper = @(t, S, u) reshape(S_dot(t, reshape(S, sys.nStates, sys.nStates), u), sys.nStates^2, 1);
    % Solve differential Ricatti equation
    S_sol = ode45(@(t, S) S_dot_wrapper(t, S, 0), [tf t0], reshape(S_f, sys.nStates^2, 1));
    S_t_traj = S_sol.x;
    S_traj = S_sol.y;
    K = zeros(lqrParam.nSteps, sys.nStates);
    eVals = zeros(lqrParam.nSteps, sys.nStates);
    % Calculate feedback gains
    for n=1:lqrParam.nSteps
        tn = (n-1)*h;
        % Get solution to differential Ricatti equation at time tn
        Sn = deval(S_sol, tn);
        Sn = reshape(Sn, sys.nStates, sys.nStates);
        % Calculate gain for time tn
        K(n, :) = R_inv*Blin_t(tn)'*Sn;
        % Calculate eigenvalues of closed loop system
        eVals(n, :) = eig(Alin_t(tn)-Blin_t(tn)*K(n, :));
    end
    lqrParam.K = K;
    lqrParam.eVals = eVals;
%     lqrParam.K_p = chebfun(lqrParam.K, 'equi');
    % Initialise u_lqr()
    u_lqr(0, 0, lqrParam.K, [t0, tf], lqrParam.nSteps, tIdxFun);
%     (t, x, Ktraj, tspan, nKnotPoints, tIdxFun)
%     u_lqr(0, 0, tIdxFun);
    u_cl_fun = @u_lqr;
end

function xu = xuOft(t, xnom, unom, tspan, system)
    % Returns the state vector and control for the nominal trajectory at
    % time t. Uses a cubic polynomial to interpolate between grid points
    % for the state, and first order hold (linear) interpolation for the
    % control. Requires initialisation by calling with all parameters (t is
    % ignored). Subsequently call like this: 
    %   [x, u] = xuOft(t)
    
    persistent x0 u0 tf t0 sys
    if nargin > 1
        % Initialise persistent variables
        x0 = xnom;
        u0 = unom;
        tf = tspan(2);
        t0 = tspan(1);
        sys = system;
        xu = 0;
        return;
    end
    
    h = (tf-t0)/(length(u0)-1); % Number of segments is one less than number of points
    if t/h+1 >= length(u0)
        x = x0(:, end);
        u = u0(end);
        xu = [x; u];
        return;
    elseif t/h+1 <= 1
        x = x0(:, 1);
        u = u0(1);
        xu = [x; u];
        return;
    end
    n = floor(t/h)+1;
    t_off = t - (n-1)*h;
    x_n = x0(:, n);
    x_np1 = x0(:, n+1);
    u_n = u0(n);
    u_np1 = u0(n+1);
    f_n = sys.x_dot_fun(t, x_n, u_n, sys.param);
    f_np1 = sys.x_dot_fun(t+h, x_np1, u_np1, sys.param);
    % Evaluate cubic polynomial at time = t_off
    x = x_n + f_n*t_off - (t_off^2*(f_n - f_np1 + (3*(2*x_n - 2*x_np1 + f_n*h + f_np1*h))/h))/(2*h) + (t_off^3*(2*x_n - 2*x_np1 + f_n*h + f_np1*h))/h^3;
    % Linearly interpolate between nominal control points
    u = u_n + (t_off/h)*(u_np1-u_n);
    xu = [x; u];
end

function u = u_lqr(t, x, Ktraj, tspan, nKnotPoints, tIdxFun)
%     u_nom = u0(tIdx(t));
%     x_nom = x0(tIdx(t));
    persistent K tIdx h nSteps
    
    if nargin > 2
        K = Ktraj;
        tIdx = tIdxFun;
        nSteps = nKnotPoints;
        h = (tspan(2) - tspan(1))/(nSteps-1);% Number of segments is one less than number of points
        u = 0;
        return;
    end
    % Get nominal trajectory state and control at time t
    xu = xuOft(t);
    x_nom = xu(1:end-1);
    u_nom = xu(end);
    % Linear interpolation of control gains
    if t/h+1 >= nSteps  % end of trajectory
        k = K(end, :);
    elseif t/h+1 <= 1   % beginning of trajectory
        k = K(1, :);
    else
        n = floor(t/h)+1;
        t_off = t - (n-1)*h;
        k_n = K(n, :);
        k_np1 = K(n+1, :);
        k = k_n + (t_off/h)*(k_np1-k_n);
    end
    x_bar = x - x_nom;
    u_bar = -k*x_bar;
    u = u_bar + u_nom;
end