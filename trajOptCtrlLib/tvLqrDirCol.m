function [lqrParam, u_cl_fun, tIdxFun] = tvLqr(sys, lqrParam, tspan, x0, u0)
    % sys, sys.x_dot_sym, sys.x_dot_fun, sys.stateVars=[q1, q2, ... q1_dot, q2_dot, ...], sys.nStates
    % lqr.Qf, lqr.Q, lqr.S, lqr.nSteps
    
    t0 = tspan(1);
    tf = tspan(2);
% %     % Create Chebyshev polynomial representation of trajectories
% %     x0_p = chebfun(x0', 'equi');
% %     u0_p = chebfun(u0', 'equi');
% %     % Anonymous function for mapping time to range -1:1 for chebfun
% %     % representations
    tIdxFun = @(t) -1+2*(t-t0)/(tf-t0);
    
% %     [nStates, nPoints] = size(x0);
% %     ts = linspace(0, tf, nPoints*10-1);
% %     xs = zeros(nStates, nPoints);
% %     us = zeros(1, nPoints);
% %     for k = 1:length(ts)
% %         [xs(:, k), us(k)] = xuOft(ts(k));
% %     end
% %     t = linspace(0, tf, nPoints);
% %     figure; hold on; set(gca,'xtick', linspace(0, tf, nPoints)); grid on;
% %     plot(t, x0(1, :), t, x0(2, :), t, x0(3, :), t, x0(4, :));
% %     plot(ts, xs(1, :), ts, xs(2, :), ts, xs(3, :), ts, xs(4, :));
% % %     plot(t, x0(1, :), ts, xs(1, :)); 
% %     legend('Nominal', 'Interpolated');
% %     figure; hold on; set(gca,'xtick', linspace(0, tf, nPoints)); grid on;
% %     plot(t, u0, '+', ts, us, '.');
% %     legend('Nominal', 'Interpolated');
    
    % Initialise xuOft()
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
%         thisCtrb = [];
%         for k = 1:sys.nStates
%             thisCtrb = [thisCtrb A^(k-1)*B];
%         end
%         linSysCtrb(n) = rank(thisCtrb);
        linSysCtrb(n) = rank(ctrb(A, B));
        % Work out ordinary LQR gains along trajectory (experiment)
%         [K(n, :), ~, ~] = lqr(A, B, lqrParam.Q, lqrParam.R);
    end
    
%     nPoints = length(x0);

% %%% DEBUGGING
% 
    
%     for n = 1:lqr.nSteps
%         t = (n-1)*h
%         temp{n} = Alin_t((n-1)*h);
%     end
%     max(cell2mat(temp(:)))
%     u0(114) = (u0(113)+u0(115))/2;
% %%% END OF DEBUGGING

    R_inv = inv(lqrParam.R);
    S_f = lqrParam.Q_f;
    S_dot = @(t, S, u) -(S*Alin_t(t) + Alin_t(t)'*S - S*Blin_t(t)*R_inv*Blin_t(t)'*S + lqrParam.Q);
    S_dot_wrapper = @(t, S, u) reshape(S_dot(t, reshape(S, sys.nStates, sys.nStates), u), sys.nStates^2, 1);
    % Solve differential Ricatti equation
%     % Fixed time step (using rk4())
%     [S_t_traj, S_traj, ~] = rk4(S_dot_wrapper, @(t, S) 0, [tf t0], reshape(S_f, sys.nStates^2, 1), lqrParam.nSteps);
    % Variable time step using ode45()
    [S_t_traj, S_traj] = ode45(@(t, S) S_dot_wrapper(t, S, 0), [tf t0], reshape(S_f, sys.nStates^2, 1));
    S_t_traj = flip(S_t_traj);
    S_traj = flip(S_traj);
%     K = zeros(length(S_t_traj), sys.nStates);
    K = zeros(lqrParam.nSteps, sys.nStates);
    eVals = zeros(length(S_t_traj), sys.nStates);
    for n=1:lqrParam.nSteps%length(S_t_traj)

        
        
        % Variable time step code
        tn = (n-1)*h;
        [~, t_Si] = min(abs(S_t_traj - tn)); % t index in S_traj
        Sn = reshape(S_traj(t_Si, :), sys.nStates, sys.nStates);
        K(n, :) = R_inv*Blin_t(S_t_traj(t_Si))'*Sn;

%         % Original fixed step code
%         Sn = reshape(S_traj(n, :), sys.nStates, sys.nStates);
%         tn = S_t_traj(n);
%         K(n, :) = R_inv*Blin_t(tn)'*Sn;
%         % End of original fixed step code
        

%         eVals(n, :) = eig(Alin_t(tn)-Blin_t(tn)*K(n, :));
    end
    % Flip K so it's forwards in time
%     lqrParam.K = flip(K, 1);
    lqrParam.K = K;

    % Test - using LQR calculated at each trajectory point rather than
    % TVLQR
%     lqrParam.K = K;
    
    %%% DEBUGGING
    for n = 1:length(K), disp([num2str(n) ': ' num2str(S_t_traj(n)) ', ' num2str(lqrParam.K(n, :))]); end
    figure; hold on;
    for n=1:6, plot(linspace(t0, tf, lqrParam.nSteps), x0(n, :)); end
    plot(linspace(t0, tf, lqrParam.nSteps), u0);
    xlabel('t (s)');
    legend('q1', 'q2', 'q3', 'q1dot', 'q2dot','q3dot', 'u')
    %%% END DEBUGGING

%     lqrParam.eVals = flip(eVals, 1);
    lqrParam.K_p = chebfun(lqrParam.K, 'equi');
    % Initialise u_lqr()
    u_lqr(0, 0, lqrParam.K_p, tIdxFun);
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

function u = u_lqr(t, x, Ktraj, tIdxFun)
    % Expects K to be a chebfun polynomial representation of lqr gains
    % Same for x0 (nominal state trajectory) and u0 (nominal force trajectory)
%     u_nom = u0(tIdx(t));
%     x_nom = x0(tIdx(t));
    persistent K tIdx
    
    if nargin > 2
%         x0 = xtraj;
%         u0 = utraj;
        K = Ktraj;
        tIdx = tIdxFun;
        u = 0;
        return;
    end
    xu = xuOft(t);
    x_nom = xu(1:end-1);
    u_nom = xu(end);
    Kn = K(tIdx(t));
    x_bar = x - x_nom;
    u_bar = -Kn*x_bar;
    u = u_bar + u_nom;
%     disp(['K: ' num2str(Kn) ', u0: ' num2str(u_nom), ', u~: ' num2str(u_bar) ', x~: ' num2str(x_bar')]);
%     fprintf('\nK: %f, u0: %f, u~: %f, x~: %f', Kn, u_nom, u_tilde, x_tilde);
end