% REWRITE

% Derives the equations of motion Lagragian in symfun L. Matlab
% symbolic toolkit won't differentiate wrt to an arbitrary symfun (i.e. wrt 
% the time derivative of the coordinate variables), so we have to use 
% subs() to substitute in dummy variable for differentiation, then swap 
% back in the original variable.
% Arguments: 
% L symfun containing Lagrangian
% i.e. L=T-U where T=sum of kinetic energy and U= sum of
% potential energy
% D Rayleigh dissipation function, where 
% D = sum(.5*b_i*x_i_dot^2)
% where b_i is damping coefficient of viscous damper, and
% x_i_dot is the speed of the damper
% coordvars cell array specifying generalised coordinate variables 
% e.g. {x(t), theta(t)
% Returns:
% eoms cell array of symfun equations of motion
function eom=evalLagrangianDissipation(T, V, N, mu, v_r, coordvars)
    eom={};
    if ~iscell(coordvars)
        error('3nd argument must be cell array of coordinate variables x_i, x_i+1 etc'); 
    end
    for k=1:length(coordvars)
        % Get current coord var
        cVar=coordvars{k};
        % Get time derivative of above
        % cVarDt=coordvars{j+1};
        cVarDt = diff(cVar, 't');
        if cVarDt == 0
            error('Coordinate variables must be functions of time.');
        end
        % Calculate dT / dq_k_dot
        dT_dq_k_dot = subs(diff(subs(T, cVarDt, 'tempVar'), 'tempVar'), 'tempVar', cVarDt);
        % Now differentiate wrt time
        ddt_dT_dq_k_dot = diff(dT_dq_k_dot, 't');
        % dT_dq_k
        dT_dq_k = subs( diff(subs(T, cVar, 'dVar'), 'dVar'), 'dVar', cVar);
        % dV/dq_k
        dV_dq_k = subs( diff(subs(V, cVar, 'dVar'), 'dVar'), 'dVar', cVar); %#ok<*NASGU>
        
        % Dissipation function
        % Normal force
        N_k = N{k};
        % Friction function
        mu_k = mu{k};
        % Sliding velocity
        v_r_k = v_r{k};
        % Non conservative force for this variable
        Q_k = -N_k*mu_k(v_r_k)*subs(diff(subs(v_r_k, cVarDt, 'tempVar'), 'tempVar'), 'tempVar', cVarDt);
       
% %         % Viscous damping - dD/Dx_i 
% %         dD_dx_dot=subs(diff(subs(D, cVarDt, 'tempVar') , 'tempVar'), 'tempVar', cVarDt);
        % Assemble final equation
        eq=ddt_dT_dq_k_dot - dT_dq_k + dV_dq_k - Q_k;
        eom{k}=simplify(eq); %#ok<AGROW>
    end
end


% syms L x(t) x_dot(t) theta(t) theta_dot(t) m I g
% % % % x_dot=diff(x, t)
% % % % theta_dot=diff(theta, t)
% % % % L=.5*m*x_dot^2 + 0.5*I*theta_dot^2 - m*g*x
% L=.5*m*diff(x, 't')^2 + 0.5*I*diff(theta, 't')^2 - m*g*x
% derive_eom_lagran(L, 0, {x, theta})
