% Derives the equations of motion of the Lagragian in symfun L. Matlab
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
function eom=evalLagrangian(L, D, coordvars)
    eom={};
    if ~iscell(coordvars)
        error('3nd argument must be cell array of coordinate variables x_i, x_i+1 etc'); 
    end
    for j=1:length(coordvars)
        % Get current coord var
        cVar=coordvars{j};
        % Get time derivative of above
        % cVarDt=coordvars{j+1};
        cVarDt = diff(cVar, 't');
        if cVarDt == 0
            error('Coordinate variables must be functions of time.');
        end
        % Calculate dL/dx_i_dot
        dL_dx_dot=subs(diff(subs(L, cVarDt, 'tempVar') , 'tempVar'), 'tempVar', cVarDt);
        % Now differentiate wrt time
        ddt_dL_dx_dot=diff(dL_dx_dot, 't');
        % dL_dx
        % If substituting in a dummy variable for a time dependent
        % function, derivatives of the time dependent variable are lost,
        % e.g. syms q(t); T = q*diff(q, t); subs(T, q, 'temp') == 0. So
        % substitute in dummy variable for time derivative function first, 
        % then substitute for the function of time. 'tVar' = temporary
        % variable. So:
        dL_dx = subs(subs(diff(subs(subs(L, cVarDt, 'tVarDt'), cVar, 'tVar'), 'tVar'), 'tVarDt', cVarDt), 'tVar', cVar);
%         dL_dx=subs( diff(subs(L, cVar, 'dVar'), 'dVar'), 'dVar', cVar);
        % Viscous damping - dD/Dx_i 
        dD_dx_dot=subs(diff(subs(D, cVarDt, 'tempVar') , 'tempVar'), 'tempVar', cVarDt);
        % Assemble final equation
        eq=simplify(ddt_dL_dx_dot - dL_dx + dD_dx_dot, 500);
        eom{j}=simplify(eq); %#ok<AGROW>
    end
end     


% syms L x(t) x_dot(t) theta(t) theta_dot(t) m I g
% % % % x_dot=diff(x, t)
% % % % theta_dot=diff(theta, t)
% % % % L=.5*m*x_dot^2 + 0.5*I*theta_dot^2 - m*g*x
% L=.5*m*diff(x, 't')^2 + 0.5*I*diff(theta, 't')^2 - m*g*x
% derive_eom_lagran(L, 0, {x, theta})
