% deriveEom     Derives equations of motion from Lagrangian and writes function file to disk. 
%   [x_dot_sym, eomFile, stateVars] = deriveEom(sysName, coordVars, L, D, Q, saveSys)
%
%   sysName     String containing name of system (used to construct filename)
%   coordVars   Cell array identifying the coordinate variables, each of
%               which must be symbolic function of time, e.g. q(t)
%   L           Lagrangian for system
%   D           Dissipation function (damping - experimental)
%   U           Force/torque inputs, as column vector, rows corresponding
%               to coordinate variable 
%
%   saveSys     Boolean. If true, sys structure will be saved with
%               name sysNameSys.mat, and numeric x_dot_fun will be saved.
%
%   Example
%       sysName = 'pendulumCartTest2';
%       syms m1 m2 c1 q1(t) q2(t) g b u;
%       coordVars = {q1, q2};
%       Q = [u; 0];
%       T = .5*m1*diff(q1, 't')^2 + .5*m2*(q1 + c1*sin(diff(q2, 't')))^2 + .5*m2*(c1*cos(diff(q2, 't')))^2;
%       V = m2*g*c1*(1-cos(q2));
%       L = T-V;
%       D = .5*b*diff(q1, 't')^2;
%       sys = deriveEom(sysName, coordVars, L, D, Q, false);
% function [x_dot_sym, x_dot_fun, stateVars] = deriveEom(sysName, coordVars, L, D, Q)
% function sys = deriveEom(sysName, coordVars, T, V, N, mu, v_r, U, saveSys)
function sys = deriveEom(sysName, coordVars, L, D, U, Q, saveSys)
    nCoords = length(coordVars);
    eom = evalLagrangian(L, D, coordVars);
%     eom = evalLagrangianDissipation(T, V, N, mu, v_r, coordVars);

    % Create manipulator form representation of equations of motion generated
    % by evalLagrangian().
    H = sym(zeros(length(eom), nCoords));
    C = sym(zeros(length(eom), nCoords));
    G = sym(zeros(length(eom), 1));
    % B = sym(zeros(length(eom), 1));
    for cEqn = 1:length(eom)
        cEom = eom{cEqn};
        % H
        for qNum = 1:nCoords
            q = coordVars{qNum};
            % H
            res = subs(cEom, diff(q, 't', 2), 0);
            coeff = simplify(cEom - res)/diff(q, 't', 2);
            cEom = res;
            H(cEqn, qNum) = coeff;
            % C
            res = subs(cEom, diff(q, 't'), 0);
            coeff = simplify(cEom - res)/diff(q, 't');
            cEom = res;
            C(cEqn, qNum) = coeff;
            % G
            G(cEqn) = cEom;
    %         % B
    %         B = Q(1)/;
        end
    end
    q_dot = [];
    for n = 1:length(coordVars)
        q_dot = [q_dot; diff(coordVars{n}, 't')];
    end
    q_ddot = H\(-C*q_dot - G + U + Q);
    q_ddot = simplify(q_ddot, 200);
    % txt = latex(q_ddot);
    % Create cell array of coordinate as strings without '(t)', and create
    % vector of coordinate variables and their time derivatives, 
    % e.q. [q1 q2 q1_dot q2_dot]
    for n = 1:nCoords
        coordNames{n} = char(coordVars{n}); %#ok<*AGROW>
        coordNames{n} = coordNames{n}(1:end-3);
        coordNamesDot{n} = [coordNames{n} '_dot'];
    end
    stateVars = sym([coordNames.'; coordNamesDot.']);
    % Create new q_ddot matrix replacing coordinate vars as functions of time, and
    % their derivatives - e.g. q1(t), diff(q1(t), t) and D(q1(t)) with new 
    % variables - q1, q1_dot
    for m = 1:nCoords
        % Grab row m of q_ddot and convert to string
        q_ddot_str = char([zeros(1, m-1) 1 zeros(1, nCoords-m)]*q_ddot);
        for n = 1:nCoords
            % Differentiated variables can be represented by diff(q1(t), t) or
            % D(q1(t)), so need to search and replace both
            q_ddot_str = strrep(q_ddot_str, ['D(' coordNames{n} ')(t)'], [coordNames{n} '_dot']);
            q_ddot_str = strrep(q_ddot_str, ['diff(' coordNames{n} '(t), t)'], [coordNames{n} '_dot']);
            % Non-differentiated variables
            q_ddot_str = strrep(q_ddot_str, [coordNames{n} '(t)'], coordNames{n});
        end
        % q_ddot_symvar is same as q_ddot but with new variable names as outlined above.
        q_ddot_symvar(m, 1) = str2sym(q_ddot_str);
    end
    q_ddot_symvar = simplify(q_ddot_symvar, 300);

    % Create matlab function for symbolic q_ddot_symvar
    if exist('saveSys', 'var') && saveSys
        matlabFunction(q_ddot_symvar, 'file', [sysName '_auto'], 'vars', symvar(q_ddot_symvar));
    else
        matlabFunction(q_ddot_symvar, 'vars', symvar(q_ddot_symvar));
    end
    
    % Generate wrapper function for auto-generated anonymous function created
    % above to output x_dot = [q_dot; q_ddot], with all system parameters -
    % mass etc - bundled up in params structure. Function call is sysNameEom(t, x, param)
    functext = {['function x_dot = ' sysName 'Eom' '(t, x, u, param)']};
    for cVar = symvar(q_ddot)
        if ~strcmp(char(cVar), 't') && ~strcmp(char(cVar), 'u')
            functext{end+1} = [char(cVar) ' = param.' char(cVar) ';']; 
        end
    end
    for cVar = 1:nCoords
        stateVarName = char(coordVars{cVar});
        stateVarName = stateVarName(1:end-3);   % Remove '(t)'
        functext{end+1} = [stateVarName ' = x(' num2str(cVar) ');'];
        functext{end+1} = [stateVarName '_dot = x(' num2str(nCoords+cVar) ');'];
    end
    varList = '';
    for cVar = symvar(q_ddot_symvar)
        varList = [varList char(cVar) ','];
    end
    varList = varList(1:end-1);
    functext{end+1} = ['x_dot(1:' num2str(nCoords) ') = x(' num2str(nCoords+1) ':' num2str(nCoords*2) ');'];
    functext{end+1} = ['x_dot(' num2str(nCoords+1) ':' num2str(nCoords*2) ') = ' sysName '_auto(' varList ');'];
    functext{end+1} = 'x_dot = x_dot'';'; 
    funcstr = sprintf('%s\n', functext{:});
    eomFile = [sysName 'Eom'];
    x_dot_fun = str2func(eomFile);
    fileId = fopen([eomFile '.m'],'w');
    fprintf(fileId,'%s',funcstr);
    fclose(fileId);
    
    % Create x_dot_sym
%     x_dot_sym = zeros(nStates*2, 1);
    for n=1:nCoords
        x_dot_sym(n+nCoords) = q_ddot_symvar(n);
        x_dot_sym(n) = coordNamesDot(n);
    end
    
    sys.name = sysName;
    sys.coordVars = coordVars;
    sys.nCoords = nCoords;
    sys.stateVars = stateVars;
    sys.nStates = length(stateVars);
    sys.x_dot_sym = x_dot_sym;
    sys.x_dot_fun = x_dot_fun;
    
    % Save symbolic version of dynamics
    if exist('saveSys', 'var') && saveSys
        save([sysName 'Sys.mat'], 'sys');
    end    
end