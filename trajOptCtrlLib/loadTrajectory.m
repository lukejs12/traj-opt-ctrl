function [trajOut, uOut, T, param, optOutput, p] = loadTrajectory(matFile, nPointsOut)
    % loadTrajectory(matFile) loads the trajectory from a previous trajectory
    % optimization saved in matFile and resamples it to fit nGridPoints grid
    % points. 
    load(matFile, 'traj', 'u', 'T', 'param', 'output', 'p');
    if ~exist('p', 'var')
        p = 0;
    end
    [nStates, nPointsIn] = size(traj);
    if nargin < 2
        % No interpolation
        nPointsOut = nPointsIn;
        trajOut = traj;
        uOut = u;
    else
        for k=1:nStates
            trajOut(k, :) = interp1(linspace(0, 1, nPointsIn), traj(k, :), linspace(0, 1, nPointsOut), 'pchip');
        end
        uOut = interp1(linspace(0, 1, length(u)), u, linspace(0, 1, nPointsOut), 'pchip');
%         uOut = resample(u, nPointsOut, nPointsIn);
    end
    optOutput = output;
end