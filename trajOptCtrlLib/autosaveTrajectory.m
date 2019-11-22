% Simple script to construct file name and save trajectory, for use after
% generating an optimised trajectory using trajOpt
str = [sys.name '_' num2str(nPoints) '_' method '_'];
if cost.T > 0, str = [str num2str(cost.T, 2) 'Tsq_']; end
if cost.u > 0, str = [str num2str(cost.u, 2) 'usq_']; end
if cost.accSmooth > 0, str = [str num2str(cost.accSmooth, 2) 'accSmooth_']; end
if cost.uSmooth > 0, str = [str num2str(cost.uSmooth, 2) 'uSmooth_']; end
str = [str num2str(uMax) 'uMx'];
filename = str;
condition = true;
n = 2;
while condition
    if exist([filename '.mat'], 'file') == 2
        filename = [str '_(' num2str(n) ')'];
    else
        condition = false;
    end
    n = n + 1;
end
filename = strrep(filename, '.', '_');
disp(['Saving trajectory ' filename]);
saveTrajectory(filename);