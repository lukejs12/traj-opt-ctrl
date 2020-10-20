function saveTrajectory(matFile)
% loadTrajectory(matFile) loads the trajectory from a previous trajectory
% optimization saved in matFile and resamples it to fit nGridPoints grid
% points.
% Horrible way to do this, but need to think about how I'm going to package
% optimized trajectory solutions before I sort this out.

fStr = ['save(''' matFile ''', ''traj'', ''u'', ''T'', ''param'', ''output'');'];
evalin('caller', fStr);

% CODE TO SAVE METADATA - REQUIRES EXCEL
% % 
% % metaDat = {'duration', 'T', 'x0', 'xf', 'sys.param', 'cost', 'nPoints', 'xLims', 'uMax', 'tLims'};
% % for k = 1:length(metaDat)
% %     metaDat{2, k} = evalin('caller', metaDat{1, k});
% % end
% % % metaDat = {duration, T, x0, xf, sys.param, cost, nPoints, xLims, uMax, tLims};
% % 
% % clear metaCell;
% % n=1;
% % for k = 1:length(metaDat)
% %     metaCell{1, n} = metaDat{1, k}; %#ok<AGROW>
% %     if isa(metaDat{2, k}, 'numeric')
% %         for j = 1:length(metaDat{2, k})
% %             metaCell{j + 1, n} = metaDat{2, k}(j);
% %         end
% %         n = n+1;
% %     elseif isa(metaDat{2, k}, 'struct')
% %         fields = fieldnames(metaDat{2, k});
% %         for j = 1:length(fields)
% %             metaCell{j+1, n} = fields{j};
% %             metaCell{j+1, n+1} = metaDat{2, k}.(fields{j});
% %         end
% %         n = n+2;
% %     end
% % end
% % xlswrite('trajMetaDat.xls', metaCell);
            
        