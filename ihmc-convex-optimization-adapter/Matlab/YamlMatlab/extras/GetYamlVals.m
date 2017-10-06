function [vals, timeaxis] = GetYamlVals(yamldata)
% this function converts data formatted in yaml style (cells containing timestamps and values) 
% into matlab user friendly matrices.

% obtain number of samples
n = max(size(yamldata));

if n
    if not(iscell(yamldata{1}))
        timeaxis = double(yamldata{1});
        vals   = cell2mat(yamldata(2:end));
    else

        % create output matrices
        timeaxis =  NaN*ones(n,1);
        if n % only if there are some elements of timeaxis
            vals = NaN*ones(n,numel(yamldata{1})-1);
        end
        for i=1:n
            timeaxis(i) = double(yamldata{i}{1});
            vals(i,:)   = cell2mat(yamldata{i}(2:end));
        end
    end

end

end % end of function

