%==========================================================================
% Recursively walks through a Matlab hierarchy and substitutes cell vectors
% by a matrix when possible.
% Specifically substitutes cell objects like
%    
%   {{1,2,3},{4,5,6}}
%
% by
%
%   {1,2,3;4,5,6}
%
% It leaves other objects unchanged except that it may change cell
% orientations (from column to row, etc.)
%
% Parameter makeords determines whether to convert from cells to normal
% matrices whenever possible (1) or leave matrices as cells (0).
%==========================================================================
function result = makematrices(r, makeords)
    result = recurse(r, 0, [], makeords);
end

%--------------------------------------------------------------------------
% 
%
function result = recurse(data, level, addit, makeords)
    if iscell(data)
        result = iter_cell(data, level, addit, makeords);
    elseif isstruct(data)
        result = iter_struct(data, level, addit, makeords);
    else
        result = scan_data(data, level, addit);
    end;
end

%--------------------------------------------------------------------------
% Iterates through cell array data. A cell array here is treated as a
% simple sequence, hence it is processed regardless its shape. The array is
% transformed to a cell matrix if it satisfies following conditions:
%   - It is vector
%   - All its items are cells
%   - All its items are vectors
%   - All its items are alligned (are of the same size)
%   - All its items are rows of a matrix (see ismatrixrow(...))
% Otherwise the content is left unchanged.
%
function result = iter_cell(data, level, addit, makeords)
    if  isvector(data) && ...
        iscell_all(data) && ...
        isvector_all(data) && ...
        isaligned_all(data) && ...
        ismatrixrow_all(data)
        
        tmp = data;
        tmp = cellfun(@cell2mat, tmp, 'UniformOutput', 0);
        tmp = cellfun(@torow, tmp, 'UniformOutput', 0);
        tmp = tocolumn(tmp);
        tmp = cell2mat(tmp);
        if ~makeords
            tmp = num2cell(tmp);
        end;
        result = tmp;
    elseif isempty(data)
        result = [];
    else   
        result = {};
        for i = 1:length(data)
            result{i} = recurse(data{i}, level + 1, addit, makeords);
        end;
    end;
end

%--------------------------------------------------------------------------
%
%
function result = iter_struct(data, level, addit, makeords)
    result = struct();
    for i = fields(data)'
        fld = char(i);
        result.(fld) = recurse(data.(fld), level + 1, addit, makeords);
    end;
end

%--------------------------------------------------------------------------
%
%
function result = scan_data(data, level, addit)
    result = data;
end

%--------------------------------------------------------------------------
%
%
function result = iscell_all(cellvec)
    result = all(cellfun(@iscell, cellvec));
end

%--------------------------------------------------------------------------
% Determines whether all items of cellvec are of the same length.
%
function result = isaligned_all(cellvec)
    siz = numel(cellvec{1});
    result = all(cellfun(@numel, cellvec) ==  siz);
end

%--------------------------------------------------------------------------
%
%
function result = ismatrixrow_all(cellvec)
    result = all(cellfun(@ismatrixrow, cellvec));
end

%--------------------------------------------------------------------------
% Determines whether cellvec can constitute a matrix row. The vector is a
% matrix row candidate if:
%   - all of its items are numeric
%   - all of its items are single (neither vectors nor matrices etc.)
%   - all of its items are compatible for concatenation to an ordinary 
%     vector (this is maybe automatically reached by isnumeric_all)
%
function result = ismatrixrow(cellvec)
    result = ...
        (isnumeric_all(cellvec) || islogical_all(cellvec) || isstruct_all(cellvec)) && ...
        issingle_all(cellvec) && ...
        iscompatible_all(cellvec);
end

%--------------------------------------------------------------------------
%
%
function result = isnumeric_all(cellvec)
    result = all(cellfun(@isnumeric, cellvec));
end

%--------------------------------------------------------------------------
%
%
function result = islogical_all(cellvec)
    result = all(cellfun(@islogical, cellvec));
end

%--------------------------------------------------------------------------
%
%
function result = issingle_all(cellvec)
    result = all(cellfun(@issingle, cellvec));
end

%--------------------------------------------------------------------------
%
%
function result = iscompatible_all(cellvec)
    result = true;
    for i = 1:(length(cellvec) - 1)
        result = result && iscompatible(cellvec{i}, cellvec{i + 1});
    end
end

%--------------------------------------------------------------------------
%
%
function result = iscompatible(obj1, obj2)
    result = isequal(class(obj1), class(obj2));
end

%--------------------------------------------------------------------------
%
%
function result = isvector_all(cellvec)
    result = all(cellfun(@isvector, cellvec));
end

%--------------------------------------------------------------------------
%
%
function result = isstruct_all(cellvec)
    result = all(cellfun(@isstruct, cellvec));
end

%--------------------------------------------------------------------------
%
%
function result = torow(vec)
    result = tocolumn(vec).';
end

%--------------------------------------------------------------------------
%
%
function result = tocolumn(vec)
    result = vec(:);
end