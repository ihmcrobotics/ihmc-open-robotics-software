%==========================================================================

%==========================================================================
function result = dosubstitution(r, dictionary)
    if ~exist('dictionary','var')
        dictionary = {};
    end;
    result = recurse(r, 0, dictionary);
end

function result = recurse(data, level, dictionary)
    if iscell(data) && ~ismymatrix(data)
        result = iter_cell(data, level, dictionary);
    elseif isstruct(data)
        result = iter_struct(data, level, dictionary);
    elseif ischar(data) && isfield(dictionary, data)
        result = dictionary.(data);
    else
        result = data;
    end;
end

function result = iter_cell(data, level, dictionary)
    result = {};
    for i = 1:length(data)
        result{i} = recurse(data{i}, level + 1, dictionary);
    end;
end

function result = iter_struct(data, level, dictionary)
    result = data;
    for i = fields(data)'
        fld = char(i);
        result.(fld) = recurse(data.(fld), level + 1, dictionary);
    end;
end