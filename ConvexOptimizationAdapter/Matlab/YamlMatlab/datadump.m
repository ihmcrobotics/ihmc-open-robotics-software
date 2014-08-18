function datadump(data)
    recurse(data, 0, []);
end

function result = recurse(data, level, addit)
    indent = repmat(' | ',1,level);
    if iscell(data) && ~ismymatrix(data)
        result = iter_cell(data, level, addit);
    elseif isstruct(data)
        result = iter_struct(data, level, addit);
    else
        fprintf([indent,' +-Some data: ']);
        disp(data);
        result = data;
    end;
end

function result = iter_cell(data, level, addit)
    indent = repmat(' | ',1,level);
    result = {};
    fprintf([indent,'cell {\n']);
    for i = 1:length(data)
        result{i} = recurse(data{i}, level + 1, addit);
    end;
    fprintf([indent,'} cell\n']);
end

function result = iter_struct(data, level, addit)
    indent = repmat(' | ',1,level);
    result = struct();
    fprintf([indent,'struct {\n']);
    for i = fields(data)'
        fld = char(i);
        fprintf([indent,' +-field ',fld,':\n']);
        result.(fld) = recurse(data.(fld), level + 1, addit);
    end;
    fprintf([indent,'} struct\n']);
end

