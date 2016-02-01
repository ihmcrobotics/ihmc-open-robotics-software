%==========================================================================
% Walks through a tree structure data. Whenever it finds a structure, which
% have field named 'import' it assumes that in that field is a cell array
% and merges all structures found in that array. Parameter verb is used for
% debugging purposes.
%==========================================================================
function result = mergeimports(data, verb)
    if ~exist('verb','var')
        verb = 0;
    end;
    result = recurse(data, 0, [], verb);
end

%--------------------------------------------------------------------------
% Recursion dispatcher, calls appropriate method for cell/structure or
% displays data if the parameter data is not of mentioned type. 
%   addit ... for possible future use, now unused
%   verb  ... for debugging
%
function result = recurse(data, level, addit, verb)
    indent = repmat(' | ',1,level); % for debugging
    if iscell(data)
        result = iter_cell(data, level, addit, verb);
    elseif isstruct(data)
        result = iter_struct(data, level, addit, verb);
    else
        if any(verb == 1) % for debugging
            fprintf([indent,'Some data: ']);
            disp(data);
        end;
        result = data;
    end;
end

%--------------------------------------------------------------------------
% Walks through a cell array and calls recurse on every field. 
%   data        ... Assumed to be a cell. Data to be walked.
%   level       ... Level in the tree, root has zero.
%   addit, verb ... for debugging
%
function result = iter_cell(data, level, addit, verb)
    indent = repmat(' | ',1,level); % for debugging
    result = {};
    if any(verb == 1); fprintf([indent,'cell {\n']); end; % for debugging
    for i = 1:length(data)
        itemcontent = recurse(data{i}, level + 1, addit, verb);
        result{end + 1} = itemcontent;
    end;
    if any(verb == 1); fprintf([indent,'} cell\n']); end; % for debugging
end

%--------------------------------------------------------------------------
% Walks through a struct and calls recurse on every field. If there is a 
% field called 'import' it calls process_import_field on its content. Then
% merges processed import with the rest of the structure. Meaning of all
% parameters is similar to those of iter_cell.
%
function result = iter_struct(data, level, addit, verb)
    indent = repmat(' | ',1,level); % for debugging
    result = struct();
    collected_imports = {};
    if any(verb == 1); fprintf([indent,'struct {\n']); end; % for debugging
    for i = fields(data)'
        fld = char(i);
        if any(verb == 1); fprintf([indent,' +-field ',fld,':\n']); end; % for debugging
        result.(fld) = recurse(data.(fld), level + 1, addit, verb);
        % Tree back-pass - all potential underlying imports were processed,
        % so process import here, if needed.
        if isequal(fld, 'import')
            processed_import = process_import_field(result.(fld));
            result = rmfield(result, 'import');
            if isstruct(processed_import)
                collected_imports{end+1} = processed_import;
                % It is maybe useless to collect imports to the array since
                % there can be only one field named 'import' per structure.
                % collected_imports is proposed to be changed to a simple 
                % variable.
            else
                % One of imports was not a struct. In following versions it
                % probably won't be an error and will merge to a cell with
                % structs.
                disp(processed_import);
                error('Expected struct, otherwise it cannot be merged with the rest.');
            end;
        end;
    end;
    for i = 1:length(collected_imports)
        result = merge_struct(result, collected_imports{i}, {}, 'deep');
    end;
    if any(verb == 1); fprintf([indent,'} struct\n']); end; % for debugging
end

%--------------------------------------------------------------------------
% Walks through the data parameter, which is assumed to be a cell. Merges
% all structures in that cell and returns them as a struct or possibly as a
% cell of merged struct and unmeregeable data.
%
function result = process_import_field(data)
    if iscell(data)
        merged_structs = struct();
        collected_nonstruct = {};
        for i = 1:length(data)
            if isstruct(data{i})
                merged_structs = merge_struct(merged_structs, data{i}, {}, 'deep');
            else
                collected_nonstruct{end+1} = data{i};
            end;
        end;
        if isempty(collected_nonstruct)
            result = merged_structs;
        elseif isempty(merged_structs)
            result = collected_nonstruct;
        else
            result = {merged_structs; collected_nonstruct};
        end;
    else
        % For clarity and simplicity, the whole transformation is done so 
        % that every import field in a struct is cell array even there is 
        % only one object to be imported.
        error('BUG: import field should always contain a cell.');
    end;
end

%==========================================================================