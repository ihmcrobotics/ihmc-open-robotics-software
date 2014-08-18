%==========================================================================
% Searches through some hierarchy and performs inheritance. Whenever finds
% struct field named 'parent' it tries to find all points, defined by its
% content and uses them as the struct ancestors. Example:
%
% Given:
%
%   s.a.a = 1
%   s.a.b = 2
%   s.a.parent: 'b'
%
%   s.b.a = 3
%   s.b.c = 4
%
% the result of r = doinheritance(s) is:
% 
%   r.a = 1
%   r.c = 4
%   r.b = 2
%   r.parent = 'b'
%
% Multiple inheritance is allowed using cell array of parent point strings
% instead of one simple string.
%
%==========================================================================
function result = doinheritance(r, tr)
    if ~exist('tr','var')
        tr = r;
    end;
    result = recurse(r, 0, {tr});
end

function result = recurse(data, level, addit)
    if iscell(data) && ~ismymatrix(data)
        result = iter_cell(data, level, addit);
    elseif isstruct(data)
        result = iter_struct(data, level, addit);
    else
        result = data;
    end;
end

function result = iter_cell(data, level, addit)
    result = {};
    for i = 1:length(data)
        result{i} = recurse(data{i}, level + 1, addit);
    end;
    
    for i = 1:length(data)
        if isstruct(result{i}) && isfield(result{i}, kwd_parent())
            result{i} = inherit(result{i}, result{i}.(kwd_parent()), [], addit{1}, {}); % !!!
        end;
    end;
end

function result = iter_struct(data, level, addit)
    result = data;
    for i = fields(data)'
        fld = char(i);
        result.(fld) = recurse(data.(fld), level + 1, addit);
    end;
    
    for i = fields(result)'
        fld = char(i);
        if isstruct(result.(fld)) && isfield(result.(fld), kwd_parent())
            result.(fld) = inherit(result.(fld), result.(fld).(kwd_parent()), [], addit{1}, {});
        end;
    end;
end

function result = inherit(child, parent_chr, container, oaroot, loc_imported)
    result = child;
    if ~iscell(parent_chr)
        parent_chr = {parent_chr};
    end;
    for i = length(parent_chr):-1:1
        if contains(loc_imported, parent_chr{i})
            error('MATLAB:MATYAML:inheritedtwice',['Cyclic inheritance: ', parent_chr{i}]);
        end;
        try
            parentstruct = eval(['oaroot.',parent_chr{i}]);
        catch ex
            switch ex.identifier
                case {'MATLAB:nonExistentField', 'MATLAB:badsubscript'}
                    error('MATLAB:MATYAML:NonExistentParent', ['Parent was not found: ',parent_chr{i}]);
            end;
            rethrow(ex);
        end;
        if isstruct(parentstruct) && isfield(parentstruct, kwd_parent())
            next_loc_imported = loc_imported;
            next_loc_imported{end + 1} = parent_chr{i};
            result = merge_struct(inherit(parentstruct, parentstruct.(kwd_parent()), [], oaroot, next_loc_imported), result, {'import'});
        end;
        result = merge_struct(parentstruct, result, {'import'});
    end;
end

function result = contains(list, chr)
    for i = 1:length(list)
        if strcmp(list{i}, chr)
            result = true;
            return;
        end;
    end;
    result = false;
end