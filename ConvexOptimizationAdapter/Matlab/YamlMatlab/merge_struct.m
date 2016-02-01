
%--------------------------------------------------------------------------
% Does merge of two structures. The result is structure which is union of
% fields of p and s. If there are equal field names in p and s, fields in p
% are overwriten with their peers from s.
%
function result = merge_struct(p, s, donotmerge, deep)
    if ~( isstruct(p) && isstruct(s) )
        error('Only structures can be merged.');
    end;
    if ~exist('donotmerge','var')
        donotmerge = {};
    end
    if ~exist('deep','var')
        deep = 0;
    elseif strcmp(deep, 'deep')
        deep = 1;
    end;
    

    
    result = p;
    for i = fields(s)'
        fld = char(i);
        if any(cellfun(@(x)isequal(x, fld), donotmerge))
            continue;
        end;
 %       if isfield(result, fld)
 %           % Just give the user a hint that there may be some information
 %           % lost.
 %           fprintf(['Overwriting field ',fld,'\n']);
 %       end;
        %disp('Assigning:')
        %disp(['fieldname: ',fld]);
        %disp(s.(fld));
        %disp('----------');
        if deep == 1 && isfield(result, fld) && isstruct(result.(fld)) && isstruct(s.(fld))
            result.(fld) = merge_struct(result.(fld), s.(fld), donotmerge, deep);
        else
            result.(fld) = s.(fld);
        end;
    end;
end