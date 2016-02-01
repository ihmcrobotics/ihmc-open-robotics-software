%==========================================================================
% Reads YAML file, converts YAML sequences to MATLAB cell columns and YAML
% mappings to MATLAB structs
%
%  filename ... name of yaml file to be imported
%  verbose  ... verbosity level (0 or absent = no messages, 
%                                          1 = notify imports)
%==========================================================================
function result = ReadYamlRaw(filename, verbose, nosuchfileaction, treatasdata)
    if ~exist('verbose','var')
        verbose = 0;
    end;
    
    if ~exist('nosuchfileaction','var')
        nosuchfileaction = 0;
    end;
    if ~ismember(nosuchfileaction,[0,1])
        error('nosuchfileexception parameter must be 0,1 or missing.');
    end;
    
    if(~exist('treatasdata','var'))
        treatasdata = 0;
    end;
    if ~ismember(treatasdata,[0,1])
        error('treatasdata parameter must be 0,1 or missing.');
    end;
    
    [pth,~,~] = fileparts(mfilename('fullpath'));       
    try
        import('org.yaml.snakeyaml.*');
        javaObject('Yaml');
    catch
        dp = [pth filesep 'external' filesep 'snakeyaml-1.9.jar'];
        if not(ismember(dp, javaclasspath ('-dynamic')))
        	javaaddpath(dp); % javaaddpath clears global variables...!?
        end
        import('org.yaml.snakeyaml.*');
    end;
    
    setverblevel(verbose);
    % import('org.yaml.snakeyaml.Yaml'); % import here does not affect import in load_yaml ...!?
    result = load_yaml(filename, nosuchfileaction, treatasdata);
end

%--------------------------------------------------------------------------
% Actually performs YAML load. 
%  - If this is a first call during recursion it changes cwd to the path of
%  given filename and stores the old path. Then it calls the YAML parser
%  and runs the recursive transformation. After transformation or when an
%  error occurs, it sets cwd back to the stored value.
%  - Otherwise just calls the parser and runs the transformation.
%
function result = load_yaml(inputfilename, nosuchfileaction, treatasdata)

    persistent nsfe;

    if exist('nosuchfileaction','var') %isempty(nsfe) && 
        nsfe = nosuchfileaction;
    end;
    
    persistent tadf;
    
    if isempty(tadf) && exist('treatasdata','var')
        tadf = treatasdata;
    end;
   
    yaml = org.yaml.snakeyaml.Yaml(); % It appears that Java objects cannot be persistent...!?
    if ~tadf
        [filepath, filename, fileext] = fileparts(inputfilename);
        if isempty(filepath)
            pathstore = cd();
        else
            pathstore = cd(filepath);
        end;
    end;
    try
        if ~tadf
            result = scan(yaml.load(fileread([filename, fileext])));
        else
            result = scan(yaml.load(inputfilename));
        end;
    catch ex
        if ~tadf
            cd(pathstore);
        end;
        switch ex.identifier
            case 'MATLAB:fileread:cannotOpenFile'
                if nsfe == 1
                    error('MATLAB:MATYAML:FileNotFound', ['No such file to read: ',filename,fileext]);
                elseif nsfe == 0
                    warning('MATLAB:MATYAML:FileNotFound', ['No such file to read: ',filename,fileext]);
                    result = struct();
                    return;
                end;
        end;
        rethrow(ex);
    end;
    if ~tadf
        cd(pathstore);    
    end;
end

%--------------------------------------------------------------------------
% Determine node type and call appropriate conversion routine. 
%
function result = scan(r)
    if isa(r, 'char')
        result = scan_string(r);
    elseif isa(r, 'double')
        result = scan_numeric(r);
    elseif isa(r, 'logical')
        result = scan_logical(r);
    elseif isa(r, 'java.util.Date')
        result = scan_datetime(r);
    elseif isa(r, 'java.util.List')
        result = scan_list(r);
    elseif isa(r, 'java.util.Map')
        result = scan_map(r);
    else
        error(['Unknown data type: ' class(r)]);
    end;
end

%--------------------------------------------------------------------------
% Transforms Java String to MATLAB char
%
function result = scan_string(r)
    result = char(r);
end

%--------------------------------------------------------------------------
% Transforms Java double to MATLAB double
%
function result = scan_numeric(r)
    result = double(r);
end

%--------------------------------------------------------------------------
% Transforms Java boolean to MATLAB logical
%
function result = scan_logical(r)
    result = logical(r);
end

%--------------------------------------------------------------------------
% Transforms Java Date class to MATLAB DateTime class
%
function result = scan_datetime(r)
    result = DateTime(r);
end

%--------------------------------------------------------------------------
% Transforms Java List to MATLAB cell column running scan(...) recursively
% for all ListS items.
%
function result = scan_list(r)
    result = cell(r.size(),1);
    it = r.iterator();
    ii = 1;
    while it.hasNext()
        i = it.next();
        result{ii} = scan(i);
        ii = ii + 1;
    end;
end

%--------------------------------------------------------------------------
% Transforms Java Map to MATLAB struct running scan(...) recursively for
% content of every Map field.
% When there is field, which is recognized to be the >import keyword<, an
% attempt is made to import file given by the field content.
%
% The result of import is so far stored as a content of the item named 'import'.
%
function result = scan_map(r)
    it = r.keySet().iterator();
    while it.hasNext()
        next = it.next();
        i = next;
        ich = char(i);
        if iskw_import(ich)
            result.(ich) = perform_import(r.get(java.lang.String(ich)));
        else
            result.(genvarname(ich)) = scan(r.get(java.lang.String(ich)));
        end;
    end;
    if not(exist('result','var'))
        result={};
    end
end

%--------------------------------------------------------------------------
% Determines whether r contains a keyword denoting import.
%
function result = iskw_import(r)
    result = isequal(r, 'import');
end

%--------------------------------------------------------------------------
% Transforms input hierarchy the usual way. If the result is char, then
% tries to load file denoted by this char. If the result is cell then tries
% to do just mentioned for each cellS item. 
% 
function result = perform_import(r)
    r = scan(r);
    if iscell(r) && all(cellfun(@ischar, r))
        result = cellfun(@load_yaml, r, 'UniformOutput', 0);
    elseif ischar(r)
        result = {load_yaml(r)};
    else
        disp(r);
        error(['Importer does not unterstand given filename. '...
               'Invalid node displayed above.']);
    end;
end

%--------------------------------------------------------------------------
% Sets verbosity level for all load_yaml infos.
%
function setverblevel(level)
    global verbose_readyaml;
    verbose_readyaml = 0;
    if exist('level','var')
        verbose_readyaml = level;
    end;
end

%--------------------------------------------------------------------------
% Returns current verbosity level.
%
function result = getverblevel()
    global verbose_readyaml; 
    result = verbose_readyaml;
end

%--------------------------------------------------------------------------
% For debugging purposes. Displays a message as level is more than or equal
% the current verbosity level.
%
function info(level, text, value_to_display)
    if getverblevel() >= level
        fprintf(text);
        if exist('value_to_display','var')
            disp(value_to_display);
        else
            fprintf('\n');
        end;
    end;
end
%==========================================================================

