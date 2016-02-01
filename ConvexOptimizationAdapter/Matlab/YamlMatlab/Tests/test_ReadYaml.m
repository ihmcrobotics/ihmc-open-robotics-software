function stat = test_ReadYaml()
% this function tests reading in the yaml file

stat.ok = 1;
stat.desc = '';
try
    %stat.test_ReadYaml_SimpleStructure = test_ReadYaml_SimpleStructure();   
    %stat.test_ReadYaml_DateTime = test_ReadYaml_DateTime();  
    fprintf('Testing read ');
    stat.test_RY_Matrices = test_RY_Matrices();
    fprintf('.');
    stat.test_RY_Whitespaces = test_RY_Whitespaces();
    fprintf('.');
    stat.test_RY_FloatingPoints = test_RY_FloatingPoints();
    fprintf('.');
    stat.test_RY_Indentation = test_RY_Indentation();
    fprintf('.');
    stat.test_RY_SequenceMapping = test_RY_SequenceMapping();
    fprintf('.');
    stat.test_RY_Simple = test_RY_Simple();
    fprintf('.');
    stat.test_RY_Time = test_RY_Time();
    fprintf('.');
    stat.test_RY_TimeVariants = test_RY_TimeVariants();
    fprintf('.');
    stat.test_RY_Import = test_RY_Import();
    fprintf('.');
    stat.test_RY_ImportDef = test_RY_ImportDef();
    fprintf('.');
    stat.test_RY_ImportNonex = test_RY_ImportNonex();
    fprintf('.');
    stat.test_RY_Inheritance = test_RY_Inheritance();
    fprintf('.');
    stat.test_RY_InheritanceMultiple = test_RY_InheritanceMultiple();
    fprintf('.');
    stat.test_RY_InheritanceLoop = test_RY_InheritanceLoop();
    fprintf('.');
    stat.test_RY_usecase_01 = test_RY_usecase_01();
    fprintf('.\n');
    
catch    
    stat.ok = 0;
    stat.desc  = 'Program crash';
end

end

function result = PTH_PRIMITIVES()
    result = sprintf('Data%stest_primitives%s',filesep,filesep);
end

function result = PTH_IMPORT()
    result = sprintf('Data%stest_import%s',filesep,filesep);
end

function result = PTH_INHERITANCE()
    result = sprintf('Data%stest_inheritance%s',filesep,filesep);
end

function stat = test_RY_Matrices()
    stat.ok = 1;
    stat.desc = '';
    try
        ry = ReadYaml([PTH_PRIMITIVES() 'matrices.yaml']);
        tv = load([PTH_PRIMITIVES() 'matrices.mat']);
        if ~isequal(ry, tv.testval)
            stat.desc  = 'Wrong values loaded';
            stat.ok = 0;         
        end;
    catch
        stat.ok = 0;
        stat.desc = 'Crash';
    end;
end

function stat = test_RY_FloatingPoints()
    stat.ok = 1;
    stat.desc = '';
    try
        ry = ReadYaml([PTH_PRIMITIVES() 'floating_points.yaml']);
        tv = load([PTH_PRIMITIVES() 'floating_points.mat']);
        if ~isequalwithequalnans(ry, tv.testval)
            stat.desc  = 'Wrong values loaded';
            stat.ok = 0;         
        end;
    catch
        stat.ok = 0;
        stat.desc = 'Crash';
    end;
end

function stat = test_RY_Indentation()
    stat.ok = 1;
    stat.desc = '';
    try
        ry = ReadYaml([PTH_PRIMITIVES() 'indentation.yaml']);
        tv = load([PTH_PRIMITIVES() 'indentation.mat']);
        if ~isequal(ry, tv.testval)
            stat.desc  = 'Wrong values loaded';
            stat.ok = 0;         
        end;
    catch
        stat.ok = 0;
        stat.desc = 'Crash';
    end;
end

function stat = test_RY_SequenceMapping()
    stat.ok = 1;
    stat.desc = '';
    try
        ry = ReadYaml([PTH_PRIMITIVES() 'sequence_mapping.yaml']);
        tv = load([PTH_PRIMITIVES() 'sequence_mapping.mat']);
        if ~isequal(ry, tv.testval)
            stat.desc  = 'Wrong values loaded';
            stat.ok = 0;         
        end;
    catch
        stat.ok = 0;
        stat.desc = 'Crash';
    end;
end

function stat = test_RY_Simple()
    stat.ok = 1;
    stat.desc = '';
    try
        ry = ReadYaml([PTH_PRIMITIVES() 'simple.yaml']);
        tv = load([PTH_PRIMITIVES() 'simple.mat']);
        if ~isequal(ry, tv.testval)
            stat.desc  = 'Wrong values loaded';
            stat.ok = 0;         
        end;
    catch
        stat.ok = 0;
        stat.desc = 'Crash';
    end;
end

function stat = test_RY_Time()
    stat.ok = 1;
    stat.desc = '';
    try
        ry = ReadYaml([PTH_PRIMITIVES() 'time.yaml']);
        tv = load([PTH_PRIMITIVES() 'time.mat']);
        if ~isequal(ry, tv.testval)
            stat.desc  = 'Wrong values loaded';
            stat.ok = 0;         
        end;
    catch
        stat.ok = 0;
        stat.desc = 'Crash';
    end;
end

function stat = test_RY_TimeVariants()
    stat.ok = 1;
    stat.desc = '';
    try
        ry = ReadYaml([PTH_PRIMITIVES() 'time_variants.yaml']);
        tv = load([PTH_PRIMITIVES() 'time_variants.mat']);
        if ~isequal(ry, tv.testval)
            stat.desc  = 'Wrong values loaded';
            stat.ok = 0;         
        end;
    catch
        stat.ok = 0;
        stat.desc = 'Crash';
    end;
end

function stat = test_RY_Import()
    stat.ok = 1;
    stat.desc = '';
    try
        ry = ReadYaml([PTH_IMPORT() 'import.yaml']);
        tv = load([PTH_IMPORT() 'import.mat']);
        if ~isequal(ry, tv.testval)
            stat.desc  = 'Wrong values loaded';
            stat.ok = 0;         
        end;
    catch
        stat.ok = 0;
        stat.desc = 'Crash';
    end;
end

function stat = test_RY_ImportDef()
    stat.ok = 1;
    stat.desc = '';
    try
        ry = ReadYaml([PTH_IMPORT() 'import_def.yaml']);
        tv = load([PTH_IMPORT() 'import_def.mat']);
        if ~isequal(ry, tv.testval)
            stat.desc  = 'Wrong values loaded';
            stat.ok = 0;         
        end;
    catch
        stat.ok = 0;
        stat.desc = 'Crash';
    end;
end

function stat = test_RY_ImportNonex()
    stat.ok = 0;
    stat.desc = 'Did not end with any exception.';
    try
        try
            ry = ReadYaml([PTH_IMPORT() 'import_nonex.yaml'],1);
        catch ex
            if strcmp(ex.identifier, 'MATLAB:MATYAML:FileNotFound')
                stat.desc = '';
                stat.ok = 1;
            else
                rethrow(ex);
            end;
        end;      
        
    catch
        stat.ok = 0;
        stat.desc = 'Crash';
    end;
end

function stat = test_RY_Inheritance()
    stat.ok = 1;
    stat.desc = '';
    try
        ry = ReadYaml([PTH_INHERITANCE() 'inheritance.yaml']);
        tv = load([PTH_INHERITANCE() 'inheritance.mat']);
        if ~isequal(ry, tv.testval)
            stat.desc  = 'Wrong values loaded';
            stat.ok = 0;         
        end;
    catch
        stat.ok = 0;
        stat.desc = 'Crash';
    end;
end

function stat = test_RY_InheritanceMultiple()
    stat.ok = 1;
    stat.desc = '';
    try
        ry = ReadYaml([PTH_INHERITANCE() 'inheritance_multiple.yaml']);
        tv = load([PTH_INHERITANCE() 'inheritance_multiple.mat']);
        if ~isequal(ry, tv.testval)
            stat.desc  = 'Wrong values loaded';
            stat.ok = 0;         
        end;
    catch
        stat.ok = 0;
        stat.desc = 'Crash';
    end;
end

function stat = test_RY_InheritanceLoop()
    stat.ok = 0;
    stat.desc = 'Did not end with any exception.';
    try
        try
            ry = ReadYaml([PTH_INHERITANCE() 'inheritance_loop.yaml']);
        catch ex
            if strcmp(ex.identifier, 'MATLAB:MATYAML:inheritedtwice')
                stat.desc = '';
                stat.ok = 1;
            else
                rethrow(ex);
            end;
        end;      
        
    catch
        stat.ok = 0;
        stat.desc = 'Crash';
    end;
end

function stat = test_RY_Whitespaces()
    stat.ok = 1;
    stat.desc = '';
    try
        ry = ReadYaml([PTH_PRIMITIVES() 'whitespaces.yaml']);
        if ~isfield(ry,'ImageFile') || ~isfield(ry,'ContoursCount')         
            stat.desc  = 'Wrong values loaded';
            stat.ok = 0;         
        end;
    catch
        stat.ok = 0;
        stat.desc = 'Crash';
    end;
end

function stat = test_RY_usecase_01()
    stat.ok = 1;
    stat.desc = '';
    try
        ry = ReadYaml([PTH_PRIMITIVES() 'usecase_struct_01.yaml']);
        tv = load([PTH_PRIMITIVES() 'usecase_struct_01.mat']);
        if ~isequalwithequalnans(ry, tv.testval)
            stat.desc  = 'Wrong values loaded';
            stat.ok = 0;         
        end;
    catch
        stat.ok = 0;
        stat.desc = 'Crash';
    end;
end


function stat = test_ReadYaml_SimpleStructure()

stat.ok = 1;
stat.desc = '';
try
    s = ReadYaml('simple.yaml');
    
    ages = [s.age];
    
    if not(isequal([33 27], ages))  || not(all(ismember({'John Smith', 'Mary Smith'}, {s.name}) ))
        stat.desc  = ' Wrong values loaded';
        stat.ok = 0;
    end
    
catch   
       stat.desc  = 'Program crash';
       stat.ok = 0;
end


end

function stat = test_ReadYaml_DateTime()

stat.ok = 1;
stat.desc = '';
try
    s = ReadYaml('time.yaml');
    
    if ~isa(s.Data.B1_S_SW{1},'DateTime')
        stat.desc  = ' Wrong data type of datetick';
        stat.ok = 0;
    end
    if isa(s.Data.B1_S_SW{2},'DateTime')
        stat.desc  = ' Wrong data type of datetick';
        stat.ok = 0;
    end
catch
       stat.desc  = 'Program crash';
       stat.ok = 0;
end
end