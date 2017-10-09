function stat = test_WriteYaml()

stat.ok = 1;
stat.desc = '';
try
    fprintf('Testing write ');
    stat.test_WY_Matrices = test_WY_Universal(PTH_PRIMITIVES(), 'matrices');
    fprintf('.');
    stat.test_WY_FloatingPoints = test_WY_Universal(PTH_PRIMITIVES(), 'floating_points');
    fprintf('.');
    stat.test_WY_Indentation = test_WY_Universal(PTH_PRIMITIVES(), 'indentation');
    fprintf('.');
    stat.test_WY_SequenceMapping = test_WY_Universal(PTH_PRIMITIVES(), 'sequence_mapping');
    fprintf('.');
    stat.test_WY_Simple = test_WY_Universal(PTH_PRIMITIVES(), 'simple');
    fprintf('.');
    stat.test_WY_Time = test_WY_Universal(PTH_PRIMITIVES(), 'time');
    fprintf('.');
    stat.test_WY_ComplexStructure = test_WY_Universal(PTH_IMPORT(), 'import');
    fprintf('.');
    stat.test_WY_usecase_01 = test_WY_Universal(PTH_PRIMITIVES(), 'usecase_struct_01');    
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

function stat = test_WY_Universal(path, filename)
    stat.ok = 1;
    stat.desc = '';
    try
        data = load([path, filesep, filename, '.mat']);
        WriteYaml('~temporary.yaml',data.testval);
        ry = ReadYaml('~temporary.yaml');
        if ~isequalwithequalnans(ry, data.testval)
            stat.desc  = 'Wrong values loaded';
            stat.ok = 0;         
        end;
    catch
        stat.ok = 0;
        stat.desc = 'Crash';
    end;    
end