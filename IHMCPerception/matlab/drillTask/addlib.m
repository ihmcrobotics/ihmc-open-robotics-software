
folders = dir('lib');
for f=folders'
    if(f.isdir && f.name(1)~='.' )
        addpath(['lib/' f.name])
    end
end
