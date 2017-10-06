%% ReadMpsAdapter takes the output from readmps and puts the data in a form 
% that can be directly fed into quadprog. Simply call the function by 
% 
% obj = ReadMpsAdapter('filename',vararg)
%
% where vararg is a variable argument that if present will tell the
% function to solve the QP.

%%

function processedObj = ReadMpsAdapter(fileName,varargin)

addpath(genpath(pwd));
filePath = strcat('MpsParser/QpsFiles/',fileName);

fileNameNoExtensionVector = strsplit(fileName,'.');
fileNameToWrite = strcat(pwd,'/../resources/YamlQpProblems/',fileNameNoExtensionVector(1),'.yaml');

rawObj = readmps(filePath);
processedObj = convert(rawObj);

%optional argument to solve the QP given at filePath
if(~isempty(varargin))
    x0 = rand(size(processedObj.H,1),1);
    options = optimset('Algorithm','active-set','Display','on');
    [processedObj.X,processedObj.FVAL,exitFlag,output] = quadprog(2*processedObj.H,processedObj.f',processedObj.A,processedObj.b,processedObj.Aeq, ...
        processedObj.beq,processedObj.lb,processedObj.ub,x0,options)
end

fprintf(1,'write out\n');
fileToWrite = fopen(fileNameToWrite{1},'w+');
%WriteYaml(fileNameToWrite{1},processedObj);

end

function qpObj = convert(obj)

numberGreaterThanConstraints = length(find(strcmp('G',obj.rowtypes)));
numberEqualityConstraints = length(find(strcmp('E',obj.rowtypes)));
numberLessThanConstraints = length(find(strcmp('L',obj.rowtypes)));

qpObj.f = full(obj.A(strcmp('obj',obj.rownames),:)); % Convert from sparse to full
qpObj.H = full(obj.Q); 

[row,col] = find(obj.Q);
% H is symmetric but Q is created as sparse, so fill in extra elements
for i = 1:length(row)
   if row(i)~=col(i)
       qpObj.H(col(i),row(i)) = obj.Q(row(i),col(i));
   end 
end


if numberGreaterThanConstraints>0
    A_greater = zeros(numberGreaterThanConstraints,length(qpObj.H));
    b_greater = zeros(numberGreaterThanConstraints,1);
else
    A_greater = [];
    b_greater = [];
end

if numberLessThanConstraints>0
    A_lower = zeros(numberLessThanConstraints,length(qpObj.H));
    b_lower = zeros(numberLessThanConstraints,1);
else
    A_lower = [];
    b_lower = [];
end

if numberEqualityConstraints>0
    A_equality = zeros(numberEqualityConstraints,length(qpObj.H));
    b_equality = zeros(numberEqualityConstraints,1);
else
    A_equality = [];
    b_equality = [];
end

A = full(obj.A);
rhs = full(obj.rhs);
if ~isempty(find(obj.lbnds))
    qpObj.lb = full(obj.lbnds)';        
else
    % No lower bounds
    qpObj.lb = [];
end

if ~isempty(find(obj.ubnds));
    qpObj.ub = full(obj.ubnds)';
else
    % No upper bounds
    qpObj.ub = [];
end

Lrow = 1;
Grow = 1;
Erow = 1;

for i = 1:length(obj.rownames)
    
    rowname = obj.rownames{i};
    rowtype = obj.rowtypes{tablelookup(obj.rowtable,rowname)};
    % Fix this if stuff, might not be needed if I am more clever about it
    if strcmp(rowtype,'L') || strcmp(rowtype,'G')
        if strcmp(rowtype,'L')
            A_lower(Lrow,:) = A(tablelookup(obj.rowtable,rowname),:);
            b_lower(Lrow) = rhs(tablelookup(obj.rowtable,rowname));
            Lrow = Lrow+1;
        elseif strcmp(rowtype,'G')
            A_greater(Grow,:) = A(tablelookup(obj.rowtable,rowname),:);
            b_greater(Grow) = rhs(tablelookup(obj.rowtable,rowname));
            Grow=Grow+1;
        end
    elseif strcmp(rowtype,'E')
        A_equality(Erow,:) = A(tablelookup(obj.rowtable,rowname),:);
        b_equality(Erow) = rhs(tablelookup(obj.rowtable,rowname));
        Erow=Erow+1;
    elseif strcmp(rowtype,'N')
        display('Row name is OBJ, still not sure what this one is. I think its useless though.');
    else
        err = MException('Unrecognized ROW type', ...
        'Row type not recognized. You have L,E,G, and N to pick from.');
        throw(err);
    end
    
end

% Combine less than and greater than into a single constraint
qpObj.A = [A_lower;-A_greater];
qpObj.b = [b_lower;-b_greater];
qpObj.Aeq = A_equality;
qpObj.beq = b_equality;


end
