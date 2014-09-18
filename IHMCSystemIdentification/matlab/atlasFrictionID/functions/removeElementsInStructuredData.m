function [filteredData] = removeElementsInStructuredData(indexToRemove, allData)

if (isempty(indexToRemove))
    warning('indexs to temove is empty');
end
if (isempty(allData))
    warning('input data are not valid');
end

filteredData = allData;

% if necessary transform to column vectors
allData = toColumnVectorStructuredData(allData);

allDataField = fieldnames(allData);

for j = 1:length(allDataField)
	eval(['filteredData.',allDataField{j},'(indexToRemove) = [];']);
end