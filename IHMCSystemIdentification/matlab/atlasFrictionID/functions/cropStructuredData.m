function [croppedData] = cropStructuredData(inIndex, outIndex, allData)

if (isempty(inIndex) || isempty(outIndex) || isempty(allData))
    error('inputs are not valid');
end

% if necessary transform to column vectors
allData = toColumnVectorStructuredData(allData);

allDataField = fieldnames(allData);

for j = 1:length(allDataField)
	eval(['croppedData.',allDataField{j},' = allData.',allDataField{j},'(inIndex:outIndex,1)']);
end