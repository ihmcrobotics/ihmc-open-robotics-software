function [ret] = toColumnVectorStructuredData(allData)

allDataField = fieldnames(allData);

% if necessary transform to column vectors

for j = 1:length(allDataField)
    eval(['isColumnVector = iscolumn(allData.',allDataField{j},');']);
    if ~isColumnVector
        eval(['ret.',allDataField{j},' = allData.',allDataField{j},'''',';']);
    else
        eval(['ret.',allDataField{j},' = allData.',allDataField{j},';']);
    end
end
