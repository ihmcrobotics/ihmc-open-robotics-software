function [equalizedReferenceVector, equalizedAllData] = equalizeSampleNumber(referenceVector, numberOfIntervals, ...
                                                           maxNumberOfPointsInEachInterval, allData, maxValue, minValue)
% This function equalizes the number of point in a vector, and all vectors
% which refers to this vector, divinding the vector in subgroups and
% randomly reducing the number of sample in the considered subgroup up to
% the desired maximum number of point per subgroup.
% If the number of points in a subgroup is less than the desired, the
% subgroup is not affected.
% Inputs:
% - referenceVector, the vector used as reference (x axis)
% - numberOfIntervals
% - maxNumberOfPointsInEachInterval
% - allData, struct with all other vector which will be sapled as the referenceVector
% - OPTIONAL: min and max values of the interval
% Output:
% - equalizedReferenceVector
% - equalizedAllData
narginchk(4, 6)

if nargin == 6
    if min(referenceVector) > minValue || max(referenceVector) < maxValue
        warning('input min/max value is less/greater than the min/max value of the vector');
    end
    interval = [minValue, maxValue];
else
    interval = [min(referenceVector), max(referenceVector)];
end

delta = (interval(2) - interval(1))/numberOfIntervals;
allDataField = fieldnames(allData);
equalizedReferenceVector = [];

% if necessary transform to column vectors
if ~iscolumn(referenceVector)
    referenceVector = referenceVector';
end
allData = toColumnVectorStructuredData(allData);
    


for j = 1:length(allDataField)
    eval(['equalizedAllData.',allDataField{j},' = [];']);
end
for i = 1:numberOfIntervals
    lowerBound = interval(1) + (i-1) * delta;
    upperBound = interval(1) + i * delta;
    index = find(referenceVector > lowerBound & referenceVector < upperBound);
    if length(index) > maxNumberOfPointsInEachInterval
        idx = datasample(index,maxNumberOfPointsInEachInterval,'Replace',false);
    else
        idx = index;
    end
    equalizedReferenceVector = [equalizedReferenceVector; referenceVector(idx)];
    for j = 1:length(allDataField)
        eval(['equalizedAllData.',allDataField{j},' = [equalizedAllData.',allDataField{j},'; allData.',allDataField{j},'(idx)];']);
    end
end
