function [data] = loadFrictionData(filename, delimiter, startRow, onlineSampled)
%% Initialize variables
formatSpec = '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';

%% Open the text file
fileID = fopen(filename,'r');
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'EmptyValue' ,NaN,'HeaderLines' ,startRow-1, 'ReturnOnError', false);
fclose(fileID);

%% Allocate imported array to column variable names
if onlineSampled
    data.Time = dataArray{:, 1};
    data.prePosition = dataArray{:, 2};
    data.postPosition = dataArray{:, 3};
    data.preTransmissionVelocity = dataArray{:, 4};
    data.postTransmissionVelocity = dataArray{:, 5};
    data.positivePressure = dataArray{:, 6};
    data.negativePressure = dataArray{:, 7};
    data.drivingTorque = dataArray{:, 8};
    data.commandedVelocity = dataArray{:, 9};
    data.preTransmissionAcceleration = dataArray{:, 10};
    data.postTransmissionAcceleration = dataArray{:, 11};
    data.currentTestIndex = dataArray{:, 12};
else
    data.Time = dataArray{:, 1};
    data.prePosition = dataArray{:, 2};
    data.preTransmissionVelocity = dataArray{:, 3};
    data.postTransmissionVelocity = dataArray{:, 4};
    data.positivePressure = dataArray{:, 5};
    data.negativePressure = dataArray{:, 6};
    data.drivingTorque = dataArray{:, 7};
    data.commandedVelocity = dataArray{:, 8};
end

%% Clear temporary variables
clearvars filename delimiter startRow formatSpec fileID dataArray ans;