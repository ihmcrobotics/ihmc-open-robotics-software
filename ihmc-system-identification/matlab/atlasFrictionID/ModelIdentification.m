% In this file we load the data coming from the experimental test on a
% single joint, and we use different models to fit the friction of the
% joint. 
% To improve the reliability of results the identification considers gravity
% and inertia components.
% All models are identified using a nonlinear optimization tecnique. 

% Inputs(csv), each sample is an ordered column containing:
% "time", "prePosition", "postPosition", "preVelocity", "postVelocity", 
% "positivePressure", "negativePressure", "measuredTorque", 
% "commandedVelocities", "preAcceleration", "postAcceleration", "currentTestIndex"
%

% Outputs:
% - link inertia
% - lumped mass*radius
% - depends on the friction model

clear all
close all
clc

onlineSampled = true;
affectedByGravity = true; 

% add subpaths
f = filesep;
addpath([pwd,f,'functions']);
addpath([pwd,f,'functions',f,'DiffEvolution']);

%% Optimization parametrs
% select the model to use 

% Available models
% 'coulombViscous', 'coulombViscousStribeck', 'pressureBased',
% 'sumOfPressures', 'hyperbolicCoulombViscous', 'asymmetric5Polynomial'
parameter.model = 'hyperbolicCoulombViscous';

% Available minimization algorithms
% 'differentialEvolution', 'fminsearchbnd' , 'fminsearch'
parameter.solver = 'fminsearchbnd'; 

% define initial values for each model
% as a rule the parameters are defined in the following order:
% - friction parameters. In case of asymmetric, positive velocity first
tempInf = 10^3;
switch parameter.model
   case 'coulombViscous'
       % [ pCoulomb, pViscous, nCoulomb, nViscous]
       parameter.initial = [10,0.2,10,0.2];
       parameter.UpperBound = [tempInf, tempInf, tempInf, tempInf];
       parameter.LowerBound = [0, 0, 0, 0];
   case 'hyperbolicCoulombViscous'
       % [ pCoulomb, pHyperCoeff, pViscous, nCoulomb, pHyperCoeff, nViscous]
       parameter.initial = [9.0000, 0.1, 0.5, 9.0000, 0.1, 0.5];
       parameter.UpperBound = [tempInf, tempInf, tempInf, tempInf, tempInf, tempInf];
       parameter.LowerBound = [0, 0, 0, 0, 0, 0];
   case 'coulombViscousStribeck'
       % [pSigma, pFc0, pFs0, pCs, nSigma, nFc0, nFs0, nCs]
       parameter.initial = [1,0.2,1,0.05,1,1,1,0.05];
       parameter.UpperBound = [tempInf, tempInf, tempInf, 0.1, tempInf, tempInf, tempInf, 0.1];
       parameter.LowerBound = [0, 0, 0, 0, 0, 0, 0, 0];
   case 'pressureBased'
       % [x1, x2, x3, x4, x5]
       parameter.initial = [1.2, 0.0139, 0.0179, -0.0053, 0.2];
       parameter.UpperBound = [tempInf, tempInf, tempInf, tempInf, tempInf];
       parameter.LowerBound = [-tempInf, -tempInf, -tempInf, -tempInf, -tempInf];
   case 'asymmetric5Polynomial'
       % [px0, px1, px2, px3, px4, px5, nx0, nx1, nx2, nx3, nx4, nx5]
       parameter.initial = [10, 0.3, 0, 0, 0, 0, -10, 0.3, 0, 0, 0, 0];
       parameter.UpperBound = [tempInf, tempInf, tempInf, tempInf, tempInf, tempInf, tempInf, tempInf, tempInf, tempInf, tempInf, tempInf];
       parameter.LowerBound = [-tempInf, -tempInf, -tempInf, -tempInf, -tempInf, -tempInf, -tempInf, -tempInf, -tempInf, -tempInf, -tempInf, -tempInf];
   otherwise
      error('Specified friction model is not valid');
end

% adding: inertia, lumped mass*radius, offset angle
parameter.initial = [0.01, 1, 0.5,parameter.initial];
parameter.UpperBound = [1, 100, pi, parameter.UpperBound];
parameter.LowerBound = [0, 0, -pi, parameter.LowerBound];


%% Load File
armSide = 'R'; 
jointName = 'SHY';
filename = [pwd,f,'data',f,'robotHot',f,'JOINT_',armSide,'_ARM_',jointName,'_frictionID.csv'];
delimiter = ',';
startRow = 2;

[data] = loadFrictionData(filename, delimiter, startRow, onlineSampled);

% make the time vector starting from 0
time = data.Time;
time = time - time(1);

%% Compute joint acceleration
if ~onlineSampled
    postAcceleration = derivativeWithLowNoise(time, data.postTransmissionVelocity);
    preAcceleration = derivativeWithLowNoise(time, data.preTransmissionVelocity);

    % time of switch between constant velocities and chirp
    velChirpSwitchTime = 371;
    velChirpSwitchIndex = find(data.Time>velChirpSwitchTime, 1);

    % acceleration looks nice for constant velocities using pre, and nice for
    % chirp using post. so we create a new acceleration vector using both
    acceleration =[preAcceleration(1:velChirpSwitchIndex); postAcceleration(velChirpSwitchIndex+1:end)];

    figure
    plot(time, postAcceleration)
    title('Post Acceleration')

    figure
    plot(time, preAcceleration)
    title('Pre Acceleration')
    
    data.jointAcceleration = acceleration;
else
    % switch between constant velocities and chirp
    velChirpSwitchIndex = find(data.currentTestIndex>0, 1);
    % acceleration looks nice for constant velocities using pre, and nice for
    % chirp using post. so we create a new acceleration vector using both
    acceleration =[data.preTransmissionAcceleration(1:velChirpSwitchIndex-1); data.postTransmissionAcceleration(velChirpSwitchIndex:end)];
    data.jointAcceleration = acceleration;
end

figure 
plot(data.Time,acceleration)
title('mixed acceleration')
grid on

%% Plot constant velocity and chirp data
velChirpSwitchIndex = find(data.currentTestIndex>0, 1);
figure 
plot(data.preTransmissionVelocity(1:velChirpSwitchIndex-1), data.drivingTorque(1:velChirpSwitchIndex-1),'x');
grid on
title('constant velocity tests')

figure
plot(data.preTransmissionVelocity(velChirpSwitchIndex:end), data.drivingTorque(velChirpSwitchIndex:end),'x')
grid on
title('chirp tests')

%% Apply filters to the input data
clear filteredData croppedData

% plot all data
figure
plot(data.Time, data.preTransmissionVelocity,'o');
title('ALL DATA - PRE-VELOCITY');
grid on

figure
plot(data.Time, data.jointAcceleration,'o');
title('ALL DATA - ACCELERATION');
grid on

% remove transitions from constant velocity tests
indexToRemove =[];
inIndex = 1;
outIndex = find(data.currentTestIndex>0, 1);
croppedData = cropStructuredData(inIndex, outIndex, data);
for i=2:length(croppedData.commandedVelocity)
    if (abs(croppedData.commandedVelocity(i) - croppedData.commandedVelocity(i-1)) > 0 && i >100)
        indexToRemove = [indexToRemove, i-30:i+300];
    end
end
croppedData = removeElementsInStructuredData(indexToRemove, data);

% remove first 300 point of the constant velocity (so we have the same
% number of point of the first neghative constant velocity)
indexToRemove =[];
for i=1:length(croppedData.drivingTorque)
    if (i < 300)
        indexToRemove = [indexToRemove, i];
    end
end
croppedData = removeElementsInStructuredData(indexToRemove, croppedData);

% remove all point with low transition torques
indexToRemove =[];
torqueThreshold = 5;
inIndex = 1;
for i=1:length(croppedData.drivingTorque)
    if (abs(croppedData.drivingTorque(i)) < torqueThreshold)
        indexToRemove = [indexToRemove, i];
    end
end
croppedData = removeElementsInStructuredData(indexToRemove, croppedData);

% remove all point with discondart velocity and torque (not for SHX because gravity heavily affects the torques)
if ~strcmp(jointName,'SHX')
    indexToRemove =[];
    inIndex = 1;
    for i=1:length(croppedData.drivingTorque)
        if (sign(croppedData.drivingTorque(i)) ~= sign(croppedData.preTransmissionVelocity(i)))
            indexToRemove = [indexToRemove, i];
        end
     end
    croppedData = removeElementsInStructuredData(indexToRemove, croppedData);
end

figure
plot(croppedData.Time, croppedData.preTransmissionVelocity);
title('INTERVAL SELECTED - WILL BE FILTERED AND USED FOR ESTIMATION');
grid on

figure 
plot(croppedData.Time,croppedData.jointAcceleration,'o')
title('mixed acceleration without transition during constant velocity')
grid on

% create velocity intervals with equal number of points
numberOfIntervals = 20;
maxNumberOfPointsInEachInterval = 50000;
maxVelocity = 3;
minVelocity = -3;
[FilteredReferenceVector, filteredData] = equalizeSampleNumber(croppedData.preTransmissionVelocity,...
                                               numberOfIntervals, maxNumberOfPointsInEachInterval,...
                                               croppedData, maxVelocity, minVelocity);
% [FilteredReferenceVector, filteredData] = equalizeSampleNumber(croppedData.preTransmissionVelocity,...
%                                                numberOfIntervals, maxNumberOfPointsInEachInterval,croppedData);
figure
plot(filteredData.Time, filteredData.preTransmissionVelocity,'*');
title('FILTERED DATA - WILL BE USED FOR ESTIMATION');
grid on
    
%% minimize square error
switch parameter.solver
    case 'fminsearch'
        options = optimset('Display','iter','MaxFunEvals',5000*length(parameter.initial));
        [x,fval,exitflag] = fminsearch(@(Variables)squareErrorComputer(Variables),parameter.initial,options);
    case 'fminsearchbnd'
        options = optimset('Display','iter','MaxFunEvals',5000*length(parameter.initial));
        [x,fval,exitflag,output] = fminsearchbnd(@(Variables)squareErrorComputer(Variables),parameter.initial,...
                                                  parameter.LowerBound, parameter.UpperBound, options);
    case 'differentialEvolution'
        [x, fval, bestFctParams] = differentialEvolution(@squareErrorComputer, parameter.model, parameter.initial,...
                                                                  parameter.LowerBound, parameter.UpperBound);
    otherwise
        error('Solver has not been properly selected')
end

%% test results
clear fr frAll frCropped
switch parameter.model
   case 'coulombViscous'
        % test results of CoulombViscous
        inertia = x(1);
        lumpedMass =  x(2);
        angleOffset = x(3);
        frictionParameters.pCoulomb = x(4);
        frictionParameters.pViscous = x(5);
        frictionParameters.nCoulomb = x(6);
        frictionParameters.nViscous = x(7);
        for i = 1:length(filteredData.preTransmissionVelocity)
            fr(i) = asymmetricCoulombViscous(filteredData.preTransmissionVelocity(i), frictionParameters);
            inertiaComp(i) = inertia * filteredData.jointAcceleration(i);
            gravity(i) = lumpedMass * cos(filteredData.prePosition(i) + angleOffset);
            fr(i) = fr(i) + gravity(i) + inertiaComp(i);
        end
        for i = 1:length(data.preTransmissionVelocity)
            frAll(i) = asymmetricCoulombViscous(data.preTransmissionVelocity(i), frictionParameters);
            inertiaCompAll(i) = inertia * data.jointAcceleration(i);
            gravityAll(i) = lumpedMass * cos(data.prePosition(i) + angleOffset);
            frAll(i) = frAll(i) + gravityAll(i) + inertiaCompAll(i);
        end
        for i = 1:length(croppedData.preTransmissionVelocity)
            frCropped(i) = asymmetricCoulombViscous(croppedData.preTransmissionVelocity(i), frictionParameters);
            inertiaCompCropped(i) = inertia * croppedData.jointAcceleration(i);
            gravityCropped(i) = lumpedMass * cos(croppedData.prePosition(i) + angleOffset);
            frCropped(i) = frCropped(i) + gravityCropped(i) + inertiaCompCropped(i);
        end
        
   case 'hyperbolicCoulombViscous'
        % test results of CoulombViscous
        inertia = x(1);
        lumpedMass =  x(2);
        angleOffset = x(3);
        frictionParameters.pCoulomb = x(4);
        frictionParameters.pHyperbolicCoeff = x(5);
        frictionParameters.pViscous = x(6);
        frictionParameters.nCoulomb = x(7);
        frictionParameters.nHyperbolicCoeff = x(8);
        frictionParameters.nViscous = x(9);
        
        for i = 1:length(filteredData.preTransmissionVelocity)
            fr(i) = asymmetricHyperbolicCoulombViscous(filteredData.preTransmissionVelocity(i), frictionParameters);
            inertiaComp(i) = inertia * filteredData.jointAcceleration(i);
            gravity(i) = lumpedMass * cos(filteredData.prePosition(i) + angleOffset);
            fr(i) = fr(i) + gravity(i) + inertiaComp(i);
        end
        for i = 1:length(data.preTransmissionVelocity)
            frAll(i) = asymmetricHyperbolicCoulombViscous(data.preTransmissionVelocity(i), frictionParameters);
            inertiaCompAll(i) = inertia * data.jointAcceleration(i);
            gravityAll(i) = lumpedMass * cos(data.prePosition(i) + angleOffset);
            frAll(i) = frAll(i) + gravityAll(i) + inertiaCompAll(i);
        end
        for i = 1:length(croppedData.preTransmissionVelocity)
            frCropped(i) = asymmetricHyperbolicCoulombViscous(croppedData.preTransmissionVelocity(i), frictionParameters);
            inertiaCompCropped(i) = inertia * croppedData.jointAcceleration(i);
            gravityCropped(i) = lumpedMass * cos(croppedData.prePosition(i) + angleOffset);
            frCropped(i) = frCropped(i) + gravityCropped(i) + inertiaCompCropped(i);
        end
   case 'coulombViscousStribeck'
       % test results of CoulombViscous
        inertia = x(1);
        lumpedMass =  x(2);
        angleOffset = x(3);
        frictionParameters.positiveSigma = x(4);
        frictionParameters.positiveFc0 = x(5);
        frictionParameters.positiveFs0 = x(6);
        frictionParameters.positiveCs = x(7);
        frictionParameters.negativeSigma = x(8);
        frictionParameters.negativeFc0 = x(9);
        frictionParameters.negativeFs0 = x(10);
        frictionParameters.negativeCs = x(11);
        for i = 1:length(filteredData.preTransmissionVelocity)
            fr(i) = asymmetricCoulombViscousStribeck(filteredData.preTransmissionVelocity(i), frictionParameters);
            inertiaComp(i) = inertia * filteredData.jointAcceleration(i);
            gravity(i) = lumpedMass * cos(filteredData.prePosition(i) + angleOffset);
            fr(i) = fr(i) + gravity(i) + inertiaComp(i);
        end
        for i = 1:length(data.preTransmissionVelocity)
            frAll(i) = asymmetricCoulombViscousStribeck(data.preTransmissionVelocity(i), frictionParameters);
            inertiaCompAll(i) = inertia * data.jointAcceleration(i);
            gravityAll(i) = lumpedMass * cos(data.prePosition(i) + angleOffset);
            frAll(i) = frAll(i) + gravityAll(i) + inertiaCompAll(i);
        end
        for i = 1:length(croppedData.preTransmissionVelocity)
            frCropped(i) = asymmetricCoulombViscousStribeck(croppedData.preTransmissionVelocity(i), frictionParameters);
            inertiaCompCropped(i) = inertia * croppedData.jointAcceleration(i);
            gravityCropped(i) = lumpedMass * cos(croppedData.prePosition(i) + angleOffset);
            frCropped(i) = frCropped(i) + gravityCropped(i) + inertiaCompCropped(i);
        end
        
   case 'pressureBased'
        % test results of CoulombViscous
         inertia = x(1);
         lumpedMass =  x(2);
         angleOffset = x(3);
         frictionParameters.x1 = x(4);
         frictionParameters.x2 = x(5);
         frictionParameters.x3 = x(6);
         frictionParameters.x4 = x(7);
         frictionParameters.x5 = x(8);
        for i = 1:length(filteredData.preTransmissionVelocity)
            fr(i) = asymmetricPressureBased(filteredData.preTransmissionVelocity(i), filteredData.positivePressure(i), filteredData.negativePressure(i), frictionParameters); 
            inertiaComp(i) = inertia * filteredData.jointAcceleration(i);
            gravity(i) = lumpedMass * cos(filteredData.prePosition(i) + angleOffset);
            fr(i) = fr(i) + gravity(i) + inertiaComp(i);
        end
        for i = 1:length(data.preTransmissionVelocity)
            frAll(i) = asymmetricPressureBased(data.preTransmissionVelocity(i), data.positivePressure(i), data.negativePressure(i), frictionParameters); 
            inertiaCompAll(i) = inertia * data.jointAcceleration(i);
            gravityAll(i) = lumpedMass * cos(data.prePosition(i) + angleOffset);
            frAll(i) = frAll(i) + gravityAll(i) + inertiaCompAll(i);
        end
        for i = 1:length(croppedData.preTransmissionVelocity)
            frCropped(i) = asymmetricPressureBased(croppedData.preTransmissionVelocity(i), croppedData.positivePressure(i), croppedData.negativePressure(i), frictionParameters);
            inertiaCompCropped(i) = inertia * croppedData.jointAcceleration(i);
            gravityCropped(i) = lumpedMass * cos(croppeData.prePosition(i) + angleOffset);
            frCropped(i) = frCropped(i) + gravityCropped(i) + inertiaCompCropped(i);
        end
        
         case 'asymmetric5Polynomial'
        % test results of asymmetric4Polynomial
        inertia = x(1);
        lumpedMass =  x(2);
        angleOffset = x(3);
        frictionParameters.px0 = x(4);
        frictionParameters.px1 = x(5);
        frictionParameters.px2 = x(6);
        frictionParameters.px3 = x(7);
        frictionParameters.px4 = x(8);
        frictionParameters.px5 = x(9);
        frictionParameters.nx0 = x(10);
        frictionParameters.nx1 = x(11);
        frictionParameters.nx2 = x(12);
        frictionParameters.nx3 = x(13);
        frictionParameters.nx4 = x(14);
        frictionParameters.nx5 = x(15);
        for i = 1:length(filteredData.preTransmissionVelocity)
            fr(i) = asymmetric5Polynomial(filteredData.preTransmissionVelocity(i), frictionParameters);
            inertiaComp(i) = inertia * filteredData.jointAcceleration(i);
            gravity(i) = lumpedMass * cos(filteredData.prePosition(i) + angleOffset);
            fr(i) = fr(i) + gravity(i) + inertiaComp(i);
        end
        for i = 1:length(data.preTransmissionVelocity)
            frAll(i) = asymmetric5Polynomial(data.preTransmissionVelocity(i), frictionParameters);
            inertiaCompAll(i) = inertia * data.jointAcceleration(i);
            gravityAll(i) = lumpedMass * cos(data.prePosition(i) + angleOffset);
            frAll(i) = frAll(i) + gravityAll(i) + inertiaCompAll(i);
        end
        for i = 1:length(croppedData.preTransmissionVelocity)
            frCropped(i) = asymmetric5Polynomial(croppedData.preTransmissionVelocity(i), frictionParameters);
            inertiaCompCropped(i) = inertia * croppedData.jointAcceleration(i);
            gravityCropped(i) = lumpedMass * cos(croppedData.prePosition(i) + angleOffset);
            frCropped(i) = frCropped(i) + gravityCropped(i) + inertiaCompCropped(i);
        end
        
   otherwise
      error('Specified friction model is not valid');
end

% plot training set
figure
plot(filteredData.preTransmissionVelocity,filteredData.drivingTorque,'*')
hold on
plot(filteredData.preTransmissionVelocity, fr, 'ro')
title('plot over training data');
grid on

% plot cropped set
figure
plot(croppedData.preTransmissionVelocity,croppedData.drivingTorque,'*')
hold on
plot(croppedData.preTransmissionVelocity, frCropped, 'ro')
title('plot over cropped data');
grid on

% plot all set
figure
plot(data.preTransmissionVelocity,data.drivingTorque,'*')
hold on
plot(data.preTransmissionVelocity, frAll, 'ro')
title('plot over all data');
grid on

figure
plot(croppedData.prePosition,gravityCropped)
title('gravity vs joint position');
grid on

figure
plot(data.preTransmissionVelocity,data.drivingTorque - gravityAll','*')
title('torque - gravity vs velocity');
grid on

figure
plot(data.preTransmissionVelocity,data.drivingTorque' - (gravityAll + inertiaCompAll),'*')
title('torque - gravity - inertialComponent vs velocity');
grid on