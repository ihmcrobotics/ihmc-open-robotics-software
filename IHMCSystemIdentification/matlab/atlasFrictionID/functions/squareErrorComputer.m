function [SquareError] = squareErrorComputer(parameters)
% Square error computer for different friction models.
% Expected inputs:
% * 'parameters', array or structure containing:
%   - inertial parameter
%   - lumped mass radius
%   - offset angle
%   - ordered stiction parameters

% Furthermore the following data are assumed to be in the BASE workspace:
% * 'dataFiltered', structure containing:
%   - joint space velocity
%   - actuator positive pressure
%   - actuator negative pressure
%   - actuator driving torque (measured torque)
%   - joint acceleration
% * 'parameter.model' containing the selected friction model

data = evalin('base','filteredData');
frictionModel = evalin('base','parameter.model');

jointPrePosition = data.prePosition;
jointPreVelocity = data.preTransmissionVelocity;
positivePressure = data.positivePressure;
negativePressure = data.negativePressure;
drivingTorque = data.drivingTorque;
jointAcceleration = data.jointAcceleration;

numberOfAcquisition = length(data.drivingTorque);

switch frictionModel
   case 'coulombViscous' 
       if isstruct(parameters)
           inertia = parameters.inertia;
           lumpedMass =  parameters.lumpedMass;
           angleOffset = parameters.angleOffset;
           frictionParameters.pCoulomb = parameters.pCoulomb;
           frictionParameters.pViscous = parameters.pViscous;
           frictionParameters.nCoulomb = parameters.nCoulomb;
           frictionParameters.nViscous = parameters.nViscous;
       else
           inertia = parameters(1);
           lumpedMass =  parameters(2);
           angleOffset = parameters(3);
           frictionParameters.pCoulomb = parameters(4);
           frictionParameters.pViscous = parameters(5);
           frictionParameters.nCoulomb = parameters(6);
           frictionParameters.nViscous = parameters(7);
       end
       squareErrors = zeros(1, numberOfAcquisition);
       for i = 1 : numberOfAcquisition
           frictionForce = asymmetricCoulombViscous(jointPreVelocity(i),frictionParameters);
           inertialComponent = inertia * jointAcceleration(i);
           gravity = lumpedMass * cos(jointPrePosition(i) + angleOffset);
           forceError = drivingTorque(i) - frictionForce - inertialComponent - gravity;
           squareErrors(i) = forceError^2;
       end
       
   case 'hyperbolicCoulombViscous' 
       if isstruct(parameters)
           inertia = parameters.inertia;
           lumpedMass =  parameters.lumpedMass;
           angleOffset = parameters.angleOffset;
           frictionParameters.pCoulomb = parameters.pCoulomb;
           frictionParameters.pHyperbolicCoeff = parameters.pHyperbolicCoeff;
           frictionParameters.pViscous = parameters.pViscous;
           frictionParameters.nCoulomb = parameters.nCoulomb;
           frictionParameters.nHyperbolicCoeff = parameters.nHyperbolicCoeff;
           frictionParameters.nViscous = parameters.nViscous;
       else
           inertia = parameters(1);
           lumpedMass =  parameters(2);
           angleOffset = parameters(3);
           frictionParameters.pCoulomb = parameters(4);
           frictionParameters.pHyperbolicCoeff = parameters(5);
           frictionParameters.pViscous = parameters(6);
           frictionParameters.nCoulomb = parameters(7);
           frictionParameters.nHyperbolicCoeff = parameters(8);
           frictionParameters.nViscous = parameters(9);
           
           
       end
       squareErrors = zeros(1, numberOfAcquisition);
       for i = 1 : numberOfAcquisition
           frictionForce = asymmetricHyperbolicCoulombViscous(jointPreVelocity(i),frictionParameters);
           inertialComponent = inertia * jointAcceleration(i);
           gravity = lumpedMass * cos(jointPrePosition(i) + angleOffset);
           forceError = drivingTorque(i) - frictionForce - inertialComponent - gravity;
           squareErrors(i) = forceError^2;
       end
       
   case 'pressureBased'
       if isstruct(parameters)
           inertia = parameters.inertia;
           lumpedMass =  parameters.lumpedMass;
           angleOffset = parameters.angleOffset;
           frictionParameters.x1 = parameters.x1;
           frictionParameters.x2 = parameters.x2;
           frictionParameters.x3 = parameters.x3;
           frictionParameters.x4 = parameters.x4;
           frictionParameters.x5 = parameters.x5;
       else
           inertia = parameters(1);
           lumpedMass =  parameters(2);
           angleOffset = parameters(3);
           frictionParameters.x1 = parameters(4);
           frictionParameters.x2 = parameters(5);
           frictionParameters.x3 = parameters(6);
           frictionParameters.x4 = parameters(7);
           frictionParameters.x5 = parameters(8);
       end
       squareErrors = zeros(1, numberOfAcquisition);
       for i = 1 : numberOfAcquisition 
           frictionForce = asymmetricPressureBased(jointPreVelocity(i), positivePressure(i),...
                                                   negativePressure(i), frictionParameters);
           inertialComponent = inertia * jointAcceleration(i);
           gravity = lumpedMass * cos(jointPrePosition(i) + angleOffset);
           forceError = drivingTorque(i) - frictionForce - inertialComponent - gravity;
           squareErrors(i) = forceError^2;
       end
       
   case 'coulombViscousStribeck'
       if isstruct(parameters)
           inertia = parameters.inertia;
           lumpedMass =  parameters.lumpedMass;
           angleOffset = parameters.angleOffset;
           frictionParameters.positiveSigma = parameters.positiveSigma;
           frictionParameters.positiveFc0 = parameters.positiveFc0;
           frictionParameters.positiveFs0 = parameters.positiveFs0;
           frictionParameters.positiveCs = parameters.positiveCs;
           frictionParameters.negativeSigma = parameters.negativeSigma;
           frictionParameters.negativeFc0 = parameters.negativeFc0;
           frictionParameters.negativeFs0 = parameters.negativeFs0;
           frictionParameters.negativeCs = parameters.negativeCs;
       else
           inertia = parameters(1);
           lumpedMass =  parameters(2);
           angleOffset = parameters(3);
           frictionParameters.positiveSigma = parameters(4);
           frictionParameters.positiveFc0 = parameters(5);
           frictionParameters.positiveFs0 = parameters(6);
           frictionParameters.positiveCs = parameters(7);
           frictionParameters.negativeSigma = parameters(8);
           frictionParameters.negativeFc0 = parameters(9);
           frictionParameters.negativeFs0 = parameters(10);
           frictionParameters.negativeCs = parameters(11);
       end
       squareErrors = zeros(1, numberOfAcquisition);
       for i = 1 : numberOfAcquisition
           frictionForce = asymmetricCoulombViscousStribeck(jointPreVelocity(i),frictionParameters);
           inertialComponent = inertia * jointAcceleration(i);
           gravity = lumpedMass * cos(jointPrePosition(i) + angleOffset);
           forceError = drivingTorque(i) - frictionForce - inertialComponent - gravity;
           squareErrors(i) = forceError^2;
       end
       
       case 'asymmetric5Polynomial'
       if isstruct(parameters)
           inertia = parameters.inertia;
           lumpedMass =  parameters.lumpedMass;
           angleOffset = parameters.angleOffset;
           frictionParameters.px0 = parameters.px0;
           frictionParameters.px1 = parameters.px1;
           frictionParameters.px2 = parameters.px2;
           frictionParameters.px3 = parameters.px3;
           frictionParameters.px4 = parameters.px4;
           frictionParameters.px5 = parameters.px5;
           frictionParameters.nx0 = parameters.nx0;
           frictionParameters.nx1 = parameters.nx1;
           frictionParameters.nx2 = parameters.nx2;
           frictionParameters.nx3 = parameters.nx3;
           frictionParameters.nx4 = parameters.nx4;
           frictionParameters.nx5 = parameters.px5;
       else
           inertia = parameters(1);
           lumpedMass =  parameters(2);
           angleOffset = parameters(3);
           frictionParameters.px0 = parameters(4);
           frictionParameters.px1 = parameters(5);
           frictionParameters.px2 = parameters(6);
           frictionParameters.px3 = parameters(7);
           frictionParameters.px4 = parameters(8);
           frictionParameters.px5 = parameters(9);
           frictionParameters.nx0 = parameters(10);
           frictionParameters.nx1 = parameters(11);
           frictionParameters.nx2 = parameters(12);
           frictionParameters.nx3 = parameters(13);
           frictionParameters.nx4 = parameters(14);
           frictionParameters.nx5 = parameters(15);
       end
       squareErrors = zeros(1, numberOfAcquisition);
       for i = 1 : numberOfAcquisition 
           frictionForce = asymmetric5Polynomial(jointPreVelocity(i), frictionParameters);
           inertialComponent = inertia * jointAcceleration(i);
           gravity = lumpedMass * cos(jointPrePosition(i) + angleOffset);
           forceError = drivingTorque(i) - frictionForce - inertialComponent - gravity;
           squareErrors(i) = forceError^2;
       end
       
   otherwise
      error('Specified friction model is not valid');
end

SquareError = sum(squareErrors);

% str = sprintf('%5.3f %5.3f %5.3f %5.3f %5.3f %5.3f', parameters.inertia,parameters.x1,...
%               parameters.x2,parameters.x3,parameters.x4,parameters.x5,SquareError);
% disp(str);