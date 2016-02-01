function [bestmem, bestval, bestFctParams] = differentialEvolution(objFctHandle, frictionModel, initialValues,...
                                                                   LowerBound, UpperBound)
% DIFFERENTIALEVOLUTION
% configuration file
%
% Avoid unuseful warnings 
%#ok<*NBRAK>
%#ok<*RAND>

% set title
optimInfo.title = 'Friction Identification';

% define parameter names, ranges and quantization:
% 1. column: parameter names
% 2. column: paramter ranges
% 3. column: paramter quantizations
% 4. column: initial values (optional)
switch frictionModel
    case 'coulombViscous'
        paramDefCell = {
            'inertia', [LowerBound(1) UpperBound(1)], 0
            'lumpedMass', [LowerBound(2) UpperBound(2)],0
            'angleOffset',[LowerBound(3) UpperBound(3)], 0
            'pCoulomb', [LowerBound(4) UpperBound(4)], 0
            'pViscous', [LowerBound(5) UpperBound(5)], 0 
            'nCoulomb', [LowerBound(6) UpperBound(6)], 0
            'nViscous', [LowerBound(7) UpperBound(7)], 0 
            };
        objFctParams.inertia =  initialValues(1);
        objFctParams.lumpedMass =  initialValues(2);
        objFctParams.angleOffset =  initialValues(3);
        objFctParams.pCoulomb =  initialValues(4);
        objFctParams.pViscous =  initialValues(5);
        objFctParams.nCoulomb =  initialValues(6);
        objFctParams.nViscous =  initialValues(7);
        
   case 'hyperbolicCoulombViscous'
        paramDefCell = {
            'inertia', [LowerBound(1) UpperBound(1)], 0
            'lumpedMass', [LowerBound(2) UpperBound(2)],0
            'angleOffset',[LowerBound(3) UpperBound(3)], 0
            'pCoulomb', [LowerBound(4) UpperBound(4)], 0
            'pHyperbolicCoeff', [LowerBound(5) UpperBound(5)], 0
            'pViscous', [LowerBound(6) UpperBound(6)], 0 
            'nCoulomb', [LowerBound(7) UpperBound(7)], 0
            'nHyperbolicCoeff', [LowerBound(8) UpperBound(8)], 0
            'nViscous', [LowerBound(9) UpperBound(9)], 0 
            };
        objFctParams.inertia =  initialValues(1);
        objFctParams.lumpedMass =  initialValues(2);
        objFctParams.angleOffset =  initialValues(3);
        objFctParams.pCoulomb =  initialValues(4);
        objFctParams.pHyperbolicCoeff =  initialValues(5);
        objFctParams.pViscous =  initialValues(6);
        objFctParams.nCoulomb =  initialValues(7);
        objFctParams.nHyperbolicCoeff =  initialValues(8);
        objFctParams.nViscous =  initialValues(9);
        
    case 'pressureBased'
        paramDefCell = {
            'inertia', [LowerBound(1) UpperBound(1)], 0  
            'lumpedMass', [LowerBound(2) UpperBound(2)],0
            'angleOffset',[LowerBound(3) UpperBound(3)], 0
            'x1', [LowerBound(4) UpperBound(4)], 0
            'x2', [LowerBound(5) UpperBound(5)], 0 
            'x3', [LowerBound(6) UpperBound(6)], 0
            'x4', [LowerBound(7) UpperBound(7)], 0 
            'x5', [LowerBound(8) UpperBound(8)], 0 
            };
        objFctParams.inertia =  initialValues(1);
        objFctParams.lumpedMass =  initialValues(2);
        objFctParams.angleOffset =  initialValues(3);
        objFctParams.x1 =  initialValues(4);
        objFctParams.x2 =  initialValues(5);
        objFctParams.x3 =  initialValues(6);
        objFctParams.x4 =  initialValues(7);
        objFctParams.x5 =  initialValues(8);
    case 'coulombViscousStribeck'
        paramDefCell = {
            'inertia', [LowerBound(1) UpperBound(1)], 0 
            'lumpedMass', [LowerBound(2) UpperBound(2)],0
            'angleOffset',[LowerBound(3) UpperBound(3)], 0
            'positiveSigma', [LowerBound(4) UpperBound(4)], 0
            'positiveFc0', [LowerBound(5) UpperBound(5)], 0 
            'positiveFs0', [LowerBound(6) UpperBound(6)], 0
            'positiveCs', [LowerBound(7) UpperBound(7)], 0 
            'negativeSigma', [LowerBound(8) UpperBound(8)], 0
            'negativeFc0', [LowerBound(9) UpperBound(9)], 0 
            'negativeFs0', [LowerBound(10) UpperBound(10)], 0
            'negativeCs', [LowerBound(11) UpperBound(11)], 0 
            };
        objFctParams.inertia =  initialValues(1);
        objFctParams.lumpedMass =  initialValues(2);
        objFctParams.angleOffset =  initialValues(3);
        objFctParams.positiveSigma = initialValues(4);
        objFctParams.positiveFc0 = initialValues(5);
        objFctParams.positiveFs0 = initialValues(6);
        objFctParams.positiveCs = initialValues(7);
        objFctParams.negativeSigma = initialValues(8);
        objFctParams.negativeFc0 = initialValues(9);
        objFctParams.negativeFs0 = initialValues(10);
        objFctParams.negativeCs = initialValues(11);
    otherwise
        error('Solver has not been properly selected')
end

% set single additional function parameter
objFctSettings = {};

% get default DE parameters
DEParams = getdefaultparams;

% set number of population members (often 10*D is suggested) 
DEParams.NP = 10 * length(initialValues);

% do not use slave process here
DEParams.feedSlaveProc = 0;

% set times
DEParams.maxiter       = 2000;
DEParams.maxtime       = [Inf];   % in seconds
DEParams.maxclock      = [];

% set display options
DEParams.refreshiter   = 1;
DEParams.refreshtime   = 60;  % in seconds
DEParams.refreshtime2  = 200;  % in seconds
DEParams.refreshtime3  = 400000;  % in seconds

% do not send E-mails
emailParams = [];

% set random state in order to always use the same population members here
rand('state', 1); 

% start differential evolution
[bestmem, bestval, bestFctParams] = ...
	differentialevolution(DEParams, paramDefCell, objFctHandle, ...
	objFctSettings, objFctParams, emailParams, optimInfo); 

disp(' ');
disp('Best parameter set returned by function differentialevolution:');
disp(bestFctParams);