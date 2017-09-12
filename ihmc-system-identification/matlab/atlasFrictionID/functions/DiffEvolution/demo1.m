function demo1
%DEMO1  Demo for usage of DIFFERENTIALEVOLUTION.
%    DEMO1 starts searching the minimum of Rosenbrock's saddle as a demo.
%    Modify this function for your first optimization.
%
%    <a href="differentialevolution.html">differentialevolution.html</a>  <a href="http://www.mathworks.com/matlabcentral/fileexchange/18593">File Exchange</a>  <a href="https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=KAECWD2H7EJFN">Donate via PayPal</a>
%
%    Markus Buehren
%    Last modified 05.07.2011
%
%    See also DIFFERENTIALEVOLUTION, ROSENBROCKSADDLE.

% set title
optimInfo.title = 'Demo 1 (Rosenbrock''s saddle)';

% specify objective function
objFctHandle = @rosenbrocksaddle;

% define parameter names, ranges and quantization:
paramDefCell = {
	'parameter1', [-3 3], 0.01
	'parameter2', [-3 3], 0.01
};
% 1. column: parameter names
% 2. column: paramter ranges
% 3. column: paramter quantizations
% 4. column: initial values (optional)

% set initial parameter values in struct objFctParams 
objFctParams.parameter1 =  -2;
objFctParams.parameter2 = 2.5;

% set single additional function parameter
objFctSettings = 100;

% get default DE parameters
DEParams = getdefaultparams;

% set number of population members (often 10*D is suggested) 
DEParams.NP = 20;

% do not use slave process here
DEParams.feedSlaveProc = 0;

% set times
DEParams.maxiter       = 100;
DEParams.maxtime       = 60;  % in seconds
DEParams.maxclock      = [];

% set display options
DEParams.refreshiter   = 1;
DEParams.refreshtime   = 10;  % in seconds
DEParams.refreshtime2  = 20;  % in seconds
DEParams.refreshtime3  = 40;  % in seconds

% do not send E-mails
emailParams = [];

% set random state in order to always use the same population members here
rand('state', 1);

% start differential evolution
[bestmem, bestval, bestFctParams] = ...
	differentialevolution(DEParams, paramDefCell, objFctHandle, ...
	objFctSettings, objFctParams, emailParams, optimInfo); %#ok

disp(' ');
disp('Best parameter set returned by function differentialevolution:');
disp(bestFctParams);

