function demo2
%DEMO2  Demo for usage of DIFFERENTIALEVOLUTION.
%    DEMO2 starts searching the minimum of Shekel's Foxholes as a demo.
%    Modify this function for your first optimization.
%
%    <a href="differentialevolution.html">differentialevolution.html</a>  <a href="http://www.mathworks.com/matlabcentral/fileexchange/18593">File Exchange</a>  <a href="https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=KAECWD2H7EJFN">Donate via PayPal</a>
%
%    Markus Buehren
%    Last modified 05.07.2011
%
%    See also DIFFERENTIALEVOLUTION, FOXHOLES.

% set title
optimInfo.title = 'Demo 2 (Shekel''s Foxholes)';

% specify objective function
objFctHandle = @foxholes;

% define parameter names, ranges, quantizations and initial values :
paramDefCell = {'', [-65 65; -65 65], [0; 0], [0; 0]};
% (single vector-valued parameter with no name, as function foxholes is
% called with a parameter vector as only input) 

% no additional function parameters needed
objFctSettings = {};

% no parameter vector needed
objFctParams = [];

% get default DE parameters
DEParams = getdefaultparams;

% set number of population members (often 10*D is suggested; here we use
% more as we know that the Foxholes functions has many local minima).
DEParams.NP = 50;

% do not use slave process here
DEParams.feedSlaveProc = 0;

% set times
DEParams.maxiter       = 100;
DEParams.maxtime       = 90;  % in seconds
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
[bestmem, bestval] = ...
	differentialevolution(DEParams, paramDefCell, objFctHandle, ...
	objFctSettings, objFctParams, emailParams, optimInfo); %#ok

disp(' ');
disp('Best parameter set returned by function differentialevolution:');
disp(bestmem);

