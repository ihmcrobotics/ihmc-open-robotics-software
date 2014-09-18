function demo3
%DEMO3  Demo for usage of DIFFERENTIALEVOLUTION.
%    DEMO3 starts searching the minimum of Shekel's Foxholes as a demo. This
%    demo is similar to DEMO2 except that it uses a function to check
%    parameter vectors for validity. Modify this function for your first
%    optimization. 
%
%    <a href="differentialevolution.html">differentialevolution.html</a>  <a href="http://www.mathworks.com/matlabcentral/fileexchange/18593">File Exchange</a>  <a href="https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=KAECWD2H7EJFN">Donate via PayPal</a>
%
%    Markus Buehren
%    Last modified 05.07.2011
%
%    See also DIFFERENTIALEVOLUTION, FOXHOLES.

% set title
optimInfo.title = 'Demo 3 (Shekel''s Foxholes with constraint)';

% specify objective function
objFctHandle = @foxholes;

% define parameter names, ranges, quantizations and initial values :
paramDefCell = {'', [-65 65; -65 65], [0; 0], [0; -30]};
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

% do not use slave processes here
DEParams.feedSlaveProc  = 0;

% use a subfunction to check parameter vectors for validity
DEParams.validChkHandle = @demo3_constraint;

% set times
DEParams.maxiter        = 100;
DEParams.maxtime        = 60;  % in seconds
DEParams.maxclock       = [];

% set display options
DEParams.refreshiter    = 1;
DEParams.refreshtime    = 10;  % in seconds
DEParams.refreshtime2   = 20;  % in seconds
DEParams.refreshtime3   = 40;  % in seconds

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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function valid = demo3_constraint(x)
% constraint function must accept the same input arguments as the objective
% function!

x0 = [-30, -30];
r0 = 40;

% as an example, set values outside some circle to invalid
valid = (x(1) - x0(1))^2 + (x(2) - x0(2))^2 < r0^2;

