function val = rosenbrocksaddle(scale, params)
%ROSENBROCKSADDLE  Standard optimization function for demonstration.
%		VALUE = ROSENBROCKSADDLE(SCALE, PAR) returns the value of Rosenbrock's
%		saddle at (PAR.parameter1, PAR.parameters2) scaled with input parameter
%		SCALE. The absolute minimum of the function is at (1, 1).
%
%		Markus Buehren
%		Last modified 03.02.2008 

x = params.parameter1(1);
y = params.parameter2(1);
val = scale*((y-x.^2).^2+(1-x).^2);
pause(0.1);
