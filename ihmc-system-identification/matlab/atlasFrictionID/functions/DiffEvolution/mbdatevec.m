function vec = mbdatevec(n)
%MBDATEVEC  Return date vector for time in format as of function MBTIME.
%		VEC = MBDATEVEC(N) returns the date vector VEC for the time N as 
%		returned by function MBTIME (serial date number converted to seconds).
%
%		Example:
%		time = mbtime;
%		vec = mbdatevec(time);
%
%		Markus Buehren
%		Last modified 21.04.2008 
%
%		See also MBTIME, DATEVEC.

vec = datevec(n/86400);
