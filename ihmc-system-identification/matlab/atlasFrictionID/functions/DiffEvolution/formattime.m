function timeString = formattime(time, mode)
%FORMATTIME  Return formatted time string.
%		STR = FORMATTIME(TIME) returns a formatted time string for the given
%		time difference TIME in seconds, i.e. '1 hour and 5 minutes' for TIME =
%		3900.
%
%		FORMATTIME(TIME, MODE) uses the specified display mode ('long' or
%		'short'). Default is long display.
%
%		Example:
%		str = formattime(142, 'long');
%
%		FORMATTIME (without input arguments) shows further examples.
%
%		Markus Buehren
%		Last modified 21.04.2008 
%
%		See also ETIME.

if nargin == 0
	disp(sprintf('\nExamples for strings returned by function %s.m:', mfilename));
	time = [0 1e-4 0.1 1 1.1 2 60 61 62 120 121 122 3600 3660 3720 7200 7260 7320 ...
		3600*24 3600*25 3600*26 3600*48 3600*49 3600*50];
	for k=1:length(time)
		disp(sprintf('time = %6g, timeString = ''%s''', time(k), formattime(time(k))));
	end
	if nargout > 0
		timeString = '';
	end
	return
end

if ~exist('mode', 'var')
	mode = 'long';
end

if time < 0
	disp('Warning: Time must be greater or equal zero.');
	timeString = '';
elseif time >= 3600*24
	days = floor(time / (3600*24));
	if days > 1
		dayString = 'days';
	else
		dayString = 'day';
	end
	hours = floor(mod(time, 3600*24) / 3600);
	if hours == 0
		timeString = sprintf('%d %s', days, dayString);
	else
		if hours > 1
			hourString = 'hours';
		else
			hourString = 'hour';
		end
		timeString = sprintf('%d %s and %d %s', days, dayString, hours, hourString);
	end

elseif time >= 3600
	hours = floor(mod(time, 3600*24) / 3600);
	if hours > 1
		hourString = 'hours';
	else
		hourString = 'hour';
	end
	minutes = floor(mod(time, 3600) / 60);
	if minutes == 0
		timeString = sprintf('%d %s', hours, hourString);
	else
		if minutes > 1
			minuteString = 'minutes';
		else
			minuteString = 'minute';
		end
		timeString = sprintf('%d %s and %d %s', hours, hourString, minutes, minuteString);
	end

elseif time >= 60
	minutes = floor(time / 60);
	if minutes > 1
		minuteString = 'minutes';
	else
		minuteString = 'minute';
	end
	seconds = floor(mod(time, 60));
	if seconds == 0
		timeString = sprintf('%d %s', minutes, minuteString);
	else
		if seconds > 1
			secondString = 'seconds';
		else
			secondString = 'second';
		end
		timeString = sprintf('%d %s and %d %s', minutes, minuteString, seconds, secondString);
	end

else
	if time > 10
		seconds = floor(time);
	else
		seconds = floor(time * 100) / 100;
	end
	if seconds > 0
		if seconds ~= 1
			timeString = sprintf('%.4g seconds', seconds);
		else
			timeString = '1 second';
		end
	else
		timeString = sprintf('%.4g seconds', time);
	end
end


switch mode
	case 'long'
		% do nothing
	case 'short'
		timeString = strrep(timeString, ' and ', ' ');
		timeString = strrep(timeString, ' days', 'd');
		timeString = strrep(timeString, ' day', 'd');
		timeString = strrep(timeString, ' hours', 'h');
		timeString = strrep(timeString, ' hour', 'h');
		timeString = strrep(timeString, ' minutes', 'm');
		timeString = strrep(timeString, ' minute', 'm');
		timeString = strrep(timeString, ' seconds', 's');
		timeString = strrep(timeString, ' second', 's');
	otherwise
		error('Mode ''%s'' unknown in function %s.', mode, mfilename);
end
