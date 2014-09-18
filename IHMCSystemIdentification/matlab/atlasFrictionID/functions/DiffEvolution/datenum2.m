function dateNr = datenum2(dateStr) %#ok
%DATENUM2  Return serial date number.
%		DATENUM2(STR) returns the serial date number for the given date string
%		STR. Function DATENUM2 is a wrapper for DATENUM which replaces german
%		month abbreviations like "Okt" by the english versions like "Oct"
%		before forwarding the string to function DATENUM.
%
%		Function DATENUM2 first tries to build a date vector from the given
%		string for performance reasons. However, only date format 0
%		(dd-mmm-yyyy HH:MM:SS) is supported for this. If the given date string
%		is in a different format, the string is forwarded to function DATENUM.
%
%		Markus Buehren
%		Last modified 28.12.2008 
%
%		See also DATENUM, TRANSLATEDATESTR.

tokenCell = regexp(dateStr, '(\d+)-(\w+)-(\d+) (\d+):(\d+):(\d+)', 'tokens');
tokenCell = tokenCell{1};
if length(tokenCell) == 6
	% supported date format (at least it seems so)

	% get month
	month = [];
	switch tokenCell{2}
		case 'Jan'
			month = 1;
		case 'Feb'
			month = 2;
		case {'Mar', 'Mär', 'Mrz'}
			month = 3;
		case 'Apr'
			month = 4;
		case {'May', 'Mai'}
			month = 5;
		case 'Jun'
			month = 6;
		case 'Jul'
			month = 7;
		case 'Aug'
			month = 8;
		case 'Sep'
			month = 9;
		case {'Oct', 'Okt'}
			month = 10;
		case 'Nov'
			month = 11;
		case {'Dec', 'Dez'}
			month = 12;
		otherwise
			% try to find the month for some chinese Matlab versions
			try
				monthCell = regexp(tokenCell{2},'(\d+)','tokens');
				if ~isempty(monthCell)
					month = str2double(monthCell{1}{1});
				end
			catch
				% do nothing
			end
	end

	if isempty(month) || month < 1 || month > 12
		% obviously the data format is not supported
		disp('Date format not supported (1)');
		dateNr = datenum(translatedatestr(dateStr));
		return
	end
	
	dateVec = [...
		str2double(tokenCell{3}), ... % year
		month, ...                    % month
		str2double(tokenCell{1}), ... % day
		str2double(tokenCell{4}), ... % hours
		str2double(tokenCell{5}), ... % minutes
		str2double(tokenCell{6}), ... % seconds
		];
	dateNr = datenum(dateVec);

else
	% unknown date format
	disp('Date format not supported (2)');
	dateNr = datenum(translatedatestr(dateStr));
end
