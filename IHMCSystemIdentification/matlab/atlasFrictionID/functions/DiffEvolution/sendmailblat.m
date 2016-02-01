function varargout = sendmailblat(subject, body, varargin)
%SENDMAILBLAT  Send E-mail using the Windows executable BLAT.EXE.
%		SENDMAILBLAT(SUBJECT, BODY, PARAMS) sends an E-mail using BLAT. On
%		Linux systems, the command MAIL is used. 
%
%		This function is called from function DIFFERENTIALEVOLUTION and not
%		intended for direct usage. 
%
%		Markus Buehren
%		Last modified 09.02.2008 
%
%		See also DIFFERENTIALEVOLUTION.

persistent sendingPossible blat blatExisting
persistent serveraddress fromaddress toaddress password username

if nargin > 2
	sendingPossible = true;
	emailParams = varargin{1};
	if ~isempty(emailParams)
		if isfield(emailParams, 'serveraddress') && isfield(emailParams, 'fromaddress')
			serveraddress = emailParams.serveraddress;
			fromaddress   = emailParams.fromaddress;
		elseif ispc
			disp('Warning: Server address and from address needed for sending E-mail with blat.exe');
			sendingPossible = false;
		end
		toaddress = emailParams.toaddress;
		if isfield(emailParams, 'username')
			username = emailParams.username;
			if isfield(emailParams, 'password')
				password = emailParams.password;
			else
				error('When proving a username for sending E-mails, also a password has to be provided!');
			end
		end
	else
		sendingPossible = false;
	end
	varargout = {sendingPossible};
	return
end

if ~sendingPossible
	return
end

if ispc && isempty(blat)
	blat = 'blat.exe';
	blatExisting = existfile(blat);
	if ~blatExisting
		error('File blat.exe not found, please download from http://www.blat.net/. Unable to send message.');
	end
end

if ispc && blatExisting

	blatSuccessful = false;
	if ~isempty(serveraddress)
		try %#ok
			% put E-mail body into temporary file
			textFileName = sprintf('diffevolmessagetext%s.txt', sprintf('%02d', round(clock)));
			if exist('./data', 'dir')
				textFileName = ['data' filesep textFileName];
			end

			% get exclusive file access
			sem = setfilesemaphore(textFileName);

			fid = fopen(textFileName, 'w');
			fwrite(fid, body, 'uchar');
			fclose(fid);

			% send E-mail with blat
			[res, status] = system(sprintf('%s -install %s %s', blat, serveraddress, fromaddress)); %#ok
			if isempty(password)
				[res2, status2] = system(sprintf('%s %s -to %s -s \"%s\"', ...
					blat, textFileName, toaddress, subject)); %#ok
			else
				[res2, status2] = system(sprintf('%s %s -to %s -s \"%s\" -u %s -pw %s', ...
					blat, textFileName, toaddress, subject, username, password)); %#ok
			end
			if ~res && ~res2
				blatSuccessful = true;
			end

			% remove temporary file
			delete(textFileName);

			% release file
			removefilesemaphore(sem);
		end
	end
	if ~blatSuccessful
		disp('Sending message with blat.exe failed.');
	end

elseif ~ispc
	% send E-mail over linux command line
	result = system(sprintf('echo \"%s\" | mail %s -s \"%s\"', body, toaddress, subject));
	if result
		disp('Sending message over linux command line failed.');
	end
end
