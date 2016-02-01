function semaphore = setfilesemaphore(fileList)
%SETFILESEMAPHORE  Set semaphore for file access.
%   SEMAPHORE = SETFILESEMAPHORE(FILENAME) sets a semaphore to get
%   exclusive access on file FILE. The semaphore is realized by generating
%   a simple Matlab data file after checking that no other semaphores are
%   existing. The function exits if the semaphore is set. Exclusive file
%   access is of course only guaranteed if all other Matlab processes use
%   semaphores to access the same file.
%
%   The output variable SEMAPHORE is needed to correctly remove the file
%   semaphore after file access. It is an error to call function
%   SETFILESEMAPHORE without output arguments.
%
%   SEMAPHORE = SETFILESEMAPHORE(FILELIST) sets semaphores for all files
%   given in cell array FILELIST. Note that function SETFILESEMAPHORE waits
%   for exclusive file access on ALL files in the list before exiting.
%
%		Note: A semaphore older than 20 seconds is considered as invalid and
%		will immediately be deleted.
%
%		Example:
%		sem = setfilesemaphore('test.mat');
%		% access file test.mat here
%		dir test.mat.semaphore.*
%		removefilesemaphore(sem);
%
%		Markus Buehren
%		Last modified 21.12.2008
%
%   See also REMOVEFILESEMAPHORE.

persistent filesToIgnore

% set times (all in seconds)
semaphoreOldTime = 20;
fixedWaitTime    = 0.05;
checkWaitTime    = 0.1;
waitInfoPeriod   = 5;
maxRandomTime    = 0.3;

if nargout ~= 1
	error('Function %s must be called with one output argument!', mfilename);
end

if ischar(fileList)
	% single file given
	fileList = {fileList};
end

nOfFiles = length(fileList);
semaphore = cell(nOfFiles, 1);
for fileNr = 1:nOfFiles

	fileName = fileList{fileNr};

	% check if given file is itself a semaphore file
	if ~isempty(regexp(fileName, '\.semaphore\.\w+\.\d+\.mat$', 'once'))
		semaphore{fileNr, 1} = '';
		continue
	end

	% generate semaphore file pattern of current file
	semaphorePattern     = [fileName, '.semaphore.*.mat'];
	semaphorePatternPath = fileparts(semaphorePattern);

	startWaitTime   = now;
	displayWaitInfo = true;
	while 1
		dirStruct = dir(semaphorePattern);

		if ~isempty(dirStruct)
			%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
			% other semaphore file existing %
			%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
			
			% check if any semaphore is very old
			allSemaphoresOld = true;
			for k=1:length(dirStruct)
				
				% get file date
				if isempty(dirStruct(k).date)
					% it happens that the info in dirStruct is damaged, file is
					% checked again later in that case
					allSemaphoresOld = false;
					continue
				elseif isfield(dirStruct, 'datenum')
					fileDatenum = dirStruct(k).datenum;
				else
					% in older Matlab version, the field datenum seems not to exist
					fileDatenum = datenum2(dirStruct(k).date);
				end
				
				if now - fileDatenum > semaphoreOldTime / (3600*24)
					oldSemaphoreFileName = concatpath(semaphorePatternPath, dirStruct(k).name);

					% avoid to issue more than one warning for each file
					if ~isempty(filesToIgnore) && ismember(oldSemaphoreFileName, filesToIgnore)
						% ignore file
						continue
					end

					% add file to ignore list
					filesToIgnore{end+1} = oldSemaphoreFileName; %#ok
					disp(textwrap2(sprintf('Warning: Ignoring old semaphore of file %s.', fileName)));
					% limit the number of saved files
					if length(filesToIgnore) > 200
						filesToIgnore = filesToIgnore(end-100:end);
					end

					% turn off file permission warnings
					warnID = 'MATLAB:DELETE:Permission';
					warnState = warning('query', warnID);
					warning('off', warnID);

					% check if semaphore file exists
					if existfile(oldSemaphoreFileName)

						% try to remove semaphore
						nrOfAttempts = 3;
						for attemptNr = 1:nrOfAttempts
							lastwarn('');
							try
								% deletion may cause an error or a file permission warning
								delete(oldSemaphoreFileName); %% file access %%
								break
							catch
								if attemptNr == nrOfAttempts
									disp(textwrap2(sprintf('Warning: Error occured in removing old semaphore of file %s.', fileName)));
								end
							end
							pause(checkWaitTime);
						end

						% check last warning
						[lastMsg, lastWarnID] = lastwarn;
						if strcmp(lastWarnID, warnID)
							disp(textwrap2(sprintf('Warning: Unable to remove old semaphore of file %s.', fileName)));
						end
					end
					warning(warnState);
				else
					allSemaphoresOld = false;
				end
			end % k=1:length(dirStruct)

			if allSemaphoresOld
				continue
			end

			% display info
			if displayWaitInfo && (now - startWaitTime)*86400 >= waitInfoPeriod
				fprintf('Waiting for semaphore of file %s to disappear.\n', fileName);
				displayWaitInfo = false;
			end

			% wait before checking again
			pause(checkWaitTime);

		else
			%%%%%%%%%%%%%%%%%%%%%%%%%%
			% set own semaphore file %
			%%%%%%%%%%%%%%%%%%%%%%%%%%
			for attemptNr = 1:10
				% generate semaphore file name
				[randomNr, randomStr] = generaterandomnumber; %#ok
				semaphoreFileName = [fileName, '.semaphore.', gethostname, '.', randomStr, '.mat'];
				
				try
					touchfile__(semaphoreFileName); %% file access %%
					break
				catch
					fprintf('An error occured while accessing semaphore file %s:\n', semaphoreFileName);
					displayerrorstruct;

					% in very very very unlikely cases two processes might have
					% generated the same semaphore file name --> wait random time and try again
					pause(checkWaitTime + maxRandomTime * randomNr);
				end
			end

			% wait fixed time
			pause(fixedWaitTime);

			% in very unlikely cases, two semaphore files might have been created
			% at the same time
			removeOwnSemaphore = false;

			dirStruct = dir(semaphorePattern);
			if length(dirStruct) > 1
				for k=1:length(dirStruct)
					currFileName = dirStruct(k).name;
					
					if ~strcmp(currFileName, semaphoreFileName) && ...
							~isempty(filesToIgnore) && ...
							~ismember(currFileName, filesToIgnore)
						% at least one the semaphores found may not be ignored
						removeOwnSemaphore = true;
						break
					end
				end
			end
			
			if removeOwnSemaphore
				% remove own semaphore file
				for attemptNr = 1:10
					try
						% deletion may cause an error or a file permission warning
						delete(semaphoreFileName); %% file access %%
						break
					catch
						disp(textwrap2(sprintf('Warning: Error occured in removing semaphore of file %s.', fileName)));
					end
					pause(checkWaitTime);
				end

				% wait RANDOM time before checking again
				pause(maxRandomTime * generaterandomnumber);
			else
				% exclusive file access is guaranteed
				% save semaphore file name and leave while loop
				semaphore{fileNr, 1} = semaphoreFileName;
				
				if ~displayWaitInfo
					disp('File semaphore disappeared.');
				end
				break
			end
		end % if ~isempty(dirStruct)
	end % while 1
end % for fileNr = 1:nOfFiles

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [randomNr, randomStr] = generaterandomnumber
%GENERATERANDOMNUMBER
%   in very unlikely cases, it might happen that the random states of rand
%   and randn are equal in two Matlab processes calling function
%   SETFILESEMAPHORE. For this reason, the system and cpu time are used to
%   create different random numbers even in this unlikely case.
%
%		This all were not necessary if it were possible to get some sort of a
%		Matlab process ID.

nOfDigits = 8; % length of random string will be 4*nOfDigits

randNr    = rand;
randnNr   = mod(randn+0.5, 1);
cputimeNr = mod(cputime, 100)/100;
nowNr     = mod(rem(now,1)*3600*24, 100)/100;

% random number is used for random pause after conflict
randomNr = 0.25 * (randNr + randnNr + cputimeNr + nowNr);

% random string is used for the semaphore file name
if nargout > 1
	ee = 10^nOfDigits;
	randomStr = sprintf('%.0f%.0f%.0f%.0f', ...
		ee * randNr,    ...
		ee * randnNr,   ...
		ee * cputimeNr, ...
		ee * nowNr      ...
		);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function displayerrorstruct(errorStruct)
%DISPLAYERRORSTRUCT  Display structure returned by function lasterror.
%		DISPLAYERRORSTRUCT displays the structure returned by function
%		LASTERROR. Useful when catching errors.
%
%		Markus Buehren
%
%   See also LASTERROR.

if nargin == 0
	errorStruct = lasterror;
end

disp(errorStruct.message);
errorStack = errorStruct.stack;
for k=1:length(errorStack)
	fprintf('Error in ==> %s at %d.\n', errorStack(k).name, errorStack(k).line);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function success = touchfile__(fileName)
%TOUCHFILE  Generate empty file.
%		TOUCHFILE(FILENAME) generates an empty file with the given file name.
%
%		Markus Buehren
%
%   See also FOPEN.

try
	[fid, message] = fopen(fileName, 'w');
catch
	fid = -1;
	% do nothing
end

if fid == -1
	disp(message);
	success = 0;
else
	fclose(fid);
	success = 1;
end
