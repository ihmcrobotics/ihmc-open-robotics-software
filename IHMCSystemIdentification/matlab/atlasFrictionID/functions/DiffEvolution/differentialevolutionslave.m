function differentialevolutionslave(slaveFileDir)
%DIFFERENTIALEVOLUTIONSLAVE  Start differential evolution slave process.
%  DIFFERENTIALEVOLUTIONSLAVE(DIRNAME) starts a slave process for function
%  DIFFERENTIALEVOLUTION. The given directory DIRNAME is checked for data
%  files including which function to run and which parameters to use.
%
%  DIFFERENTIALEVOLUTIONSLAVE (without input arguments) uses the default
%  directory <TEMPDIR2>/differentialevolution, where <TEMPDIR2> is the
%  temporary directory returned by function TEMPDIR2.
%
%  <a href="differentialevolution.html">differentialevolution.html</a>  <a href="http://www.mathworks.com/matlabcentral/fileexchange/18593">File Exchange</a>  <a href="https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=KAECWD2H7EJFN">Donate via PayPal</a>
%
%  Markus Buehren
%  Last modified 05.07.2011
%
%  See also DIFFERENTIALEVOLUTION, TEMPDIR2.

% get slave file directory name
if ~exist('slaveFileDir', 'var') || isempty(slaveFileDir)
  slaveFileDir = concatpath(tempdir2, 'differentialevolution');	
end
if ~exist(slaveFileDir, 'dir')
  % create slave file directory if not existing
  mkdir(slaveFileDir);
end

firstWarnTime = 10;
startWarnTime = 10*60;
maxWarnTime   = 24*3600;
startWaitTime = 0.5;
maxWaitTime   = 5;

% set random states
rand ('state', sum(clock*100));
randn('state', sum(clock*100));

lastEvaluationEndClock = clock;
lastWarnClock          = clock;
firstRun    = true;
curWarnTime = firstWarnTime;
curWaitTime = startWaitTime;
while 1
  slaveFileList = findfiles(slaveFileDir, ...
    'iteration_*_member_*_parameters.mat', 'nonrecursive');
  
  for fileNr = 1:length(slaveFileList)
    % check if there are invalid results
    resultFileName = strrep(slaveFileList{fileNr}, 'parameters', 'result');
    if existfile(resultFileName)
      deletewithsemaphores(resultFileName);
    end
  end

  % get last file that is not a semaphore file
  slaveFileName = '';
  for fileNr = length(slaveFileList):-1:1
    if isempty(strfind(slaveFileList{fileNr}, 'semaphore'))
      slaveFileName = slaveFileList{fileNr};
      break
    end
  end
  
  if ~isempty(slaveFileName)

    % load parameters from last file and remove file
    slaveFileName  = slaveFileList{end};
    resultFileName = strrep(slaveFileName, 'parameters', 'result');
    loadSuccessful = true;
    sem = setfilesemaphore(slaveFileName);
    if existfile(slaveFileName)
      try
        load(slaveFileName, 'objFctHandle', 'argumentCell');
        delete(slaveFileName);
      catch
        fprintf('Error loading or deleting file %s.\n', slaveFileName);
        displayerrorstruct;
        loadSuccessful = false;
      end
    else
      loadSuccessful = false;
    end
    removefilesemaphore(sem);

    % check
    if loadSuccessful && (~exist('argumentCell', 'var') || ...
        ~exist('objFctHandle', 'var'))
      disp(textwrap2(sprintf('Warning: Either variable ''%s'' or ''%s'' not existing after loading file %s.', ...
                             'objFctHandle', 'argumentCell', slaveFileName)));
      loadSuccessful = false;
    end

    % continue if loading was not successful
    if ~loadSuccessful
      continue
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % evaluate objective function %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if firstRun
      fprintf('First function evaluation (%s)\n', datestr(clock, 'mmm dd, HH:MM'));
      firstRun = false;
    elseif etime(clock, lastEvaluationEndClock) > 60
      fprintf('First function evaluation after %s (%s)\n', ...
                   formattime(etime(clock, lastEvaluationEndClock)), ...
                   datestr(clock, 'mmm dd, HH:MM'));
    end  
    testval = objFctHandle(argumentCell{:}); %#ok

    % check if result is valid and save
    sem = setfilesemaphore({slaveFileName, resultFileName});
    if ~existfile(slaveFileName)
      % if the same slave file was generated again in the meantime, the
      % current result is invalid
      save(resultFileName, 'testval');
    end
    removefilesemaphore(sem);  
    
    % save time
    lastEvaluationEndClock = clock;
    curWarnTime = startWarnTime;
    curWaitTime = startWaitTime;

    % remove variables before next run
    clear argumentCell objFctHandle
    
  else
    % display message if idle for long time
    timeSinceLastEvaluation = etime(clock, lastEvaluationEndClock);
    if min(timeSinceLastEvaluation, etime(clock, lastWarnClock)) > curWarnTime
      if timeSinceLastEvaluation >= 10*60
        % round to minutes
        timeSinceLastEvaluation = 60 * round(timeSinceLastEvaluation / 60);
      end
      fprintf('Warning: No slave files found in last %s (%s).\n', ...
        formattime(timeSinceLastEvaluation), datestr(clock, 'mmm dd, HH:MM'));
      lastWarnClock = clock;
      if firstRun
        curWarnTime = startWarnTime;
      else
        curWarnTime = min(curWarnTime * 2, maxWarnTime);
      end
      curWaitTime = min(curWaitTime + 1, maxWaitTime);
    end

    % wait before next check
    pause(curWaitTime);    
  end
end
