function displayoptimizationhistory(varargin)
%DISPLAYOPTIMIZATIONHISTORY  Display optimization history.
%		DISPLAYOPTIMIZATIONHISTORY(PARAMDEFCELL, ALLMEM, ALLVAL) displays some
%		graphs visualizing the optimization history. The structure PARAMDEFCELL
%		contains the parameter definitions as passed to function
%		DIFFERENTIALEVOLUTION. Matrix ALLMEM contains the evaluated parameter
%		vectors in its columns and vector ALLVAL cotains the corresponding
%		evaluation values.
%
%   DISPLAYOPTIMIZATIONHISTORY(OPTIMRESULTS) uses the structure
%   OPTIMRESULTS (as saved by function DIFFERENTIALEVOLUTION).
%
%		Markus Buehren
%		Last modified 20.08.2008 
%
%		See also DIFFERENTIALEVOLUTION.

switch nargin
	case 1
		optimResult  = varargin{1};
		paramDefCell = optimResult.paramDefCell;
		allmem       = optimResult.allTestedMembers;
		allval       = optimResult.allEvaluationValues;
	case 3
		paramDefCell = varargin{1};
		allmem       = varargin{2};
		allval       = varargin{3};
	otherwise
		error('Wrong number of input arguments.');
end

if isempty(allmem)
	disp('No parameter vectors to display.');
	return
end

% initialize function
addbuttons__;

% plot one parameter into one figure
defaultFigurePosition = get(0, 'DefaultFigurePosition');
for parNr = 1:size(paramDefCell, 1)
	paramName  = getparametername2__(paramDefCell, parNr);
	paramName  = strrep(paramName, '_', '\_');
	paramName2 = regexprep(paramName, '^parameter ', '');
	selectfigure(sprintf('Parameter %s', paramName2), 'Position', defaultFigurePosition); clf;
	axes('FontSize', 14); hold on; addbuttons__(gcf);

	[allmemSorted, sortIndex] = sort(allmem(parNr, :));
	plot(allmemSorted, allval(sortIndex), 'b.');

	title(sprintf('All evaluation values over parameter %s', paramName2));
	xlabel(paramName);
	ylabel('evaluation value');
end

% get color map
mp      = jet(256);
mp(1,:) = [0 0 0.4]; % set dark blue for best value
Nmp     = size(mp, 1);

% scale cost values
a = 10;
p = 0.9;
maxVal = quantile2(allval, p);
if maxVal > 0
	allvalScaled = maxVal*log(a*allval/maxVal+1)/log(a+1);
	index = allval > maxVal;
	allvalScaled(index) = maxVal;
else
	allvalScaled = allval;
end

% compute bins for cost value color indices
minValScaled = min(allvalScaled);
maxValScaled = max(allvalScaled);
if minValScaled ~= maxValScaled
	edges = linspace(minValScaled, maxValScaled, Nmp+1);
	edges(1)   = -inf;
	edges(end) =  inf;
end

% compute image index
parameterBounds = cell2mat(paramDefCell(:,2));
parGridVector   = cell2mat(paramDefCell(:,3));
parGridIndex    = (parGridVector > 0);
allmemImg = zeros(size(allmem));
if any(parGridIndex)
	allmemImg(parGridIndex,:) = round((allmem(parGridIndex,:) - repmat(parameterBounds(parGridIndex,1), ...
		1, size(allmem,2))) ./ repmat(parGridVector(parGridIndex), 1, size(allmem,2)))+1;
end

% plot 2 parameters into one figure
nOfFigures = floor(size(paramDefCell, 1) / 2);
for k=1:nOfFigures

	selectfigure(sprintf('Parameters %d and %d', 2*k-1, 2*k), 'Position', ...
		defaultFigurePosition); clf;
	addbuttons__(gcf); axes('FontSize', 14); hold on;

	title('All evaluated members');
	paramName1 = getparametername2__(paramDefCell, 2*k-1);
	paramName2 = getparametername2__(paramDefCell, 2*k  );
	xlabel(strrep(paramName1, '_', '\_'));
	ylabel(strrep(paramName2, '_', '\_'));

	% create image
	createImage = all(parGridVector(2*k-1:2*k) > 0) && ...
		all(diff(parameterBounds(2*k-1:2*k,1:2), 1, 2) ./ ...
		parGridVector(2*k-1:2*k) < 500);
	if createImage
		imgX = parameterBounds(2*k-1,1):parGridVector(2*k-1): ...
			(parameterBounds(2*k-1,2)+0.1*parGridVector(2*k-1));
		imgY = parameterBounds(2*k  ,1):parGridVector(2*k)  : ...
			(parameterBounds(2*k,  2)+0.1*parGridVector(2*k)  );
		img = ones(length(imgX), length(imgY), 3);
	end

	% set axis
	ax = [paramDefCell{2*k-1,2} paramDefCell{2*k,2}];
	if createImage
		ax = [...
			[min(imgX) max(imgX)] + [-1 1]*0.5*parGridVector(2*k-1) ...
			[min(imgY) max(imgY)] + [-1 1]*0.5*parGridVector(2*k  )];
	else
		ax = [...
			ax(1)-0.02*(ax(2)-ax(1)), ax(2)+0.02*(ax(2)-ax(1)), ...
			ax(3)-0.02*(ax(4)-ax(3)), ax(4)+0.02*(ax(4)-ax(3))];
	end

	% divide members into bins and plot
	if minValScaled ~= maxValScaled
		for n=length(edges)-1:-1:1
			index = find(allvalScaled >= edges(n) & allvalScaled < edges(n+1));
			if createImage
				for m=1:length(index)
					img(allmemImg(2*k-1,index(m)), allmemImg(2*k,index(m)), :) = mp(n,:);
				end
			else
				plot(allmem(2*k-1, index), allmem(2*k, index), '.', 'Color', mp(n,:));
			end
		end
	else
		if createImage
			for m=1:length(allval)
				img(allmemImg(2*k-1,m), allmemImg(2*k,m), :) = mp(1,:);
			end
		else
			plot(allmem(2*k-1, :), allmem(2*k, :), '.', 'Color', mp(1,:));
		end
	end
	if createImage
		image(imgX, imgY, permute(img, [2 1 3]));
	end
	axis(ax);
end

% delete last 'next' button
childrenVec = get(gcf, 'Children');
for n=1:length(childrenVec)
	if strcmp(get(childrenVec(n), 'Type'), 'uicontrol') && ...
			strcmp(get(childrenVec(n), 'String'), 'next')
		delete(childrenVec(n));
		break
	end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function addbuttons__(figureHandle)

persistent firstFigureFinished
if nargin == 0
	firstFigureFinished = false;
	return
end

if firstFigureFinished
	uicontrol(figureHandle, 'Style', 'pushbutton', 'String', 'prev.', 'FontSize', 8, ...
		'Callback', sprintf('figure(%d)', figureHandle-1), 'Position', [20 6 60 20]);
end
uicontrol(figureHandle, 'Style', 'pushbutton', 'String', 'next', 'FontSize', 8, ...
	'Callback', sprintf('figure(%d)', figureHandle+1), 'Position', [100 6 60 20]);

firstFigureFinished = true;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function str = getparametername2__(paramDefCell, parNr)

if strcmp(paramDefCell{1,1}, '_1')
	str = sprintf('parameter %d', parNr);
else
	% return for example "bla(2)" for parameter name "bla_2"
	str = regexprep(paramDefCell{parNr,1}, '_(\d)+$', '($1)');
end
