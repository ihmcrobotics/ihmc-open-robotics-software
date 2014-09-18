function figureHandle = selectfigure(figureTag, varargin)
%SELECTFIGURE  Select figure by user-defined tag.
%		HANDLE = SELECTFIGURE(TAG) sets the figure with the 'Tag' property
%		value equal to TAG as the active figure. If no figure with that tag is
%		existing, a new figure with name and tag TAG is generated. A handle to
%		the figure is returned in output argument HANDLE.
%
%		HANDLE = SELECTFIGURE(TAG, Prop1, Val1, Prop2, Val2, ...) passes the
%		property value pairs to the figure command. For example, use HANDLE =
%		SELECTFIGURE(TAG, 'Name', NAME) to set the figure name to NAME instead
%		of TAG.
%
%		Example:
%		figure('Tag', 'testfig', 'Name', 'Test figure');
%		% do something else
%		selectfigure('testfig');		
%
%		Markus Buehren
%		Last modified 21.04.2008 

if nargin == 0
	figureTag = 'default figure';
elseif isempty(figureTag) || ~ischar(figureTag)
	error('First argument must be a non-empty string.');
end
if mod(length(varargin), 2) == 1
	error('Incomplete property-value list.');
end
if any(strcmp(varargin(1:2:end), 'Tag'))
	error('''Tag'' property may only be provided in first argument.');
end

ch = get(0, 'Children');
figureHandle = [];
for k=1:length(ch)
	if strcmp(get(ch(k), 'Type'), 'figure') && isequal(get(ch(k), 'Tag'), figureTag) 
		figureHandle = ch(k);
		break
	end
end

if ~isempty(figureHandle)
	% figure with given tag found
	figure(figureHandle);

	% (re-)set figure properties
	if ~isempty(varargin)
		%set(figureHandle, 'Name', figureTag, varargin{:});
		set(figureHandle, varargin{:});
	end
else
	% generate figure with given tag
	figureHandle = figure('Tag', figureTag, 'Name', figureTag, 'Numbertitle', 'off', varargin{:});
end
