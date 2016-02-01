% SWIPE  Interactive image swiping between 2, 3 or 4 images.
%
% Usage  swipe(im, figNo)
%
% Arguments:  im - 2D Cell array of images to be blended.  Two, three or four
%                  images can be blended.
%          figNo - Optional figure window number to use.
%
% Click in the image to toggle in/out of swiping mode.  Move the cursor 
% within the image to swipe between the input images.
%
% See also: TERNARYMIX, CYCLEMIX, CLIQUEMIX, LINIMIX, BILINIMIX
%

% Copyright (c) 2013 Peter Kovesi
% Centre for Exploration Targeting
% The University of Western Australia
% peter.kovesi at uwa edu au
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in 
% all copies or substantial portions of the Software.
%
% The Software is provided "as is", without warranty of any kind.

% June 2013
    
function swipe(im, figNo)    
    
    if ~exist('im', 'var'), im = []; end
    [im, nImages, fname] = collectncheckimages(im);
    
    if isempty(im)  % Cancel
        return;
    end
    
    if length(im) < 2 || length(im) > 4
        error('Can only swipe between 2, 3 or 4 images');
    end
    
    [rows,cols,~] = size(im{1});
    
    fprintf('\nClick in the image to toggle in/out of swiping mode \n');
    fprintf('Move the cursor  within the image to \n');
    fprintf('swipe between the input images\n\n');
    
    % Set up figure and handles to image data
    if exist('figNo','var')
        fig = figure(figNo); clf;
    else
        fig = figure;
    end
    
    S = warning('off');
    imshow(im{1});
%    imshow(im{1}, 'border', 'tight');
    drawnow
    warning(S)
    
    set(fig, 'name', 'CET Image Swiper')
    set(fig,'Menubar','none');
    ah = get(fig,'CurrentAxes');
    imHandle = get(ah,'Children'); 
    
    % Set up button down callback
    set(fig,'WindowButtonDownFcn',@wbdcb);
    swiping = 0;
    
    % Set up custom pointer
    myPointer = [circularstruct(7) zeros(15,1); zeros(1, 16)];
    myPointer(~myPointer) = NaN;
    
    
%-----------------------------------------------------------------------    
% Window button down callback.  This toggles swiping on and off changing the
% cursor appropriately.

function wbdcb(src,evnt)
    if strcmp(get(src,'SelectionType'),'normal')
        if ~swiping  % Turn swiping on
            swiping = 1;
            set(src,'Pointer','custom', 'PointerShapeCData', myPointer,...
                    'PointerShapeHotSpot',[8 8])
            set(src,'WindowButtonMotionFcn',@wbmcb)
        
        else         % Turn swiping off
            swiping = 0;
            set(src,'Pointer','arrow')
            set(src,'WindowButtonMotionFcn','')
        end
    end
end 

%-----------------------------------------------------------------------    
% Window button move call back

function wbmcb(src,evnt)
    cp = round(get(ah,'CurrentPoint'));
    x = cp(1,1); y = cp(1,2);
    swipe(x, y)
end

%-----------------------------------------------------------------------
function swipe(x, y)
    
    % Clamp x and y to image limits
    x = max(1,x); x = min(cols,x);      
    y = max(1,y); y = min(rows,y);  
    
    % Construct swipe image from the input images
    if length(im) == 2;      % Vertical swipe between 2 images
        swipeim = [im{1}(1:y, :, :)
                   im{2}(y+1:end, :, :)];
        
    % Three images. 1st image occupies top half, swiped vertically. 2nd and
    % 3rd images share bottom half and are swiped horizontally.
    elseif length(im) == 3;
        swipeim = [           im{1}(1:y, :, :)
                   im{2}(y+1:end, 1:x, :)  im{3}(y+1:end, x+1:end, :)];
    
    % Four input images placed in quadrants
    elseif length(im) == 4;
        swipeim = [im{1}(1:y, 1:x, :)      im{2}(1:y, x+1:end, :)
                   im{3}(y+1:end, 1:x, :)  im{4}(y+1:end, x+1:end, :)];
    end
    
    set(imHandle,'CData', swipeim);
    
    set(fig, 'name', 'CET Image Swiper')       
end 

%---------------------------------------------------------------------------
end % of swipe
    
    