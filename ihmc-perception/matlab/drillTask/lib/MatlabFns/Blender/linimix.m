% LINIMIX  An Interactive Image for viewing multiple images
%
% Usage:  linimix(im, B, figNo, XY)
%
% Arguments:  im - 1D Cell array of images to be blended.  If this is not
%                  supplied, or is empty, the user is prompted with a file
%                  dialog to select a series of images.
%              B - Parameter controlling the weighting function when
%                  blending between two images.
%                  B = 0    Linear transition between images (default)
%                  B = 2^4  Sigmoidal transition at midpoint.
%                  B = 2^10 Near step-like transition at midpoint.
%          figNo - Optional figure window number to use.
%             XY - Character 'X' or 'Y' indicating whether x or y movements
%                  of the cursor controls the blending.  Defaults to 'Y'.
%
% This function provides an 'Interactive Image'.  It is intended to allow
% efficient visual exploration of a sequence of images that have been processed
% with a series of different parameter values, for example, scale.  The vertical
% position of the cursor within the image controls the linear blend between
% images.  With the cursor at the top the first image is displayed, at the
% bottom the last image is displayed. At positions in between blends of
% intermediate images are dislayed.  
%
% Click in the image to toggle in/out of blending mode.  Move the cursor up and
% down within the image to blend between the input images.
%
% Use BILINIMIX if you want the horizontal position of the cursor to be used
% too.  This will allow visual exploration of a sequence of images controlled by
% two different processing parameters.  Alternatively one could blend between
% images of two different modalities over some varing parameter, say scale.
%
% See also: BILINIMIX, TERNARYMIX, CLIQUEMIX, CYCLEMIX, LOGISTICWEIGHTING

% Reference:
% Peter Kovesi, Eun-Jung Holden and Jason Wong, 2014.
% "Interactive Multi-Image Blending for Visualization and Interpretation",
% Computers & Geosciences 72 (2014) 147-155.
% http://doi.org/10.1016/j.cageo.2014.07.010

% Copyright (c) 2012-2014 Peter Kovesi
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
%
% March 2012
% March 2014  - Allow horizontal or vertical mouse movements to control blending

function  linimix(im, B, figNo, XY)

    if ~exist('im', 'var'), im = []; end
    [im, nImages, fname] = collectncheckimages(im);

    if ~exist('B', 'var') || isempty(B), B = 0; end
    if exist('figNo','var')
        fig = figure(figNo); clf;
    else
        fig = figure;
    end
    if ~exist('XY', 'var'), XY = 'Y'; end
    XY = upper(XY);
    
    % Generate nodes for the multi-image linear blending
    [rows,cols,~] = size(im{1});
    if XY == 'Y'
        v = round([0:1/(nImages-1):1] * rows); 
    elseif XY == 'X'
        v = round([0:1/(nImages-1):1] * cols); 
    else
        error('XY must be ''x'' or ''y'' ')
    end

    fprintf('\nClick in the image to toggle in/out of blending mode \n');
    fprintf('Move the cursor within the image to blend between the input images.\n\n');
    
    S = warning('off');
    imshow(im{ max(1,fix(nImages/2)) }, 'border', 'tight');
%    imshow(im{ max(1,fix(nImages/2)) });
    % Uncomment the following line if you want axes displayed
%    imagesc(im{ max(1,fix(nImages/2)) }); colormap(gray(256))
    drawnow
    warning(S)
    
    ah = get(fig,'CurrentAxes');
    imHandle = get(ah,'Children'); % Handle to image data
    
    % Set up button down callback and window title
    set(fig, 'WindowButtonDownFcn',@wbdcb);
    set(fig, 'NumberTitle', 'off')
    set(fig, 'name', 'CET Linear Image Blender')
    set(fig, 'Menubar','none');    
    blending = 0;
    
    % Set up custom pointer
    myPointer = [circularstruct(7) zeros(15,1); zeros(1, 16)];
    myPointer(~myPointer) = NaN;
    
    hspot = 0;
    hold on

%-----------------------------------------------------------------------    
% Window button down callback.  This toggles blending on and off changing the
% cursor appropriately.

function wbdcb(src,evnt)
    if strcmp(get(src,'SelectionType'),'normal')
        if ~blending  % Turn blending on
            blending = 1;
            set(src,'Pointer','custom', 'PointerShapeCData', myPointer,...
                    'PointerShapeHotSpot',[9 9])
            set(src,'WindowButtonMotionFcn',@wbmcb)
        
            if hspot  % For paper illustration
                delete(hspot);
            end            
            
        else         % Turn blending off
            blending = 0;
            set(src,'Pointer','arrow')
            set(src,'WindowButtonMotionFcn','')
            
            % For paper illustration  (need marker size of 95 if using export_fig)
            cp = get(ah,'CurrentPoint');
            x = cp(1,1);
            y = cp(1,2);
            hspot = plot(x, y, '.', 'color', [0 0 0], 'Markersize', 50'); 
            
        end
    end
end 

%-----------------------------------------------------------------------    
% Window button move call back

function wbmcb(src,evnt)
    cp = get(ah,'CurrentPoint');
    if XY == 'Y'
        p = cp(1,2);
        p = max(0,p); p = min(rows,p);  % clamp to range 0-rows
    else
        p = cp(1,1);
        p = max(0,p); p = min(cols,p);  % clamp to range 0-cols
    end
    
    blend(p)
end

%-----------------------------------------------------------------------
function blend(p)
    
    % Find distance from each of the vertices v
    dist = abs(v - p);
    
    % Find the two closest vertices
    [dist, ind] = sort(dist);      
    
    % w1 is the fractional distance from the cursor to the 2nd image
    % relative to the distance between the 1st and 2nd images
    w1 = dist(2)/(dist(1)+dist(2));
    
    % Apply the logistics wighting function to w1 to obtain the desired
    % transition weighting
    w = logisticweighting(w1, B, [0 1 0 1]);
    
    blendim = w*im{ind(1)} + (1-w)*im{ind(2)};
    
    set(imHandle,'CData', blendim);
    set(fig, 'name', 'CET Image Blender');
end 

%---------------------------------------------------------------------------
end % of linimix

















