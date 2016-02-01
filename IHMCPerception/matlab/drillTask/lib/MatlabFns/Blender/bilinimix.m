% BILINIMIX  An Interactive Image for viewing multiple images
%
% Usage:  bilinimix(im, figNo)
%
% Arguments:  im - 2D Cell array of greyscale images to be blended.  
%          figNo - Optional figure window number to use.
%
% This function provides an 'Interactive Image'.  It is intended to allow
% efficient visual exploration of a sequence of images that have been processed
% with a series of two different parameter values, for example, scale and image
% mix.  The horizontal and vertical position of the cursor within the image
% controls the bilinear blend of the two parameters over the images.  
%
% To achieve this you need to prepare a set of images that cover a coarsely
% quantized range of properties you want to view.  Think of these images as
% being arranged in a 2D grid corresponding to the 2D cell array of images
% supplied to the function.  Positioning the cursor at some location within the
% grid will result in a blended image being constructed from the bilinear
% interpolation of the 4 images surrounding the cursor position.
%
% Click in the image to toggle in/out of blending mode.  Move the cursor 
% within the image to blend between the input images.
%
% See also: LINIMIX, TERNARYMIX, CLIQUEMIX, CYCLEMIX, LOGISTICWEIGHTING
%
% Note this code is a bit rough in places but is still reasonably useful at this
% stage.  I suggest you stick with greyscale images at this stage.

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
% April 2012 - Original version
% March 2014 - General cleanup

function  bilinimix(im, figNo)
    
    assert(iscell(im), 'Input images must be in a cell array');
    [gridRows gridCols] = size(im);

    [im, nImages, fname] = collectncheckimages(im);    
    [minrows,mincols,~] = size(im{1,1});

    fprintf('\nClick in the image to toggle in/out of blending mode \n');
    fprintf('Move the cursor up-down and left-right within the image to \n');
    fprintf('blend between the input images\n\n');

    if exist('figNo','var')
        fig = figure(figNo); clf
    else
        fig = figure;
    end
    
    S = warning('off');
    imshow(im{ max(1,fix(nImages/2)) }, 'border', 'tight');
    drawnow
    warning(S)


    ah = get(fig,'CurrentAxes');
    imHandle = get(ah,'Children');  % Gives access to Cdata in the figure

    % Set up button down callback and window title    
    set(fig, 'WindowButtonDownFcn',@wbdcb);
    set(fig, 'NumberTitle', 'off')
    set(fig, 'name', 'CET Bilinear Image Blender')    
    set(fig, 'Menubar','none');
    blending = 0;
    
    % Set up custom pointer
    myPointer = [circularstruct(7) zeros(15,1); zeros(1, 16)];
    myPointer(~myPointer) = NaN;

    hspot = 0;
    hold on
    
%-----------------------------------------------------------------------    
% Window button down callback
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
            
            % For paper illustration (need marker size of 95 if using export_fig)
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
    x = cp(1,1); y = cp(1,2);
    blend(x, y)
end

%-----------------------------------------------------------------------
function blend(x, y)
    
    % Clamp x and y to image limits
    x = max(0,x); x = min(mincols,x);  
    y = max(0,y); y = min(minrows,y);  
    
    % Compute grid coodinates of (x,y)
    % im{1,1}        im{1,2} ... im{1, gridCols}
    %    ..             ..              ..
    % im{gridRows,1}    ..   ... im{gridRows, gridCols}
    gx = x/mincols * (gridCols-1) + 1;
    gy = y/minrows * (gridRows-1) + 1;
    
    % Compute bilinear interpolation between the images
    gxf = floor(gx); gxc = ceil(gx);
    gyf = floor(gy); gyc = ceil(gy);

    frac = gxc-gx;
    blendyf = (frac)*im{gyf,gxf} + (1-frac)*im{gyf,gxc};
    blendyc = (frac)*im{gyc,gxf} + (1-frac)*im{gyc,gxc};

    frac = gyc-gy;
    blendim = (frac)*blendyf + (1-frac)*blendyc;
    
    set(imHandle,'CData', blendim);
    
end 

%---------------------------------------------------------------------------
end % of bilinimix
    
