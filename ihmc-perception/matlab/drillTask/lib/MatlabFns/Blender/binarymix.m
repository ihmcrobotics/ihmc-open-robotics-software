% BINARYMIX  Image blending and swiping between two images
%
% Function blends two images. Each image is coloured with two lightness
% matched colours that sum to white.  Like a ternary image but binary!
% You can also switch between blending and swiping.
%
% Usage: binarymix(im, nodeLabel, normBlend, figNo)
%
% Arguments:   im - 2-element cell array of images to blend. If omitted or
%                   empty a file selection dialog box is presented.
%       nodeLabel - Optional cell array of strings for labeling the 2 nodes
%                   of the interface.  
%       normBlend - Sets the image normalisation that is used.
%                     0 - No normalisation applied other than image clamping.
%                     1 - Image is normalised so that max value in any
%                         channel is 1. This is the default
%                     2 - Normalise to have fixed mean grey value.
%                         (This is hard-wired at 0.2 .  Edit function wbmcb
%                         below to change) 
%                     3 - Normalise so that max grey value is a fixed value
%                         (This is hard-wired at 0.6 .  Edit function wbmcb
%                         below to change) 
%           figNo - Optional figure number to use.  
%
% This function sets up an interface consisting of a line with nodes at each end
% corresponding to the two input images.  Positioning the cursor along the line
% produces an interactive blend of the images formed from a weighted sum of the
% images.
%
% Hitting 'c' cycles the display between two choices for the image basis colours
% and grey scale images.  Each set of two basis colours sum to white.  Where your
% two images/data sets are in agreement, and you are at the blend mid-point, you
% will see achromatic regions in the blended result.
%
% Hitting 's' toggles between blending and swiping mode.
%
% See also: TERNARYMIX, LINIMIX, BILINIMIX, CLIQUEMIX, CYCLEMIX

% Reference:
% Peter Kovesi, Eun-Jung Holden and Jason Wong, 2014.
% "Interactive Multi-Image Blending for Visualization and Interpretation",
% Computers & Geosciences 72 (2014) 147-155.
% http://doi.org/10.1016/j.cageo.2014.07.010
%
% For information about the construction of the lighness matched basis
% colours see:
% http://peterkovesi.com/projects/colourmaps/ColourmapTheory/index.html#ternary

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
% Dec   2014 - Adapted from TERNARYMIX

% [0.00 0.71 0.00], [1.00 0.29 1.00];   % Green - Magenta
% [1 0.4  0];  b = [0 0.6 1];           % Red/Orange - Blue/Cyan

function  binarymix(im, nodeLabel, normBlend, figNo)
    
    if ~exist('im', 'var'),        im = [];       end
    if ~exist('normBlend', 'var') || isempty(normBlend), normBlend = 1; end
    if ~exist('figNo', 'var') || isempty(figNo)
        fig = figure;
    else
        fig = figure(figNo);     
    end

    [im, nImages, fname] = collectncheckimages(im);
    assert(nImages == 2, 'Three images must be supplied');

    [rows,cols] = size(im{1});
    
    if ~exist('nodeLabel', 'var') || isempty(nodeLabel)
        for n = 1:2
            nodeLabel{n} = namenpath(fname{n});
        end
    end
    
    % Set up two 'basis' images from the two input images and their
    % corresponding basis colours.
    % Set up initial lightness matched colours
    colourdisplay = 'COL1';
    imcol = {[0.00 0.71 0.00], [1.00 0.29 1.00]};
    for n = 1:2
        % It is useful to truncate extreme image values that are at the ends
        % of the histogram.  Here we remove the 0.25% extremes of the histogram.
        im{n} = histtruncate(im{n}, 0.25, 0.25);
        im{n} = normalise(im{n});  % Renormalise 0-1
        basis{n} = applycolour(im{n}, imcol{n});
    end    

    % Display 1st image and get handle to Cdata in the figure
    % Suppress display warnings for images that are large
    S = warning('off');
    figure(fig); clf, 
    imposition = [0.35 0.0 0.65 1.0];
    subplot('position', imposition);
    imshow(basis{1}); drawnow
    imHandle = get(gca,'Children');
    warning(S)
    
    % Set up interface figure 
    ah = subplot('position',[0.0 0.4 0.35 0.35]);    

    % Draw slider interface and label vertices
    theta = [0 pi];
    v = [cos(theta)
         sin(theta)];
    
    hold on;
    plot(v(1,:), v(2,:),  'k.', 'markersize',40)
    plot(v(1,:), v(2,:), '.', 'markersize',20, 'Color', [.8 .8 .8])
    line(v(1,:), v(2,:), 'LineWidth', 2)
    
    for n = 1:nImages
        text(v(1,n)*1.2, v(2, n)*1.2, nodeLabel{n}, ...
         'HorizontalAlignment','center', 'FontWeight', 'bold', 'FontSize', 16);
    end
    
    r = 1.1;
    axis ([-r r -r r]), axis off, axis equal
    
    % Set callback function and window title
    setwindowsize(fig, [rows cols], imposition);
    set(fig, 'WindowButtonDownFcn',@wbdcb);
    set(fig, 'KeyReleaseFcn', @keyreleasecb);    
    set(fig, 'NumberTitle', 'off')    
    set(fig, 'name', '  Binary Mix')
    set(fig, 'Menubar','none');

    % Set up custom pointer
    myPointer = [circularstruct(7) zeros(15,1); zeros(1, 16)];
    myPointer(~myPointer) = NaN;

    fprintf('\nClick and move the mouse on the slider.\n');
    fprintf('Left-click to toggle blending on and off.\n');
    fprintf('Hit ''c'' to cycle between two colour choices and grey .\n');
    fprintf('Hit ''s'' to toggle between swiping and blending modes.\n\n');
    blending = 0;
    hspot = 0;
    blendswipe = 'blend';  % Start in blending mode    
    
%--------------------------------------------------------------------
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
            
        else  % Turn blending off
            blending = 0;
            set(src,'Pointer','arrow')
            set(src,'WindowButtonMotionFcn','')
            
            % For paper illustration
            cp = get(ah,'CurrentPoint');
            x = cp(1,1);
            y = cp(1,2);
            if gca == ah
               hspot = plot(x, y, '.', 'color', [0 0 0], 'Markersize', 40'); 
            end
        end
    end
end
        
%--------------------------------------------------------------------
% Window button move call back
function wbmcb(src,evnt)
    cp = get(ah,'CurrentPoint');
    xy = [cp(1,1); cp(1,2)];
    
    % Project xy onto segment v1 v2
    L = norm(v(:,2) - v(:,1));
    v1v2 = (v(:,2) - v(:,1))/L;
    w = dot( xy - v(:,1), v1v2) / L;
    
    if w < 0, w = 0; end
    if w > 1, w = 1; end
    
    if strcmp(blendswipe, 'swipe')
        swipe(1-w);   
    
    else  % Blend
        blend = (1-w)*basis{1} + w*basis{2};
        
        % Various normalisation options 
        
        if normBlend == 0       % No normalisation
            blend(blend>1) = 1; % but clamp values to 1
            
        elseif normBlend == 1 || strcmpi(col, 'grey') % Normalise by max value in blend
        blend = blend/max(blend(:));
        
        elseif normBlend == 2  % Normalise to have fixed mean grey value
           g = 0.299*blend(:,:,1) + 0.587*blend(:,:,2) + 0.114*blend(:,:,3);
           g(isnan(g(:))) = [];
           meang = mean(g(:));
           blend = blend/meang*0.2;      % Mean grey value of 0.2
           blend(blend>1) = 1;           % Clamp values to 1
           
        elseif normBlend == 3  % Normalise so that max grey value is a fixed value
           g = 0.299*blend(:,:,1) + 0.587*blend(:,:,2) + 0.114*blend(:,:,3);
           blend = blend/max(g(:))*0.6;  % Max grey of 0.6
           blend(blend>1) = 1;           % Clamp values to 1
        end

        set(imHandle,'CData', blend);           
    end
    
end

%-----------------------------------------------------------------------
% Key Release callback
% If 'c' is pressed the display cycles between two choices of Lightness matched
% colours and Grey.
% Pressing 's' toggles between swiping and blending modes

function keyreleasecb(src,evnt)
    
    if evnt.Character == 'c'   %  Toggle colours
        if strcmp(colourdisplay, 'COL1')
            colourdisplay = 'COL2';
            imcol = {[1.0 0.4 0.0], [0.0 0.6 1.0]};
            for n = 1:2
                basis{n} = applycolour(im{n}, imcol{n});
            end
        
        elseif strcmp(colourdisplay, 'COL2')            
            colourdisplay = 'GREY';    
            for n = 1:2
                basis{n} = applycolour(im{n}, [1 1 1]);
            end                                
            
        elseif strcmp(colourdisplay, 'GREY')             
            colourdisplay = 'COL1';    
            imcol = {[0.00 0.71 0.00], [1.00 0.29 1.00]};
            for n = 1:2
                basis{n} = applycolour(im{n}, imcol{n});
            end                    
        end
        
    elseif evnt.Character == 's'  % Swipe/Blend toggle
       if strcmp(blendswipe, 'swipe')
           blendswipe = 'blend';
       else
           blendswipe = 'swipe';
       end
       
    end
    
    wbmcb(src,evnt); % update the display
end     

%-----------------------------------------------------------------------
% Generate swipe image given value of w between 0 and 1
function swipe(w)

    x = round(w*(cols-1));
    
    % Construct swipe image from the input images
    swipeim = [basis{1}(:, 1:x, :) basis{2}(:, x+1:end, :)];
    
    set(imHandle,'CData', swipeim);
    
end 


%------------------------------------------------------------------------
end  % of main function and its nested functions 

%------------------------------------------------------------------------
% Function to set up window size nicely
function setwindowsize(fig, imsze, imposition)
    
    screen = get(0,'ScreenSize');
    scsze = [screen(4) screen(3)];
    window = get(fig, 'Position');
    
    % Set window height to match image height
    window(4) = imsze(1);
    
    % Set window width to match image width allowing for fractional width of
    % image specified in imposition
    window(3) = imsze(2)/imposition(3);
    
    % Check size of window relative to screen.  If larger rescale the window
    % size to fit 80% of screen dimension.
    winsze =  [window(4) window(3)];
    ratio = max(winsze./scsze);
    if ratio > 1
        winsze = winsze/ratio * 0.8;
    end
    
    window(3:4) = [winsze(2) winsze(1)];
    set(fig, 'Position', window);
    
end

%----------------------------------------------------------------------------------
% Apply a colour to a scalar image

function rgbim = applycolour(im, col)
    
    [rows,cols] = size(im);
    rgbim = zeros(rows,cols,3);
    
    for r = 1:rows
        for c = 1:cols
            rgbim(r,c,:) = im(r,c)*col(:);        
        end
    end
end    