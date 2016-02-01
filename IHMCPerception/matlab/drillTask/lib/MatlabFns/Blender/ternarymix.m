% TERNARYMIX  Image blending and swiping over three images
%
% Function uses Barycentric coordinates over a triangle to interpolate/blend
% three images.  You can also switch to a swiping mode of display between the
% three images.
%
% Usage: ternarymix(im, nodeLabel, normBlend, figNo)
%
% Arguments:   im - 3-element cell array of images to blend. If omitted or
%                   empty a file selection dialog box is presented.
%       nodeLabel - Optional cell array of strings for labeling the 3 nodes
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
% This function sets up an interface consisting of a triangle with nodes
% corresponding to the three input images.  Positioning the cursor within the
% triangle produces an interactive blend of the images formed from a weighted
% sum of the images.  The weights correspond to the barycentric coordinates of
% the cursor within the triangle.  The program starts out assigning a lightness
% matched basis colour to each image.  These colours are nominally red, green
% and blue but have been constructed so that they are matched in lightness
% values and also have secondary colours that are closely matched.
% Unfortunately the normalisation of the blended image undoes some of this
% lightness matching but I am not sure there is a good way around this.
%
% Hitting 'c' cycles the basis colours between the set of lightness matched
% Isoluminant colours, the RGB primaries, and grey scale images.
%
% Hitting 's' toggles between blending and swiping mode.
%
% See also: LINIMIX, BILINIMIX, CLIQUEMIX, CYCLEMIX, BINARYMIX

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
% May   2012 - Original version
% March 2014 - General cleanup and added option to specify colours associated
%              with each image when using the RGB option.
% Dec   2014 - Updated isoluminant basis colours.  Added interactive colour
%              mode section and swiping.

function  ternarymix(im, nodeLabel, normBlend, figNo)
    
    if ~exist('im', 'var'),        im = [];       end
    if ~exist('normBlend', 'var') || isempty(normBlend), normBlend = 1; end
    if ~exist('figNo', 'var') || isempty(figNo)
        fig = figure;
    else
        fig = figure(figNo);     
    end
    
    [im, nImages, fname] = collectncheckimages(im);
    assert(nImages == 3, 'Three images must be supplied');

    [rows,cols] = size(im{1});
    
    if ~exist('nodeLabel', 'var') || isempty(nodeLabel)
        for n = 1:3
            nodeLabel{n} = namenpath(fname{n});
        end
    end
    
    % Set up three 'basis' images from the three input images and their
    % corresponding basis colours.
    colourdisplay = 'ISO';
    imcol = {[0.90 0.17 0.00], [0.00 0.50 0.00], [0.10 0.33 1.00]};
    
    for n = 1:3
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

    % Draw triangle interface and label vertices
    theta = [0 2/3*pi 4/3*pi]+pi/2;
    v = [cos(theta)
         sin(theta)];
    
    hold on;
    plot([v(1,:) 0], [v(2,:) 0], 'k.', 'markersize',40)
    plot([v(1,:) 0], [v(2,:) 0], '.', 'markersize',20, 'Color', [.8 .8 .8])
    line([v(1,:) v(1,1)], [v(2,:) v(2,1)], 'LineWidth', 2)
    
    for n = 1:nImages
        text(v(1,n)*1.2, v(2, n)*1.2, nodeLabel{n}, ...
         'HorizontalAlignment','center', 'FontWeight', 'bold', 'FontSize', 16);
    end
    
    coltxt = text(-1,-1,'ISO','FontWeight', 'bold', 'FontSize', 16);
    
    r = 1.1;
    axis ([-r r -r r]), axis off, axis equal
    
    % Set callback function and window title
    setwindowsize(fig, [rows cols], imposition);
    set(fig, 'WindowButtonDownFcn',@wbdcb);
    set(fig, 'KeyReleaseFcn', @keyreleasecb);    
    set(fig, 'NumberTitle', 'off')    
    set(fig, 'name', '  Ternary Mix')
    set(fig, 'Menubar','none');

    % Set up custom pointer
    myPointer = [circularstruct(7) zeros(15,1); zeros(1, 16)];
    myPointer(~myPointer) = NaN;

    fprintf('\nClick and move the mouse within the triangle interface.\n');    
    fprintf('Left-click to toggle blending on and off.\n');    
    fprintf('Hit ''c'' to cycle between isoluminant colours, RGB and grey.\n');   
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

    if strcmp(blendswipe, 'swipe')
        swipe(xy);   

    else   % Blend
        [w1,w2,w3] = barycentriccoords(xy, v(:,1), v(:,2), v(:,3));
        blend = w1*basis{1} + w2*basis{2} + w3*basis{3};
        
        % Various normalisation options 
        
        if normBlend == 0       % No normalisation
            blend(blend>1) = 1; % but clamp values to 1
            
        elseif normBlend == 1 || strcmpi(colourdisplay, 'GREY') % Normalise by max value in blend
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
% If 'c' is pressed the display cycles between using Lightness matched
% colours, RGB and Grey.  If 's' is pressed we toggle between swiping and
% blending.

function keyreleasecb(src,evnt)
    
    if evnt.Character == 'c' 
        if strcmp(colourdisplay, 'ISO')
            colourdisplay = 'RGB';
            set(coltxt, 'String', 'RGB');
            imcol = {[1 0 0], [0 1 0], [0 0 1]}; 
            for n = 1:3
                basis{n} = applycolour(im{n}, imcol{n});
            end
        
        elseif strcmp(colourdisplay, 'RGB')            
            colourdisplay = 'GREY';    
            set(coltxt, 'String', 'Grey');
            for n = 1:3
                basis{n} = applycolour(im{n}, [1 1 1]);
            end                                
            
        elseif strcmp(colourdisplay, 'GREY')             
            colourdisplay = 'ISO';    
            set(coltxt, 'String', 'ISO');
            imcol = {[0.90 0.17 0.00], [0.00 0.50 0.00], [0.10 0.33 1.00]};     
            for n = 1:3
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
% Generate swipe image given xy which has its origin at the centre of the
% triangle with vertices at a radius of 1
function swipe(xy)

    % Convert cursor position to row and col coords
    x = round(( xy(1) + 1) * cols/2);
    y = round((-xy(2) + 1) * rows/2);
    
    % Clamp x and y to image limits
    x = max(1,x); x = min(cols,x);      
    y = max(1,y); y = min(rows,y);      
    
    % Three images. 1st image occupies top half, swiped vertically. 2nd and
    % 3rd images share bottom half and are swiped horizontally.

    swipeim = [           basis{1}(1:y, :, :)
               basis{2}(y+1:end, 1:x, :)  basis{3}(y+1:end, x+1:end, :)];
    
    set(imHandle,'CData', swipeim);
    
end 

%------------------------------------------------------------------------
end  % of main function and its nested functions 

%------------------------------------------------------------------------
% Given 3 vertices and a point convert the xy coordinates of the point to
% Barycentric coordinates with respect to vertices v1, v2 and v3. 
% Assumes all arguements are 2-element column vectors.  
% Returns 3 barycentric coordinates / weights.

function [l1,l2,l3] = barycentriccoords(p, v1, v2, v3)
    
    l = [v1-v3, v2-v3]\(p-v3);

    l1 = l(1);
    l2 = l(2);
    l3 = 1 - l1 - l2;
    
    % Simple clamping of resulting coords to range [0 1]
    l1 = min(max(l1,0),1);
    l2 = min(max(l2,0),1);
    l3 = min(max(l3,0),1);

    % Perhaps ideally if p is outside triangle p should be projected to the
    % closest point on the triangle before computing barycentric coords    
end    

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