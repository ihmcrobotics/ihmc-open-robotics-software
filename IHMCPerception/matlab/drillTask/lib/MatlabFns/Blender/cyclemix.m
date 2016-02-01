% CYCLEMIX  Multi-image blending over a cyclic sequence of images
%
% Usage: cyclemix(im, figNo, nodeLabel)
%
% Arguments:
%              im - A cell array of images to be blended.  If omitted, or
%                   empty, the user is prompted to select images via a file
%                   dialog. 
%           figNo - Optional figure number.  
%       nodeLabel - Optional cell array of strings specifying the labels to
%                   be associated with the nodes on the blending interface.
%
% This function sets up an interface consisting of a circle with equispaced
% nodes around it corresponding to the input images to be blended.  An
% additional virtual node corresponding to the average of all the images is
% placed at the centre of the circle.  Positioning the mouse at some location
% within the circle generates an interactive blend formed from the weighted sum
% of the two image nodes that the mouse is between (in an angular sense) and
% between the central average image according to the radial position of the
% mouse.
%
% An application where this blending tool can be useful is to blend between a
% sequence of images that have been filtered according to a parameter that is
% cyclic, say an orientation filter.
%
% See also: LINIMIX, BILINIMIX, CLIQUEMIX, TERNARYMIX

% Reference:
% Peter Kovesi, Eun-Jung Holden and Jason Wong, 2014.
% "Interactive Multi-Image Blending for Visualization and Interpretation",
% Computers & Geosciences 72 (2014) 147-155.
% http://doi.org/10.1016/j.cageo.2014.07.010

% Copyright (c) 2011-2014 Peter Kovesi
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

%  August 2011  Original version
%  March  2014  Cleanup and documentation

function  cyclemix(im, figNo)
    
    if ~exist('im', 'var'), im = []; end
    if ~exist('figNo', 'var') || isempty(figNo)
        fig = figure;
    else
        fig = figure(figNo);     
    end

    [im, nImages, fname] = collectncheckimages(im);
    
    % Compute average image
    avim = zeros(size(im{1}));
    for n = 1:nImages
        avim = avim + im{n};
    end
    avim = avim/nImages;
    
    % Generate vertices of blending polygon interface
    deltaTheta = 2*pi/nImages;
    theta = [0:(nImages-1)] * deltaTheta;
    v = [cos(theta)
         sin(theta)];

    % Display 1st image and get handle to Cdata in the figure
    % Suppress display warnings for images that are large
    S = warning('off');    
    figure(fig), clf,
    imposition = [0.35 0.0 0.65 1.0];
    subplot('position', imposition);
    imshow(normalise(im{1}), 'border', 'tight');
    set(fig, 'NumberTitle', 'off')    
    set(fig, 'name', '  CycleMix')
    set(fig, 'Menubar','none');
    imHandle = get(gca,'Children');
    warning(S);
    
    % Set up interface figure 
    ah = subplot('position',[0.0 0.4 0.35 0.35]);        

    h =    circle([0 0], 1, 36, [0 0 1]);
    set(h,'linewidth',2)
    hold on
    plot(v(1,:), v(2,:), '.', 'color', [0 0 0], 'markersize', 40')
    plot(v(1,:), v(2,:), '.', 'color',[.9 .9 .9], 'markersize',20)
    
    radsc = 1.2;
    for n = 1:nImages
        if v(1,n) < -0.1
            text(v(1,n)*radsc, v(2, n)*radsc, namenpath(fname{n}), ...
                 'FontSize', 16, 'FontWeight', 'bold',...
                 'HorizontalAlignment','right');
        elseif v(1,n) > 0.1
            text(v(1,n)*radsc, v(2, n)*radsc, namenpath(fname{n}), ...
                 'FontSize', 16, 'FontWeight', 'bold',...
                 'HorizontalAlignment','left');
        else
            text(v(1,n)*radsc, v(2, n)*radsc, namenpath(fname{n}), ...
                 'FontSize', 16, 'FontWeight', 'bold',...
                 'HorizontalAlignment','center');            
        end
    end
    
    r = 1.25;
    axis ([-r r -r r]), axis off, axis equal

    % Set callback function
    set(fig,'WindowButtonDownFcn',@wbdcb);
    [rows, cols, ~] = size(im{1});
    setwindowsize(fig, [rows cols], imposition);
    
    % Set up custom pointer
    myPointer = [circularstruct(7) zeros(15,1); zeros(1, 16)];
    myPointer(~myPointer) = NaN;    

    fprintf('\nLeft-click to toggle blending on and off.\n');    
    blending = 0;
    hspot = 0;

%----------------------------------------------------------------    
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

%----------------------------------------------------------------
% Window button move call back
function wbmcb(src,evnt)
    cp = get(ah,'CurrentPoint');
    x = cp(1,1);
    y = cp(1,2);
    blend(x,y)
end
        
%-------------------------------------------------------------------
function blend(x,y)
    
    % Get radius and angle of cursor relative to centre of circle
    radius = min(norm([x y]), .999); % Clamp radius to a max of 1
    ang = atan2(y,x);
    
    % Define a sigmoidal weighting function for radius values ranging from 0 to
    % 1 yielding weights that range from 1 to 0.  This is the weighting that is
    % applied to the average image associated with the centre of the circular
    % interface.  Using a sigmoidal function provides a slight 'flat spot' at
    % the centre (and edges) of the interface where the weights do not change
    % much.  This makes the interface a bit easier to use near the centre
    % which otherwise forms a singularity with respect to cursor movements.
    radialWeight = logisticweighting(radius, 2^4, [0 1 1 0]);
    
    % Compute angular distance to all vertices
    ds = sin(theta) * cos(ang) - cos(theta) * sin(ang);    % Difference in sine.
    dc = cos(theta) * cos(ang) + sin(theta) * sin(ang);    % Difference in cosine.
    distTheta = abs(atan2(ds,dc));                         % Absolute angular distance.
    
    % First form a 2-image blend from the two images that are on each side of
    % the cursor in an angular sense
    
    % Zero out angular distances > deltaTheta, these will correspond to nodes
    % that are not on each side of the cursor in an angular sense.  There should
    % be only two non-zero elements after this.
    distTheta(distTheta > deltaTheta) = 0;
    
    % Form normalised angular blending weights.
    angularWeight = (deltaTheta-distTheta)/deltaTheta;
    
    % Identify the indices of the non-zero weights
    ind = find(distTheta); 
    
    if length(ind) == 2
        blend = angularWeight(ind(1))*im{ind(1)} + angularWeight(ind(2))*im{ind(2)};
    elseif length(ind) == 1 % Unlikely, but possible.
        blend = angularWeight(ind(1))*im{ind(1)};
    end
    
    % Now form a 2-image blend between this image and the centre, average image.
    blend = (1-radialWeight)*blend + radialWeight*avim;
    
    blend = normalise(blend);
    set(imHandle,'CData', blend);
    
end  % end of blend

%---------------------------------------------------------------------
end  % of main function and nested functions
    
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
