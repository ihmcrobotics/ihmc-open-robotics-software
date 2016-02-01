% CLIQUEMIX  Multi-image blending and swiping over a clique
%
% Function allows blending and swiping between any pair within a collection of images
%
% Usage: cliquemix(im, B, figNo, nodeLabel)
%
% Arguments:
%              im - A cell array of images to be blended.  If omitted, or
%                   empty, the user is prompted to select images via a file
%                   dialog. 
%               B - Parameter controlling the weighting function when
%                   blending between two images.
%                   B = 0    Linear transition between images (default)
%                   B = 2^4  Sigmoidal transition at midpoint.
%                   B = 2^10 Near step-like transition at midpoint.
%           figNo - Optional figure number.  
%       nodeLabel - Optional cell array of strings specifying the labels to
%                   be associated with the nodes on the blending interface.
%
% This function sets up an interface consisting of a regular polygon with nodes
% corresponding to the input images to be blended.  Each node is conected to
% every other node forming a clique.  Positioning the cursor along any of the
% edges joining two nodes will generate an interactive blend formed from the
% 2-image blend of the images connected by the edge.  As the mouse is moved
% around the interface the blend snaps to the edge closest to the mouse position
% (with some hysteresis to reduce unwanted transitions).  A right-click on any
% edge will lock the blending to that edge irrespective of the mouse position.
% A second right-click will unlock the edge.   
%
% Hitting 's' toggles between blending and swiping mode.
%
% This tool allows you to compare any image with any other image in a collection
% with ease.  However once you go beyond about 8 images the difference in edge
% lengths on the interface starts to make the tool less natural to use.
%
% See also: LINIMIX, BILINIMIX, TERNARYMIX, BINARYMIX, CYCLEMIX, LOGISTICWEIGHTING

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
%
% August   2011 - Original version
% May      2012 - General rework and cleanup
% March    2014 - More tidying up
% December 2014 - Incorporation of swiping option

function  cliquemix(im, B, figNo, nodeLabel)
    
    if ~exist('B', 'var') || isempty(B),   B = 0;  end % linear blending
    if ~exist('im', 'var'), im = []; end
    if ~exist('figNo', 'var') || isempty(figNo)
        fig = figure;
    else
        fig = figure(figNo);     
    end
    
    [im, nImages, fname] = collectncheckimages(im);
    
    % If interface node names have not been supplied use default image names
    if ~exist('nodeLabel', 'var') 
        nodeLabel = fname;
    end

    % Display 1st image and get handle to Cdata in figure(2)
    % Suppress display warnings for images that are large
    S = warning('off');
    figure(fig), clf;
    imposition = [0.35 0.0 0.65 1.0];
    subplot('position', imposition);
    imshow(im{1},'border', 'tight'); drawnow
    imHandle = get(gca,'Children');
    warning(S)
    
    % Draw the interface clique and label vertices 
    ah = subplot('position',[0.0 0.4 0.35 0.35]);    

    % Generate vertices of blending polygon interface
    deltaTheta = 2*pi/nImages;
    theta = [0:(nImages-1)] * deltaTheta + pi/2;
    v = [cos(theta)
         sin(theta)];

    % Construct list of edges that link the vertices
    eNo = 0;
    for n = 1:(nImages-1)
        for m = (n+1):nImages
            eNo = eNo+1;
            edge{eNo}.n1 = n;
            edge{eNo}.n2 = m;
            edge{eNo}.v1 = v(:,n);
            edge{eNo}.v2 = v(:,m);
            
            edge{eNo}.h = ...
                line([edge{eNo}.v1(1) edge{eNo}.v2(1)], ...
                     [edge{eNo}.v1(2) edge{eNo}.v2(2)], ...
                     'LineWidth', 2);
        end
    end
    
    hold on
    plot(v(1,:), v(2,:), '.', 'color', [0 0 0], 'Markersize', 30')
    hold off

    radsc = 1.2;
    for n = 1:nImages
        if v(1,n) < -0.1
            text(v(1,n)*radsc, v(2, n)*radsc, namenpath(nodeLabel{n}), ...
                 'FontSize', 16, 'FontWeight', 'bold',...
                 'HorizontalAlignment','right');
        elseif v(1,n) > 0.1
            text(v(1,n)*radsc, v(2, n)*radsc, namenpath(nodeLabel{n}), ...
                 'FontSize', 16, 'FontWeight', 'bold',...
                 'HorizontalAlignment','left');
        else
            text(v(1,n)*radsc, v(2, n)*radsc, namenpath(nodeLabel{n}), ...
                 'FontSize', 16, 'FontWeight', 'bold',...
                 'HorizontalAlignment','center');            
        end
    end

    r = 1.2; axis ([-r r -r r]), axis off, axis equal
    
    % Set callback function and window title
    [rows,cols,chan] = size(im{1});
    setwindowsize(fig, [rows cols], imposition);
    
    set(fig, 'WindowButtonDownFcn',@wbdcb);
    set(fig, 'KeyReleaseFcn', @keyreleasecb);
    set(fig, 'NumberTitle', 'off')    
    set(fig, 'name', '  CliqueMix')
    set(fig, 'Menubar','none');
    
    % Set up custom pointer
    myPointer = [circularstruct(7) zeros(15,1); zeros(1, 16)];
    myPointer(~myPointer) = NaN;    
    
    blending = 0;
    locked = 0;   
    currentEdge = 1;
    blendswipe = 'blend';  % Start in blending mode    
    
    fprintf('\nLeft-click to toggle blending on and off.\n');
    fprintf('Right-click to toggle edge locking on and off.\n');
    fprintf('A right-click will also turn blending on if it was off\n');
    fprintf('Hit ''s'' to toggle between swiping and blending modes.\n\n');
    
%--------------------------------------------------------------------
% Window button down callback
function wbdcb(src,evnt)
    
    click = get(src,'SelectionType');
    
    % Left click toggles blending
    if strcmp(click,'normal')
        if ~blending  % Turn blending on
            blending = 1;
            set(src,'Pointer','custom', 'PointerShapeCData', myPointer,...
                    'PointerShapeHotSpot',[9 9])            
            set(src,'WindowButtonMotionFcn',@wbmcb)
            
        else  % Turn blending off and unlock
            blending = 0;
            locked = 0;                
            set(src,'Pointer','arrow')
            set(src,'WindowButtonMotionFcn','')
            set(edge{currentEdge}.h, 'color', [0 0 1]);
        end
    
    % Right click toggles edge locking
    elseif strcmp(click,'alt')
        if blending && ~locked
            % Lock to closest edge
            e = closestedge;
            if e ~= currentEdge
                set(edge{currentEdge}.h, 'color', [0 0 1]);
                currentEdge = e;
            end
            set(edge{currentEdge}.h, 'color', [1 0 0]);
            locked = 1;
            
        elseif ~blending && ~locked
            % Turn blending on and lock to closest edge
            blending = 1;
            locked = 1;
            set(src,'Pointer','custom', 'PointerShapeCData', myPointer,...
                    'PointerShapeHotSpot',[9 9])            
            set(src,'WindowButtonMotionFcn',@wbmcb)            

            currentEdge = closestedge;
            set(edge{currentEdge}.h, 'color', [1 0 0]);
            
        elseif locked        % Unlock
            locked = 0;
            set(edge{currentEdge}.h, 'color', [0 0.5 0]);
        end
    end
end
        
%--------------------------------------------------------------------
% Window button move call back
function wbmcb(src,evnt)
    cp = get(ah,'CurrentPoint');
    x = cp(1,1); y = cp(1,2);
    
    [e, n1, n2, frac] = activeedge(x,y);
    
    if strcmp(blendswipe, 'blend')
        w = logisticweighting(frac, B,[0 1 0 1]);
        
        blend = w*im{n1} + (1-w)*im{n2};
    else
        blend = swipe(n1, n2, frac);
    end
    set(imHandle,'CData', blend);           
end

%---------------------------------------------------------------------
% ACTIVEEDGE
% Establish the edge in the graph that is currently 'active'. Return the node
% numbers, n1 and n2, that define the edge and the fractional distance of the
% projected cursor point along the edge from n1 to n2
%
% This version maintains a ring buffer that records the edge that the cursor
% is closest to for the last N instantiations of this function.  The 'active'
% edge is set to the edge number that appears most frequently in the ring
% buffer (the mode).  Seems to work quite well however the bahaviour is
% dependent on the speed of the computer.  It would be good to have something
% that was tied to actual time.

function [e, n1, n2, frac] = activeedge(x,y)

    N = 30;            % Buffer size
    persistent ebuf;   % Buffer for recording which edge we have been on
    persistent ptr;    % Pointer into buffer.
    
    if isempty(ebuf), ebuf = currentEdge*ones(1,N); end  
    if isempty(ptr), ptr = 0; end  
    
    if locked
        v1 = edge{currentEdge}.v1; v2 = edge{currentEdge}.v2;            
        [eDist, pfrac] = dist2segment(v1, v2, x, y);

        e = currentEdge;
        n1 = edge{currentEdge}.n1;
        n2 = edge{currentEdge}.n2;
        frac = 1-pfrac;
    
    else   % Not locked. Find edge closest to x,y

        eDist = zeros(1,length(edge));
        pfrac  = zeros(1,length(edge));    
        
        for n = 1:length(edge)
            v1 = edge{n}.v1; v2 = edge{n}.v2;            
            [eDist(n), pfrac(n)] = dist2segment(v1, v2, x, y);
        end
        
        [minDist, ind] = min(eDist);
        ptr = mod(ptr+1,N-1);
        ebuf(ptr+1) = ind;
        
        % Set edge to be the one which is most common within ebuf
        e = mode(ebuf);
        
        if e ~= currentEdge
            set(edge{currentEdge}.h, 'color', [0 0 1]);
            currentEdge = e;
        end
        
        set(edge{currentEdge}.h, 'color', [0 0.5 0]);
        
        % Identify the indices of the images associated with this edge and
        % also calculate the fractional position we are along this edge for
        % the blending
        n1 = edge{e}.n1;
        n2 = edge{e}.n2;
        frac = 1-pfrac(e);
    end
end % of activeedge


%------------------------------------------------------------------------
% Find edge closest to cursor
function e = closestedge

    cp = get(ah,'CurrentPoint');
    x = cp(1,1); y = cp(1,2);
    eDist = zeros(1,length(edge));
    
    for n = 1:length(edge)
        v1 = edge{n}.v1; v2 = edge{n}.v2;            
        eDist(n) = dist2segment(v1, v2, x, y);
    end
    
    [~, e] = min(eDist);
end

%------------------------------------------------------------------------
% Given an x,y coordinate and a line segment with end points v1 and v2. Find the
% closest point on the line segment.  Return the distance to the line segment
% and the fractional distance of the closest point on the segment from v1 to v2.

function [d, frac] = dist2segment(v1, v2, x, y)

    xy = [x;y];
    v1v2 = v2-v1;       % Vector from v1 to v2
    D = norm(v1v2);     % Distance v1 to v2
    v1xy = xy-v1;       % Vector from v1 to xy
    
    % projection of v1xy on unit vector v1v2
    proj = dot(v1v2/D, v1xy);
    
    if proj < 0      % Closest point is v1
        d = norm(v1xy);
        frac = 0;
    elseif proj > D  % Closest point is v2
        d = norm(xy-v2);
        frac = 1;
    else             % Somewhere in the middle
        pt = v1 + proj*v1v2/D;
        d = norm(xy-pt);
        frac = proj/D;
    end    

end % of dist2segment
    
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

%-----------------------------------------------------------------------
% Generate swipe image given the two end nodes of the active edge and the
% fractional distance along the edge.

function  swipeim = swipe(n1, n2, frac)
    
    % Form vectors from the centre of the image to the corners and construct
    % the dot product between v(:,n2)-v(:,n1) to determine the extreme
    % corners of the image that are relevant
    corner = [1   cols  cols   1
              1    1    rows  rows];

    % Vectors from centre of image to top-left, top-right, bottom-right,
    % bottom-left corners.
    c = corner - repmat([cols/2; rows/2], 1, 4);
        
    % Vector from node 1 to node 2
    d = v(:,n2) - v(:,n1);
    d(2) = -d(2);   % Negate y to match image coordinate frame
    d1 = d/norm(d);
    
    % Find c2, the index of corner direction maximally alligned with d
    dotcd = c'*d;   % Dot product between c and d
    [~,c1] = max(dotcd);
    c2 = mod((c1-1)-2, 4) + 1;  % Opposite corner to c2
    c1c2 = corner(:,c2)-corner(:,c1);

    % Compute distance from c1 to c2 in the direction of d
    dist = dot(c1c2, d1);    
    
    % Construct location that is 'frac' units along line parallel to
    % v(:,n2)-v(:,n1) between the two extreme image corners
    % This has to be frac*dist from c1 to c2
    p = corner(:,c1) + frac*dist*d1; 

    % Cut the image in two at the point along a line perpendicular to
    % v(:,n2)-v(:,n1) and construct a mask for defining the image parts to be
    % combined.         
    % Form equation (p - [c;r]).d
    % mask is where this value is > 0 giving all image points closest to n2
    [x,y] = meshgrid(1:cols, 1:rows);
    x = p(1) - x;
    y = p(2) - y;
    mask = (x*d(1) + y*d(2)) < 0;
    
    swipeim = im{n2};
    if chan == 1
        swipeim(mask) = im{n1}(mask);
    else  % Colour
        swipeim = zeros(rows,cols,chan);
        for ch = 1:chan
            swipeim(:,:,ch) = im{n1}(:,:,ch) .* mask + im{n2}(:,:,ch) .* ~mask;
        end
    end
    
end 

%-----------------------------------------------------------------------
% Key Release callback
% If 's' is pressed system toggles between swiping and blending modes

function keyreleasecb(src,evnt)
    
    if evnt.Character == 's'  % Swipe/Blend toggle
       if strcmp(blendswipe, 'swipe')
           blendswipe = 'blend';
       else
           blendswipe = 'swipe';
       end
    end
    
    wbmcb(src,evnt); % update the display
end    

%------------------------------------------------------------------------
end  % of everything
