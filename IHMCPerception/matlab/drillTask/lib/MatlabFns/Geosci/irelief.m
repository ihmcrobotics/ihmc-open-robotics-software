% IRELIEF  Interactive Relief Shading
%
% Usage:  irelief(im, rgbim, figNo)
%
% Arguments:  im - Image/heightmap to be relief shaded
%          rgbim - Optional RGB image to which the shading pattern derived
%                  from 'im' is applied. Alternatively, rgbim can be a Nx3
%                  RGB colourmap which is applied to the input
%                  image/heightmap in order to obtain a RGB image to which
%                  the shading pattern is applied.
%          figNo - Optional figure window number to use.
%
% This function provides an interactive relief shading image
%
% Click in the image to toggle in/out of interactive relief shading mode.  The
% location of the click defines a reference point which is indicated by a
% surrounding circle.  Positioning the cursor at the centre of the circle
% corresponds to the sun being placed directly overhead.  Moving it radially
% outwards reduces the sun's elevation and moving it around the circle
% corresponds to changing the sun's azimuth.  The intended use is that you would
% click on a feature of interest within an image and then move the sun around
% with respect to that feature to illuminate it from various directions.
%
% Lambertian shading is used to form the relief image.  This is the cosine of
% the angle between the surface normal and light direction.  Note that shadows
% are ignored.  Thus a small feature that might otherwise be in the shadow of a
% nearby large structure is rendered as if the large feature was not there.
%
% Note that the scale of the surface gradient of the input image is arbitrary,
% indeed it is likely to be of mixed units.  However, the gradient scale has a
% big effect on the resulting image display.  Initially the gradient is scaled
% to achieve a median value of 0.25.  Using the up and down arrow keys the
% user can successively double or halve the gradient values to obtain a
% pleasing result.
%
% A note on colourmaps: It is strongly suggested that you use a constant
% lightness colourmap, or low contrast colourmap, to construct the image to
% which the shading is applied.  The reason for this is that the perception of
% features within the data is provided by the relief shading.  If the colourmap
% itself has a wide range of lightness values within its colours then these will
% induce an independent shading pattern that will interfere with the relief
% shading.  Thus, the use of a map having colours of uniform lightness ensures
% that they do not interfere with the perception of features induced by the
% relief shading.
%
% See also: RELIEF, APPLYCOLOURMAP

% Copyright (c) 2014 Peter Kovesi
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

% April 2014  

function  irelief(im, rgbim, figNo)

    [rows, cols, chan] = size(im);
    assert(chan==1)
    
    if exist('rgbim', 'var') 
        [rr, cc, ch] = size(rgbim);
        if cc == 3 && ch == 1  % Assume this is a colourmap that is to be
                               % applied to the image/heightmap 
            rgbim = applycolourmap(im, rgbim);
        
        elseif ~isempty(rgbim) % Check its size
            if rows ~= rr || cols ~= cc || ch ~= 3
               error('Sizes of im and rgbim are not compatible'); 
            end
        end

        shrgbim = zeros(rows,cols,3);  % Allocate space for shaded image
        
    else  % No image supplied
        rgbim = []; 
    end
    
    % Compute a gradient scaling to give initial median gradient of 0.25   
    gradscale = initialgradscale;
    loggrad = 'linear';

    % Max radius value for normalising radius values when determining
    % elevation and azimuth
    maxRadius =  min(rows/6,cols/6);
    
    fprintf('\nClick in the image to toggle in/out of relief rendering mode. \n\n');
    fprintf(['The clicked location is the reference point for specifying the' ...
             ' sun direction. \n']);
    fprintf('Move the cursor with respect to this point to change the illumination.\n\n');
    fprintf('Use up and down arrow keys to increase/decrease surface gradients.\n\n');
    
    if exist('figNo','var')
        fig = figure(figNo); clf
    else
        fig = figure;
    end
    
    % precompute surface normals    
    [n1, n2, n3] = surfacenormals(im, gradscale, loggrad);
    
    % Generate an initial dummy image to display and obtain its handle
    S = warning('off');
    imHandle = imshow(ones(rows,cols), 'border', 'tight');
    ah = get(fig,'CurrentAxes');
    
    if isempty(rgbim)    % Use a slightly reduced contrast grey colourmap
        colormap(labmaplib(2));  
    end
    
    % Set initial reference point to the centre of the image and draw a
    % circle around this point.
    xo = cols/2;
    yo = rows/2;
    [xd, yd] = circlexy([xo, yo], maxRadius); 
    ho = line(xd, yd, 'Color', [.8 .8 .8]);
    set(ho,'Visible', 'off');

    % Set up button down callback and window title    
    set(fig, 'WindowButtonDownFcn', @wbdcb);
    set(fig, 'KeyReleaseFcn', @keyreleasecb);
    set(fig, 'NumberTitle', 'off')
    set(fig, 'name', 'CET Interactive Relief Shading Tool')    
    set(fig, 'Menubar','none');
    
    % Text area to display current azimuth, elevation, gradscale values
    texth = text(50, 50,'', 'color', [1 1 1], 'FontSize', 20);
  
    % Generate initial image display.
    relief(xo+maxRadius, yo-maxRadius);
    
    drawnow
    warning(S)    
    
    blending = 0;
    
    % Set up custom pointer
    myPointer = [circularstruct(7) zeros(15,1); zeros(1, 16)];
    myPointer(~myPointer) = NaN;

    hold on
    
%-----------------------------------------------------------------------    
% Window button down callback
function wbdcb(src,evnt)
    
    if strcmp(get(src,'SelectionType'),'normal')
        if ~blending  % Turn blending on
            blending = 1;
            set(ho,'Visible', 'on');
            set(src,'Pointer','custom', 'PointerShapeCData', myPointer,...
                    'PointerShapeHotSpot',[9 9])
            set(src,'WindowButtonMotionFcn',@wbmcb)

            % Reset reference point to the click location
            cp = get(ah,'CurrentPoint');
            xo = cp(1,1);
            yo = cp(1,2);
            
            [xd, yd] = circlexy([xo, yo], maxRadius); 
            set(ho,'XData', xd);
            set(ho,'YData', yd);
            
        else         % Turn blending off
            blending = 0;
            set(ho,'Visible', 'off');
            set(src,'Pointer','arrow')
            set(src,'WindowButtonMotionFcn','')
            
            % For paper illustration (need marker size of 95 if using export_fig)
            cp = get(ah,'CurrentPoint');
            x = cp(1,1);
            y = cp(1,2);
        end
    
    % Right-clicks while blending also reset the origin for determining elevation and azimuth
    elseif blending && strcmp(get(src,'SelectionType'),'alt') 

        cp = get(ah,'CurrentPoint');
        xo = cp(1,1);
        yo = cp(1,2);
        
        [xd, yd] = circlexy([xo, yo], maxRadius); 
        set(ho,'XData', xd);
        set(ho,'YData', yd);
        
        wbmcb(src,evnt); % update the display
    end
    
end 

%-----------------------------------------------------------------------    
% Window button move call back
function wbmcb(src,evnt)
    cp = get(ah,'CurrentPoint');
    x = cp(1,1); y = cp(1,2);
    relief(x, y);
end

%-----------------------------------------------------------------------
% Key Release callback
% If '+' or up is pressed the gradients in the image are doubled
% '-' or down halves the gradients

function keyreleasecb(src,evnt)
    
    if evnt.Character == '+' | evnt.Character == 30
        gradscale = gradscale*2;
        [n1, n2, n3] = surfacenormals(im, gradscale, loggrad);
    
    elseif evnt.Character == '-' | evnt.Character == 31
        gradscale = gradscale/2;     
        [n1, n2, n3] = surfacenormals(im, gradscale, loggrad);
    
    elseif evnt.Character == 'l'
        if strcmp(loggrad, 'linear')
            loggrad = 'log';
            fprintf('Using log of gradients\n');
            
        elseif strcmp(loggrad, 'log')            
            loggrad = 'loglog';    
            fprintf('Using log of log of gradients\n');            
            
        elseif strcmp(loggrad, 'loglog')             
            loggrad = 'logloglog';    
            fprintf('Using log log log gradients\n');            
            
        elseif strcmp(loggrad, 'logloglog')             
            loggrad = 'linear';    
            fprintf('Using raw data gradients\n');                
            
        end
        [n1, n2, n3] = surfacenormals(im, gradscale, loggrad);
        
    end
    
    wbmcb(src,evnt); % update the display
end    

%-----------------------------------------------------------------------
function relief(x, y)
    
    % Clamp x and y to image limits
    x = max(0,x); x = min(cols,x);  
    y = max(0,y); y = min(rows,y);  
    
    % Convert to polar coordinates with respect to reference point
    xp = x - xo;
    yp = y - yo;
    
    radius = sqrt(xp.^2 + yp.^2);
    
    % Compute azimuth. We want 0 azimuth pointing up increasing
    % clockwise. Hence yp must be negaated becasuse +ve y points down in the
    % image, pi/2 must be subtracted to shift 0 from east to north, and the
    % overall result must be negated to make angles +ve clockwise (yuk).
    azimuth = -(atan2(-yp, xp) - pi/2);   
    
    % Convert radius to normalised coords with respect to image size a
    radius = radius/maxRadius;
    radius = min(radius,1);
    
    % Convert azimuth and elevation to a light direction.   Note that
    % the vector is constructed so that an azimuth of 0 points upwards and
    % increases clockwise.
    elevation = pi/2 - radius*2*pi/8;
    I = [cos(elevation)*sin(azimuth), cos(elevation)*cos(azimuth), sin(elevation)];    
    I = I./norm(I); % Ensure I is a unit vector     
    
    % Display light direction and current gradscale value
    set(texth, 'String', sprintf('Az: %d  El: %d  Gs: %.2f', ...
                                 round(azimuth/pi*180), round(elevation/pi*180), gradscale));
    
    % Generate Lambertian shading - dot product between surface normal and light
    % direction.  Note that the product with n2 is negated to account for the
    % image +ve y increasing downwards.
    shading = I(1)*n1 - I(2)*n2 + I(3)*n3; 

    % Remove -ve shading values (surfaces pointing away from light source)
    shading(shading < 0) = 0;
    
    % If an image has been supplied apply shading to it
    if ~isempty(rgbim)
        for n = 1:3
            shrgbim(:,:,n)  = rgbim(:,:,n).*shading;
        end
        set(imHandle,'CData', shrgbim);
    
    else % Just display shading image
        set(imHandle,'CData', shading);
    end
    
end 

%---------------------------------------------------------------------------
% Compute image/heightmap surface normals

function [n1, n2, n3] = surfacenormals(im, gradscale, loggrad)
    
    % Compute partial derivatives of z.
    % p = dz/dx, q = dz/dy

    [p,q] = gradient(im);  
    p = p*gradscale;
    q = q*gradscale;
    
    % If specified take logs of gradient 
    if strcmpi(loggrad, 'log')
        p = sign(p).*log1p(abs(p));
        q = sign(q).*log1p(abs(q));
    elseif strcmpi(loggrad, 'loglog')
        p = sign(p).*log1p(log1p(abs(p)));
        q = sign(q).*log1p(log1p(abs(q)));
    elseif strcmpi(loggrad, 'logloglog')
        p = sign(p).*log1p(log1p(log1p(abs(p))));
        q = sign(q).*log1p(log1p(log1p(abs(q))));
    end
    
    % Generate surface unit normal vectors. Components stored in n1, n2
    % and n3 
    mag = sqrt(1 + p.^2 + q.^2);
    n1 = -p./mag;
    n2 = -q./mag;
    n3 =  1./mag;
end

%---------------------------------------------------------------------------
% Generate coordinates of points around a circle with centre c, radius r

function [xd, yd] = circlexy(c, r)
    nsides = 32;
    a = [0:2*pi/nsides:2*pi];
    xd = r*cos(a) + c(1);
    yd = r*sin(a) + c(2);
end

%---------------------------------------------------------------------------
% Estimate initial gradient scale for a reasonable initial shading result
% Aim for a median gradient of around 0.25

function gradscale = initialgradscale
    mediantarget = 0.25;
    [p,q] = gradient(im);  
    pt = abs([p(:); q(:)]);
    pt(isnan(pt) | pt <eps) = [];
    
    gradscale = mediantarget/median(pt);
end

%---------------------------------------------------------------------------
end % of dynamicrelief
    
