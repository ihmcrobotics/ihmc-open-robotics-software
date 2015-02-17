% LABMAP - Generates a colourmap based on L*a*b* space
%
% This function can generate a wide range of colourmaps but it can be a bit
% difficult to drive...
%
% The idea: Create a spiral path within in L*a*b* space to use as a
% colourmap. L*a*b* space is designed to be perceptually uniform so, in
% principle, it should be a good space in which to design colourmaps.
%
% L*a*b* space is treated (a bit inappropriately) as a cylindrical colourspace.
% The spiral path is created by specifying linear ramps in: the angular value
% around the origin of the a*b* plane; the Lightness variation; and the
% saturation (the radius out from the centre of the a*b* plane).
%
% As an alternative an option is available to use a simple straight line
% interpolation from the first colour, through the colourspace, to the final
% colour.  One is much less likely to go outside of the rgb gamut but the
% variety of colourmaps will be limited
%
% Usage:  map = labmap(theta, L, saturation, N, linear, debug)
%
% Arguments:
%               theta - 2-vector specifyinhg start and end angles in the a*b*
%                       plane over which to define the colourmap (radians).
%                       These angles specify a +ve or -ve ramp of values over
%                       which opponent colour values vary.  If you want
%                       values to straddle the origin use theta values > 2pi
%                       or < 0 as needed. 
%                   L - 2-vector specifying the lightness variation from
%                       start to end over the  colourmap.   Values are in the
%                       range 0-100.  (You normally want a lightness
%                       variation over the colourmap)
%          saturation - 2-vector specifying the saturation variation from
%                       start to end over the  colourmap.  This specifies the
%                       radius out from the centre of the a*b* plane where
%                       the colours are defined. Values are in the range 0-127.  
%                   N - Number of elements in the colourmap. Default = 256.
%              linear - Flag 0/1.  If this flag is set a simple straight line
%                       interpolation, from the first to last colour, through
%                       the colourspace is used.  Default value is 0.
%               debug - Optional flag 0/1.  If debug is set a plot of the
%                       colourmap is displayed along with a diagnostic plot
%                       of the specified L*a*b* values is generated along
%                       with the values actually achieved.  These are usually
%                       quite different due to gamut limitations.  However,
%                       as long as the lightness varies in a near linear
%                       fashion the colourmap is probably ok.
%
% theta, L and saturation can be specified as single values in which case
% they are assumed to be specifying a 'ramp' of constant value.
%
% The colourmap is generated from a* and b* values that form a spiral about
% the point (0, 0).  a* = saturation*cos(theta) and b* = saturation*sin(theta)
% while Lightness varies along the specified ramp.
% a* +ve indicates magenta, a* -ve indicates cyan 
% b* +ve indicates yellow, b* -ve indicates blue
%
% Changing lightness values can change things considerably and you will need
% some experimentation to get the colours you want.  It is often useful to try
% reversing the lightness ramp on your coloumap to see what it does.  Note also
% it is quite possible (quite common) to end up specifying L*a*b* values that
% are outside the gamut of RGB space.  Run the function with the debug flag
% set to monitor this.
%
% A weakness of this simple cylindrical coordinate approach used here is that it
% generates colours that are out of gamut far too readily.  To do: A better approach
% might be to show the allowable colours for a set of lightness values, allow
% the user to specify 3 colours, and then fit some kind of spline through these
% colours in lab space.
%
% L*a*b* space is more perceptually uniform than, say, HSV space.  HSV and other
% rainbow-like colourmaps can be quite problematic, especially around yellow
% because lightness varies in a non-linear way along the colourmap. Personally I
% find you can generate some rather nice colourmaps with LABMAP.
% 
%
% Coordinates of standard colours in L*a*b* space and in cylindrical coordinates
% 
% red      L 54   a   81  b   70    theta  0.7  radius  107
% green    L 88   a  -79  b   81    theta  2.3  radius  113
% blue     L 30   a   68  b -112    theta -1.0  radius  131
% yellow   L 98   a  -16  b   93    theta  1.7  radius   95
% cyan     L 91   a  -51  b  -15    theta -2.9  radius   53
% magenta  L 60   a   94  b  -61    theta -0.6  radius  111
%
% Example colourmaps:
%
% labmap([pi     pi/2],   [20 100], [60 127]);  % Dark green to yellow colourmap
% labmap([3*pi/2 pi/3],   [10 100], [60 127]);  % Blue to yellow
% labmap([3*pi/2 9*pi/4], [10 60],  [60 127]);  % Blue to red
% lapmap( 0, [0 100], 0);                       % Lightness greyscale
%
% See also: VIEWLABSPACE, HSVMAP, GRAYMAP, HSV, GRAY, RANDMAP, BILATERALMAP

% Copyright (c) 2012-2013 Peter Kovesi
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

% March 2012 
% March 2013  Allow theta, lightness and saturation to vary as a ramps. Allow
%             cylindrical and linear interpolation across the colour space

function map = labmap(theta, L, sat, N, linear, debug)
    
    if ~exist('theta', 'var'), theta = [0 2*pi]; end
    if ~exist('L', 'var'),         L = 60;       end    
    if ~exist('sat', 'var'),     sat = 127;      end    
    if ~exist('N', 'var'),         N = 256;      end
    if ~exist('linear', 'var'), linear = 0;      end
    if ~exist('debug', 'var'), debug = 0;        end

    if length(theta) == 1, theta = [theta theta]; end
    if length(L)     == 1,     L = [L L];         end
    if length(sat)   == 1,   sat = [sat sat];     end
    
    if ~linear % Use cylindrical interpolation
               % Generate linear ramps in theta, lightness and saturation
        thetar = [0:N-1]'/(N-1) * (theta(2)-theta(1)) + theta(1);
        Lr     = [0:N-1]'/(N-1) * (L(2)-L(1)) + L(1);
        satr   = [0:N-1]'/(N-1) * (sat(2)-sat(1)) + sat(1);
        
        lab = [Lr satr.*cos(thetar) satr.*sin(thetar)];
        map = applycform(lab, makecform('lab2srgb'));
        
    else  % Interpolate a straight line between start and end colours
        c1 = [L(1) sat(1)*cos(theta(1)) sat(1)*sin(theta(1))];
        c2 = [L(2) sat(2)*cos(theta(2)) sat(2)*sin(theta(2))];
        dc = c2-c1;
        
        lab = [[0:N-1]'/(N-1).*dc(:,1)+c1(:,1)  [0:N-1]'/(N-1).*dc(:,2)+c1(:,2)...
               [0:N-1]'/(N-1).*dc(:,3)+c1(:,3)];
        map = applycform(lab, makecform('lab2srgb'));
    end
    
    if debug
        % Display colourmap 
        ramp = repmat(0:0.5:255, 100, 1);
        show(ramp,1), colormap(map);
        
        % Test 'integrity' of colourmap. Convert rgb back to lab.  If this is
        % significantly different from the original lab map then we have gone
        % outside the rgb gamut
        labmap = applycform(map, makecform('srgb2lab'));    
        
        diff = lab-labmap;
        R = 1:N;
        figure(2), plot(R,lab(:,1),'k-',R,lab(:,2),'r-',R,lab(:,3),'b-',...
                        R,labmap(:,1),'k--',R,labmap(:,2),'r--',R,labmap(:,3),'b--')
        
        legend('Specified L', 'Specified a*', 'Specified b*',...
               'Achieved L', 'Achieved a*', 'Achieved b*')
        title('Specified and achieved L*a*b* values in colourmap')
        
        % Crude threshold on adherance to specified lab values
        if max(diff(:)) > 10
            warning(sprintf(['Colormap is probably out of gamut. \nMaximum difference' ...
                             ' between desired and achieved lab values is %d'], ...
                            round(max(diff(:)))));
        end
    end
