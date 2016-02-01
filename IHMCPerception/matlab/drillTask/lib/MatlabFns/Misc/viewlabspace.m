% VIEWLABSPACE  Visualisation of L*a*b* space
%
% Usage:    viewlabspace(dL)
%
% Argument:   dL - Optional increment in lightness with each slice of L*a*b*
%                  space. Defaults to 5
%
% Function allows interactive viewing of a sequence of images corresponding to
% different slices of lightness in L*a*b* space.  Lightness varies from 0 to
% 100.  Initially a slice at a lightness of 50 is displayed.
% Pressing 'l' or arrow up/right will increase the lightness by dL.
% Pressing 'd' or arrow down/left will darken by dL.
% Press 'x' to exit.
%
% The CIELAB colour coordinates of the cursor position within the slice images
% is updated continuously.  This is useful for determining suitable controls
% points for the definition of colourmap paths through CIELAB space.
%
% See also: COLOURMAPPATH, ISOCOLOURMAPPATH, CMAP

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

% March 2013
% November 2013  Interactive CIELAB coordinate feedback from mouse position

function viewlabspace(dL, figNo)
    
    if ~exist('dL', 'var'), dL = 5; end
    if ~exist('figNo', 'var'), figNo = 100; end
    
    % Define some reference colours in rgb
    rgb = [1 0 0
           0 1 0
           0 0 1
           1 1 0
           0 1 1
           1 0 1
           1 .5 0
           .5 0 1
           0  .5 1
           0   1 .5
           1  0 .5];
    
    colours = {'red    '
               'green  '
               'blue   '
               'yellow '
               'cyan   '
               'magenta'
               'orange '
               'violet '
               'blue-cyan'
               'green-cyan'
               'red-magenta'};
    
    % ... and convert them to lab
    % Use D65 whitepoint to match typical monitors.
    wp = whitepoint('D65');
    labv = applycform(rgb, makecform('srgb2lab', 'AdaptedWhitePoint', wp));

    % Obtain cylindrical coordinates in lab space
    labradius = sqrt(labv(:,2).^2+labv(:,3).^2);
    labtheta = atan2(labv(:,3), labv(:,2));
    
    % Define a*b* grid for image
    scale = 2;
    [a, b] = meshgrid([-127:1/scale:127]);
    [rows,cols] = size(a);
    
    % Scale and offset lab coords to fit image coords
    labc = zeros(size(labv));
    labc(:,1) = round(labv(:,1));
    labc(:,2) = round(scale*labv(:,2) + cols/2);
    labc(:,3) = round(scale*labv(:,3) + rows/2);
    
    % Print out lab values
    labv = round(labv);
    fprintf('\nCoordinates of standard colours in L*a*b* space\n\n');
    for n = 1:length(labv)
        fprintf('%s  L%3d   a %4d  b %4d    angle %4.1f  radius %4d\n',...
                colours{n}, ...
                labv(n,1), labv(n,2), ...
                labv(n,3),  labtheta(n), round(labradius(n)));
    end
    
    fprintf('\n\n')
    
    % Generate axis tick values
    tickval = [-100 -50 0 50 100];
    tickcoords = scale*tickval + cols/2;
    ticklabels = {'-100'; '-50'; '0'; '50'; '100'};

    fig = figure(figNo);
    set(fig, 'WindowButtonMotionFcn', @labcoords)
    set(fig, 'KeyReleaseFcn', @keyreleasecb);
    texth = text(50, 50,'', 'color', [1 1 1]);  % Text area to display
                                                % current a and b values
    
    fprintf('Place cursor within figure\n');
    fprintf('Press ''l'' to lighten, ''d'' to darken, or use arrow keys\n');
    fprintf('''x'' to exit\n');

    L = 50;
    renderlabslice(L);
    
    
%------------------------------------------------------
% Window button move call back function
function labcoords(src, evnt)
    cp = get(gca,'CurrentPoint');
    x = cp(1,1); y = cp(1,2);

    aval = round((x-(cols/2))/scale);
    bval = round((y-(rows/2))/scale);

    radius = sqrt(aval.^2 + bval.^2);
    hue = atan2(bval, aval);
    
    set(texth, 'String', sprintf('a %d   b %d   radius %d angle %d',...
                                 aval, bval, round(radius), round(hue/pi*180)));
end

%-----------------------------------------------------------------------
% Key Release callback
% If '+' or up is pressed we move up in lighness
% '-' or down moves us down in lightness

function keyreleasecb(src,evnt)
    
    if evnt.Character == 'l' | evnt.Character == 30
        L = min(L + dL, 100);

    elseif evnt.Character == 'd' | evnt.Character == 31
        L = max(L - dL, 0);
        
    elseif evnt.Character == 's'  % Save slice as an image
        tmp = rgb;
        tmp(:,:,1) = flipud(tmp(:,:,1));
        tmp(:,:,2) = flipud(tmp(:,:,2));
        tmp(:,:,3) = flipud(tmp(:,:,3));
        % Draw a red cross at the achromatic point
        [r,c,~] = size(tmp);
        tmp = imageline(tmp, [c/2-5, r/2], [c/2+5, r/2], [255, 0 0]);
        tmp = imageline(tmp, [c/2, r/2-5], [c/2, r/2+5], [255, 0 0]);
        imwrite(tmp, sprintf('LAB_slice_L_%d.png',L));
        
    elseif evnt.Character == 'x'
        delete(figNo);
        return
        
    end

    renderlabslice(L);     % Render a slice at this new lightness level
    labcoords(src, evnt);  % Regenerate the cursor coordinates
end    

%--------------------------------------------------------

function renderlabslice(L)
    
% Build image in lab space
    lab = zeros(rows,cols,3);
    lab(:,:,1) = L;
    lab(:,:,2) = a;
    lab(:,:,3) = b;
    
    
    wp = whitepoint('D65');
    
    % Generate rgb values from lab
    rgb = applycform(lab, makecform('lab2srgb', 'AdaptedWhitePoint', wp));
    
    % Invert to reconstruct the lab values
    lab2 = applycform(rgb, makecform('srgb2lab', 'AdaptedWhitePoint', wp));
    
    % Where the reconstructed lab values differ from the specified values is
    % an indication that we have gone outside of the rgb gamut.  Apply a
    % mask to the rgb values accordingly
    mask = max(abs(lab-lab2),[],3);
    
    for n = 1:3
        rgb(:,:,n) = rgb(:,:,n).*(mask<2);  % tolerance of 1
    end
    
    figure(figNo), image(rgb), title(sprintf('Lightness %d', L));
    axis square,         axis xy
    % Recreate the text handle in the new image
    texth = text(50, 50,'', 'color', [1 1 1]);
    
    set(gca, 'xtick', tickcoords);
    set(gca, 'ytick', tickcoords);
    set(gca, 'xticklabel', ticklabels);
    set(gca, 'yticklabel', ticklabels);
    xlabel('a*'); ylabel('b*');    
    
    hold on, 
    plot(cols/2, rows/2, 'r+');   % Centre point for reference
    
    % Plot reference colour positions
    for n = 1:length(labc)
        plot(labc(n,2), labc(n,3), 'w+')
        text(labc(n,2), labc(n,3), ...
             sprintf('   %s\n  %d %d %d  ',colours{n},...
                     labv(n,1), labv(n,2), labv(n,3)),...
             'color', [1 1 1])
    end
    
    hold off
    
end

%--------------------------------------------------------
end % of viewlabspace