% VIEWLABSPACE2  Visualisation of L*a*b* space
%
% Usage:    viewlabspace2(dtheta)
%
% Argument:   dtheta - Optional specification of increment in angle of plane
%                      through L*a*b* space. Defaults to pi/30
%
% Function allows interactive viewing of a sequence of images corresponding to
% different vertical slices in L*a*b* space. 
% Initially a vertical slice in the a* direction is displayed.
% Pressing  arrow up/right will rotate the plane +dtheta
% Pressing  arrow down/left will rotate the plane by -dtheta
% Press 'x' to exit.
%
% See also:  VIEWLABSPACE, CMAP

% To Do:  Should be integrated with VIEWLABSPACE so that we get both views

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

function viewlabspace2(dtheta)
    
    if ~exist('dtheta', 'var'), dtheta = pi/30; end
    
    % Define some reference colours in rgb
    rgb = [1 0 0
           0 1 0
           0 0 1
           1 1 0
           0 1 1
           1 0 1];
    
    colours = {'red    '
               'green  '
               'blue   '
               'yellow '
               'cyan   '
               'magenta'};
    
    % ... and convert them to lab
    labv = applycform(rgb, makecform('srgb2lab'));

    % Obtain cylindrical coordinates in lab space
    labradius = sqrt(labv(:,2).^2+labv(:,3).^2);
    labtheta = atan2(labv(:,3), labv(:,2));
    
    % Define lightness - radius grid for image
    scale = 2;
    [rad, L] = meshgrid([-140:1/scale:140], [0:1/scale:100]);
    [rows,cols] = size(rad);
    
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

    
    ytickval = [0 20 40 60 80 100];
    ytickcoords = scale*ytickval;
    yticklabels = {'0'; '20'; '40'; '60'; '80'; '100'};    
    
    
    fprintf('Place cursor within figure\n');
    fprintf('Use arrow keys to rotate the plane through L*a*b* space\n');
    fprintf('''x'' to exit\n');
    
    ch = 'l';
    theta = 0;
    while ch ~= 'x'

        % Build image in lab space
        lab = zeros(rows,cols,3);
        lab(:,:,1) = L;
        lab(:,:,2) = rad.*cos(theta);
        lab(:,:,3) = rad.*sin(theta);

        % Generate rgb values from lab
        rgb = applycform(lab, makecform('lab2srgb'));
        
        % Invert to reconstruct the lab values
        lab2 = applycform(rgb, makecform('srgb2lab'));

        % Where the reconstructed lab values differ from the specified values is
        % an indication that we have gone outside of the rgb gamut.  Apply a
        % mask to the rgb values accordingly
        mask = max(abs(lab-lab2),[],3);
        
        for n = 1:3
            rgb(:,:,n) = rgb(:,:,n).*(mask<2);  % tolerance of 1
        end

        figure(2), image(rgb), title(sprintf('Angle %d', round(theta/pi*180)));
        axis square, axis xy
        
        set(gca, 'xtick', tickcoords);
        set(gca, 'ytick', ytickcoords);
        set(gca, 'xticklabel', ticklabels);
        set(gca, 'yticklabel', yticklabels);
        xlabel('a*b* radius'); ylabel('L*');    
        impixelinfo

        hold on, 
        plot(cols/2, rows/2, 'r+');   % Centre point for reference
%{        
        % Plot reference colour positions
        for n = 1:length(labc)
            plot(labc(n,2), labc(n,3), 'w+')
            text(labc(n,2), labc(n,3), ...
                 sprintf('   %s\n  %d %d %d  ',colours{n},...
                         labv(n,1), labv(n,2), labv(n,3)),...
                 'color', [1 1 1])
        end
%}        
        hold off

        % Handle keypresses within the figure
        pause        
        ch = lower(get(gcf,'CurrentCharacter'));
        if  ch == 29 || ch == 30
            theta = mod(theta + dtheta, 2*pi);
        elseif ch == 28 || ch == 31
            theta = mod(theta - dtheta, 2*pi);
        end
        
    end
    
    
