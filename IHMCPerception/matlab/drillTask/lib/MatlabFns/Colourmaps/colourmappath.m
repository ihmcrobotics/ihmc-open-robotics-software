% COLOURMAPPATH Plots the path of a colour map through colour space
%
% Usage: colourmappath(map, param_name, value, ....)
%
% Required argument:
%            map - The colourmap to be plotted
%
% Optional parameter-value pairs, default values in brackets:
%              'N' - The nmber of slices through the colourspace to plot (6).
%       'colspace' - String 'rgb' or 'lab' indicating the colourspace to plot
%                    ('lab'). 
%            'fig' - Optional figure number to use (new figure created).
%      'linewidth' - Width of map path line (2.5).
%       'dotspace' - Spacing between colour map entries where dots are plotted
%                    along the path (5).
%        'dotsize' - (15). Use 0 if you want no dots plotted.
%      'dotcolour' - ([0.8 0.8 0.8])
%       'fontsize' - (14).
%     'fontweight' - ('bold').
%
% The colour space is represented as a series of semi-transparent 2D slices
% which hopefully lets you visualise the colour space and the colour map path
% simultaneously.
%
% Note that for some reason repeated calls of this function to render a colour
% map path in RGB space seem to ultimately cause MATLAB to crash (under MATLAB
% 2013b on OSX).  Rendering paths in CIELAB space cause no problem.
%
% See also: CMAP, EQUALISECOLOURMAP, VIEWLABSPACE, SINERAMP, CIRCSINERAMP

% Copyright (c) 2013-2014 Peter Kovesi
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

% October 2013 - Original version
% October 2014 - Parameter options added

%function colourmappath(map, N, colspace, fig)  % Original argument list
function colourmappath(varargin)

    [map, N, colspace, fig, linewidth, ds, dotcolour, dotsize, fontsize, fontweight] ...
        = parseinputs(varargin{:});    
    
    mapref = map;

    %% RGB plot ---------------------------------------------------------
    if strcmpi(colspace, 'rgb')
        
        R = 255;
        delta = 2;  % Downsampling factor for red and green
        [x,y] = meshgrid(0:delta:R, 0:delta:R);
        [sze, ~] = size(x);
        im = zeros(sze, sze, 3, 'uint8');
        
        figure(fig), clf, axis([-1 sze+1 -1 sze+1 -1 sze+1])
        
        % Draw N red-green planes of increasing blue value through RGB space 
        for b = 0:R/(N-1):R
            for r = 0:delta:R
                for g = 0:delta:R
                    im(g/delta+1, r/delta+1, :) = [r; g; round(b)];
                end
            end
            
            h = warp(b*ones(sze,sze), im); hold on
            set(h, 'FaceAlpha', 0.9);
        end
    
        % Plot the colour map path through the RGB colour space.  Rescale map
        % to account for scaling to 255 and for downsampling (yuk)
        map(:,1:2) = map(:,1:2)*255/delta;
        map(:,3) = map(:,3)*255;
        line(map(:,1), map(:,2), map(:,3), 'linewidth', linewidth, 'color', [0 0 0]);
        hold on
        
        % Plot a dot for every 'dotspace' point along the colour map to indicate the
        % spacing of colours in the colour map
        if dotsize
            plot3(map(1:ds:end,1), map(1:ds:end,2), map(1:ds:end,3), '.', ...
                  'Color', dotcolour, 'MarkerSize', dotsize);
        end
        hold off

        % Generate axis tick values
        tickcoords = [0:50:250]/delta;
        ticklabels = {'0'; '50'; '100'; '150'; '200'; '250'};
        
        set(gca, 'xtick', tickcoords);
        set(gca, 'ytick', tickcoords);
        set(gca, 'xticklabel', ticklabels);
        set(gca, 'yticklabel', ticklabels);    
        
        xlabel('Red');
        ylabel('Green');
        zlabel('Blue');
    
        set(gca,'Fontsize', fontsize);
        set(gca,'Fontweight', fontweight);
        
        axis vis3d
        
    
    %% CIELAB plot ----------------------------------------------------
    elseif strcmpi(colspace, 'lab')
        
        if iscell(mapref)  % We have a cell array of colour maps
            figure(fig), clf,     axis([-110 110 -110 110  0 100])
            for n = 1:length(mapref)
                lab = rgb2lab(mapref{n});
                line(lab(:,2), lab(:,3), lab(:,1), 'linewidth', linewidth, 'color', [0 0 0])
                hold on
                if dotsize
                    plot3(lab(1:ds:end,2), lab(1:ds:end,3), lab(1:ds:end,1), '.', ...
                          'Color', dotcolour, 'MarkerSize', dotsize);
                end
            end
        
        elseif  ~isempty(map) % Single colour map supplied
            lab = rgb2lab(mapref);
            
            figure(fig), clf,     axis([-110 110 -110 110  0 100])
            line(lab(:,2), lab(:,3), lab(:,1), 'linewidth', linewidth, 'color', [0 0 0])
            hold on
            if dotsize
                plot3(lab(1:ds:end,2), lab(1:ds:end,3), lab(1:ds:end,1), '.', ...
                  'Color', dotcolour, 'MarkerSize', dotsize);
            end
            %    line([0 0], [0 0], [0 100])  % Line up a = 0, b = 0 axis
        end
        
        %% Generate Lab image slices
    
        % Define a*b* grid for image
        scale = 1;
        [a, b] = meshgrid([-127:1/scale:127]);
        [rows,cols] = size(a);
    
        %  Generate N equi-spaced lightness levels between 5 and 95.
        for L = 5:90/(N-1):95  % For each lightness level...
            
            % Build image in lab space
            lab = zeros(rows,cols,3);    
            lab(:,:,1) = L;
            lab(:,:,2) = a;
            lab(:,:,3) = b;
            
            % Generate rgb values from lab
            rgb = applycform(lab, makecform('lab2srgb'));
            
            % Invert to reconstruct the lab values
            lab2 = applycform(rgb, makecform('srgb2lab'));
            
            % Where the reconstructed lab values differ from the specified values is
            % an indication that we have gone outside of the rgb gamut.  Apply a
            % mask to the rgb values accordingly
            mask = max(abs(lab-lab2),[],3);
            
            for n = 1:3
                rgb(:,:,n) = rgb(:,:,n).*(mask<2);  % tolerance of 2
            end
            
            [x,y,z,cdata] = labsurface(lab, rgb, L, 100);
            h = surface(x,y,z,cdata);  shading interp; hold on
            set(h, 'FaceAlpha', 0.9);
            
        end % for each lightness level
        
        % Generate axis tick values
        tickval = [-100 -50 0 50 100];
        tickcoords = tickval; %scale*tickval + cols/2;
        ticklabels = {'-100'; '-50'; '0'; '50'; '100'};
        
        set(gca, 'xtick', tickcoords);
        set(gca, 'ytick', tickcoords);
        set(gca, 'xticklabel', ticklabels);
        set(gca, 'yticklabel', ticklabels);    
        
        ztickval = [0 20 40 60 80 100];
        zticklabels = {'0' '20' ' 40' '60' '80' '100'};
        set(gca, 'ztick', ztickval);
        set(gca, 'zticklabel', zticklabels);            

        set(gca,'Fontsize', fontsize);
        set(gca,'Fontweight', fontweight);
        
        % Label axes.  Note option for manual placement for a and b
        manual = 0;
        if ~manual
            xlabel('a', 'Fontsize', fontsize, 'FontWeight', fontweight);
            ylabel('b', 'Fontsize', fontsize, 'FontWeight', fontweight);
        else
            text(0, -170, 0, 'a', 'Fontsize', fontsize, 'FontWeight', ...
                 fontweight);        
            text(155, 0, 0, 'b', 'Fontsize', fontsize, 'FontWeight', ...
                 fontweight);
        end
        zlabel('L', 'Fontsize', fontsize, 'FontWeight', fontweight);
        
        axis vis3d
        
        grid on, box on
        view(64, 28)
        hold off
        
    else
        error('colspace must be rgb or lab')
    end
    
%-------------------------------------------------------------------
% LABSURFACE
%
% Idea to generate lab surfaces.  Generate lab surface image, Sample a
% parametric grid of x, y, z and colour values.  Texturemap the colour values
% onto the surface.
%
% Find first and last rows of image containing valid values.  Interpolate y
% over this range.  For each value of y find first and last valid x values.
% Interpolate x over this range sampling colour values as one goes.

function [x, y, z, cdata] = labsurface(lab, rgb, zval, N)
    
    x = zeros(N,N);
    y = zeros(N,N);
    z = zeros(N,N);
    cdata = zeros(N,N,3);
    
    % Determine top and bottom edges of non-zero section of image
    gim = sum(rgb, 3);  % Sum over all colour channels
    rowsum = sum(gim, 2); % Sum over rows
    
    ind = find(rowsum);
    top = ind(1);
    bottom = ind(end);
    rowvals = round(top + (0:N-1)/(N-1)*(bottom-top));

    rowind = 1;
    for row = rowvals
       % Find first and last non-zero elements in this row
       ind = find(gim(row,:));
       left = ind(1);
       right = ind(end);
       colvals = round(left + (0:N-1)/(N-1)*(right-left));

       % Fill matrices
       colind = 1;
       for col = colvals
           x(rowind,colind) = lab(row,col,2);
           y(rowind,colind) = lab(row,col,3);          
           
           z(rowind,colind) = zval;
           cdata(rowind, colind, :) = rgb(row, col, :);
           
           colind = colind+1;
       end
        
       rowind = rowind+1;
    end
    
%-----------------------------------------------------------------------
% Function to parse the input arguments and set defaults

function [map, N, colspace, fig, linewidth, dotspace, dotcolour, dotsize, ...
          fontsize, fontweight] = parseinputs(varargin)
    
    p = inputParser;
%    numericorcell = @(x) isnumeric(x) || iscell(x);
    
%    addRequired(p, 'map', @numericorcell); 
    addRequired(p, 'map'); 
    
    % Optional parameter-value pairs and their defaults    
    addParameter(p, 'N',     6, @isnumeric);  
    addParameter(p, 'colspace',  'LAB', @ischar);  
    addParameter(p, 'fig', -1, @isnumeric);     
    addParameter(p, 'linewidth', 2.5, @isnumeric);        
    addParameter(p, 'dotspace', 5, @isnumeric);
    addParameter(p, 'dotsize', 15, @isnumeric);
    addParameter(p, 'dotcolour', [0.8 0.8 0.8], @isnumeric);
    addParameter(p, 'fontsize', 14, @isnumeric);
    addParameter(p, 'fontweight', 'bold', @ischar);
    
    parse(p, varargin{:});
    
    map        = p.Results.map;
    N          = p.Results.N;
    colspace   = p.Results.colspace;
    dotspace   = p.Results.dotspace;
    dotcolour  = p.Results.dotcolour;
    dotsize    = p.Results.dotsize;
    linewidth  = p.Results.linewidth;
    fontsize   = p.Results.fontsize;
    fontweight = p.Results.fontweight;
    fig        = p.Results.fig; 
    if fig < 0,  fig = figure; end    
    
    