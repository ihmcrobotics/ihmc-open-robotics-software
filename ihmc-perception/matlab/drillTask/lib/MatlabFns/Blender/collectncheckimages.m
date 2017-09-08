% COLLECTNCHECKIMAGES Collects and checks images prior to blending
%
% Usage: [im, nImages, fname, pathname] = collectncheckimages(im)
%
% Used by image blending functions
%
%  Argument:   im - Cell arry of images.  If omitted a dialog box is
%                   presented so that images can be selected interactively.
%
%  Returns:    im - Cell array of images of consistent size and colour class.
%                   If input images are of different sizes the images are
%                   trimmed to the size of the smallest image. If some
%                   images are colour any greyscale images are converted to
%                   colour by copying the image to the R G and B channels.
%         nImages - Number of images in the cell array.
%           fname - A cell array of the filenames of the images if they were
%                   interactively selected via a dialog box.
%        pathname - The file path if the images were selected via a dialog
%                   box.
%
%  If the image selection via a dialog box is cancelled all values are returned
%  as empty cell arrays or 0
%
% See also:  LINIMIX, BILINIMIX, TERNARYMIX, CLIQUEMIX, CYCLEMIX

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
% May   2012  - Original version
% March 2014  - Provison for 'im' being a 2D cell array
% August 2014 - Fix to avoid normalisation of colour images

function [im, nImages, fname, pathname] = collectncheckimages(im)
    
    if ~exist('im', 'var') | isempty(im)  % We need to select several images
        
        fprintf('Select several images...\n');
        [fname, pathname] = uigetfile({'*.png';'*.tiff';'*.jpg'},...
                                      'MultiSelect','on');

        if isnumeric(fname) && fname == 0  % Selection was cancelled
            im = {};
            nImages = 0;
            fname = {};
            pathname = {};
            return
        end
        
        % If single file has been selected place its name in a single element
        % cell array so that it is consistent with the rest of the code.
        if isa(fname, 'char')
            tmp = fname;
            fname = {};
            fname{1} = tmp;
        end
        
        nImages = length(fname);
        im = cell(1,nImages);
        
        for n = 1:nImages
            im{n} = double(imread([pathname fname{n}]));
        end
        
    elseif iscell(im)  % A cell array of images to blend has been supplied. 

        % Give each image a nominal name
        nImages = prod(size(im));   % (im might be a 2D cell array of images)
        fname = cell(1,nImages);
        for n = 1:nImages
            fname{n} = sprintf('%d', n);
        end
        pathname = '';
    end    
    
    if nImages == 1  % See if we have a multi-channel image
        [rows, cols, chan] = size(im{1});
        
        assert(chan > 1, ...
         'Input must be a cell arrayof images, or a multi-channel image');
        
        % Copy the channels out into a cell array of separate images and
        % generate nominal names.
        nImages = chan;
        tmp = im{1}; 
        im = cell(1,nImages);
        fname = cell(1,nImages);
        for n = 1:nImages
            im{n} = tmp(:,:,n);
            fname{n} = sprintf('Band %d', n);
        end
        pathname = '';
    end        

    % Check sizes of images
    rows = zeros(nImages,1); cols = zeros(nImages,1); chan = zeros(nImages,1);
    for n = 1:nImages
        [rows(n) cols(n), chan(n)] = size(im{n});
    end

    % If necessary trim all images down to the size of the smallest image and
    % give a warning.  Given that the imput images need to be registered this
    % should perhaps be classed as an error.
    if ~all(rows == rows(1)) || ~all(cols == cols(1)) 
        fprintf('Not all images are the same size\n');
        fprintf('Trimming images to the match the smallest\n');
        
        minrows = min(rows); mincols = min(cols);
        for n = 1:nImages
            im{n} = im{n}(1:minrows, 1:mincols, :);    
        end
    end

    % If one image is RGB ensure every image is stored as rgb, even if it
    % is greyscale. 
    if ~(all(chan == 3) || all(chan == 1))
        for n = 1:nImages
            if chan(n) == 1
                %  Replicate image in R, G and B channels
                im{n} = repmat(im{n},[1 1 3]);  
            end
        end
    end
        
    % Finally normalise images as needed and ensure class is double.  Note that
    % if an input image had three channels we assume it is already normalised
    % 0-255 (if uint8) or 0-1 (if double) hence we only normalise a double image
    % if it originally only had one channel.
    for n = 1:nImages
        if strcmp(class(im{n}),'uint8')
            im{n} = double(im{n})/255;
        elseif chan(n) == 1
            im{n} = normalise(double(im{n}));    
        end
    end