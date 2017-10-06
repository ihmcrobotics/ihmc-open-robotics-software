% FILTERREGIONPROPERTIES  Filters regions on their property's values
% 
% Usage: bw = filterregionproperties(bw, {property, fn, value}, { ... } )
%
% Arguments:  
%                     bw  - Binary image
%  {property, fn, value}  - 3-element cell array consisting of:
%                           property - String matching one of the properties
%                                      computed by REGIONPROPS.
%                           fn       - Handle to a function that will compare
%                                      the region property to a
%                                      value. Typically @lt or @gt.
%                           value    - The value that the region property is
%                                      compared against
%
% You can specify multiple {property, fn, value} cell arrays in the argument
% list.  Blobs/regions in the binary image that satisfy all the specified
% constaints are retained in the image.
%
% Examples:
%
% Retain blobs that have an area greater than 50 pixels
%  >>  bw = filterregionproperties(bw, {'Area', @gt, 50});
%
% Retain blobs with an area less than 20 and having a major axis orientation
% that is greater than 0
%  >>  bw = filterregionproperties(bw, {'Area', @lt, 20}, {'Orientation', @gt, 0});
%
% See also: REGIONPROPS

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

% PK May 2013

function bw = filterregionproperties(bw,  varargin)  

    varargin = varargin(:);
    
    % Form separate cell arrays of the  properties, functions and values, 
    % and perform basic check of datatypes
    nOps = length(varargin);
    property = cell(nOps,1);
    fn       = cell(nOps,1);
    value    = cell(nOps,1);
        
    for m = 1:nOps
        if length(varargin{m}) ~= 3
            error('Property, function and value must be a 3 element cell array');
        end

        property{m} = varargin{m}{1};
        fn{m} = varargin{m}{2};
        value{m} = varargin{m}{3};        

        if ~ischar(property{m})
            error('Property must be a string');
        end
        if ~strcmp(class(fn{m}), 'function_handle')
            error('Invalid function handle');
        end
    end
    
    [L, N] = bwlabel(bw);         % Label image
    s = regionprops(L, property); % Get region properties

    % Form a table indicating which labeled region should be zeroed out on
    % the basis that its properties do not satisfy the
    % property-function-value requirements
    table = ones(N,1);
    for n = 1:N
        for m = 1:nOps
            if ~feval(fn{m}, s(n).(property{m}), value{m});
                table(n) = 0;
            end
        end
    end

    % Step through the binary image applying the table to zero out the
    % appropriate blobs
    for n = 1:numel(bw)
        if bw(n)
            bw(n) = table(L(n));
        end
    end
    
    