% FINDIMAGES - invokes image dialog box for multiple image loading
%
% Usage:  [im, filename] = findimages
%
% Returns:
%             im - Cell array of images
%       filename - Cell arrauy of filenames of images
%
% See Also: FINDIMAGE

% Peter Kovesi  
% Centre for Exploration Targeting
% The University of Western Australia
% peter.kovesi at uwa edu au
%
% March 2013

function [im, filename] = findimages
    
    [filename, pathname] = uigetfile({'*.*'}, ...
                                     'Select images' ,'multiselect','on');
    if ~iscell(filename)  % Assume canceled
        im = {};
        filename = {};
        return;
    end
    
    for n = 1:length(filename)
        filename{n} = [pathname filename{n}];
        im{n} = imread(filename{n});    
    end
    
    
     
     