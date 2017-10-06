% CMAP Library of perceptually uniform colour maps
%
% Usage:  1:  [map, name, desc] = cmap(I, param_name, value ...)
%         2:  cmap
%         3:  cmap(str)
%
% Arguments for Usage 1:
%
%             I - A string label indicating the colour map to be generated or a
%                 string specifying a colour map name or attribute to search
%                 for.  Type 'cmap' with no arguments to get a full list of
%                 possible colour maps and their corresponding labels.
%
%   labels:  'L1' - 'L15'  for linear maps
%            'D1' - 'D12'  for diverging maps
%            'C1' - 'C9'   for cyclic maps
%            'R1' - 'R2'   for rainbow maps
%            'I1' - 'I3'   for isoluminant maps
%
%  Some colour maps have alternate labels for convenience and readability.
%  >> map = cmap('L1')  or map = cmap('grey')  will produce a linear grey map.
%  >> cmap  % With no arguments lists all colour maps and labels.
%
%  Possible param_name - value options:
%
%     'chromaK' - The scaling to apply to the chroma values of the colour map,
%                 0 - 1.  The default is 1 giving a fully saturated colour map
%                 as designed. However, depending on your application you may
%                 want a colour map with reduced chroma/saturation values.
%                 You can use values greater than 1 however gamut clipping is
%                 likely to occur giving rise to artifacts in the colour map. 
%           'N' - Number of values in the colour map. Defaults to 256.
%       'shift' - Fraction of the colour map length N that the colour map is
%                 to be cyclically rotated, may be negative.  (Should only be
%                 applied to cyclic colour maps!). Defaults to 0.
%     'reverse' - If set to 1 reverses the colour map. Defaults to 0.
% 'diagnostics' - If set to 1 displays various diagnostic plots. Note the
%                 diagnostic plots will be for the map _before_ any cyclic
%                 shifting or reversing is applied. Defaults to 0.
%
% The parameter name strings can be abbreviated to their first letter.
%
% Returns:
%           map - Nx3 rgb colour map
%          name - A string giving a nominal name for the colour map
%          desc - A string giving a brief description of the colour map
%
%
% Usage 2 and 3:  cmap(str)
%
% Given the large number of colour maps that this function can create this usage
% option provides some help by listing the numbers of all the colour maps with
% names containing the string 'str'.  Typically this is used to search for
% colour maps having a specified attribute: 'linear', 'diverging', 'rainbow',
% 'cyclic', or 'isoluminant' etc.  If 'str' is omitted all colour maps are
% listed.  
%
%  >> cmap                % lists all colour maps
%  >> cmap('diverging')   % lists all diverging colour maps
%
% Note the listing of colour maps can be a bit slow because each colour map has to
% be created in order to determine its full name.
%
%
% Colour Map naming convention:
%
%                    linear_kryw_5-100_c67_n256
%                      /      /    |    \    \
%  Colour Map attribute(s)   /     |     \   Number of colour map entries
%                           /      |      \
%     String indicating nominal    |      Mean chroma of colour map
%     hue sequence.                |
%                              Range of lightness values
%
% In addition, the name of the colour map may have cyclic shift information
% appended to it, it may also have a flag indicating it is reversed. 
%                                              
%              cyclic_wrwbw_90-40_c42_n256_s25_r
%                                          /    \
%                                         /   Indicates that the map is reversed.
%                                        / 
%                  Percentage of colour map length
%                  that the map has been rotated by.
%
% * Attributes may be: linear, diverging, cyclic, rainbow, or isoluminant.  A
%   colour map may have more than one attribute. For example, diverging-linear or
%   cyclic-isoluminant.
%
% * Lightness values can range from 0 to 100. For linear colour maps the two
%   lightness values indicate the first and last lightness values in the
%   map. For diverging colour maps the second value indicates the lightness value
%   of the centre point of the colour map (unless it is a diverging-linear
%   colour map). For cyclic and rainbow colour maps the two values indicate the
%   minimum and maximum lightness values. Isoluminant colour maps have only
%   one lightness value. 
%
% * The string of characters indicating the nominal hue sequence uses the following code
%      r - red      g - green      b - blue
%      c - cyan     m - magenta    y - yellow
%      o - orange   v - violet 
%      k - black    w - white      j - grey
%
%   ('j' rhymes with grey). Thus a 'heat' style colour map would be indicated by
%   the string 'kryw'. If the colour map is predominantly one colour then the
%   full name of that colour may be used. Note these codes are mainly used to
%   indicate the hues of the colour map independent of the lightness/darkness and
%   saturation of the colours.
% 
% * Mean chroma/saturation is an indication of vividness of the colour map. A
%   value of 0 corresponds to a greyscale. A value of 50 or more will indicate a
%   vivid colour map.
%
%
% See also: EQUALISECOLOURMAP, VIEWLABSPACE, SINERAMP, CIRCLESINERAMP, COLOURMAPPATH

% Adding your own colour maps is straightforward.
%
% 1) Colour maps are almost invariably defined via a spline path through CIELAB
%    colourspace.  Use VIEWLABSPACE to work out the positions of the spline
%    control points in CIELAB space to achieve the colour map path you desire.
%    These are stored in an array 'colpts' with columns corresponding to L a and
%    b.  If desired the path can be specified in terms of RGB space by setting
%    'colourspace' to 'RGB'.  See the ternary colour maps as an example.  Note
%    the case expression for the colour map label must be upper case.
%
% 2) Set 'splineorder' to 2 for a linear spline segments. Use 3 for a quadratic
%    b-spline.
%
% 3) If the colour map path has lightness gradient reversals set 'sigma' to a
%    value of around 5 to 7 to smooth the gradient reversal.
%
% 4) If the colour map is of very low lightness contrast, or isoluminant, set
%    the lightness, a and b colour difference weight vector W to [1 1 1].
%    See EQUALISECOLOURMAP for more details
%
% 5) Set the attribute and hue sequence strings ('attributeStr' and 'hueStr')
%    appropriately so that a colour map name can be generated.  Note that if you
%    are constructing a cyclic colour map it is important that 'attributeStr'
%    contains the word 'cyclic'.  This ensures that a periodic b-spline is used
%    and also ensures that smoothing is applied in a cyclic manner.  Setting the
%    description string is optional.
%
% 6) Run CMAP specifying the number of your new colour map with the diagnostics
%    flag set to one.  Various plots are generated allowing you to check the
%    perceptual uniformity, colour map path, and any gamut clipping of your
%    colour map.

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

% December  2013  Original version
% March     2014  Various enhancements
% August    2014  Catalogue listing
% September 2014  Provision for specifying paths in RGB, cyclic shifting and
%                 reversing 
% October   2014  Renamed to CMAP (from LABMAPLIB) and all the failed
%                 experimental maps cleaned out and retained maps renumbered
%                 in a more coherent way

function [map, name, desc] = cmap(varargin)

    [I, N, chromaK, shift, reverse, diagnostics]  = parseinputs(varargin{:});    
    
    % Default parameters for colour map construction.  
    % Individual colour map specifications may override some of these.
    colourspace = 'LAB'; % Control points specified in CIELAB space
    sigma = 0;           % Default smoothing for colour map lightness equalisation.
    splineorder = 3;     % Default order of b-spline colour map curve.
    formula = 'CIE76';   % Default colour difference formula.
    W = [1 0 0];         % Colour difference weighting for Lightness,
                         % chroma and hue (default is to only use Lightness)
    desc = '';
    name = '';
    attributeStr = '';
    hueStr = '';
    
    switch I   % Note all case expressions must be upper case.
    
    %-----------------------------------------------------------------------------        
    %% Linear series
        
     case {'L1', 'GREY'}  % Grey 0 - 100 
      desc = 'Grey scale'; 
      attributeStr = 'linear';
      hueStr = 'grey';
      colpts = [  0 0 0      
                  100 0 0];
      splineorder = 2;
      
     case {'L2', 'REDUCEDGREY'} %  Grey 10 - 95
      desc = ['Grey scale with slightly reduced contrast to '...
              'avoid display saturation problems'];
      attributeStr = 'linear';
      hueStr = 'grey';      
      colpts = [10 0 0
                95 0 0];
      splineorder = 2;
      
     case {'L3', 'HEAT', 'HEATWHITE'}
      desc = 'Black-Red-Yellow-White heat colour map';
      attributeStr = 'linear';
      hueStr = 'kryw';

      colpts = [5 0 0
                15 37 21
                25 49 37
                35 60 50
                45 72 60
                55 80 70
                65 56 73
                75 31 78
                85  9 84
                100 0 0 ];
 
     case {'L4', 'HEATYELLOW'}
      desc = 'Black-Red-Yellow heat colour map';
      attributeStr = 'linear';
      hueStr = 'kry';
      colpts = [5 0 0
                15 37 21
                25 49 37
                35 60 50
                45 72 60
                55 80 70
                65 56 73
                75 31 78
                85  9 84
                98 -16 93];
      
     case 'L5'
      desc = 'Colour Map along the green edge of CIELAB space';
      attributeStr = 'linear';
      hueStr = 'green';
      colpts = [ 5 -9  5
                 15 -23 20
                 25 -31 31
                 35 -39 39
                 45 -47 47
                 55 -55 55
                 65 -63 63
                 75 -71 71
                 85 -79 79
                 95 -38 90]; 
      
     case 'L6'
      desc = 'Blue shades running vertically up the blue edge of CIELAB space';
      attributeStr = 'linear';
      hueStr = 'blue';
      colpts = [ 5 30 -52
                 15 49 -80
                 25 64 -105
                 35 52 -103
                 45 26 -87
                 55  6 -72
                 65 -12 -56
                 75 -29 -40
                 85 -44 -24
                 95 -31 -9]; 
      
     case 'L7'
      desc = 'Blue-Pink-Light Pink colour map';
      attributeStr = 'linear';
      hueStr = 'bmw';
      colpts = [ 5 30 -52
                 15 49 -80
                 25 64 -105
                 35 73 -105
                 45 81 -88
                 55 90 -71
                 65 85 -55
                 75 58 -38
                 85 34 -23
                 95 10 -7]; 

     case 'L8'
      desc = 'Blue-Magenta-Orange-Yellow highly saturated colour map';
      attributeStr = 'linear';
      hueStr = 'bmy';
      colpts = [10 ch2ab( 78,-60)  
                20 ch2ab(100,-60)
                30 ch2ab( 78,-40)
                40 ch2ab( 74,-20)                
                50 ch2ab( 80,  0)                 
                60 ch2ab( 80, 20)
                70 ch2ab( 72, 50)
                80 ch2ab( 84, 77)
                95 ch2ab( 90, 95)];

     case 'L9'   % Blue to yellow section of R1 with short extensions at
                 % each end 
       attributeStr = 'linear';
       hueStr = 'bgyw';       
       colpts = [15  50 -65
                 35  67 -100
                 45 -14 -30
                 60 -55 60
                 85 -10 80
                 95 -17 50
                 100 0  0];
      
     case {'L10', 'GEOGRAPHIC1'}
      desc = ['A ''geographical'' colour map.  '...
              'Best used with relief shading'];
      attributeStr = 'linear';
      hueStr = 'gow';
      colpts = [60 ch2ab(20, 180)   % pale blue green
                65 ch2ab(30, 135)
                70 ch2ab(35, 75)
                75 ch2ab(45, 85)
                80 ch2ab(22, 90)        
                85 0 0   ];
      
     case {'L11', 'GEOGRAPHIC2'}  %  Lighter version of L10 with a bit more chroma 
      desc = ['A ''geographical'' colour map.  '...
              'Best used with relief shading'];
      attributeStr = 'linear';
      hueStr = 'gow';
      colpts = [65 ch2ab(50, 135)   % pale blue green
                75 ch2ab(45, 75)
                80 ch2ab(45, 85)
                85 ch2ab(22, 90)        
                90 0 0   ];

     case {'L12', 'DEPTH'}
      desc =  'A ''water depth'' colour map';
      attributeStr = 'linear';
      hueStr = 'blue';
      colpts = [95 0 0
                80 ch2ab(20, -95)
                70 ch2ab(25, -95)
                60 ch2ab(25, -95)
                50 ch2ab(35, -95)];

      
     % The following three colour maps are for ternary images, eg Landsat images
     % and radiometric images.  These colours form the nominal red, green and
     % blue 'basis colours' that are used to form the composite image.  They are
     % designed so that they, and their secondary colours, have nearly the same
     % lightness levels and comparable chroma.  This provides consistent feature
     % salience no matter what channel-colour assignment is made.  The colour
     % maps are specified as straight lines in RGB space.  For their derivation
     % see
     % http://peterkovesi.com/projects/colourmaps/ColourmapTheory/index.html#ternary
     
     case {'L13', 'REDTERNARY'}
      desc = 'red colour map for ternary images';
      attributeStr = 'linear';
      hueStr = 'ternary-red';
      colourspace = 'RGB';
      colpts = [0.00 0.00 0.00
                0.90 0.17 0.00];

      splineorder = 2; 
      
     case {'L14', 'GREENTERNARY'}
      desc = 'green colour map for ternary images';
      attributeStr = 'linear';
      hueStr = 'ternary-green';
      colourspace = 'RGB';
      colpts = [0.00 0.00 0.00
                0.00 0.50 0.00];
      
      splineorder = 2; 
      
     case {'L15', 'BLUETERNARY'}
      desc = 'blue colour map for ternary images';
      attributeStr = 'linear';
      hueStr = 'ternary-blue';
      colourspace = 'RGB';
      colpts = [0.00 0.00 0.00
                0.10 0.33 1.00];

      splineorder = 2; 

      
      
      
      
      

     %--------------------------------------------------------------------------------
     %% Diverging colour maps
     
     % Note that on these colour maps often we do not go to full white but use a
     % lightness value of 95. This helps avoid saturation problems on monitors.
     % A lightness smoothing sigma of 5 to 7 is used to avoid generating a false
     % feature at the white point in the middle.  Note however, this does create
     % a small perceptual contrast blind spot at the middle.
        
     case 'D1'
      desc = 'Diverging blue-white-red colour map';
      attributeStr = 'diverging';
      hueStr = 'bwr';
      colpts = [40  ch2ab(83,-64)
                95  0   0
                40  ch2ab(83, 39)];   
      sigma = 7;
      splineorder = 2; 

      
     case 'D1A'   % Attempt to imrove metric property of D1
      desc = 'Diverging blue-white-red colour map';
      attributeStr = 'diverging';
      hueStr = 'bwr';
      colpts = [40  ch2ab(83,-64)
                65  ch2ab(70, -28)
                95  0   0
                95  0   0
%                65  ch2ab(60, 13)
                65  ch2ab(70, 75)
                40  ch2ab(83, 39)];   
      sigma = 7;
      splineorder = 3; 
      
      
     case 'D2'
      desc = 'Diverging green-white-violet colour map';
      attributeStr = 'diverging';
      hueStr = 'gwv';
      colpts = [55 -50  55
                95   0   0
                55  60 -55];  
      sigma = 7;
      splineorder = 2; 
     
     case 'D3'
      desc = 'Diverging green-white-red colour map';
      attributeStr = 'diverging';
      hueStr = 'gwr';
      colpts = [55 -50 55
                95   0  0
                55  63 39];  
      sigma = 7;
      splineorder = 2;  
      
     case 'D4'
      desc = 'Diverging blue - black - red colour map';
      attributeStr = 'diverging';
      hueStr = 'bkr';
      colpts = [55 ch2ab(70, -85)
                10  0   0
                55 ch2ab(70, 35)];   
      sigma = 7;
      splineorder = 2; 
      
     case 'D5'
      desc = 'Diverging green - black - red colour map';
      attributeStr = 'diverging';
      hueStr = 'gkr';
      colpts = [60 ch2ab(80, 134)
                10  0   0
                60 ch2ab(80, 40)];   
      sigma = 7;
      splineorder = 2; 
      
     case 'D6'
      desc = 'Diverging blue - black - yellow colour map';
      attributeStr = 'diverging';
      hueStr = 'bky';
      colpts = [60 ch2ab(60, -95)
                10  0   0
                60 ch2ab(60, 85)];   
      sigma = 7;
      splineorder = 2;        

     case 'D7'  % Linear diverging  blue - grey - yellow.  Works well
      attributeStr = 'diverging-linear';
      hueStr = 'bjy';
      colpts = [30 ch2ab(89, -59)
                60 0 0                 
                90 ch2ab(89,96)];
      splineorder = 2;
      
     case 'D7B'  % Linear diverging blue - grey - yellow.
                 % Similar to 'D7' but with slight curves in the path to
                 % slightly increase the chroma at the 1/4 and 3/4 locations
                 % in the map.
      attributeStr = 'diverging-linear';
      hueStr = 'bjy';
      
      h1 = -59; h2 = 95;
      mh = (h1+h2)/2;
      dh = 10;
      
      colpts = [30 ch2ab(88, h1)
                48 ch2ab(40, h1+dh)
                60 0 0       
                60 0 0       
                72 ch2ab(40, h2-dh)
                90 ch2ab(88, h2)];
      splineorder = 3;

     case 'D8' % Linear diverging  blue - grey - red
      attributeStr = 'diverging-linear';
      hueStr = 'bjr';      
      colpts = [30 ch2ab(105, -60)
                42.5 0 0                 
                55 ch2ab(105,40)];
      splineorder = 2;
      
     case 'D9'  % Lightened version of D1 for relief shading - Good.
      desc = 'Diverging low contrast blue - red colour map';
      attributeStr = 'diverging';
      hueStr = 'bwr';
      colpts = [55  ch2ab(73,-74)
                98  0   0
                55  ch2ab(73, 39)];   
      sigma = 2;   % Less smoothing needed for low contrast
      splineorder = 2; 
      
     case 'D10' % low contrast diverging map for when you want to use
                % relief shading 
      desc = 'Diverging low contrast cyan - white - magenta colour map';
      attributeStr = 'diverging';
      hueStr = 'cwm';
      colpts = [80 ch2ab(44, -135)
                100  0   0
                80 ch2ab(44, -30)];   
      sigma = 0;   % No smoothing needed for lightness range of 20
      splineorder = 2; 
      W = [1 1 1];
      
     case 'D11' % Constant lightness diverging map for when you want to use
                % relief shading  ? Perhaps lighten the grey so it is not quite
                % isoluminant ?
      desc = 'Diverging isoluminat lightblue - lightgrey - orange colour map';
      attributeStr = 'diverging-isoluminant';
      hueStr = 'cjo';
      colpts = [70 ch2ab(50, -115)
                70  0   0
                70 ch2ab(50, 45)];   
      sigma = 7;
      splineorder = 2; 
      W = [1 1 1];
      
     case 'D12' % Constant lightness diverging map for when you want to use
                % relief shading  ? Perhaps lighten the grey so it is not quite
                % isoluminant ?
      desc = 'Diverging isoluminat lightblue - lightgrey - pink colour map';
      attributeStr = 'diverging-isoluminant';
      hueStr = 'cjm';
      colpts = [75 ch2ab(48, -127)
                75  0   0
                75 ch2ab(48, -30)];   
      sigma = 7;
      splineorder = 2; 
      W = [1 1 1];
       
 
     %-------------------------------------------------------------------------      
     %% Cyclic colour maps
    
     case 'C1' % I think this is my best zigzag style cyclic map - Good!
               % Control points are placed so that lightness steps up and
               % down are equalised.  Additional intermediate points are
               % placed to try to even up the 'spread' of the key colours.
       attributeStr = 'cyclic';
       hueStr = 'mrybm'; 
       mag = [75 60 -40];
       yel = [75 0 77];
       blu = [35  70 -100];
       red = [35 60 48];
       colpts = [mag
                 55 70 0 
                 red
                 55 35 62
                 yel
                 50 -20 -30
                 blu
                 55 45 -67
                 mag];
       
       sigma = 7;
       splineorder = 2;  % linear path 

     case 'C2' % A big diamond across the gamut.  Really good!  Incorporates two
               % extra cotnrol points around blue to extend the width of that
               % segment slightly.
      attributeStr = 'cyclic';
      hueStr = 'mygbm';
      colpts = [62.5  83 -54
                80 20 25
                95 -20 90
                62.5 -65 62   
                42 10 -50
                30 75 -103                
                48 70 -80  
                62.5  83 -54];
      sigma = 7;
      splineorder = 2;
      
     case 'C3'   % red-white-blue-black-red allows quadrants to be identified
      desc = 'Cyclic: red - white - blue - black - red';  
      attributeStr = 'cyclic';
      hueStr = 'rwbkr';
      colpts = [50 ch2ab(85, 39)
                85 0 0
                50 ch2ab(85, -70)
                15 0 0
                50 ch2ab(85, 39)];
      sigma = 7;
      splineorder = 2;
      
     case 'C4'   % white-red-white-blue-white Works nicely
      desc = 'Cyclic: white - red - white - blue';  
      attributeStr = 'cyclic';
      hueStr = 'wrwbw';
      colpts = [90 0 0
                40 65 56
                90 0 0
                40 31 -80
                90 0 0];
      sigma = 7;
      splineorder = 2;      

     case {'C5', 'CYCLICGREY'}   % Cyclic greyscale  Works well
      desc = 'Cyclic: greyscale'; 
      attributeStr = 'cyclic';
      hueStr = 'grey';
      colpts = [50 0 0
                85 0 0
                15 0 0
                50 0 0];
      sigma = 7;
      splineorder = 2;
      
     case 'C6'  % Circle at 67  - sort of ok but a bit flouro
      attributeStr = 'cyclic-isoluminant';
      hueStr = 'mgbm';
      chr = 42;
      ang = 124;
      colpts = [67  ch2ab(chr,  ang-90)
                67  ch2ab(chr,  ang)
                67  ch2ab(chr,  ang+90)
                67  ch2ab(chr,  ang+180)
                67  ch2ab(chr,  ang-90)];
      W = [1 1 1];
      
     case 'C7'  % Elliptical path - ok
      attributeStr = 'cyclic';
      hueStr = 'mygbm';
      ang = 112;
      colpts = [70    ch2ab(46,  ang-90)
                90    ch2ab(82,  ang)
                70    ch2ab(46,  ang+90)
                50    ch2ab(82,  ang+180)
                70    ch2ab(46,  ang-90)];
      W = [1 1 1];
      
     case 'C8' % Elliptical path.  Greater range of lightness values and
               % slightly more saturated colours.  Seems to work however I do
               % not find the colour sequence that attractive. This is a
               % constraint of the gamut.
      attributeStr = 'cyclic';
      hueStr = 'mybm';      
      ang = 124;
      colpts = [60    ch2ab(40,  ang-90)
                100   ch2ab(98,  ang)
                60    ch2ab(40,  ang+90)
                20    ch2ab(98,  ang+180)
                60    ch2ab(40,  ang-90)];
      W = [1 1 1];
      sigma = 7;                        

     case 'C9' % Variation of C1. Perceptually this is good. Excellent balance
               % of colours in the quadrants but the colour mix is not to my
               % taste.  Don't like the green.  The red-green transition clashes
       attributeStr = 'cyclic';
       hueStr = 'bgrmb';       
       blu = [35  70 -100];
       colpts = [blu
                 70 -70 64
                 35 65 50
                 70 75 -46
                 blu        ];      
       sigma = 7;
       splineorder = 2;  % linear path 
      
      
     %-----------------------------------------------------------------------------    
     %%  Rainbow style colour maps
    
     case {'R1', 'RAINBOW'}   % Reasonable rainbow colour map after it has been
                              % fixed by equalisecolourmap.
       desc = ['The least worst rainbow colour map I can devise.  Note there are' ...
              ' small perceptual blind spots at yellow and red'];
       attributeStr = 'rainbow';
       hueStr = 'bgyrm';       
       colpts = [35 60 -100
                 45 -15 -30
                 60 -55 60
                 85 0 80
                 55 70 65
                 75 55 -35];      
       sigma = 7;
       splineorder = 2;  % linear path 

     case {'R2', 'RAINBOW2'}   % Similar to R1 but with the colour map finishing
                               % at red rather than continuing onto pink.
       desc = ['Reasonable rainbow colour map from blue to red.  Note there is' ...
              ' a small perceptual blind spot at yellow'];
       attributeStr = 'rainbow';
       hueStr = 'bgyr';
       colpts = [35 60 -100
                 45 -15 -30
                 60 -55 60
                 85 0 80
                 55 75 70];
       sigma = 5;
       splineorder = 2;  % linear path 
       
       
     case 'R3'   % Diverging rainbow.  The blue and red points are matched in
                 % lightness and chroma as are the green and magenta points 
       attributeStr = 'diverging-rainbow';
       hueStr = 'bgymr';
       colpts = [45 39 -83
                 52 -23 -23
                 60 -55 55
                 85 -2 85
                 60 74 -17
                 45 70 59];
       
       sigma = 5;
       splineorder = 2;  % linear path 
       
       

     %-----------------------------------------------------------------------------    
     %%  Isoluminant colour maps       
      
     case 'I1'
      desc = ['Isoluminant blue to green to orange at lightness 70.  '...
              'Poor on its own but works well with relief shading'];
      attributeStr = 'isoluminant';
      hueStr = 'cgo';
      colpts = [70 ch2ab(40, -115)
                70 ch2ab(50, 160)
                70 ch2ab(50,  90)
                70 ch2ab(50,  45)];
      W = [1 1 1];

     case 'I2'  % Adaptation of I1 shifted to 80 from 70
      desc = ['Isoluminant blue to green to orange at lightness 80.  '...
              'Poor on its own but works well with relief shading'];
      attributeStr = 'isoluminant';
      hueStr = 'cgo';
      colpts = [80 ch2ab(40, -115)
                80 ch2ab(50, 160)
                80 ch2ab(50,  90)
                80 ch2ab(50,  45)];
      W = [1 1 1];
      
     case 'I3'
      attributeStr = 'isoluminant';
      hueStr = 'cm';
      colpts = [70 ch2ab(40, -125)
                70 ch2ab(40, -80)
                70 ch2ab(40, -40)
                70 ch2ab(50,  0)];
      W = [1 1 1];      
      

      
     %-----------------------------------------------------------------------------    
     %%  Experimental colour maps and colour maps that illustrate some design principles


     case 'X1'
      desc = ['Two linear segments with different slope to illustrate importance' ...
              ' of lightness gradient.'];
      attributeStr = 'linear-lightnessnormalised';
      hueStr = 'by';
      colpts = [30 ch2ab(102, -54)
                40  0   0
                90  ch2ab(90, 95)];
      W = [1 0 0];
      splineorder = 2;
      
     case 'X2'
      desc = ['Two linear segments with different slope to illustrate importance' ...
              ' of lightness gradient.'];
      attributeStr = 'linear-CIE76normalised';
      hueStr = 'by';
      colpts = [30 ch2ab(102, -54)
                40  0   0
                90  ch2ab(90, 95)];
      W = [1 1 1];
      splineorder = 2;      

      
     case 'X3' % Constant lightness 'v' path to test unimportance of having a smooth
               % path in hue/chroma.  Slight 'feature' at the red corner (Seems more
               % important on poor monitors)
      attributeStr = 'isoluminant-HueChromaSlopeDiscontinuity';
      hueStr = 'brg';
      colpts = [50 17 -78
                50 77 57
                50 -48 50];
      splineorder = 2;  % linear path      
      W = [1 1 1];      
      
     
       
     % A set of isoluminant colour maps only varying in saturation to test
     % the importance of saturation (not much) Colour Maps are linear with
     % a reversal to test importance of continuity.  
       
     case 'X10'  % Isoluminant 50 only varying in saturation
      attributeStr = 'isoluminant';
      hueStr = 'r';
      colpts = [50 0 0
                50 77 64
                50 0 0];
      splineorder = 2;
      sigma = 0;
      W = [1 1 1];
      
     case 'X11'  % Isoluminant 50 only varying in saturation
      attributeStr = 'isoluminant';
      hueStr = 'b';
      colpts = [50 0 0
                50 0 -56
                50 0 0];
      splineorder = 2;
      sigma = 0;
      W = [1 1 1];      
      
     case 'X12'  % Isoluminant 90 only varying in saturation
      attributeStr = 'isoluminant';
      hueStr = 'isoluminant_90_g';
      colpts = [90 0 0
                90 -76 80
                90 0 0];
      splineorder = 2;
      sigma = 0;
      W = [1 1 1];      

      
    % Difference in CIE76 and CIEDE2000 in chroma      
      
    case 'X13'  % Isoluminant 55 only varying in chroma. CIEDE76
      attributeStr= 'isoluminant-CIE76';
      hueStr = 'jr';
      colpts = [55 0 0
                55 80 67];
      splineorder = 2;
      W = [1 1 1];      
      formula = 'CIE76';
      
     case 'X14'  % Same as X13 but using CIEDE2000
      attributeStr= 'isoluminant-CIEDE2000';
      hueStr = 'jr';
      colpts = [55 0 0
                55 80 67];
      splineorder = 2;
      W = [1 1 1];      
      formula = 'CIEDE2000';      
      
     case 'X15'  % Grey 0 - 100. Same as No 1 but with CIEDE2000
      desc = 'Grey scale'; 
      attributeStr= 'linear-CIEDE2000';
      hueStr = 'grey';
      colpts = [  0 0 0      
                  100 0 0];
      splineorder = 2;
      formula = 'CIEDE2000';   

     case 'X16'  % Isoluminant 30 only varying in chroma
      attributeStr= 'isoluminant';
      hueStr = 'b';
      colpts = [30 0 0
                30 77 -106];
      splineorder = 2;
      W = [1 1 1];      
    
      
     case 'X21' % Blue to yellow section of rainbow map R1 for illustrating
                % colour ordering issues
       attributeStr= 'rainbow-section1';
       hueStr = 'bgy';       
       colpts = [35 60 -100
                45 -15 -30
                60 -55 60
                85 0 80];
       splineorder = 2;  % linear path 

     case 'X22' % Red to yellow section of rainbow map R1 for illustrating
                % colour ordering issues
       attributeStr= 'rainbow-section2';
       hueStr = 'ry';       
       colpts = [55 70 65
                85 0 80];
       splineorder = 2;  % linear path 

     case 'X23' % Red to pink section of rainbow map R1 for illustrating
                % colour ordering issues
       attributeStr= 'rainbow-section3';                
       hueStr = 'rm';       
       colpts = [55 70 65
                75 55 -35];      
       splineorder = 2;  % linear path 

       
     case 'XD1'  % Same as D1 but with no smoothing
      desc = 'Diverging blue-white-red colour map';
      attributeStr = 'diverging';
      hueStr = 'bwr';
      colpts = [40  ch2ab(83,-64)
                95  0   0
                40  ch2ab(83, 39)];   
      sigma = 0;
      splineorder = 2; 
      

     case 'X30'
      desc = 'red - green - blue interpolated in rgb';
      attributeStr = 'linear';
      hueStr = 'rgb';
      colourspace = 'RGB';
      colpts = [1.00 0.00 0.00
                0.00 1.00 0.00
                0.00 0.00 1.00];
      sigma = 0;
      W = [0 0 0];
      splineorder = 2; 
      
     case 'X31'
      desc = 'red - green - blue interpolated in CIELAB';
      attributeStr = 'linear';
      hueStr = 'rgb';
      colourspace = 'LAB';
      colpts = [53  80   67
                88 -86   83
                32  79 -108];
      sigma = 0;
      W = [0 0 0];
      splineorder = 2;       

     case 'XD7A'  % Linear diverging blue - magenta- grey - orange - yellow.
                 % Modified from 'D7' to have a double arch shaped path in an attempt
                 % to improve its Metric properties.  Also starts at lightness
                 % of 40 rather than 30.  The centre grey region is a bit too
                 % prominant and overall the map is perhaps a bit too 'bright'
      attributeStr = 'diverging-linear';
      hueStr = 'bmjoy';
      colpts = [40 ch2ab(88, -64)
                55 ch2ab(70, -30)
                64 ch2ab(2.5, -72.5)
                65 0 0       
                66 ch2ab(2.5, 107.5)
                75 ch2ab(70, 70)
                90 ch2ab(88,100)];
      splineorder = 3;

      
     case 'XC15B'   % Francesca Samsel's beautiful c15b blue-white-green
                    % diverging map.  Map is reproduced in its original form
                    % with no equalisation of lightness gradient.  (It is
                    % close to being equal as originally designed.)
      desc = 'Francesca Samsel''s c15b blue-white-green diverging map';
      attributeStr = 'diverging';
      hueStr = 'bwg';
      colourspace = 'RGB';
      colpts = [0.231373 0.247059 0.329412
                0.266667 0.305882 0.411765
                0.286275 0.368627 0.478431
                0.301961 0.439216 0.549020
                0.309804 0.521569 0.619608
                0.380392 0.631373 0.690196
                0.454902 0.745098 0.760784
                0.541176 0.831373 0.803922
                0.631373 0.901961 0.843137
                0.768627 0.960784 0.894118
                0.901961 1.000000 0.949020
                0.768627 0.960784 0.835294
                0.635294 0.909804 0.698039
                0.552941 0.850980 0.576471
                0.490196 0.780392 0.466667
                0.447059 0.701961 0.384314
                0.407843 0.611765 0.305882
                0.360784 0.509804 0.231373
                0.305882 0.400000 0.160784
                0.231373 0.278431 0.098039
                0.141176 0.149020 0.043137];
      
      sigma = 0;  
      W = [0 0 0];  % Set to zero to reproduce original map exactly
      splineorder = 2;             

     case 'XC15BM'   % Francesca's map modified slightly with additional
                     % starting control point to create symmetric lightness
                     % profile. Alternatively, remove the near black colour at
                     % the end.  Map is equalised for lightness gradient.  A
                     % really nice map!
      desc = 'Francesca Samsel''s c15b blue-white-green diverging map';
      attributeStr = 'diverging';
      hueStr = 'bwg';
      colourspace = 'RGB';
      colpts = [%0.120752 0.138784 0.214192 % additional point to match lightness at end
                0.231373 0.247059 0.329412
                0.266667 0.305882 0.411765
                0.286275 0.368627 0.478431
                0.301961 0.439216 0.549020
                0.309804 0.521569 0.619608
                0.380392 0.631373 0.690196
                0.454902 0.745098 0.760784
                0.541176 0.831373 0.803922
                0.631373 0.901961 0.843137
                0.768627 0.960784 0.894118
                0.901961 1.000000 0.949020
                0.768627 0.960784 0.835294
                0.635294 0.909804 0.698039
                0.552941 0.850980 0.576471
                0.490196 0.780392 0.466667
                0.447059 0.701961 0.384314
                0.407843 0.611765 0.305882
                0.360784 0.509804 0.231373
                0.305882 0.400000 0.160784
%                0.231373 0.278431 0.098039
                  0.2218    0.2690    0.0890 % tweeked to match start lightness
               % 0.141176 0.149020 0.043137
               ];
      
      sigma = 7;  
      W = [1 0 0];
      splineorder = 2;             
      
      
     case 'XD7C'  % Linear diverging  green - grey - yellow
                  % Reasonable, perhaps easier on the eye than D7

      attributeStr = 'diverging-linear';
      hueStr = 'gjy';
      rad = 65;
      colpts = [40 ch2ab(rad, 136)
                65 0 0                 
                90 ch2ab(rad, 95)];
      splineorder = 2;
      
      
      
      
     %%-------------------------------------------------------------  
       
     otherwise
      % Invoke the catalogue search to help the user
      catalogue(I);
       
      clear map;
      clear name;
      clear desc;
      return
      
    end

    
    % Adjust chroma/saturation but only if colourspace is LAB
    if strcmpi(colourspace, 'LAB')
        colpts(:,2:3) = chromaK * colpts(:,2:3);
    end
    
    % Colour map path is formed via a b-spline in the specified colour space
    Npts = size(colpts,1);
    
    if Npts < 2
        error('Number of input points must be 2 or more')
    elseif Npts < splineorder
        splineorder = Npts;
        fprintf('Warning: Spline order is greater than number of data points\n')
        fprintf('Reducing order of spline to %d\n', Npts)
    end
    
    % Rely on the attribute string to identify if colour map is cyclic.  We may
    % want to construct a colour map that has identical endpoints but do not
    % necessarily want continuity in the slope of the colour map path.
    if strfind(attributeStr, 'cyclic')
        cyclic = 1;
        labspline = pbspline(colpts', splineorder, N);
    else
        cyclic = 0;
        labspline = bbspline(colpts', splineorder, N);
    end    
    
    % Apply contrast equalisation with required parameters. Note that sigma is
    % normalised with respect to a colour map of length 256 so that if a short
    % colour map is specified the smoothing that is applied is adjusted to suit.
    sigma = sigma*N/256;
    map = equalisecolourmap(colourspace, labspline', formula,...
                            W, sigma, cyclic, diagnostics);    

    % If specified apply a cyclic shift to the colour map
    if shift
        if isempty(strfind(attributeStr, 'cyclic'))
            fprintf('Warning: Colour map shifting being applied to a non-cyclic map\n');
        end
        map = circshift(map, round(N*shift));    
    end
    
    if reverse
       map = flipud(map);
    end    
        
    % Compute mean chroma of colour map
    lab = rgb2lab(map);
    meanchroma = sum(sqrt(sum(lab(:,2:3).^2, 2)))/N;
    
    % Construct lightness range description
    if strcmpi(colourspace, 'LAB')  % Use the control points
        L = colpts(:,1);
    else  % For RGB use the converted CIELAB values
        L = round(lab(:,1));
    end
    minL = min(L);
    maxL = max(L);
    
    if minL == maxL     % Isoluminant
        LStr = sprintf('%d', minL);
    
    elseif L(1) == maxL && ...
                 (~isempty(strfind(attributeStr, 'diverging')) ||...
                  ~isempty(strfind(attributeStr, 'linear')))
        LStr = sprintf('%d-%d', maxL, minL);
        
    else          
        LStr = sprintf('%d-%d', minL, maxL);
    end
    
    % Build overall colour map name
    name = sprintf('%s_%s_%s_c%d_n%d',...
                   attributeStr, hueStr, LStr, round(meanchroma), N);
    
    if shift
       name = sprintf('%s_s%d', name, round(shift*100)); 
    end
    
    if reverse
       name = sprintf('%s_r', name);
    end    
    
    
    if diagnostics  % Print description and plot path in colourspace
        fprintf('%s\n',desc);
        colourmappath(map, 'fig', 10)
    end
    
    
%------------------------------------------------------------------
% Conversion from (chroma, hue angle) description to (a*, b*) coords

function ab = ch2ab(chroma, angle_degrees)
    
    theta = angle_degrees/180*pi;
    ab = chroma*[cos(theta) sin(theta)];
    

%------------------------------------------------------------------
%
% Function to list colour maps with names containing a specific string.
% Typically this is used to search for colour maps having a specified attribute:
% 'linear', 'diverging', 'rainbow', 'cyclic', 'isoluminant' or 'all'.
% This code is awful!

function catalogue(str)
    
    if ~exist('str', 'var')
        str = 'all';
    end
    
    % Get all case expressions in this function
    caseexpr = findcaseexpr;
    
    % Construct all colour map names
    for n = 1:length(caseexpr)
        % Get 1st comma separated element in caseexpr
        label = strtok(caseexpr{n},',');  
        [~, name{n}] = cmap(label);
    end

    % Check each colour name for the specified search string.  Exclude the
    % experimental maps with label starting with X.
    fprintf('\n  CMAP label(s)            Colour Map name\n')
    fprintf('------------------------------------------------\n')
    
    found = 0;
    
    for n = 1:length(caseexpr)    
        if caseexpr{n}(1) ~= 'X'            
            if any(strfind(upper(name{n}), str)) || strcmpi(str, 'all')
                fprintf('%-20s   %s\n', caseexpr{n}, name{n});
                found = 1;
            end
        end
    end        

    if ~found
        fprintf('Sorry, no colour map with label or attribute %s found\n', str);
    end


%-----------------------------------------------------------------------
% Function to find all the case expressions in this function.  Yuk there must
% be a better way!  Assumes the case statement do not span more than one line

function caseexpr = findcaseexpr
    [fid, msg] = fopen([mfilename('fullpath') '.m'], 'r');
    error(msg);

    caseexpr = {};
    n = 0;
    line = fgetl(fid);

    while ischar(line) 
        [tok, remain] = strtok(line);
        
        if strcmpi(tok, 'case')
            % If we are here remain should be the case expression.
            % Remove any trailing comment from case expression
            [tok, remain] = strtok(remain,'%');            

            % Remove any curly brackets from the case expression
            tok(tok=='{') = [];
            tok(tok=='}') = [];
            tok(tok=='''') = [];

            n = n+1;
            caseexpr{n} = tok;            
        end
        line = fgetl(fid);
    end
    
    caseexpr = strtrim(caseexpr);
    
    fclose(fid);

%-----------------------------------------------------------------------
% Function to parse the input arguments and set defaults

function [I, N, chromaK, shift, reverse, diagnostics] = parseinputs(varargin)
    
    p = inputParser;

    numericORchar    = @(x) isnumeric(x) || ischar(x);
    numericORlogical = @(x) isnumeric(x) || islogical(x);
    
    % The first argument is either a colour map label string or a string to
    % search for in a colourmap name. If no argument is supplied it is assumed
    % the user wants to list all possible colourmaps.
    addOptional(p, 'I', 'all', @ischar); 
    
    % Optional parameter-value pairs and their defaults    
    addParameter(p, 'N',     256, @isnumeric);  
    addParameter(p, 'shift',   0, @isnumeric);  
    addParameter(p, 'chromaK', 1, @isnumeric);     
    addParameter(p, 'reverse', 0, numericORlogical);  
    addParameter(p, 'diagnostics', 0, numericORlogical);  
    
    parse(p, varargin{:});
    
    I = strtrim(upper(p.Results.I));
    N = p.Results.N;
    chromaK     = p.Results.chromaK;
    shift       = p.Results.shift;
    reverse     = p.Results.reverse;
    diagnostics = p.Results.diagnostics;    
    
    if abs(shift) > 1
        error('Cyclic shift fraction magnitude cannot be larger than 1');
    end
    
    if chromaK < 0
        error('chromaK must be greater than 0')
    end    
    
    if chromaK > 1
        fprintf('Warning: chromaK is greater than 1. Gamut clipping may occur')
    end        
    
    
    