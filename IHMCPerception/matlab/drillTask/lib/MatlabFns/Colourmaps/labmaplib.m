% LABMAPLIB Library of colour maps designed in CIELAB or RGB space
%
% Usage:  1:  [map, name, desc] = labmaplib(I, param_name, value ...)
%         2:  labmaplib
%         3:  labmaplib(str)
%
% Arguments for Usage 1:
%
%             I - An integer specifying the index of the colour map to be
%                 generated or a string specifying a colour map name to search
%                 for.  Type labmaplib with no arguments to get a full list of
%                 possible colour maps and their corresponding numbers.
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
% Usage 2 and 3:  labmaplib(str)
%
% Given the large number of colour maps that this function can create this usage
% option provides some help by listing the numbers of all the colour maps with
% names containing the string 'str'.  Typically this is used to search for
% colour maps having a specified attribute: 'linear', 'diverging', 'rainbow',
% 'cyclic', or 'isoluminant' etc.  If 'str' is omitted all colour maps are
% listed.  
%
%  >> labmaplib                % lists all colour maps
%  >> labmaplib('diverging')   % lists all diverging colour maps
%
% Note the listing of colour maps can be a bit slow because each colour map has to
% be created in order to determine its full name.
%
% See also: EQUALISECOLOURMAP, VIEWLABSPACE, SINERAMP, CIRCLESINERAMP, COLOURMAPPATH

% Adding your own colour maps is straightforward.
%
% 1) Colour maps are almost invariably defined via a spline path through CIELAB
%    colourspace.  Use VIEWLABSPACE to work out the positions of the spline
%    control points in CIELAB space to achieve the colour map path you desire.
%    These are stored in an array 'colpts' with columns corresponding to L a
%    and b.  If desired the path can be specified in terms of RGB space by
%    setting 'colourspace' to 'RGB'.  See the ternary colour maps as an example.
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
% 6) Run LABMAPLIB specifying the number of your new colour map with the
%    diagnostics flag set to one.  Various plots are generated allowing you to
%    check the perceptual uniformity, colour map path, and any gamut clipping of
%    your colour map.

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

function [map, name, desc] = labmaplib(varargin)

    [I, N, chromaK, shift, reverse, diagnostics]  = parseinputs(varargin{:});    
    
    % Check if the user wants to list a catalogue of colour maps
    if ischar(I)
        catalogue(I);
        return
    end

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
    
    switch I
        
     %% 1-19 series:  Linear scales going from low to high
        
     case 1  % Grey 0 - 100 
      desc = 'Grey scale'; 
      attributeStr = 'linear';
      hueStr = 'grey';
      colpts = [  0 0 0      
                  100 0 0];
      splineorder = 2;
      
     case 2 %  Grey 10 - 95
      desc = ['Grey scale with slightly reduced contrast to '...
              'avoid display saturation problems'];
      attributeStr = 'linear';
      hueStr = 'grey';      
      colpts = [10 0 0
                95 0 0];
      splineorder = 2;
      
     case 3 
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
 
     case 4 
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
      
     case 5 
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
      
     case 6 
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
      
     case 7 
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

     case 10  
      desc = 'Blue-Magenta-Orange-Yellow highly saturated colour map';
      attributeStr = 'linear';
      hueStr = 'bmy';
      colpts = [10 polar2ab( 78,-60)  
                20 polar2ab(100,-60)
                30 polar2ab( 78,-40)
                40 polar2ab( 74,-20)                
                50 polar2ab( 80,  0)                 
                60 polar2ab( 80, 20)
                70 polar2ab( 72, 50)
                80 polar2ab( 84, 77)
                95 polar2ab( 90, 95)];

     case 11  
      desc = 'Blue-Green-Yellow colour map';
      attributeStr = 'linear';
      hueStr = 'bgy';
      colpts = [10 polar2ab( 78,-60)      
                20 polar2ab(100,-60)   
                30 polar2ab( 80,-70)
                40 polar2ab( 40,-100)                
                50 polar2ab( 37, 180)                 
                60 polar2ab( 80, 135)
                70 polar2ab( 93, 135)
                80 polar2ab( 105, 135)  % green
                95 polar2ab( 90, 95)];  % yellow
      
     case 12  
      desc = 'Blue-Magenta-Green-Yellow highly saturated colour map';
      attributeStr = 'linear';
      hueStr = 'bmgy';
      colpts = [10 polar2ab( 78,-60)      
                20 polar2ab(100,-60)   
                30 polar2ab( 78,-40)
                40 polar2ab( 74,-20)                
                50 polar2ab( 80,  0)                 
                60 polar2ab( 95, 45)
                70 polar2ab( 70, 110)
                80 polar2ab( 105, 135)
                95 polar2ab( 90, 95)];      
      
     case 13  
      desc = 'Dark Red-Brown-Green-Yellow colour map';
      attributeStr = 'linear';
      hueStr = 'rgy';
      colpts = [10 polar2ab( 35, 24)      
                20 polar2ab( 50, 35)   
                30 polar2ab( 49, 55)
                40 polar2ab( 50, 75)                
                50 polar2ab( 55, 95)                 
                60 polar2ab( 67, 115)
                70 polar2ab( 75, 115)
                80 polar2ab( 80, 100)
                95 polar2ab( 90, 95)];            
      
     case 14
      desc = ['Attempt at a ''geographical'' colour map.  '...
              'Best used with relief shading'];
      attributeStr = 'linear';
      hueStr = 'gow';
      colpts = [60 polar2ab(20, 180)   % pale blue green
                65 polar2ab(30, 135)
                70 polar2ab(35, 75)
                75 polar2ab(45, 85)
                80 polar2ab(22, 90)        
                85 0 0   ];
      
     case 15  % Adaptation of 14.  Lighter and slightly more saturated. I
              % think this is better 
      desc = ['Attempt at a ''geographical'' colour map.  '...
              'Best used with relief shading'];
      attributeStr = 'linear';
      hueStr = 'gow';
      colpts = [65 polar2ab(50, 135)   % pale blue green
                75 polar2ab(45, 75)
                80 polar2ab(45, 85)
                85 polar2ab(22, 90)        
                90 0 0   ];

     case 16
      desc =  'Attempt at a ''water depth'' colour map';
      attributeStr = 'linear';
      hueStr = 'blue';
      colpts = [95 0 0
                80 polar2ab(20, -95)
                70 polar2ab(25, -95)
                60 polar2ab(25, -95)
                50 polar2ab(35, -95)];

      
     % The following three colour maps are for ternary images, eg Landsat images
     % and radiometric images.  These colours form the nominal red, green and
     % blue 'basis colours' that are used to form the composite image.  They are
     % designed so that they, and their secondary colours, have nearly the same
     % lightness levels and comparable chroma.  This provides consistent feature
     % salience no matter what channel-colour assignment is made.  The
     % colour maps are specified as straight lines in RGB space.  For their
     % derivation see
     % http://peterkovesi.com/projects/colourmaps/ColourmapTheory/index.html#ternary
     
     case 17
      desc = 'red colour map for ternary images';
      attributeStr = 'linear';
      hueStr = 'ternary-red';
      colourspace = 'RGB';
      colpts = [0.00 0.00 0.00
                0.90 0.17 0.00];

      splineorder = 2; 
      
     case 18
      desc = 'green colour map for ternary images';
      attributeStr = 'linear';
      hueStr = 'ternary-green';
      colourspace = 'RGB';
      colpts = [0.00 0.00 0.00
                0.00 0.50 0.00];
      
      splineorder = 2; 
      
     case 19
      desc = 'blue colour map for ternary images';
      attributeStr = 'linear';
      hueStr = 'ternary-blue';
      colourspace = 'RGB';
      colpts = [0.00 0.00 0.00
                0.10 0.33 1.00];

      splineorder = 2; 
      
     %% 20 Series:  Diverging colour maps
     
     % Note that on these colour maps we do not go to full white but use a
     % lightness value of 95. This helps avoid saturation problems on monitors.
     % A lightness smoothing sigma of 7 is used to avoid generating a false
     % feature at the white point in the middle.  Note however, this does create
     % a small perceptual contrast blind spot at the middle.
        
     case 20 
      desc = 'Diverging blue-white-red colour map';
      attributeStr = 'diverging';
      hueStr = 'bwr';
      colpts = [40  polar2ab(83,-64)
                95  0   0
                40  polar2ab(83, 39)];   
      sigma = 7;
      splineorder = 2; 
      
     case 21 
      desc = 'Diverging green-white-violet colour map';
      attributeStr = 'diverging';
      hueStr = 'gwv';
      colpts = [55 -50  55
                95   0   0
                55  60 -55];  
      sigma = 7;
      splineorder = 2; 
     
     case 22  
      desc = 'Diverging green-white-red colour map';
      attributeStr = 'diverging';
      hueStr = 'gwr';
      colpts = [55 -50 55
                95   0  0
                55  63 39];  
      sigma = 7;
      splineorder = 2;  
      
     case 23  
      desc = 'Diverging orange/brown to crimson/purple';
      attributeStr = 'diverging';
      hueStr ='owm';
      colpts = [45 polar2ab(63, 61)
                95 0 0 
                45 polar2ab(63, -15)];  
      sigma = 7;
      splineorder = 2;  
      
     case 24
      desc = 'Diverging blue - black - red colour map';
      attributeStr = 'diverging';
      hueStr = 'bkr';
      colpts = [55 polar2ab(70, -85)
                10  0   0
                55 polar2ab(70, 35)];   
      sigma = 7;
      splineorder = 2; 
      
     case 25
      desc = 'Diverging green - black - red colour map';
      attributeStr = 'diverging';
      hueStr = 'gkr';
      colpts = [60 polar2ab(80, 134)
                10  0   0
                60 polar2ab(80, 40)];   
      sigma = 7;
      splineorder = 2; 
      
     case 26
      desc = 'Diverging blue - black - yellow colour map';
      attributeStr = 'diverging';
      hueStr = 'bky';
      colpts = [60 polar2ab(60, -95)
                10  0   0
                60 polar2ab(60, 85)];   
      sigma = 7;
      splineorder = 2;        
      
     case 27
      desc = 'Diverging green - black - magenta colour map';
      attributeStr = 'diverging';
      hueStr = 'gkm';
      colpts = [60 polar2ab(80, 134)
                10  0   0
                60 polar2ab(80, -37)];   
      sigma = 7;
      splineorder = 2; 
      
     case 28  % Constant lightness diverging map for when you want to use
              % relief shading  ? Perhaps lighten the grey so it is not quite
              % isoluminant ?
      desc = 'Diverging isoluminat lightblue - lightgrey - orange colour map';
      attributeStr = 'diverging-isoluminant';
      hueStr = 'cjo';
      colpts = [70 polar2ab(50, -115)
                70  0   0
                70 polar2ab(50, 45)];   
      sigma = 7;
      splineorder = 2; 
      W = [1 1 1];

      
     case 29  % Constant lightness diverging map for when you want to use
              % relief shading  ? Perhaps lighten the grey so it is not quite
              % isoluminant ?
      desc = 'Diverging isoluminat lightblue - lightgrey - pink colour map';
      attributeStr = 'diverging-isoluminant';
      hueStr = 'cjm';
      colpts = [75 polar2ab(48, -127)
                75  0   0
                75 polar2ab(48, -30)];   
      sigma = 7;
      splineorder = 2; 
      W = [1 1 1];
       
      
    % 30 Series: Cyclic colour maps
      
     case 31 % Cyclic isoluminant path around perimeter of 60 slice Looks
             % reasonable, may want a bit of cyclic shifting, but looks good.
             % It really hurts to look at a sinewave with this map!
      attributeStr = 'cyclic-isoluminant';
      hueStr = 'grmbg';
      colpts = [60 -55 60
                60   0 60
                60  60 65
                60  70  0
                60  95 -60
                60  40 -60
                60   5 -60
                60  -40 0
                60 -55 60];
      W = [1 1 1];
      
     case 32 % Cyclic non-isoluminant path between 45 and 55 lightness
             % Interesting but not as successful as No 31
             % Problem in the blue section
      attributeStr = 'cyclic';
      hueStr = 'bormb';
      colpts = [55 -55 -55
                55  15  60
                55  80  70
                50  80  -5
                45  80  -90
                45  55  -90 
                45  25  -90
                50 -25 -25
                55 -55 -55];      
      W = [1 1 1];
      
     case 33 % Attempt at an isoluminant green-magenta cyclic map
             % Perhaps apply a circshift of -25, or + 35 to it
             % It really hurts to look at a sinewave with this map!
      attributeStr = 'cyclic-isoluminant';
      hueStr = 'mvgom';             
      colpts = [65 70 -15
                65 85 -55
                65 40 -55
                65 -55 25
                65 -60 60
                65 -20 65
                65 70 -15];
      W = [1 1 1];
       
     case 34  % Circle at 55
      attributeStr = 'cyclic-isoluminant';
      hueStr = 'mgbm';
      colpts = [55 polar2ab(36,0)
                55 polar2ab(36,60)
                55 polar2ab(36,120)
                55 polar2ab(36,180)
                55 polar2ab(36,240)
                55 polar2ab(36,300)
                55 polar2ab(36,360)];
      W = [1 1 1];

     case 35  % Circle at 67  - sort of ok but a bit flouro
      attributeStr = 'cyclic-isoluminant';
      hueStr = 'mgbm';
      rad = 42;
      ang = 124;
      colpts = [67  polar2ab(rad,  ang-90)
                67  polar2ab(rad,  ang)
                67  polar2ab(rad,  ang+90)
                67  polar2ab(rad,  ang+180)
                67  polar2ab(rad,  ang-90)];
      W = [1 1 1];
      
     case 36  % Same as 35 with shift for cycle of length pi
      attributeStr = 'cyclic-isoluminant_s';
      hueStr = 'mgbm';
      rad = 42;
      ang = 124;
      colpts = [67  polar2ab(rad,  ang-90)
                67  polar2ab(rad,  ang)
                67  polar2ab(rad,  ang+90)
                67  polar2ab(rad,  ang+180)
                67  polar2ab(rad,  ang-90)];
      W = [1 1 1];
      shift = N/4;

      
     case 37  % Distorted larger circle at 55 but with reduced radius across
              % the cyan section of the circle to fit within the gamut.
              % Quite a good compromise.
      attributeStr = 'cyclic-isoluminant';
      hueStr = 'mogbm';              
      colpts = [55 polar2ab(60,0)
                55 polar2ab(60,60)
                55 polar2ab(60,120)
                55 polar2ab(60,145)                
                55 polar2ab(37,180)
                55 polar2ab(35,240)
                55 polar2ab(50,270)
                55 polar2ab(60,300)
                55 polar2ab(60,360)];
      W = [1 1 1];
      
     case 39 % Constant lightness 'v' path to test importance of having a smooth
             % path in hue.  Slight 'feature' at the green corner (Seems more
             % important on poor monitors) but is a good example of the
             % relative unimportance of hue slope continuity 
      attributeStr = 'isoluminant-example';
      hueStr = 'vgr';
      colpts = [50 80 -75
                50 -50 50
                50 80 65];
      splineorder = 2;  % linear path      
      W = [0 0 0];
      
    
     %% 40 Series: Rainbow style colour maps
    
     case 41   % Reasonable rainbow colour map after it has been fixed by
               % equalisecolour map with a smoothing sigma value of about 7.
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

     case 42  % A sort of rainbow map.  Deliberately illustrates issues in the
              % design of these maps. The direction/lightness discontinuities in
              % LAB space at green, yellow and red produce features in the
              % colour map. It is also interesting in that it shows the
              % importance of constant change in L.  The slope is low from blue
              % to green relative green to yellow.  This is noticeable when the
              % sineramp test image is viewed with this colour map
      attributeStr = 'rainbow';             
      hueStr = 'bgyrm';       
      colpts = [45 30 -80
                55 -30 -10
                60 -55 60
                85 0 80
                55 70 65
                60 80 -55];
      splineorder = 2;  % linear path    
      W = [0 0 0];
      
     case 43  % Constant lightness of 50 version of the colour map above. 
      attributeStr = 'isoluminant';
      hueStr = 'bgrm';
      colpts = [50 30 -80
                50 -30 -10
                50 -55 60
                50 0 80
                50 70 65
                50 80 -55];      
      splineorder = 2;  % linear path            
      W = [1 1 1];

     case 44   % Similar to 41 but with the colour map finishing at red rather
               % than continuing onto pink.
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

       
     %% Miscellaneous      
     case 50   % Line from max blue saturation to max green saturation
      attributeStr = 'linear';
      hueStr = 'bg';
      colpts = [30 polar2ab(135, -59)
                90 polar2ab(112, 135)];
      splineorder = 2;  % linear path            
      
     case 51   % Line from max red saturation to max green saturation
      attributeStr = 'linear';
      hueStr = 'rog';
      colpts = [50 polar2ab(100, 40)
                90 polar2ab(112, 135)];
      splineorder = 2;  % linear path                  
      
     case 52   % Line from max blue saturation to max red saturation
      desc = 'Line from max blue saturation to max red saturation';
      attributeStr = 'linear';
      hueStr = 'bmr';
      colpts = [20 polar2ab(110, -59)
                60 polar2ab(100, 46)];
      splineorder = 2;  % linear path                        
      
     case 53   % Line from black- max blue saturation to max green saturation
               % - white
      attributeStr = 'linear';               
      hueStr = 'bgw';               
      colpts = [5 0 0
                30 polar2ab(100, -59)
                40 polar2ab(115, -59)
                80 polar2ab(112, 135)
                90 polar2ab(100, 135)
                95 0 0];      
      
     case 54 
      desc = 'Isoluminant arc blue-magenta-yellow at lightness = 65';
      attributeStr = 'isoluminant';
      hueStr = 'cmo';
      colpts = [65 -13 -55
                65 48 -54
                65 67 -7
                65 61 40
                65 22 68];
      W = [1 1 1];
      
     case 55 
      desc = 'Low contrast arc blue-magenta-yellow ';
      attributeStr = 'linear';
      hueStr = 'bmo';
      colpts = [50 polar2ab(80, -80)
                55 polar2ab(90, -50)
                60 polar2ab(82, -10)
                65 polar2ab(80, 51)
                75 polar2ab(77, 83)];
      W = [1 1 1];

     case 56 
      desc = 'Isoluminant green to orange at lightness 75';
      attributeStr = 'isoluminant';
      hueStr = 'go';
      colpts = [75 polar2ab(53, 135)
                75 polar2ab(53, 80)];
      
      W = [1 1 1];
      splineorder = 2;
      
     case 57 %  Grey 10 - 90
      desc = ['Grey scale with slightly reduced contrast to '...
              'avoid display saturation problems'];
      attributeStr = 'linear';
      hueStr = 'grey';
      colpts = [10 0 0
                90 0 0];
      splineorder = 2;      
      
     case 58
      desc = ['Diverging blue - red colour map.  No smoothing to illustrate' ...
              ' feature problem'];
      attributeStr = 'diverging-nosmoothing';
      hueStr = 'bwr';
      colpts = [40 31 -80
                95  0   0
                40 65  56];   
      sigma = 0;
      splineorder = 2; 
      
     case 59 
      desc = ['Isoluminant blue to green to orange at lightness 70.  '...
              'Poor on its own but works well with relief shading'];
      attributeStr = 'isoluminant';
      hueStr = 'cgo';
      colpts = [70 polar2ab(40, -115)
                70 polar2ab(50, 160)
                70 polar2ab(50,  90)
                70 polar2ab(50,  45)];
      W = [1 1 1];
      
     case 60 
      desc = ['Diverging blue - red colour map.  Large degree of smoothing for' ...
              ' poster'];
      attributeStr = 'diverging-oversmoothed';
      hueStr = 'bwr';
      colpts = [40 31 -80
                95  0   0
                40 65  56];   
      sigma = 12;
      splineorder = 2; 
      
     case 61 
      desc = 'Non-Isoluminant version of No 62, lightness 25 to 75.';
      attributeStr = 'linear';
      hueStr = 'bm';
      colpts = [25 polar2ab(40, -125)
                42 polar2ab(40, -80)
                59 polar2ab(40, -40)
                75 polar2ab(50,  0)];
      
     case 62
      desc = ['Isoluminant version of No 61, lightness 70.  '...
              'Poor on its own but works well with relief shading'];
      attributeStr = 'isoluminant';
      hueStr = 'cm';
      colpts = [70 polar2ab(40, -125)
                70 polar2ab(40, -80)
                70 polar2ab(40, -40)
                70 polar2ab(50,  0)];
      W = [1 1 1];      
      
     case 63 
      desc = 'Non-Isoluminant version of No 62, lightness 35 to 75.';
      attributeStr = 'linear';
      hueStr = 'bm';
      
      colpts = [35 polar2ab(40, -125)
                48 polar2ab(40, -80)
                62 polar2ab(40, -40)
                75 polar2ab(50,  0)];
      
     case 64  % Adaptation of 59 shifted to 80 from 70
      desc = ['Isoluminant blue to green to orange at lightness 80.  '...
              'Poor on its own but works well with relief shading'];
      attributeStr = 'isoluminant';
      hueStr = 'cgo';
      colpts = [80 polar2ab(40, -115)
                80 polar2ab(50, 160)
                80 polar2ab(50,  90)
                80 polar2ab(50,  45)];
      W = [1 1 1];
      
     %%  More cyclic attempts       
    
     case 70  % 29-9-2014 Everything one tries converges to a similar solution!
              % How do you get 4 perceptually equivalent land mark colours from
              % a tristimulaus system?!
      
              % Hit the corners of the gamut red - green - blue/cyan - blue with
              % small tweeks to keep linear path within the gamut.  However,
              % keep the control points symmetric in terms of lightness and only
              % weight for lightness when equalising.  This achieves symmetry of
              % the colour arrangement despite the non symmetric path because
              % the control points are symmetric in terms of lightness.
              % BUT the trouble is we 'see' only three colours! The colourmap
              % looks green for top half, blue for bottom left and red for
              % bottom right.  Interesting example
            attributeStr = 'cyclic';
            hueStr = 'rgcbr';
            colpts = [55 75 65
                      80 -80 76
                      55 19 -71
                      30 65 -95
                      55 80 67];
            W = [1 0 0];
            sigma = 3;            
            splineorder = 2;
    
            
            
     case 71   % Attempt at cyclic map based on No 41 a rainbow map
               % Looks really nice!!
               % Problem is that key colours are not spaced as well as one
               % would like.
       attributeStr = 'cyclic';
       hueStr = 'bgyrmb';       
       colpts = [35 60 -100
                45 -15 -30
                60 -55 60
                85 0 80
                55 70 65
                75 55 -35
                35 60 -100];      
       sigma = 5;
       splineorder = 2;  % linear path 

       
     case 72   % Variation of 78. Perceptually this is good. Excellent balance
               % of colours in the quadrants but the colour mix is not to my
               % taste.  Don't like the green.  The reg-green transition clashes
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
       
     case 73   % 2nd Attempt to modify 71 so that key colours are placed at
               % quadrants. Competing with 74 but I think this is a bit
               % better but 74 has better symmetry

       attributeStr = 'cyclic';
       hueStr = 'bgyrm';       
       colpts = [35 60 -100
                45 -15 -30
                80 0 80
                53 70 65
                80 47 -31
                35 60 -100];      
       sigma = 7;
       splineorder = 2;  % linear path 

       
     case 74   % 3rd Attempt to modify 71 so that key colours are placed at
               % quadrants. Competing with 73.  Has better symmetry

       attributeStr = 'cyclic';
       hueStr = 'byrmb';       
       colpts = [40 70 -95
                 60 -25 -20
                 80 0 80
                 40 70 -95
                 80 47 -31
                 60 80 -65
                 40 70 -95];      
       sigma = 7;
       splineorder = 2;  % linear path 

            
            
     case 75   % Like 70 but use yellow instead of green - Nicer to look at
      
      attributeStr = 'cyclic';
      hueStr = 'rycbr';
      colpts = [55 75 65
                90 -10 85
                55 19 -71
                20 50 -77
                55 80 67];
      W = [1 0 0];
      sigma = 3;            
      splineorder = 2;
      
      
      
     case 76  % Try a variation based on 78 with green instead of magenta. OK
              % but I dislike the colour mix
       attributeStr = 'cyclic';
       hueStr = 'byrmb';       
       blu = [35  70 -100];
       colpts = [blu
                 85 -85 81
                 35 60 48
%                 35 -40 40
                 85  3 87
                 blu        ];      
       sigma = 7;
       splineorder = 2;  % linear path 

     case 476  % okish

       attributeStr = 'cyclic';
       hueStr = 'byrmb';       
       blu = [35  70 -100];
       colpts = [blu
                 85 -40 -25
                 35 -40 40
                 85  10 87
                 60 35 -60
                 blu        ];      
       sigma = 7;
       splineorder = 2;  % linear path 


     case 479  % Try same as 477 but with yellow/orange instead of green and
               % cyan instead of magenta.  Nup!
       attributeStr = 'cyclic';
       hueStr = 'byrmb';       
       blu = [35  70 -100];
       colpts = [blu
                 70 37 77
                 35 65 50
                 70 -12 -50
                 blu        ];      
       sigma = 7;
       splineorder = 2;  % linear path 
      
      
     case 77  % OK
      attributeStr = 'cyclic';
      hueStr = 'mygbm';
      ang = 112;
      colpts = [70    polar2ab(46,  ang-90)
                90    polar2ab(82,  ang)
                70    polar2ab(46,  ang+90)
                50    polar2ab(82,  ang+180)
                70    polar2ab(46,  ang-90)];
      W = [1 1 1];
      

     case 78   % Development of 74 to improve placement of key colours.
               % Control points are placed so that lightness steps up and
               % down are equalised.  Additional intermediate points are
               % placed to try to even up the 'spread' of the key colours.
               % I think this is my best zigzag style cyclic map - Good!
               
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
      
      
      case 79  % We start at dark pink with 0 phase 90 (the peak corresponds to
               % the brightest point in the colour map at yellow, then down to
               % green/blue at 180, blue at 270 (darkest point) then back up to
               % dark pink.
      attributeStr = 'cyclic';         
      hueStr = 'mybm';
      colpts = [67.5   42.5 15
                95    -30 85
                67.5  -42.5 -15
                40    30 -85
                67.5  42.5 15];  
      W = [1 1 1];
      sigma = 9;                        
      
     case 80   % Try to push 79 a bit further.  Greater range of lightness
               % values and slightly more saturated colours.  Seems to work
               % however I do not find the colour sequence that
               % attractive. This is a constraint of the gamut.
      attributeStr = 'cyclic';
      hueStr = 'mybm';      
      ang = 124;
      colpts = [60    polar2ab(40,  ang-90)
                100   polar2ab(98,  ang)
                60    polar2ab(40,  ang+90)
                20    polar2ab(98,  ang+180)
                60    polar2ab(40,  ang-90)];
      W = [1 1 1];
      sigma = 7;                        
      
     case 82
      desc = 'Cyclic: greyscale';  % Works well
      attributeStr = 'cyclic';
      hueStr = 'grey';
      colpts = [50 0 0
                85 0 0
                15 0 0
                50 0 0];
      sigma = 7;
      splineorder = 2;
      
     case 84  %   red-white-blue-black-red allows quadrants to be identified
      desc = 'Cyclic: red - white - blue - black - red';  
      attributeStr = 'cyclic';
      hueStr = 'rwbkr';
      colpts = [50 polar2ab(85, 39)
                85 0 0
                50 polar2ab(85, -70)
                15 0 0
                50 polar2ab(85, 39)];
      sigma = 7;
      splineorder = 2;

     case 85 % A big diamond across the gamut.  Really good!
             % Incorporates two extra conrol points around blue to extend the
             % width of that segment slightly.
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
      
      
     case 87  % works fine but I think I would rather use 88 below instead
      desc = 'Cyclic: grey-yellow-grey-blue';  
      attributeStr = 'cyclic';
      hueStr = 'jyjbj';
      colpts = [60 0 0
                90 -10 90
                60 0 0
                30 79 -108
                60 0 0];
      sigma = 7;
      splineorder = 2;      

     case 88 %   % white-red-white-blue-white Works nicely
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
      

      %-------------------------------------------
     case 101
      desc = ['Two linear segments with different slope to illustrate importance' ...
              ' of lightness gradient. Equalised lightness.'];
      attributeStr = 'linear-lightnessnormalised';
      hueStr = 'by';
      colpts = [40 polar2ab(60, -75)
                72  0   0
                80  polar2ab(70, 105)];      
      splineorder = 2;
      
     case 102
      desc = ['Two linear segments with different slope to illustrate importance' ...
              ' of lightness gradient. Equalised CIE76'];
      attributeStr = 'linear-CIE76normalised';
      hueStr = 'by';
      colpts = [40 polar2ab(60, -75)
                72  0   0
                80  polar2ab(70, 105)];
      W = [1 1 1];
      splineorder = 2;
      
       
     case 103 % Constant lightness 'v' path to test unimportance of having a smooth
             % path in hue.  Slight 'feature' at the red corner (Seems more
             % important on poor monitors)
      attributeStr = 'isoluminant-HueSlopeDiscontinuity';
      hueStr = 'brg';
      colpts = [50 17 -78
                50 77 57
                50 -48 50];
      splineorder = 2;  % linear path      
      W = [1 1 1];      
      
     case 104 % Linear diverging  blue - grey - orange. This works but the
              % blue and orange are not nice to look at
      attributeStr = 'diverging-linear';
      hueStr = 'bjo';
      colpts = [30 polar2ab(90, -60)
                47.5 0 0                 
                65 polar2ab(90,55)];
      splineorder = 2;
       
     case 105 % Linear diverging  blue - grey - yellow.  Works well
      attributeStr = 'diverging-linear';
      hueStr = 'bjy';
      colpts = [30 polar2ab(89, -59)
                60 0 0                 
                90 polar2ab(89,96)];
      splineorder = 2;
      
     case 106 % Linear diverging  blue - grey - green
      attributeStr = 'diverging-linear';
      hueStr = 'bjg';
      colpts = [20 polar2ab(110, -58)
                55 0 0                 
                90 polar2ab(110,135)];
      splineorder = 2;
      
     case 107 % Linear diverging  blue - grey - red
      attributeStr = 'diverging-linear';
      hueStr = 'bjr';      
      colpts = [30 polar2ab(105, -60)
                42.5 0 0                 
                55 polar2ab(105,40)];
      splineorder = 2;
      
      
     case 108 % Linear diverging  blue - grey - magenta
      attributeStr = 'diverging-linear';
      hueStr = 'bjm';      
      colpts = [30 polar2ab(105, -60)
                45 0 0                 
                60 polar2ab(105,-32)];
      splineorder = 2;
       
     case 109 % low contrast diverging map for when you want to use
              % relief shading  Adapted from 29.  Works well
      desc = 'Diverging lowcontrast cyan - white - magenta colour map';
      attributeStr = 'diverging';
      hueStr = 'cwm';
      colpts = [80 polar2ab(44, -135)
                100  0   0
                80 polar2ab(44, -30)];   
      sigma = 0; % Try zero smoothing- not needed for low contrast
      splineorder = 2; 
      W = [1 1 1];

     case 110 % low contrast diverging map for when you want to use
              % relief shading.  Try green-white-yellow becuase you have have
              % much more strongly saturated colours.  Works well but colours
              % are a bit unusual.  Unsure which should be +ve or -ve
      desc = 'Diverging lowcontrast green - white - yellow colour map';
      attributeStr = 'diverging';
      hueStr = 'gwy';
      colpts = [80 polar2ab(83, 135)
                95  0   0
                80 polar2ab(83, 80)];   
      sigma = 0;
      splineorder = 2; 
      W = [1 1 1];
      
     case 111 % Slightly increased contrast version of 109 works well with
              % relief shading.
      attributeStr = 'diverging';
      hueStr = 'bwr';
      colpts = [60 polar2ab(60, -85)
                95  0   0
                60 polar2ab(60, 40)];   
      sigma = 0;
      splineorder = 2; 
      W = [1 1 1];
      
     case 112 % low contrast linear diverging attempt
              % magenta grey yellow.  OK but colour interpretation not so
              % intuitive 
      attributeStr = 'diverging';
      hueStr = 'mjy';
      colpts = [70 polar2ab(87, -32)
                82.5  0   0
                95 polar2ab(87, 100)];   
      sigma = 0;
      splineorder = 2; 
      W = [1 1 1];      
      
     case 113  % Lightened version of 20 for relief shading - Good.
      desc = 'Diverging blue - red colour map';
      attributeStr = 'diverging';
      hueStr = 'bwr';
      colpts = [55  polar2ab(73,-74)
                98  0   0
                55  polar2ab(73, 39)];   
      sigma = 3;   % Less smoothing needed for low contrast
      splineorder = 2; 
      
       
     %% Special colour maps designed to illustrate various design principles.
       
     % A set of isoluminant colour maps only varying in saturation to test
     % the importance of saturation (not much) Colour Maps are linear with
     % a reversal to test importance of continuity.  
     % uses de2000 with no smoothing
       
     case 200  % Isoluminant 50 only varying in saturation
      attributeStr = 'isoluminant';
      hueStr = 'r';
      colpts = [50 0 0
                50 77 64
                50 0 0];
      splineorder = 2;
      W = [1 1 1];
      
     case 201  % Isoluminant 50 only varying in saturation
      attributeStr = 'isoluminant';
      hueStr = 'b';
      colpts = [50 0 0
                50 0 -56
                50 0 0];
      splineorder = 2;
      W = [1 1 1];      
      
     case 202  % Isoluminant 90 only varying in saturation
      attributeStr = 'isoluminant';
      hueStr = 'isoluminant_90_g';
      colpts = [90 0 0
                90 -76 80
                90 0 0];
      splineorder = 2;
      W = [1 1 1];      

      
    case 203  % Isoluminant 55 only varying in saturation. CIEDE76
      attributeStr= 'isoluminant-CIE76';
      hueStr = 'jr';
      colpts = [55 0 0
                55 80 67];
      splineorder = 2;
      W = [1 1 1];      
      formula = 'CIE76';
      
     case 204  % Same as 203 but using CIEDE2000
      attributeStr= 'isoluminant-CIEDE2000';
      hueStr = 'jr';
      colpts = [55 0 0
                55 80 67];
      splineorder = 2;
      W = [1 1 1];      
      formula = 'CIEDE2000';      
      
     case 205  % Grey 0 - 100. Same as No 1 but with CIEDE2000
      desc = 'Grey scale'; 
      attributeStr= 'linear-CIEDE2000';
      hueStr = 'grey';
      colpts = [  0 0 0      
                  100 0 0];
      splineorder = 2;
      formula = 'CIEDE2000';   

     case 206  % Isoluminant 30 only varying in saturation
      attributeStr= 'isoluminant';
      hueStr = 'b';
      colpts = [30 0 0
                30 77 -106];
      splineorder = 2;
      W = [1 1 1];      
    
     case 210   % Blue to yellow section of rainbow map 41 for illustrating
                % colour ordering issues
       attributeStr= 'rainbow-section1';
       hueStr = 'bgy';       
       colpts = [35 60 -100
                45 -15 -30
                60 -55 60
                85 0 80];
       splineorder = 2;  % linear path 

     case 211   % Red to yellow section of rainbow map 41 for illustrating
                % colour ordering issues
       attributeStr= 'rainbow-section2';
       hueStr = 'ry';       
       colpts = [55 70 65
                85 0 80];
       splineorder = 2;  % linear path 

     case 212   % Red to pink section of rainbow map 41 for illustrating
                % colour ordering issues
       attributeStr= 'rainbow-section3';                
       hueStr = 'rm';       
       colpts = [55 70 65
                75 55 -35];      
       splineorder = 2;  % linear path 
      

       
      %% Another attempt at getting a better set of ternary colour maps. Try
      %  slightly unequal lightness values
     case 217
      desc = '5-55 lightness red colour map for radiometric data';
      attributeStr = 'linear';
      hueStr = 'ternary-red';
      ang = 36;  % 37
      colpts = [5 0 0
                30 polar2ab(38,ang)  % 40
                55 polar2ab(94, ang)];  % 100 39
      splineorder = 2;
      
     case 218
      desc = '5-55 lightness green colour map for radiometric data';
      attributeStr = 'linear';
      hueStr = 'ternary-green';
      colpts = [5 0 0
                65 polar2ab(88, 137)]; % 90 136
      splineorder = 2;      
      
     case 219
      desc = '5-55 lightness blue colour map for radiometric data';
      attributeStr = 'linear';
      hueStr = 'ternary-blue';
      colpts = [5 0 0
                43 polar2ab(87, -65)]; % 93 54
      splineorder = 2;      
      

      
      %% ------  Rejects / Maps that need more work  -------------
      % These are not listed in a catalogue search
      
     case 336  % Circle at 75  - is too bright ? might be ok with relief
              % shading if that makes any sense with cyclic data
      attributeStr = 'cyclic-isoluminant';
      hueStr = 'mgbm';
      colpts = [75 polar2ab(43,0)
                75 polar2ab(43,60)
                75 polar2ab(43,120)
                75 polar2ab(43,180)
                75 polar2ab(43,240)
                75 polar2ab(43,300)
                75 polar2ab(43,360)];
      W = [1 1 1];      
      
      
     case 374   % More circular less lightness contrast. Dk Green violet red,
                % orange greem cycle.  Not centred on grey.  Might be ok with
                % some work.  Want some compromise between equalisation on 1
                % or 2.
      attributeStr = 'cyclic';
      hueStr = 'gmog';
      colpts = [50 polar2ab(50,  90)
                40 polar2ab(40, -235)
                50 polar2ab(50, -25)
                60 polar2ab(90,  45)
                50 polar2ab(50,  90)];
      W = [1 1 1];
      sigma = 9;            
      
      
     case 378  % Similar philosophy to 77 but hue shift
               % pink yellow/green blue purple pink - also looks good but a
               % bit more lairy than 77
      attributeStr = 'cyclic';
      hueStr = 'mgbvm';
      colpts = [70   30 32
                90  -75 80                
                70 -30 -32
                50  80 -75
                70   30 32];
      W = [1 1 1];
      sigma = 9;                        
      
      
     case 381  % green cyan pink/red green  - A bit dark and needs some
              % equalisation work
      attributeStr = 'cyclic';
      hueStr = 'gcmg';      
      colpts = [50   -40 25
                70  -25 -40                
                50   40 -25
                30   25 40 
                50   -40 25];
      W = [1 1 1];
      sigma = 9;              
      
     case 384   % Nuh
      desc =  'Cyclic:  red - white - green - black - red';  
      attributeStr = 'cyclic';
      hueStr = 'rwgkr';
      colpts = [50 polar2ab(40, 30)
                85 0 0
                50 polar2ab(40, 140)
                15 0 0
                50 polar2ab(40, 30)];
      sigma = 7;
      splineorder = 2;

      
     case 385   % Cyclic: red - green.  Problem is that red is perceived as
               % +ve but is a darker colour than green giving confilicting
               % perceptual cues
      desc = 'Cyclic: red - green'; 
      attributeStr = 'cyclic';
      hueStr = 'gr';
      colpts = [65   0 60
                45  70 50
                85 -70 70                
                65   0 60];
      sigma = 7;
      splineorder = 2;      
      
     case 386      
      desc = 'Cyclic: red - blue';  % A bit harsh on the eye but the colour
                                    % and lighness cues are in sync
      attributeStr = 'cyclic';
      hueStr = 'vrvbv';
      colpts = [45 60 -17.5
                55  70   65
                35  50 -100
                45 60 -17.5];
      sigma = 7;
      splineorder = 2;      
      
      
      
      %%-------------------------------------------------------------  
       
     otherwise
      warning('Sorry, no colour map option with that number');
      map = [];
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
        colourmappath(map, 6, 'lab', 10)
    end
    
    
%------------------------------------------------------------------

function ab = polar2ab(radius, angle_degrees)
    
    theta = angle_degrees/180*pi;
    ab = radius*[cos(theta) sin(theta)];
    

%------------------------------------------------------------------
%
% Function to list colour maps with names containing a specific string.
% Typically this is used to search for colour maps having a specified attribute:
% 'linear', 'diverging', 'rainbow', 'cyclic', 'isoluminant' or 'all'.

function catalogue(str)
    
    if ~exist('str', 'var')
        str = 'all';
    end
    
    fprintf('\n  No   Colour Map name\n')
    fprintf('---------------------------------\n')
    
    warning('off');
    for n = 1:300
        [m, name] = labmaplib(n);
        if ~isempty(m)
            if any(strfind(name, str)) || strcmpi(str, 'all')
                fprintf('%4d   %s\n', n, name);
            end
        end
    end
    
    warning('on');

%-----------------------------------------------------------------------
% Function to parse the input arguments and set defaults

function [I, N, chromaK, shift, reverse, diagnostics] = parseinputs(varargin)
    
    p = inputParser;

    numericORchar    = @(x) isnumeric(x) || ischar(x);
    numericORlogical = @(x) isnumeric(x) || islogical(x);
    
    % The first argument is either a colour map number or a string to search for
    % in a colourmap name. If no argument is supplied it is assumed the user
    % wants to list all possible colourmaps. 
    addOptional(p, 'I', 'all', numericORchar); 
    
    % Optional parameter-value pairs and their defaults    
    addParameter(p, 'N',     256, @isnumeric);  
    addParameter(p, 'shift',   0, @isnumeric);  
    addParameter(p, 'chromaK', 1, @isnumeric);     
    addParameter(p, 'reverse', 0, numericORlogical);  
    addParameter(p, 'diagnostics', 0, numericORlogical);  
    
    parse(p, varargin{:});
    
    I = p.Results.I;
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
    
    
    