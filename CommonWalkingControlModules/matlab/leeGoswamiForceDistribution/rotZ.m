%
% rotZ.m
%
% Returns a rotation matrix that rotates about the z-axis
% Twan Koolen, May 12, 2009 
%

function rotZ = rotZ(radians)
alpha = radians;
sAlpha = sin(alpha);
cAlpha = cos(alpha);

rotZ = [cAlpha, -sAlpha, 0;
        sAlpha,  cAlpha, 0;
        0     ,       0, 1];
end