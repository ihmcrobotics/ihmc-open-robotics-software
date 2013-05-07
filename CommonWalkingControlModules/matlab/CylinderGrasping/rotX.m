%
% rotX.m
%
% Returns a rotation matrix that rotates about the x-axis
% Twan Koolen, May 12, 2009 
%

function rotX = rotX(radians)
gamma = radians;
sGamma = sin(gamma);
cGamma = cos(gamma);

rotX = [1, 0, 0;
        0, cGamma, -sGamma;
        0, sGamma,  cGamma];
end