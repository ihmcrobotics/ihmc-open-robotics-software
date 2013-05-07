%
% rotY.m
%
% Returns a rotation matrix that rotates about the y-axis
% Twan Koolen, May 12, 2009 
%

function rotY = rotY(radians)
beta = radians;
sBeta = sin(beta);
cBeta = cos(beta);

rotY = [cBeta, 0, sBeta;
        0,     1, 0;
       -sBeta, 0,  cBeta];
end