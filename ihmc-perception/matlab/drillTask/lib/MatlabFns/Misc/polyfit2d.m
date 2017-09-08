% POLYFIT2D Fits 2D polynomial surface to data
%
% Usage:  c = polyfit2d(x, y, z, degree)
%    
% Arguments:  x, y, z - coordinates of data points
%              degree - degree of polynomial surface
%                  
% Returns:          c - The coefficients of polynomial surface. 
%                       There will be (degree+1)*(degree+2)/2
%                       coefficients. 
%
% For a degree 3 surface the coefficients progress in the form
%   00 01 02 03 10 11 12 20 21 30
% where the first digit is the y exponent and the 2nd the x exponent
%
%      0 0      0 1      0 2      0 3      1 0      1 1      1 2
%  c1 x y + c2 x y + c3 x y + c4 x y + c5 x y + c6 x y + c7 x y + ...
%                   
% To reduce numerical problems this function rescales the values of x and y to a
% maximum magnitude of 1.  The calculated coefficients are then rescaled to
% account for this. Ideally the values of x and y would also be centred to have
% zero mean.  However, the correction that would then have to be applied to the
% coefficients is not so simply done.  If you do find that you have numerical
% problems you could try centering x and y prior to calling this function.
%
% See also: POLYVAL2D
    
% Peter Kovesi 2014
% Centre for Exploration Targeting
% The University of Western Australia
% peter.kovesi at uwa edu au

% PK July 2014
    
function c = polyfit2d(x, y, z, degree)
    
    % Ensure input are column vectors
    x = x(:);
    y = y(:);
    z = z(:);
    
    % To reduce numerical problems we perform normalisation of the data to keep
    % the maximum magnitude of x and y to 1.
    scale = max(abs([x; y]));
    x = x/scale;
    y = y/scale;
    
    nData = length(x);
    ncoeff = (degree+1)*(degree+2)/2;

    % Build Vandermonde matrix.  p1 is the x exponent and p2 is the y exponent
    V = zeros(nData, ncoeff);

    col = 1;
    for p2 = 0:degree
        for p1 = 0:(degree-p2)
            V(:,col) = x.^p1 .* y.^p2;
            col = col+1;
        end
    end

    [Q,R] = qr(V,0); % Solution via QR decomposition
    c = R\(Q'*z);
    
    if condest(R) > 1e10
        warning('Solution is ill conditioned. Coefficient values will be suspect')
    end
        
    % Scale coefficients to account for the earlier normalisation of x and y.
    col = 1;
    for p2 = 0:degree
        for p1 = 0:(degree-p2)
            c(col) = c(col)/scale^(p1+p2);
            col = col+1;
        end
    end    
