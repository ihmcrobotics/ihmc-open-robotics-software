% PBSPLINE - Basic Periodic B-spline
%
% Usage:  S = pbspline(P, k, N)
% 
% Arguments:   P - [dim x Npts] array of control points
%              k - order of spline (>= 2). 
%                  k = 2: Linear
%                  k = 3: Quadratic, etc
%              N - Optional number of points to evaluate along
%                  spline. Defaults to 100.
%
% Returns:     S - spline curve  [dim x N] spline points
%
% See also: BBSPLINE

% PK March 2014

% Needs a bit of tidying up and checking on domain of curve

function S = pbspline(P, k, N)
    
    if ~exist('N', 'var'), N = 100; end
    
    % For a closed spline check if 1st and last control points match.  If not
    % add another control point so that they do match
    if norm(P(:,1) - P(:,end)) > 0.01
        P = [P P(:,1)];
    end
    
    % Now add k - 1 control points that wrap over the first control points
    P = [P P(:,2:2+k-1)];
    
    [dim, np1] = size(P);
    n = np1-1;

    assert(k >= 2, 'Spline order must be 2 or greater');    
    assert(np1 >= k, 'No of control points must be >= k');
    assert(N >= 2, 'Spline must be evaluated at 2 or more points');
    
    % Form a uniform sequence. Number of knot points is m + 1 where m = n + k + 1
    ti = [0:(n+k+1)]/(n+k+1);
    nK = length(ti);

    % Domain of curve is [ti_k to ti_n] or [ti_(k+1) to ti_(n+1)] ???
    tstart = ti(k);
    tend = ti(n);

    dt = (tend-tstart)/(N-1);
    t = tstart:dt:tend;
    
    % Build complete array of basis functions
    N = cell(nK, k);
    
    % 1st level of recursive construction
    for i = 1:nK-1
       N{i,1} = t >= ti(i) & t < ti(i+1)  & ti(i) < ti(i+1); 
    end

    % Subsequent levels of recursive basis construction.  Note the logic to
    % handle repeated knot values where ti(i) == ti(i+1)
    for ki = 2:k    
        for i = 1:nK-ki

            if (ti(i+ki-1) - ti(i)) < eps
                V1 = 0;
            else
                V1 = (t - ti(i))/(ti(i+ki-1) - ti(i)) .* N{i,ki-1};
            end
            
            if (ti(i+ki) - ti(i+1)) < eps
                V2 = 0;
            else
                V2 = (ti(i+ki) - t)/(ti(i+ki) - ti(i+1)) .* N{i+1,ki-1};
            end
            
            N{i,ki} = V1 + V2;

%           This is the ideal equation that the code above implements            
%            N{i,ki} = (t - ti(i))/(ti(i+ki-1) - ti(i)) .* N{i,ki-1} + ...
%                      (ti(i+ki) - t)/(ti(i+ki) - ti(i+1)) .* N{i+1,ki-1};
        end
    end
    
    % Apply basis functions to the control points
    S = zeros(dim, length(t));
    for d = 1:dim
        for i = 1:np1
            S(d,:) = S(d,:) + P(d,i)*N{i,k};
        end
    end
    
   % Finally, because of the knot arrangements, the start of the spline may not
   % be close to the first control point if the spline order is 3 or greater.
   % Normally for a closed spline this is irrelevant.  However for our purpose
   % of using closed bplines to form paths in a colourspace this is important to
   % us.  The simple brute force solution used here is to search through the
   % spline points for the point that is closest to the 1st control point and
   % then rotate the spline points accordingly
   
   distsqrd = 0;
   for d = 1:dim
       distsqrd = distsqrd + (S(d,:) - P(d,1)).^2;
   end
   
   [~,ind] = min(distsqrd);
   
   S = circshift(S, [0, -ind+1]);