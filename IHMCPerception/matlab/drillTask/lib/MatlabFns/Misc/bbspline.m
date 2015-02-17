% BBSPLINE - Basic B-spline
%
% Usage:  S = bbspline(P, k, N)
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
% See also: PBSPLINE

% ** have t as an argument.  If omitted it defaults to 0:0.01:1 ?

% PK Jan 2014


function S = bbspline(P, k, N)
    
    if ~exist('N', 'var'), N = 100; end
    
    [dim, np1] = size(P);
    n = np1-1;

    assert(k >= 2, 'Spline order must be 2 or greater');    
    assert(np1 >= k, 'No of control points must be >= k');
    assert(N >= 2, 'Spline must be evaluated at 2 or more points');

    % Knot vector for a periodic spline (does not seem to be quite right)
%    ti = 1:(k+n+1);  % Periodic uniform knot vector

    % Set up open uniform knot vector from 0 - 1.  
    % There are k repeated knots at each end.
    ti = 0:(k+n - 2*(k-1));    
    ti = ti/ti(end);
    ti = [repmat(ti(1), 1, k-1), ti, repmat(ti(end), 1, k-1)];
 
    nK = length(ti);
    
    % Generate values of t that the spline will be evaluated at
    dt = (ti(end)-ti(1))/(N-1);
    t = ti(1):dt:ti(end);
    
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
    
    % Set the last point of the spline. This is not evaluated by the code above
    % because the basis functions are defined from ti(i) <= t < ti(i+1)
    S(:,end) = P(:,end);