function y = quantile2(x, p, dim)
%QUANTILE2  Compute quantiles of a data sample.
%		Y = QUANTILE(X,P) returns quantiles of the values in X. P is a scalar
%		or a vector of cumulative probability values. Y is the same size as P,
%		and Y(i) contains the P(i)-th quantile. For matrices, QUANTILE(X,P)
%		works on the columns of X and returns a vector or matrix of quantiles.
%
%		Y = QUANTILE(X,P,DIM) works on the dimension DIM. Only dimensions 1 and
%		2 are supported.
%
%		Note: This is a simple replacement for MATLAB-7-function QUANTILE of
%		the statistics toolbox.
%
%		Example:
%		numbers = rand(10000, 1);
%		q = quantile(numbers, 0.7); % result should be close to 0.7
%
%		Markus Buehren
%		Last modified 21.04.2008 
%
%		See also QUANTILE.

if ~exist('dim', 'var')
	dim = 1;
	if any(size(x) == 1)
		x = x(:);
	end
end
if dim == 2
	x = x';
elseif dim ~= 1
	error('Only dimensions 1 and 2 are supported in function %s.', mfilename);
end

if any(p < 0) || any(p > 1)
	error('Values of P must be between 0 and 1.');
end

x = sort(x,1);
[M, N] = size(x);

% with linear interpolation, vectorized
p = p(:) * (M-1) + 1;
index1 = floor(p);
index2 = ceil(p);
index3 = ones(1,N);
if isequal(index1, index2)
	y = x(index1,:);
else
	c = index2 - p;
	y = x(index1,:) .* c(:,index3) + x(index2,:) .* (1-c(:,index3));
end

if dim == 2
	y = y';
end
