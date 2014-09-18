function y = foxholes(x, noPause)
%FOXHOLES  Evaluate Shekel's Foxholes function.
%		Y = FOXHOLES(X) with a two-element input vector X evaluates Shekel's
%		Foxholes function at X. The elements of X have to be from the interval
%		[-65.536, 65.536]. The global minimum of this function is about
%		0.998004 at X = [-32, -32].
%
%		FOXHOLES (without input arguments) displays the function as a contour
%		plot.
%
%		Markus Buehren
%		Last modified 03.02.2008 

switch nargin
	case {1, 2}
		% compute function value

		ai0 = [-32, -16, 0, 16, 32];
		a = [
			repmat(ai0, 1, 5);
			reshape(repmat(ai0, 5 , 1), 1, 25);
			];

		tmp = 0;
		for i = 1:24
			tmp2 = 0;
			for j = 1:2
				tmp2 = tmp2 + (x(j) - a(j,i)).^6;
			end
			tmp = tmp + 1 / (i + tmp2);
		end
		y = 1 / (0.002 + tmp);
		
		if nargin == 1 || ~noPause
			pause(0.05);
		end
		
	case 0
		% plot function
		x = -40:1:40;
		N = length(x);
		F = zeros(N);
		for m=1:N
			for n=1:N
				F(m,n) = foxholes([x(m); x(n)], 1);
			end
		end

		figure;
		contour(x,x,F,10);
		title('Contour plot of Shekel''s Foxholes');

	otherwise
		error('Wrong number of input arguments.');
end


