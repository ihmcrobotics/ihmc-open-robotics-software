function popnew = computenewpopulation(pop, bestmem, params)
%COMPUTENEWPOPULATION  Compute competing population of parameter vectors.
%		POPNEW = COMPUTENEWPOPULATION(POP, BESTMEM, PARAMS) computes a
%		competing population POPNEW for the current population POP with its
%		best member BESTMEM. POP contains parameter vectors in rows, BESTMEM
%		has to be a row vector. The structure PARAMS has to contain at least
%		the field 'algorithm' which selects the algorithm to use. At this time,
%		only the Differential Evolution algorithm is implemented. Extend this
%		function if you want to use your own favorite evolutionary algorithm.
%
%		This function is based on the differential evolution (DE) algorithm of
%		Rainer Storn (http://www.icsi.berkeley.edu/~storn/code.html). The core
%		evolutional algorithm was taken from function devec3.m.
%
%		<a href="differentialevolution.html">differentialevolution.html</a>  <a href="http://www.mathworks.com/matlabcentral/fileexchange/authors/18593">File Exchange</a>  <a href="https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=KAECWD2H7EJFN">Donate via PayPal</a>
%
%		Markus Buehren
%		Last modified 05.07.2011
%
%		See also DIFFERENTIALEVOLUTION.

switch params.algorithm
	case 'DE'

		% get dimensions
		[NP, D] = size(pop);
		
		% get parameters
		CR       = params.CR;
		F        = params.F;
		strategy = params.strategy;
		
		% check parameters
		if ((CR < 0) || (CR > 1))
			error('Crossover probability CR must be from interval [0,1].');
		end
		if ((F < 0) || (F > 2))
			error('Stepsize F must be from interval [0,2].');
		end

		% the following code was taken from the DE website (function devec3.m):
		% http://www.icsi.berkeley.edu/~storn/code.html

		% popold is the population which has to compete. It is static
		% through one iteration. pop is the newly emerging population.

		% note: parameter vectors are saved in pop as rows, in allmem as columns!

		popold = pop;                   % save the old population

		ind = randperm(4);              % index pointer array

		rot  = 0:1:NP-1;                % rotation index array (size NP)
		a1  = randperm(NP);             % shuffle locations of vectors
		rt = rem(rot+ind(1),NP);        % rotate indices by ind(1) positions
		a2  = a1(rt+1);                 % rotate vector locations
		rt = rem(rot+ind(2),NP);
		a3  = a2(rt+1);
		rt = rem(rot+ind(3),NP);
		a4  = a3(rt+1);
		rt = rem(rot+ind(4),NP);
		a5  = a4(rt+1);

		pm1 = popold(a1,:);             % shuffled population 1
		pm2 = popold(a2,:);             % shuffled population 2
		pm3 = popold(a3,:);             % shuffled population 3
		pm4 = popold(a4,:);             % shuffled population 4
		pm5 = popold(a5,:);             % shuffled population 5

		bm = bestmem(ones(NP, 1), :);   % population filled with the best 
		                                % member of the last iteration

		mui = double(rand(NP,D) < CR);  % all random numbers < CR are 1, 0 otherwise

		rotd = 0:1:D-1;                 % rotation index array (size D)
		if (strategy > 5)
			st = strategy-5;		          % binomial crossover
		else
			st = strategy;		            % exponential crossover
			mui = sort(mui, 2)';          % transpose, collect 1's in each column
			for i=1:NP
				n = floor(rand*D);
				if n > 0
					rtd = rem(rotd+n,D);
					mui(:,i) = mui(rtd+1,i);  % rotate column i by n
				end
			end
			mui = mui';			              % transpose back
		end
		mpo = mui < 0.5;                % inverse mask to mui

		if (st == 1)                            % DE/best/1
			ui = bm + F*(pm1 - pm2);              % differential variation
		elseif (st == 2)                        % DE/rand/1
			ui = pm3 + F*(pm1 - pm2);             % differential variation
		elseif (st == 3)                        % DE/rand-to-best/1
			ui = popold + F*(bm-popold) + F*(pm1 - pm2);
		elseif (st == 4)                        % DE/best/2
			ui = bm + F*(pm1 - pm2 + pm3 - pm4);  % differential variation
		elseif (st == 5)                        % DE/rand/2
			ui = pm5 + F*(pm1 - pm2 + pm3 - pm4); % differential variation
		else
			ui = zeros(NP,D);
		end
		
		popnew = popold.*mpo + ui.*mui;         % crossover

	otherwise
		error('Optimization method %s unknown.', params.optimizationMethod);
end
