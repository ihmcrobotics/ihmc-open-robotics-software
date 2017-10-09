function [ X ] = expsmooth(X, alpha)
    % get number of column vector signals (M) and their lengths (N)
    [ N, M ] = size( X );

    % for each column vector signal
    for m = 1:M

        % for each sample
        for n = 2:N

            % apply the exponential smoother [1]
            X(n,m) = alpha*X(n-1,m) + (1-alpha)*X(n,m);
        end

    end