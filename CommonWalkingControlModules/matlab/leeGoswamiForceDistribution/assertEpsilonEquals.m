function assertEpsilonEquals(messageString,vector1,expected,epsilon)
valid = true;
minEpsilon = epsilon;
assert(all(size(expected)==size(vector1)), [messageString,': sizes must be equal']);
for i = 1:size(expected,1)
    for j = 1:size(expected,2)
        err = abs(vector1(i,j)-expected(i,j));
        if err>=minEpsilon
            minEpsilon = err;
            valid = false;
        end
    end
end
assert(valid,[messageString,': error is %d'], minEpsilon);
end