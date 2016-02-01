function assertNotEquals(messageString,vector1,expected,epsilon)
valid = true;
assert(all(size(expected)==size(vector1)), [messageString,': sizes must be equal']);
for i = 1:size(expected,1)
    for j = 1:size(expected,2)
        valid = valid && (abs(vector1(i,j)-expected(i,j))<=epsilon);
    end
end
assert(~valid,messageString);
end