function ret = throwAwayRepeated(in)

j = 1;
for i = 2 : length(in)
    if (in(i) ~= in(i - 1))
        ret(j) = in(i - 1);
        j = j + 1;
    end
end
end