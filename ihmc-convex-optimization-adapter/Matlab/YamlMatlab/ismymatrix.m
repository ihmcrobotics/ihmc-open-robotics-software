function result = ismymatrix(obj)
    result = ndims(obj) == 2 && all(size(obj) > 1);
end