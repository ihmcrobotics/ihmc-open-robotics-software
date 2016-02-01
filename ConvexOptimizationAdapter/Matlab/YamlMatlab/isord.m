function result = isord(obj)
    result = ~iscell(obj) && any(size(obj) > 1);
end