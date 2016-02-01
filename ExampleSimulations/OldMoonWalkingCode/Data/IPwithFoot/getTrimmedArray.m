function array = getTrimmedArray(x)

numberOfZeros = 4;
endIndex = length(x);
for i=1:endIndex
    if i<=(endIndex-numberOfZeros)
        if x(i+1) == 0.0
            if x(i+2) == 0.0
                array(i) = x(i);
                break;
            end
        end       
    end
    
    array(i) = x(i);
end
