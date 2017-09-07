% derivative with low noise
function [derivative] = derivativeWithLowNoise(time, signal)
outTime = 0;
derivative(1) = 0;
for i = 2:length(signal)
    if (signal(i) == signal(i-1)) % TODO add a tolerance as in the java class
        derivative(i,1) = derivative(i-1);
    else
        derivative(i,1) = (signal(i)-signal(i-1))/(time(i)-outTime); % maybe use also last change signal
        outTime = time(i);
    end
end
