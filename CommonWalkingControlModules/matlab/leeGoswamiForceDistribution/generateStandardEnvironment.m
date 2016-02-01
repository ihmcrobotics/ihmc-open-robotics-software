function env = generateStandardEnvironment(x,y)
if nargin == 0
    x=0.25;
    y=0.25;
end
env.rG = [x; y; 1.0];
env.m = 100;
env.g = [0; 0; -9.81];
end