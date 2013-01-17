function [feet,env,control]=generateImpossibleTiltMonopod(xTilt,yTilt)
params=generateOriginalTwanFootParams();
feet = generateStandardSlipperyMonopod(params);
env = generateEnvironment(xTilt,yTilt);
control = generateStandardControl();
end