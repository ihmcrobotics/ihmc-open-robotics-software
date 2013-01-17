function [feet,env,control]=generateUnsolvableMonopod(footTiltTheta,wk,epsilonf,y)
params=generateOriginalTwanFootParams();
feet = generateStandardSlipperyMonopod(params,footTiltTheta);
env = generateStandardEnvironment(0,y);
control = generateStationKeepingControl(wk,epsilonf);
end