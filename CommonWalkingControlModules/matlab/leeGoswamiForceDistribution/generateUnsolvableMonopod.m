function [feet,env,control]=generateUnsolvableMonopod(footTiltTheta,wk,epsilonf,y, mu)
params=generateSimpleFootParams();
feet = generateStandardSlipperyMonopod(params,footTiltTheta, mu);
env = generateStandardEnvironment(0,y);
control = generateStationKeepingControl(wk,epsilonf);
end