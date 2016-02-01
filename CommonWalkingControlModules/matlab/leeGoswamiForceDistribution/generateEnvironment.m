function env = generateEnvironment(thetaX,thetaY)
cosY=cos(thetaY);
sinY=sin(thetaY);
cosX=cos(thetaX);
sinX=sin(thetaX);
env.rG = [sinX; sinY; cosX*cosY];
env.m = 100;
env.g = [0; 0; -9.81];
end