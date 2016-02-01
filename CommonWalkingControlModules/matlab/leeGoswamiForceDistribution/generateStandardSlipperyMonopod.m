function feet = generateStandardSlipperyMonopod(params,footTiltTheta, mu)
feet(1).r = [0; 0; 0;];
feet(1).R = rotX(-footTiltTheta) * rotY(0.0);
feet(1).mu =mu;
feet(1).tauMax = 0.1;
feet(1).params = params;
end