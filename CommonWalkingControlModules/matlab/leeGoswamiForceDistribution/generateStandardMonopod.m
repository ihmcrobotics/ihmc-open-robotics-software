function feet = generateStandardMonopod(params)
feet(1).r = [0; 0; 0;];
feet(1).R = rotX(-0.2) * rotY(0.1);
feet(1).mu =0.01;
feet(1).tauMax = 0.1;
feet(1).params = params;
end