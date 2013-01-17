function feet=generateTwanOriginalTriped(params)

%% individual feet
feet(1).r = [0; 0; 0;];
feet(1).R = rotX(-0.2) * rotY(0.1);
feet(1).mu =0.6;
feet(1).tauMax = 10;
feet(1).params = params;
    
feet(2).r = [0.5; 0.5; 0.1];
feet(2).R =  rotX(0.1) * rotY(-0.3);
feet(2).mu = 0.6;
feet(2).tauMax = 10;
feet(2).params = params;

feet(3).r = [0.25; 0.5; 0.8];
feet(3).R = rotX(0.3) * rotY(-0.2);
feet(3).mu = 0.4;
feet(3).tauMax = 3;
feet(3).params = params;

end