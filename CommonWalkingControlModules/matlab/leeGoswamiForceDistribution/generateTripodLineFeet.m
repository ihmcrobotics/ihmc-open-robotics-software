function feet = generateTripodLineFeet(footTilt,mu)
params = generateLineFootParams();
for i = 1:3
    angle = 2*pi/3*(i-1);
    feet(i).r = 0.35*[cos(angle); sin(angle); 0;];
    feet(i).R = rotZ(angle)*rotY(footTilt);
    feet(i).mu =mu;
    feet(i).tauMax = 0.1;
    feet(i).params = params;
end
end