function yawPitchRoll = getYawPitchRoll(R)
yawPitchRoll = [getYaw(R); getPitch(R); getRoll(R)];

end