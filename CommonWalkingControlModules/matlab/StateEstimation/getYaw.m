function yaw = getYaw(R)
yaw = atan2(R(2, 1), R(1, 1));
end