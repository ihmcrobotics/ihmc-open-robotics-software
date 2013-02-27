function visualize(rG, rs, Rs, dMin, dMax, H, F, cops)

fScaling = 0.001;

nFeet = size(rs, 2);
vectorLength = 3;

figure();
hold on;
plot3(rG(1), rG(2), rG(3), 'k*');


for i = 1 : nFeet
    RStartIndex = (i - 1) * vectorLength + 1;
    REndIndex = RStartIndex + vectorLength - 1;
    R = Rs(:, RStartIndex : REndIndex);
    
    r = rs(:, i);
    
    dMini = dMin(:, i);
    dMaxi = dMax(:, i);
    
    h = H(i);

    cornerPoints(:, 1) = R * [dMini(1); dMini(2); -h] + r;
    cornerPoints(:, 2) = R * [dMaxi(1); dMini(2); -h] + r;
    cornerPoints(:, 3) = R * [dMaxi(1); dMaxi(2); -h] + r;
    cornerPoints(:, 4) = R * [dMini(1); dMaxi(2); -h] + r;
    
    patch(cornerPoints(1, :), cornerPoints(2, :), cornerPoints(3, :), 'b');
    
    cop = cops(:, i);
    f = F(:, i);
    
    fStart = cop;
    fEnd = fStart + fScaling * f;    
    mArrow3(fStart, fEnd, 'color', 'red', 'stemWidth',0.004);

%     fLineInfo = [fStart, fEnd];
%     line(fLineInfo(1, :), fLineInfo(2, :), fLineInfo(3, :), 'Color','r','LineWidth', 1, 'Marker', 'o');
end

hold off;
axis image;
view(3);
grid on;

end