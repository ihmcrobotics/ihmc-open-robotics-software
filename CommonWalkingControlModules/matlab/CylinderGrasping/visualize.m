function visualize( result )

fScaling = 0.001;
mScaling = 0.001;

figure();
hold on;
plot3(0,0,0, 'k*');
displayWrench = result.unintentionalCOMForce;
point = [0,0,0]';
mArrow3(point, (point+displayWrench(1:3)*fScaling), 'color', 'red', 'stemWidth',0.004);
mArrow3(point, (point+displayWrench(4:6)*mScaling), 'color', 'blue', 'stemWidth',0.004);


for i = 1 : length(result.feet)

    wrenchAboutOrigin = result.feet(i).ReactionWrench;
    point = result.feet(i).localPosition;
    wrenchAboutPoint = wrenchTransform(eye(3),-point)*wrenchAboutOrigin;
  
    mArrow3(point, (point+wrenchAboutPoint(1:3)*fScaling), 'color', 'red', 'stemWidth',0.004);
    mArrow3(point, (point+wrenchAboutPoint(4:6)*mScaling), 'color', 'blue', 'stemWidth',0.004);
    if result.feet(i).isPlane
        plot3(point(1),point(2),point(3), 'g*');
    else
        plot3(point(1),point(2),point(3), 'c*');
    end

end

hold off;
axis image;
view(3);
grid on;

end