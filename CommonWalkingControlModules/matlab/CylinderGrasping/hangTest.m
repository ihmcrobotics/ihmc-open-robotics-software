function [feet,env,control] = hangTest()
leanForward=0.8;
feet(1).isPlane = true;
feet(1).localRotation = eye(3);
feet(1).localPosition = [leanForward,0.25,-1]';
feet(1).rhoMin = 0.01*ones(12,1);
feet(1).rhoMax = 100*ones(12,1);

feet(2).isPlane = true;
feet(2).localRotation = eye(3);
feet(2).localPosition = [leanForward,-0.25,-1]';
feet(2).rhoMin = 0.01*ones(12,1);
feet(2).rhoMax = 100*ones(12,1);

feet(3).isPlane = false;
feet(3).localRotation = rotX(pi/2);
feet(3).localPosition = [0,-0.5,0.5]';

feet(4).isPlane = false;
feet(4).localRotation = rotY(-pi/4)*rotX(pi/2);
feet(4).localPosition = [0,0.5,0.6]';

for i = 1:length(feet)
    if feet(i).isPlane
        feet(i).localQ=footQ();
        feet(i).rhoMin = .01*ones(12,1);
        feet(i).rhoMax = 300*ones(12,1);
    else
        feet(i).localQ=handQ();
        feet(i).rhoMin(1:5,1) =-200*[0.3,1,1,0.3*0.25,0.4]';
        feet(i).rhoMax(1:5,1) = 200*ones(5,1);
        feet(i).rhoMin(6:13) = .01*ones(8,1);
        feet(i).rhoMax(6:13) = 300*ones(8,1);
    end
    feet(i).localTransform = wrenchTransform(feet(i).localRotation,feet(i).localPosition);
end

env.com.localPosition = [0,0,0]';
env.externalWrench = [0,0,-9.81*70,0,0,0]';

control.comWrenchWeightings=[1,1,10,1,1,1];
control.desiredWrench = [10,0,0,0,0,0]';
control.wRho = 0.00001;

end