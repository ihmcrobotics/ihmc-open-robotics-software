function localQ=handQ()
mu=0.3;
halfHandWidth = .05;
cylinderRadius = .03;
localQ=zeros(6,13);
I=eye(6);
localQ(:,1:5)=I(:,1:5);
j=6;
for x=-1:2:1
    for xx=-1:2:1
        for zz=-1:2:1
            localQ(:,j)=unit([mu*x,-1,0,mu*cylinderRadius*xx,0,halfHandWidth*zz]);
            j=j+1;
        end
    end
end
end