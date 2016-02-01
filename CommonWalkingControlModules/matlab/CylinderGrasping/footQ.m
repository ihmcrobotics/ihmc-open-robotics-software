function localQ = footQ()
mu=0.3;
f=0.3;
b=0.15;
s=0.1;
supportVectors = [unit([mu,0,1]'),unit([-mu*1/2,-mu*sqrt(3)/2,1]'),unit([-mu*1/2,mu*sqrt(3)/2,1]')];
contactPoints = [[f,s,0]',[f,-s,0]',[-b,-s,0]',[-b,s,0]'];
localQ = zeros(6,12);
for p=1:4
    for v=1:3
        localQ(:,(p-1)*3+v)=[supportVectors(:,v);cross(supportVectors(:,v),contactPoints(:,p))];
    end
end