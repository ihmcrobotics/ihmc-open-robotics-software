%% Solve GRF magnitude and COP placement problem for multiple feet

%% Run a test that we know works.
params=generateOriginalTwanFootParams();
feet = generateTwanOriginalTriped(params);
env = generateStandardEnvironment();
control = generateStandardControl();

result = runForceDistributionProblemSolver(feet,env,control);
assertEpsilonEquals('result should make kd tolerance',result.kdError,[0;0;0],1e-5);
assertEpsilonEquals('result should make ld tolerance',result.ldError,[0;0;0],1e-2);

%% Setup test to be reused, plot example of physical setup
wks = logspace(-1,-1,1);

kdEpsilons=[];
ldEpsilons=[];
for wk = wks
    [kdEps,ldEpsr]=runMonopodTest(0.0,wk,1e-5,true);
    kdEpsilons=[kdEpsilons,kdEps];
    ldEpsilons=[ldEpsilons,ldEpsr];
end

%% Compare performance at zero foot angle, low mu, monopod, with varying wk
wks = logspace(-5,5,15);
figure();
kdEpsilons=[];
ldEpsilons=[];
for wk = wks
    [kdEps,ldEpsr]=runMonopodTest(0.0,wk,1e-5);
    kdEpsilons=[kdEpsilons,kdEps];
    ldEpsilons=[ldEpsilons,ldEpsr];
end
loglog(wks',[ldEpsilons',kdEpsilons']);
title('Compare performance at zero foot angle, low mu, monopod, with varying wk');
xlabel('weighting on rotational momenta term relative to linear momenta term');
ylabel('simplistic deviation from desired value');
legend({'linear momentum deviation','angular momentum deviation'});

%% Compare performance at various foot angles angle, low mu, monopod, with
%% varying wk
%% simple models representing foot angles
footAngles = linspace(0,atan(.25),3);  
for footAngle = footAngles
    runMonopodTest(footAngle,.1,1e-5,true);
end
%% big unsolvable monopod graph
wks = logspace(-2,2,10);
footAngles = linspace(0,atan(.25),12);
figure();   hold on; 
for footAngle = footAngles
    kdEpsilons=[];
    ldEpsilons=[];
    for wk = wks
        [kdEps,ldEpsr]=runMonopodTest(footAngle,wk,1e-5);
        kdEpsilons=[kdEpsilons,log(kdEps)];
        ldEpsilons=[ldEpsilons,log(ldEpsr)];
    end
    
    plot(log(wks'),[ldEpsilons',kdEpsilons']);
end
title('Compare performance at zero foot angle, low mu=0.04, monopod, with varying wk');
xlabel('log of weighting on rotational momenta term relative to linear momenta term');
ylabel('log of deviation from desired value');
legend({'linear momentum deviation','angular momentum deviation'});
%% Try a higher Mu
mu=0.2;
%% simple models representing foot angles
footAngles = linspace(0,atan(.25),3);  
for footAngle = footAngles
    runMonopodTest(footAngle,.1,1e-5,true,0.25,mu);
end
%% big unsolvable monopod graph with higher mu
wks = logspace(-5,2,10);
footAngles = linspace(0,atan(.25)/3,6);
figure();   hold on; 
for footAngle = footAngles
    kdEpsilons=[];
    ldEpsilons=[];
    for wk = wks
        [kdEps,ldEpsr]=runMonopodTest(footAngle,wk,1e-5,false,0.25,mu);
        kdEpsilons=[kdEpsilons,log(kdEps)];
        ldEpsilons=[ldEpsilons,log(ldEpsr)];
    end
    
    plot(log(wks'),[ldEpsilons',kdEpsilons']);
end
title('Compare performance at zero foot angle, mu = 0.2, monopod, with varying wk');
xlabel('log of weighting on rotational momenta term relative to linear momenta term');
ylabel('log of deviation from desired value');
legend({'linear momentum deviation','angular momentum deviation'});

%% Unlike the last test, this one should be totally solvable
%% simple models representing foot angles
ydis = 0.0;
n=3;
footAngles = linspace(0,pi/2,n+1); 
footAngles = footAngles(1:n);
mus = tan(footAngles)*1.05;
for i=1:n
    runMonopodTest(footAngles(i),.1,1e-5,true,ydis,mus(i));
end
%% big solvable monopod graph
ydis = 0.0;
n=8;
m=4;
footAngles = linspace(0,pi/2,n+1); 
footAngles = footAngles(1:n);
mus = tan(footAngles)*1.05;
wks = logspace(-2,2,m);
figure();   hold on; 
for i=1:n
    kdEpsilons=[];
    ldEpsilons=[];
    for wk = wks
        [kdEps,ldEpsr]=runMonopodTest(footAngles(i),wk,1e-5,false,ydis,mus(i));
        kdEpsilons=[kdEpsilons,log(kdEps)];
        ldEpsilons=[ldEpsilons,log(ldEpsr)];
    end
    
    plot(log(wks'),[ldEpsilons',kdEpsilons']);[kdEps,ldEpsr]=runMonopodTest(footAngles(i),wk,1e-5,false,ydis,mus(i));
end
title('Compare performance at various foot angles with appropriate mu to fully be solvable and varying wk');
xlabel('log of weighting on rotational momenta term relative to linear momenta term');
ylabel('log of deviation from desired value');
legend({'linear momentum deviation','angular momentum deviation'});

%% Run a test which balances the robot on three line feet
mu = 0.1;
tiltAngle = -0.2;
wk= 0.1;
epsilonf = 0.001;
control = generateStationKeepingControl(wk,epsilonf);
testLineFeetTripod(mu,tiltAngle,control);

% additionally, try to move the com
control.ld = [70,70,0.0]';
control.kd = [0,0,25]';
result = testLineFeetTripod(mu,tiltAngle,control);
assertEpsilonEquals('result should make kd tolerance',result.kdEps,0,1e-5);
assertEpsilonEquals('result should make ld tolerance',result.ldEps,0,1e-2);

