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
%% big graph
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
title('Compare performance at zero foot angle, low mu, monopod, with varying wk');
xlabel('log of weighting on rotational momenta term relative to linear momenta term');
ylabel('log of deviation from desired value');
legend({'linear momentum deviation','angular momentum deviation'});

%% Run a test which is capable of making ld but which will fail to due to
%% control weighting being very high for kd
[feet,env,control]=generateUnsolvableMonopod(0.0,10,0.001, .25);
result = runForceDistributionProblemSolver(feet,env,control);
assertNotEquals('result should failt to make kd tolerance',result.kdError,[0;0;0],1e-5);
assertEpsilonEquals('result should make ld tolerance',result.ldError,[0;0;0],1e-2);

%% Run a test which should fail to meet in only ld 
[feet,env,control]=generateUnsolvableMonopod(1.2);
result = runForceDistributionProblemSolver(feet,env,control);
assertEpsilonEquals('result failed to make kd tolerance',result.kdError,[0;0;0],1e1);
assertNotEquals('result should failt to make ld tolerance',result.ldError,[0;0;0],1e1);

%% Run a test which should make it impossible to maintain the force line
%% through the ankle
[feet,env,control]=generateImpossibleTiltMonopod(pi/2,pi/2);
result = runForceDistributionProblemSolver(feet,env,control);
assertNotEquals('result should failt to make kd tolerance',result.kdError,[0;0;0],1e3);
assertNotEquals('result should failt to make ld tolerance',result.ldError,[0;0;0],1e-1);