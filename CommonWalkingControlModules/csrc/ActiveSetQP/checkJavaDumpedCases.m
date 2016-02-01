clear java
% generate 1.6 classes
%!javac   ClassLoaderObjectInputStream.java
javaaddpath ../../classes/
javaaddpath ../../../ThirdParty/ThirdPartyJars/EJML/EJML.jar
javaaddpath ../../../IHMCUtilities/classes/
javaaddpath (pwd)
%%
import java.io.*
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.*

%problemDump = ['../../../Atlas/'  'ActiveSetQPMomentumOptimizer_diverence1399891506516816000'];
%problemDump = ['../../../Atlas/'  'ActiveSetQPMomentumOptimizer_diverence1399891506516816000'];
problemDump = '../../../ValkyrieHardwareDrivers/ActiveSetQPMomentumOptimizer_diverence1402652193717240000'
% problemDump = 'test';
fis = FileInputStream(problemDump);
% ois = ObjectInputStream(fis);
ois=ClassLoaderObjectInputStream(fis);
s = ois.readObject();
ois.close();
fis.close();
clear ois fis

%%
ejml2mat = @(x) reshape(x.data, x.getNumCols(), x.getNumRows())';
A = ejml2mat(s.A);
C = ejml2mat(s.C);
b = ejml2mat(s.b);
Js = ejml2mat(s.Js);
ps = ejml2mat(s.ps);
Ws = ejml2mat(s.Ws);
Lambda =ejml2mat(s.Lambda);
WRho = ejml2mat(s.WRho);
prevRho = ejml2mat(s.prevRho);
prevVd = ejml2mat(s.prevVd);
WRhoSmoother = ejml2mat(s.WRhoSmoother);
rhoPrevAvg = ejml2mat(s.rhoPrevMean);
WRhoCop = ejml2mat(s.WRhoCoPPenalty);
QRho = ejml2mat(s.QRho);
c = ejml2mat(s.c);

Jp = ejml2mat(s.Jp);
pp = ejml2mat(s.pp);

rhoMin = ejml2mat(s.rhoMin);

[nWrench nDoF] = size(A);
nRho = length(prevRho);
%% 0.5 x'Qx +f'x
Q = blkdiag(A'*C*A + Js'*Ws*Js + Lambda, WRho+WRhoSmoother + WRhoCop);
f = [ -A'*C*b - Js'*Ws*ps; -WRhoSmoother*prevRho - WRhoCop*rhoPrevAvg];
Aeq = [-A QRho; 
       Jp zeros(nDoF,nRho)];
beq = [c;pp];
Ain = [zeros(nRho,nDoF) -eye(nRho)];
bin = -rhoMin;
%     X = fmincon(FUN,X0,A,B,Aeq,Beq,LB,UB) defines a set of lower and upper
x0 = [prevVd;prevRho];
lb = [-inf(size(prevVd)); rhoMin];

Aeq(end-4:end,:)=[];
beq(end-4:end)=[];


%%
%xopt = fmincon(@(x) x'*Q*x+x'*f, x0,Ain, bin, Aeq, beq);
%Aeq=Aeq(1:37,:);
%beq=beq(1:37);x\

disp('#--------------------------------------------')
Q=(Q+Q')/2;
param = optimset('Algorithm','active-set');

%[xopt, ~, stat]= quadprog(Q,f,Ain, bin, Aeq, beq,[],[],x0,param);
A=[Aeq;Ain];b=[beq;bin];
[nc nv]=size(A);
addpath ~/matlablib/
ub=[10000*ones(nDoF,1); inf(nRho,1)];
lb=[-10000*ones(nDoF,1); -inf(nRho,1)];
xopt = myqpsub(Q,f,[Aeq;Ain], [beq;bin],lb,ub, x0, size(Aeq,1),3,'quadprog', nc, nv,param);
checkOpt(xopt, Q, f, Aeq, beq, Ain, bin)

%
Ain2= [eye(nDoF) zeros(nDoF,nRho); 
    -eye(nDoF) zeros(nDoF,nRho); 
    Ain];
bin2=[10000*ones(nDoF,1);10000*ones(nDoF,1);bin];
active=[];
[xopt2, act_opt, fail]=fastQP(Q,f,Aeq, beq, Ain2, bin2 ,active);
checkOpt(xopt2, Q, f, Aeq, beq, Ain, bin)

%% check constraints
subplot(1,3,[1 2])
imagesc(log(abs(Aeq)));colorbar
subplot(1,3,3);
imagesc(log(abs(beq)))