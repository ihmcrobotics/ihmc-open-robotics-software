clear java
% generate 1.6 classes
%!javac  ../../src/us/ihmc/commonWalkingControlModules/controlModules/nativeOptimization/{MomentumOptimizerInterface.java,ActiveSetQPMomentumOptimizer.java}  -cp ../../../ThirdParty/ThirdPartyJars/EJML/EJML.jar:../../../IHMCUtilities/classes/ -d ../../classes
javaaddpath ../../classes/
javaaddpath ../../../ThirdParty/ThirdPartyJars/EJML/EJML.jar
javaaddpath ../../../IHMCUtilities/classes/
javaaddpath (pwd)
%%
import java.io.*
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.*

problemDump = ['../../../ValkyrieHardwareDrivers/'  'ActiveSetQPMomentumOptimizer_diverence1398849788667735000'];
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
%clear java
save 'snapshot'
%%     X = fmincon(FUN,X0,A,B,Aeq,Beq,LB,UB) defines a set of lower and upper
x0 = [prevVd;prevRho];
lb = [-inf(size(prevVd)); rhoMin];
%xopt = fmincon(@(x) x'*Q*x+x'*f, x0,Ain, bin, Aeq, beq);
xopt = fmincon(@(x) x'*Q*x+x'*f, x0,[], [], Aeq, beq, lb,[]);
vdopt=xopt(1:nDoF);
rhoopt=xopt(nDoF+1:end);
bar([x0 xopt])
legend('x0','x*')