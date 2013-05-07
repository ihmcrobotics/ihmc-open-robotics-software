function result=runForceDistributionProblemSolver(feet,env,control, plot)
if nargin < 4
    plot = true;
end

%% combine foot information for solving
Q = zeros (6,50);
rhoMin = zeros (50,1);
rhoMax = zeros (50,1);
j = 1;
for i=1:length(feet)
    if (feet(i).isPlane)
        Q(:,j:j+11)=feet(i).localTransform*feet(i).localQ;
        rhoMin(j:j+11,1)=feet(i).rhoMin;
        rhoMax(j:j+11,1)=feet(i).rhoMax;
        j=j+12;
    else
        Q(:,j:j+12)=feet(i).localTransform*feet(i).localQ;
        rhoMin(j:j+12,1)=feet(i).rhoMin;
        rhoMax(j:j+12,1)=feet(i).rhoMax;
        j=j+13;
    end
end

%% unpack control
c = control.desiredWrench - env.externalWrench;
C = diag(control.comWrenchWeightings);
wRho = control.wRho;

%% Run the solver.
[rho]=runForceOpt(C,Q,c,rhoMin, rhoMax,wRho);
%% Pack the result
result.feet=feet;
netCOMWrench = Q * rho;
result.unintentionalCOMForce = netCOMWrench+env.externalWrench-control.desiredWrench;
result.netCOMWrench = netCOMWrench;
j=1;
for i=1:length(result.feet)
    
    if (feet(i).isPlane)
        result.feet(i).ReactionWrench=result.feet(i).localTransform * result.feet(i).localQ*rho(j:j+11,1);
        j=j+12;
    else
        result.feet(i).ReactionWrench=result.feet(i).localTransform * result.feet(i).localQ*rho(j:j+12,1);
        j=j+13;
    end
end

if plot
    visualize( result );
end

end