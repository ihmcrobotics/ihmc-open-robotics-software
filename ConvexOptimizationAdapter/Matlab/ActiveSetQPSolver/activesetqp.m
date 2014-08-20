%solve
% min  0.5 x'G'x + c'x
%  st Aeq x = beq
%     Ain x <= bin
%
%

function test()
    seed=randi(100)
    %seed=6;
    suptitle(num2str(seed));
    rng(seed);
    gg=rand(2,2)
    p(1).G = gg*gg';
    p(1).f = [0;0];
    p(1).Aeq=zeros(0,2);
    p(1).beq=zeros(0,1);
    p(1).Ain=zeros(0,2);
    p(1).bin=zeros(0,1);
    p(1).x0=[1;2];
    p(1).xopt=[0;0];
    p(1).axis=[-4 4 -4 4];

    p(2)=p(1);
    p(2).Aeq=[1 0;0 1];
    p(2).beq=[1;2];
    p(2).x0=[];
    p(2).xopt=[1;2];
    
    p(3)=p(2);
    p(3).Aeq=[1 0];
    p(3).beq=1;
    p(3).Ain=[0 -1;-1 -1];
    p(3).bin=[-1;-3];
    p(3).x0=[];
    p(3).xopt=[1;2];
    
    p(4)=p(3);
    p(4).Aeq=zeros(0,2);
    p(4).beq=zeros(0);
    p(4).Ain=[0 -1;-1 -1];
    p(4).bin=[-1;-1];
    p(4).x0=[-1;3];
    p(4).xopt=[];
    
    p(5)=p(4);
    p(5).Ain(end+1,:)=[-1 -1];
    p(5).bin(end+1)=-1+1e-2;
    
    theta=rand()*pi*2;
    rotm = [cos(theta) -sin(theta);sin(theta) cos(theta)];
    p(6)=p(1);
    p(6).G=rotm'*[0 0;0 1]*rotm;
    p(6).x0=rand(2,1)*5;
    p(6).xopt=[];%rotm*[p(6).x0(1);0];
    
    p(7)=p(1);
    p(7).G=rotm'*[1 0;0 0]*rotm;
    p(7).x0=rand(2,1);
    p(7).xopt=[]; %rotm*[0;p(7).x0(2)];
    
    p(8)=p(5);
  
    RandStream.getGlobalStream()
    gg=rand(2,2);
    p(8).G=gg*gg';gg*gg'
    p(8).x0=[3;3];
    p(8).xopt=[];
    
    p(9).x0=[2;0];
    p(9).G=[2 0;0 2];
    p(9).f=[-2;-5];
    p(9).Aeq=zeros(0,2);
    p(9).beq=zeros(0);
    p(9).Ain=[  -1 +2;
                1 +2;
                1 -2;
                -1 0;
                0 -1];
    p(9).bin=[2;6;2;0;0];
    p(9).xopt=[];
    p(9).axis=[-1 5 -1 3];

    figure(1)
    for i=1:length(p)
        xopt_matlab=myqpsubStruct(p(i))

        subplot(3,3,i)
        cla
        hold on
        drawProblem(p(i));
        title_str=sprintf('problem %d',i);
        disp(title_str);
        title(title_str);
        
        %lagrainge multiplier
        [xopt his]=activesetqpStruct(p(i));
        sc=0.5;
        for h=1:length(his)
            for j=1:length(his(h).lambda)
                quiver(his(h).x(1), his(h).x(2),+his(h).Aw(j,1)*his(h).lambda(j)*sc, ...
                    his(h).Aw(j,2)*his(h).lambda(j)*sc,1,'r','Linewidth',3)
            end
            %gradient
            g=p(i).G*his(h).x+p(i).f;
            quiver(his(h).x(1), his(h).x(2),g(1)*sc,g(2)*sc,1,'g')
            if(h>1)
                plot([his(h-1).x(1);his(h).x(1)],[his(h-1).x(2);his(h).x(2)],'k-','Linewidth',3)
            end
            plot(his(h).x(1),his(h).x(2),'rx','Linewidth',3)
            text(his(h).x(1)+0.1,his(h).x(2),num2str(h))
        end
        
        if ~isempty(p(i).xopt)
            err=norm(xopt-p(i).xopt);
            assert(err<1e-5);
            
            err_matlab=norm(xopt-xopt_matlab);
            assert(err_matlab<1e-5)
        end
        hold off
        pause(0.5);
    end
   
end

function xopt=myqpsubStruct(p)
    ACTIND=[];
    if isempty(p.x0)
        x0=[10;10];
    else
        x0=p.x0;
    end
    xopt=myqpsub(p.G,p.f,[p.Aeq;p.Ain],[p.beq;p.bin],size(p.Aeq,1), x0,ACTIND);
    if ~isempty(p.xopt)
        err=xopt-p.xopt;
        assert(norm(err)<1e-10);
    end
end

function drawProblem(p)
    %equality
    ep=1e-10;
    for i=1:length(p.beq)
        fplot(@(x) (p.beq(i)-p.Aeq(i,1)*x)/(ep+p.Aeq(i,2)),p.axis(1:2),'k');
    end
    
    %inequality
    for i=1:length(p.bin)
        fplot(@(x) (p.bin(i)-p.Ain(i,1)*x)/(ep+p.Ain(i,2)),p.axis(1:2),'b-');
        fplot(@(x) (p.bin(i)-p.Ain(i,1)*x-0.05)/(ep+p.Ain(i,2)),p.axis(1:2),'b--');
    end

    n_sp=20;
    [x1 x2]=meshgrid(linspace(p.axis(1),p.axis(2),n_sp), linspace(p.axis(3),p.axis(4),n_sp));
    z=0.5*x1.^2*p.G(1,1) +0.5*x2.^2*p.G(2,2) +0.5*x1.*x2.*(p.G(1,2)+p.G(2,1))+p.f(1)*x1+p.f(2)*x2;
    contour(x1,x2,z,100);
    axis(p.axis,'equal');
end


function v=defval(G)
    eigG = eig(G+G');
    if any(abs(eigG)<eps)        
        %at least one zero eig
        if(all(eigG>-eps))
            v = 1; %PSD
        elseif all(eigG<eps)
            v=-1; %NSD
        else
            v=0; %ID
        end
    else
        %non zero eigs
        if all(eigG>0)
            v=2; %PD
        elseif all(eigG<0)
            v=-2; %ND
        else
            v=0; %ID
        end        
    end
end
function [x his]=activesetqpStruct(p)
    [x his]=activesetqp_impl(p.G, p.f, p.Aeq, p.beq, p.Ain, p.bin, p.x0);
end
function [x his]=activesetqp_impl(G,f,Aeq,beq, Ain, bin, x0)
    Gdef = defval(G);
    errnorm = 0.01*sqrt(eps); 

    if(Gdef<=0)
        error('G needs to be PSD')
    end
    
    %phase 1 problem if necessary
    if isempty(x0) 
        [x,~,lpExitFlag]=linprog([0 0]',Ain,bin, Aeq,beq);
        if(lpExitFlag<0)
            error('feasible region empty');
        else
            disp(sprintf('initial simplex solved x0'));
            disp(x);
        end
    else
        x=x0;
    end

    %ensure Aeq is full rank, and thus initial A is full rank
    [Aeq beq]=removeDependentConstraints(Aeq,beq); 
    neq = length(beq);
    nin = length(bin);
    ws=zeros(1,neq+nin);
    ws(1:neq)=1;
    
    A=[Aeq;Ain];
    b=[beq;bin];
    
    %find gradient direction
    
    %gradient = H*x+f;
    
    maxiter=100;
    iter=1;
    while iter < maxiter        
        %find stepDirection
        Aw =A(find(ws),:);bw=b(find(ws));
        [stepDirection lambda] = solveQPWithEqualityConstraint(G,f,Aw,bw,x,Gdef);
        assert(norm(Aw*(x+stepDirection)-bw,inf)<1e-10)
        
        %record
        his(iter).Aw = Aw;
        his(iter).lambda = lambda;
        his(iter).ws = ws;
        his(iter).x = x;
        
        if(iter>50)
            breakhere=1;
        end
        
        if norm(stepDirection,inf)<errnorm
            if all(lambda(neq+1:end)<=0)
                disp('optimum x found');
                break
            else
                %remove the free constraint
                [~,jIneq]=max(lambda(neq+1:end)); %maybe max, depending on direction
                ws(neq+jIneq)=0;                                
            end
        else
            %make step
            alpha=(b-A*x)./(A*stepDirection+eps);
            alpha(alpha<=0)=inf; 
            alpha(A*stepDirection <= errnorm*norm(stepDirection))=inf; %prevent colinear constraint
            
            [minAlpha jMin]=min(alpha); %always take the smaller one, first encounter
            if minAlpha<1
                disp(sprintf('take minimum step blocke by %d',jMin));
                ws(jMin)=1;    
                x=x+minAlpha*stepDirection;
            else
                disp('take full step')
                disp(stepDirection)
                x=x+stepDirection;
            end            
            disp('new x');
            disp(x);
        end

        %make step
        iter=iter+1;
    end
    
    if(iter>=maxiter)
        error('maximum iteration reached')
    end
end

function d=mydiag(x)
    if size(x,1)==1 || size(x,2)==1
        d=x(1);
    else
        d=diag(x);
    end
end

function [p lambda]= solveQPWithEqualityConstraint(G,f,A,b,x,Gdef)
    %page 455-457
   h=A*x-b;
   g=G*x+f;
   switch(Gdef)
       case 1 %PSD
           %null space approach
           %-Z*((Z'*H*Z)\(Z'*gradient));           
           threshold=1e-10;
           [Q, R, E]=qr(A');
           firstZeroIdx=find([abs(mydiag(R));0]<threshold,1);
           Z=Q(:,firstZeroIdx:end);
           Y=Q(:,1:firstZeroIdx-1);
           py=(A*Y)\-h;
           
           projG=Z'*G*Z;
           %pz=(Z'*G*Z)\(-Z'*G*Y*py-Z'*g); %can be done faster with cholesky decomposition
           [L,neg]=chol(projG);           
           if(neg==0) %PD
               pz=L\(L'\(-Z'*G*Y*py-Z'*g)); %Newton step
           else
               pz=-Z'*g; %Gradient step
           end
               
           p=Y*py+Z*pz;           
           lambda = (A*Y)'\(Y'*(g+G*p));  %lagrainge multiplier
           
       case 2 %GPD 
        %Shur complemenet approach
        lambda = (A/G*A')\(A/G*g-h);
        p=G\(A'*lambda-g);
       otherwise
           error('only support PSD G matrix')
   end
   if any(isnan(p))
       error('invalid step');
   end
end

function pd=ispd(A)
    pd=all(eig(A)>0);
end

function [A b]=removeDependentConstraints(A,b)
    nContraint=size(A,1);
    if nContraint>1
        [Q R permIdx]=qr(A','vector');
        threshold=1e-6;
        removeIdx=find([abs(mydiag(R));0]<threshold,1):nContraint;
        if ~isempty(removeIdx)kkk
            removeRows=permIdx(removeIdx);
            A(removeRows,:)=[];
            b(removeRows)=[];
            warning(['remove dependent constraints:' num2str(removeRows)]);
        end
    end
end



