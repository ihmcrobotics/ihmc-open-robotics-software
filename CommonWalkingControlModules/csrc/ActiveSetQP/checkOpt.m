function []=checkOpt(x, Q,f, Aeq, beq, Ain, bin)
    disp('----------------------')
    eq=norm(Aeq*x-beq,1)
    ineq=all(Ain*x <=bin+1e-8)
    obj=x'*Q*x+2*f'*x
    if(ineq==0)
        mostViolation=max(Ain*x - bin)
    end
