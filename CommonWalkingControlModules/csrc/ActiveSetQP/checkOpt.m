function []=checkOpt(x, Q,f, Aeq, beq, Ain, bin)
    disp('----------------------')
    eq=norm(Aeq*x-beq,1)
    ineq=all(Ain*x <=bin)
    obj=x'*Q*x+2*f'*x