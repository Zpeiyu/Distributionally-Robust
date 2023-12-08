function [opt_X, opt_f,fval_out,firstorderopt_out] = Work4_OptEngine(parameters)

[A, b]= Work4_SetInequationC(parameters);


[Aeq, beq] = Work4_SetEquation(parameters);


Objective_func = @(X)Work4_Obj(X, parameters);


init_X = Work4_InitalSolution(parameters);                                                                                                                                   t_X = Work4_InitalSolution(parameters);


options = optimoptions('fmincon','Display','iter','Algorithm','sqp', 'MaxFunctionEvaluations',1e6,'OutputFcn', @OutFun);

global history
history.fval = [];
history.firstorderopt=[];

[opt_X, opt_f] =  fmincon(Objective_func,init_X,...
     A,b,Aeq,beq,[],[],[],options);
 
fval_out = history.fval; 
firstorderopt_out = history.firstorderopt;

end

