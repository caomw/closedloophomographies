function [x fval] = callObjConstr(x0, options)

if nargin < 2
    options = optimset('Algorithm', 'sqp');
end

[x fval] = fmincon(@objective, x0, [], [], [], [], [], [], @lineareq, options); %FIX THIS TO BE MORE ACCURATE

function f = objective(xin)
f = xin(1)^2 + xin(2)^2;

function [c, ceq] = lineareq(x)
ceq = x(1) + x(2) - 9;
c = [];