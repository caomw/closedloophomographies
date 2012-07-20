function [x fval] = callObjConstr(x0, options)

if nargin < 2
    options = optimset('Algorithm', 'sqp');
end

[x fval] = fmincon(@objectiveh16istruth, x0, [], [], [], [], [], [], @lineareq, options); %FIX THIS TO BE MORE ACCURATE

function f = objectiveh16istruth(xin)
h16 = importXMLtoMATLAB('testset9test2/homografia_new0006.xml');
x12 = [xin(1), xin(2), xin(3); xin(4), xin(5), xin(6); xin(7), xin(8), 1];
x23 = [xin(9), xin(10), xin(11); xin(12), xin(13), xin(14); xin(15), xin(16), 1];
x34 = [xin(17), xin(18), xin(19); xin(20), xin(21), xin(22); xin(23), xin(24), 1];
x45 = [xin(25), xin(26), xin(27); xin(28), xin(29), xin(30); xin(31), xin(32), 1];
x56 = [xin(33), xin(34), xin(35); xin(36), xin(37), xin(38); xin(39), xin(40), 1];
xcum = x12*x23*x34*x34*x45*x56;
f = abs(xcum(1, 1) - h16(1, 1)) + abs(xcum(1, 2) - h16(1, 2)) + ...
    abs(xcum(1, 3) - h16(1, 3)) + abs(xcum(2, 1) - h16(2, 1)) + ...
    abs(xcum(2, 2) - h16(2, 2)) + abs(xcum(2, 3) - h16(2, 3)) + ...
    abs(xcum(3, 1) - h16(3, 1)) + abs(xcum(3, 2) - h16(3, 2)) + ...
    abs(xcum(3, 3) - h16(3, 3)); 

function [c, ceq] = lineareq(x)
ceq = x(1) + x(2) - 9;
c = [];