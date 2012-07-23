function [x fval] = callObjConstr(x0, newHomo, detthresh, changethresh,...
    entry33thresh, smallsmallthresh, options)

if nargin < 7
    options = optimset('Algorithm', 'sqp');
end

[x fval] = fmincon(@objectiveh16istruth, x0, [], [], [], [], [], [], @nonlinearconst, options);

    function f = objectiveh16istruth(xin)
    x12 = [xin(1), xin(2), xin(3); xin(4), xin(5), xin(6); xin(7), xin(8), 1];
    x23 = [xin(9), xin(10), xin(11); xin(12), xin(13), xin(14); xin(15), xin(16), 1];
    x34 = [xin(17), xin(18), xin(19); xin(20), xin(21), xin(22); xin(23), xin(24), 1];
    x45 = [xin(25), xin(26), xin(27); xin(28), xin(29), xin(30); xin(31), xin(32), 1];
    x56 = [xin(33), xin(34), xin(35); xin(36), xin(37), xin(38); xin(39), xin(40), 1];
    xcum = x12*x23*x34*x34*x45*x56;
    f = abs(xcum(1, 1) - newHomo(1, 1)) + abs(xcum(1, 2) - newHomo(1, 2)) + ...
        abs(xcum(1, 3) - newHomo(1, 3)) + abs(xcum(2, 1) - newHomo(2, 1)) + ...
        abs(xcum(2, 2) - newHomo(2, 2)) + abs(xcum(2, 3) - newHomo(2, 3)) + ...
        abs(xcum(3, 1) - newHomo(3, 1)) + abs(xcum(3, 2) - newHomo(3, 2)) + ...
        abs(xcum(3, 3) - newHomo(3, 3)); 
    end

    function f = loop15(xin)
    x0102 = [xin(1), xin(2), xin(3); xin(4), xin(5), xin(6); xin(7), xin(8), 1];
    x0203 = [xin(9), xin(10), xin(11); xin(12), xin(13), xin(14); xin(15), xin(16), 1];
    x0304 = [xin(17), xin(18), xin(19); xin(20), xin(21), xin(22); xin(23), xin(24), 1];
    x0405 = [xin(25), xin(26), xin(27); xin(28), xin(29), xin(30); xin(31), xin(32), 1];
    x0506 = [xin(33), xin(34), xin(35); xin(36), xin(37), xin(38); xin(39), xin(40), 1];
    x0607 = [xin(41), xin(42), xin(43); xin(44), xin(45), xin(46); xin(47), xin(48), 1];
    x0708 = [xin(49), xin(50), xin(51); xin(52), xin(53), xin(54); xin(55), xin(56), 1];
    x0809 = [xin(57), xin(58), xin(59); xin(60), xin(61), xin(62); xin(63), xin(64), 1];
    x0910 = [xin(65), xin(66), xin(67); xin(68), xin(69), xin(70); xin(71), xin(72), 1];
    x1011 = [xin(73), xin(74), xin(75); xin(76), xin(77), xin(78); xin(79), xin(80), 1];
    x1112 = [xin(81), xin(82), xin(83); xin(84), xin(85), xin(86); xin(87), xin(88), 1];
    x1213 = [xin(89), xin(90), xin(91); xin(92), xin(93), xin(94); xin(95), xin(96), 1];
    x1314 = [xin(97), xin(98), xin(99); xin(100), xin(101), xin(102); xin(103), xin(104), 1];
    x1415 = [xin(105), xin(106), xin(107); xin(108), xin(109), xin(110); xin(111), xin(112), 1];
    x1516 = [xin(113), xin(114), xin(115); xin(116), xin(117), xin(118); xin(119), xin(120), 1];
    xcum = x0102*x0203*x0304*x0405*x0506*x0607*x0708*x0809*x0910*x1011*x1112*x1213*x1314*x1415*x1516;
    f = abs(xcum(1, 1) - newHomo(1, 1)) + abs(xcum(1, 2) - newHomo(1, 2)) + ...
        abs(xcum(1, 3) - newHomo(1, 3)) + abs(xcum(2, 1) - newHomo(2, 1)) + ...
        abs(xcum(2, 2) - newHomo(2, 2)) + abs(xcum(2, 3) - newHomo(2, 3)) + ...
        abs(xcum(3, 1) - newHomo(3, 1)) + abs(xcum(3, 2) - newHomo(3, 2)) + ...
        abs(xcum(3, 3) - newHomo(3, 3)); 
    end

    function [c, ceq] = nonlinearconst5homogs(x)
    %detthresh = 0.10;
    %changethresh = 0.04;
    %entry33thresh = 0.01;
    %smallsmallthresh = 0.0002;
    x12 = [x(1), x(2), x(3); x(4), x(5), x(6); x(7), x(8), 1];
    x23 = [x(9), x(10), x(11); x(12), x(13), x(14); x(15), x(16), 1];
    x34 = [x(17), x(18), x(19); x(20), x(21), x(22); x(23), x(24), 1];
    x45 = [x(25), x(26), x(27); x(28), x(29), x(30); x(31), x(32), 1];
    x56 = [x(33), x(34), x(35); x(36), x(37), x(38); x(39), x(40), 1];
    x13 = x12*x23;
    x14 = x13*x34;
    x15 = x14*x45;
    x16 = x15*x56;
    ceq = [];
    c(1) = abs(det(x12) - 1) - detthresh;
    c(2) = abs(det(x23) - 1) - detthresh;
    c(3) = abs(det(x34) - 1) - detthresh;
    c(4) = abs(det(x45) - 1) - detthresh;
    c(5) = abs(det(x56) - 1) - detthresh;
    c(6:7) = abs(x(1:2) - x0(1:2)) - changethresh;
    c(8:9) = abs(x(4:5) - x0(4:5)) - changethresh;
    c(10:11) = abs(x(9:10) - x0(9:10)) - changethresh;
    c(12:13) = abs(x(12:13) - x0(12:13)) - changethresh;
    c(14:15) = abs(x(17:18) - x0(17:18)) - changethresh;
    c(16:17) = abs(x(20:21) - x0(20:21)) - changethresh;
    c(18:19) = abs(x(25:26) - x0(25:26)) - changethresh;
    c(20:21) = abs(x(28:29) - x0(28:29)) - changethresh;
    c(22:23) = abs(x(33:34) - x0(33:34)) - changethresh;
    c(24:25) = abs(x(36:37) - x0(36:37)) - changethresh;
    c(26) = abs(x13(3, 3) - 1) - entry33thresh;
    c(27) = abs(x14(3, 3) - 1) - entry33thresh;
    c(28) = abs(x15(3, 3) - 1) - entry33thresh;
    c(29) = abs(x16(3, 3) - 1) - entry33thresh;
    c(30:31) = abs(x(7:8) - x0(7:8)) - smallsmallthresh;
    c(32:33) = abs(x(15:16) - x0(15:16)) - smallsmallthresh;
    c(34:35) = abs(x(23:24) - x0(23:24)) - smallsmallthresh;
    c(36:37) = abs(x(31:32) - x0(31:32)) - smallsmallthresh;
    c(38:39) = abs(x(39:40) - x0(39:40)) - smallsmallthresh;
    end

    function [c, ceq] = nonlinearconst15matrix(x)
        
    end
end