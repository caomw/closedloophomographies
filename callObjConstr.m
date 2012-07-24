function [x fval] = callObjConstr(x0, newHomo, detthresh, changethresh,...
    entry33thresh, smallsmallthresh, transthreshx, transthreshy, options)

if nargin < 12
    options = optimset('Algorithm', 'sqp', 'MaxFunEvals', 20000);
end

%[x fval] = fmincon(@objectiveh16istruth, x0, [], [], [], [], [], [], @nonlinearconst, options);
%[x fval] = fmincon(@loop15, x0, [], [], [], [], [], [], @nonlinearconst15, options);
%[x fval] = fmincon(@loop15includingfirst, x0, [], [], [], [], [], [], @nonlinearconst15includingfirst, options);
[x fval] = fmincon(@loop15includingfirst, x0, [], [], [], [], [], [], @nonlinearconst15includingfirstandnosignchange, options);

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

    
function f = loop15includingfirst(xin)
x0001 = [xin(1), xin(2), xin(3); xin(4), xin(5), xin(6); xin(7), xin(8), 1];
x0102 = [xin(9), xin(10), xin(11); xin(12), xin(13), xin(14); xin(15), xin(16), 1];
x0203 = [xin(17), xin(18), xin(19); xin(20), xin(21), xin(22); xin(23), xin(24), 1];
x0304 = [xin(25), xin(26), xin(27); xin(28), xin(29), xin(30); xin(31), xin(32), 1];
x0405 = [xin(33), xin(34), xin(35); xin(36), xin(37), xin(38); xin(39), xin(40), 1];
x0506 = [xin(41), xin(42), xin(43); xin(44), xin(45), xin(46); xin(47), xin(48), 1];
x0607 = [xin(49), xin(50), xin(51); xin(52), xin(53), xin(54); xin(55), xin(56), 1];
x0708 = [xin(57), xin(58), xin(59); xin(60), xin(61), xin(62); xin(63), xin(64), 1];
x0809 = [xin(65), xin(66), xin(67); xin(68), xin(69), xin(70); xin(71), xin(72), 1];
x0910 = [xin(73), xin(74), xin(75); xin(76), xin(77), xin(78); xin(79), xin(80), 1];
x1011 = [xin(81), xin(82), xin(83); xin(84), xin(85), xin(86); xin(87), xin(88), 1];
x1112 = [xin(89), xin(90), xin(91); xin(92), xin(93), xin(94); xin(95), xin(96), 1];
x1213 = [xin(97), xin(98), xin(99); xin(100), xin(101), xin(102); xin(103), xin(104), 1];
x1314 = [xin(105), xin(106), xin(107); xin(108), xin(109), xin(110); xin(111), xin(112), 1];
x1415 = [xin(113), xin(114), xin(115); xin(116), xin(117), xin(118); xin(119), xin(120), 1];
x1516 = [xin(121), xin(122), xin(123); xin(124), xin(125), xin(126); xin(127), xin(128), 1];
xcum = x0001*x0102*x0203*x0304*x0405*x0506*x0607*x0708*x0809*x0910*x1011*x1112*x1213*x1314*x1415*x1516;
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
    c(40) = abs(x12(3, 3) - 1) - entry33thresh;
    end

    function [c, ceq] = nonlinearconst15(x)
         ceq = [];
         x0102 = [x(1), x(2), x(3); x(4), x(5), x(6); x(7), x(8), 1];
x0203 = [x(9), x(10), x(11); x(12), x(13), x(14); x(15), x(16), 1];
x0304 = [x(17), x(18), x(19); x(20), x(21), x(22); x(23), x(24), 1];
x0405 = [x(25), x(26), x(27); x(28), x(29), x(30); x(31), x(32), 1];
x0506 = [x(33), x(34), x(35); x(36), x(37), x(38); x(39), x(40), 1];
x0607 = [x(41), x(42), x(43); x(44), x(45), x(46); x(47), x(48), 1];
x0708 = [x(49), x(50), x(51); x(52), x(53), x(54); x(55), x(56), 1];
x0809 = [x(57), x(58), x(59); x(60), x(61), x(62); x(63), x(64), 1];
x0910 = [x(65), x(66), x(67); x(68), x(69), x(70); x(71), x(72), 1];
x1011 = [x(73), x(74), x(75); x(76), x(77), x(78); x(79), x(80), 1];
x1112 = [x(81), x(82), x(83); x(84), x(85), x(86); x(87), x(88), 1];
x1213 = [x(89), x(90), x(91); x(92), x(93), x(94); x(95), x(96), 1];
x1314 = [x(97), x(98), x(99); x(100), x(101), x(102); x(103), x(104), 1];
x1415 = [x(105), x(106), x(107); x(108), x(109), x(110); x(111), x(112), 1];
x1516 = [x(113), x(114), x(115); x(116), x(117), x(118); x(119), x(120), 1];
x0103 = x0102*x0203;
x0104 = x0103*x0304;
x0105 = x0104*x0405;
x0106 = x0105*x0506;
x0107 = x0106*x0607;
x0108 = x0107*x0708;
x0109 = x0108*x0809;
x0110 = x0109*x0910;
x0111 = x0110*x1011;
x0112 = x0111*x1112;
x0113 = x0112*x1213;
x0114 = x0113*x1314;
x0115 = x0114*x1415;
x0116 = x0115*x1516;
c(1) = abs(det(x0102) - 1) - detthresh;
c(2) = abs(det(x0203) - 1) - detthresh;
c(3) = abs(det(x0304) - 1) - detthresh;
c(4) = abs(det(x0405) - 1) - detthresh;
c(5) = abs(det(x0506) - 1) - detthresh;
c(6) = abs(det(x0607) - 1) - detthresh;
c(7) = abs(det(x0708) - 1) - detthresh;
c(8) = abs(det(x0809) - 1) - detthresh;
c(9) = abs(det(x0910) - 1) - detthresh;
c(10) = abs(det(x1011) - 1) - detthresh;
c(11) = abs(det(x1112) - 1) - detthresh;
c(12) = abs(det(x1213) - 1) - detthresh;
c(13) = abs(det(x1314) - 1) - detthresh;
c(14) = abs(det(x1415) - 1) - detthresh;
c(15) = abs(det(x1516) - 1) - detthresh;
c(16) = abs(x0102(3, 3) - 1) - entry33thresh;
c(17) = abs(x0103(3, 3) - 1) - entry33thresh;
c(18) = abs(x0104(3, 3) - 1) - entry33thresh;
c(19) = abs(x0105(3, 3) - 1) - entry33thresh;
c(20) = abs(x0106(3, 3) - 1) - entry33thresh;
c(21) = abs(x0107(3, 3) - 1) - entry33thresh;
c(22) = abs(x0108(3, 3) - 1) - entry33thresh;
c(23) = abs(x0109(3, 3) - 1) - entry33thresh;
c(24) = abs(x0110(3, 3) - 1) - entry33thresh;
c(25) = abs(x0111(3, 3) - 1) - entry33thresh;
c(26) = abs(x0112(3, 3) - 1) - entry33thresh;
c(27) = abs(x0113(3, 3) - 1) - entry33thresh;
c(28) = abs(x0114(3, 3) - 1) - entry33thresh;
c(29) = abs(x0115(3, 3) - 1) - entry33thresh;
c(30) = abs(x0116(3, 3) - 1) - entry33thresh;
c(31:32) = abs(x(1:2) - x0(1:2)) - changethresh;
c(33) = abs(x(3) - x0(3)) - transthresh;
c(34:35) = abs(x(4:5) - x0(4:5)) - changethresh;
c(36) = abs(x(6) - x0(6)) - transthresh;
c(37:38) = abs(x(7:8) - x0(7:8)) - smallsmallthresh;
c(39:40) = abs(x(9:10) - x0(9:10)) - changethresh;
c(41) = abs(x(11) - x0(11)) - transthresh;
c(42:43) = abs(x(12:13) - x0(12:13)) - changethresh;
c(44) = abs(x(14) - x0(14)) - transthresh;
c(45:46) = abs(x(15:16) - x0(15:16)) - smallsmallthresh;
c(47:48) = abs(x(17:18) - x0(17:18)) - changethresh;
c(49) = abs(x(19) - x0(19)) - transthresh;
c(50:51) = abs(x(20:21) - x0(20:21)) - changethresh;
c(52) = abs(x(22) - x0(22)) - transthresh;
c(53:54) = abs(x(23:24) - x0(23:24)) - smallsmallthresh;
c(55:56) = abs(x(25:26) - x0(25:26)) - changethresh;
c(57) = abs(x(27) - x0(27)) - transthresh;
c(58:59) = abs(x(28:29) - x0(28:29)) - changethresh;
c(60) = abs(x(30) - x0(30)) - transthresh;
c(61:62) = abs(x(31:32) - x0(31:32)) - smallsmallthresh;
c(63:64) = abs(x(33:34) - x0(33:34)) - changethresh;
c(65) = abs(x(35) - x0(35)) - transthresh;
c(66:67) = abs(x(36:37) - x0(36:37)) - changethresh;
c(68) = abs(x(38) - x0(38)) - transthresh;
c(69:70) = abs(x(39:40) - x0(39:40)) - smallsmallthresh;
c(71:72) = abs(x(41:42) - x0(41:42)) - changethresh;
c(73) = abs(x(43) - x0(43)) - transthresh;
c(74:75) = abs(x(44:45) - x0(44:45)) - changethresh;
c(76) = abs(x(46) - x0(46)) - transthresh;
c(77:78) = abs(x(47:48) - x0(47:48)) - smallsmallthresh;
c(79:80) = abs(x(49:50) - x0(49:50)) - changethresh;
c(81) = abs(x(51) - x0(51)) - transthresh;
c(82:83) = abs(x(52:53) - x0(52:53)) - changethresh;
c(84) = abs(x(54) - x0(54)) - transthresh;
c(85:86) = abs(x(55:56) - x0(55:56)) - smallsmallthresh;
c(87:88) = abs(x(57:58) - x0(57:58)) - changethresh;
c(89) = abs(x(59) - x0(59)) - transthresh;
c(90:91) = abs(x(60:61) - x0(60:61)) - changethresh;
c(92) = abs(x(62) - x0(62)) - transthresh;
c(93:94) = abs(x(63:64) - x0(63:64)) - smallsmallthresh;
c(95:96) = abs(x(65:66) - x0(65:66)) - changethresh;
c(97) = abs(x(67) - x0(67)) - transthresh;
c(98:99) = abs(x(68:69) - x0(68:69)) - changethresh;
c(100) = abs(x(70) - x0(70)) - transthresh;
c(101:102) = abs(x(71:72) - x0(71:72)) - smallsmallthresh;
c(103:104) = abs(x(73:74) - x0(73:74)) - changethresh;
c(105) = abs(x(75) - x0(75)) - transthresh;
c(106:107) = abs(x(76:77) - x0(76:77)) - changethresh;
c(108) = abs(x(78) - x0(78)) - transthresh;
c(109:110) = abs(x(79:80) - x0(79:80)) - smallsmallthresh;
c(111:112) = abs(x(81:82) - x0(81:82)) - changethresh;
c(113) = abs(x(83) - x0(83)) - transthresh;
c(114:115) = abs(x(84:85) - x0(84:85)) - changethresh;
c(116) = abs(x(86) - x0(86)) - transthresh;
c(117:118) = abs(x(87:88) - x0(87:88)) - smallsmallthresh;
c(119:120) = abs(x(89:90) - x0(89:90)) - changethresh;
c(121) = abs(x(91) - x0(91)) - transthresh;
c(122:123) = abs(x(92:93) - x0(92:93)) - changethresh;
c(124) = abs(x(94) - x0(94)) - transthresh;
c(125:126) = abs(x(95:96) - x0(95:96)) - smallsmallthresh;
c(127:128) = abs(x(97:98) - x0(97:98)) - changethresh;
c(129) = abs(x(99) - x0(99)) - transthresh;
c(130:131) = abs(x(100:101) - x0(100:101)) - changethresh;
c(132) = abs(x(102) - x0(102)) - transthresh;
c(133:134) = abs(x(103:104) - x0(103:104)) - smallsmallthresh;
c(135:136) = abs(x(105:106) - x0(105:106)) - changethresh;
c(137) = abs(x(107) - x0(107)) - transthresh;
c(138:139) = abs(x(108:109) - x0(108:109)) - changethresh;
c(140) = abs(x(110) - x0(110)) - transthresh;
c(141:142) = abs(x(111:112) - x0(111:112)) - smallsmallthresh;
c(143:144) = abs(x(113:114) - x0(113:114)) - changethresh;
c(145) = abs(x(115) - x0(115)) - transthresh;
c(146:147) = abs(x(116:117) - x0(116:117)) - changethresh;
c(148) = abs(x(118) - x0(118)) - transthresh;
c(149:150) = abs(x(119:120) - x0(119:120)) - smallsmallthresh;
    end

function [c, ceq] = nonlinearconst15includingfirst(x) 
 ceq = [];
x0001 = [x(1), x(2), x(3); x(4), x(5), x(6); x(7), x(8), 1];
x0102 = [x(9), x(10), x(11); x(12), x(13), x(14); x(15), x(16), 1];
x0203 = [x(17), x(18), x(19); x(20), x(21), x(22); x(23), x(24), 1];
x0304 = [x(25), x(26), x(27); x(28), x(29), x(30); x(31), x(32), 1];
x0405 = [x(33), x(34), x(35); x(36), x(37), x(38); x(39), x(40), 1];
x0506 = [x(41), x(42), x(43); x(44), x(45), x(46); x(47), x(48), 1];
x0607 = [x(49), x(50), x(51); x(52), x(53), x(54); x(55), x(56), 1];
x0708 = [x(57), x(58), x(59); x(60), x(61), x(62); x(63), x(64), 1];
x0809 = [x(65), x(66), x(67); x(68), x(69), x(70); x(71), x(72), 1];
x0910 = [x(73), x(74), x(75); x(76), x(77), x(78); x(79), x(80), 1];
x1011 = [x(81), x(82), x(83); x(84), x(85), x(86); x(87), x(88), 1];
x1112 = [x(89), x(90), x(91); x(92), x(93), x(94); x(95), x(96), 1];
x1213 = [x(97), x(98), x(99); x(100), x(101), x(102); x(103), x(104), 1];
x1314 = [x(105), x(106), x(107); x(108), x(109), x(110); x(111), x(112), 1];
x1415 = [x(113), x(114), x(115); x(116), x(117), x(118); x(119), x(120), 1];
x1516 = [x(121), x(122), x(123); x(124), x(125), x(126); x(127), x(128), 1];
x0002 = x0001*x0102;
x0003 = x0002*x0203;
x0004 = x0003*x0304;
x0005 = x0004*x0405;
x0006 = x0005*x0506;
x0007 = x0006*x0607;
x0008 = x0007*x0708;
x0009 = x0008*x0809;
x0010 = x0009*x0910;
x0011 = x0010*x1011;
x0012 = x0011*x1112;
x0013 = x0012*x1213;
x0014 = x0013*x1314;
x0015 = x0014*x1415;
x0016 = x0015*x1516;
c(1) = abs(det(x0001) - 1) - detthresh;
c(2) = abs(det(x0102) - 1) - detthresh;
c(3) = abs(det(x0203) - 1) - detthresh;
c(4) = abs(det(x0304) - 1) - detthresh;
c(5) = abs(det(x0405) - 1) - detthresh;
c(6) = abs(det(x0506) - 1) - detthresh;
c(7) = abs(det(x0607) - 1) - detthresh;
c(8) = abs(det(x0708) - 1) - detthresh;
c(9) = abs(det(x0809) - 1) - detthresh;
c(10) = abs(det(x0910) - 1) - detthresh;
c(11) = abs(det(x1011) - 1) - detthresh;
c(12) = abs(det(x1112) - 1) - detthresh;
c(13) = abs(det(x1213) - 1) - detthresh;
c(14) = abs(det(x1314) - 1) - detthresh;
c(15) = abs(det(x1415) - 1) - detthresh;
c(16) = abs(det(x1516) - 1) - detthresh;
c(17) = abs(x0001(3, 3) - 1) - entry33thresh;
c(18) = abs(x0002(3, 3) - 1) - entry33thresh;
c(19) = abs(x0003(3, 3) - 1) - entry33thresh;
c(20) = abs(x0004(3, 3) - 1) - entry33thresh;
c(21) = abs(x0005(3, 3) - 1) - entry33thresh;
c(22) = abs(x0006(3, 3) - 1) - entry33thresh;
c(23) = abs(x0007(3, 3) - 1) - entry33thresh;
c(24) = abs(x0008(3, 3) - 1) - entry33thresh;
c(25) = abs(x0009(3, 3) - 1) - entry33thresh;
c(26) = abs(x0010(3, 3) - 1) - entry33thresh;
c(27) = abs(x0011(3, 3) - 1) - entry33thresh;
c(28) = abs(x0012(3, 3) - 1) - entry33thresh;
c(29) = abs(x0013(3, 3) - 1) - entry33thresh;
c(30) = abs(x0014(3, 3) - 1) - entry33thresh;
c(31) = abs(x0015(3, 3) - 1) - entry33thresh;
c(32) = abs(x0016(3, 3) - 1) - entry33thresh;
c(33:34) = abs(x(1:2) - x0(1:2)) - changethresh;
c(35) = abs(x(3) - x0(3)) - transthreshx;
c(36:37) = abs(x(4:5) - x0(4:5)) - changethresh;
c(38) = abs(x(6) - x0(6)) - transthreshy;
c(39:40) = abs(x(7:8) - x0(7:8)) - smallsmallthresh;
c(41:42) = abs(x(9:10) - x0(9:10)) - changethresh;
c(43) = abs(x(11) - x0(11)) - transthreshx;
c(44:45) = abs(x(12:13) - x0(12:13)) - changethresh;
c(46) = abs(x(14) - x0(14)) - transthreshy;
c(47:48) = abs(x(15:16) - x0(15:16)) - smallsmallthresh;
c(49:50) = abs(x(17:18) - x0(17:18)) - changethresh;
c(51) = abs(x(19) - x0(19)) - transthreshx;
c(52:53) = abs(x(20:21) - x0(20:21)) - changethresh;
c(54) = abs(x(22) - x0(22)) - transthreshy;
c(55:56) = abs(x(23:24) - x0(23:24)) - smallsmallthresh;
c(57:58) = abs(x(25:26) - x0(25:26)) - changethresh;
c(59) = abs(x(27) - x0(27)) - transthreshx;
c(60:61) = abs(x(28:29) - x0(28:29)) - changethresh;
c(62) = abs(x(30) - x0(30)) - transthreshy;
c(63:64) = abs(x(31:32) - x0(31:32)) - smallsmallthresh;
c(65:66) = abs(x(33:34) - x0(33:34)) - changethresh;
c(67) = abs(x(35) - x0(35)) - transthreshx;
c(68:69) = abs(x(36:37) - x0(36:37)) - changethresh;
c(70) = abs(x(38) - x0(38)) - transthreshy;
c(71:72) = abs(x(39:40) - x0(39:40)) - smallsmallthresh;
c(73:74) = abs(x(41:42) - x0(41:42)) - changethresh;
c(75) = abs(x(43) - x0(43)) - transthreshx;
c(76:77) = abs(x(44:45) - x0(44:45)) - changethresh;
c(78) = abs(x(46) - x0(46)) - transthreshy;
c(79:80) = abs(x(47:48) - x0(47:48)) - smallsmallthresh;
c(81:82) = abs(x(49:50) - x0(49:50)) - changethresh;
c(83) = abs(x(51) - x0(51)) - transthreshx;
c(84:85) = abs(x(52:53) - x0(52:53)) - changethresh;
c(86) = abs(x(54) - x0(54)) - transthreshy;
c(87:88) = abs(x(55:56) - x0(55:56)) - smallsmallthresh;
c(89:90) = abs(x(57:58) - x0(57:58)) - changethresh;
c(91) = abs(x(59) - x0(59)) - transthreshx;
c(92:93) = abs(x(60:61) - x0(60:61)) - changethresh;
c(94) = abs(x(62) - x0(62)) - transthreshy;
c(95:96) = abs(x(63:64) - x0(63:64)) - smallsmallthresh;
c(97:98) = abs(x(65:66) - x0(65:66)) - changethresh;
c(99) = abs(x(67) - x0(67)) - transthreshx;
c(100:101) = abs(x(68:69) - x0(68:69)) - changethresh;
c(102) = abs(x(70) - x0(70)) - transthreshy;
c(103:104) = abs(x(71:72) - x0(71:72)) - smallsmallthresh;
c(105:106) = abs(x(73:74) - x0(73:74)) - changethresh;
c(107) = abs(x(75) - x0(75)) - transthreshx;
c(108:109) = abs(x(76:77) - x0(76:77)) - changethresh;
c(110) = abs(x(78) - x0(78)) - transthreshy;
c(111:112) = abs(x(79:80) - x0(79:80)) - smallsmallthresh;
c(113:114) = abs(x(81:82) - x0(81:82)) - changethresh;
c(115) = abs(x(83) - x0(83)) - transthreshx;
c(116:117) = abs(x(84:85) - x0(84:85)) - changethresh;
c(118) = abs(x(86) - x0(86)) - transthreshy;
c(119:120) = abs(x(87:88) - x0(87:88)) - smallsmallthresh;
c(121:122) = abs(x(89:90) - x0(89:90)) - changethresh;
c(123) = abs(x(91) - x0(91)) - transthreshx;
c(124:125) = abs(x(92:93) - x0(92:93)) - changethresh;
c(126) = abs(x(94) - x0(94)) - transthreshy;
c(127:128) = abs(x(95:96) - x0(95:96)) - smallsmallthresh;
c(129:130) = abs(x(97:98) - x0(97:98)) - changethresh;
c(131) = abs(x(99) - x0(99)) - transthreshx;
c(132:133) = abs(x(100:101) - x0(100:101)) - changethresh;
c(134) = abs(x(102) - x0(102)) - transthreshy;
c(135:136) = abs(x(103:104) - x0(103:104)) - smallsmallthresh;
c(137:138) = abs(x(105:106) - x0(105:106)) - changethresh;
c(139) = abs(x(107) - x0(107)) - transthreshx;
c(140:141) = abs(x(108:109) - x0(108:109)) - changethresh;
c(142) = abs(x(110) - x0(110)) - transthreshy;
c(143:144) = abs(x(111:112) - x0(111:112)) - smallsmallthresh;
c(145:146) = abs(x(113:114) - x0(113:114)) - changethresh;
c(147) = abs(x(115) - x0(115)) - transthreshx;
c(148:149) = abs(x(116:117) - x0(116:117)) - changethresh;
c(150) = abs(x(118) - x0(118)) - transthreshy;
c(151:152) = abs(x(119:120) - x0(119:120)) - smallsmallthresh;
c(153:154) = abs(x(121:122) - x0(121:122)) - changethresh;
c(155) = abs(x(123) - x0(123)) - transthreshx;
c(156:157) = abs(x(124:125) - x0(124:125)) - changethresh;
c(158) = abs(x(126) - x0(126)) - transthreshy;
c(159:160) = abs(x(127:128) - x0(127:128)) - smallsmallthresh;
end

    function [c, ceq] = nonlinearconst15includingfirstcum(x)
       ceq = []
       c = []
    end



function [c, ceq] = nonlinearconst15includingfirstandnosignchange(x) 
x0001 = [x(1), x(2), x(3); x(4), x(5), x(6); x(7), x(8), 1];
x0102 = [x(9), x(10), x(11); x(12), x(13), x(14); x(15), x(16), 1];
x0203 = [x(17), x(18), x(19); x(20), x(21), x(22); x(23), x(24), 1];
x0304 = [x(25), x(26), x(27); x(28), x(29), x(30); x(31), x(32), 1];
x0405 = [x(33), x(34), x(35); x(36), x(37), x(38); x(39), x(40), 1];
x0506 = [x(41), x(42), x(43); x(44), x(45), x(46); x(47), x(48), 1];
x0607 = [x(49), x(50), x(51); x(52), x(53), x(54); x(55), x(56), 1];
x0708 = [x(57), x(58), x(59); x(60), x(61), x(62); x(63), x(64), 1];
x0809 = [x(65), x(66), x(67); x(68), x(69), x(70); x(71), x(72), 1];
x0910 = [x(73), x(74), x(75); x(76), x(77), x(78); x(79), x(80), 1];
x1011 = [x(81), x(82), x(83); x(84), x(85), x(86); x(87), x(88), 1];
x1112 = [x(89), x(90), x(91); x(92), x(93), x(94); x(95), x(96), 1];
x1213 = [x(97), x(98), x(99); x(100), x(101), x(102); x(103), x(104), 1];
x1314 = [x(105), x(106), x(107); x(108), x(109), x(110); x(111), x(112), 1];
x1415 = [x(113), x(114), x(115); x(116), x(117), x(118); x(119), x(120), 1];
x1516 = [x(121), x(122), x(123); x(124), x(125), x(126); x(127), x(128), 1];
x0002 = x0001*x0102;
x0003 = x0002*x0203;
x0004 = x0003*x0304;
x0005 = x0004*x0405;
x0006 = x0005*x0506;
x0007 = x0006*x0607;
x0008 = x0007*x0708;
x0009 = x0008*x0809;
x0010 = x0009*x0910;
x0011 = x0010*x1011;
x0012 = x0011*x1112;
x0013 = x0012*x1213;
x0014 = x0013*x1314;
x0015 = x0014*x1415;
x0016 = x0015*x1516;
c(1) = abs(det(x0001) - 1) - detthresh;
c(2) = abs(det(x0102) - 1) - detthresh;
c(3) = abs(det(x0203) - 1) - detthresh;
c(4) = abs(det(x0304) - 1) - detthresh;
c(5) = abs(det(x0405) - 1) - detthresh;
c(6) = abs(det(x0506) - 1) - detthresh;
c(7) = abs(det(x0607) - 1) - detthresh;
c(8) = abs(det(x0708) - 1) - detthresh;
c(9) = abs(det(x0809) - 1) - detthresh;
c(10) = abs(det(x0910) - 1) - detthresh;
c(11) = abs(det(x1011) - 1) - detthresh;
c(12) = abs(det(x1112) - 1) - detthresh;
c(13) = abs(det(x1213) - 1) - detthresh;
c(14) = abs(det(x1314) - 1) - detthresh;
c(15) = abs(det(x1415) - 1) - detthresh;
c(16) = abs(det(x1516) - 1) - detthresh;
c(17) = abs(x0001(3, 3) - 1) - entry33thresh;
c(18) = abs(x0002(3, 3) - 1) - entry33thresh;
c(19) = abs(x0003(3, 3) - 1) - entry33thresh;
c(20) = abs(x0004(3, 3) - 1) - entry33thresh;
c(21) = abs(x0005(3, 3) - 1) - entry33thresh;
c(22) = abs(x0006(3, 3) - 1) - entry33thresh;
c(23) = abs(x0007(3, 3) - 1) - entry33thresh;
c(24) = abs(x0008(3, 3) - 1) - entry33thresh;
c(25) = abs(x0009(3, 3) - 1) - entry33thresh;
c(26) = abs(x0010(3, 3) - 1) - entry33thresh;
c(27) = abs(x0011(3, 3) - 1) - entry33thresh;
c(28) = abs(x0012(3, 3) - 1) - entry33thresh;
c(29) = abs(x0013(3, 3) - 1) - entry33thresh;
c(30) = abs(x0014(3, 3) - 1) - entry33thresh;
c(31) = abs(x0015(3, 3) - 1) - entry33thresh;
c(32) = abs(x0016(3, 3) - 1) - entry33thresh;
c(33:34) = abs(x(1:2) - x0(1:2)) - changethresh;
c(35) = abs(x(3) - x0(3)) - transthreshx;
c(36:37) = abs(x(4:5) - x0(4:5)) - changethresh;
c(38) = abs(x(6) - x0(6)) - transthreshy;
c(39:40) = abs(x(7:8) - x0(7:8)) - smallsmallthresh;
c(41:42) = abs(x(9:10) - x0(9:10)) - changethresh;
c(43) = abs(x(11) - x0(11)) - transthreshx;
c(44:45) = abs(x(12:13) - x0(12:13)) - changethresh;
c(46) = abs(x(14) - x0(14)) - transthreshy;
c(47:48) = abs(x(15:16) - x0(15:16)) - smallsmallthresh;
c(49:50) = abs(x(17:18) - x0(17:18)) - changethresh;
c(51) = abs(x(19) - x0(19)) - transthreshx;
c(52:53) = abs(x(20:21) - x0(20:21)) - changethresh;
c(54) = abs(x(22) - x0(22)) - transthreshy;
c(55:56) = abs(x(23:24) - x0(23:24)) - smallsmallthresh;
c(57:58) = abs(x(25:26) - x0(25:26)) - changethresh;
c(59) = abs(x(27) - x0(27)) - transthreshx;
c(60:61) = abs(x(28:29) - x0(28:29)) - changethresh;
c(62) = abs(x(30) - x0(30)) - transthreshy;
c(63:64) = abs(x(31:32) - x0(31:32)) - smallsmallthresh;
c(65:66) = abs(x(33:34) - x0(33:34)) - changethresh;
c(67) = abs(x(35) - x0(35)) - transthreshx;
c(68:69) = abs(x(36:37) - x0(36:37)) - changethresh;
c(70) = abs(x(38) - x0(38)) - transthreshy;
c(71:72) = abs(x(39:40) - x0(39:40)) - smallsmallthresh;
c(73:74) = abs(x(41:42) - x0(41:42)) - changethresh;
c(75) = abs(x(43) - x0(43)) - transthreshx;
c(76:77) = abs(x(44:45) - x0(44:45)) - changethresh;
c(78) = abs(x(46) - x0(46)) - transthreshy;
c(79:80) = abs(x(47:48) - x0(47:48)) - smallsmallthresh;
c(81:82) = abs(x(49:50) - x0(49:50)) - changethresh;
c(83) = abs(x(51) - x0(51)) - transthreshx;
c(84:85) = abs(x(52:53) - x0(52:53)) - changethresh;
c(86) = abs(x(54) - x0(54)) - transthreshy;
c(87:88) = abs(x(55:56) - x0(55:56)) - smallsmallthresh;
c(89:90) = abs(x(57:58) - x0(57:58)) - changethresh;
c(91) = abs(x(59) - x0(59)) - transthreshx;
c(92:93) = abs(x(60:61) - x0(60:61)) - changethresh;
c(94) = abs(x(62) - x0(62)) - transthreshy;
c(95:96) = abs(x(63:64) - x0(63:64)) - smallsmallthresh;
c(97:98) = abs(x(65:66) - x0(65:66)) - changethresh;
c(99) = abs(x(67) - x0(67)) - transthreshx;
c(100:101) = abs(x(68:69) - x0(68:69)) - changethresh;
c(102) = abs(x(70) - x0(70)) - transthreshy;
c(103:104) = abs(x(71:72) - x0(71:72)) - smallsmallthresh;
c(105:106) = abs(x(73:74) - x0(73:74)) - changethresh;
c(107) = abs(x(75) - x0(75)) - transthreshx;
c(108:109) = abs(x(76:77) - x0(76:77)) - changethresh;
c(110) = abs(x(78) - x0(78)) - transthreshy;
c(111:112) = abs(x(79:80) - x0(79:80)) - smallsmallthresh;
c(113:114) = abs(x(81:82) - x0(81:82)) - changethresh;
c(115) = abs(x(83) - x0(83)) - transthreshx;
c(116:117) = abs(x(84:85) - x0(84:85)) - changethresh;
c(118) = abs(x(86) - x0(86)) - transthreshy;
c(119:120) = abs(x(87:88) - x0(87:88)) - smallsmallthresh;
c(121:122) = abs(x(89:90) - x0(89:90)) - changethresh;
c(123) = abs(x(91) - x0(91)) - transthreshx;
c(124:125) = abs(x(92:93) - x0(92:93)) - changethresh;
c(126) = abs(x(94) - x0(94)) - transthreshy;
c(127:128) = abs(x(95:96) - x0(95:96)) - smallsmallthresh;
c(129:130) = abs(x(97:98) - x0(97:98)) - changethresh;
c(131) = abs(x(99) - x0(99)) - transthreshx;
c(132:133) = abs(x(100:101) - x0(100:101)) - changethresh;
c(134) = abs(x(102) - x0(102)) - transthreshy;
c(135:136) = abs(x(103:104) - x0(103:104)) - smallsmallthresh;
c(137:138) = abs(x(105:106) - x0(105:106)) - changethresh;
c(139) = abs(x(107) - x0(107)) - transthreshx;
c(140:141) = abs(x(108:109) - x0(108:109)) - changethresh;
c(142) = abs(x(110) - x0(110)) - transthreshy;
c(143:144) = abs(x(111:112) - x0(111:112)) - smallsmallthresh;
c(145:146) = abs(x(113:114) - x0(113:114)) - changethresh;
c(147) = abs(x(115) - x0(115)) - transthreshx;
c(148:149) = abs(x(116:117) - x0(116:117)) - changethresh;
c(150) = abs(x(118) - x0(118)) - transthreshy;
c(151:152) = abs(x(119:120) - x0(119:120)) - smallsmallthresh;
c(153:154) = abs(x(121:122) - x0(121:122)) - changethresh;
c(155) = abs(x(123) - x0(123)) - transthreshx;
c(156:157) = abs(x(124:125) - x0(124:125)) - changethresh;
c(158) = abs(x(126) - x0(126)) - transthreshy;
c(159:160) = abs(x(127:128) - x0(127:128)) - smallsmallthresh;
ceq(1) = sign(x(1)) - sign(x0(1));
ceq(2) = sign(x(2)) - sign(x0(2));
ceq(3) = sign(x(3)) - sign(x0(3));
ceq(4) = sign(x(4)) - sign(x0(4));
ceq(5) = sign(x(5)) - sign(x0(5));
ceq(6) = sign(x(6)) - sign(x0(6));
ceq(7) = sign(x(7)) - sign(x0(7));
ceq(8) = sign(x(8)) - sign(x0(8));
ceq(9) = sign(x(9)) - sign(x0(9));
ceq(10) = sign(x(10)) - sign(x0(10));
ceq(11) = sign(x(11)) - sign(x0(11));
ceq(12) = sign(x(12)) - sign(x0(12));
ceq(13) = sign(x(13)) - sign(x0(13));
ceq(14) = sign(x(14)) - sign(x0(14));
ceq(15) = sign(x(15)) - sign(x0(15));
ceq(16) = sign(x(16)) - sign(x0(16));
ceq(17) = sign(x(17)) - sign(x0(17));
ceq(18) = sign(x(18)) - sign(x0(18));
ceq(19) = sign(x(19)) - sign(x0(19));
ceq(20) = sign(x(20)) - sign(x0(20));
ceq(21) = sign(x(21)) - sign(x0(21));
ceq(22) = sign(x(22)) - sign(x0(22));
ceq(23) = sign(x(23)) - sign(x0(23));
ceq(24) = sign(x(24)) - sign(x0(24));
ceq(25) = sign(x(25)) - sign(x0(25));
ceq(26) = sign(x(26)) - sign(x0(26));
ceq(27) = sign(x(27)) - sign(x0(27));
ceq(28) = sign(x(28)) - sign(x0(28));
ceq(29) = sign(x(29)) - sign(x0(29));
ceq(30) = sign(x(30)) - sign(x0(30));
ceq(31) = sign(x(31)) - sign(x0(31));
ceq(32) = sign(x(32)) - sign(x0(32));
ceq(33) = sign(x(33)) - sign(x0(33));
ceq(34) = sign(x(34)) - sign(x0(34));
ceq(35) = sign(x(35)) - sign(x0(35));
ceq(36) = sign(x(36)) - sign(x0(36));
ceq(37) = sign(x(37)) - sign(x0(37));
ceq(38) = sign(x(38)) - sign(x0(38));
ceq(39) = sign(x(39)) - sign(x0(39));
ceq(40) = sign(x(40)) - sign(x0(40));
ceq(41) = sign(x(41)) - sign(x0(41));
ceq(42) = sign(x(42)) - sign(x0(42));
ceq(43) = sign(x(43)) - sign(x0(43));
ceq(44) = sign(x(44)) - sign(x0(44));
ceq(45) = sign(x(45)) - sign(x0(45));
ceq(46) = sign(x(46)) - sign(x0(46));
ceq(47) = sign(x(47)) - sign(x0(47));
ceq(48) = sign(x(48)) - sign(x0(48));
ceq(49) = sign(x(49)) - sign(x0(49));
ceq(50) = sign(x(50)) - sign(x0(50));
ceq(51) = sign(x(51)) - sign(x0(51));
ceq(52) = sign(x(52)) - sign(x0(52));
ceq(53) = sign(x(53)) - sign(x0(53));
ceq(54) = sign(x(54)) - sign(x0(54));
ceq(55) = sign(x(55)) - sign(x0(55));
ceq(56) = sign(x(56)) - sign(x0(56));
ceq(57) = sign(x(57)) - sign(x0(57));
ceq(58) = sign(x(58)) - sign(x0(58));
ceq(59) = sign(x(59)) - sign(x0(59));
ceq(60) = sign(x(60)) - sign(x0(60));
ceq(61) = sign(x(61)) - sign(x0(61));
ceq(62) = sign(x(62)) - sign(x0(62));
ceq(63) = sign(x(63)) - sign(x0(63));
ceq(64) = sign(x(64)) - sign(x0(64));
ceq(65) = sign(x(65)) - sign(x0(65));
ceq(66) = sign(x(66)) - sign(x0(66));
ceq(67) = sign(x(67)) - sign(x0(67));
ceq(68) = sign(x(68)) - sign(x0(68));
ceq(69) = sign(x(69)) - sign(x0(69));
ceq(70) = sign(x(70)) - sign(x0(70));
ceq(71) = sign(x(71)) - sign(x0(71));
ceq(72) = sign(x(72)) - sign(x0(72));
ceq(73) = sign(x(73)) - sign(x0(73));
ceq(74) = sign(x(74)) - sign(x0(74));
ceq(75) = sign(x(75)) - sign(x0(75));
ceq(76) = sign(x(76)) - sign(x0(76));
ceq(77) = sign(x(77)) - sign(x0(77));
ceq(78) = sign(x(78)) - sign(x0(78));
ceq(79) = sign(x(79)) - sign(x0(79));
ceq(80) = sign(x(80)) - sign(x0(80));
ceq(81) = sign(x(81)) - sign(x0(81));
ceq(82) = sign(x(82)) - sign(x0(82));
ceq(83) = sign(x(83)) - sign(x0(83));
ceq(84) = sign(x(84)) - sign(x0(84));
ceq(85) = sign(x(85)) - sign(x0(85));
ceq(86) = sign(x(86)) - sign(x0(86));
ceq(87) = sign(x(87)) - sign(x0(87));
ceq(88) = sign(x(88)) - sign(x0(88));
ceq(89) = sign(x(89)) - sign(x0(89));
ceq(90) = sign(x(90)) - sign(x0(90));
ceq(91) = sign(x(91)) - sign(x0(91));
ceq(92) = sign(x(92)) - sign(x0(92));
ceq(93) = sign(x(93)) - sign(x0(93));
ceq(94) = sign(x(94)) - sign(x0(94));
ceq(95) = sign(x(95)) - sign(x0(95));
ceq(96) = sign(x(96)) - sign(x0(96));
ceq(97) = sign(x(97)) - sign(x0(97));
ceq(98) = sign(x(98)) - sign(x0(98));
ceq(99) = sign(x(99)) - sign(x0(99));
ceq(100) = sign(x(100)) - sign(x0(100));
ceq(101) = sign(x(101)) - sign(x0(101));
ceq(102) = sign(x(102)) - sign(x0(102));
ceq(103) = sign(x(103)) - sign(x0(103));
ceq(104) = sign(x(104)) - sign(x0(104));
ceq(105) = sign(x(105)) - sign(x0(105));
ceq(106) = sign(x(106)) - sign(x0(106));
ceq(107) = sign(x(107)) - sign(x0(107));
ceq(108) = sign(x(108)) - sign(x0(108));
ceq(109) = sign(x(109)) - sign(x0(109));
ceq(110) = sign(x(110)) - sign(x0(110));
ceq(111) = sign(x(111)) - sign(x0(111));
ceq(112) = sign(x(112)) - sign(x0(112));
ceq(113) = sign(x(113)) - sign(x0(113));
ceq(114) = sign(x(114)) - sign(x0(114));
ceq(115) = sign(x(115)) - sign(x0(115));
ceq(116) = sign(x(116)) - sign(x0(116));
ceq(117) = sign(x(117)) - sign(x0(117));
ceq(118) = sign(x(118)) - sign(x0(118));
ceq(119) = sign(x(119)) - sign(x0(119));
ceq(120) = sign(x(120)) - sign(x0(120));
ceq(121) = sign(x(121)) - sign(x0(121));
ceq(122) = sign(x(122)) - sign(x0(122));
ceq(123) = sign(x(123)) - sign(x0(123));
ceq(124) = sign(x(124)) - sign(x0(124));
ceq(125) = sign(x(125)) - sign(x0(125));
ceq(126) = sign(x(126)) - sign(x0(126));
ceq(127) = sign(x(127)) - sign(x0(127));
ceq(128) = sign(x(128)) - sign(x0(128));

end


end