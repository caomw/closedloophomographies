function [x fval] = callObjConstr(x0, newHomo, detthresh, changethresh,...
    entry33thresh, smallsmallthresh, transthresh, options)

if nargin < 8
    options = optimset('Algorithm', 'sqp', 'MaxFunEvals', 30000);
end

%[x fval] = fmincon(@objectiveh16istruth, x0, [], [], [], [], [], [], @nonlinearconst, options);
[x fval] = fmincon(@loop15, x0, [], [], [], [], [], [], @nonlinearconst15homogs, options);

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
    c(40) = abs(x12(3, 3) - 1) - entry33thresh;
    end

    function [c, ceq] = nonlinearconst15homogs(x)
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
end