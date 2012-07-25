function runabunchoftests

%runtest('test_set_16', 'testset16test4', .11, .06, .02, .0008, 300, 225);
%nofeasiblesolution
%runtest('test_set_16', 'testset16test5', .10, .05, .01, .0005, 300, 225);
%nofeasiblesolution
%runtest('test_set_16', 'testset16test6', .10, .05, .02, .0008, 200, 150);
%nofeasiblesolution
%runtest('test_set_16', 'testset16test7', .11, .06, .02, .0008, 200, 150);
%nofeasiblesolution
%runtest('test_set_16', 'testset16test8', .12, .06, .02, .0008, 300, 225);
%nofeasiblesolution
%runtest('test_set_16', 'testset16test4', .12, .05, .02, .0008, 300, 225);
%nofeasiblesolution
%runtest('test_set_16', 'testset16test5', .11, .06, .02, .0008, 200, 150);
%nofeasiblesolution
%runtest('test_set_16', 'testset16test4', .10, .05, .01, .001, 300, 225);
%nofeasiblesolution
%runtest('test_set_16', 'testset16test8', .10, .05, .01, .0005, 300, 225);
%nofeasible solution
%runtest('test_set_16', 'testset16test8', .10, .25, .01, .001, 300, 225);
%nofeasiblesolution
%runtest('test_set_16', 'testset16test8', .15, .25, .01, .001, 300, 225);
%nofeasiblesolution
%runtest('test_set_16', 'testset16test8', .10, .15, .01, .001, 300, 225);
%nofeasiblesolution
%runtest('test_set_16', 'testset16test8', .20, .15, .01, .001, 300, 225);
%nofeasiblesolution
%runtest('test_set_16', 'testset16test10', .20, .12, .005, .001, 300, 225);
%no feasible solution
%runtest('test_set_16', 'testset16test10', .20, .10, .005, .001, 100, 100);
%no feasible solution
%runtest('test_set_16', 'testset16test10', .20, .10, .01, .001, 100, 100);
%no feasible solution
%runtest('test_set_16', 'testset16test10', .20, .15, .01, .001, 100, 100);
%no feasible solution
%from here below, tried using the "no switching signs" metric and it didn't
%yield any results.
%runtest('test_set_16', 'testset16test11', .20, .05, .01, .001, 120, 90);
%no feasible solution
%runtest('test_set_16', 'testset16test11', .15, .15, .01, .001, 120, 90);
%no feasible solution
%runtest('test_set_16', 'testset16test12', .20, .05, .01, .001, 150, 100);
%no feasible solution
%runtest('test_set_16', 'testset16test13', .20, .05, .01, .001, 200, 150);
%no feasible solution
%runtest('test_set_16', 'testset16test11', .11, .05, .02, .0008, 300, 225);
%nofeasiblesolution
%runtest('test_set_16', 'testset16test12', .10, .08, .01, .001, 300, 225);
%nofeasiblesolution
%runtest('test_set_16', 'testset16test13', .15, .15, .01, .001, 120, 90);
%nofeasiblesolution
%runtest('test_set_16', 'testset16test14', .15, .15, .01, .001, 280, 210);
%nofeasiblesolution
%runtest('test_set_16', 'testset16test15', .20, .25, .01, .001, 280, 210);
%nofeasiblesolution
%runtest('test_set_16', 'testset16test11', .20, .10, .01, .001, 300, 225);
%nofeasiblesolution
%runtest('test_set_16', 'testset16test11', .25, .25, .25, .01, 300, 225);
%nofeasiblesolutions
%runtest('test_set_16', 'testset16test11', .15, .15, .05, .01, 300, 225);
%nofeasiblesolutions
%runtest('test_set_16', 'testset16test12', .15, .15, .01, .01, 120, 90);
%nofeasiblesolutions

%runtest('test_set_16', 'testset16test6', .11, .05, .02, .0008, 300, 225);
%localminimumfound
%runtest('test_set_16', 'testset16test7', .11, .05, .002, .001, 300, 225);
%localminimumfound
%runtest('test_set_16', 'testset16test5', .10, .08, .01, .001, 300, 225);
%localminimumfound
%runtest('test_set_16', 'testset16test4', .15, .15, .01, .001, 300, 225);
%localminimumfound
%runtest('test_set_16', 'testset16test8', .20, .05, .01, .001, 300, 225);
%local minimum possible
%runtest('test_set_16', 'testset16test9', .20, .10, .01, .001, 300, 225);
%local minimum possible
%runtest('test_set_16', 'testset16test10', .20, .05, .01, .001, 150, 100);
%local minimum found

runtest('test_set_16', 'testset16test11', .15, .15, .05, .01, 300, 225);
%
runtest('test_set_16', 'testset16test12', .10, .10, .01, .01, 280, 210);
%
runtest('test_set_16', 'testset16test13', .20, .10, .01, .001, 120, 90);
%
runtest('test_set_16', 'testset16test14', .10, .10, .01, .01, 120, 90);
%
runtest('test_set_16', 'testset16test15', .20, .10, .01, .001, 280, 210);
%

end