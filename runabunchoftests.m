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
%runtest('test_set_16', 'testset16test6', .11, .05, .02, .0008, 300, 225);
%localminimumfound
%runtest('test_set_16', 'testset16test7', .11, .05, .02, .001, 300, 225);
%localminimumfound
%runtest('test_set_16', 'testset16test4', .10, .05, .01, .001, 300, 225);
%nofeasiblesolution
%runtest('test_set_16', 'testset16test5', .10, .08, .01, .001, 300, 225);
%localminimumfound
%runtest('test_set_16', 'testset16test8', .10, .05, .01, .0005, 300, 225);
%nofeasible solution
%runtest('test_set_16', 'testset16test4', .15, .15, .01, .001, 300, 225);
%localminimumfound
%runtest('test_set_16', 'testset16test8', .10, .25, .01, .001, 300, 225);
%nofeasiblesolution
%runtest('test_set_16', 'testset16test8', .15, .25, .01, .001, 300, 225);
%nofeasiblesolution
runtest('test_set_16', 'testset16test8', .11, .05, .01, .001, 300, 225);
%nofeasiblesolution

end