import sys

def main():
    if (len(sys.argv) < 3):
        print "not enough arguments"
        return 0
    
    first = int(sys.argv[1])
    last = int(sys.argv[2])

    # optimization function generation

    if (False):
        print 'function f = loop' + str(last - first) + '(xin)'
        xcumstring = ''
        for i in xrange(first, last):
            curx = ('x%(one)02d%(two)02d') % {'one': i, 'two': i + 1}
            if i == 1:
                xcumstring = curx
            else:
                xcumstring += '*' + curx
            print (curx + ' = [xin(' + str(8*i) + '), xin(' + str(8*i + 1) + 
                    '), xin(' + str(8*i + 2) + '); xin(' + str(8 * i + 3) + '), xin(' +
                    str(8*i + 4) + '), xin(' + str(8*i + 5) + '); xin(' + str(8*i + 6) +
                    '), xin(' + str(8*i + 7) + '), xin(' + str(8*i + 8) + '), 1];')
        print 'xcum = ' + xcumstring + ';'
        print ('f = abs(xcum(1, 1) - newHomo(1, 1)) + abs(xcum(1, 2) - newHomo(1, 2)) + ...\n' + 
            '\tabs(xcum(1, 3) - newHomo(1, 3)) + abs(xcum(2, 1) - newHomo(2, 1)) + ...\n' + 
            '\tabs(xcum(2, 2) - newHomo(2, 2)) + abs(xcum(2, 3) - newHomo(2, 3)) + ...\n' +
            '\tabs(xcum(3, 1) - newHomo(3, 1)) + abs(xcum(3, 2) - newHomo(3, 2)) + ...\n' +
            '\tabs(xcum(3, 3) - newHomo(3, 3));')
        print 'end'

        print '\n\n'

    # optimization function generation INCLUDING THE FIRST HOMOGRAPHY

    if (False):
        print 'function f = loop' + str(last - first) + 'includingfirst(xin)'
        xcumstring = ''
        for i in xrange(first - 1, last):
            curx = ('x%(one)02d%(two)02d') % {'one': i, 'two': i + 1}
            if i == 0:
                xcumstring = curx
            else:
                xcumstring += '*' + curx
            print (curx + ' = [xin(' + str(8*i + 1) + '), xin(' + str(8*i + 2) + 
                    '), xin(' + str(8*i + 3) + '); xin(' + str(8 * i + 4) + '), xin(' +
                    str(8*i + 5) + '), xin(' + str(8*i + 6) + '); xin(' + str(8*i + 7) +
                    '), xin(' + str(8*i + 8) + '), 1];')
        print 'xcum = ' + xcumstring + ';'
        print ('f = abs(xcum(1, 1) - newHomo(1, 1)) + abs(xcum(1, 2) - newHomo(1, 2)) + ...\n' + 
            '\tabs(xcum(1, 3) - newHomo(1, 3)) + abs(xcum(2, 1) - newHomo(2, 1)) + ...\n' + 
            '\tabs(xcum(2, 2) - newHomo(2, 2)) + abs(xcum(2, 3) - newHomo(2, 3)) + ...\n' +
            '\tabs(xcum(3, 1) - newHomo(3, 1)) + abs(xcum(3, 2) - newHomo(3, 2)) + ...\n' +
            '\tabs(xcum(3, 3) - newHomo(3, 3));')
        print 'end'

        print '\n\n'


    # linear constraint generation 

    if (False):
        print 'function [c, ceq] = nonlinearconst' + str(last - first) + 'matrix(x) \n ceq = [];'
        listofvarnames = []
        for i in xrange(first - 1, last - 1):
            curx = ('x%(one)02d%(two)02d') % {'one': i + 1, 'two': i + 2}
            print (curx + ' = [x(' + str(8*i + 1) + '), x(' + str(8*i + 2) + 
                    '), x(' + str(8*i + 3) + '); x(' + str(8 * i + 4) + '), x(' +
                    str(8*i + 5) + '), x(' + str(8*i + 6) + '); x(' + str(8*i + 7) +
                    '), x(' + str(8*i + 8) + '), 1];')
            listofvarnames.append(curx)
        for i in xrange(first + 2, last + 1):
            print ('x%(one)02d%(two)02d = x%(one)02d%(three)02d*x%(three)02d%(two)02d;') % {'one': 1, 'two': i, 'three': i - 1}
        counter = 1;
        for varname in listofvarnames:
            print 'c(' + str(counter) + ') = abs(det(' + varname + ') - 1) - detthresh;'
            counter += 1
        for i in xrange(first + 1, last + 1):
            print ('c(' + str(counter) + ') = abs(x01%(one)02d(3, 3) - 1) - entry33thresh;') % {'one': i}     
            counter += 1
        for i in xrange(first - 1 , last - 1):
            print ('c(' + str(counter) + ':' + str(counter + 1) + 
                 ') = abs(x(%(onei)d:%(twoi)d) - x0(%(onei)d:%(twoi)d)) - changethresh;') % {'onei': 8*i + 1, 'twoi': 8*i + 2}
            counter += 2
            print ('c(' + str(counter) + ') = abs(x(%(onei)d) - x0(%(onei)d)) - transthresh;') % {'onei': 8*i + 3}
            counter += 1
            print ('c(' + str(counter) + ':' + str(counter + 1) + 
                 ') = abs(x(%(onei)d:%(twoi)d) - x0(%(onei)d:%(twoi)d)) - changethresh;') % {'onei': 8*i + 4, 'twoi': 8*i + 5}
            counter += 2
            print ('c(' + str(counter) + ') = abs(x(%(onei)d) - x0(%(onei)d)) - transthresh;') % {'onei': 8*i + 6}
            counter += 1
            print ('c(' + str(counter) + ':' + str(counter + 1) + 
                 ') = abs(x(%(onei)d:%(twoi)d) - x0(%(onei)d:%(twoi)d)) - smallsmallthresh;') % {'onei': 8*i + 7, 'twoi': 8*i + 8}
            counter += 2
        print 'end'

        print '\n\n\n'

    # linear constraint generation INCLUDING FIRST HOMOGRAPHY 

    if (False):
        print 'function [c, ceq] = nonlinearconst' + str(last - first) + 'matrixincludingfirst(x) \n ceq = [];'
        listofvarnames = []
        for i in xrange(first - 1, last):
            curx = ('x%(one)02d%(two)02d') % {'one': i, 'two': i + 1}
            print (curx + ' = [x(' + str(8*i + 1) + '), x(' + str(8*i + 2) + 
                    '), x(' + str(8*i + 3) + '); x(' + str(8 * i + 4) + '), x(' +
                    str(8*i + 5) + '), x(' + str(8*i + 6) + '); x(' + str(8*i + 7) +
                    '), x(' + str(8*i + 8) + '), 1];')
            listofvarnames.append(curx)
        for i in xrange(first + 1, last + 1):
            print ('x%(one)02d%(two)02d = x%(one)02d%(three)02d*x%(three)02d%(two)02d;') % {'one': 0, 'two': i, 'three': i - 1}
        counter = 1;
        for varname in listofvarnames:
            print 'c(' + str(counter) + ') = abs(det(' + varname + ') - 1) - detthresh;'
            counter += 1
        for i in xrange(first, last + 1):
            print ('c(' + str(counter) + ') = abs(x00%(one)02d(3, 3) - 1) - entry33thresh;') % {'one': i}     
            counter += 1
        for i in xrange(first - 1 , last):
            print ('c(' + str(counter) + ':' + str(counter + 1) + 
                 ') = abs(x(%(onei)d:%(twoi)d) - x0(%(onei)d:%(twoi)d)) - changethresh;') % {'onei': 8*i + 1, 'twoi': 8*i + 2}
            counter += 2
            print ('c(' + str(counter) + ') = abs(x(%(onei)d) - x0(%(onei)d)) - transthreshx;') % {'onei': 8*i + 3}
            counter += 1
            print ('c(' + str(counter) + ':' + str(counter + 1) + 
                 ') = abs(x(%(onei)d:%(twoi)d) - x0(%(onei)d:%(twoi)d)) - changethresh;') % {'onei': 8*i + 4, 'twoi': 8*i + 5}
            counter += 2
            print ('c(' + str(counter) + ') = abs(x(%(onei)d) - x0(%(onei)d)) - transthreshy;') % {'onei': 8*i + 6}
            counter += 1
            print ('c(' + str(counter) + ':' + str(counter + 1) + 
                 ') = abs(x(%(onei)d:%(twoi)d) - x0(%(onei)d:%(twoi)d)) - smallsmallthresh;') % {'onei': 8*i + 7, 'twoi': 8*i + 8}
            counter += 2
        print 'end'

        print '\n\n\n'

    # linear constraint generation INCLUDING FIRST HOMOGRAPHY cumulative constrants -- TODO. is it even worth it?

    if (False):
        print 'function [c, ceq] = nonlinearconst' + str(last - first) + 'matrixincludingfirst(x) \n ceq = [];'
        listofvarnames = []
        for i in xrange(first - 1, last):
            curx = ('x%(one)02d%(two)02d') % {'one': i, 'two': i + 1}
            print (curx + ' = [x(' + str(8*i + 1) + '), x(' + str(8*i + 2) + 
                    '), x(' + str(8*i + 3) + '); x(' + str(8 * i + 4) + '), x(' +
                    str(8*i + 5) + '), x(' + str(8*i + 6) + '); x(' + str(8*i + 7) +
                    '), x(' + str(8*i + 8) + '), 1];')
            listofvarnames.append(curx)
        for i in xrange(first + 1, last + 1):
            print ('x%(one)02d%(two)02d = x%(one)02d%(three)02d*x%(three)02d%(two)02d;') % {'one': 0, 'two': i, 'three': i - 1}
        counter = 1;
        for varname in listofvarnames:
            print 'c(' + str(counter) + ') = abs(det(' + varname + ') - 1) - detthresh;'
            counter += 1
        for i in xrange(first, last + 1):
            print ('c(' + str(counter) + ') = abs(x00%(one)02d(3, 3) - 1) - entry33thresh;') % {'one': i}     
            counter += 1
        for i in xrange(first - 1 , last):
            print ('c(' + str(counter) + ':' + str(counter + 1) + 
                 ') = abs(x(%(onei)d:%(twoi)d) - x0(%(onei)d:%(twoi)d)) - changethresh;') % {'onei': 8*i + 1, 'twoi': 8*i + 2}
            counter += 2
            print ('c(' + str(counter) + ') = abs(x(%(onei)d) - x0(%(onei)d)) - transthreshx;') % {'onei': 8*i + 3}
            counter += 1
            print ('c(' + str(counter) + ':' + str(counter + 1) + 
                 ') = abs(x(%(onei)d:%(twoi)d) - x0(%(onei)d:%(twoi)d)) - changethresh;') % {'onei': 8*i + 4, 'twoi': 8*i + 5}
            counter += 2
            print ('c(' + str(counter) + ') = abs(x(%(onei)d) - x0(%(onei)d)) - transthreshy;') % {'onei': 8*i + 6}
            counter += 1
            print ('c(' + str(counter) + ':' + str(counter + 1) + 
                 ') = abs(x(%(onei)d:%(twoi)d) - x0(%(onei)d:%(twoi)d)) - smallsmallthresh;') % {'onei': 8*i + 7, 'twoi': 8*i + 8}
            counter += 2
        print 'end'

        print '\n\n\n'

    # linear constraint generation INCLUDING FIRST HOMOGRAPHY and including not allowing sign changes 

    if (True):
        print 'function [c, ceq] = nonlinearconst' + str(last - first) + 'matrixincludingfirstandnosignchange(x);'
        listofvarnames = []
        for i in xrange(first - 1, last):
            curx = ('x%(one)02d%(two)02d') % {'one': i, 'two': i + 1}
            print (curx + ' = [x(' + str(8*i + 1) + '), x(' + str(8*i + 2) + 
                    '), x(' + str(8*i + 3) + '); x(' + str(8 * i + 4) + '), x(' +
                    str(8*i + 5) + '), x(' + str(8*i + 6) + '); x(' + str(8*i + 7) +
                    '), x(' + str(8*i + 8) + '), 1];')
            listofvarnames.append(curx)
        for i in xrange(first + 1, last + 1):
            print ('x%(one)02d%(two)02d = x%(one)02d%(three)02d*x%(three)02d%(two)02d;') % {'one': 0, 'two': i, 'three': i - 1}
        counter = 1;
        for varname in listofvarnames:
            print 'c(' + str(counter) + ') = abs(det(' + varname + ') - 1) - detthresh;'
            counter += 1
        for i in xrange(first, last + 1):
            print ('c(' + str(counter) + ') = abs(x00%(one)02d(3, 3) - 1) - entry33thresh;') % {'one': i}     
            counter += 1
        for i in xrange(first - 1 , last):
            print ('c(' + str(counter) + ':' + str(counter + 1) + 
                 ') = abs(x(%(onei)d:%(twoi)d) - x0(%(onei)d:%(twoi)d)) - changethresh;') % {'onei': 8*i + 1, 'twoi': 8*i + 2}
            counter += 2
            print ('c(' + str(counter) + ') = abs(x(%(onei)d) - x0(%(onei)d)) - transthreshx;') % {'onei': 8*i + 3}
            counter += 1
            print ('c(' + str(counter) + ':' + str(counter + 1) + 
                 ') = abs(x(%(onei)d:%(twoi)d) - x0(%(onei)d:%(twoi)d)) - changethresh;') % {'onei': 8*i + 4, 'twoi': 8*i + 5}
            counter += 2
            print ('c(' + str(counter) + ') = abs(x(%(onei)d) - x0(%(onei)d)) - transthreshy;') % {'onei': 8*i + 6}
            counter += 1
            print ('c(' + str(counter) + ':' + str(counter + 1) + 
                 ') = abs(x(%(onei)d:%(twoi)d) - x0(%(onei)d:%(twoi)d)) - smallsmallthresh;') % {'onei': 8*i + 7, 'twoi': 8*i + 8}
            counter += 2
        for i in xrange(1, 8*(last - first + 1) + 1):
            print ('ceq(%(one)d) = sign(x(%(one)d)) - sign(x0(%(one)d));') % {'one': i}
        print 'end'

        print '\n\n\n'

    # linear constraint generation INCLUDING FIRST HOMOGRAPHY using t and R constraints non-cumulative 

    if (True):
        print 'function [c, ceq] = nonlinearconst' + str(last - first) + 'matrixincludingfirstandnosignchange(x);'
        listofvarnames = []
        for i in xrange(first - 1, last):
            curx = ('x%(one)02d%(two)02d') % {'one': i, 'two': i + 1}
            print (curx + ' = [x(' + str(8*i + 1) + '), x(' + str(8*i + 2) + 
                    '), x(' + str(8*i + 3) + '); x(' + str(8 * i + 4) + '), x(' +
                    str(8*i + 5) + '), x(' + str(8*i + 6) + '); x(' + str(8*i + 7) +
                    '), x(' + str(8*i + 8) + '), 1];')
            listofvarnames.append(curx)
        for i in xrange(first + 1, last + 1):
            print ('x%(one)02d%(two)02d = x%(one)02d%(three)02d*x%(three)02d%(two)02d;') % {'one': 0, 'two': i, 'three': i - 1}
        counter = 1;
        for varname in listofvarnames:
            print 'c(' + str(counter) + ') = abs(det(' + varname + ') - 1) - detthresh;'
            counter += 1
        for i in xrange(first, last + 1):
            print ('c(' + str(counter) + ') = abs(x00%(one)02d(3, 3) - 1) - entry33thresh;') % {'one': i}     
            counter += 1
        for i in xrange(first - 1 , last):
            print ('c(' + str(counter) + ':' + str(counter + 1) + 
                 ') = abs(x(%(onei)d:%(twoi)d) - x0(%(onei)d:%(twoi)d)) - changethresh;') % {'onei': 8*i + 1, 'twoi': 8*i + 2}
            counter += 2
            print ('c(' + str(counter) + ') = abs(x(%(onei)d) - x0(%(onei)d)) - transthreshx;') % {'onei': 8*i + 3}
            counter += 1
            print ('c(' + str(counter) + ':' + str(counter + 1) + 
                 ') = abs(x(%(onei)d:%(twoi)d) - x0(%(onei)d:%(twoi)d)) - changethresh;') % {'onei': 8*i + 4, 'twoi': 8*i + 5}
            counter += 2
            print ('c(' + str(counter) + ') = abs(x(%(onei)d) - x0(%(onei)d)) - transthreshy;') % {'onei': 8*i + 6}
            counter += 1
            print ('c(' + str(counter) + ':' + str(counter + 1) + 
                 ') = abs(x(%(onei)d:%(twoi)d) - x0(%(onei)d:%(twoi)d)) - smallsmallthresh;') % {'onei': 8*i + 7, 'twoi': 8*i + 8}
            counter += 2
        print 'end'

        print '\n\n\n'



    # run test main function

    if (False):
        listofnewvarnames = []
        for varname in listofvarnames:
            newvarname = varname.replace('x', 'h')
            listofnewvarnames.append(newvarname)
        print ('function runtest' + str(last - first) + 'loop')
        print ('basefile = sprintf(\'%s/homografia\', testset);')
        print ('[' + ', '.join(listofnewvarnames) + '] = loadMatFromOpenCVXML(basefile, ' + str(first + 1) + ', ' + str(last) + ');')
        print ('basefile2 = sprintf(\'%s/homografia_new') + ('%(one)04d.xml\', test_name);') % {'one': last}
        correct = ('h%(one)02d%(two)02d') % {'one': first, 'two': last}
        print (correct + ' = importXMLtoMATLAB(basefile2);')
        for varname in listofnewvarnames:
            print (varname + 'vec = reshape(' + varname + '\', 9, 1);')
        print ('x0 = [' + 'vec(1:8); '.join(listofnewvarnames) + 'vec(1:8)];')
        print ('[x val] = callObjConstr(x0, ' + correct + ', detthresh, changethresh, ... \n entry33thresh, smallsmallthresh, transthreshx, transthreshy);')
        print ('vecToOpenCVXML(x, test_name)')
        print 'end'
      
        print '\n\n\n'
 
    # run test main function INCLUDING FIRST ONE

    if (False):
        listofnewvarnames = []
        for varname in listofvarnames:
            newvarname = varname.replace('x', 'h')
            listofnewvarnames.append(newvarname)
        print ('function runtest' + str(last - first) + 'loopincludingfirstone')
        print ('basefile = sprintf(\'%s/homografia\', testset);')
        print ('[' + ', '.join(listofnewvarnames) + '] = loadMatFromOpenCVXML(basefile, ' + str(first) + ', ' + str(last) + ');')
        print ('basefile2 = sprintf(\'%s/homografia_new') + ('%(one)04d.xml\', test_name);') % {'one': last}
        correct = ('h%(one)02d%(two)02d') % {'one': first - 1, 'two': last}
        print (correct + ' = importXMLtoMATLAB(basefile2);')
        for varname in listofnewvarnames:
            print (varname + 'vec = reshape(' + varname + '\', 9, 1);')
        print ('x0 = [' + 'vec(1:8); '.join(listofnewvarnames) + 'vec(1:8)];')
        print ('[x val] = callObjConstr(x0, ' + correct + ', detthresh, changethresh, ... \n entry33thresh, smallsmallthresh);')
        print ('vecToOpenCVXML(x, test_name)')
        print 'end'
      
        print '\n\n\n'

    # vecToOpenCVXML function
   
    if (False): 
        dummycount = 0;
        for varname in listofnewvarnames:
            print (varname + 'newvec = [x(%(one)d:%(two)d); 1];') % {'one': 8*dummycount + 1, 'two': 8*dummycount + 8}
            print ('writetoFile(%(one)d, ' + varname + 'newvec, directory);') % {'one': dummycount + 2}
            dummycount += 1
        
        print '\n\n\n'
 
    # vecToOpenCVXML function INCLUDING FIRST ONE
   
    if (False): 
        dummycount = 0;
        for varname in listofnewvarnames:
            print (varname + 'newvec = [x(%(one)d:%(two)d); 1];') % {'one': 8*dummycount + 1, 'two': 8*dummycount + 8}
            print ('writetoFile(%(one)d, ' + varname + 'newvec, directory);') % {'one': dummycount + 1}
            dummycount += 1
        
        print '\n\n\n'
    
    # matlab command to run
 
if __name__ == "__main__":
    main()
