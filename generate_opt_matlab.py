import sys

def main():
    if (len(sys.argv) < 3):
        print "not enough arguments"
        return 0
    
    first = int(sys.argv[1])
    last = int(sys.argv[2])

    # optimization function generation

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

    # linear constraint generation 

    print 'function [c, ceq] = nonlinearconst' + str(last - first) + 'matrix(x) \n ceq = [];'
    listofvarnames = []
    for i in xrange(first, last):
        curx = ('x%(one)02d%(two)02d') % {'one': i, 'two': i + 1}
        print (curx + ' = [xin(' + str(8*i) + '), xin(' + str(8*i + 1) + 
                '), xin(' + str(8*i + 2) + '); xin(' + str(8 * i + 3) + '), xin(' +
                str(8*i + 4) + '), xin(' + str(8*i + 5) + '); xin(' + str(8*i + 6) +
                '), xin(' + str(8*i + 7) + '), xin(' + str(8*i + 8) + '), 1];')
        listofvarnames.append(curx)
    for i in xrange(first + 2, last + 1):
        print ('x01%(one)02d = x1' + str(i - 1) + '*x' + str(i - 1) + str(i) + ';') % {'one': i}
    counter = 1;
    for varname in listofvarnames:
        print 'c(' + str(counter) + ') = abs(det(' + varname + ') - 1) - detthresh;'
        counter += 1
        
    print 'end'
 
if __name__ == "__main__":
    main()
