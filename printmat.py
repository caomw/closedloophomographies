import cv, numpy, sys

if len(sys.argv) < 4:
    sys.exit(1)

max_range = int(sys.argv[3])
min_range = int(sys.argv[2])
filepath = sys.argv[1]

for i in range(min_range, max_range + 1):
    print filepath + "%04d.xml" % i
    print numpy.asarray(cv.Load(filepath + "%04d.xml" % i)) 
