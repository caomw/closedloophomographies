import cv, numpy, sys, os

if len(sys.argv) < 3:
    sys.exit(1)

img = []
img.append(int(sys.argv[1]))
img.append(int(sys.argv[2]))

basefile = "imagenes/mosaico"

for i in img:
    filename = basefile + "%04d.tif" % i
    print filename
    cmd = "shotwell " + filename + " &"
    os.system(cmd)

raw_input("Press any key")
