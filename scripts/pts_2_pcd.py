#!/usr/bin/env python2

from numpy import loadtxt, savetxt

filename="Prueba1.pts"

print "Loading"
lines = loadtxt(filename,skiprows=1)
size=str(lines[:,0].size)
print "Loaded"

header = "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH "+size+"\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS "+size+"\nDATA ascii"

pcd=lines[:,[0,1,2]]
print "converting"

savetxt("Prueba1.pcd", pcd, fmt="%f",header=header,comments='')
print "saved"