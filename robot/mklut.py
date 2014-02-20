#!/usr/bin/python
from math import cos, pi, log, exp
import sys

min=4
max=64

def dnorm(x, m, s):
    return exp(-0.5 * log(2*pi) - log(s) - 0.5*((x-m)*1.0/s)**2)

# Old cosine based distribution
#def dvmax(v,min,max):
#    return (max-min) / 2 * (cos(2*pi/255*v) + 1) + min

# New gaussian distribution
def dvmax(v,min,max):
    return 4250 * (.0125 - dnorm(v, 127, 28)) + 11.5

sys.stdout.write("const unsigned char dvmaxlut[] = {")
for i in range(0, 254):
    if i % 10 == 0: sys.stdout.write('\n   ')
    sys.stdout.write (' %2d,' % (dvmax(i, min, max)))

sys.stdout.write (' %02d\n};' % (dvmax(255, min, max)))

# dnorm(x, m, s) = exp(-0.5 * log(2*pi) - log(s) - 0.5*((x-m)*1.0/s)**2)
# dnorm = 4250 * (.0125 - dnorm(v, 127, 28)) + 11.5
