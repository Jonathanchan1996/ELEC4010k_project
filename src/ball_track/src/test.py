#!/usr/bin/env python
import numpy as np
x=np.array([3,2,3,4])
def fun(a, b):
    a[0]=a[0]+b
    return a[0]


print fun(x,5)
print x
