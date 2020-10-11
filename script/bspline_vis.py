#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Sep 20 22:00:44 2020

@author: navision
"""
import numpy as np
import  matplotlib as mpl
import matplotlib.pyplot as plt
import ctypes

so = ctypes.CDLL('/home/navision/catkin_ws/devel/lib/libTEST.so')

a = so.trajectorySize()
size = (a - 3 + 2)/0.01
b = so.pathSize()


pyarray = np.linspace(1,4,int(size))
carrayx = (ctypes.c_double*len(pyarray))(*pyarray)
carrayy = (ctypes.c_double*len(pyarray))(*pyarray)

pypath = np.linspace(1,4, b)
cpathx = (ctypes.c_double*len(pypath))(*pypath)
cpathy = (ctypes.c_double*len(pypath))(*pypath)

so.splineVector(carrayx,carrayy,len(carrayx))
so.pathVector(cpathx, cpathy, len(cpathx))

plt.plot(carrayx,carrayy,lw = 1.5)
plt.plot(cpathx, cpathy,'o', lw = 1.5, linestyle="-.")

plt.grid(True)
plt.show()


