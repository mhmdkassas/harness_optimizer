# -*- coding: utf-8 -*-
"""
Created on Mon Feb 11 10:52:14 2019

@author: mhmdk
"""

from model_returner import modelForces
import math
import opensim as osim
from scipy.optimize import minimize

#choose the index of the file you want to model
fileID = 0

arr = modelForces(index = fileID)
copy = arr[0]
model = arr[5]
frcSet = copy.getForceSet()
f1_index = arr[2]
f2_index = arr[3]

modelType = arr[4]

forceSetIndex = arr[6]


def optimizer_callBack(x):
    #the parameters that the optimizer changes
    ks = x[0]*10000
    
    #initializes the system and sets up forward dynamics tool
    copy.initSystem()
    reporter = osim.ForceReporter(copy)
    copy.addAnalysis(reporter)
    fwd_tool = osim.ForwardTool()
    fwd_tool.setModel(copy)
    fwd_tool.setFinalTime(3)
    
    #gets the geometry path of the right spring
    baseR = frcSet.get(forceSetIndex[0])
    right = osim.PathSpring.safeDownCast(baseR)
    
    #sets the stiffness of the right spring
    right.setStiffness(ks)
    
    #gets the geometry path of the left spring
    baseL = frcSet.get(forceSetIndex[1])
    left = osim.PathSpring.safeDownCast(baseL)
    
    #sets the stiffness of the left spring
    left.setStiffness(ks)
        
    angles = [0, 7.5, 15]
    forces = [(0,0),(0,0),(0,0)]
    
    for i in range(0,3):
        
        coord = model.getCoordinateSet()
        d = coord.get(0)
        d.set_default_value(angles[i]*math.pi/180)
        
        fwd_tool.run()
    
        storage = reporter.getForceStorage()
        stateVec = storage.getLastStateVector()
        dataSet = stateVec.getData()
        force1 = dataSet.get(f1_index)
        force2 = dataSet.get(f2_index)
        
        forces[i] = (force1, force2)
    
    return (math.fabs(forces[0][0]) - math.fabs(forces[0][1]))**2 + (math.fabs(forces[1][0]) - math.fabs(forces[1][1]))**2 + (math.fabs(forces[2][0]) - math.fabs(forces[2][1]))**2 
    
def main():
    x0 = [1.4, 50]
    sol = minimize(optimizer_callBack, x0, method = 'Nelder-Mead', tol = 5)
    print(sol)
    
    
main()
