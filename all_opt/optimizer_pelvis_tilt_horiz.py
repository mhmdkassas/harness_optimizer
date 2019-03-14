# -*- coding: utf-8 -*-
"""
Created on Mon Feb 11 09:35:18 2019

@author: mhmdk
"""

import opensim as osim
import math
from scipy.optimize import minimize


#CONSTANTS REPRESENTING LOADING AND UNLOADING
UNLOADING_FLAG = 0
LOADING_FLAG = 1

#CHOOSE LOADING OR UNLOADING
flag_model = UNLOADING_FLAG

#TODO: ensure correct path for your files
unloading_path = "C:\Users\mhmdk\Desktop\Co-op files\co-op semester 1\optimizers\harnessOptimizers\hangingHarnessModel_horiz.osim"
unloading_copy_path = "C:\Users\mhmdk\Desktop\Co-op files\co-op semester 1\optimizers\harnessOptimizers\copy_hangingHarnessModel_horiz.osim"

loading_path = "C:\Users\mhmdk\Desktop\Co-op files\co-op semester 1\optimizers\harnessOptimizers\loadingHarnessModel_horiz.osim"
loading_copy_path = "C:\Users\mhmdk\Desktop\Co-op files\co-op semester 1\optimizers\harnessOptimizers\copy_loadingHarnessModel_horiz.osim"


#loads the correct model according to flag
if flag_model == UNLOADING_FLAG:
    model = osim.Model(unloading_path)
    
    model.printToXML(unloading_copy_path)
    copy = osim.Model(unloading_copy_path)
    
    f1_index = 2
    f2_index = 8
    
elif flag_model == LOADING_FLAG:
    model = osim.Model(loading_path)
    
    model.printToXML(loading_copy_path)
    copy = osim.Model(loading_copy_path)
    
    f1_index = 4  
    f2_index = 8

print("model loaded")
#gets force set of the specified model
frcSet = copy.getForceSet()
    

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
    baseR = frcSet.get(1)
    right = osim.SpringGeneralizedForce.safeDownCast(baseR)
    
    
    #sets the stiffness of the right spring
    right.setStiffness(ks)
    
    #gets the geometry path of the left spring
    baseL = frcSet.get(2)
    left = osim.SpringGeneralizedForce.safeDownCast(baseL)
    
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
    x0 = [1.4]
    sol = minimize(optimizer_callBack, x0, method = 'Nelder-Mead', tol = 5)
    print(sol)
    
    
main()