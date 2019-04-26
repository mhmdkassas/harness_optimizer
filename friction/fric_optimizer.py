# -*- coding: utf-8 -*-
"""
Created on Mon Apr  8 16:09:04 2019

@author: mhmdk
"""

import opensim as osim
from controlsWriter import controls_writer
from scipy.optimize import minimize
import math
import os

model = osim.Model("C:/Users/mhmdk/Desktop/Co-op files/co-op semester 1/optimizers_2/friction/hangingHarnessModel_fric.osim")

def opt_changer(index, val, frcSet):
    frcBase = frcSet.get(index)
    frc = osim.PathActuator.safeDownCast(frcBase)
    frc.setOptimalForce(val)
    
file_name = "cont1.xml"
controls_writer(file_name, 1)

    
def optimizer_callBack(x):
    
    model.printToXML("C:/Users/mhmdk/Desktop/Co-op files/co-op semester 1/optimizers_2/friction/copy_hangingHarnessModel_fric.osim")
    copy = osim.Model("C:/Users/mhmdk/Desktop/Co-op files/co-op semester 1/optimizers_2/friction/copy_hangingHarnessModel_fric.osim")
    frcSet = copy.getForceSet()
    
    opt_changer(7, x[0]*10, frcSet)
    opt_changer(8, x[0]*10, frcSet)
    copy.printToXML("C:/Users/mhmdk/Desktop/Co-op files/co-op semester 1/optimizers_2/friction/copy_hangingHarnessModel_fric.osim")
    
    copy.initSystem()
    reporter = osim.ForceReporter(copy)
    copy.addAnalysis(reporter)
    fwd_tool = osim.ForwardTool()
    fwd_tool.setModel(copy)
    fwd_tool.setControlsFileName(file_name)
    fwd_tool.addControllerSetToModel()
    fwd_tool.setFinalTime(3)
    
    baseR = frcSet.get(1)
    rightLeg = osim.PathSpring.safeDownCast(baseR)
    rightLeg.setStiffness(x[1]*10000)
        
    baseL = frcSet.get(2)
    leftLeg = osim.PathSpring.safeDownCast(baseL)
    leftLeg.setStiffness(x[1]*10000)
    
    fwd_tool.run()
    
    storage = reporter.getForceStorage()
    stateVec = storage.getLastStateVector()
    dataSet = stateVec.getData()
    force1 = dataSet.get(4)
    force2 = dataSet.get(6)
    force3 = dataSet.get(8)
    
    os.remove("copy_hangingHarnessModel_fric.osim")
    
    print("force1: " + str(force1))
    print("force2: " + str(force2))
    print("force3: " + str(force3))
    
    print(force3**2)
    print(x[0])
    
    return  1000*x[0]**2 + (math.floor(float(force1)) - math.floor(float(force2)))**2

def main():
    x0 = [6, 0.5]
    sol = minimize(optimizer_callBack, x0, method = "nelder-mead", tol = 1)
    print(sol)
    
    
main()