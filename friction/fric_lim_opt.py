# -*- coding: utf-8 -*-
"""
Created on Tue Apr  9 14:10:35 2019

@author: mhmdk
"""

import opensim as osim
from controls_writer import controls_writer
import math
from scipy.optimize import minimize
import os

model = osim.Model("C:/Users/mhmdk/Desktop/Co-op files/co-op semester 1/optimizers_2/friction/hangingHarnessModel_fric_lim.osim")

def opt_changer(index, val, frcSet):
    frcBase = frcSet.get(index)
    frc = osim.PathActuator.safeDownCast(frcBase)
    frc.setOptimalForce(val)

def k_changer(index, val, frcSet):
    frcBase = frcSet.get(index)
    frc = osim.PathSpring.safeDownCast(frcBase)
    frc.setStiffness(val)  
    
file_name = "cont1.xml"
controls_writer(file_name, 1)

arr = [0,0,0,0,0,0,0,0,0,0,0]

def optimizer_callBack(x):
    
    model.printToXML("C:/Users/mhmdk/Desktop/Co-op files/co-op semester 1/optimizers_2/friction/copy_hangingHarnessModel_fric_lim.osim")
    copy = osim.Model("C:/Users/mhmdk/Desktop/Co-op files/co-op semester 1/optimizers_2/friction/copy_hangingHarnessModel_fric_lim.osim")
    frcSet = copy.getForceSet()

    copy.initSystem()
    reporter = osim.ForceReporter(copy)
    copy.addAnalysis(reporter)
    fwd_tool = osim.ForwardTool()
    fwd_tool.setModel(copy)
    file_name = "cont1.xml"
    controls_writer(file_name, x[0]*10)
    opt_changer(7, 1, frcSet)
    opt_changer(8, 1, frcSet)
    fwd_tool.setControlsFileName(file_name)
    fwd_tool.addControllerSetToModel()
    fwd_tool.setFinalTime(3)
    
    baseR = frcSet.get(1)
    rightLeg = osim.PathSpring.safeDownCast(baseR)
    rightLeg.setStiffness(x[1]*10000)
        
    baseL = frcSet.get(2)
    leftLeg = osim.PathSpring.safeDownCast(baseL)
    leftLeg.setStiffness(x[1]*10000)
    
    copy.printToXML("C:/Users/mhmdk/Desktop/Co-op files/co-op semester 1/optimizers_2/friction/copy_hangingHarnessModel_fric.osim")
    
    fwd_tool.run()
    
    storage = reporter.getForceStorage()
    lbls = storage.getColumnLabels()
    stateVec = storage.getLastStateVector()
    dataSet = stateVec.getData()
    
    os.remove("copy_hangingHarnessModel_fric_lim.osim")
    
    print("\nLoop: \n")
    for i in range(1, 11):
        arr_ind = i - 1
        lbls_ind = i + 1
        arr[arr_ind] = dataSet.get(i)
        print(lbls.get(lbls_ind) + ": " + str(dataSet.get(i)))
    
    return  1000*x[0]**2 + (math.floor(float(arr[2])) - math.floor(float(arr[4])))**2 + 1000*dataSet.get(9)**2

def main():
    x0 = [6, 0.5]
    sol = minimize(optimizer_callBack, x0, method = "nelder-mead", tol = 1)
    print(sol)
main()