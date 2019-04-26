# -*- coding: utf-8 -*-
"""
Created on Tue Apr  9 14:07:58 2019

@author: mhmdk
"""

import opensim as osim
from controls_writer import controls_writer

model = osim.Model("C:/Users/mhmdk/Desktop/Co-op files/co-op semester 1/optimizers_2/friction/hangingHarnessModel_fric_lim.osim")
model.printToXML("C:/Users/mhmdk/Desktop/Co-op files/co-op semester 1/optimizers_2/friction/copy_hangingHarnessModel_fric_lim.osim")
copy = osim.Model("C:/Users/mhmdk/Desktop/Co-op files/co-op semester 1/optimizers_2/friction/copy_hangingHarnessModel_fric_lim.osim")
frcSet = copy.getForceSet()

def opt_changer(index, val):
    frcBase = frcSet.get(index)
    frc = osim.PathActuator.safeDownCast(frcBase)
    frc.setOptimalForce(val)

copy.initSystem()
reporter = osim.ForceReporter(copy)
copy.addAnalysis(reporter)
fwd_tool = osim.ForwardTool()
fwd_tool.setModel(copy)
file_name = "control_lim.xml"
controls_writer(file_name, 1)
friction = -5
opt_changer(7, friction)
opt_changer(8, friction)
fwd_tool.setControlsFileName(file_name)
fwd_tool.addControllerSetToModel()
fwd_tool.setFinalTime(3)
  
copy.printToXML("C:/Users/mhmdk/Desktop/Co-op files/co-op semester 1/optimizers_2/friction/copy_hangingHarnessModel_fric_lim.osim")
    
fwd_tool.run()

storage = reporter.getForceStorage()
lbls = storage.getColumnLabels()
stateVec = storage.getLastStateVector()
dataSet = stateVec.getData()

for i in range (1, 11):
    index = i + 1
    print(lbls.get(index) + ": " + str(dataSet.get(i)))

