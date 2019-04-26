# -*- coding: utf-8 -*-
"""
Created on Tue Apr  9 10:48:50 2019

@author: mhmdk
"""
import opensim as osim
from controls_writer import controls_writer

model = osim.Model("C:/Users/mhmdk/Desktop/Co-op files/co-op semester 1/optimizers_2/friction/hangingHarnessModel_fric.osim")
model.printToXML("C:/Users/mhmdk/Desktop/Co-op files/co-op semester 1/optimizers_2/friction/copy_hangingHarnessModel_fric.osim")
copy = osim.Model("C:/Users/mhmdk/Desktop/Co-op files/co-op semester 1/optimizers_2/friction/copy_hangingHarnessModel_fric.osim")
frcSet = copy.getForceSet()

def opt_changer(index, val):
    frcBase = frcSet.get(index)
    frc = osim.PathActuator.safeDownCast(frcBase)
    frc.setOptimalForce(val)

file_name = "control.xml"
controls_writer(file_name, 1)

copy.initSystem()
reporter = osim.ForceReporter(copy)
copy.addAnalysis(reporter)
fwd_tool = osim.ForwardTool()
fwd_tool.setModel(copy)

#change the value of the friction below
friction = 50
opt_changer(7, friction)
opt_changer(8, friction)
fwd_tool.setControlsFileName(file_name)
fwd_tool.addControllerSetToModel()
fwd_tool.setFinalTime(3)
  
copy.printToXML("C:/Users/mhmdk/Desktop/Co-op files/co-op semester 1/optimizers_2/friction/copy_hangingHarnessModel_fric.osim")
    
fwd_tool.run()

storage = reporter.getForceStorage()
lbls = storage.getColumnLabels()
stateVec = storage.getLastStateVector()
dataSet = stateVec.getData()


for i in range (1, 9):
    index = i + 1
    print(lbls.get(index) + ": " + str(dataSet.get(i)))
