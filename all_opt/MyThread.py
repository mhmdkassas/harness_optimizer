# -*- coding: utf-8 -*-
"""
Created on Wed Feb  6 10:11:52 2019

@author: mhmdk
"""

import threading
import opensim as osim

exitFlag = 0

unloading_path = "C:\Users\mhmdk\Desktop\Co-op files\co-op semester 1\harnessOptimizers\hangingHarnessModel.osim"
unloading_copy_path1 = "C:\Users\mhmdk\Desktop\Co-op files\co-op semester 1\harnessOptimizers\copy_hangingHarnessModel1.osim"
unloading_copy_path2 = "C:\Users\mhmdk\Desktop\Co-op files\co-op semester 1\harnessOptimizers\copy_hangingHarnessModel2.osim"
unloading_copy_path3 = "C:\Users\mhmdk\Desktop\Co-op files\co-op semester 1\harnessOptimizers\copy_hangingHarnessModel3.osim"
unloading_copy_path4 = "C:\Users\mhmdk\Desktop\Co-op files\co-op semester 1\harnessOptimizers\copy_hangingHarnessModel4.osim"
unloading_copy_path5 = "C:\Users\mhmdk\Desktop\Co-op files\co-op semester 1\harnessOptimizers\copy_hangingHarnessModel5.osim"

model = osim.Model(unloading_path)
    
model.printToXML(unloading_copy_path1)
copy1 = osim.Model(unloading_copy_path1)
    
model.printToXML(unloading_copy_path2)
copy2 = osim.Model(unloading_copy_path2)
    
model.printToXML(unloading_copy_path3)
copy3 = osim.Model(unloading_copy_path3)

model.printToXML(unloading_copy_path4)
copy4 = osim.Model(unloading_copy_path4)

model.printToXML(unloading_copy_path5)
copy5 = osim.Model(unloading_copy_path5)

copies = [copy1, copy2, copy3, copy4, copy5]

class MyThread(threading.Thread):
    
    def __init__(self, model, name):
        threading.Thread.__init__(self)
        self.model = model
        self.name = name
        
    def run(self):
        print("starting: " + str(self.name))
        frwd_runner(self.model)
        print("end: " + str(self.name))
        
def frwd_runner(model):
    model.initSystem()
    reporter = osim.ForceReporter(model)
    model.addAnalysis(reporter)
    fwd_tool = osim.ForwardTool()
    fwd_tool.setModel(model)
    fwd_tool.setFinalTime(3)  

    fwd_tool.run()      


# Create new threads
threads = [0,0,0,0,0]
for i in range (0,5):
    threads[i] = MyThread(copies[i], i)

for i in range(0,5):
    threads[i].start()

print "Exiting Main Thread"