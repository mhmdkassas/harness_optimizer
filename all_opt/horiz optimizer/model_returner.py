# -*- coding: utf-8 -*-
"""
Created on Mon Feb 11 10:04:41 2019

@author: mhmdk
"""

import opensim as osim

#TODO: YOU MUST CHANGE THE PATHS IN THIS FILE

#this assumes that the necessary files are in the following paths
def comDir():
    return "C:\Users\mhmdk\Desktop\Co-op files\co-op semester 1\optimizers\harnessOptimizers\horiz optimizer"

def modelList():
    return ["hangingHarnessModel_horiz.osim", "loadingHarnessModel_horiz.osim"]

def returnOrigPath(index):
    arr = modelList()
    return comDir() + "\\" + arr[index]

def returnCopyPath(index):
    arr = modelList()
    return comDir() + "\\copy_" + arr[index]

#this is getting the column correspdonging to force1 and force2
def f1Index(index):
    if index == 0:
        return 2
    else: 
        return 4
    
def f2Index():
        return 8
def frcSet(index):
    if index == 0:
        return (5,6)
    else:
        return (1,2)

def modelOrigReturner(iD):
    return osim.Model(returnOrigPath(index = iD))

def modelForces(index):
    model = modelOrigReturner(iD = index)
    path = returnCopyPath(index = index)
    model.printToXML(path)
    copy = osim.Model(path)
    
    if index == 0:
        modelType = "hanging"
    elif index == 1:
        modelType = "loading"
    
    return [copy, path, f1Index(index = index), f2Index(), modelType, model, frcSet(index = index)]


