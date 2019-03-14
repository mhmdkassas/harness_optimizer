# -*- coding: utf-8 -*-
"""
Created on Tue Feb  5 10:02:39 2019

@author: mhmdk
"""

import opensim as osim
import math
from vector_operations import normalize, refactor
from geometry_func import length_calc, angle_shifter
from scipy.optimize import minimize

#CONSTANTS REPRESENTING LOADING AND UNLOADING
UNLOADING_FLAG = 0
LOADING_FLAG = 1

#CHOOSE LOADING OR UNLOADING
flag_model = LOADING_FLAG

#TODO: ensure correct path for your files
unloading_path = "C:\Users\mhmdk\Desktop\Co-op files\co-op semester 1\harnessOptimizers\hangingHarnessModel.osim"
unloading_copy_path = "C:\Users\mhmdk\Desktop\Co-op files\co-op semester 1\harnessOptimizers\copy_hangingHarnessModel.osim"

loading_path = "C:\Users\mhmdk\Desktop\Co-op files\co-op semester 1\harnessOptimizers\loadingHarnessModel.osim"
loading_copy_path = "C:\Users\mhmdk\Desktop\Co-op files\co-op semester 1\harnessOptimizers\copy_loadingHarnessModel.osim"

#loads the correct model according to flag
if flag_model == UNLOADING_FLAG:
    model = osim.Model(unloading_path)
    model.printToXML(unloading_copy_path)
    copy = osim.Model(unloading_copy_path)
    
elif flag_model == LOADING_FLAG:
    model = osim.Model(loading_path)
    model.printToXML(loading_copy_path)
    copy = osim.Model(loading_copy_path)

#gets force set of the specified model
frcSet = copy.getForceSet()
    
#Bodies
hip_pelvis = [-0.13, 0.15, 0]
shR_torso = [-0.025, 0.4, 0.17]
shL_torso = [-0.025, 0.4, -0.17]
torso_pelvis = [-0.103175206, 0.08350327, 0]
legR_femurR = [0.01, -0.2, 0.05]
legL_femurL = [0.01, -0.2, -0.05]
femurR_pelvis = [-0.072437806, - 0.067724738, 0.085552]
femurL_pelvis = [-0.072437806, - 0.067724738, -0.085552]
    
#PathPoints and PathSprings
shR_start = [0, 0, 0.17]
shR_end = [0, 0, 0]
    
shL_start = [0, 0, -0.17]
shL_end = [0, 0, 0]

legR_start = [0.0675622, 0, 0.13552]
legR_end = [0, 0, 0]
    
legL_start = [0.0675622, 0, -0.13552]
legL_end = [0, 0, 0]

#lenOld = 0
    
if flag_model == UNLOADING_FLAG:
    
    #Makes all the bodies in reference to the pelvis
    legR_pelvis = normalize(arr = legR_femurR, arrNorm = femurR_pelvis)
    legL_pelvis = normalize(arr = legL_femurL, arrNorm = femurL_pelvis)
        
    #Makes all the PathPoints and PathSpring in reference to the pelvis    
    legR_start_norm = normalize(arr = legR_start, arrNorm = hip_pelvis)
    legR_end_norm = normalize(arr = legR_end, arrNorm = legR_pelvis)
    legR_strap = (legR_start_norm, legR_end_norm)
        
    legL_start_norm = normalize(arr = legL_start, arrNorm = hip_pelvis)
    legL_end_norm = normalize(arr = legL_end, arrNorm = legL_pelvis)
    legL_strap = (legL_start_norm, legL_end_norm)
    
    lenOld = math.fabs(legR_strap[0][1] - legR_strap[1][1])
    
elif flag_model == LOADING_FLAG:
    
    #Makes all the bodies in reference to the pelvis
    shR_pelvis = normalize(arr = shR_torso, arrNorm = torso_pelvis)
    shL_pelvis = normalize(arr = shL_torso, arrNorm = torso_pelvis)
    
    #Makes all the PathPoints and PathSpring in reference to the pelvis
    shR_start_norm = normalize(arr = shR_start, arrNorm = hip_pelvis)
    shR_end_norm = normalize(arr = shR_end, arrNorm = shR_pelvis)
    shR_strap = (shR_start_norm, shR_end_norm)
        
    shL_start_norm = normalize(arr = shL_start, arrNorm = hip_pelvis)
    shL_end_norm = normalize(arr = shL_end, arrNorm = shL_pelvis)
    shL_strap = (shL_start_norm, shL_end_norm)
    
    lenOld = math.fabs(shR_strap[0][1] - shR_strap[1][1])
    
def optimizer_callBack(x):
    
    #the parameters that the optimizer changes
    ks = x[0]*1000
    angleRad = x[1]*math.pi/180
    
    #initializes the system and sets up forward dynamics tool
    copy.initSystem()
    reporter = osim.ForceReporter(copy)
    copy.addAnalysis(reporter)
    fwd_tool = osim.ForwardTool()
    fwd_tool.setModel(copy)
    fwd_tool.setFinalTime(3)
    
    if flag_model == UNLOADING_FLAG:
        shifted_strap_right = angle_shifter(length = length_calc(legR_strap), angle = angleRad, s = "r", tup = legR_strap)
        shifted_strap_left = angle_shifter(length = length_calc(legL_strap), angle = angleRad, s = "l", tup = legL_strap)
    elif flag_model == LOADING_FLAG:
        shifted_strap_right = angle_shifter(length = length_calc(shR_strap), angle = angleRad, s = "r", tup = shR_strap)
        shifted_strap_left = angle_shifter(length = length_calc(shL_strap), angle = angleRad, s = "l", tup = shL_strap)
    
    len_new = length_calc(shifted_strap_right)
    
    right_start = shifted_strap_right[0]
    right_end = shifted_strap_right[1]
    left_start = shifted_strap_left[0]
    left_end = shifted_strap_left[1]
    
    rs =  refactor(arr = right_start, arrRef = hip_pelvis)
    ls = refactor(arr = left_start, arrRef = hip_pelvis)
    
    if flag_model == UNLOADING_FLAG:
        rE = refactor(arr = right_end, arrRef = legR_pelvis)
        le = refactor(arr = left_end, arrRef = legL_pelvis)
        
    elif flag_model == LOADING_FLAG:
        rE = refactor(arr = right_end, arrRef = shR_pelvis)
        le = refactor(arr = left_end, arrRef = shL_pelvis)
    
    
    #creates 3-D vectors
    vecRs = osim.Vec3(rs[0], rs[1], rs[2])
    vecRe = osim.Vec3(rE[0], rE[1], rE[2])
    vecLs = osim.Vec3(ls[0], ls[1], ls[2])
    vecLe = osim.Vec3(le[0], le[1], le[2])
    
    #gets the geometry path of the right spring
    baseR = frcSet.get(1)
    right = osim.PathSpring.safeDownCast(baseR)
    geoR = right.upd_GeometryPath()
    ppset_r = geoR.updPathPointSet()
    
    #changes the start and end of the right spring
    point_start_r = ppset_r.get(0)
    point_end_r = ppset_r.get(1)
    
    #sets the stiffness and the resting length of the right spring
    right.setRestingLength(len_new)
    right.setStiffness(ks)
    
    #sets location of the start and end of the right spring
    point_start_r.setLocation(copy.initSystem(), vecRs)
    point_end_r.setLocation(copy.initSystem(), vecRe)
    
    #gets the geometry path of the left spring
    baseL = frcSet.get(2)
    left = osim.PathSpring.safeDownCast(baseL)
    geoL = left.upd_GeometryPath()
    ppset_l = geoL.updPathPointSet()
    
    #changes the start and end of the left spring
    point_start_l = ppset_l.get(0)
    point_end_l = ppset_l.get(1)
    
    #sets the stiffness and the resting length of the left spring
    left.setRestingLength(len_new)
    left.setStiffness(ks)
    
    #sets location of the start and end of the left spring
    point_start_l.setLocation(copy.initSystem(), vecLs)
    point_end_l.setLocation(copy.initSystem(), vecLe)
    
    if flag_model == UNLOADING_FLAG:
        copy.printToXML(unloading_copy_path)        
    elif flag_model == LOADING_FLAG:
        copy.printToXML(loading_copy_path)
        
    fwd_tool.run()
    
    storage = reporter.getForceStorage()
    stateVec = storage.getLastStateVector()
    dataSet = stateVec.getData()
    force1 = dataSet.get(3)
    force2 = dataSet.get(5)
    
    return ( (math.floor(float(force1))-math.floor(float(force2)))**2 )
    
def main():
    x0 = [1.5, 0]
    sol = minimize(optimizer_callBack, x0, method = 'Nelder-Mead', tol = 1)
    print(sol)
    
    
main()
    
    
    