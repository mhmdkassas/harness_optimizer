# -*- coding: utf-8 -*-
"""
Created on Fri Jan 25 15:54:09 2019

@author: mhmdk
"""

import opensim as osim
import math
from scipy.optimize import minimize


model = osim.Model("\Users\mhmdk\Desktop\Co-op files\co-op semester 1\optimizers_2\hangingHarnessModel\hangingHarnessModel.osim")
model.printToXML("\Users\mhmdk\Desktop\Co-op files\co-op semester 1\optimizers_2\hangingHarnessModel\copy_hangingHarnessModel.osim")
copy = osim.Model("\Users\mhmdk\Desktop\Co-op files\co-op semester 1\optimizers_2\hangingHarnessModel\copy_hangingHarnessModel.osim")
frcSet = copy.getForceSet()

#all the indices representing the individual forces are
#dependent on their location in the osim file above
#MAKE SURE THEY CORRESPOND CORRECTLY

def external_tension(num):
    stiffness = float(num)
    tensionBase = frcSet.get(0)
    tension = osim.PathSpring.safeDownCast(tensionBase)
     
    tension.setStiffness(stiffness)
    
def right_leg_hip(num):
    stiffness = float(num)
    leg_hipBase = frcSet.get(1)
    leg_hip = osim.PathSpring.safeDownCast(leg_hipBase)
    
    leg_hip.setStiffness(stiffness)
    
def left_leg_hip(num):
    stiffness = float(num)
    leg_hipBase = frcSet.get(2)
    leg_hip = osim.PathSpring.safeDownCast(leg_hipBase)
    
    leg_hip.setStiffness(stiffness)
    
def right_hip_body(num):
    stiffness = float(num)
    hip_bodyBase = frcSet.get(3)
    hip_body = osim.PathSpring.safeDownCast(hip_bodyBase)
    
    hip_body.setStiffness(stiffness)
    
def left_hip_body(num):
    stiffness = float(num)
    hip_bodyBase = frcSet.get(4)
    hip_body = osim.PathSpring.safeDownCast(hip_bodyBase)
    
    hip_body.setStiffness(stiffness)
    
def right_leg_body(num):
    stiffness = float(num)
    leg_bodyBase = frcSet.get(5)
    leg_body = osim.PathSpring.safeDownCast(leg_bodyBase)
    
    leg_body.setStiffness(stiffness)
    
def left_leg_body(num):
    stiffness = float(num)
    leg_bodyBase = frcSet.get(6)
    leg_body = osim.PathSpring.safeDownCast(leg_bodyBase)
    
    leg_body.setStiffness(stiffness)
    
def normalize(arr, arrNorm):
    newArr = [0, 0, 0]
    for i in range(0,3):
        newArr[i] = arr[i]+arrNorm[i]
    return newArr

#Bodies
hip_pelvis = [-0.13, 0.15, 0]
legR_femurR = [0.01, -0.2, 0.05]
legL_femurL = [0.01, -0.2, -0.05]
femurR_pelvis = [-0.072437806, - 0.067724738, 0.085552]
femurL_pelvis = [-0.072437806, - 0.067724738, -0.085552]
    
#PathPoints and PathSprings    
legR_start = [0.0675622, 0, 0.13552]
legR_end = [0, 0, 0]
    
legL_start = [0.0675622, 0, -0.13552]
legL_end = [0, 0, 0]
    
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

lenLegOld = math.fabs(legR_strap[0][1] - legR_strap[1][1])

def angle_shifter(length, tup, angle, s):
    #angle is in radians
    new_z = 0
    if s == "r":
        new_z = tup[1][2] + length*(math.sin(angle)+math.tan(angle)*(1-math.cos(angle)))
    elif s == "l":
        new_z = tup[1][2] - length*(math.sin(angle)+math.tan(angle)*(1-math.cos(angle)))    
    new_arr = [tup[0][0], tup[1][1], new_z]
    new_tup = (tup[0], new_arr)
    return new_tup

def length_calc(tup):
    
    delta_x = tup[0][0]-tup[1][0]
    delta_y = tup[0][1]-tup[1][1]
    delta_z = tup[0][2]-tup[1][2]
    
    return math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)

def refactor(arr, arrRef):
    newArr = [0, 0, 0]
    for i in range(0,3):
        newArr[i] = arr[i] - arrRef[i]
    return newArr

def optimizer_callBack(x):
    
    ks = x[0]*10000
    angleRad = 0
    
    copy.initSystem()
    reporter = osim.ForceReporter(copy)
    copy.addAnalysis(reporter)
    fwd_tool = osim.ForwardTool()
    fwd_tool.setModel(copy)
    fwd_tool.setFinalTime(3)
    
    shifted_strap_right = angle_shifter(length = length_calc(legR_strap), angle = angleRad, s = "r", tup = legR_strap)
    shifted_strap_left = angle_shifter(length = length_calc(legL_strap), angle = angleRad, s = "l", tup = legL_strap)
    
    len_new = length_calc(shifted_strap_right)
    
    right_start = shifted_strap_right[0]
    right_end = shifted_strap_right[1]
    rs = refactor(arr = right_start, arrRef = hip_pelvis)
    rE = refactor(arr = right_end, arrRef = legR_pelvis)
    
    left_start = shifted_strap_left[0]
    left_end = shifted_strap_left[1]
    ls = refactor(arr = left_start, arrRef = hip_pelvis)
    le = refactor(arr = left_end, arrRef = legL_pelvis)
    
    vecRs = osim.Vec3(rs[0], rs[1], rs[2])
    vecRe = osim.Vec3(rE[0], rE[1], rE[2])
    vecLs = osim.Vec3(ls[0], ls[1], ls[2])
    vecLe = osim.Vec3(le[0], le[1], le[2])
    
    baseR = frcSet.get(1)
    rightLeg = osim.PathSpring.safeDownCast(baseR)
    geoR = rightLeg.upd_GeometryPath()
    ppset_r = geoR.updPathPointSet()
    
    point_start_r = ppset_r.get(0)
    point_end_r = ppset_r.get(1)
    
    rightLeg.setRestingLength(len_new)
    rightLeg.setStiffness(ks)
    
    point_start_r.setLocation(copy.initSystem(), vecRs)
    point_end_r.setLocation(copy.initSystem(), vecRe)
    
    baseL = frcSet.get(2)
    leftLeg = osim.PathSpring.safeDownCast(baseL)
    geoL = leftLeg.upd_GeometryPath()
    ppset_l = geoL.updPathPointSet()
    
    point_start_l = ppset_l.get(0)
    point_end_l = ppset_l.get(1)
    
    leftLeg.setRestingLength(len_new)
    leftLeg.setStiffness(ks)
    
    point_start_l.setLocation(copy.initSystem(), vecLs)
    point_end_l.setLocation(copy.initSystem(), vecLe)
    
    copy.printToXML("C:\Users\mhmdk\Desktop\Co-op files\co-op semester 1\optimizers_2\hangingHarnessModel\copy_hangingHarnessModel.osim")
    
    fwd_tool.run()
    
    storage = reporter.getForceStorage()
    stateVec = storage.getLastStateVector()
    dataSet = stateVec.getData()
    force1 = dataSet.get(3)
    force2 = dataSet.get(5)
    
    return  (math.floor(float(force1))-math.floor(float(force2)))**2 

def main():
    x0 = [1.4]
    sol = minimize(optimizer_callBack, x0, method = 'Nelder-Mead', tol = 1)
    print(sol)
    
main()
    
    
    
    
    
    
    