# -*- coding: utf-8 -*-
"""
Created on Tue Apr 23 16:38:44 2019

@author: mhmdk
"""

import opensim as osim
import math
from scipy.optimize import minimize


#TODO: ensure correct path for your files
unloading_path = "C:/Users/mhmdk/Desktop/Co-op files/co-op semester 1/optimizers_2/all_opt/hangingHarnessModel_lim_hip.osim"
unloading_copy_path = "C:/Users/mhmdk/Desktop/Co-op files/co-op semester 1/optimizers_2/all_opt/copy_hangingHarnessModel_lim_hip.osim"

model = osim.Model(unloading_path)
    
model.printToXML(unloading_copy_path)
copy = osim.Model(unloading_copy_path)

frcSet = copy.getForceSet()

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
shR_torso = [-0.025, 0.4, 0.17]
shL_torso = [-0.025, 0.4, -0.17]
torso_pelvis = [-0.103175206, 0.08350327, 0]
    
#PathPoints and PathSprings
shR_start = [0, 0, 0.17]
shR_end = [0, 0, 0]
    
shL_start = [0, 0, -0.17]
shL_end = [0, 0, 0]
    
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
    
    ks = 39810
    angleRad = x[0]*math.pi/180
    
    copy.initSystem()
    reporter = osim.ForceReporter(copy)
    copy.addAnalysis(reporter)
    fwd_tool = osim.ForwardTool()
    fwd_tool.setModel(copy)
    fwd_tool.setFinalTime(3)
    
    shifted_strap_right = angle_shifter(length = length_calc(shR_strap), angle = angleRad, s = "r", tup = shR_strap)
    shifted_strap_left = angle_shifter(length = length_calc(shL_strap), angle = angleRad, s = "l", tup = shL_strap)
    
    len_new = length_calc(shifted_strap_right)
    
    right_start = shifted_strap_right[0]
    right_end = shifted_strap_right[1]
    rs = refactor(arr = right_start, arrRef = hip_pelvis)
    rE = refactor(arr = right_end, arrRef = shR_pelvis)
    
    left_start = shifted_strap_left[0]
    left_end = shifted_strap_left[1]
    ls = refactor(arr = left_start, arrRef = hip_pelvis)
    le = refactor(arr = left_end, arrRef = shL_pelvis)
    
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
    
    copy.printToXML("C:/Users/mhmdk/Desktop/Co-op files/co-op semester 1/optimizers_2/all_opt/copy_hangingHarnessModel_lim_hip.osim")

    angles = [0, 7.5, 15]
    forces = [(0,0),(0,0),(0,0)]
    force_lim = [0, 0, 0]

    for i in range(0,3):
        
        coord = model.getCoordinateSet()
        d = coord.get(0)
        d.set_default_value(angles[i]*math.pi/180)
        
        fwd_tool.run()
        
        storage = reporter.getForceStorage()
        stateVec = storage.getLastStateVector()
        dataSet = stateVec.getData()
        force1 = dataSet.get(4)
        force2 = dataSet.get(6)
        force3 = dataSet.get(8)
        
        forces[i] = (force1, force2)
        force_lim[i] = force3
    print(forces)
    return (forces[0][0] - forces[0][1])**2 + (forces[1][0] - forces[1][1])**2 + (forces[2][0] - forces[2][1])**2 + force_lim[0] + force_lim[1] + force_lim[2]

def main():
    x0 = [45]
    sol = minimize(optimizer_callBack, x0, method = 'Nelder-Mead', tol = 5)
    print(sol)
    
    
main()