# -*- coding: utf-8 -*-
"""
Created on Tue Feb  5 10:47:24 2019

@author: mhmdk
"""

import math
import opensim as osim

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

def get_forces(frcSet):
    baseR = frcSet.get(1)
    right = osim.PathSpring.safeDownCast(baseR)
    
    baseL = frcSet.get(2)
    left = osim.PathSpring.safeDownCast(baseL)
    
    return (right, left)

def get_Geometry(forces):
    geoR = forces[0].upd_GeometryPath()
    ppset_r = geoR.updPathPointSet()
    
    point_start_r = ppset_r.get(0)
    point_end_r = ppset_r.get(1)
    
    geoL = forces[1].upd_GeometryPath()
    ppset_l = geoL.updPathPointSet()
    
    point_start_l = ppset_l.get(0)
    point_end_l = ppset_l.get(1)
    
    return((point_start_r, point_end_r), (point_start_l, point_end_l))
    
    