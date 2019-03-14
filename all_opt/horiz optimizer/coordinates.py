# -*- coding: utf-8 -*-
"""
Created on Mon Feb 11 11:05:58 2019

@author: mhmdk
"""

from vector_operations import normalize, refactor

HANGING_INDEX= 0
LOADING_INDEX = 1
    
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

#Makes all the bodies in reference to the pelvis
shR_pelvis = normalize(arr = shR_torso, arrNorm = torso_pelvis)
shL_pelvis = normalize(arr = shL_torso, arrNorm = torso_pelvis)

legR_pelvis = normalize(arr = legR_femurR, arrNorm = femurR_pelvis)
legL_pelvis = normalize(arr = legL_femurL, arrNorm = femurL_pelvis)

def getStraps(modelType):
    if modelType == "hanging":
        typeIndex = HANGING_INDEX
    else:
        typeIndex = LOADING_INDEX
        
    if typeIndex == HANGING_INDEX:
            
        #Makes all the PathPoints and PathSpring in reference to the pelvis    
        legR_start_norm = normalize(arr = legR_start, arrNorm = hip_pelvis)
        legR_end_norm = normalize(arr = legR_end, arrNorm = legR_pelvis)
        legR_strap = (legR_start_norm, legR_end_norm)
            
        legL_start_norm = normalize(arr = legL_start, arrNorm = hip_pelvis)
        legL_end_norm = normalize(arr = legL_end, arrNorm = legL_pelvis)
        legL_strap = (legL_start_norm, legL_end_norm)
        
        strapR = legR_strap
        strapL = legL_strap     
   
    
    elif typeIndex == LOADING_INDEX:
        
        #Makes all the PathPoints and PathSpring in reference to the pelvis
        shR_start_norm = normalize(arr = shR_start, arrNorm = hip_pelvis)
        shR_end_norm = normalize(arr = shR_end, arrNorm = shR_pelvis)
        shR_strap = (shR_start_norm, shR_end_norm)
            
        shL_start_norm = normalize(arr = shL_start, arrNorm = hip_pelvis)
        shL_end_norm = normalize(arr = shL_end, arrNorm = shL_pelvis)
        shL_strap = (shL_start_norm, shL_end_norm)
        
        strapR = shR_strap
        strapL = shL_strap     
        
    return (strapR, strapL)
    
def getRef(right_s, left_s, right_e, left_e, modelType):
    if modelType == "hanging":
        typeIndex = HANGING_INDEX
    else:
        typeIndex = LOADING_INDEX
    
    rs =  refactor(arr = right_s, arrRef = hip_pelvis)
    ls = refactor(arr = left_s, arrRef = hip_pelvis)
    
    if typeIndex == HANGING_INDEX:
        rE = refactor(arr = right_e, arrRef = legR_pelvis)
        le = refactor(arr = left_e, arrRef = legL_pelvis)
        
    elif typeIndex == LOADING_INDEX:
        rE = refactor(arr = right_e, arrRef = shR_pelvis)
        le = refactor(arr = left_e, arrRef = shL_pelvis)
        
    return ((rs, rE),(ls, le))
