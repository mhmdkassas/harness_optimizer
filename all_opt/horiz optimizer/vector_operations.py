# -*- coding: utf-8 -*-
"""
Created on Tue Feb  5 10:41:03 2019

@author: mhmdk
"""

def refactor(arr, arrRef):
    newArr = [0, 0, 0]
    for i in range(0,3):
        newArr[i] = arr[i] - arrRef[i]
    return newArr

def normalize(arr, arrNorm):
    newArr = [0, 0, 0]
    for i in range(0,3):
        newArr[i] = arr[i]+arrNorm[i]
    return newArr

