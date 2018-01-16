#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun Jan 14 15:06:00 2018

@author: akashan
"""
from lego_robot import *
from slam_b_library import filter_step
from slam_04_a_project_landmarks import\
     compute_scanner_cylinders, write_cylinders
from math import sqrt, atan2

def compute_center(point_list):
    # Safeguard against empty list.
    if not point_list:
        return (0.0, 0.0)
    # If not empty, sum up and divide.
    sx = sum([p[0] for p in point_list])
    sy = sum([p[1] for p in point_list])
    return (sx / len(point_list), sy / len(point_list))

def estimate_transform(left_list, right_list, fix_scale = False):
    # Compute left and right center.
    lc = compute_center(left_list)
    rc = compute_center(right_list)

    cs = 0
    ss = 0
    rr = 0
    ll = 0
    # --->>> Insert here your code to compute lambda, c, s and tx, ty.
    ldash = [(left_list[i][0]-lc[0] , left_list[i][1]-lc[1]) for i in xrange(len(left_list))]
    rdash = [(right_list[i][0]-rc[0] , right_list[i][1]-rc[1]) for i in xrange(len(right_list))]
    
    for i in xrange(len(left_list)):
        cs += rdash[i][0]*ldash[i][0] + rdash[i][1]*ldash[i][1]
        ss += -rdash[i][0]*ldash[i][1] + rdash[i][1]*ldash[i][0]
        rr += rdash[i][0]*rdash[i][0] + rdash[i][1]*rdash[i][1]
        ll += ldash[i][0]*ldash[i][0] + ldash[i][1]*ldash[i][1]
    if ll==0 or rr==0:
        return None
    
    print cs,ss
    la = sqrt(rr/ll)
    c = cs/sqrt(cs**2+ss**2)
    print 'hello'
    s = ss/sqrt(cs**2+ss**2)
    tx = rc[0] - la*(c*lc[0]-s*lc[1])
    ty = rc[1] - la*(s*lc[0]+c*lc[1])
    # --->>> Insert your previous solution here.

    return la, c, s, tx, ty

# Given a similarity transformation:
# trafo = (scale, cos(angle), sin(angle), x_translation, y_translation)
# and a point p = (x, y), return the transformed point.
def apply_transform(trafo, p):
    la, c, s, tx, ty = trafo
    lac = la * c
    las = la * s
    x = lac * p[0] - las * p[1] + tx
    y = las * p[0] + lac * p[1] + ty
    return (x, y)

# Correct the pose = (x, y, heading) of the robot using the given
# similarity transform. Note this changes the position as well as
# the heading.
def correct_pose(pose, trafo):
    
    # --->>> This is what you'll have to implement.
    if trafo is None:
        return pose
    la, c, s, tx, ty = trafo
    x = apply_transform(trafo,(pose[0],pose[1]))
    alpha = atan2(s,c)
    return (x[0], x[1], pose[2]+alpha)  # Replace this by the corrected pose.