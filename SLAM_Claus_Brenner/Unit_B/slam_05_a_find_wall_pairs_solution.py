#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun Jan 14 15:13:26 2018

@author: akashan
"""

from lego_robot import *
from slam_b_library import filter_step, compute_cartesian_coordinates

# Takes one scan and subsamples the measurements, so that every sampling'th
# point is taken. Returns a list of (x, y) points in the scanner's
# coordinate system.
def get_subsampled_points(scan, sampling = 10):
    # Subsample from scan
    index_range_tuples = []
    for i in xrange(0, len(scan), sampling):
        index_range_tuples.append( (i, scan[i]) )
    return compute_cartesian_coordinates(index_range_tuples, 0.0)

# Given a set of points, checks for every point p if it is closer than
# eps to the left, right, upper or lower wall of the arena. If so,
# adds the point to left_list, and the closest point on the wall to
# right_list.
def get_corresponding_points_on_wall(points,
                                     arena_left = 0.0, arena_right = 2000.0,
                                     arena_bottom = 0.0, arena_top = 2000.0,
                                     eps = 150.0):
    left_list = []
    right_list = []

    # ---> Implement your code here.
    for p in points:
        if p[0]<eps:
            left_list.append((p[0],p[1]))
            right_list.append((0,p[1]))
        if p[0]>arena_right-eps:
            left_list.append((p[0],p[1]))
            right_list.append((arena_right,p[1]))
        if p[1]<eps:
            left_list.append((p[0],p[1]))
            right_list.append((p[0],0))
        if p[1]>arena_top-eps: 
            left_list.append((p[0],p[1]))
            right_list.append((p[0],arena_top))
    return left_list, right_list