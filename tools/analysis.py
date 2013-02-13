#!/usr/bin/env python

import sys, re, os

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import matplotlib.cbook as cbook

file_name = sys.argv[1]

overall_time = []
subsampling_time = []
roi_time = []
platform_extraction_time = []
candidate_extraction_time = []
tracking_time = []
drawing_tracking_time = []

iterations = 0

for line in open( file_name, 'r' ):
	if 'subsampling_time' in line:
		temp = line.split('=')[1]
		temp = re.sub( "[^0-9.]", "", temp )
		subsampling_time.append( temp )
	if 'roi_time' in line:
		temp = line.split('=')[1]
		temp = re.sub( "[^0-9.]", "", temp )
		roi_time.append( temp )
	if 'platform_extraction_time' in line:
		temp = line.split('=')[1]
		temp = re.sub( "[^0-9.]", "", temp )
		platform_extraction_time.append( temp )
	if 'candidate_extraction_time' in line:
		temp = line.split('=')[1]
		temp = re.sub( "[^0-9.]", "", temp )
		candidate_extraction_time.append( temp )
	if 'tracking_time' in line:
		temp = line.split('=')[1]
		temp = re.sub( "[^0-9.]", "", temp )
		tracking_time.append( temp )
	if 'drawing_tracking_time' in line:
		temp = line.split('=')[1]
		temp = re.sub( "[^0-9.]", "", temp )
		drawing_tracking_time.append( temp )
	if 'overall_time' in line:
		iterations += 1
		temp = line.split('=')[1]
		temp = re.sub( "[^0-9.]", "", temp )
		overall_time.append( temp )

avg_roi = 0
avg_subsampling = 0
avg_platform_extraction = 0
avg_candidate_extraction = 0
avg_tracking = 0
avg_drawing_tracking = 0
avg_overall_time = 0


for x in subsampling_time:
	avg_subsampling += float(x)

for x in roi_time:
	avg_roi += float(x)

for x in platform_extraction_time:
	avg_platform_extraction += float(x)

for x in candidate_extraction_time:
	avg_candidate_extraction += float(x)

for x in tracking_time:
	avg_tracking += float(x)

for x in drawing_tracking_time:
	avg_drawing_tracking += float(x)

for x in overall_time:
	avg_overall_time += float(x)

avg_subsampling /= iterations
avg_roi /= iterations
avg_platform_extraction /= iterations
avg_candidate_extraction /= iterations
avg_tracking /= iterations
avg_drawing_tracking /= iterations
avg_overall_time /= iterations

print "Iterations: ", iterations
print "avg_roi: ", avg_roi
print "avg_subsampling: ", avg_subsampling
print "avg_platform_extraction: ", avg_platform_extraction
print "avg_candidate_extraction: ", avg_candidate_extraction
print "avg_tracking: ", avg_tracking
print "avg_drawing_tracking: ", avg_drawing_tracking
print "avg_overall_time: ", avg_overall_time

avg_hz = 1 / avg_overall_time

print "avg_hz: ", avg_hz