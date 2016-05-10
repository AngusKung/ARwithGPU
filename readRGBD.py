#!/usr/bin/env python

import cv2
import numpy as np
import argparse
import random
import pdb

import sys
sys.setrecursionlimit(2000483647)

def readPNG(prefix,imageNum):
	
	rgbImageFile = prefix+'/'+prefix+'_'+str(imageNum)+'.png'
	depthFile = prefix+'/'+prefix+'_'+str(imageNum)+'_depth.png'
	
	rgbImage = cv2.imread(rgbImageFile)
	depth = cv2.imread(depthFile)
	
	imageHeight = depth.shape[0]
	imageWidth = depth.shape[1]

	return rgbImage, depth, imageHeight, imageWidth

def pickAndReplaceRGB(rgbImage,depth, imageHeight, imageWidth):
	pickList = list(range(imageWidth*imageHeight))
	labelNum = 0
	labelImage = depth
	while(len(pickList)!=0):
		print len(pickList)
		random.shuffle(pickList)
		pickNum = pickList[0]
		startD = depth[pickNum/imageWidth][pickNum%imageWidth][0]
		pickList.pop(0)
		findNeighbor(labelImage,pickList,depth,pickNum,imageWidth,imageHeight,startD,labelNum)
		labelNum+=1

	resultImage = labelImage
	return resultImage

def findNeighbor(labelImage,pickList,depth,pickNum,imageWidth,imageHeight,lastD,labelNum):
	W = pickNum%imageWidth
	H = pickNum/imageWidth
	labelImage[H][W][0] = labelNum
	labelImage[H][W][1] = labelNum
	labelImage[H][W][2] = labelNum
	if W!=imageWidth-1:
		rightD = depth[H][W+1][0]
		if abs(rightD-lastD)<=0 :
			newNum = W+1 + H*imageWidth
			if newNum in pickList:
				pickList.remove(newNum)
				findNeighbor(labelImage,pickList,depth,newNum,imageWidth,imageHeight,rightD,labelNum)
	if W!=0:
		leftD = depth[H][W-1][0]
		if abs(leftD-lastD)<=0 :
			newNum = W-1 + H*imageWidth
			if newNum in pickList:
				pickList.remove(newNum)
				findNeighbor(labelImage,pickList,depth,newNum,imageWidth,imageHeight,leftD,labelNum)
	if H!=imageHeight-1:
		upD = depth[H+1][W][0]
		if abs(upD-lastD)<=0 :
			newNum = W + (H+1)*imageWidth
			if newNum in pickList:
				pickList.remove(newNum)
				findNeighbor(labelImage,pickList,depth,newNum,imageWidth,imageHeight,upD,labelNum)
	if H!=0:
		downD = depth[H-1][W][0]
		if abs(downD-lastD)<=0 :
			newNum = W + (H-1)*imageWidth
			if newNum in pickList:
				pickList.remove(newNum)
				findNeighbor(labelImage,pickList,depth,newNum,imageWidth,imageHeight,downD,labelNum)
	return
	#pdb.set_trace()

'''def writePNG(imageNum):
	file = open(plyFile,"w")
	file.close()'''


if __name__ == '__main__':
	parser = argparse.ArgumentParser(description='''
    This script reads two sequences of png, one representing RGB while another representing Depth.
    ''')
	parser.add_argument('png_file_prefix', help='Directory of the input PNGs, EX:desk_1')
	args = parser.parse_args()

	for num in range(1,98):
		rgbImage , depth, imageHeight, imageWidth = readPNG(args.png_file_prefix,num)
    	result = pickAndReplaceRGB(rgbImage,depth,imageHeight,imageWidth)
    	pdb.set_trace()