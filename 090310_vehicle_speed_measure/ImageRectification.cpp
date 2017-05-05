#include "StdAfx.h"
#include "ImageRectification.h"

ImageRectification::ImageRectification(void) { }
ImageRectification::~ImageRectification(void) { }


void ImageRectification::affineRectifyWithOneVanishingPoint(IplImage* srcImg, CvPoint vanishingPoint, CvPoint shiftLen, IplImage* desImg) {	
	int w = srcImg->width, h = srcImg->height, step = srcImg->widthStep, nChannels = srcImg->nChannels;
	const uchar* srcData = (uchar*)srcImg->imageData;
	uchar* desData = (uchar*)desImg->imageData;

	//
	int scaleFactor = 1;
	int f1x = vanishingPoint.x;
	int f1y = vanishingPoint.y;
	double H[9] = {	scaleFactor,	scaleFactor * (-f1x * 1.0 / f1y),		scaleFactor * shiftLen.x,
					0,				1,										shiftLen.y,
					0,				-1.0 / f1y,								1 };


	// ------------- rectify image -------------
	int imgSize = w*h;
	bool* visitedImgPosition = new bool[imgSize]; // is used to store the positions visited by the following loops
	for (int i = 0; i < imgSize; i++) visitedImgPosition[i] = false;

	for (int i = 0; i < h; i++) {
		for (int j = 0; j < w; j++) {
			CvPoint newPos = map(H, i, j); // newPosition = H*oldPosition
			int newI = min(h - 1, max(0, newPos.y));
			int newJ = min(w - 1, max(0, newPos.x));
			for (int k = 0; k < nChannels; k++) desData[newI*step + newJ*nChannels + k] = srcData[i*step + j*nChannels + k];
		
			visitedImgPosition[newI*w + newJ] = true; // store the visited position
		}
	}


	// ------------- (cubic) interpolate "black pixels" (unvisited) from the undistorted image -------------
	for (int i = 1; i < h - 1; i++) {
		for (int j = 1; j < w - 1; j++) {
			int currPos = i*w + j;
			if (!visitedImgPosition[currPos]) { // not visited
				int nPixelsAroundBlackPixel = visitedImgPosition[i*w + (j-1)]		+ visitedImgPosition[i*w + (j+1)]		+ visitedImgPosition[(i-1)*w + j] 
									   + visitedImgPosition[(i+1)*w + j]		+ visitedImgPosition[(i-1)*w + (j-1)]	+ visitedImgPosition[(i-1)*w + (j+1)] 
									   + visitedImgPosition[(i+1)*w + (j-1)]	+ visitedImgPosition[(i+1)*w + (j+1)];
				if (nPixelsAroundBlackPixel != 0) {
					for (int k = 0; k < nChannels; k++) {
						int left		=	desData[i*step + (j-1)*nChannels + k];
						int right		=	desData[i*step + (j+1)*nChannels + k];
						int up			=	desData[(i-1)*step + j*nChannels + k];
						int down		=	desData[(i+1)*step + j*nChannels + k];
						int upLeft		=	desData[(i-1)*step + (j-1)*nChannels + k];
						int upRight		=	desData[(i-1)*step + (j+1)*nChannels + k];
						int downLeft	=	desData[(i+1)*step + (j-1)*nChannels + k];
						int downRight	=	desData[(i+1)*step + (j+1)*nChannels + k];
						desData[i*step + j*nChannels + k] = (left + right + up + down + upLeft + upRight + downLeft + downRight) / nPixelsAroundBlackPixel;
					}
					visitedImgPosition[currPos] = true;
				}
			}
		}
	}

	delete visitedImgPosition;
}


CvPoint ImageRectification::inverseAffineRectifyForOnePoint(CvPoint srcPoint, CvPoint vanishingPoint, int shiftY /* not used shiftX */) { // not used scaleFactor
	double denominator = vanishingPoint.y + shiftY;
	double inverseH[9] = {	1,		vanishingPoint.x * 1.0 / denominator,		- vanishingPoint.x * shiftY * 1.0 / denominator,
							0,		vanishingPoint.y * 1.0 / denominator,		- vanishingPoint.y * shiftY * 1.0 / denominator,
							0,		1.0 / denominator,							vanishingPoint.y * 1.0 / denominator };
	CvPoint desPoint = map(inverseH, srcPoint.y, srcPoint.x);
	//return cvPoint(desPoint.y, desPoint.x);
	return cvPoint(desPoint.x, desPoint.y); // inverse x and y to use easily
}


// ================================ PRIVATE ================================

CvPoint ImageRectification::map(double* H, int i, int j) {
	double x1 = H[0]*j + H[1]*i + H[2];
	double x2 = H[3]*j + H[4]*i + H[5];
	double x3 = H[6]*j + H[7]*i + H[8];
	return cvPoint(int(x1/x3), int(x2/x3));
}