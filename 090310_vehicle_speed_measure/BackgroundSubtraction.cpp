#include "StdAfx.h"
#include "BackgroundSubtraction.h"

BackgroundSubtraction::BackgroundSubtraction(void) { }
BackgroundSubtraction::~BackgroundSubtraction(void) { }


void BackgroundSubtraction::createBackground(IplImage** listOfImg, int nImg, IplImage* desImg) {
	int w = listOfImg[0]->width, h = listOfImg[0]->height, step = listOfImg[0]->widthStep, nChannels = listOfImg[0]->nChannels;
	uchar* desData = (uchar *) desImg->imageData;

	for (int i = 0; i < h; i++) {
		for (int j = 0; j < w; j++) {
			int initPos = i*step + j*nChannels;			
			int nImgUsed = nImg;
			int** intensityValues = new int*[nChannels];	for (int k = 0; k < nChannels; k++) intensityValues[k] = new int[nImg]; // khoi tao mang 2 chieu intensityValues[nChannels][nImg]

			for (int k = 0; k < nChannels; k++) {				
				// nap cac gia tri pixel vao intensityValues, de tinh mean & standard deviation
				for (int n = 0; n < nImg; n++) if (intensityValues[k][n] != -1) intensityValues[k][n] = ((uchar*)listOfImg[n]->imageData)[initPos + k];

				//if (nImgUsed == 0) continue; 
				for ( ; ; ) {
					CvPoint2D32f meanAndStdDeviation = calculateMeanAndStdDeviationOfPixels(intensityValues[k], nImg, nImgUsed);
					bool isInRange = true;
					for (int n = 0; n < nImg; n++) {
						if ( (intensityValues[k][n] != -1)
						&& (intensityValues[k][n] < (meanAndStdDeviation.x - meanAndStdDeviation.y) || (meanAndStdDeviation.x + meanAndStdDeviation.y) < intensityValues[k][n]) ) {
							intensityValues[0][n] = intensityValues[1][n] = intensityValues[2][n] = -1; // chi can 1 channel ko thoa thi 2 channels con lai deu ko su dung
							nImgUsed--; // ~nImg duoc su dung de tinh ra pixel cho ne^`n
							isInRange = false;
						}
					}
					if (isInRange) break; // van con it nhat 1 pixel chua vuot khoi range											
				}
			}

			// lay mang intensityValues de tinh ket qua cuoi cung cho ne^`n
			for (int k = 0; k < nChannels; k++) {
				int sum = 0;
				int count = 0;
				for (int n = 0; n < nImg; n++) {					
					if (intensityValues[k][n] != -1) {
						sum += intensityValues[k][n];
						count++;
					}
				}				
				if (count != 0) desData[initPos + k] = int(sum * 1.0 / count);
			}

			// release intensityValues
			for (int k = 0; k < nChannels; k++) delete[] intensityValues[k];
			delete[] intensityValues;
		}
	}
}


// ================================================ PRIVATE ================================================

CvPoint2D32f BackgroundSubtraction::calculateMeanAndStdDeviationOfPixels(int* intensityValues, int nValues, int nElements) {
	
	// mean
	int totalIntensity = 0;
	for (int i = 0; i < nValues; i++) {
		if (intensityValues[i] != -1)
			totalIntensity += intensityValues[i];
	}
	double mean = totalIntensity * 1.0 / nElements;

	// standard deviation
	double totalOfSqrDeviations = 0;
	for (int i = 0; i < nValues; i++) {
		if (intensityValues[i] != -1) {
			double deviation = mean - intensityValues[i];
			totalOfSqrDeviations += deviation*deviation;
		}
	}
	double stdDeviation = sqrt(totalOfSqrDeviations / nElements);

	return cvPoint2D32f(mean, stdDeviation);
}
