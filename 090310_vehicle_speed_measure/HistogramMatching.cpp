#include "StdAfx.h"
#include "HistogramMatching.h"

HistogramMatching::HistogramMatching(IplImage* previousImg, IplImage* currentImg) { 
	imgHeight = previousImg->height;
	imgWidth = previousImg->width;

	this->previousImg = previousImg;
	this->currentImg = currentImg;	
}

HistogramMatching::~HistogramMatching(void) { 
	//releaseVerHist();
	//releaseHorHist();
}

CvPoint HistogramMatching::match() {
	return matchWithRanges(cvPoint(-imgWidth, imgWidth), cvPoint(-imgHeight, imgHeight));
}

CvPoint HistogramMatching::matchWithRanges(CvPoint rangeDx, CvPoint rangeDy) { // return (dx, dy)
	return cvPoint(matchWithRangeDx(rangeDx), matchWithRangeDy(rangeDy));
}

// -------------

void HistogramMatching::initVerHist() {
	calculatedVerHistOfPrevImg = new int*[imgWidth];
	calculatedVerHistOfCurrImg = new int*[imgWidth];
	for(int i = 0; i < imgWidth; i++) {
		calculatedVerHistOfPrevImg[i] = calculateVerticalHistogramOfOneColumn(previousImg, i);
		calculatedVerHistOfCurrImg[i] = calculateVerticalHistogramOfOneColumn(currentImg, i);
	}
}

void HistogramMatching::initHorHist() {
	calculatedHorHistOfPrevImg = new int*[imgHeight];
	calculatedHorHistOfCurrImg = new int*[imgHeight];
	for(int i = 0; i < imgHeight; i++) {
		calculatedHorHistOfPrevImg[i] = calculateHorizontalHistogramOfOneRow(previousImg, i);
		calculatedHorHistOfCurrImg[i] = calculateHorizontalHistogramOfOneRow(currentImg, i);
	}
}

int HistogramMatching::matchWithRangeDx(CvPoint rangeDx) { // rangeDx = [-w, w]
	double maxValForVerHistMatch = 0;
	int dx = 0;
	for (int i = rangeDx.x; i < rangeDx.y; i++) {
		double tmpMaxVal = matchVerticalHistogramsOfTwoImages(i);
		if (maxValForVerHistMatch <= tmpMaxVal) {
			maxValForVerHistMatch = tmpMaxVal;
			dx = i;
		}
	}
	return dx;
}

int HistogramMatching::matchWithRangeDy(CvPoint rangeDy) {
	double maxValForHorHistMatch = 0;
	int dy = 0;
	for (int i = rangeDy.x; i < rangeDy.y; i++) {
		double tmpMaxVal = matchHorizontalHistogramsOfTwoImages(i);
		if (maxValForHorHistMatch <= tmpMaxVal) {
			maxValForHorHistMatch = tmpMaxVal;
			dy = i;
		}
	}
	return dy;
}

void HistogramMatching::releaseVerHist() {
	for(int i = 0; i < imgWidth; i++) {		delete[] calculatedVerHistOfPrevImg[i];		delete[] calculatedVerHistOfCurrImg[i];		}
	delete[] calculatedVerHistOfPrevImg;
	delete[] calculatedVerHistOfCurrImg;
}

void HistogramMatching::releaseHorHist() {
	for(int i = 0; i < imgHeight; i++) {	delete[] calculatedHorHistOfPrevImg[i];		delete[] calculatedHorHistOfCurrImg[i];		}
	delete[] calculatedHorHistOfPrevImg;
	delete[] calculatedHorHistOfCurrImg;
}

// =================================== PRIVATE ===================================

double HistogramMatching::matchVerticalHistogramsOfTwoImages(int dx) {
	double sumOfD = 0;
	int totalPos = 0;
	if (dx > 0) {
		totalPos = imgWidth - 1 - dx;		
		for (int i = 0; i < totalPos; i++) 
			sumOfD += matchTwoVerticalHistogramsOfTwoColumns(calculatedVerHistOfPrevImg[i + dx], calculatedVerHistOfCurrImg[i], imgHeight);
	} else {
		totalPos = imgWidth - 1 + dx;
		for (int i = 0; i < totalPos; i++) 
			sumOfD += matchTwoVerticalHistogramsOfTwoColumns(calculatedVerHistOfPrevImg[i], calculatedVerHistOfCurrImg[i - dx], imgHeight);
	}
	return sumOfD * 1.0 / totalPos;
}


double HistogramMatching::matchHorizontalHistogramsOfTwoImages(int dy) {
	double sumOfD = 0;
	int totalPos = 0;
	if (dy > 0) {
		totalPos = imgHeight - 1 - dy;
		for (int i = 0; i < totalPos; i++) 
			sumOfD += matchTwoHorizontalHistogramsOfTwoRows(calculatedHorHistOfPrevImg[i + dy], calculatedHorHistOfCurrImg[i], imgWidth);
	} else {
		totalPos = imgHeight - 1 + dy;
		for (int i = 0; i < totalPos; i++)
			sumOfD += matchTwoHorizontalHistogramsOfTwoRows(calculatedHorHistOfPrevImg[i], calculatedHorHistOfCurrImg[i - dy], imgWidth);
	}
	return sumOfD * 1.0 / totalPos;
}


double HistogramMatching::matchTwoVerticalHistogramsOfTwoColumns(int* srcHist, int* desHist, int imgHeight) {
	int d = 0;
	for (int i = 0; i < NUM_BINS_RGB; i++)	
		d += abs(srcHist[i] - desHist[i]);
	return 1.0 - d * 1.0 / (2 * imgHeight);
}


double HistogramMatching::matchTwoHorizontalHistogramsOfTwoRows(int* srcHist, int* desHist, int imgWidth) {
	int d = 0;
	for (int i = 0; i < NUM_BINS_RGB; i++)	d += abs(srcHist[i] - desHist[i]);
	return 1.0 - d * 1.0 / (2 * imgWidth);
}


int* HistogramMatching::calculateVerticalHistogramOfOneColumn(IplImage* img, int ithColumn) { // RGB
	int channels = img->nChannels, step = img->widthStep, height = img->height;
	const uchar* imgData = (uchar *)img->imageData;

	// init histogram bins to all zero
	int* histogram = new int[NUM_BINS_RGB]; // RGB: 8x8x8
	for (int i = 0; i < NUM_BINS_RGB; i++) histogram[i] = 0;

	//
	for (int i = 0; i < height; i++) {
		int pos = i*step + ithColumn*channels;
		int binIndexB = imgData[pos] / 32; 	// 256 / 8 = 32
		int binIndexG = imgData[pos + 1] / 32;
		int binIndexR = imgData[pos + 2] / 32;
		histogram[binIndexB*64 + binIndexG*8 + binIndexR]++;
	}

	return histogram;
}


int* HistogramMatching::calculateHorizontalHistogramOfOneRow(IplImage* img, int ithRow) { // RGB
	int channels = img->nChannels, step = img->widthStep, width = img->width;
	const uchar* imgData = (uchar *)img->imageData;

	// init histogram bins to all zero
	int* histogram = new int[NUM_BINS_RGB]; // RGB: 8x8x8
	for (int i = 0; i < NUM_BINS_RGB; i++) histogram[i] = 0;

	//
	for (int i = 0; i < width; i++) {
		int pos = ithRow*step + i*channels;
		int binIndexB = imgData[pos] / 32; 	// 256 / 8 = 32
		int binIndexG = imgData[pos + 1] / 32;
		int binIndexR = imgData[pos + 2] / 32;
		histogram[binIndexB*64 + binIndexG*8 + binIndexR]++;
	}

	return histogram;
}