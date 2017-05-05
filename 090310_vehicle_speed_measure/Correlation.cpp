#include "StdAfx.h"
#include "Correlation.h"

#define NUM_BINS_RGB	(512)

Correlation::Correlation(void) { }
Correlation::~Correlation(void) { }


double Correlation::correlationCoefficient(double* sampleColorDistribution, double* objColorDistribution) {
	double meanOfSample = mean(sampleColorDistribution);
	double meanOfObj = mean(objColorDistribution);

	double numerator = 0;
	double denominatorSample = 0, denominatorObj = 0;
	for (int i = 0; i < NUM_BINS_RGB; i++) {
		double subtractSample = sampleColorDistribution[i] - meanOfSample;
		double subtractObj = objColorDistribution[i] - meanOfObj;
		numerator += subtractSample*subtractObj;
		denominatorSample += subtractSample*subtractSample;
		denominatorObj += subtractObj*subtractObj;
	}
	return numerator / sqrt(denominatorSample*denominatorObj);
}


double Correlation::bhattacharyyaCoefficient(double* sampleColorDistribution, double* objColorDistribution) {
	double result = 0;
	for (int i = 0; i < NUM_BINS_RGB; i++) 
		result += sqrt(sampleColorDistribution[i]*objColorDistribution[i]);
	return result;
}


double* Correlation::calculateRGBHistogram(IplImage* img, int startX, int endX, int startY, int endY) { 
	int channels = img->nChannels, step = img->widthStep, width = img->width, height = img->height;
	const uchar* imgData = (uchar *)img->imageData;

	int hx = (endX - startX) / 2;
	int hy = (endY - startY) / 2;
	int centerX = (endX + startX) / 2;
	int centerY = (endY + startY) / 2;
	double bandwidthOfPatch = euclideanDistance(hx, hy);

	// init histogram bins to all zero
	double* histogram = new double[NUM_BINS_RGB]; // RGB: 8x8x8
	for (int i = 0; i < NUM_BINS_RGB; i++) histogram[i] = 0;

	//
	for (int i = startY; i < endY; i++)
		for (int j = startX; j < endX; j++) {
			int pos = i*step + j*channels;
			int binIndexB = imgData[pos] / 32; 	// 256 / 8 = 32
			int binIndexG = imgData[pos + 1] / 32;
			int binIndexR = imgData[pos + 2] / 32;
			histogram[binIndexB*64 + binIndexG*8 + binIndexR] += kernel(j - centerX, i - centerY, bandwidthOfPatch);
		}
			
	// normalize histogram
	double sumOfHistogramValue = 0;
	for (int i = 0; i < NUM_BINS_RGB; i++) sumOfHistogramValue += histogram[i];
	for (int i = 0; i < NUM_BINS_RGB; i++) histogram[i] /= sumOfHistogramValue;

	return histogram;
}


// ========================================== PRIVATE ==========================================

double Correlation::euclideanDistance(int x, int y) { // a
	return sqrt(double(x*x + y*y));
}

double Correlation::kernel(int x1, int x2, double bandwidthPatch) {
	double r = euclideanDistance(x1, x2) / bandwidthPatch;
	if (r < 1) return 1 - r*r;
	else return 0;
}

double Correlation::mean(double* distribution) {
	double sum = 0;
	for (int i = 0; i < NUM_BINS_RGB; i++) sum += distribution[i];
	return sum / NUM_BINS_RGB;
}