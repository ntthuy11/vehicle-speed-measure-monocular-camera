#pragma once

#include "cv.h"
#include "highgui.h"

class Correlation
{
public:
	Correlation(void);
	~Correlation(void);

	double correlationCoefficient(double* sampleColorDistribution, double* objColorDistribution);
	double bhattacharyyaCoefficient(double* sampleColorDistribution, double* objColorDistribution);
	double* calculateRGBHistogram(IplImage* img, int startX, int endX, int startY, int endY);

private:	
	double euclideanDistance(int x, int y);
	double kernel(int x1, int x2, double bandwidthPatch);
	double mean(double* distribution);
};
