#pragma once

#include "cv.h"
#include "highgui.h"

class BackgroundSubtraction
{
public:
	BackgroundSubtraction(void);
	~BackgroundSubtraction(void);

	void createBackground(IplImage** listOfImg, int nImg, IplImage* desImg);
	
private:
	CvPoint2D32f calculateMeanAndStdDeviationOfPixels(int* intensityValues, int nValues, int nElements);
};
