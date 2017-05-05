#pragma once

#include "cv.h"
#include "highgui.h"

class ImageRectification
{
public:
	ImageRectification(void);
	~ImageRectification(void);

	void affineRectifyWithOneVanishingPoint(IplImage* srcImg, CvPoint vanishingPoint, CvPoint shiftLen, IplImage* desImg);
	CvPoint inverseAffineRectifyForOnePoint(CvPoint srcPoint, CvPoint vanishingPoint, int shiftY);

private:
	CvPoint map(double* H, int i, int j);
};
