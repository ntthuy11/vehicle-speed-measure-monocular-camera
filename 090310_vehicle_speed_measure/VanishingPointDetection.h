#pragma once

#include "cv.h"
#include "highgui.h"

class VanishingPointDetection
{
public:
	VanishingPointDetection(void);
	~VanishingPointDetection(void);

	CvPoint findVanishingPoint(IplImage* srcImg, CvPoint initLeftTop, CvPoint initRightBottom, int level, int voteThr);
	void filterPointsWithRespectToRegion(CvSeq* srcPoints, CvPoint leftTop, CvPoint rightBottom, CvSeq* desPoints);
	void findIntersectionPoints(IplImage* srcImg, CvSeq* intersectionPoints, int voteThr);

private:
	double calculateX(double rho1, double rho2, double theta1, double theta2);
	double calculateY(double rho1, double theta1, double x);
	CvPoint meanPoint(CvSeq* points);
};
