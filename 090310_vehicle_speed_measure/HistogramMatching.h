#pragma once

#include "cv.h"
#include "highgui.h"

#define NUM_BINS_RGB	(512)


class HistogramMatching
{
public:
	HistogramMatching(IplImage* previousImg, IplImage* currentImg);
	~HistogramMatching(void);

	CvPoint match();
	CvPoint matchWithRanges(CvPoint rangeDx, CvPoint rangeDy);

	void initVerHist();
	void initHorHist();
	int matchWithRangeDx(CvPoint rangeDx);
	int matchWithRangeDy(CvPoint rangeDy);	
	void releaseVerHist();
	void releaseHorHist();

private:		

	double matchVerticalHistogramsOfTwoImages(int dx);
	double matchHorizontalHistogramsOfTwoImages(int dy);

	double matchTwoVerticalHistogramsOfTwoColumns(int* srcHist, int* desHist, int imgHeight);
	double matchTwoHorizontalHistogramsOfTwoRows(int* srcHist, int* desHist, int imgWidth);	

	int* calculateVerticalHistogramOfOneColumn(IplImage* img, int ithColumn);
	int* calculateHorizontalHistogramOfOneRow(IplImage* img, int ithRow);


	int imgHeight, imgWidth;
	IplImage* previousImg;
	IplImage* currentImg;

	int** calculatedVerHistOfPrevImg;	int** calculatedVerHistOfCurrImg;
	int** calculatedHorHistOfPrevImg;	int** calculatedHorHistOfCurrImg;
};
