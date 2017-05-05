#include "StdAfx.h"
#include "VanishingPointDetection.h"

VanishingPointDetection::VanishingPointDetection(void) { }
VanishingPointDetection::~VanishingPointDetection(void) { }


CvPoint VanishingPointDetection::findVanishingPoint(IplImage* srcImg, CvPoint initLeftTop, CvPoint initRightBottom, int level, int voteThr) {
	
	// find all points intersected by detected lines
	CvMemStorage* intersectionPointsStorage = cvCreateMemStorage(0);
	CvSeq* intersectionPoints = cvCreateSeq(CV_SEQ_ELTYPE_GENERIC, sizeof(CvSeq), sizeof(CvPoint), intersectionPointsStorage);
	findIntersectionPoints(srcImg, intersectionPoints, voteThr);


	// filter intersectionPoints with respect to defined region
	CvMemStorage* filteredPointsStorage = cvCreateMemStorage(0);
	CvSeq* filteredPoints = cvCreateSeq(CV_SEQ_ELTYPE_GENERIC, sizeof(CvSeq), sizeof(CvPoint), filteredPointsStorage);
	//CvPoint initRegionLeftTop = cvPoint(srcImg->width / 3, 0); // img duoc chia lam 6 regions, 3 cot, 2 dong => region o giua trong 3 regions cot
	//CvPoint initRegionRightBottom = cvPoint(srcImg->width / 3 * 2, srcImg->height - 1);
	filterPointsWithRespectToRegion(intersectionPoints, initLeftTop, initRightBottom, filteredPoints);


	// "divide by 4" for each region
	CvPoint leftTop = cvPoint(initLeftTop.x, initLeftTop.y);
	CvPoint rightBottom = cvPoint(initRightBottom.x, initRightBottom.y);

	for (int i = 0; i < level; i++) {
		/*		---------
				| 1 | 2 |
				---------
				| 3 | 4 |
				--------- */

		// region 1
		CvPoint leftTopReg1 = cvPoint(leftTop.x, leftTop.y);
		CvPoint rightBottomReg1 = cvPoint( (leftTop.x + rightBottom.x) / 2, (leftTop.y + rightBottom.y) / 2 );
		CvMemStorage* filteredPoints1Storage = cvCreateMemStorage(0);
		CvSeq* filteredPoints1 = cvCreateSeq(CV_SEQ_ELTYPE_GENERIC, sizeof(CvSeq), sizeof(CvPoint), filteredPoints1Storage);
		filterPointsWithRespectToRegion(filteredPoints, leftTopReg1, rightBottomReg1, filteredPoints1);

		// region 2
		CvPoint leftTopReg2 = cvPoint( (leftTop.x + rightBottom.x) / 2, leftTop.y);
		CvPoint rightBottomReg2 = cvPoint( rightBottom.x, (leftTop.y + rightBottom.y) / 2 );
		CvMemStorage* filteredPoints2Storage = cvCreateMemStorage(0);
		CvSeq* filteredPoints2 = cvCreateSeq(CV_SEQ_ELTYPE_GENERIC, sizeof(CvSeq), sizeof(CvPoint), filteredPoints2Storage);
		filterPointsWithRespectToRegion(filteredPoints, leftTopReg2, rightBottomReg2, filteredPoints2);

		// region 3
		CvPoint leftTopReg3 = cvPoint(leftTop.x, (leftTop.y + rightBottom.y) / 2);
		CvPoint rightBottomReg3 = cvPoint( (leftTop.x + rightBottom.x) / 2, rightBottom.y );
		CvMemStorage* filteredPoints3Storage = cvCreateMemStorage(0);
		CvSeq* filteredPoints3 = cvCreateSeq(CV_SEQ_ELTYPE_GENERIC, sizeof(CvSeq), sizeof(CvPoint), filteredPoints3Storage);
		filterPointsWithRespectToRegion(filteredPoints, leftTopReg3, rightBottomReg3, filteredPoints3);

		// region 4
		CvPoint leftTopReg4 = cvPoint(rightBottomReg1.x, rightBottomReg1.y);
		CvPoint rightBottomReg4 = cvPoint(rightBottom.x, rightBottom.y);
		CvMemStorage* filteredPoints4Storage = cvCreateMemStorage(0);
		CvSeq* filteredPoints4 = cvCreateSeq(CV_SEQ_ELTYPE_GENERIC, sizeof(CvSeq), sizeof(CvPoint), filteredPoints4Storage);
		filterPointsWithRespectToRegion(filteredPoints, leftTopReg4, rightBottomReg4, filteredPoints4);

		// compare number of intersected points in each region together
		int maxNPoints = max(filteredPoints1->total, max(filteredPoints2->total, max(filteredPoints3->total, filteredPoints4->total)));
		if (maxNPoints == filteredPoints1->total) {
			leftTop = cvPoint(leftTopReg1.x, leftTopReg1.y);
			rightBottom = cvPoint(rightBottomReg1.x, rightBottomReg1.y);
			filteredPoints = cvCloneSeq(filteredPoints1, filteredPointsStorage);
		} else {
			if (maxNPoints == filteredPoints2->total) {
				leftTop = cvPoint(leftTopReg2.x, leftTopReg2.y);
				rightBottom = cvPoint(rightBottomReg2.x, rightBottomReg2.y);
				filteredPoints = cvCloneSeq(filteredPoints2, filteredPointsStorage);
			} else {
				if (maxNPoints == filteredPoints3->total) {
					leftTop = cvPoint(leftTopReg3.x, leftTopReg3.y);
					rightBottom = cvPoint(rightBottomReg3.x, rightBottomReg3.y);
					filteredPoints = cvCloneSeq(filteredPoints3, filteredPointsStorage);
				} else {
					leftTop = cvPoint(leftTopReg4.x, leftTopReg4.y);
					rightBottom = cvPoint(rightBottomReg4.x, rightBottomReg4.y);
					filteredPoints = cvCloneSeq(filteredPoints4, filteredPointsStorage);
				}
			}
		}
	
		cvRectangle(srcImg, leftTop, rightBottom, CV_RGB(255, 0, 0));

		// release
		cvReleaseMemStorage(&filteredPoints1Storage);
		cvReleaseMemStorage(&filteredPoints2Storage);
		cvReleaseMemStorage(&filteredPoints3Storage);
		cvReleaseMemStorage(&filteredPoints4Storage);	
	}

	CvPoint result = meanPoint(filteredPoints);
	cvCircle(srcImg, result, 1, CV_RGB(0, 0, 255));

	// release	
	cvReleaseMemStorage(&intersectionPointsStorage);
	cvReleaseMemStorage(&filteredPointsStorage);

	return result;
}


void VanishingPointDetection::filterPointsWithRespectToRegion(CvSeq* srcPoints, CvPoint leftTop, CvPoint rightBottom, CvSeq* desPoints) {
	for(int i = 0; i < srcPoints->total; i++) {
		CvPoint* point = (CvPoint*)cvGetSeqElem(srcPoints, i);
		if ( (leftTop.x <= point->x && point->x <= rightBottom.x) && (leftTop.y <= point->y && point->y <= rightBottom.y) ) // point is inside [leftTop, rightBottom]
			cvSeqPush(desPoints, point);
	}
}


void VanishingPointDetection::findIntersectionPoints(IplImage* srcImg, CvSeq* intersectionPoints, int voteThr) { // srcImg is a RGB image
	
	// convert srcImg to grayImg
	IplImage* grayImg = cvCreateImage(cvSize(srcImg->width, srcImg->height), IPL_DEPTH_8U, 1);
	cvConvertImage(srcImg, grayImg);


	// detect edge on grayImg
	IplImage* edgeImg = cvCreateImage(cvGetSize(grayImg), IPL_DEPTH_8U, 1); 
	cvCanny(grayImg, edgeImg, 50, 200, 3);										// THONG SO


	// detect lines, luu lai theo dang diem dau, diem cuoi (Ax, Ay, Bx, By)
	CvMemStorage* storage = cvCreateMemStorage(0);
	double rho = 1, theta = 0.01;												// THONG SO
	int linesMax = 20;											// THONG SO
	
	CvSeq* lines = cvHoughLines2(edgeImg, storage, CV_HOUGH_STANDARD, rho, theta, voteThr);
	int nLines = MIN(lines->total, linesMax);

	for(int i = 0; i < nLines; i++) {
		float* lineA = (float*)cvGetSeqElem(lines, i); // lineA = (rho, theta)

		for(int j = i + 1; j < nLines; j++) {
			float* lineB = (float*)cvGetSeqElem(lines, j);

			// find the intersected point between 2 lines
			double x = calculateX(lineA[0], lineB[0], lineA[1], lineB[1]);
			double y = calculateY(lineA[0], lineA[1], x);
			CvPoint point = cvPoint((int)x, (int)y);
			cvSeqPush(intersectionPoints, &point);

			// draw the result
			//cvCircle(srcImg, point, 1, CV_RGB(255, 0, 0));
		}

		// used to draw the result
		float rho = lineA[0], theta = lineA[1];
		CvPoint pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;
		pt1.x = cvRound(x0 + 1000*(-b));	pt2.x = cvRound(x0 - 1000*(-b));
		pt1.y = cvRound(y0 + 1000*(a));     pt2.y = cvRound(y0 - 1000*(a));
		cvLine(srcImg, pt1, pt2, CV_RGB(0, 255, 0));
	} 


	// release
	cvReleaseImage(&grayImg);
	cvReleaseImage(&edgeImg);
	cvReleaseMemStorage(&storage);
}


// ============================================= PRIVATE =============================================

double VanishingPointDetection::calculateX(double rho1, double rho2, double theta1, double theta2) {
	double sinTheta1 = sin(theta1);
	double sinTheta2 = sin(theta2);
	return (rho2*sinTheta1 - rho1*sinTheta2) / (cos(theta2)*sinTheta1 - cos(theta1)*sinTheta2);
}

double VanishingPointDetection::calculateY(double rho1, double theta1, double x) {
	return (rho1 - x*cos(theta1)) / sin(theta1);
}

CvPoint VanishingPointDetection::meanPoint(CvSeq* points) {
	if (points->total != 0) {
		int sumX = 0, sumY = 0;
		for (int i = 0; i < points->total; i++) {
			CvPoint* point = (CvPoint*)cvGetSeqElem(points, i);
			sumX += point->x;
			sumY += point->y;
		}
		return cvPoint( int(sumX/points->total), int(sumY/points->total) );
	} else
		return cvPoint(0, 0);
}