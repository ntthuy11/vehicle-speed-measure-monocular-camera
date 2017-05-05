#pragma once

#include "cv.h"
#include "highgui.h"

#include "BlobResult.h"


class Util
{
public:
	Util(void);
	~Util(void);


	static void writeVehicleBlobsToFile(CvSeq* blobInfoList) {
		//CStdioFile f;	f.Open(L"vehicleBlobs.txt", CFile::modeCreate | CFile::modeWrite);
		CStdioFile f;	f.Open(L"vehicleBlobs.txt", CFile::modeCreate | CFile::modeWrite | CFile::modeNoTruncate);
		f.SeekToEnd();

		CString text;

		const int nImgInOneSecond = 5; // 0.2s to capture 2 successive frames
		const double ratioBtwCarLengthInMetersAndInPixels = 0.57; // 4m / 7pixels
		const double ratioMToKm = 3.6; // 3600s / 1000m

		for (int i = 0; i < blobInfoList->total; i++) {
			CvSeq* aBlobInfo = (CvSeq*)cvGetSeqElem(blobInfoList, i);

			for (int j = 0; j < aBlobInfo->total; j += 2) {
				int* centerXOfABlob = (int*)cvGetSeqElem(aBlobInfo, j);			text.Format(L"[%3d  ", *centerXOfABlob);	f.WriteString(text);
				int* distanceOfABlob = (int*)cvGetSeqElem(aBlobInfo, j + 1);	text.Format(L"%2d  ", *distanceOfABlob);	f.WriteString(text);
				
				double velocity = (*distanceOfABlob) * ratioBtwCarLengthInMetersAndInPixels * nImgInOneSecond * ratioMToKm;	
				text.Format(L"%3.2f]  ", velocity);			f.WriteString(text);
			}
			f.WriteString(L"\n");
		}
		f.WriteString(L"\n");
		f.Close();
	}


	static void storeVehicleBlobs(CvSeq* blobInfoList, int centerXOfNewBlob, int distance) {
		int disThr = 5;
		bool isFoundExistingBlob = false;
		
		for (int i = 0; i < blobInfoList->total; i++) {
			CvSeq* aBlobInfo = (CvSeq*)cvGetSeqElem(blobInfoList, i); // tuong ung voi moi frame track duoc, thi luu 2 thong so: toa do x, va distant

			for (int j = 0; j < aBlobInfo->total; j += 2) {
				int* centerXOfABlob = (int*)cvGetSeqElem(aBlobInfo, j);

				if (abs(*centerXOfABlob - centerXOfNewBlob) <= disThr) {
					cvSeqPush(aBlobInfo, &centerXOfNewBlob);
					cvSeqPush(aBlobInfo, &distance);
					isFoundExistingBlob = true;
					break;
				}
			}
		}

		if (isFoundExistingBlob == false) {
			CvMemStorage* aBlobInfoStorage = cvCreateMemStorage(0);
			CvSeq* aBlobInfo = cvCreateSeq(CV_SEQ_ELTYPE_GENERIC, sizeof(CvSeq), sizeof(int), aBlobInfoStorage);
			cvSeqPush(aBlobInfo, &centerXOfNewBlob);
			cvSeqPush(aBlobInfo, &distance);
			cvSeqPush(blobInfoList, aBlobInfo);
		}
	}


	static CvPoint3D32f minimizeIFD(IplImage* src1, IplImage* src2) { // both images are gray
		int width = src1->width, height = src1->height;
		double minAvgIntensity = 1000;
		int resultDx, resultDy;

		//for (int dy = -(height - 1); dy < height; dy++) {
		//	for (int dx = -(width - 1); dx < width; dx++) {
		//int d = 4; // [-3; 3]
		int d = 10; // [-9; 9]
		for (int dy = -(d-1); dy < d; dy++) {
			for (int dx = -(d-1); dx < d; dx++) {
				double avgIntensity = subtractImgWithAvgIntensity(src1, src2, dx, dy);
				if (minAvgIntensity > avgIntensity) {
					minAvgIntensity = avgIntensity;
					resultDx = dx;
					resultDy = dy;
				}
			}
		}

/*		int step = src1->widthStep;
		const uchar* src1Data = (uchar *)src1->imageData;
		const uchar* src2Data = (uchar *)src2->imageData;
		int n = 0;
		int totalError = 0;
		for (int i = 0; i < height; i++) {
			int newI = i + d;
			if (0 <= newI && newI < height) {
				for (int j = 0; j < width; j++) {
					int newJ = j + d;
					if (0 <= newJ && newJ < width) {
						int diff = src1Data[i*step + j] - src2Data[newI*step + newJ];
						if (diff < 0) diff = -diff;
						n++;
						totalError += diff;
					}
				}
			}
		}*/
		
		return cvPoint3D32f(resultDx, resultDy, minAvgIntensity);
	}


	static double subtractImgWithAvgIntensity(IplImage* src1, IplImage* src2, int dx, int dy) { // both images are gray
		int step = src1->widthStep, width = src1->width, height = src1->height;
		const uchar* src1Data = (uchar *)src1->imageData;
		const uchar* src2Data = (uchar *)src2->imageData;

		int heightFrom, heightTo, widthFrom, widthTo;
		if (dy > 0) {	heightFrom = 0;			heightTo = height - dy;	}
		else		{	heightFrom = abs(dy);	heightTo = height;		}
		if (dx > 0) {	widthFrom = 0;			widthTo = width - dx;	}
		else		{	widthFrom = abs(dx);	widthTo = width;		}

		int sumIntensity = 0;
		for (int i = heightFrom; i < heightTo; i++) 
			for (int j = widthFrom; j < widthTo; j++) 
				sumIntensity += abs(src2Data[i*step + j] - src1Data[(i + dy)*step + (j + dx)]);

		return sumIntensity * 1.0 / ( (widthTo - widthFrom) * (heightTo - heightFrom) );
	}


	static double calculateAvgIntensity(IplImage* src, int dx, int dy) { // src is gray
		int step = src->widthStep, width = src->width, height = src->height;
		const uchar* srcData = (uchar *)src->imageData;

		int heightFrom, heightTo, widthFrom, widthTo;
		if (dy > 0) {	heightFrom = 0;			heightTo = height - dy;	}
		else		{	heightFrom = abs(dy);	heightTo = height;		}
		if (dx > 0) {	widthFrom = 0;			widthTo = width - dx;	}
		else		{	widthFrom = abs(dx);	widthTo = width;		}
		
		int sumIntensity = 0;
		for (int i = heightFrom; i < heightTo; i++)	
			for (int j = widthFrom; j < widthTo; j++)		
				sumIntensity += srcData[i*step + j];

		return sumIntensity * 1.0 / ( (widthTo - widthFrom) * (heightTo - heightFrom) );
	}


	static void subtractImg(IplImage* src1, IplImage* src2, int dx, int dy, IplImage* des) {
		int channels = src1->nChannels, step = src1->widthStep, width = src1->width, height = src1->height;
		const uchar* src1Data = (uchar *)src1->imageData;
		const uchar* src2Data = (uchar *)src2->imageData;
		uchar* desData = (uchar *)des->imageData;

		for (int i = 0; i < height; i++) {
			for (int j = 0; j < width; j++) {
				for (int k = 0; k < channels; k++) {
					int pos = i*step + j*channels + k;

					int iPlusDy = i + dy;
					int jPlusDx = j + dx;
					if ( (iPlusDy < 0 || height <= iPlusDy) || (jPlusDx < 0 || width <= jPlusDx) )	
						desData[pos] = 0;
					else 
						desData[pos] = abs(src2Data[pos] - src1Data[iPlusDy*step + jPlusDx*channels + k]);					

					/*int iMinusDy = i - dy;
					int jMinusDx = j - dx;
					if ( (iMinusDy < 0 || height <= iMinusDy) || (jMinusDx < 0 || width <= jMinusDx) )	
						desData[pos] = 0;
					else 
						desData[pos] = abs(src2Data[iMinusDy*step + jMinusDx*channels + k] - src1Data[pos]);*/
				}
			}
		}
	}


	static void cropImg(IplImage* src, CvRect roi, IplImage* des) {
		int srcStep = src->widthStep;	const uchar* srcData = (uchar *)src->imageData;
		int desStep = des->widthStep;	uchar* desData = (uchar *)des->imageData;

		int startX = roi.x;		int endX = startX + roi.width;
		int startY = roi.y;		int endY = startY + roi.height;
		for(int i = startY; i < endY; i++) 
			for(int j = startX; j < endX; j++) 
				for(int k = 0; k < src->nChannels; k++) 
					desData[(i-startY)*desStep + (j-startX)*src->nChannels + k] = srcData[i*srcStep + j*src->nChannels + k];
	}


	static void copyFile(LPCTSTR srcFilename, LPCTSTR desFilename) {
		// open files
		CFile srcFile;
		CFile desFile;
		srcFile.Open(srcFilename, CFile::modeRead | CFile::shareDenyWrite);
		desFile.Open(desFilename, CFile::modeCreate | CFile::modeWrite | CFile::shareExclusive);
		
		// copy files
		BYTE buffer[4096];
		DWORD dwRead;		
		do { // Read in 4096-byte blocks, remember how many bytes were actually read, and try to write that many out. This loop ends when there are no more bytes to read.
			dwRead = srcFile.Read(buffer, 4096);
			desFile.Write(buffer, dwRead);
		} while (dwRead > 0);

		// close files
		desFile.Close();
		srcFile.Close();
	}


	static void printHistogram(double* hist) {
		CStdioFile f;	f.Open(L"histograms.txt", CFile::modeCreate | CFile::modeWrite | CFile::modeNoTruncate);
		f.SeekToEnd();

		CString text;
		for (int i = 0; i < NUM_BINS_RGB; i++) {
			text.Format(L"%1.2f  ", hist[i]);
			f.WriteString(text);
		}
		f.Close();
	}


	// http://www.codeproject.com/cpp/floatutils.asp?df=100&forumid=208&exp=0&select=14154
	static double roundDouble(double doValue, int nPrecision) {
		static const double doBase = 10.0;
		double doComplete5, doComplete5i;
	    
		doComplete5 = doValue * pow(doBase, (double) (nPrecision + 1));
	    
		if(doValue < 0.0) doComplete5 -= 5.0;
		else doComplete5 += 5.0;
	    
		doComplete5 /= doBase;
		modf(doComplete5, &doComplete5i);
	    
		return doComplete5i / pow(doBase, (double) nPrecision);
	}
};
