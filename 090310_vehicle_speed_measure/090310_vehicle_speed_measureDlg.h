// 090310_vehicle_speed_measureDlg.h : header file
//

#pragma once

#include "cv.h"
#include "highgui.h"
#include "ImageRectification.h"
#include "BackgroundSubtraction.h"
#include "BlobResult.h"
#include "Correlation.h"
#include "HistogramMatching.h"
#include "Util.h"
#include "VanishingPointDetection.h"
#include "ParticleFilter.h"
#include "afxmt.h"


// CMy090310_vehicle_speed_measureDlg dialog
class CMy090310_vehicle_speed_measureDlg : public CDialog
{
// Construction
public:
	CMy090310_vehicle_speed_measureDlg(CWnd* pParent = NULL);	// standard constructor

// Dialog Data
	enum { IDD = IDD_MY090310_VEHICLE_SPEED_MEASURE_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support


// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()

private:
	void fromBinarizedBackgroundImgToBlobDetectionResult();
	void matchObj(CBlob objBlob, IplImage* preImg, IplImage* curImg, int matchingWinHeightLocal, int velocityLocal, double coefficientThrLocal, int i);
	void particleFilterTrackingMethod(int i, IplImage* previousImg);

	void initVariables();
	void initGUI();
	void getInputParam();
	

public:
	void displayResult();

	afx_msg void OnBnClickedButtonLoadImg();
	afx_msg void OnBnClickedButtonQuit();
	afx_msg void OnBnClickedButtonAffineRectifyOneVanishingPoint();
	afx_msg void OnBnClickedButtonGetForeObj();
	afx_msg void OnBnClickedButtonVehicleTracking();
	afx_msg void OnBnClickedButtonSeqBrowseImg();
	afx_msg void OnBnClickedButtonSeqRun();
	afx_msg void OnBnClickedButtonSeqCreateBackground();
	afx_msg void OnBnClickedButtonSeqBrowseBackgroundImg();
	afx_msg void OnBnClickedButtonPhmLoadImg();
	afx_msg void OnBnClickedButtonPhmCompensateBackground();
	afx_msg void OnBnClickedButtonFindVanishingPoint();
	afx_msg void OnBnClickedButtonFindVanishingPointBrowseImg();
	afx_msg void OnBnClickedButtonExComparePhmIfdPhm();
	afx_msg void OnBnClickedButtonExComparePhmIfdIfd();
	afx_msg void OnBnClickedButtonExComparePhmIfdNo();
	afx_msg void OnBnClickedButtonSeqLoad1BackgroundImg();
	afx_msg void OnBnClickedButtonExConvertRgbToGray();
	afx_msg void OnBnClickedButtonExBatchRename();
	afx_msg void OnBnClickedButtonExCombineM2dResults();
};
