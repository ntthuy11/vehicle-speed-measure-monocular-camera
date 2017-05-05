// 090310_vehicle_speed_measureDlg.cpp : implementation file
//

#include "stdafx.h"
#include "090310_vehicle_speed_measure.h"
#include "090310_vehicle_speed_measureDlg.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// ==========================================================================================================

#define FRAME_WIDTH		(640)
#define FRAME_HEIGHT	(480)

#define N_COLORS		(6)


// -------- "Using Image" group box --------

#define SHIFT_X			(0)
#define SHIFT_Y			(0)

#define VANISHING_POINT_X			(304)
#define VANISHING_POINT_Y			(-33)

#define BINARIZE_THR				(20)
#define MOR_CLOSE_STRUCTURE_SIZE	(5)
#define MOR_CLOSE_ITERATION			(3)
#define N_BLOBS_DETECTED			(6)
#define BLOB_AREA_THR				(50)

#define MATCHING_WIN_HEIGHT			(10)
#define VELOCITY					(100)
#define COEFFICIENT_THR				(0.95)


// -------- "Using Image Sequence" group box --------

//#define SEQ_N_IMG_TO_CREATE_BCKGRD		(4)		// not used
#define SEQ_VANISHING_POINT_X			(304)
#define SEQ_VANISHING_POINT_Y			(33)
#define SEQ_SHIFT_Y						(-600)
#define SEQ_VANISHING_POINT_VOTE_THR	(250)
#define SEQ_VANISHING_POINT_LEVEL		(3)

#define SEQ_BINARIZE_THR				(40)
#define SEQ_MOR_CLOSE_STRUCTURE_SIZE	(3)
#define SEQ_MOR_CLOSE_ITERATION			(1)
#define SEQ_N_BLOBS_DETECTED			(4)
#define SEQ_BLOB_AREA_THR				(15)

#define SEQ_CROP_FROM_TOP				(0)
#define SEQ_CROP_FROM_BOTTOM			(300)
#define SEQ_CROP_FROM_LEFT				(240)
#define SEQ_CROP_FROM_RIGHT				(240)

#define SEQ_PHM_RANGE_DX_FROM			(-5)
#define SEQ_PHM_RANGE_DX_TO				(5)
#define SEQ_PHM_RANGE_DY_FROM			(-5)
#define SEQ_PHM_RANGE_DY_TO				(5)
#define SEQ_PHM_SMOOTH_WIN_SIZE			(7)

#define SEQ_MATCHING_WIN_HEIGHT			(10)
#define SEQ_VELOCITY					(20)
#define SEQ_COEFFICIENT_THR				(0.8)

#define SEQ_PF_N_PARTICLES				(50)
#define SEQ_PF_POSITION_SIGMA			(5.0)
#define SEQ_PF_VELOCITY_SIGMA			(3.0)
#define SEQ_PF_PARTICLE_WIDTH			(6)
#define SEQ_PF_PARTICLE_HEIGHT			(6)


// -------- "Projection Histogram Matching" group box --------

#define PHM_RANGE_DX_FROM				(-10)
#define PHM_RANGE_DX_TO					(10)
#define PHM_RANGE_DY_FROM				(-10)
#define PHM_RANGE_DY_TO					(10)
#define PHM_SMOOTH_WIN_SIZE				(9)


// -------- "Experiments for the paper" group box --------

#define EX_FROM_IMG						(8)

#define EX_N_TEST_IMG					(97)
//#define EX_N_TEST_IMG					(100)

#define EX_DISPLACEMENT_RANGE			(5)		// [-4; 4]
//#define EX_DISPLACEMENT_RANGE			(10)	// [-9; 9]


// ==========================================================================================================

ImageRectification imgRectification;
BackgroundSubtraction backgrdSubtraction;
CBlobResult blobResult;
Correlation correlation;
VanishingPointDetection vpDetection;
ParticleFilter* particleFilters;

IplImage *img1, *img2, *img3, *img4, *img5, *img6, *img7, *img8, *currentImg;
CvScalar* colors;
CvFont font;

unsigned long timeDiff;
__int64 freq, tStart, tStop;


CvMemStorage* blobInfoListStorage;
CvSeq* blobInfoList;
CStdioFile aFile;

int numBlobs;

// -------- "Using Image" group box --------
int shiftX, shiftY, vanishingPointX, vanishingPointY;
int binarizeThr, morCloseStructureSize, morCloseIteration, nBlobsDetected, blobAreaThr;

int matchingWinHeight, velocity;
double coefficientThr;


// -------- "Using Image Sequence" group box --------
int seqNImg, seqVanishingPointX, seqVanishingPointY, seqShiftY, seqVanishingPointVoteThr, seqVanishingPointLevel, seqNImgToCreateBckgrd, seqBinarizeThr, seqMorCloseStructureSize, seqMorCloseIteration;
int seqNBlobsDetected, seqBlobAreaThr;
int seqCropFromTop, seqCropFromBottom, seqCropFromLeft, seqCropFromRight;
int seqPhmRangeDxFrom, seqPhmRangeDxTo, seqPhmRangeDyFrom, seqPhmRangeDyTo, seqPhmSmoothWinSize;
int seqMatchingWinHeight, seqVelocity;
double seqCoefficientThr;
CString *filenames, *backgrdFilenames;
CvRect seqRectForROI;
CvPoint seqDisplacement = cvPoint(-100, -100);

int seqPFnParticles, seqPFparticleWidth, seqPFparticleHeight;
double seqPFpositionSigma, seqPFvelocitySigma;
CvPoint* previousPositions;


// -------- "Projection Histogram Matching" group box --------
int phmNImg, phmRangeDxFrom, phmRangeDxTo, phmRangeDyFrom, phmRangeDyTo, phmSmoothWinSize;
CString *phmFilenames;
CvPoint displacement = cvPoint(-100, -100);
double phmError;


// -------- "Experiments for the paper" group box --------
double exAvgDx, exAvgDy;
int exMaxDx, exMaxDy;
double exAvgError;
double exAvgTimeDiff;


// ========== using thread ==========
int _dx, _dy;
CSemaphore semaDx, semaDy; 

UINT matchWithRangeDxThread(LPVOID pParam) {
	CSingleLock semlock(&semaDx);
	semlock.Lock();

	HistogramMatching* histMatching = (HistogramMatching *)pParam;
	histMatching->initVerHist();
	_dx = histMatching->matchWithRangeDx(cvPoint(-EX_DISPLACEMENT_RANGE, EX_DISPLACEMENT_RANGE));
	//histMatching->releaseVerHist();

	semlock.Unlock();
	return 0;
}

UINT matchWithRangeDyThread(LPVOID pParam) {
	CSingleLock semlock(&semaDy);
	semlock.Lock();

	HistogramMatching* histMatching = (HistogramMatching *)pParam;
	histMatching->initHorHist();
	_dy = histMatching->matchWithRangeDy(cvPoint(-EX_DISPLACEMENT_RANGE, EX_DISPLACEMENT_RANGE));
	//histMatching->releaseHorHist();

	semlock.Unlock();
	return 0;
}

// ==========================================================================================================

CMy090310_vehicle_speed_measureDlg::CMy090310_vehicle_speed_measureDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CMy090310_vehicle_speed_measureDlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CMy090310_vehicle_speed_measureDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CMy090310_vehicle_speed_measureDlg, CDialog)
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	//}}AFX_MSG_MAP
	ON_BN_CLICKED(IDC_BUTTON_LOAD_IMG, &CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonLoadImg)
	ON_BN_CLICKED(IDC_BUTTON_QUIT, &CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonQuit)
	ON_BN_CLICKED(IDC_BUTTON_AFFINE_RECTIFY_ONE_VANISHING_POINT, &CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonAffineRectifyOneVanishingPoint)
	ON_BN_CLICKED(IDC_BUTTON_GET_FORE_OBJ, &CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonGetForeObj)
	ON_BN_CLICKED(IDC_BUTTON_VEHICLE_TRACKING, &CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonVehicleTracking)
	ON_BN_CLICKED(IDC_BUTTON_SEQ_BROWSE_IMG, &CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonSeqBrowseImg)
	ON_BN_CLICKED(IDC_BUTTON_SEQ_RUN, &CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonSeqRun)
	ON_BN_CLICKED(IDC_BUTTON_SEQ_CREATE_BACKGROUND, &CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonSeqCreateBackground)
	ON_BN_CLICKED(IDC_BUTTON_SEQ_BROWSE_BACKGROUND_IMG, &CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonSeqBrowseBackgroundImg)
	ON_BN_CLICKED(IDC_BUTTON_PHM_LOAD_IMG, &CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonPhmLoadImg)
	ON_BN_CLICKED(IDC_BUTTON_PHM_COMPENSATE_BACKGROUND, &CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonPhmCompensateBackground)
	ON_BN_CLICKED(IDC_BUTTON_FIND_VANISHING_POINT, &CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonFindVanishingPoint)
	ON_BN_CLICKED(IDC_BUTTON_FIND_VANISHING_POINT_BROWSE_IMG, &CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonFindVanishingPointBrowseImg)
	ON_BN_CLICKED(IDC_BUTTON_EX_COMPARE_PHM_IFD_PHM, &CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonExComparePhmIfdPhm)
	ON_BN_CLICKED(IDC_BUTTON_EX_COMPARE_PHM_IFD_IFD, &CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonExComparePhmIfdIfd)
	ON_BN_CLICKED(IDC_BUTTON_EX_COMPARE_PHM_IFD_NO, &CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonExComparePhmIfdNo)
	ON_BN_CLICKED(IDC_BUTTON_SEQ_LOAD_1_BACKGROUND_IMG, &CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonSeqLoad1BackgroundImg)
	ON_BN_CLICKED(IDC_BUTTON_EX_CONVERT_RGB_TO_GRAY, &CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonExConvertRgbToGray)
	ON_BN_CLICKED(IDC_BUTTON_EX_BATCH_RENAME, &CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonExBatchRename)
	ON_BN_CLICKED(IDC_BUTTON_EX_COMBINE_M2D_RESULTS, &CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonExCombineM2dResults)
END_MESSAGE_MAP()


// CMy090310_vehicle_speed_measureDlg message handlers

BOOL CMy090310_vehicle_speed_measureDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	// TODO: Add extra initialization here
	initVariables();
	initGUI();

	return TRUE;  // return TRUE  unless you set the focus to a control
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CMy090310_vehicle_speed_measureDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialog::OnPaint();
	}
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CMy090310_vehicle_speed_measureDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

// ==========================================================================================================

void CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonLoadImg() {
	CFileDialog dlg(TRUE, _T("*.avi"), _T(""), OFN_FILEMUSTEXIST|OFN_PATHMUSTEXIST|OFN_HIDEREADONLY, _T("Image (*.bmp)|*.bmp|All Files (*.*)|*.*||"),NULL);	
	if (dlg.DoModal() != IDOK) return;
	
	// load image
	char filenameInMultiByte[256];
	WideCharToMultiByte(CP_ACP, 0, dlg.GetPathName(), -1, filenameInMultiByte, 256, NULL, NULL);	

	// img1
	cvReleaseImage(&img1);
	img1 = cvvLoadImage(filenameInMultiByte);
	cvShowImage("img1", img1);

	// img2
	cvReleaseImage(&img2);
	img2 = cvCreateImage(cvSize(img1->width, img1->height), IPL_DEPTH_8U, img1->nChannels);
	cvZero(img2);
}

void CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonAffineRectifyOneVanishingPoint() {
	getInputParam();

	cvZero(img2);

	//imgRectification.affineRectifyWithOneVanishingPoint(img1, cvPoint(316, 79), cvPoint(shiftX, shiftY), img2);	
	//imgRectification.inverseAffineRectifyForImage(img1, cvPoint(316, 79), shiftY, img2);	
	
	//imgRectification.inverseAffineRectifyForImage(img1, cvPoint(207, 45), shiftY, img2);	
	//imgRectification.affineRectifyWithOneVanishingPoint(img1, cvPoint(207, 45), cvPoint(shiftX, shiftY), img2);	// road2(0, -330)

	//imgRectification.affineRectifyWithOneVanishingPoint(img1, cvPoint(252, 71), cvPoint(shiftX, shiftY), img2);	// road3(0, -380)
	//imgRectification.affineRectifyWithOneVanishingPoint(img1, cvPoint(268, 69), cvPoint(shiftX, shiftY), img2);	// road4(0, -380)
	//imgRectification.affineRectifyWithOneVanishingPoint(img1, cvPoint(192, 90), cvPoint(shiftX, shiftY), img2);		// road5(0, -300)
	//imgRectification.affineRectifyWithOneVanishingPoint(img1, cvPoint(182, 122), cvPoint(shiftX, shiftY), img2);		// road6(0, -360)

	//imgRectification.affineRectifyWithOneVanishingPoint(img1, cvPoint(304, -33), cvPoint(shiftX, shiftY), img2);

	imgRectification.affineRectifyWithOneVanishingPoint(img1, cvPoint(vanishingPointX, vanishingPointY), cvPoint(shiftX, shiftY), img2);

	cvShowImage("img2", img2);
}


void CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonGetForeObj() {
	getInputParam();

	// ----------- create "empty" background -----------

	IplImage* im1 = cvvLoadImage("..\\testdata\\test_imgRectify\\road\\road2_(207,45)(0,-330)\\rectified_road\\1.bmp");
	IplImage* im2 = cvvLoadImage("..\\testdata\\test_imgRectify\\road\\road2_(207,45)(0,-330)\\rectified_road\\2.bmp");
	IplImage* im3 = cvvLoadImage("..\\testdata\\test_imgRectify\\road\\road2_(207,45)(0,-330)\\rectified_road\\3.bmp");
	IplImage* im4 = cvvLoadImage("..\\testdata\\test_imgRectify\\road\\road2_(207,45)(0,-330)\\rectified_road\\4.bmp");	
	
	/*IplImage* im1 = cvvLoadImage("..\\testdata\\1_vanishing_point\\road2_(207,45)(0,-330)\\backgrd\\1_background1.bmp");
	IplImage* im2 = cvvLoadImage("..\\testdata\\1_vanishing_point\\road2_(207,45)(0,-330)\\backgrd\\1_background2.bmp");
	IplImage* im3 = cvvLoadImage("..\\testdata\\1_vanishing_point\\road2_(207,45)(0,-330)\\backgrd\\1_background3.bmp");
	IplImage* im4 = cvvLoadImage("..\\testdata\\1_vanishing_point\\road2_(207,45)(0,-330)\\backgrd\\1_background4.bmp");*/

	/*IplImage* im1 = cvvLoadImage("..\\testdata\\1_vanishing_point\\road2_(207,45)(0,-330)\\rectified_backgrd\\1_background1_result.bmp");
	IplImage* im2 = cvvLoadImage("..\\testdata\\1_vanishing_point\\road2_(207,45)(0,-330)\\rectified_backgrd\\1_background2_result.bmp");
	IplImage* im3 = cvvLoadImage("..\\testdata\\1_vanishing_point\\road2_(207,45)(0,-330)\\rectified_backgrd\\1_background3_result.bmp");
	IplImage* im4 = cvvLoadImage("..\\testdata\\1_vanishing_point\\road2_(207,45)(0,-330)\\rectified_backgrd\\1_background4_result.bmp");*/
	
	img1 = cvCreateImage(cvSize(im1->width, im1->height), IPL_DEPTH_8U, im1->nChannels);
	IplImage** listOfImg = new IplImage*[4];	listOfImg[0] = im1;		listOfImg[1] = im2;		listOfImg[2] = im3;		listOfImg[3] = im4;	

	backgrdSubtraction.createBackground(listOfImg, 4, img1);
	cvShowImage("img1", img1);

	cvReleaseImage(&im1);	cvReleaseImage(&im2);	cvReleaseImage(&im3);	cvReleaseImage(&im4);
	delete[] listOfImg;


	// ----------- subtract background -----------

	IplImage* im5 = cvvLoadImage("..\\testdata\\test_imgRectify\\road\\road2_(207,45)(0,-330)\\rectified_road\\1.bmp");	
	//IplImage* im5 = cvvLoadImage("..\\testdata\\1_vanishing_point\\road2_(207,45)(0,-330)\\backgrd\\1_background1.bmp");	
	//IplImage* im5 = cvvLoadImage("..\\testdata\\1_vanishing_point\\road2_(207,45)(0,-330)\\rectified_backgrd\\1_background1_result.bmp");	

	img2 = cvCreateImage(cvSize(im5->width, im5->height), IPL_DEPTH_8U, im5->nChannels);
	cvSub(img1, im5, img2);


	// ----------- fromSubtractedForegroundImgToBlobDetectionResult -----------

	fromBinarizedBackgroundImgToBlobDetectionResult();
	
	CBlob* blobsWithLargestArea = new CBlob[nBlobsDetected];

	CvScalar* colors = new CvScalar[nBlobsDetected];
	colors[0] = CV_RGB(255, 0, 0);
	colors[1] = CV_RGB(0, 255, 0);
	colors[2] = CV_RGB(0, 0, 255);
	colors[3] = CV_RGB(255, 255, 0);
	colors[4] = CV_RGB(255, 0, 255);
	colors[5] = CV_RGB(0, 255, 255);

	for (int i = 0; i < nBlobsDetected; i++) {		
		blobResult.GetNthBlob(CBlobGetArea(), i, blobsWithLargestArea[i]); 
		blobsWithLargestArea[i].FillBlob(img5, colors[i]); // only used to display
	}	

	cvShowImage("img2", img2);
	cvShowImage("img3", img3);
	cvShowImage("img4", img4);
	cvShowImage("img5", img5);

	cvReleaseImage(&im5);
	delete[] blobsWithLargestArea;
	delete[] colors;
}


void CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonVehicleTracking() {
	getInputParam();
	img2 = cvvLoadImage("..\\testdata\\test_imgRectify\\road\\road2_(207,45)(0,-330)\\rectified_road\\velocity\\2_1.bmp"); // load and do vehicle detection - FRAME i
	fromBinarizedBackgroundImgToBlobDetectionResult();
	IplImage* curImg = cvvLoadImage("..\\testdata\\test_imgRectify\\road\\road2_(207,45)(0,-330)\\rectified_road\\velocity\\2_2.bmp"); // load the image for searching vehicles - FRAME (i+1)

	// find matching regions between sample vehicle (detected) and experimental vehicle
	CBlob* blobsWithLargestArea = new CBlob[nBlobsDetected];
	for (int i = 0; i < nBlobsDetected; i++) {
		blobResult.GetNthBlob(CBlobGetArea(), i, blobsWithLargestArea[i]); 
		matchObj(blobsWithLargestArea[i], img2, curImg, matchingWinHeight, velocity, coefficientThr, i); // i is used to color the matching window
	}

	// display and release
	cvShowImage("img2", img2);
	cvShowImage("img3", curImg);
	delete[] blobsWithLargestArea;
	cvReleaseImage(&curImg);
}


void CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonSeqLoad1BackgroundImg() {
	CFileDialog dlg(TRUE, _T("*.avi"), _T(""), OFN_FILEMUSTEXIST|OFN_PATHMUSTEXIST|OFN_HIDEREADONLY, _T("Image (*.bmp)|*.bmp|All Files (*.*)|*.*||"),NULL);	
	if (dlg.DoModal() != IDOK) return;
	
	// load image
	char filenameInMultiByte[256];
	WideCharToMultiByte(CP_ACP, 0, dlg.GetPathName(), -1, filenameInMultiByte, 256, NULL, NULL);

	img1 = cvvLoadImage(filenameInMultiByte);
	cvShowImage("img1", img1);
}


void CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonSeqBrowseBackgroundImg() {
	backgrdFilenames = new CString[100]; // max filename = 100

	CFileDialog dlg(TRUE, _T("*.*"), _T(""), OFN_FILEMUSTEXIST|OFN_PATHMUSTEXIST|OFN_HIDEREADONLY|OFN_ALLOWMULTISELECT, _T("Image (*.bmp)|*.bmp|All Files (*.*)|*.*||"), NULL);	
	dlg.m_ofn.lpstrInitialDir = L"..\\testdata\\090329";

	if (dlg.DoModal() == IDOK) {
		POSITION pos = dlg.GetStartPosition();
		
		seqNImgToCreateBckgrd = 0;
		if (pos) {
			do {
				backgrdFilenames[seqNImgToCreateBckgrd++] = dlg.GetNextPathName(pos);
			} while(pos);
		}
	}
}


void CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonSeqCreateBackground() {
	getInputParam();

	char filenameInMultiByte[256];
	IplImage** listOfImg = new IplImage*[seqNImgToCreateBckgrd];

	for (int i = 0; i < seqNImgToCreateBckgrd; i++) {	
		WideCharToMultiByte(CP_ACP, 0, backgrdFilenames[i], -1, filenameInMultiByte, 256, NULL, NULL);
		listOfImg[i] = cvvLoadImage(filenameInMultiByte);		
	}

	img1 = cvCreateImage(cvSize(listOfImg[0]->width, listOfImg[0]->height), IPL_DEPTH_8U, listOfImg[0]->nChannels);
	backgrdSubtraction.createBackground(listOfImg, seqNImgToCreateBckgrd, img1);	
	cvShowImage("img1", img1);

	// release listOfImg
	for (int i = 0; i < seqNImgToCreateBckgrd; i++) cvReleaseImage(&listOfImg[i]);
	delete[] listOfImg;	
}


void CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonSeqBrowseImg() {
	filenames = new CString[100]; // max filename = 100

	CFileDialog dlg(TRUE, _T("*.*"), _T(""), OFN_FILEMUSTEXIST|OFN_PATHMUSTEXIST|OFN_HIDEREADONLY|OFN_ALLOWMULTISELECT, _T("Image (*.bmp)|*.bmp|All Files (*.*)|*.*||"), NULL);	
	dlg.m_ofn.lpstrInitialDir = L"..\\testdata\\090329";

	if (dlg.DoModal() == IDOK) {
		POSITION pos = dlg.GetStartPosition();
		
		seqNImg = 0;
		if (pos) {
			do {
				filenames[seqNImg++] = dlg.GetNextPathName(pos);
			} while(pos);
		}
	}
}


void CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonSeqRun() { // already created background image
	getInputParam();
	char filenameInMultiByte[256];
	seqRectForROI = cvRect(seqCropFromLeft, seqCropFromTop, img1->width - seqCropFromLeft - seqCropFromRight, img1->height - seqCropFromTop - seqCropFromBottom);
	//seqRectForROI = cvRect(0, 0, img1->width, img1->height); // no crop


	// used to save the trajectory of each vehicle
	blobInfoListStorage = cvCreateMemStorage(0);
	blobInfoList = cvCreateSeq(CV_SEQ_ELTYPE_GENERIC, sizeof(CvSeq), sizeof(CvSeq), blobInfoListStorage);

	// init
	IplImage* previousImg = cvCreateImage(cvSize(img1->width, img1->height), IPL_DEPTH_8U, img1->nChannels);	

	for (int i = 0; i < seqNImg; i++) { // seqNImg

		// ---------- init ----------
		WideCharToMultiByte(CP_ACP, 0, filenames[i], -1, filenameInMultiByte, 256, NULL, NULL); // for "subtract current image to (created) "empty" background" step
		currentImg = cvvLoadImage(filenameInMultiByte);
		img2 = cvCreateImage(cvSize(img1->width, img1->height), IPL_DEPTH_8U, img1->nChannels);
		HistogramMatching histogramMatching(img1, currentImg);		// USING THREAD

		IplImage* rectifiedForeImg = cvCreateImage(cvSize(img2->width, img2->height), IPL_DEPTH_8U, img2->nChannels); // for "rectify just-created image, and crop it" step
		img7 = cvCreateImage(cvSize(seqRectForROI.width, seqRectForROI.height), IPL_DEPTH_8U, rectifiedForeImg->nChannels);	
		img3 = cvCreateImage(cvSize(seqRectForROI.width, seqRectForROI.height), IPL_DEPTH_8U, img7->nChannels);

		IplImage* grayOfSubtractedImg = cvCreateImage(cvSize(img3->width, img3->height), IPL_DEPTH_8U, 1); // for "binarize" step
		img4 = cvCreateImage(cvSize(img3->width, img3->height), IPL_DEPTH_8U, 1);

		img5 = cvCreateImage(cvSize(img4->width, img4->height), IPL_DEPTH_8U, img4->nChannels); // for "close operator" step
		IplImage* tmpImgForClose = cvCreateImage(cvSize(img4->width, img4->height), IPL_DEPTH_8U, img4->nChannels);
		int centerOfStructure = (seqMorCloseStructureSize - 1) / 2;
		IplConvKernel* element = cvCreateStructuringElementEx(seqMorCloseStructureSize, seqMorCloseStructureSize, centerOfStructure, centerOfStructure, CV_SHAPE_RECT);

		CBlob* blobsWithLargestArea = new CBlob[seqNBlobsDetected]; // for "choose method to track cars" step
		img6 = cvCreateImage(cvSize(img5->width, img5->height), IPL_DEPTH_8U, 3);	cvZero(img6);
		img8 = cvCreateImage(cvSize(img7->width, img7->height), IPL_DEPTH_8U, img7->nChannels);


		// ---------- subtract current image to (created) "empty" background ----------
		//QueryPerformanceFrequency((LARGE_INTEGER*)&freq);	QueryPerformanceCounter((LARGE_INTEGER*)&tStart); // tick count START

		/*histogramMatching.initVerHist(); // do not use thread
		histogramMatching.initHorHist();
		seqDisplacement = histogramMatching.matchWithRanges(cvPoint(seqPhmRangeDxFrom, seqPhmRangeDxTo), cvPoint(seqPhmRangeDyFrom, seqPhmRangeDyTo)); // SLOW (?)
		histogramMatching.releaseVerHist();
		histogramMatching.releaseHorHist();		*/
		
		CWinThread * pDxThread = AfxBeginThread(matchWithRangeDxThread, &histogramMatching);	pDxThread->ResumeThread();	pDxThread->SetThreadPriority(THREAD_PRIORITY_TIME_CRITICAL);
		CWinThread * pDyThread = AfxBeginThread(matchWithRangeDyThread, &histogramMatching);	pDyThread->ResumeThread();	pDyThread->SetThreadPriority(THREAD_PRIORITY_TIME_CRITICAL);		
		CSingleLock semlock1(&semaDx);		semlock1.Lock();
		CSingleLock semlock2(&semaDy);		semlock2.Lock();		
		seqDisplacement = cvPoint(_dx, _dy);

		Util::subtractImg(img1, currentImg, seqDisplacement.x, seqDisplacement.y, img2);
		//Util::subtractImg(img1, currentImg, 0, 0, img2); // no background compensation


		// ---------- rectify just-created image, and crop it ----------		
		imgRectification.affineRectifyWithOneVanishingPoint(img2, cvPoint(seqVanishingPointX, seqVanishingPointY), cvPoint(0, seqShiftY), rectifiedForeImg);
		Util::cropImg(rectifiedForeImg, seqRectForROI, img7);		
		cvSmooth(img7, img3, CV_GAUSSIAN, seqPhmSmoothWinSize);		


		// ---------- binarize ----------		
		cvCvtColor(img3, grayOfSubtractedImg, CV_BGR2GRAY);		
		cvThreshold(grayOfSubtractedImg, img4, seqBinarizeThr, 255, CV_THRESH_BINARY);	//cvAdaptiveThreshold(grayOfSubtractedImg, img4, 255);


		// ---------- close operator ----------		
		cvMorphologyEx(img4, img5, tmpImgForClose, element, CV_MOP_CLOSE, seqMorCloseIteration);


		// ---------- choose method to track cars ----------
		if ( ((CButton*)this->GetDlgItem(IDC_RADIO_BLOB_DETECTION_TRACKING))->GetCheck() == 1 ) { // checked
			blobResult = CBlobResult(img5, NULL, 0); // 0: background la black
			blobResult.Filter(blobResult, B_EXCLUDE, CBlobGetArea(), B_LESS, seqBlobAreaThr);
			cvCopyImage(img7, img8);

			for (int j = 0; j < seqNBlobsDetected; j++) {
				blobResult.GetNthBlob(CBlobGetArea(), j, blobsWithLargestArea[j]); 
				if (blobsWithLargestArea[j].Area() == 0) break; // is empty
				blobsWithLargestArea[j].FillBlob(img6, colors[j % 6]); // only used to display
				
				// blob/vehicle tracking
				if (i > 0) matchObj(blobsWithLargestArea[j], previousImg, img7, seqMatchingWinHeight, seqVelocity, seqCoefficientThr, j); // j is used to color the matching window

				/*if (i > 0) { // dung HSV
					IplImage* im1 = cvCreateImage(cvSize(img7->width, img7->height), IPL_DEPTH_8U, img7->nChannels);
					IplImage* im2 = cvCreateImage(cvSize(img7->width, img7->height), IPL_DEPTH_8U, img7->nChannels);
					cvCvtColor(previousImg, im1, CV_RGB2HSV);
					cvCvtColor(img7, im2, CV_RGB2HSV);
					matchObj(blobsWithLargestArea[j], im1, im2, seqMatchingWinHeight, seqVelocity, seqCoefficientThr, j);
					cvReleaseImage(&im1);
					cvReleaseImage(&im2);
				}*/
			}			
			
		} else //if ( ((CButton*)this->GetDlgItem(IDC_RADIO_PARTICLE_FILTER))->GetCheck() == 1 ) // checked
			particleFilterTrackingMethod(i, previousImg);
				

		//QueryPerformanceCounter((LARGE_INTEGER*)&tStop);	timeDiff = (unsigned long)((tStop - tStart) * 1000 / freq); // tick count STOP


		// ---------- copy fore-rectified-image to previous image, used for tracking ----------
		if (i > 0) cvReleaseImage(&previousImg);
		previousImg = cvCreateImage(cvSize(img7->width, img7->height), IPL_DEPTH_8U, img7->nChannels);
		cvCopy(img7, previousImg);		


		// ---------- display results ----------
		displayResult();
		//cvShowImage("img2", img2);	
		cvShowImage("img3", img3);
		cvShowImage("img4", img4);
		cvShowImage("img5", img5);
		cvShowImage("img6", img6);
		cvShowImage("img7", img7);
		cvShowImage("img8", img8);


		// ---------- release ----------
		//cvWaitKey(100);
		histogramMatching.releaseVerHist();
		histogramMatching.releaseHorHist();

		cvReleaseImage(&currentImg);
		cvReleaseImage(&rectifiedForeImg); 
		cvReleaseImage(&grayOfSubtractedImg);
		
		cvReleaseStructuringElement(&element);
		cvReleaseImage(&tmpImgForClose);

		delete[] blobsWithLargestArea;

		cvReleaseImage(&img2);
		cvReleaseImage(&img3);
		cvReleaseImage(&img4);
		cvReleaseImage(&img5);
		cvReleaseImage(&img6);
		cvReleaseImage(&img7);
		cvReleaseImage(&img8);


		semlock1.Unlock();
		semlock2.Unlock();
	}

	Util::writeVehicleBlobsToFile(blobInfoList); // save the trajectory for each detected vehicle
	cvReleaseImage(&previousImg);
}


void CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonPhmLoadImg() {
	phmFilenames = new CString[2]; // only 2 successive images are supported

	CFileDialog dlg(TRUE, _T("*.*"), _T(""), OFN_FILEMUSTEXIST|OFN_PATHMUSTEXIST|OFN_HIDEREADONLY|OFN_ALLOWMULTISELECT, _T("Image (*.bmp)|*.bmp|All Files (*.*)|*.*||"), NULL);	
	dlg.m_ofn.lpstrInitialDir = L"..\\testdata\\090329";

	if (dlg.DoModal() == IDOK) {
		POSITION pos = dlg.GetStartPosition();
		
		phmNImg = 0; // max(phmNImg) = 2
		if (pos) {
			do {
				phmFilenames[phmNImg++] = dlg.GetNextPathName(pos);
			} while(pos);
		}
	}
}


void CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonPhmCompensateBackground() {
	getInputParam();

	// background compensation for 2 successive images
	char filenameInMultiByte[256];	
	WideCharToMultiByte(CP_ACP, 0, phmFilenames[0], -1, filenameInMultiByte, 256, NULL, NULL);		img1 = cvvLoadImage(filenameInMultiByte);
	WideCharToMultiByte(CP_ACP, 0, phmFilenames[1], -1, filenameInMultiByte, 256, NULL, NULL);		img2 = cvvLoadImage(filenameInMultiByte);

	HistogramMatching histogramMatching(img1, img2);
	histogramMatching.initVerHist(); // do not use thread
	histogramMatching.initHorHist();
	displacement = histogramMatching.matchWithRanges(cvPoint(phmRangeDxFrom, phmRangeDxTo), cvPoint(phmRangeDyFrom, phmRangeDyTo)); 
	histogramMatching.releaseVerHist();
	histogramMatching.releaseHorHist();	
	
	displayResult();


	img3 = cvCreateImage(cvSize(img1->width, img1->height), IPL_DEPTH_8U, img1->nChannels);
	Util::subtractImg(img1, img2, displacement.x, displacement.y, img3);
	phmError = Util::subtractImgWithAvgIntensity(img1, img2, displacement.x, displacement.y); // duoc add them de tinh error
	cvShowImage("img3", img3);


	// normal background subtraction
	img4 = cvCreateImage(cvSize(img1->width, img1->height), IPL_DEPTH_8U, img1->nChannels);
	//cvSub(img1, img2, img4);
	Util::subtractImg(img1, img2, 0, 0, img4);
	cvShowImage("img4", img4);


	// smooth the background compensation result
	img5 = cvCreateImage(cvSize(img1->width, img1->height), IPL_DEPTH_8U, img1->nChannels);
	cvSmooth(img3, img5, CV_GAUSSIAN, phmSmoothWinSize);
	cvShowImage("img5", img5);

	displayResult();
}


void CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonFindVanishingPointBrowseImg() {
	CFileDialog dlg(TRUE, _T("*.avi"), _T(""), OFN_FILEMUSTEXIST|OFN_PATHMUSTEXIST|OFN_HIDEREADONLY, _T("Image (*.bmp)|*.bmp|All Files (*.*)|*.*||"),NULL);	
	dlg.m_ofn.lpstrInitialDir = L"..\\testdata\\090329";
	if (dlg.DoModal() != IDOK) return;
	
	// load image
	char filenameInMultiByte[256];
	WideCharToMultiByte(CP_ACP, 0, dlg.GetPathName(), -1, filenameInMultiByte, 256, NULL, NULL);	

	// img1
	cvReleaseImage(&img1);
	img1 = cvvLoadImage(filenameInMultiByte);
	//cvShowImage("img1", img1);
}


void CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonFindVanishingPoint() {
	getInputParam();

	CvPoint initLeftTop = cvPoint(img1->width / 3, 0); // img duoc chia lam 6 regions, 3 cot, 2 dong => region o giua trong 3 regions cot
	CvPoint initRightBottom = cvPoint(img1->width / 3 * 2, img1->height - 1);
	CvPoint vpPoint = vpDetection.findVanishingPoint(img1, initLeftTop, initRightBottom, seqVanishingPointLevel, seqVanishingPointVoteThr);	

	cvShowImage("img1", img1);

	seqVanishingPointX = vpPoint.x;
	seqVanishingPointY = vpPoint.y;
	displayResult();
}


void CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonQuit() {
	cvDestroyAllWindows();
	
	cvReleaseImage(&img1);
	cvReleaseImage(&img2);
	cvReleaseImage(&img3);
	cvReleaseImage(&img4);
	cvReleaseImage(&img5);
	cvReleaseImage(&img6);
	cvReleaseImage(&img7);

	delete[] colors;
	delete[] filenames;
	delete[] backgrdFilenames;
	delete[] phmFilenames;

	for (int i = 0; i < numBlobs; i++) particleFilters[i].release();
	delete[] particleFilters;

	cvReleaseMemStorage(&blobInfoListStorage);	

	OnOK();
}


void CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonExComparePhmIfdPhm() {
	exAvgDx = exAvgDy = exAvgError = exAvgTimeDiff = 0;
	exMaxDx = exMaxDy = 0;

	int sumDx = 0, sumDy = 0;
	double sumError = .0;
	long sumTimeDiff = 0;

	// open background image from folder
	img1 = cvvLoadImage("..\\testdata\\test_bckgrdCompensate\\MOV03673_10img\\bckgrd_7img.bmp");
	//img1 = cvvLoadImage("..\\testdata\\test_bckgrdCompensate\\MOV03677\\bckgrd_7img.bmp");
	//img1 = cvvLoadImage("..\\testdata\\test_bckgrdCompensate\\MOV03680\\bckgrd_7img.bmp");
	//img1 = cvvLoadImage("..\\testdata\\test_bckgrdCompensate\\MOV03906_16s\\bckgrd_7img.bmp");
	//img1 = cvvLoadImage("..\\testdata\\test_bckgrdCompensate\\MOV03907\\bckgrd_7img.bmp");
	//img1 = cvvLoadImage("..\\testdata\\test_bckgrdCompensate\\MOV03909_12s\\bckgrd_7img.bmp");
	//img1 = cvvLoadImage("..\\testdata\\test_bckgrdCompensate\\MOV03910\\bckgrd_7img.bmp");
	//img1 = cvvLoadImage("..\\testdata\\test_bckgrdCompensate\\MOV03912\\bckgrd_7img.bmp");
	//img1 = cvvLoadImage("..\\testdata\\test_bckgrdCompensate\\MOV03913_11s\\bckgrd_7img.bmp");
	//img1 = cvvLoadImage("..\\testdata\\test_bckgrdCompensate\\MOV03914\\bckgrd_7img.bmp");
	//img1 = cvvLoadImage("..\\testdata\\test_bckgrdCompensate\\MOV03915\\bckgrd_7img.bmp");
	img3 = cvCreateImage(cvSize(img1->width, img1->height), IPL_DEPTH_8U, 1);
	cvCvtColor(img1, img3, CV_BGR2GRAY);	

	// open remaining images
	//for (int i = EX_FROM_IMG; i <= EX_N_TEST_IMG; i++) {
	for (int i = 36; i <= 47; i++) {
		CString fn;
		fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03673_10img\\MOV03673-%02d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03677\\MOV03677-%03d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03680\\MOV03680-%03d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03906_16s\\MOV03906-%03d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03907\\MOV03907-%03d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03909_12s\\MOV03909-%03d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03910\\MOV03910-%03d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03912\\MOV03912-%03d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03913_11s\\MOV03913-%03d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03914\\MOV03914-%03d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03915\\MOV03915-%03d.bmp", i);
		CStringA filenameI (fn);
		img2 = cvvLoadImage(filenameI);		


		// do histgram matching
		HistogramMatching histogramMatching(img1, img2);		// USING THREAD

		QueryPerformanceFrequency((LARGE_INTEGER*)&freq);	QueryPerformanceCounter((LARGE_INTEGER*)&tStart); // tick count START

		CWinThread * pDxThread = AfxBeginThread(matchWithRangeDxThread, &histogramMatching);	pDxThread->ResumeThread();	pDxThread->SetThreadPriority(THREAD_PRIORITY_TIME_CRITICAL);
		CWinThread * pDyThread = AfxBeginThread(matchWithRangeDyThread, &histogramMatching);	pDyThread->ResumeThread();	pDyThread->SetThreadPriority(THREAD_PRIORITY_TIME_CRITICAL);
		
		CSingleLock semlock1(&semaDx);
		CSingleLock semlock2(&semaDy);
		semlock1.Lock();
		semlock2.Lock();

		displacement = cvPoint(_dx, _dy);
		//CvPoint displacement = histogramMatching.matchWithRanges(cvPoint(-EX_DISPLACEMENT_RANGE, EX_DISPLACEMENT_RANGE), cvPoint(-EX_DISPLACEMENT_RANGE, EX_DISPLACEMENT_RANGE)); // THONG SO


		QueryPerformanceCounter((LARGE_INTEGER*)&tStop);	timeDiff = (unsigned long)((tStop - tStart) * 1000 / freq); // tick count STOP
		sumTimeDiff += timeDiff;		


		histogramMatching.releaseVerHist();
		histogramMatching.releaseHorHist();


		// save displacements
		int dx = abs(displacement.x);	sumDx += dx;	if (exMaxDx < dx) exMaxDx = dx;
		int dy = abs(displacement.y);	sumDy += dy;	if (exMaxDy < dy) exMaxDy = dy;


		// calculate the compensation error		
		img4 = cvCreateImage(cvSize(img1->width, img1->height), IPL_DEPTH_8U, 1);
		cvCvtColor(img2, img4, CV_BGR2GRAY);
		sumError += Util::subtractImgWithAvgIntensity(img3, img4, displacement.x, displacement.y);


		// save the result
		img5 = cvCreateImage(cvSize(img1->width, img1->height), IPL_DEPTH_8U, 1);		Util::subtractImg(img3, img4, dx, dy, img5);
		fn.Format(L"..\\testdata\\%02d.bmp", i);	CStringA filenameJ (fn);			cvSaveImage(filenameJ, img5);
		cvReleaseImage(&img5);


		// release
		cvReleaseImage(&img2);
		cvReleaseImage(&img4);


		//semlock1.Unlock();
		//semlock2.Unlock();
	}


	// calculate the average displacement of all images in folder
	int nImg = EX_N_TEST_IMG - (EX_FROM_IMG - 1);
	exAvgDx = sumDx * 1.0 / nImg;
	exAvgDy = sumDy * 1.0 / nImg;

	// calculate the average compensation error
	exAvgError = sumError / nImg;

	// calculate the average time difference
	exAvgTimeDiff = sumTimeDiff * 1.0 / nImg;

	//
	cvReleaseImage(&img1);
	cvReleaseImage(&img3);
	displayResult();
}


void CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonExComparePhmIfdIfd() {
	exAvgDx = exAvgDy = exAvgError = exAvgTimeDiff = 0;
	exMaxDx = exMaxDy = 0;

	int sumDx = 0, sumDy = 0;
	double sumError = .0;
	long sumTimeDiff = 0;

	// open background image from folder
	img1 = cvvLoadImage("..\\testdata\\test_bckgrdCompensate\\MOV03673_10img\\bckgrd_7img.bmp");
	//img1 = cvvLoadImage("..\\testdata\\test_bckgrdCompensate\\MOV03677\\bckgrd_7img.bmp");
	//img1 = cvvLoadImage("..\\testdata\\test_bckgrdCompensate\\MOV03680\\bckgrd_7img.bmp");
	//img1 = cvvLoadImage("..\\testdata\\test_bckgrdCompensate\\MOV03906_16s\\bckgrd_7img.bmp");
	//img1 = cvvLoadImage("..\\testdata\\test_bckgrdCompensate\\MOV03907\\bckgrd_7img.bmp");
	//img1 = cvvLoadImage("..\\testdata\\test_bckgrdCompensate\\MOV03909_12s\\bckgrd_7img.bmp");
	//img1 = cvvLoadImage("..\\testdata\\test_bckgrdCompensate\\MOV03910\\bckgrd_7img.bmp");
	//img1 = cvvLoadImage("..\\testdata\\test_bckgrdCompensate\\MOV03912\\bckgrd_7img.bmp");
	//img1 = cvvLoadImage("..\\testdata\\test_bckgrdCompensate\\MOV03913_11s\\bckgrd_7img.bmp");
	//img1 = cvvLoadImage("..\\testdata\\test_bckgrdCompensate\\MOV03914\\bckgrd_7img.bmp");
	//img1 = cvvLoadImage("..\\testdata\\test_bckgrdCompensate\\MOV03915\\bckgrd_7img.bmp");
	img3 = cvCreateImage(cvSize(img1->width, img1->height), IPL_DEPTH_8U, 1);
	cvCvtColor(img1, img3, CV_BGR2GRAY);	

	// open remaining images
	for (int i = EX_FROM_IMG; i <= EX_N_TEST_IMG; i++) {
		CString fn;
		fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03673_10img\\MOV03673-%02d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03677\\MOV03677-%03d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03680\\MOV03680-%03d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03906_16s\\MOV03906-%03d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03907\\MOV03907-%03d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03909_12s\\MOV03909-%03d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03910\\MOV03910-%03d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03912\\MOV03912-%03d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03913_11s\\MOV03913-%03d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03914\\MOV03914-%03d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03915\\MOV03915-%03d.bmp", i);
		CStringA filenameI (fn);
		img2 = cvvLoadImage(filenameI);

		img4 = cvCreateImage(cvSize(img1->width, img1->height), IPL_DEPTH_8U, 1);
		cvCvtColor(img2, img4, CV_BGR2GRAY);


		// minimize IFD
		QueryPerformanceFrequency((LARGE_INTEGER*)&freq);	QueryPerformanceCounter((LARGE_INTEGER*)&tStart); // tick count START

		CvPoint3D32f dxDyError = Util::minimizeIFD(img3, img4);

		QueryPerformanceCounter((LARGE_INTEGER*)&tStop);	timeDiff = (unsigned long)((tStop - tStart) * 1000 / freq); // tick count STOP
		sumTimeDiff += timeDiff;


		// save displacements
		int dx = abs((int)dxDyError.x);	sumDx += dx;	if (exMaxDx < dx) exMaxDx = dx;
		int dy = abs((int)dxDyError.y);	sumDy += dy;	if (exMaxDy < dy) exMaxDy = dy;


		// calculate the compensation error
		sumError += dxDyError.z;
		

		// save the result
		/*img5 = cvCreateImage(cvSize(img1->width, img1->height), IPL_DEPTH_8U, 1);		Util::subtractImg(img3, img4, dx, dy, img5);
		fn.Format(L"..\\testdata\\%02d.bmp", i);	CStringA filenameJ (fn);			cvSaveImage(filenameJ, img5);
		cvReleaseImage(&img5);*/


		// release
		cvReleaseImage(&img2);
	}

	// calculate the average displacement of all images in folder
	int nImg = EX_N_TEST_IMG - (EX_FROM_IMG - 1);
	exAvgDx = sumDx * 1.0 / nImg;
	exAvgDy = sumDy * 1.0 / nImg;

	// calculate the average compensation error
	exAvgError = sumError / nImg;

	// calculate the average time difference
	exAvgTimeDiff = sumTimeDiff * 1.0 / nImg;

	displayResult();
}


void CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonExComparePhmIfdNo() {
	exAvgDx = exAvgDy = exAvgError = exAvgTimeDiff = 0;
	exMaxDx = exMaxDy = 0;

	double sumError = .0;
	long sumTimeDiff = 0;

	// open background image from folder
	img1 = cvvLoadImage("..\\testdata\\test_bckgrdCompensate\\MOV03673_10img\\bckgrd_7img.bmp");
	//img1 = cvvLoadImage("..\\testdata\\test_bckgrdCompensate\\MOV03677\\bckgrd_7img.bmp");
	//img1 = cvvLoadImage("..\\testdata\\test_bckgrdCompensate\\MOV03680\\bckgrd_7img.bmp");
	//img1 = cvvLoadImage("..\\testdata\\test_bckgrdCompensate\\MOV03906_16s\\bckgrd_7img.bmp");
	//img1 = cvvLoadImage("..\\testdata\\test_bckgrdCompensate\\MOV03907\\bckgrd_7img.bmp");
	//img1 = cvvLoadImage("..\\testdata\\test_bckgrdCompensate\\MOV03909_12s\\bckgrd_7img.bmp");
	//img1 = cvvLoadImage("..\\testdata\\test_bckgrdCompensate\\MOV03910\\bckgrd_7img.bmp");
	//img1 = cvvLoadImage("..\\testdata\\test_bckgrdCompensate\\MOV03912\\bckgrd_7img.bmp");
	//img1 = cvvLoadImage("..\\testdata\\test_bckgrdCompensate\\MOV03913_11s\\bckgrd_7img.bmp");
	//img1 = cvvLoadImage("..\\testdata\\test_bckgrdCompensate\\MOV03914\\bckgrd_7img.bmp");
	//img1 = cvvLoadImage("..\\testdata\\test_bckgrdCompensate\\MOV03915\\bckgrd_7img.bmp");
	img3 = cvCreateImage(cvSize(img1->width, img1->height), IPL_DEPTH_8U, 1);
	cvCvtColor(img1, img3, CV_BGR2GRAY);	

	// open remaining images
	for (int i = EX_FROM_IMG; i <= EX_N_TEST_IMG; i++) {
		CString fn;
		fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03673_10img\\MOV03673-%02d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03677\\MOV03677-%03d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03680\\MOV03680-%03d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03906_16s\\MOV03906-%03d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03907\\MOV03907-%03d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03909_12s\\MOV03909-%03d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03910\\MOV03910-%03d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03912\\MOV03912-%03d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03913_11s\\MOV03913-%03d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03914\\MOV03914-%03d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03915\\MOV03915-%03d.bmp", i);
		CStringA filenameI (fn);
		img2 = cvvLoadImage(filenameI);		


		// calculate the compensation error	
		img4 = cvCreateImage(cvSize(img1->width, img1->height), IPL_DEPTH_8U, 1);
		cvCvtColor(img2, img4, CV_BGR2GRAY);

		QueryPerformanceFrequency((LARGE_INTEGER*)&freq);	QueryPerformanceCounter((LARGE_INTEGER*)&tStart); // tick count START

		sumError += Util::subtractImgWithAvgIntensity(img3, img4, 0, 0);

		QueryPerformanceCounter((LARGE_INTEGER*)&tStop);	timeDiff = (unsigned long)((tStop - tStart) * 1000 / freq); // tick count STOP
		sumTimeDiff += timeDiff;


		// save the result
		/*img5 = cvCreateImage(cvSize(img1->width, img1->height), IPL_DEPTH_8U, 1);		Util::subtractImg(img3, img4, 0, 0, img5);
		fn.Format(L"..\\testdata\\%02d.bmp", i);	CStringA filenameJ (fn);			cvSaveImage(filenameJ, img5);
		cvReleaseImage(&img5);*/


		// release
		cvReleaseImage(&img2);
		cvReleaseImage(&img4);
	}
	
	int nImg = EX_N_TEST_IMG - (EX_FROM_IMG - 1);
	exAvgError = sumError / nImg; // calculate the average compensation error
	exAvgTimeDiff = sumTimeDiff * 1.0 / nImg; // calculate the average time difference

	//
	cvReleaseImage(&img1);
	cvReleaseImage(&img3);
	displayResult();
}


// vi cac pp khac dung anh gray, nen minh convert RGB sang gray
void CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonExConvertRgbToGray() {

	for (int i = 1; i <= 94; i++) {

		// load color image
		CString fn;
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03673_10img\\gray\\c%04d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03677\\gray\\c%04d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03680\\gray\\c%04d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03906_16s\\gray\\c%04d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03909_12s\\gray\\c%04d.bmp", i);
		fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03913_11s\\gray\\c%04d.bmp", i);

		CStringA filenameI (fn);
		img1 = cvvLoadImage(filenameI);	


		// convert color image to gray image
		img2 = cvCreateImage(cvSize(img1->width, img1->height), IPL_DEPTH_8U, 1);
		cvCvtColor(img1, img2, CV_RGB2GRAY);


		// save gray image to file
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03673_10img\\gray\\%04d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03677\\gray\\%04d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03680\\gray\\%04d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03906_16s\\gray\\%04d.bmp", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03909_12s\\gray\\%04d.bmp", i);
		fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03913_11s\\gray\\%04d.bmp", i);
		CStringA filenameIGray (fn);
		cvSaveImage(filenameIGray, img2);


		// release both images
		cvReleaseImage(&img1);
		cvReleaseImage(&img2);
	}
}


// dung de batch rename nhung file fore-image, tu 0001 => 0001, 0002 => 0003, 0003 => 0005...
// roi sau do copy file background 0000 thanh 0002, 0004, 0006...
// tap hop anh nay duoc dua vao Motion2D de test, va moi dong lenh cua Motion2D se doc tung cap file 0000 (background) va 0001 (fore), 0002 va 0003...
void CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonExBatchRename() {	
	int nEnd = 93;
	CString fn1, fn2;

	int count = 1;
	for (int i = 2; i <= nEnd; i++) { // rename foreground images		
		//fn1.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03673_10img\\png_for_motion2d\\%04d.bmp", i);	fn2.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03673_10img\\png_for_motion2d\\r\\%04d.bmp", i + count);
		//fn1.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03677\\png_for_motion2d\\%04d.bmp", i);			fn2.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03677\\png_for_motion2d\\r\\%04d.bmp", i + count);
		//fn1.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03680\\png_for_motion2d\\%04d.bmp", i);			fn2.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03680\\png_for_motion2d\\r\\%04d.bmp", i + count);
		//fn1.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03906_16s\\png_for_motion2d\\%04d.bmp", i);		fn2.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03906_16s\\png_for_motion2d\\r\\%04d.bmp", i + count);
		//fn1.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03909_12s\\png_for_motion2d\\%04d.bmp", i);		fn2.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03909_12s\\png_for_motion2d\\r\\%04d.bmp", i + count);
		fn1.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03913_11s\\png_for_motion2d\\%04d.bmp", i);		fn2.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03913_11s\\png_for_motion2d\\r\\%04d.bmp", i + count);
		CFile::Rename(fn1, fn2);
		count++;
	}

	count = 1;
	for (int i = 2; i <= nEnd; i++) { // copy background image
		//fn1.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03673_10img\\png_for_motion2d\\%04d.bmp", count*2);		
		//fn1.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03677\\png_for_motion2d\\%04d.bmp", count*2);
		//fn1.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03680\\png_for_motion2d\\%04d.bmp", count*2);
		//fn1.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03906_16s\\png_for_motion2d\\%04d.bmp", count*2);
		//fn1.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03909_12s\\png_for_motion2d\\%04d.bmp", count*2);
		fn1.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03913_11s\\png_for_motion2d\\%04d.bmp", count*2);
		
		//Util::copyFile(L"..\\testdata\\test_bckgrdCompensate\\MOV03673_10img\\png_for_motion2d\\0000.bmp", fn1);
		//Util::copyFile(L"..\\testdata\\test_bckgrdCompensate\\MOV03677\\png_for_motion2d\\0000.bmp", fn1);
		//Util::copyFile(L"..\\testdata\\test_bckgrdCompensate\\MOV03680\\png_for_motion2d\\0000.bmp", fn1);
		//Util::copyFile(L"..\\testdata\\test_bckgrdCompensate\\MOV03906_16s\\png_for_motion2d\\0000.bmp", fn1);
		//Util::copyFile(L"..\\testdata\\test_bckgrdCompensate\\MOV03909_12s\\png_for_motion2d\\0000.bmp", fn1);
		Util::copyFile(L"..\\testdata\\test_bckgrdCompensate\\MOV03913_11s\\png_for_motion2d\\0000.bmp", fn1);

		count++;
	}
}


// vi background comp. ap dung trong truong hop nay la: previousFrame la background image, con current frame la 1 incoming frame, 
// do do ket qua cua Motion2D duoc luu o tung file rieng le, voi noi dung chi co 1 dong ket qua, dai dien cho phep tinh giua 2 frame (background & incoming)
// ko the luu thanh 1 file chua nhieu ket qua duoc ==> luu thanh nhieu phai
// ==> go^.p tat ca file nay lai thanh 1 file, roi sau do su dung code (t.X) tinh compensation error cho 1 loat anh
void CMy090310_vehicle_speed_measureDlg::OnBnClickedButtonExCombineM2dResults() { // cach lam: lay dong cuoi cung o moi file, append vao cuoi file "_total.txt"
	CStdioFile f, fi;
	//CString fn("..\\testdata\\test_bckgrdCompensate\\MOV03673_10img\\png_for_motion2d\\resultAH2N\\_total.txt");
	//CString fn("..\\testdata\\test_bckgrdCompensate\\MOV03677\\png_for_motion2d\\resultAH2N\\_total.txt");
	//CString fn("..\\testdata\\test_bckgrdCompensate\\MOV03680\\png_for_motion2d\\resultAH2N\\_total.txt");
	//CString fn("..\\testdata\\test_bckgrdCompensate\\MOV03906_16s\\png_for_motion2d\\resultAH2N\\_total.txt");
	//CString fn("..\\testdata\\test_bckgrdCompensate\\MOV03909_12s\\png_for_motion2d\\resultAH2N\\_total.txt");
	CString fn("..\\testdata\\test_bckgrdCompensate\\MOV03913_11s\\png_for_motion2d\\resultAH2N\\_total.txt");
	f.Open(fn, CFile::modeCreate | CFile::modeWrite | CFile::modeNoTruncate);

	int nEnd = 184;
	for (int i = 0; i <= nEnd; i += 2) {

		// doc tung file result len		
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03673_10img\\png_for_motion2d\\resultAH2N\\AH2N%1d.txt", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03677\\png_for_motion2d\\resultAH2N\\AH2N%1d.txt", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03680\\png_for_motion2d\\resultAH2N\\AH2N%1d.txt", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03906_16s\\png_for_motion2d\\resultAH2N\\AH2N%1d.txt", i);
		//fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03909_12s\\png_for_motion2d\\resultAH2N\\AH2N%1d.txt", i);
		fn.Format(L"..\\testdata\\test_bckgrdCompensate\\MOV03913_11s\\png_for_motion2d\\resultAH2N\\AH2N%1d.txt", i);
		fi.Open(fn, CFile::modeRead);

		// doc dong cuoi cung
		CString text;
		for (int j = 0; j <= 43; j++) fi.ReadString(text);
		fi.Close();

		// xoa id dau tien
		if (i < 10) text.Delete(0, 1);
		else if (i < 100) text.Delete(0, 2);
		else if (i < 1000) text.Delete(0, 3);

		// add lai id theo thu tu
		fn.Format(L"%1d", i/2);
		text.Insert(0, fn);

		// append tung result vao file "_total.txt"
		f.SeekToEnd();
		f.WriteString(text);
		if (i < nEnd) f.WriteString(L"\n");
	}
	
	f.Close();
}


// ================================================== PRIVATE ==================================================

void CMy090310_vehicle_speed_measureDlg::fromBinarizedBackgroundImgToBlobDetectionResult() {	

	// ----------- binarize -----------	
	IplImage* im6 = cvCreateImage(cvSize(img2->width, img2->height), IPL_DEPTH_8U, 1);
	cvCvtColor(img2, im6, CV_BGR2GRAY);

	img3 = cvCreateImage(cvSize(img2->width, img2->height), IPL_DEPTH_8U, im6->nChannels);
	cvThreshold(im6, img3, binarizeThr, 255, CV_THRESH_BINARY);

	cvReleaseImage(&im6);

	// ----------- close operator -----------
	IplImage* im7 = cvCreateImage(cvSize(img3->width, img3->height), IPL_DEPTH_8U, img3->nChannels);
	img4 = cvCreateImage(cvSize(img3->width, img3->height), IPL_DEPTH_8U, img3->nChannels);

	int centerOfStructure = (morCloseStructureSize - 1) / 2;
	IplConvKernel* element = cvCreateStructuringElementEx(morCloseStructureSize, morCloseStructureSize, centerOfStructure, centerOfStructure, CV_SHAPE_RECT);
	cvDilate(img3, im7, element, morCloseIteration); 
	cvErode(im7, img4, element, morCloseIteration);

	cvReleaseStructuringElement(&element);
	cvReleaseImage(&im7);

	// ----------- blob detection -----------
	img5 = cvCreateImage(cvSize(img4->width, img4->height), IPL_DEPTH_8U, 3);
	cvZero(img5);	

	blobResult = CBlobResult(img4, NULL, 0); // 0: background la black
	blobResult.Filter(blobResult, B_EXCLUDE, CBlobGetArea(), B_LESS, blobAreaThr);
}


void CMy090310_vehicle_speed_measureDlg::matchObj(CBlob objBlob, IplImage* preImg, IplImage* curImg, int matchingWinHeightLocal, int velocityLocal, double coefficientThrLocal, int i) {
	int imgWidth = curImg->width, imgHeight = curImg->height;

	// define the window for matching vehicle, 
	// only used for the case the vehicles move toward the camera (y of vehicle is decreased in the rectified image)
	// (matching su dung dd^`au xe)
	int minXOfMatchingWin = (int)objBlob.MinX();		int minYOfMatchingWin = (int)objBlob.MinY();
	int maxXOfMatchingWin = (int)objBlob.MaxX();		int maxYOfMatchingWin = minYOfMatchingWin + matchingWinHeightLocal;
	//int maxXOfMatchingWin = (int)objBlob.MaxX();		int maxYOfMatchingWin = (int)objBlob.MaxY();
	//int minXOfMatchingWin = (int)objBlob.MinX();		int minYOfMatchingWin = maxYOfMatchingWin - matchingWinHeightLocal;
	int matchingWinWidth = maxXOfMatchingWin - minXOfMatchingWin;

	double* sampleHist = correlation.calculateRGBHistogram(preImg, minXOfMatchingWin, maxXOfMatchingWin, minYOfMatchingWin, maxYOfMatchingWin); // tim tren anh go^'c thi hay hon, hay la tim tren anh foreground?
	//Util::printHistogram(sampleHist);

	// define the window for searching the matching window
	int maxXOfSearchWin = maxXOfMatchingWin + matchingWinWidth / 2;		int maxYOfSearchWin = maxYOfMatchingWin + velocityLocal; // co the sua thanh velocityLocal/2 thoi cho searchWin nho lai
	int minXOfSearchWin = minXOfMatchingWin - matchingWinWidth / 2;		int minYOfSearchWin = minYOfMatchingWin - velocityLocal; 

	double maxCoeff = 0;
	CvPoint leftTopOfMatchedWin = cvPoint(0, 0);
	CvPoint rightBottomOfMatchedWin = cvPoint(0, 0);

	int fromY = min(max(minYOfSearchWin, 0), imgHeight);	int toY = max(min(maxYOfSearchWin, imgHeight), 0);
	int fromX = min(max(minXOfSearchWin, 0), imgWidth);		int toX = max(min(maxXOfSearchWin, imgWidth), 0);
	for (int hi = fromY; hi < toY - matchingWinHeightLocal; hi++) {
		for (int wi = fromX; wi < toX - matchingWinWidth; wi++) {
			double* objHist = correlation.calculateRGBHistogram(curImg, wi, wi + matchingWinWidth, hi, hi + matchingWinHeightLocal); // tim tren anh go^'c thi hay hon, hay la tim tren anh foreground?
			//double coeff = correlation.correlationCoefficient(sampleHist, objHist);
			double coeff = correlation.bhattacharyyaCoefficient(sampleHist, objHist);
			if (coeff > maxCoeff) {
				maxCoeff = coeff;
				leftTopOfMatchedWin = cvPoint(wi, hi);
				rightBottomOfMatchedWin = cvPoint(wi + matchingWinWidth, hi + matchingWinHeightLocal);
			}
			//cvRectangle(curImg, leftTopOfMatchedWin, rightBottomOfMatchedWin, CV_RGB(255, 0, 0));
			
			delete objHist;
		}
	}

	// only accept rectangles that satisfy this condition 
	if (maxCoeff > coefficientThrLocal) { 
		int colorIndex = i % N_COLORS;		

		// draw rectangles and displacements (in pixels) to the fore-obj&rectified&blobDetect image				
		cvRectangle(img8, leftTopOfMatchedWin, rightBottomOfMatchedWin, colors[i % N_COLORS]);		
		int distance = (maxYOfMatchingWin + minYOfMatchingWin)/2 - (leftTopOfMatchedWin.y + rightBottomOfMatchedWin.y)/2; // center truoc - center sau
		char pIToA[5];
		_itoa(distance, pIToA, 10);
		cvPutText(img8, pIToA, cvPoint(rightBottomOfMatchedWin.x + 2, rightBottomOfMatchedWin.y), &font, colors[colorIndex]);		


		// draw markers for the moving object in "real" image
		CvPoint realLeftTop = imgRectification.inverseAffineRectifyForOnePoint(cvPoint(leftTopOfMatchedWin.x + seqCropFromLeft, leftTopOfMatchedWin.y + seqCropFromTop), cvPoint(seqVanishingPointX, seqVanishingPointY), seqShiftY);
		CvPoint realRightBottom = imgRectification.inverseAffineRectifyForOnePoint(cvPoint(rightBottomOfMatchedWin.x + seqCropFromLeft, rightBottomOfMatchedWin.y + seqCropFromTop), cvPoint(seqVanishingPointX, seqVanishingPointY), seqShiftY);
		//cvRectangle(img2, realLeftTop, realRightBottom, colors[colorIndex], 2);		
		cvRectangle(currentImg, cvPoint(realLeftTop.x, realLeftTop.y - 10), cvPoint(realRightBottom.x, realRightBottom.y - 10), colors[colorIndex], 6);		cvShowImage("img2", currentImg);	
		//cvPutText(img2, pIToA, cvPoint(realRightBottom.x, realRightBottom.y - 5), &font, colors[colorIndex]);


		// save the trajectory for each detected vehicle
		//Util::storeVehicleBlobs(blobInfoList, (leftTopOfMatchedWin.x + rightBottomOfMatchedWin.x)/2, distance);		
	} 

	delete sampleHist;
}


void CMy090310_vehicle_speed_measureDlg::particleFilterTrackingMethod(int i, IplImage* previousImg) {
	int hx = seqPFparticleWidth/2;
	int hy = seqPFparticleHeight/2;

	img8 = cvCreateImage(cvSize(img7->width, img7->height), IPL_DEPTH_8U, img7->nChannels);
	cvCopyImage(img7, img8);

	if (i == 0) { // the first frame -> blob detection
		blobResult = CBlobResult(img5, NULL, 0); // 0: background la black
		blobResult.Filter(blobResult, B_EXCLUDE, CBlobGetArea(), B_LESS, seqBlobAreaThr);
		CBlob* blobsWithLargestArea = new CBlob[seqNBlobsDetected];

		particleFilters = new ParticleFilter[seqNBlobsDetected];
		previousPositions = new CvPoint[seqNBlobsDetected];
		numBlobs = 0;

		for (int j = 0; j < seqNBlobsDetected; j++) {			
			blobResult.GetNthBlob(CBlobGetArea(), j, blobsWithLargestArea[j]); 
			if (blobsWithLargestArea[j].Area() == 0) break; // is empty

			CvPoint expectedPositionOfObj = cvPoint( int( (blobsWithLargestArea[j].MinX() + blobsWithLargestArea[j].MaxX()) / 2 ), 
													 int( (blobsWithLargestArea[j].MinY() + blobsWithLargestArea[j].MaxY()) / 2 ) );
			particleFilters[j].setParamsAndInit(seqPFnParticles, hx, hy, expectedPositionOfObj, seqPFpositionSigma, seqPFvelocitySigma, img5);

			previousPositions[j] = cvPoint(expectedPositionOfObj.x, expectedPositionOfObj.y);
			numBlobs++;
		}

		delete[] blobsWithLargestArea;

	} else { // the second frame.. -> particle filter tracking

		for (int j = 0; j < numBlobs; j++) {
			CvPoint estimatedPositionOfObj = particleFilters[j].process(img5);

			int distance = previousPositions[j].y - estimatedPositionOfObj.y;
			previousPositions[j] = cvPoint(estimatedPositionOfObj.x, estimatedPositionOfObj.y);


			// draw markers to the rectifed image
			int colorIndex = j % N_COLORS;	
			cvRectangle(img8, cvPoint(estimatedPositionOfObj.x - hx, estimatedPositionOfObj.y - hy), cvPoint(estimatedPositionOfObj.x + hx, estimatedPositionOfObj.y + hy), colors[colorIndex]);
			char pIToA[5];
			_itoa(distance, pIToA, 10);
			cvPutText(img8, pIToA, cvPoint(estimatedPositionOfObj.x + 3, estimatedPositionOfObj.y), &font, colors[colorIndex]);		


			// draw markers for the moving object in "real" image
			CvPoint realLeftTop = imgRectification.inverseAffineRectifyForOnePoint(cvPoint(estimatedPositionOfObj.x - hx + seqCropFromLeft, estimatedPositionOfObj.y - hy + seqCropFromTop), 
																				   cvPoint(seqVanishingPointX, seqVanishingPointY), seqShiftY);
			CvPoint realRightBottom = imgRectification.inverseAffineRectifyForOnePoint(cvPoint(estimatedPositionOfObj.x + hx + seqCropFromLeft, estimatedPositionOfObj.y + hy + seqCropFromTop), 
																					   cvPoint(seqVanishingPointX, seqVanishingPointY), seqShiftY);
			cvRectangle(img2, realLeftTop, realRightBottom, colors[colorIndex]);
			cvPutText(img2, pIToA, realRightBottom, &font, colors[colorIndex]);


			// save the trajectory for each detected vehicle
			Util::storeVehicleBlobs(blobInfoList, estimatedPositionOfObj.x, distance);		
		}
		cvShowImage("img2", img2); 
		cvShowImage("img5", img5);
		cvShowImage("img7", img7);
		cvShowImage("img8", img8);
	}
}


// -------------------------------------

void CMy090310_vehicle_speed_measureDlg::initVariables() {
	colors = new CvScalar[N_COLORS];
	colors[0] = CV_RGB(255, 0, 0);
	colors[1] = CV_RGB(0, 0, 255);
	colors[2] = CV_RGB(0, 255, 0);
	colors[3] = CV_RGB(255, 255, 0);
	colors[4] = CV_RGB(255, 0, 255);
	colors[5] = CV_RGB(0, 255, 255);

	cvInitFont(&font, 8, 0.4, 0.4);


	// -------- "Using Image" group box --------
	shiftX = SHIFT_X;
	shiftY = SHIFT_Y;
	vanishingPointX = VANISHING_POINT_X;
	vanishingPointY = VANISHING_POINT_Y;

	binarizeThr = BINARIZE_THR;
	morCloseStructureSize = MOR_CLOSE_STRUCTURE_SIZE;
	morCloseIteration = MOR_CLOSE_ITERATION;
	nBlobsDetected = N_BLOBS_DETECTED;
	blobAreaThr = BLOB_AREA_THR;

	matchingWinHeight = MATCHING_WIN_HEIGHT;
	velocity = VELOCITY;
	coefficientThr = COEFFICIENT_THR;


	// -------- "Using Image Sequence" group box --------
	//seqNImgToCreateBckgrd = SEQ_N_IMG_TO_CREATE_BCKGRD; // not used
	seqVanishingPointX = SEQ_VANISHING_POINT_X;
	seqVanishingPointY = SEQ_VANISHING_POINT_Y;
	seqShiftY = SEQ_SHIFT_Y;
	seqVanishingPointVoteThr = SEQ_VANISHING_POINT_VOTE_THR;
	seqVanishingPointLevel = SEQ_VANISHING_POINT_LEVEL;
	seqBinarizeThr = SEQ_BINARIZE_THR;
	seqMorCloseStructureSize = SEQ_MOR_CLOSE_STRUCTURE_SIZE;
	seqMorCloseIteration = SEQ_MOR_CLOSE_ITERATION;
	seqNBlobsDetected = SEQ_N_BLOBS_DETECTED;
	seqBlobAreaThr = SEQ_BLOB_AREA_THR;

	seqCropFromTop = SEQ_CROP_FROM_TOP;
	seqCropFromBottom = SEQ_CROP_FROM_BOTTOM;
	seqCropFromLeft = SEQ_CROP_FROM_LEFT;
	seqCropFromRight = SEQ_CROP_FROM_RIGHT;

	seqPhmRangeDxFrom = SEQ_PHM_RANGE_DX_FROM;
	seqPhmRangeDxTo = SEQ_PHM_RANGE_DX_TO;
	seqPhmRangeDyFrom = SEQ_PHM_RANGE_DY_FROM;
	seqPhmRangeDyTo = SEQ_PHM_RANGE_DY_TO;
	seqPhmSmoothWinSize = SEQ_PHM_SMOOTH_WIN_SIZE;

	seqMatchingWinHeight = SEQ_MATCHING_WIN_HEIGHT;
	seqVelocity = SEQ_VELOCITY;
	seqCoefficientThr = SEQ_COEFFICIENT_THR;

	seqPFnParticles = SEQ_PF_N_PARTICLES;
	seqPFpositionSigma = SEQ_PF_POSITION_SIGMA;
	seqPFvelocitySigma = SEQ_PF_VELOCITY_SIGMA;
	seqPFparticleWidth = SEQ_PF_PARTICLE_WIDTH;
	seqPFparticleHeight = SEQ_PF_PARTICLE_HEIGHT;	


	// -------- "Projection Histogram Matching" group box --------
	phmRangeDxFrom = PHM_RANGE_DX_FROM;
	phmRangeDxTo = PHM_RANGE_DX_TO;
	phmRangeDyFrom = PHM_RANGE_DY_FROM;
	phmRangeDyTo = PHM_RANGE_DY_TO;
	phmSmoothWinSize = PHM_SMOOTH_WIN_SIZE;
}


void CMy090310_vehicle_speed_measureDlg::initGUI() {
	cvNamedWindow("img1");
	cvNamedWindow("img2");
	cvNamedWindow("img3");
	cvNamedWindow("img4");
	cvNamedWindow("img5");
	cvNamedWindow("img6");
	cvNamedWindow("img7");
	cvNamedWindow("img8");


	// -------- "Using Image" group box --------
	CString s;
	s.Format(L"%3d", SHIFT_X);					this->GetDlgItem(IDC_EDIT_ONE_VANISHING_POINT_SHIFTX)->SetWindowTextW((LPCTSTR)s); 
	s.Format(L"%3d", SHIFT_Y);					this->GetDlgItem(IDC_EDIT_ONE_VANISHING_POINT_SHIFTY)->SetWindowTextW((LPCTSTR)s); 
	s.Format(L"%3d", VANISHING_POINT_X);		this->GetDlgItem(IDC_EDIT_ONE_VANISHING_POINT_X)->SetWindowTextW((LPCTSTR)s); 
	s.Format(L"%3d", VANISHING_POINT_Y);		this->GetDlgItem(IDC_EDIT_ONE_VANISHING_POINT_Y)->SetWindowTextW((LPCTSTR)s); 

	s.Format(L"%3d", BINARIZE_THR);				this->GetDlgItem(IDC_EDIT_BINARIZE_THR)->SetWindowTextW((LPCTSTR)s); 
	s.Format(L"%2d", MOR_CLOSE_STRUCTURE_SIZE);	this->GetDlgItem(IDC_EDIT_MOR_CLOSE_STRUCTURE_SIZE)->SetWindowTextW((LPCTSTR)s); 
	s.Format(L"%2d", MOR_CLOSE_ITERATION);		this->GetDlgItem(IDC_EDIT_MOR_CLOSE_ITERATION)->SetWindowTextW((LPCTSTR)s); 
	s.Format(L"%2d", N_BLOBS_DETECTED);			this->GetDlgItem(IDC_EDIT_N_BLOBS_DETECTED)->SetWindowTextW((LPCTSTR)s); 
	s.Format(L"%3d", BLOB_AREA_THR);			this->GetDlgItem(IDC_EDIT_BLOB_AREA_THR)->SetWindowTextW((LPCTSTR)s); 

	s.Format(L"%2d", N_BLOBS_DETECTED);			this->GetDlgItem(IDC_EDIT_N_BLOBS_DETECTED_FOR_TRACKING)->SetWindowTextW((LPCTSTR)s); 
	s.Format(L"%2d", MATCHING_WIN_HEIGHT);		this->GetDlgItem(IDC_EDIT_MATCHING_WIN_HEIGHT)->SetWindowTextW((LPCTSTR)s); 
	s.Format(L"%3d", VELOCITY);					this->GetDlgItem(IDC_EDIT_VELOCITY)->SetWindowTextW((LPCTSTR)s); 
	s.Format(L"%1.2f", COEFFICIENT_THR);		this->GetDlgItem(IDC_EDIT_COEFFICIENT_THR)->SetWindowTextW((LPCTSTR)s); 


	// -------- "Using Image Sequence" group box --------
	//s.Format(L"%2d", SEQ_N_IMG_TO_CREATE_BCKGRD);	this->GetDlgItem(IDC_EDIT_SEQ_N_IMG_TO_CREATE_BCKGRD)->SetWindowTextW((LPCTSTR)s); // not used
	s.Format(L"%3d", SEQ_VANISHING_POINT_X);		this->GetDlgItem(IDC_EDIT_SEQ_VANISHING_POINT_X)->SetWindowTextW((LPCTSTR)s);
	s.Format(L"%3d", SEQ_VANISHING_POINT_Y);		this->GetDlgItem(IDC_EDIT_SEQ_VANISHING_POINT_Y)->SetWindowTextW((LPCTSTR)s);
	s.Format(L"%3d", SEQ_SHIFT_Y);					this->GetDlgItem(IDC_EDIT_SEQ_SHIFT_Y)->SetWindowTextW((LPCTSTR)s);
	s.Format(L"%3d", SEQ_VANISHING_POINT_VOTE_THR);	this->GetDlgItem(IDC_EDIT_SEQ_VANISHING_POINT_VOTE_THR)->SetWindowTextW((LPCTSTR)s);
	s.Format(L"%2d", SEQ_VANISHING_POINT_LEVEL);	this->GetDlgItem(IDC_EDIT_SEQ_VANISHING_POINT_LEVEL)->SetWindowTextW((LPCTSTR)s);

	s.Format(L"%3d", SEQ_BINARIZE_THR);				this->GetDlgItem(IDC_EDIT_SEQ_BINARIZE_THR)->SetWindowTextW((LPCTSTR)s);
	s.Format(L"%2d", SEQ_MOR_CLOSE_STRUCTURE_SIZE);	this->GetDlgItem(IDC_EDIT_SEQ_MOR_CLOSE_STRUCTURE_SIZE)->SetWindowTextW((LPCTSTR)s);
	s.Format(L"%2d", SEQ_MOR_CLOSE_ITERATION);		this->GetDlgItem(IDC_EDIT_SEQ_MOR_CLOSE_ITERATION)->SetWindowTextW((LPCTSTR)s);
	s.Format(L"%2d", SEQ_N_BLOBS_DETECTED);			this->GetDlgItem(IDC_EDIT_SEQ_N_BLOBS_DETECTED)->SetWindowTextW((LPCTSTR)s); 
	s.Format(L"%3d", SEQ_BLOB_AREA_THR);			this->GetDlgItem(IDC_EDIT_SEQ_BLOB_AREA_THR)->SetWindowTextW((LPCTSTR)s); 

	s.Format(L"%3d", SEQ_CROP_FROM_TOP);			this->GetDlgItem(IDC_EDIT_SEQ_CROP_FROM_TOP)->SetWindowTextW((LPCTSTR)s); 
	s.Format(L"%3d", SEQ_CROP_FROM_BOTTOM);			this->GetDlgItem(IDC_EDIT_SEQ_CROP_FROM_BOTTOM)->SetWindowTextW((LPCTSTR)s); 
	s.Format(L"%3d", SEQ_CROP_FROM_LEFT);			this->GetDlgItem(IDC_EDIT_SEQ_CROP_FROM_LEFT)->SetWindowTextW((LPCTSTR)s); 
	s.Format(L"%3d", SEQ_CROP_FROM_RIGHT);			this->GetDlgItem(IDC_EDIT_SEQ_CROP_FROM_RIGHT)->SetWindowTextW((LPCTSTR)s); 

	s.Format(L"%3d", SEQ_PHM_RANGE_DX_FROM);		this->GetDlgItem(IDC_EDIT_SEQ_PHM_RANGE_DX_FROM)->SetWindowTextW((LPCTSTR)s); 
	s.Format(L"%3d", SEQ_PHM_RANGE_DX_TO);			this->GetDlgItem(IDC_EDIT_SEQ_PHM_RANGE_DX_TO)->SetWindowTextW((LPCTSTR)s); 
	s.Format(L"%3d", SEQ_PHM_RANGE_DY_FROM);		this->GetDlgItem(IDC_EDIT_SEQ_PHM_RANGE_DY_FROM)->SetWindowTextW((LPCTSTR)s); 
	s.Format(L"%3d", SEQ_PHM_RANGE_DY_TO);			this->GetDlgItem(IDC_EDIT_SEQ_PHM_RANGE_DY_TO)->SetWindowTextW((LPCTSTR)s); 
	s.Format(L"%2d", SEQ_PHM_SMOOTH_WIN_SIZE);		this->GetDlgItem(IDC_EDIT_SEQ_PHM_SMOOTH_WIN_SIZE)->SetWindowTextW((LPCTSTR)s); 

	s.Format(L"%2d", SEQ_MATCHING_WIN_HEIGHT);		this->GetDlgItem(IDC_EDIT_SEQ_MATCHING_WIN_HEIGHT)->SetWindowTextW((LPCTSTR)s); 
	s.Format(L"%3d", SEQ_VELOCITY);					this->GetDlgItem(IDC_EDIT_SEQ_VELOCITY)->SetWindowTextW((LPCTSTR)s); 
	s.Format(L"%1.2f", SEQ_COEFFICIENT_THR);		this->GetDlgItem(IDC_EDIT_SEQ_COEFFICIENT_THR)->SetWindowTextW((LPCTSTR)s); 

	s.Format(L"%3d", SEQ_PF_N_PARTICLES);			this->GetDlgItem(IDC_EDIT_SEQ_PF_N_PARTICLES)->SetWindowTextW((LPCTSTR)s); 
	s.Format(L"%2.1f", SEQ_PF_POSITION_SIGMA);		this->GetDlgItem(IDC_EDIT_SEQ_PF_POSITION_SIGMA)->SetWindowTextW((LPCTSTR)s); 
	s.Format(L"%2.1f", SEQ_PF_VELOCITY_SIGMA);		this->GetDlgItem(IDC_EDIT_SEQ_PF_VELOCITY_SIGMA)->SetWindowTextW((LPCTSTR)s); 
	s.Format(L"%2d", SEQ_PF_PARTICLE_WIDTH);		this->GetDlgItem(IDC_EDIT_SEQ_PF_PARTICLE_WIDTH)->SetWindowTextW((LPCTSTR)s); 
	s.Format(L"%2d", SEQ_PF_PARTICLE_HEIGHT);		this->GetDlgItem(IDC_EDIT_SEQ_PF_PARTICLE_HEIGHT)->SetWindowTextW((LPCTSTR)s); 


	// -------- "Projection Histogram Matching" group box --------
	s.Format(L"%3d", PHM_RANGE_DX_FROM);		this->GetDlgItem(IDC_EDIT_PHM_RANGE_DX_FROM)->SetWindowTextW((LPCTSTR)s); 
	s.Format(L"%3d", PHM_RANGE_DX_TO);			this->GetDlgItem(IDC_EDIT_PHM_RANGE_DX_TO)->SetWindowTextW((LPCTSTR)s); 
	s.Format(L"%3d", PHM_RANGE_DY_FROM);		this->GetDlgItem(IDC_EDIT_PHM_RANGE_DY_FROM)->SetWindowTextW((LPCTSTR)s); 
	s.Format(L"%3d", PHM_RANGE_DY_TO);			this->GetDlgItem(IDC_EDIT_PHM_RANGE_DY_TO)->SetWindowTextW((LPCTSTR)s); 
	s.Format(L"%2d", PHM_SMOOTH_WIN_SIZE);		this->GetDlgItem(IDC_EDIT_PHM_SMOOTH_WIN_SIZE)->SetWindowTextW((LPCTSTR)s); 


	// -------- tracking methods --------
	((CButton*)this->GetDlgItem(IDC_RADIO_BLOB_DETECTION_TRACKING))->SetCheck(1);
	((CButton*)this->GetDlgItem(IDC_RADIO_PARTICLE_FILTER))->SetCheck(0);
}


void CMy090310_vehicle_speed_measureDlg::getInputParam() {

	// -------- "Using Image" group box --------
	CString shiftXString, shiftYString, vanishingPointXString, vanishingPointYString;
	this->GetDlgItem(IDC_EDIT_ONE_VANISHING_POINT_SHIFTX)->GetWindowTextW(shiftXString);
	this->GetDlgItem(IDC_EDIT_ONE_VANISHING_POINT_SHIFTY)->GetWindowTextW(shiftYString);
	this->GetDlgItem(IDC_EDIT_ONE_VANISHING_POINT_X)->GetWindowTextW(vanishingPointXString);
	this->GetDlgItem(IDC_EDIT_ONE_VANISHING_POINT_Y)->GetWindowTextW(vanishingPointYString);
	if (!shiftXString.IsEmpty())			shiftX = (int)wcstod(shiftXString, NULL);
	if (!shiftYString.IsEmpty())			shiftY = (int)wcstod(shiftYString, NULL);
	if (!vanishingPointXString.IsEmpty())	vanishingPointX = (int)wcstod(vanishingPointXString, NULL);
	if (!vanishingPointYString.IsEmpty())	vanishingPointY = (int)wcstod(vanishingPointYString, NULL);

	CString binarizeThrString, morCloseStructureSizeString, morCloseIterationString, nBlobsDetectedString, blobAreaThrString;
	this->GetDlgItem(IDC_EDIT_BINARIZE_THR)->GetWindowTextW(binarizeThrString);
	this->GetDlgItem(IDC_EDIT_MOR_CLOSE_STRUCTURE_SIZE)->GetWindowTextW(morCloseStructureSizeString);
	this->GetDlgItem(IDC_EDIT_MOR_CLOSE_ITERATION)->GetWindowTextW(morCloseIterationString);
	this->GetDlgItem(IDC_EDIT_N_BLOBS_DETECTED)->GetWindowTextW(nBlobsDetectedString);
	this->GetDlgItem(IDC_EDIT_BLOB_AREA_THR)->GetWindowTextW(blobAreaThrString);
	if (!binarizeThrString.IsEmpty())				binarizeThr = (int)wcstod(binarizeThrString, NULL);
	if (!morCloseStructureSizeString.IsEmpty())		morCloseStructureSize = (int)wcstod(morCloseStructureSizeString, NULL);
	if (!morCloseIterationString.IsEmpty())			morCloseIteration = (int)wcstod(morCloseIterationString, NULL);
	if (!nBlobsDetectedString.IsEmpty())			nBlobsDetected = (int)wcstod(nBlobsDetectedString, NULL);
	if (!blobAreaThrString.IsEmpty())				blobAreaThr = (int)wcstod(blobAreaThrString, NULL);	

	CString matchingWinHeightString, velocityString, coefficientThrString; 
	this->GetDlgItem(IDC_EDIT_N_BLOBS_DETECTED_FOR_TRACKING)->GetWindowTextW(nBlobsDetectedString);
	this->GetDlgItem(IDC_EDIT_MATCHING_WIN_HEIGHT)->GetWindowTextW(matchingWinHeightString);
	this->GetDlgItem(IDC_EDIT_VELOCITY)->GetWindowTextW(velocityString);
	this->GetDlgItem(IDC_EDIT_COEFFICIENT_THR)->GetWindowTextW(coefficientThrString);
	if (!nBlobsDetectedString.IsEmpty())		nBlobsDetected = (int)wcstod(nBlobsDetectedString, NULL);
	if (!matchingWinHeightString.IsEmpty())		matchingWinHeight = (int)wcstod(matchingWinHeightString, NULL);
	if (!velocityString.IsEmpty())				velocity = (int)wcstod(velocityString, NULL);
	if (!coefficientThrString.IsEmpty())		coefficientThr = wcstod(coefficientThrString, NULL);


	// -------- "Using Image Sequence" group box --------

	//CString seqNImgToCreateBckgrdString; // not used
	CString seqVanishingPointXString, seqVanishingPointYString, seqShiftYString, seqVanishingPointVoteThrString, seqVanishingPointLevelString, seqBinarizeThrString, seqMorCloseStructureSizeString;
	CString seqMorCloseIterationString, seqNBlobsDetectedString, seqBlobAreaThrString;	
	//this->GetDlgItem(IDC_EDIT_SEQ_N_IMG_TO_CREATE_BCKGRD)->GetWindowTextW(seqNImgToCreateBckgrdString); // not used
	this->GetDlgItem(IDC_EDIT_SEQ_VANISHING_POINT_X)->GetWindowTextW(seqVanishingPointXString);
	this->GetDlgItem(IDC_EDIT_SEQ_VANISHING_POINT_Y)->GetWindowTextW(seqVanishingPointYString);
	this->GetDlgItem(IDC_EDIT_SEQ_SHIFT_Y)->GetWindowTextW(seqShiftYString);
	this->GetDlgItem(IDC_EDIT_SEQ_VANISHING_POINT_VOTE_THR)->GetWindowTextW(seqVanishingPointVoteThrString);
	this->GetDlgItem(IDC_EDIT_SEQ_VANISHING_POINT_LEVEL)->GetWindowTextW(seqVanishingPointLevelString);
	this->GetDlgItem(IDC_EDIT_SEQ_BINARIZE_THR)->GetWindowTextW(seqBinarizeThrString);
	this->GetDlgItem(IDC_EDIT_SEQ_MOR_CLOSE_STRUCTURE_SIZE)->GetWindowTextW(seqMorCloseStructureSizeString);
	this->GetDlgItem(IDC_EDIT_SEQ_MOR_CLOSE_ITERATION)->GetWindowTextW(seqMorCloseIterationString);
	this->GetDlgItem(IDC_EDIT_SEQ_N_BLOBS_DETECTED)->GetWindowTextW(seqNBlobsDetectedString);
	this->GetDlgItem(IDC_EDIT_SEQ_BLOB_AREA_THR)->GetWindowTextW(seqBlobAreaThrString);
	//if (!seqNImgToCreateBckgrdString.IsEmpty())		seqNImgToCreateBckgrd = (int)wcstod(seqNImgToCreateBckgrdString, NULL); // not used
	if (!seqVanishingPointXString.IsEmpty())		seqVanishingPointX = (int)wcstod(seqVanishingPointXString, NULL);
	if (!seqVanishingPointYString.IsEmpty())		seqVanishingPointY = (int)wcstod(seqVanishingPointYString, NULL);
	if (!seqShiftYString.IsEmpty())					seqShiftY = (int)wcstod(seqShiftYString, NULL);
	if (!seqVanishingPointVoteThrString.IsEmpty())	seqVanishingPointVoteThr = (int)wcstod(seqVanishingPointVoteThrString, NULL);
	if (!seqVanishingPointLevelString.IsEmpty())	seqVanishingPointLevel = (int)wcstod(seqVanishingPointLevelString, NULL);
	if (!seqBinarizeThrString.IsEmpty())			seqBinarizeThr = (int)wcstod(seqBinarizeThrString, NULL);
	if (!seqMorCloseStructureSizeString.IsEmpty())	seqMorCloseStructureSize = (int)wcstod(seqMorCloseStructureSizeString, NULL);
	if (!seqMorCloseIterationString.IsEmpty())		seqMorCloseIteration = (int)wcstod(seqMorCloseIterationString, NULL);
	if (!seqNBlobsDetectedString.IsEmpty())			seqNBlobsDetected = (int)wcstod(seqNBlobsDetectedString, NULL);
	if (!seqBlobAreaThrString.IsEmpty())			seqBlobAreaThr = (int)wcstod(seqBlobAreaThrString, NULL);

	CString seqCropFromTopString, seqCropFromBottomString, seqCropFromLeftString, seqCropFromRightString;	
	this->GetDlgItem(IDC_EDIT_SEQ_CROP_FROM_TOP)->GetWindowTextW(seqCropFromTopString);
	this->GetDlgItem(IDC_EDIT_SEQ_CROP_FROM_BOTTOM)->GetWindowTextW(seqCropFromBottomString);
	this->GetDlgItem(IDC_EDIT_SEQ_CROP_FROM_LEFT)->GetWindowTextW(seqCropFromLeftString);
	this->GetDlgItem(IDC_EDIT_SEQ_CROP_FROM_RIGHT)->GetWindowTextW(seqCropFromRightString);
	if (!seqCropFromTopString.IsEmpty())			seqCropFromTop = (int)wcstod(seqCropFromTopString, NULL);	
	if (!seqCropFromBottomString.IsEmpty())			seqCropFromBottom = (int)wcstod(seqCropFromBottomString, NULL);	
	if (!seqCropFromLeftString.IsEmpty())			seqCropFromLeft = (int)wcstod(seqCropFromLeftString, NULL);	
	if (!seqCropFromRightString.IsEmpty())			seqCropFromRight = (int)wcstod(seqCropFromRightString, NULL);	

	CString seqPhmRangeDxFromString, seqPhmRangeDxToString, seqPhmRangeDyFromString, seqPhmRangeDyToString, seqPhmSmoothWinSizeString;
	this->GetDlgItem(IDC_EDIT_SEQ_PHM_RANGE_DX_FROM)->GetWindowTextW(seqPhmRangeDxFromString);
	this->GetDlgItem(IDC_EDIT_SEQ_PHM_RANGE_DX_TO)->GetWindowTextW(seqPhmRangeDxToString);
	this->GetDlgItem(IDC_EDIT_SEQ_PHM_RANGE_DY_FROM)->GetWindowTextW(seqPhmRangeDyFromString);
	this->GetDlgItem(IDC_EDIT_SEQ_PHM_RANGE_DY_TO)->GetWindowTextW(seqPhmRangeDyToString);
	this->GetDlgItem(IDC_EDIT_SEQ_PHM_SMOOTH_WIN_SIZE)->GetWindowTextW(seqPhmSmoothWinSizeString);
	if (!seqPhmRangeDxFromString.IsEmpty())			seqPhmRangeDxFrom = (int)wcstod(seqPhmRangeDxFromString, NULL);	
	if (!seqPhmRangeDxToString.IsEmpty())			seqPhmRangeDxTo = (int)wcstod(seqPhmRangeDxToString, NULL);	
	if (!seqPhmRangeDyFromString.IsEmpty())			seqPhmRangeDyFrom = (int)wcstod(seqPhmRangeDyFromString, NULL);	
	if (!seqPhmRangeDyToString.IsEmpty())			seqPhmRangeDyTo = (int)wcstod(seqPhmRangeDyToString, NULL);	
	if (!seqPhmSmoothWinSizeString.IsEmpty())		seqPhmSmoothWinSize = (int)wcstod(seqPhmSmoothWinSizeString, NULL);	

	CString seqMatchingWinHeightString, seqVelocityString, seqCoefficientThrString; 
	this->GetDlgItem(IDC_EDIT_SEQ_MATCHING_WIN_HEIGHT)->GetWindowTextW(seqMatchingWinHeightString);
	this->GetDlgItem(IDC_EDIT_SEQ_VELOCITY)->GetWindowTextW(seqVelocityString);
	this->GetDlgItem(IDC_EDIT_SEQ_COEFFICIENT_THR)->GetWindowTextW(seqCoefficientThrString);
	if (!seqMatchingWinHeightString.IsEmpty())		seqMatchingWinHeight = (int)wcstod(seqMatchingWinHeightString, NULL);
	if (!seqVelocityString.IsEmpty())				seqVelocity = (int)wcstod(seqVelocityString, NULL);
	if (!seqCoefficientThrString.IsEmpty())			seqCoefficientThr = wcstod(seqCoefficientThrString, NULL);

	CString seqPFnParticlesString, seqPFpositionSigmaString, seqPFvelocitySigmaString, seqPFparticleWidthString, seqPFparticleHeightString; 
	this->GetDlgItem(IDC_EDIT_SEQ_PF_N_PARTICLES)->GetWindowTextW(seqPFnParticlesString);
	this->GetDlgItem(IDC_EDIT_SEQ_PF_POSITION_SIGMA)->GetWindowTextW(seqPFpositionSigmaString);
	this->GetDlgItem(IDC_EDIT_SEQ_PF_VELOCITY_SIGMA)->GetWindowTextW(seqPFvelocitySigmaString);
	this->GetDlgItem(IDC_EDIT_SEQ_PF_PARTICLE_WIDTH)->GetWindowTextW(seqPFparticleWidthString);
	this->GetDlgItem(IDC_EDIT_SEQ_PF_PARTICLE_HEIGHT)->GetWindowTextW(seqPFparticleHeightString);
	if (!seqPFnParticlesString.IsEmpty())			seqPFnParticles = (int)wcstod(seqPFnParticlesString, NULL);
	if (!seqPFpositionSigmaString.IsEmpty())		seqPFpositionSigma = wcstod(seqPFpositionSigmaString, NULL);
	if (!seqPFvelocitySigmaString.IsEmpty())		seqPFvelocitySigma = wcstod(seqPFvelocitySigmaString, NULL);
	if (!seqPFparticleWidthString.IsEmpty())		seqPFparticleWidth = (int)wcstod(seqPFparticleWidthString, NULL);
	if (!seqPFparticleHeightString.IsEmpty())		seqPFparticleHeight = (int)wcstod(seqPFparticleHeightString, NULL);


	// -------- "Projection Histogram Matching" group box --------

	CString phmRangeDxFromString, phmRangeDxToString, phmRangeDyFromString, phmRangeDyToString, phmSmoothWinSizeString;
	this->GetDlgItem(IDC_EDIT_PHM_RANGE_DX_FROM)->GetWindowTextW(phmRangeDxFromString);
	this->GetDlgItem(IDC_EDIT_PHM_RANGE_DX_TO)->GetWindowTextW(phmRangeDxToString);
	this->GetDlgItem(IDC_EDIT_PHM_RANGE_DY_FROM)->GetWindowTextW(phmRangeDyFromString);
	this->GetDlgItem(IDC_EDIT_PHM_RANGE_DY_TO)->GetWindowTextW(phmRangeDyToString);
	this->GetDlgItem(IDC_EDIT_PHM_SMOOTH_WIN_SIZE)->GetWindowTextW(phmSmoothWinSizeString);
	if (!phmRangeDxFromString.IsEmpty())		phmRangeDxFrom = (int)wcstod(phmRangeDxFromString, NULL);	
	if (!phmRangeDxToString.IsEmpty())			phmRangeDxTo = (int)wcstod(phmRangeDxToString, NULL);	
	if (!phmRangeDyFromString.IsEmpty())		phmRangeDyFrom = (int)wcstod(phmRangeDyFromString, NULL);	
	if (!phmRangeDyToString.IsEmpty())			phmRangeDyTo = (int)wcstod(phmRangeDyToString, NULL);	
	if (!phmSmoothWinSizeString.IsEmpty())		phmSmoothWinSize = (int)wcstod(phmSmoothWinSizeString, NULL);	
}


void CMy090310_vehicle_speed_measureDlg::displayResult() {
	CString outTextString;

	// -------- "Using Image Sequence" group box --------
	outTextString.Format(L"%3d", seqDisplacement.x);	GetDlgItem(IDC_STATIC_SEQ_PHM_DX)->SetWindowTextW((LPCTSTR)outTextString); 
	outTextString.Format(L"%3d", seqDisplacement.y);	GetDlgItem(IDC_STATIC_SEQ_PHM_DY)->SetWindowTextW((LPCTSTR)outTextString); 
	
	outTextString.Format(L"%3d", seqVanishingPointX);	GetDlgItem(IDC_EDIT_SEQ_VANISHING_POINT_X)->SetWindowTextW((LPCTSTR)outTextString); 
	outTextString.Format(L"%3d", seqVanishingPointY);	GetDlgItem(IDC_EDIT_SEQ_VANISHING_POINT_Y)->SetWindowTextW((LPCTSTR)outTextString); 

	outTextString.Format(L"%4d", int(timeDiff));		GetDlgItem(IDC_STATIC_PROCESSING_TIME)->SetWindowTextW((LPCTSTR)outTextString); 


	// -------- "Projection Histogram Matching" group box --------
	outTextString.Format(L"%3d", displacement.x);	GetDlgItem(IDC_STATIC_PHM_DX)->SetWindowTextW((LPCTSTR)outTextString); 
	outTextString.Format(L"%3d", displacement.y);	GetDlgItem(IDC_STATIC_PHM_DY)->SetWindowTextW((LPCTSTR)outTextString); 
	outTextString.Format(L"%2.4f", phmError);		GetDlgItem(IDC_STATIC_PHM_ERROR)->SetWindowTextW((LPCTSTR)outTextString); 


	// -------- "Experiments for the paper" group box --------
	outTextString.Format(L"%1.2f", exAvgDx);		GetDlgItem(IDC_STATIC_EX_PHM_IFD_AVG_DX)->SetWindowTextW((LPCTSTR)outTextString); 
	outTextString.Format(L"%1.2f", exAvgDy);		GetDlgItem(IDC_STATIC_EX_PHM_IFD_AVG_DY)->SetWindowTextW((LPCTSTR)outTextString); 
	outTextString.Format(L"%2d", exMaxDx);			GetDlgItem(IDC_STATIC_EX_PHM_IFD_MAX_DX)->SetWindowTextW((LPCTSTR)outTextString); 
	outTextString.Format(L"%2d", exMaxDy);			GetDlgItem(IDC_STATIC_EX_PHM_IFD_MAX_DY)->SetWindowTextW((LPCTSTR)outTextString); 
	outTextString.Format(L"%1.2f", exAvgError);		GetDlgItem(IDC_STATIC_EX_PHM_IFD_AVG_ERROR)->SetWindowTextW((LPCTSTR)outTextString); 
	outTextString.Format(L"%3.2f", exAvgTimeDiff);	GetDlgItem(IDC_STATIC_EX_PHM_IFD_AVG_TIMEDIFF)->SetWindowTextW((LPCTSTR)outTextString); 
}
