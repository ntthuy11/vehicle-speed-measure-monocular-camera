// 090310_vehicle_speed_measure.h : main header file for the PROJECT_NAME application
//

#pragma once

#ifndef __AFXWIN_H__
	#error "include 'stdafx.h' before including this file for PCH"
#endif

#include "resource.h"		// main symbols


// CMy090310_vehicle_speed_measureApp:
// See 090310_vehicle_speed_measure.cpp for the implementation of this class
//

class CMy090310_vehicle_speed_measureApp : public CWinApp
{
public:
	CMy090310_vehicle_speed_measureApp();

// Overrides
	public:
	virtual BOOL InitInstance();

// Implementation

	DECLARE_MESSAGE_MAP()
};

extern CMy090310_vehicle_speed_measureApp theApp;